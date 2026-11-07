"""Byte transports for the instruments.

The Newport/New Focus 8742 does *not* enumerate as a serial port on USB: it is a
vendor-specific USB device (VID 0x104D, PID 0x4000) with one bulk OUT and one
bulk IN endpoint carrying the same ASCII protocol used over Ethernet/telnet.
We talk to it with pyusb.  The 8742 can also be reached over Ethernet (TCP port
23), which is supported here as an alternative.

The RLS/Renishaw E201 interface *does* enumerate as a USB CDC virtual COM port
(STMicro VID 0x0483), so it uses pyserial.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Optional, Protocol


class TransportError(IOError):
    """Communication failure (timeout, disconnect, ...)."""


class LineTransport(Protocol):
    def write(self, data: bytes) -> None: ...

    def readline(self, timeout: Optional[float] = None) -> bytes: ...

    def flush_input(self) -> None: ...

    def close(self) -> None: ...


NEWPORT_VID = 0x104D
PICOMOTOR_8742_PID = 0x4000


class UsbBulkTransport:
    """Line-oriented transport over a pair of USB bulk endpoints (pyusb)."""

    def __init__(
        self,
        vid: int = NEWPORT_VID,
        pid: int = PICOMOTOR_8742_PID,
        serial_number: Optional[str] = None,
        index: int = 0,
        timeout: float = 2.0,
        terminator: bytes = b"\n",
    ):
        try:
            import usb.core
            import usb.util
        except ImportError as exc:  # pragma: no cover - depends on host
            raise TransportError("pyusb is required for USB access to the 8742") from exc

        self._usb_core = usb.core
        self._usb_util = usb.util
        devices = list(usb.core.find(find_all=True, idVendor=vid, idProduct=pid) or [])
        if serial_number is not None:
            devices = [d for d in devices if _usb_serial(d) == serial_number]
        if len(devices) <= index:
            raise TransportError(
                f"No USB device {vid:04x}:{pid:04x}"
                + (f" with serial {serial_number!r}" if serial_number else "")
                + f" at index {index} (found {len(devices)}). Check cabling and udev permissions."
            )
        dev = devices[index]
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            dev.set_configuration()
        except usb.core.USBError:
            # Already configured (e.g. by a previous process) is fine.
            pass
        cfg = dev.get_active_configuration()
        intf = cfg[(0, 0)]
        usb.util.claim_interface(dev, intf.bInterfaceNumber)

        def _match(direction):
            def match(ep):
                return (
                    usb.util.endpoint_direction(ep.bEndpointAddress) == direction
                    and usb.util.endpoint_type(ep.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK
                )

            return match

        self._ep_out = usb.util.find_descriptor(intf, custom_match=_match(usb.util.ENDPOINT_OUT))
        self._ep_in = usb.util.find_descriptor(intf, custom_match=_match(usb.util.ENDPOINT_IN))
        if self._ep_out is None or self._ep_in is None:
            raise TransportError("Could not find bulk endpoints on the 8742 USB interface")
        self._dev = dev
        self._intf = intf
        self._timeout = timeout
        self._term = terminator
        self._buf = bytearray()
        self.description = f"usb:{vid:04x}:{pid:04x} serial={_usb_serial(dev)}"

    def write(self, data: bytes) -> None:
        try:
            self._ep_out.write(data, int(self._timeout * 1000))
        except self._usb_core.USBError as exc:
            raise TransportError(f"USB write failed: {exc}") from exc

    def readline(self, timeout: Optional[float] = None) -> bytes:
        timeout = self._timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while self._term not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TransportError(f"USB read timeout (partial: {bytes(self._buf)!r})")
            try:
                chunk = self._ep_in.read(
                    self._ep_in.wMaxPacketSize or 64, max(1, int(remaining * 1000))
                )
            except self._usb_core.USBTimeoutError:
                continue
            except self._usb_core.USBError as exc:
                raise TransportError(f"USB read failed: {exc}") from exc
            self._buf.extend(bytes(chunk))
        idx = self._buf.index(self._term) + len(self._term)
        line = bytes(self._buf[:idx])
        del self._buf[:idx]
        return line

    def flush_input(self) -> None:
        self._buf.clear()
        while True:
            try:
                self._ep_in.read(self._ep_in.wMaxPacketSize or 64, 20)
            except Exception:
                break

    def close(self) -> None:
        try:
            self._usb_util.release_interface(self._dev, self._intf.bInterfaceNumber)
            self._usb_util.dispose_resources(self._dev)
        except Exception:
            pass


def _usb_serial(dev) -> Optional[str]:
    try:
        return dev.serial_number
    except Exception:
        return None


class TcpTransport:
    """Line-oriented transport over TCP (8742 Ethernet port, telnet on 23)."""

    _IAC = 0xFF

    def __init__(self, host: str, port: int = 23, timeout: float = 2.0, terminator: bytes = b"\n"):
        try:
            self._sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as exc:
            raise TransportError(f"Cannot connect to {host}:{port}: {exc}") from exc
        self._timeout = timeout
        self._term = terminator
        self._buf = bytearray()
        self.description = f"tcp:{host}:{port}"

    def write(self, data: bytes) -> None:
        try:
            self._sock.sendall(data)
        except OSError as exc:
            raise TransportError(f"TCP write failed: {exc}") from exc

    def _strip_telnet(self, data: bytes) -> bytes:
        out = bytearray()
        i = 0
        while i < len(data):
            b = data[i]
            if b == self._IAC and i + 1 < len(data):
                cmd = data[i + 1]
                i += 3 if cmd in (251, 252, 253, 254) else 2
                continue
            out.append(b)
            i += 1
        return bytes(out)

    def readline(self, timeout: Optional[float] = None) -> bytes:
        timeout = self._timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while self._term not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TransportError(f"TCP read timeout (partial: {bytes(self._buf)!r})")
            self._sock.settimeout(remaining)
            try:
                chunk = self._sock.recv(256)
            except socket.timeout:
                continue
            except OSError as exc:
                raise TransportError(f"TCP read failed: {exc}") from exc
            if not chunk:
                raise TransportError("TCP connection closed by instrument")
            self._buf.extend(self._strip_telnet(chunk))
        idx = self._buf.index(self._term) + len(self._term)
        line = bytes(self._buf[:idx])
        del self._buf[:idx]
        return line

    def flush_input(self) -> None:
        self._buf.clear()
        self._sock.settimeout(0.05)
        try:
            while self._sock.recv(256):
                pass
        except OSError:
            pass
        self._sock.settimeout(self._timeout)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class SerialTransport:
    """Line-oriented transport over a (virtual) serial port via pyserial."""

    def __init__(
        self,
        port: str,
        baudrate: int = 250_000,
        timeout: float = 0.5,
        terminator: bytes = b"\r",
    ):
        try:
            import serial
        except ImportError as exc:  # pragma: no cover
            raise TransportError("pyserial is required for the E201 interface") from exc
        try:
            self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        except serial.SerialException as exc:
            raise TransportError(f"Cannot open {port}: {exc}") from exc
        self._serial = serial
        self._timeout = timeout
        self._term = terminator
        self._lock = threading.Lock()
        self.description = f"serial:{port}"

    def write(self, data: bytes) -> None:
        try:
            self._ser.write(data)
        except self._serial.SerialException as exc:
            raise TransportError(f"Serial write failed: {exc}") from exc

    def readline(self, timeout: Optional[float] = None) -> bytes:
        if timeout is not None:
            self._ser.timeout = timeout
        try:
            line = self._ser.read_until(self._term)
        except self._serial.SerialException as exc:
            raise TransportError(f"Serial read failed: {exc}") from exc
        finally:
            if timeout is not None:
                self._ser.timeout = self._timeout
        if not line.endswith(self._term):
            raise TransportError(f"Serial read timeout (partial: {line!r})")
        return line

    def flush_input(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass


def find_serial_port_by_vid(vid: int, serial_number: Optional[str] = None) -> Optional[str]:
    """Return the device path of the first serial port with the given USB VID."""
    import serial.tools.list_ports

    for p in serial.tools.list_ports.comports():
        if p.vid == vid and (serial_number is None or p.serial_number == serial_number):
            return p.device
    return None
