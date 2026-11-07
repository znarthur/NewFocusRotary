"""Driver for the RLS/Renishaw E201-9B USB BiSS-C interface with an AksIM-2 encoder.

Protocol (E201 data sheet "Command set", cross-checked against RLS's own open
source test platform, github.com/nejcropic/e201_test_platform):

* USB CDC virtual COM port (STMicro VID 0x0483); baud rate is irrelevant.
* Single-character ASCII commands, replies terminated by ``\\r``.
* ``v``  -> version string, contains ``E201``
* ``N``  -> power the encoder on (reply contains ``ON``); ``F`` -> off
* ``4``  -> one BiSS-C frame, 16 hex characters (8 bytes, big endian)
* ``Mnn`` sets the BiSS clock (index into a table), ``m`` reads it back

The 64-bit frame is MSB aligned::

    [multiturn][singleturn][nE][nW][CRC6 (inverted)][zero padding]

AksIM-2 status bits are active low (1 = OK).  The CRC is BiSS CRC-6,
polynomial x^6 + x + 1 (0x43), initial value 0, transmitted inverted, computed
over the position and status bits.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

from .transports import LineTransport, TransportError

E201_VID = 0x0483

# Frequency (kHz) -> index for the "Mnn" command on the E201-9B.
E201_9B_CLOCK_KHZ = {
    10000: 0, 5000: 1, 3333: 2, 2500: 3, 2000: 4, 1667: 5, 1429: 6, 1250: 7,
    1111: 8, 1000: 9, 909: 10, 833: 11, 769: 12, 714: 13, 667: 14, 625: 15,
    500: 17, 333: 18, 250: 19, 200: 20, 167: 21, 143: 22, 125: 23, 111: 24,
    100: 25, 91: 26, 83: 27, 77: 28, 71: 29, 67: 30, 63: 31,
}


class EncoderError(RuntimeError):
    pass


def biss_crc(value: int, nbits: int, poly: int = 0x43, width: int = 6) -> int:
    """Plain (non-inverted) BiSS CRC of the ``nbits`` LSBs of ``value``, MSB first."""
    mask = (1 << width) - 1
    crc = 0
    for i in range(nbits - 1, -1, -1):
        bit = (value >> i) & 1
        feedback = ((crc >> (width - 1)) & 1) ^ bit
        crc = (crc << 1) & mask
        if feedback:
            crc ^= poly & mask
    return crc


@dataclass(frozen=True)
class BissFrameFormat:
    singleturn_bits: int = 20
    multiturn_bits: int = 0
    status_bits: int = 2
    crc_bits: int = 6
    frame_bytes: int = 8
    crc_poly: int = 0x43
    status_active_low: bool = True

    @property
    def data_bits(self) -> int:
        return self.multiturn_bits + self.singleturn_bits + self.status_bits

    @property
    def counts_per_rev(self) -> int:
        return 1 << self.singleturn_bits

    def validate(self) -> None:
        if self.data_bits + self.crc_bits > self.frame_bytes * 8:
            raise ValueError("BiSS frame format does not fit in the E201 reply")
        if not 8 <= self.singleturn_bits <= 32:
            raise ValueError("singleturn_bits out of range")


@dataclass(frozen=True)
class EncoderReading:
    singleturn: int
    multiturn: int
    error: bool  # encoder reports position invalid
    warning: bool  # encoder reports degraded signal (position still valid)
    crc_ok: bool
    raw: bytes
    timestamp: float


def decode_frame(raw: bytes, fmt: BissFrameFormat) -> EncoderReading:
    nbits = fmt.frame_bytes * 8
    if len(raw) != fmt.frame_bytes:
        raise EncoderError(f"Expected {fmt.frame_bytes} bytes, got {len(raw)}")
    frame = int.from_bytes(raw, "big")
    shift_data = nbits - fmt.data_bits
    data = (frame >> shift_data) & ((1 << fmt.data_bits) - 1)
    rx_crc = (frame >> (shift_data - fmt.crc_bits)) & ((1 << fmt.crc_bits) - 1)
    crc_mask = (1 << fmt.crc_bits) - 1
    crc_ok = (rx_crc ^ crc_mask) == biss_crc(data, fmt.data_bits, fmt.crc_poly, fmt.crc_bits)

    status = data & ((1 << fmt.status_bits) - 1)
    position = data >> fmt.status_bits
    singleturn = position & ((1 << fmt.singleturn_bits) - 1)
    multiturn = position >> fmt.singleturn_bits
    if fmt.status_bits >= 2:
        err_bit = (status >> (fmt.status_bits - 1)) & 1
        warn_bit = (status >> (fmt.status_bits - 2)) & 1
    elif fmt.status_bits == 1:
        err_bit, warn_bit = status & 1, int(fmt.status_active_low)
    else:
        err_bit = warn_bit = int(fmt.status_active_low)
    if fmt.status_active_low:
        error, warning = err_bit == 0, warn_bit == 0
    else:
        error, warning = err_bit == 1, warn_bit == 1
    return EncoderReading(
        singleturn=singleturn,
        multiturn=multiturn,
        error=error,
        warning=warning,
        crc_ok=crc_ok,
        raw=bytes(raw),
        timestamp=time.monotonic(),
    )


def encode_frame(
    singleturn: int,
    fmt: BissFrameFormat,
    multiturn: int = 0,
    error: bool = False,
    warning: bool = False,
) -> bytes:
    """Build a frame exactly as the E201 would return it (used by the simulator/tests)."""
    position = ((multiturn & ((1 << fmt.multiturn_bits) - 1)) << fmt.singleturn_bits) | (
        singleturn & ((1 << fmt.singleturn_bits) - 1)
    )
    if fmt.status_active_low:
        e_bit, w_bit = int(not error), int(not warning)
    else:
        e_bit, w_bit = int(error), int(warning)
    status = 0
    if fmt.status_bits >= 2:
        status = (e_bit << (fmt.status_bits - 1)) | (w_bit << (fmt.status_bits - 2))
    elif fmt.status_bits == 1:
        status = e_bit
    data = (position << fmt.status_bits) | status
    crc = biss_crc(data, fmt.data_bits, fmt.crc_poly, fmt.crc_bits) ^ ((1 << fmt.crc_bits) - 1)
    word = (data << fmt.crc_bits) | crc
    nbits = fmt.frame_bytes * 8
    word <<= nbits - fmt.data_bits - fmt.crc_bits
    return word.to_bytes(fmt.frame_bytes, "big")


def guess_data_bits(frames: list[bytes], crc_bits: int = 6, poly: int = 0x43) -> list[int]:
    """Return the data lengths (position + status bits) for which every frame passes CRC.

    Useful for commissioning when the exact AksIM-2 variant is unknown: for an
    AksIM-2 with 2 status bits, ``singleturn_bits + multiturn_bits = n - 2``.
    """
    candidates = []
    for n in range(4, 64 - crc_bits + 1):
        ok = True
        for raw in frames:
            frame = int.from_bytes(raw, "big")
            nbits = len(raw) * 8
            data = (frame >> (nbits - n)) & ((1 << n) - 1)
            rx = (frame >> (nbits - n - crc_bits)) & ((1 << crc_bits) - 1)
            if (rx ^ ((1 << crc_bits) - 1)) != biss_crc(data, n, poly, crc_bits):
                ok = False
                break
            # Padding after the CRC must be zero.
            if frame & ((1 << (nbits - n - crc_bits)) - 1):
                ok = False
                break
        if ok:
            candidates.append(n)
    return candidates


class E201_9B:
    """E201-9B BiSS-C interface reading an AksIM-2 absolute encoder."""

    def __init__(
        self,
        transport: LineTransport,
        fmt: BissFrameFormat = BissFrameFormat(),
        power_on: bool = True,
        clock_khz: Optional[int] = None,
        crc_retries: int = 3,
    ):
        fmt.validate()
        self._t = transport
        self.fmt = fmt
        self.crc_retries = crc_retries
        self.lock = threading.RLock()
        with self.lock:
            self._t.flush_input()
        self.version = self.query("v")
        if "E201" not in self.version:
            raise EncoderError(f"Unexpected version reply {self.version!r}; is this an E201?")
        if power_on:
            reply = self.query("N")
            if "ON" not in reply.upper():
                raise EncoderError(f"Encoder power-on failed: {reply!r}")
            time.sleep(0.3)
        if clock_khz is not None:
            if clock_khz not in E201_9B_CLOCK_KHZ:
                raise ValueError(f"Unsupported BiSS clock {clock_khz} kHz; choose from {sorted(E201_9B_CLOCK_KHZ)}")
            self.query(f"M{E201_9B_CLOCK_KHZ[clock_khz]:02d}")

    def query(self, cmd: str) -> str:
        with self.lock:
            self._t.write(cmd.encode("ascii"))
            reply = self._t.readline().decode("ascii", errors="replace").strip()
        if "error" in reply.lower():
            raise EncoderError(f"E201 command {cmd!r} failed: {reply!r}")
        return reply

    def read_raw(self) -> bytes:
        reply = self.query("4")
        if reply.startswith("T") and "=" in reply:  # trigger-mode formatted reply
            reply = reply.split("=", 1)[1]
        try:
            raw = bytes.fromhex(reply)
        except ValueError:
            raise EncoderError(f"Malformed position reply {reply!r}") from None
        if len(raw) != self.fmt.frame_bytes:
            raise EncoderError(f"Position reply {reply!r} has {len(raw)} bytes, expected {self.fmt.frame_bytes}")
        return raw

    def read(self) -> EncoderReading:
        last: Optional[EncoderReading] = None
        for _ in range(self.crc_retries + 1):
            last = decode_frame(self.read_raw(), self.fmt)
            if last.crc_ok:
                return last
        assert last is not None
        return last

    def power_off(self) -> None:
        self.query("F")

    def close(self) -> None:
        try:
            self._t.close()
        except TransportError:
            pass
