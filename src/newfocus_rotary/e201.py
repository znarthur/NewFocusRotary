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

Register access (BiSS-C bidirectional, E201-9B only), as used by RLS's tool:

* ``Ws<value:03d>:<addr:03d>`` writes one byte; reply is a status code (0 = OK)
* ``R<len:02d>:<addr:03d>`` reads ``len`` bytes; reply ``status:details:hexdata``
* the register bank is selected by writing the bank number to address 0x40

AksIM-2 registers (RLS ``registers_preset.py``, ``aksim2_preset``):

* multiturn value: bank 0, 0x14..0x17 (32 bit, big endian).  Captured from RLS
  EncoSight 5.0.0.17 talking to a real AksIM-2: ``Ws000:064``, ``Ws000:020`` ..
  ``Ws000:023``, ``Ws205:072``, ``Ws109:073``.  (RLS's open-source preset says
  bank 3; that write is rejected by this encoder, so bank 3 is only a fallback.)
* unlock key: write 0xCD to 0x48 before a command
* command register 0x49: 0x6D (``m``) = apply multiturn value,
  0x62 (``b``) = clear the multiturn error (RLS application note MBD02)
* detailed status: bank 0, 0x4A..0x4B (16 bit); bit 15 = multiturn counter
  mismatch.  That error also clears on an encoder power cycle.
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


E201_REGISTER_STATUS = {
    "0": "OK",
    "1": "End of bank reached",
    "2": "CRC error or incorrect data length",
    "3": "Address > 127 or number of bytes > 64 or zero",
    "4": "Timeout",
}

BANK_SELECT_ADDR = 0x40
AKSIM2_KEY_ADDR = 0x48
AKSIM2_KEY = 0xCD
AKSIM2_COMMAND_ADDR = 0x49
AKSIM2_CMD_SET_MULTITURN = 0x6D
AKSIM2_CMD_CLEAR_MULTITURN_ERROR = 0x62  # 'b'
AKSIM2_CMD_SAVE = 0x63  # 'c': store configuration to non-volatile memory
# Bank 0, 0x00..0x03: position offset (single-turn counts); output = (raw - offset) mod 2^ST.
# EncoSight's zero preset rewrites 0x00..0x13 and 0x18 (other values unchanged) before saving.
AKSIM2_OFFSET_ADDR = 0x00
AKSIM2_CONFIG_BLOCK_LEN = 0x14
AKSIM2_CONFIG_EXTRA_ADDR = 0x18
AKSIM2_STATUS_MULTITURN_MISMATCH = 1 << 15

# Detailed-status bits whose meaning is confirmed (MBD02).  Others: see the application note.
AKSIM2_DETAILED_STATUS_BITS = {
    15: "multiturn counter mismatch (clear: write multiturn, command 'b', or power cycle)",
}


def describe_detailed_status(value: int) -> list[str]:
    out = []
    for bit in range(15, -1, -1):
        if value >> bit & 1:
            out.append(f"bit {bit}: " + AKSIM2_DETAILED_STATUS_BITS.get(bit, "see AksIM-2 BiSS app note MBD02"))
    return out
AKSIM2_MULTITURN_BANK = 0  # as EncoSight does it
AKSIM2_MULTITURN_BANK_FALLBACK = 3  # RLS open-source preset; rejected by the tested unit
AKSIM2_MULTITURN_ADDR = 0x14
AKSIM2_DETAILED_STATUS_BANK = 0
AKSIM2_DETAILED_STATUS_ADDR = 0x4A
AKSIM2_DETAILED_STATUS_LEN = 2


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

    def raw_query(self, cmd: str) -> str:
        """Send any command and return the reply unchecked (diagnostics)."""
        with self.lock:
            self._t.write(cmd.encode("ascii"))
            return self._t.readline().decode("ascii", errors="replace").strip()

    def query(self, cmd: str) -> str:
        reply = self.raw_query(cmd)
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

    # -- BiSS register access ------------------------------------------
    @staticmethod
    def _check_status(reply: str, what: str) -> str:
        code = reply.strip().split(":")[0].strip()
        if code != "0":
            msg = E201_REGISTER_STATUS.get(code, f"unknown reply {reply!r}")
            raise EncoderError(f"BiSS register {what} failed: {msg}")
        return reply

    def write_register(self, address: int, value: int) -> None:
        if not 0 <= value <= 255 or not 0 <= address <= 127:
            raise ValueError("register value must be 0-255 and address 0-127")
        self._check_status(self.query(f"Ws{value:03d}:{address:03d}"), f"write 0x{address:02X}")

    def select_bank(self, bank: int) -> None:
        self.write_register(BANK_SELECT_ADDR, bank)

    def read_registers(self, bank: Optional[int], address: int, length: int) -> bytes:
        if not 1 <= length <= 64 or not 0 <= address <= 127:
            raise ValueError("length must be 1-64 and address 0-127")
        with self.lock:
            if bank is not None:
                self.select_bank(bank)
            reply = self._check_status(self.query(f"R{length:02d}:{address:03d}"), f"read 0x{address:02X}")
        parts = reply.strip().split(":")
        if len(parts) < 3:
            raise EncoderError(f"Malformed register reply {reply!r}")
        return bytes.fromhex(parts[2])

    def write_registers(self, bank: Optional[int], address: int, data: bytes) -> None:
        with self.lock:
            if bank is not None:
                self.select_bank(bank)
            for i, b in enumerate(data):
                self.write_register(address + i, b)

    # -- AksIM-2 helpers --------------------------------------------------
    def read_detailed_status(self) -> int:
        """AksIM-2 detailed status register (see the AksIM-2 data sheet for the bit meanings)."""
        raw = self.read_registers(AKSIM2_DETAILED_STATUS_BANK, AKSIM2_DETAILED_STATUS_ADDR, AKSIM2_DETAILED_STATUS_LEN)
        return int.from_bytes(raw, "big")

    def set_multiturn(self, value: int = 0) -> str:
        """Write a new multiturn value (EncoSight's sequence).  Returns what was accepted.

        Bank 0, value to 0x14..0x17, key 0xCD to 0x48, command 0x6D to 0x49.
        Falls back to bank 3 if bank 0 is rejected.
        """
        data = int(value).to_bytes(4, "big", signed=value < 0)
        errors = []
        with self.lock:
            for bank in (AKSIM2_MULTITURN_BANK, AKSIM2_MULTITURN_BANK_FALLBACK):
                try:
                    self.write_registers(bank, AKSIM2_MULTITURN_ADDR, data)
                    break
                except EncoderError as exc:
                    errors.append(f"bank {bank}: {exc}")
            else:
                raise EncoderError("multiturn value write rejected: " + "; ".join(errors))
            self.write_register(AKSIM2_KEY_ADDR, AKSIM2_KEY)
            self.write_register(AKSIM2_COMMAND_ADDR, AKSIM2_CMD_SET_MULTITURN)
            time.sleep(0.1)
        return f"multiturn set to {value} (bank {bank})"

    def clear_multiturn_error_command(self) -> None:
        """Command 'b' to the command register (clears the multiturn mismatch error)."""
        with self.lock:
            self.write_register(AKSIM2_KEY_ADDR, AKSIM2_KEY)
            self.write_register(AKSIM2_COMMAND_ADDR, AKSIM2_CMD_CLEAR_MULTITURN_ERROR)
            time.sleep(0.1)

    def power_cycle(self, off_s: float = 0.3) -> None:
        with self.lock:
            self.query("F")
            time.sleep(off_s)
            reply = self.query("N")
            if "ON" not in reply.upper():
                raise EncoderError(f"Encoder power-on failed: {reply!r}")
            time.sleep(0.3)

    def read_position_offset(self) -> int:
        return int.from_bytes(self.read_registers(0, AKSIM2_OFFSET_ADDR, 4), "big")

    def _stable_singleturn(self, n: int = 5, tolerance: int = 16) -> int:
        """Median of ``n`` valid readings; refuses if the encoder is moving or in error."""
        cpr = self.fmt.counts_per_rev
        vals = []
        for _ in range(n):
            r = self.read()
            if not r.crc_ok or r.error:
                raise EncoderError("zero preset needs a valid encoder reading (CRC ok, no nE)")
            vals.append(r.singleturn)
        ref = vals[0]
        deltas = sorted((v - ref + cpr // 2) % cpr - cpr // 2 for v in vals)
        if deltas[-1] - deltas[0] > tolerance:
            raise EncoderError("encoder position not stable; stop the motor before a zero preset")
        return (ref + deltas[len(deltas) // 2]) % cpr

    def zero_preset(self, off_s: float = 1.8) -> str:
        """Make the current position read 0 (single-turn 0, multiturn 0), stored in the encoder.

        Replays RLS EncoSight 5.0.0.17's "Zero preset" exactly, as captured on a real
        AksIM-2 / E201-9B: read bank 0, multiturn <- 0, write the new position offset
        (with the rest of 0x00..0x13 and 0x18 unchanged), key + 'c' (save to flash),
        power-cycle the encoder, multiturn <- 0 again.  Writes the encoder's
        non-volatile memory, so use it for commissioning, not routinely.
        """
        cpr = self.fmt.counts_per_rev
        with self.lock:
            self.select_bank(0)
            block = bytearray(self.read_registers(None, 0x00, 64))
            old_offset = int.from_bytes(block[0:4], "big")
            st = self._stable_singleturn()
            new_offset = (st + old_offset) % cpr  # raw position of "here"
            self.set_multiturn(0)
            block[0:4] = new_offset.to_bytes(4, "big")
            self.write_registers(0, 0x00, bytes(block[:AKSIM2_CONFIG_BLOCK_LEN]))
            self.write_register(AKSIM2_CONFIG_EXTRA_ADDR, block[AKSIM2_CONFIG_EXTRA_ADDR])
            self.write_register(AKSIM2_KEY_ADDR, AKSIM2_KEY)
            self.write_register(AKSIM2_COMMAND_ADDR, AKSIM2_CMD_SAVE)
            time.sleep(0.2)
            # The encoder reports nE until restarted with the new offset.
            self.power_cycle(off_s=off_s)
            self.set_multiturn(0)
            time.sleep(0.1)
            r = self.read()
        after = (r.singleturn + cpr // 2) % cpr - cpr // 2
        if r.error or not r.crc_ok or abs(after) > 64:
            raise EncoderError(
                f"zero preset written (offset {old_offset} -> {new_offset}) but the encoder reads "
                f"st={r.singleturn} mt={r.multiturn} err={int(r.error)}; check with encoder-status"
            )
        return f"zero preset: offset {old_offset} -> {new_offset}, now st={r.singleturn} mt={r.multiturn}"

    def reset_multiturn(self, value: int = 0) -> str:
        """Clear a multiturn error, trying in order:

        1. write the multiturn value (sets the turn count and clears the error),
        2. command 'b' (clears the error, turn count unchanged),
        3. power-cycle the encoder through the E201.

        Returns a description of what worked; raises EncoderError if the error
        remains after all three.
        """
        attempts = []
        steps = [
            ("set value", lambda: self.set_multiturn(value)),
            ("command 'b'", lambda: (self.clear_multiturn_error_command(), "cleared with command 'b'")[1]),
            ("power cycle", lambda: (self.power_cycle(), "cleared by encoder power cycle")[1]),
        ]
        with self.lock:
            for name, fn in steps:
                try:
                    result = fn()
                except EncoderError as exc:
                    attempts.append(f"{name}: {exc}")
                    continue
                time.sleep(0.1)
                if not self.read().error:
                    return result
                attempts.append(f"{name}: accepted, but the error bit is still set")
        raise EncoderError("multiturn error not cleared; " + " | ".join(attempts))

    def power_off(self) -> None:
        self.query("F")

    def close(self) -> None:
        try:
            self._t.close()
        except TransportError:
            pass
