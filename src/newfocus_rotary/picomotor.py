"""Driver for the Newport / New Focus 8742 Picomotor controller.

Protocol summary (8742 user manual, section 6; same over USB and Ethernet):

* ASCII commands, terminated by ``\\r``; replies terminated by ``\\r\\n``.
* Commands are prefixed by the axis number (1-4), e.g. ``1PR+100``.
* Only queries (ending in ``?``) produce a reply.
* For daisy-chained controllers (RS-485) the command is prefixed by
  ``<addr>>`` and the reply is prefixed the same way.

The picomotor is an open-loop stick-slip piezo actuator: ``TP?`` is just a count
of commanded steps, and the actual distance per step varies with direction,
load and speed.  Closed-loop control is therefore done in software from the
external encoder (see :mod:`newfocus_rotary.axis`).
"""

from __future__ import annotations

import threading
from typing import Optional

from .transports import LineTransport, TransportError

MAX_VELOCITY = 2000  # steps/s
MAX_ACCELERATION = 200_000  # steps/s^2
MAX_STEPS = 2_147_483_647

MOTOR_TYPES = {0: "none", 1: "unknown", 2: "tiny", 3: "standard"}


class PicomotorError(RuntimeError):
    pass


class Picomotor8742:
    def __init__(self, transport: LineTransport, address: Optional[int] = None):
        self._t = transport
        self._address = address
        self.lock = threading.RLock()
        with self.lock:
            self._t.flush_input()
        self.identity = self.query("*IDN?")
        if "8742" not in self.identity:
            raise PicomotorError(f"Unexpected *IDN? reply {self.identity!r}; is this an 8742?")

    # -- low level -------------------------------------------------------
    def _prefix(self) -> str:
        return f"{self._address}>" if self._address is not None else ""

    def command(self, cmd: str) -> None:
        with self.lock:
            self._t.write((self._prefix() + cmd + "\r").encode("ascii"))

    def query(self, cmd: str, timeout: Optional[float] = None) -> str:
        with self.lock:
            self._t.write((self._prefix() + cmd + "\r").encode("ascii"))
            reply = self._t.readline(timeout).decode("ascii", errors="replace").strip()
        prefix = self._prefix()
        if prefix:
            if not reply.startswith(prefix):
                raise PicomotorError(f"Reply {reply!r} lacks address prefix {prefix!r}")
            reply = reply[len(prefix):]
        return reply

    def _query_int(self, cmd: str) -> int:
        reply = self.query(cmd)
        try:
            return int(reply)
        except ValueError:
            raise PicomotorError(f"Non-integer reply {reply!r} to {cmd!r}") from None

    # -- errors ----------------------------------------------------------
    def error(self) -> tuple[int, str]:
        """Read and clear the oldest error (``TB?`` -> ``"code, MESSAGE"``)."""
        reply = self.query("TB?")
        code_s, _, msg = reply.partition(",")
        try:
            code = int(code_s)
        except ValueError:
            return -1, reply
        return code, msg.strip()

    def check_error(self) -> None:
        code, msg = self.error()
        if code != 0:
            raise PicomotorError(f"8742 error {code}: {msg}")

    # -- motion ----------------------------------------------------------
    def move_relative(self, axis: int, steps: int) -> None:
        steps = max(-MAX_STEPS, min(MAX_STEPS, int(steps)))
        self.command(f"{axis}PR{steps:+d}")

    def move_absolute(self, axis: int, steps: int) -> None:
        self.command(f"{axis}PA{int(steps):+d}")

    def move_indefinite(self, axis: int, direction: int) -> None:
        self.command(f"{axis}MV{'+' if direction >= 0 else '-'}")

    def stop(self, axis: int) -> None:
        """Stop with deceleration."""
        self.command(f"{axis}ST")

    def abort(self) -> None:
        """Abort all motion immediately (no deceleration)."""
        self.command("AB")

    def motion_done(self, axis: int) -> bool:
        return self._query_int(f"{axis}MD?") == 1

    def position(self, axis: int) -> int:
        """Commanded step counter (open loop; not a measured position)."""
        return self._query_int(f"{axis}TP?")

    def define_home(self, axis: int, steps: int = 0) -> None:
        self.command(f"{axis}DH{int(steps)}")

    # -- parameters ------------------------------------------------------
    def set_velocity(self, axis: int, steps_per_s: int) -> None:
        v = max(1, min(MAX_VELOCITY, int(round(steps_per_s))))
        self.command(f"{axis}VA{v}")

    def velocity(self, axis: int) -> int:
        return self._query_int(f"{axis}VA?")

    def set_acceleration(self, axis: int, steps_per_s2: int) -> None:
        a = max(1, min(MAX_ACCELERATION, int(round(steps_per_s2))))
        self.command(f"{axis}AC{a}")

    def acceleration(self, axis: int) -> int:
        return self._query_int(f"{axis}AC?")

    def motor_type(self, axis: int) -> str:
        return MOTOR_TYPES.get(self._query_int(f"{axis}QM?"), "?")

    def motor_check(self) -> None:
        """Ask the controller to detect connected motor types (``MC``)."""
        self.command("MC")

    def close(self) -> None:
        try:
            self._t.close()
        except TransportError:
            pass
