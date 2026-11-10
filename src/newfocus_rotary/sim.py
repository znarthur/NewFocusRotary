"""Simulated 8742 + rotary picomotor stage + E201/AksIM-2, for testing without hardware.

The simulated stage deliberately behaves like a real picomotor: the angle per
step differs between directions, varies with angle (load) and has random
jitter, so open-loop step counting is wrong by several percent and the
closed-loop logic actually has something to correct.
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass
from typing import Optional

from .e201 import BissFrameFormat, EncoderReading, decode_frame, encode_frame
from .picomotor import MAX_ACCELERATION, MAX_VELOCITY


@dataclass
class SimStageParams:
    deg_per_step_fwd: float = 0.0009
    deg_per_step_rev: float = 0.0011
    jitter: float = 0.05  # relative per-step 1-sigma
    load_ripple: float = 0.15  # relative step-size variation with angle
    encoder_offset_deg: float = 123.456  # encoder zero vs. stage zero
    encoder_sign: int = 1  # +1: encoder counts up for positive picomotor steps
    encoder_noise_counts: float = 0.5
    initial_angle_deg: float = 10.0
    speedup: float = 1.0  # simulated time runs this much faster than wall time
    seed: Optional[int] = 1234


class SimRotaryStage:
    def __init__(self, params: SimStageParams = SimStageParams()):
        self.p = params
        self.rng = random.Random(params.seed)
        self.lock = threading.RLock()
        self.angle = params.initial_angle_deg  # true mechanical angle, degrees
        self.steps = 0  # 8742 TP? counter
        self.velocity = 1750
        self.accel = 100000
        self._target_steps: Optional[int] = None  # remaining signed steps, None = idle
        self._indefinite = 0
        self._frac = 0.0
        self._t_last = time.monotonic()
        self.fail_encoder = False
        self.stuck = False  # simulate a stalled/slipping actuator

    def _step_size(self, direction: int) -> float:
        base = self.p.deg_per_step_fwd if direction > 0 else self.p.deg_per_step_rev
        return base * (1.0 + self.p.load_ripple * math.sin(math.radians(self.angle) * 3))

    def _apply_steps(self, n: int, direction: int) -> None:
        if n <= 0:
            return
        size = self._step_size(direction)
        noise = self.rng.gauss(0.0, self.p.jitter / math.sqrt(n)) if self.p.jitter else 0.0
        if not self.stuck:
            self.angle += direction * n * size * (1.0 + noise)
        self.steps += direction * n

    def advance(self) -> None:
        with self.lock:
            now = time.monotonic()
            dt = (now - self._t_last) * self.p.speedup
            self._t_last = now
            if self._target_steps is None and not self._indefinite:
                self._frac = 0.0
                return
            self._frac += dt * self.velocity
            n = int(self._frac)
            self._frac -= n
            if self._indefinite:
                self._apply_steps(n, self._indefinite)
                return
            rem = self._target_steps
            take = min(n, abs(rem))
            direction = 1 if rem > 0 else -1
            self._apply_steps(take, direction)
            rem -= direction * take
            self._target_steps = rem if rem != 0 else None

    def move_relative(self, steps: int) -> None:
        with self.lock:
            self.advance()
            self._indefinite = 0
            self._frac = 0.0
            self._target_steps = int(steps) if steps else None

    def move_indefinite(self, direction: int) -> None:
        with self.lock:
            self.advance()
            self._target_steps = None
            self._indefinite = 1 if direction >= 0 else -1

    def stop(self) -> None:
        with self.lock:
            self.advance()
            self._target_steps = None
            self._indefinite = 0

    def moving(self) -> bool:
        with self.lock:
            self.advance()
            return self._target_steps is not None or self._indefinite != 0

    def encoder_angle(self) -> float:
        with self.lock:
            self.advance()
            return self.p.encoder_sign * self.angle + self.p.encoder_offset_deg


class SimPicomotor8742:
    """Drop-in replacement for :class:`Picomotor8742` driving a :class:`SimRotaryStage`."""

    identity = "New_Focus 8742 v2.2 (simulated)"

    def __init__(self, stage: SimRotaryStage, axis: int = 1):
        self.stage = stage
        self.sim_axis = axis
        self.lock = threading.RLock()
        self._other_steps = {a: 0 for a in range(1, 5)}

    def _check(self, axis: int) -> bool:
        return axis == self.sim_axis

    def command(self, cmd: str) -> None:  # pragma: no cover - debugging aid
        pass

    def query(self, cmd: str, timeout=None) -> str:
        if cmd == "*IDN?":
            return self.identity
        if cmd == "TB?":
            return "0, NO ERROR DETECTED"
        raise NotImplementedError(cmd)

    def error(self):
        return 0, "NO ERROR DETECTED"

    def check_error(self) -> None:
        pass

    def move_relative(self, axis: int, steps: int) -> None:
        if self._check(axis):
            self.stage.move_relative(steps)

    def move_absolute(self, axis: int, steps: int) -> None:
        if self._check(axis):
            self.stage.move_relative(int(steps) - self.position(axis))

    def move_indefinite(self, axis: int, direction: int) -> None:
        if self._check(axis):
            self.stage.move_indefinite(direction)

    def stop(self, axis: int) -> None:
        if self._check(axis):
            self.stage.stop()

    def abort(self) -> None:
        self.stage.stop()

    def motion_done(self, axis: int) -> bool:
        return not self.stage.moving() if self._check(axis) else True

    def position(self, axis: int) -> int:
        if self._check(axis):
            with self.stage.lock:
                self.stage.advance()
                return self.stage.steps
        return self._other_steps[axis]

    def define_home(self, axis: int, steps: int = 0) -> None:
        if self._check(axis):
            with self.stage.lock:
                self.stage.advance()
                self.stage.steps = int(steps)

    def set_velocity(self, axis: int, steps_per_s: int) -> None:
        if self._check(axis):
            self.stage.velocity = max(1, min(MAX_VELOCITY, int(round(steps_per_s))))

    def velocity(self, axis: int) -> int:
        return self.stage.velocity

    def set_acceleration(self, axis: int, steps_per_s2: int) -> None:
        if self._check(axis):
            self.stage.accel = max(1, min(MAX_ACCELERATION, int(round(steps_per_s2))))

    def acceleration(self, axis: int) -> int:
        return self.stage.accel

    def motor_type(self, axis: int) -> str:
        return "standard" if self._check(axis) else "none"

    def close(self) -> None:
        pass


class SimE201:
    """Drop-in replacement for :class:`E201_9B` reading a :class:`SimRotaryStage`."""

    version = "E201-9B (simulated)"

    def __init__(self, stage: SimRotaryStage, fmt: BissFrameFormat = BissFrameFormat()):
        self.stage = stage
        self.fmt = fmt
        self.lock = threading.RLock()
        self._rng = random.Random(4321)

    def read_raw(self) -> bytes:
        angle = self.stage.encoder_angle()
        cpr = self.fmt.counts_per_rev
        counts = angle / 360.0 * cpr + self._rng.gauss(0.0, self.stage.p.encoder_noise_counts)
        counts = int(math.floor(counts))
        st = counts % cpr
        mt = counts // cpr
        return encode_frame(st, self.fmt, multiturn=mt, error=self.stage.fail_encoder)

    def read(self) -> EncoderReading:
        return decode_frame(self.read_raw(), self.fmt)

    def read_detailed_status(self) -> int:
        return 0x0001 if self.stage.fail_encoder else 0x0000

    def reset_multiturn(self, value: int = 0) -> None:
        """Like the AksIM-2: clears a multiturn error and sets the turn count."""
        with self.stage.lock:
            turns = math.floor(self.stage.encoder_angle() / 360.0)
            self.stage.p.encoder_offset_deg += (value - turns) * 360.0
            self.stage.fail_encoder = False
        return f"multiturn set to {value} (simulated)"

    def close(self) -> None:
        pass
