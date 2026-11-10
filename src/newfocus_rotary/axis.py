"""Closed-loop rotary axis: a picomotor driven by the 8742, servoed on the AksIM-2 encoder.

All positions here are *dial* degrees (motor-record terminology): dial increases
for positive picomotor steps.  User coordinates (OFF, DIR) are handled by the
IOC layer, exactly as the EPICS motor record does.

The control strategy suits stick-slip piezo actuators, whose step size is not
repeatable:

1. Estimate the step count from the error and a learned steps/degree
   figure (kept separately for each direction).
2. Move in chunks no longer than ``chunk_time_s`` at the requested speed, watching
   the encoder the whole time, and stop early once inside the fine zone.
3. Inside the fine zone, move a fraction (``fine_gain``) of the estimated steps
   so the target is approached from one side.
4. Repeat until ``|error| <= RDBD`` or ``RTRY`` corrective moves have been
   used (the motor record sets MISS in that case).

After every chunk the observed steps/degree updates the estimate (EWMA), so
the controller adapts to load and direction asymmetry.

Open-loop mode (``closed_loop=False``, motor-record UEIP=No) drives the
picomotor by step count only: the dial position is integrated from the 8742
step counter with the per-direction steps/degree, and encoder faults are
reported but never block motion.  Use it when the encoder is unavailable or
faulty; the encoder (if present) is still read for diagnostics.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import math
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .e201 import BissFrameFormat, EncoderError, EncoderReading
from .picomotor import MAX_VELOCITY, PicomotorError
from .transports import TransportError

log = logging.getLogger(__name__)

HW_ERRORS = (TransportError, PicomotorError, EncoderError, OSError)


class AxisError(RuntimeError):
    pass


@dataclass
class AxisConfig:
    """Static tuning, normally from the YAML config file."""

    picomotor_axis: int = 1
    encoder_sign: int = 1  # +1 if encoder counts increase for positive picomotor steps
    steps_per_degree: float = 1000.0  # initial estimate, positive direction
    steps_per_degree_reverse: Optional[float] = None  # initial estimate, negative (default = forward)
    adapt: bool = True
    adapt_gain: float = 0.3
    adapt_ratio_limits: tuple[float, float] = (0.25, 4.0)  # allowed range vs. initial estimate
    min_learn_steps: int = 20
    fine_zone_deg: float = 0.05
    fine_gain: float = 0.8
    chunk_time_s: float = 0.5
    settle_time_s: float = 0.05
    poll_period_s: float = 0.03
    idle_poll_period_s: float = 0.1
    stall_chunks: int = 5
    limit_margin_deg: float = 0.5  # stop a move that runs this far past a soft limit
    state_file: Optional[str] = None  # persists zero offset, turn count and learned steps/degree
    # Use the encoder's own multiturn count.  False: ignore it and count turns in
    # software from the single-turn angle (use when the multiturn counter is unreliable).
    use_encoder_multiturn: bool = True
    # Raw encoder turn count treated as turn 0 (as printed by "newfocus-rotary-probe encoder",
    # e.g. 65377; negative values are taken modulo 2^multiturn_bits).  Turns are counted
    # relative to it, wrapping correctly at the counter boundary.
    multiturn_reference: int = 0
    detailed_status_period_s: float = 2.0  # poll the AksIM-2 detailed status register when idle (0 = off)


@dataclass
class AxisParams:
    """Runtime parameters, mapped to motor-record fields by the IOC."""

    deadband_deg: float = 0.002  # RDBD
    max_retries: int = 10  # RTRY
    velocity_deg_s: float = 1.0  # VELO
    accel_time_s: float = 0.2  # ACCL
    backlash_deg: float = 0.0  # BDST
    jog_velocity_deg_s: float = 0.5  # JVEL
    home_position_deg: float = 0.0  # dial position used by HOMF/HOMR
    closed_loop: bool = True  # UEIP
    enabled: bool = True  # CNEN
    low_limit_deg: Optional[float] = None  # DLLM (None = no limit)
    high_limit_deg: Optional[float] = None  # DHLM


@dataclass
class AxisStatus:
    dial: float = math.nan
    encoder_counts: int = 0  # unwrapped raw encoder counts
    encoder_singleturn: int = 0
    encoder_multiturn: int = 0
    steps: int = 0  # 8742 step counter (TP?)
    busy: bool = False
    moving: bool = False
    direction: int = 1
    target: Optional[float] = None
    retries: int = 0
    missed: bool = False
    stalled: bool = False
    problem: bool = False
    comm_error: bool = False
    encoder_error: bool = False
    encoder_warning: bool = False
    encoder_detailed_status: Optional[int] = None  # AksIM-2 detailed status register, if readable
    crc_errors: int = 0
    homed: bool = False
    at_high_limit: bool = False
    at_low_limit: bool = False
    steps_per_deg_fwd: float = 0.0
    steps_per_deg_rev: float = 0.0
    counts_per_rev: int = 1
    message: str = "starting"
    valid: bool = False


@dataclass
class _Cmd:
    kind: str
    seq: int
    args: dict = field(default_factory=dict)


class _Abort(Exception):
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class ClosedLoopRotaryAxis:
    def __init__(self, picomotor, encoder, config: AxisConfig = AxisConfig(), params: Optional[AxisParams] = None):
        self.pm = picomotor
        self.enc = encoder
        self.cfg = config
        self.params = params or AxisParams()
        self._lock = threading.RLock()
        self._queue: "queue.Queue[_Cmd]" = queue.Queue()
        self._seq = 0
        self._stop_seq = 0
        self._pending = 0
        self._active: Optional[_Cmd] = None
        self._retargeted = False
        self._shutdown = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._listeners: list[Callable[[AxisStatus], None]] = []

        fmt = self.enc.fmt if self.enc is not None else BissFrameFormat()
        self.cpr = fmt.counts_per_rev
        self.eres_deg = 360.0 / self.cpr
        self._has_multiturn = fmt.multiturn_bits > 0 and config.use_encoder_multiturn
        self._count_source = "encoder-multiturn" if self._has_multiturn else "software-unwrap"
        if self.enc is None:
            self.params.closed_loop = False
        # Open-loop position, integrated from the 8742 step counter.
        self._ol_dial = 0.0
        self._ol_last_steps: Optional[int] = None
        self.zero_offset_deg = 0.0
        self._total_counts: Optional[int] = None
        self._last_st: Optional[int] = None
        self._spd0 = {1: float(config.steps_per_degree), -1: float(config.steps_per_degree_reverse or config.steps_per_degree)}
        self.spd = dict(self._spd0)
        self._last_state_save = 0.0
        self._persisted_counts: Optional[int] = None
        self._last_velocity_steps: Optional[int] = None
        self._last_accel_steps: Optional[int] = None
        self._last_tp_read = 0.0
        self._last_fault: Optional[str] = None
        self._last_detailed_read = 0.0
        self._detailed_status_ok = encoder is not None and hasattr(encoder, "read_detailed_status")

        self.status = AxisStatus(counts_per_rev=self.cpr)
        self._load_state()
        self._publish_spd()

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        try:
            self._read_steps()
            self._poll_position()
            self._set_message("idle" if self.params.closed_loop else "idle (open loop: encoder not used)")
        except HW_ERRORS as exc:
            self._comm_fault(exc)
        self._thread = threading.Thread(target=self._run, name="rotary-axis", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._shutdown.set()
        try:
            self.stop()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._save_state(force=True)

    def add_listener(self, fn: Callable[[AxisStatus], None]) -> None:
        self._listeners.append(fn)

    # ------------------------------------------------------------------ public API (thread safe)
    def get_status(self) -> AxisStatus:
        with self._lock:
            return dataclasses.replace(self.status)

    @property
    def has_encoder(self) -> bool:
        return self.enc is not None

    def set_params(self, **kwargs) -> None:
        with self._lock:
            was_closed = self.params.closed_loop
            for k, v in kwargs.items():
                if not hasattr(self.params, k):
                    raise AttributeError(k)
                setattr(self.params, k, v)
            if self.enc is None:
                self.params.closed_loop = False
            if was_closed and not self.params.closed_loop:
                # Continue from the last known position so the readback does not jump.
                if math.isfinite(self.status.dial):
                    self._ol_dial = self.status.dial
                self.status.dial = self._ol_dial
                self.status.problem = False
                self.status.message = "open loop: moving by step count, encoder ignored"
            elif not was_closed and self.params.closed_loop:
                self.status.valid = False
                if self._total_counts is not None:
                    self.status.dial = self._counts_to_dial(self._total_counts)
                self.status.message = "closed loop: position from encoder"
            self._update_limit_flags()

    def move_to(self, dial_deg: float) -> int:
        """Start a move to a dial position.  Retargets an in-flight move."""
        if not math.isfinite(dial_deg):
            raise AxisError("target is not finite")
        with self._lock:
            self._check_can_move()
            self._seq += 1
            act = self._active
            if (
                act is not None
                and act.kind == "move"
                and self._pending == 1
                and self._stop_seq < act.seq
            ):
                act.seq = self._seq
                act.args["target"] = float(dial_deg)
                self.status.target = float(dial_deg)
                self._retargeted = True
                return self._seq
            return self._enqueue("move", target=float(dial_deg))

    def move_relative(self, delta_deg: float) -> int:
        with self._lock:
            base = self.status.target if (self.status.busy and self.status.target is not None) else self.status.dial
        return self.move_to(base + delta_deg)

    def jog(self, direction: int) -> int:
        with self._lock:
            self._check_can_move()
            self._seq += 1
            return self._enqueue("jog", direction=1 if direction >= 0 else -1)

    def home(self) -> int:
        with self._lock:
            self._check_can_move()
            self._seq += 1
            return self._enqueue("home")

    def calibrate(self, steps: int = 2000) -> int:
        """Measure steps/degree in both directions with a +steps / -steps move pair."""
        with self._lock:
            self._check_can_move()
            if not self.params.closed_loop:
                raise AxisError("calibration needs the encoder (UEIP=Yes)")
            self._seq += 1
            return self._enqueue("calibrate", steps=int(abs(steps)))

    def reset_multiturn(self, value: int = 0) -> int:
        """Reset the AksIM-2 multiturn counter (clears a multiturn error); keeps the dial continuous."""
        with self._lock:
            if self.enc is None or not hasattr(self.enc, "reset_multiturn"):
                raise AxisError("encoder does not support a multiturn reset")
            if self.status.busy:
                raise AxisError("cannot reset the multiturn counter while moving")
            self._seq += 1
            return self._enqueue("reset_multiturn", value=int(value))

    def read_detailed_status(self) -> Optional[int]:
        if self.enc is None or not hasattr(self.enc, "read_detailed_status"):
            return None
        value = self.enc.read_detailed_status()
        with self._lock:
            self.status.encoder_detailed_status = value
        return value

    def stop(self) -> None:
        with self._lock:
            self._seq += 1
            self._stop_seq = self._seq
            while True:
                try:
                    self._queue.get_nowait()
                    self._pending -= 1
                except queue.Empty:
                    break
            self.status.busy = self._pending > 0
        try:
            self.pm.stop(self.cfg.picomotor_axis)
        except HW_ERRORS as exc:
            self._comm_fault(exc)

    def set_dial_position(self, dial_deg: float) -> None:
        """Redefine the current position (motor record SET mode on DVAL)."""
        with self._lock:
            if self.status.busy:
                raise AxisError("cannot redefine position while moving")
            if not self.params.closed_loop:
                self._ol_dial = float(dial_deg)
                self.status.dial = self._ol_dial
                self.status.homed = False
                self._update_limit_flags()
                self._save_state(force=True)
                return
            if self._total_counts is None:
                raise AxisError("no valid encoder reading yet")
            enc_deg = self.cfg.encoder_sign * self._total_counts * self.eres_deg
            self.zero_offset_deg = float(dial_deg) - enc_deg
            self.status.dial = self._counts_to_dial(self._total_counts)
            self.status.homed = False
            self._save_state(force=True)

    def reset_learning(self) -> None:
        with self._lock:
            self.spd = dict(self._spd0)
            self._publish_spd()
            self._save_state(force=True)

    def set_steps_per_degree(self, fwd: float, rev: Optional[float] = None) -> None:
        with self._lock:
            if fwd <= 0 or (rev is not None and rev <= 0):
                raise ValueError("steps/degree must be positive")
            self.spd[1] = float(fwd)
            self.spd[-1] = float(rev if rev is not None else fwd)
            self._spd0 = dict(self.spd)
            self._publish_spd()
            self._save_state(force=True)

    def wait_idle(self, timeout: float = 60.0) -> AxisStatus:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self.get_status()
            if not st.busy:
                return st
            time.sleep(0.01)
        raise TimeoutError("axis still busy")

    # ------------------------------------------------------------------ internals: queue
    def _check_can_move(self) -> None:
        if not self.params.enabled:
            raise AxisError("axis disabled (CNEN=Disable)")
        if self._shutdown.is_set():
            raise AxisError("axis shut down")

    def _enqueue(self, kind: str, **args) -> int:
        cmd = _Cmd(kind, self._seq, args)
        self._pending += 1
        self.status.busy = True
        if kind == "move":
            self.status.target = args["target"]
        self._queue.put(cmd)
        return cmd.seq

    def _run(self) -> None:
        while not self._shutdown.is_set():
            try:
                cmd = self._queue.get(timeout=self.cfg.idle_poll_period_s)
            except queue.Empty:
                self._idle_poll()
                continue
            with self._lock:
                self._active = cmd
                self._retargeted = False
            try:
                if cmd.seq > self._stop_seq:
                    self._execute(cmd)
            except _Abort as ab:
                self._set_message(ab.reason)
            except HW_ERRORS as exc:
                self._comm_fault(exc)
                self._safe_stop()
            except AxisError as exc:
                with self._lock:
                    self.status.problem = True
                self._set_message(str(exc))
                self._safe_stop()
            except Exception as exc:  # pragma: no cover - defensive
                log.exception("unexpected error in axis worker")
                with self._lock:
                    self.status.problem = True
                self._set_message(f"internal error: {exc}")
                self._safe_stop()
            finally:
                with self._lock:
                    self._active = None
                    self._pending -= 1
                    self.status.busy = self._pending > 0
                    self.status.moving = False
                self._save_state()
                self._notify()

    def _execute(self, cmd: _Cmd) -> None:
        with self._lock:
            self.status.missed = False
            self.status.stalled = False
            self.status.problem = False
            self.status.retries = 0
        if cmd.kind == "move":
            self._do_move(cmd)
        elif cmd.kind == "jog":
            self._do_jog(cmd)
        elif cmd.kind == "home":
            cmd.args["target"] = self.params.home_position_deg
            with self._lock:
                self.status.target = cmd.args["target"]
            self._do_move(cmd)
            with self._lock:
                if not (self.status.missed or self.status.problem) and self._stop_seq < cmd.seq:
                    self.status.homed = True
        elif cmd.kind == "calibrate":
            self._do_calibrate(cmd)
        elif cmd.kind == "reset_multiturn":
            self._do_reset_multiturn(cmd)

    def _stop_requested(self, cmd: _Cmd) -> bool:
        return self._stop_seq > cmd.seq or self._shutdown.is_set()

    def _safe_stop(self) -> None:
        try:
            self.pm.stop(self.cfg.picomotor_axis)
        except HW_ERRORS:
            pass

    # ------------------------------------------------------------------ internals: position
    def _counts_to_dial(self, counts: int) -> float:
        return self.cfg.encoder_sign * counts * self.eres_deg + self.zero_offset_deg

    def _read_encoder(self) -> EncoderReading:
        reading = self.enc.read()
        if not reading.crc_ok:
            with self._lock:
                self.status.crc_errors += 1
            raise EncoderError("BiSS CRC error (check frame format / cabling)")
        return reading

    def _poll_position(self) -> float:
        if not self.params.closed_loop:
            return self._poll_open_loop()
        return self._poll_encoder(strict=True)

    def _poll_open_loop(self) -> float:
        """Open loop: position from the step counter; encoder read for diagnostics only."""
        self._read_steps()
        if self.enc is not None:
            try:
                self._poll_encoder(strict=False)
            except HW_ERRORS as exc:
                with self._lock:
                    if isinstance(exc, EncoderError) and "CRC" not in str(exc):
                        self.status.encoder_error = True
                log.debug("encoder unavailable in open loop: %s", exc)
        with self._lock:
            self.status.dial = self._ol_dial
            self.status.valid = True
            self._update_limit_flags()
            dial = self.status.dial
        self._notify()
        return dial

    def _poll_encoder(self, strict: bool) -> float:
        reading = self._read_encoder()
        with self._lock:
            self.status.encoder_warning = reading.warning
            self.status.encoder_error = reading.error
            if reading.error:
                if not strict:
                    return self.status.dial
                self.status.problem = True
                self.status.valid = False
                raise EncoderError(
                    "AksIM-2 reports position error (nE bit); set UEIP=No to move open loop"
                )
            st = reading.singleturn
            if self._has_multiturn:
                mt = self._relative_turns(reading.multiturn)
                total = mt * self.cpr + st
            elif self._total_counts is None:
                total = st
                if self._persisted_counts is not None:
                    k = round((self._persisted_counts - st) / self.cpr)
                    total = st + k * self.cpr
            else:
                half = self.cpr // 2
                delta = (st - self._last_st + half) % self.cpr - half
                total = self._total_counts + delta
            self._last_st = st
            self._total_counts = total
            self.status.encoder_counts = total
            self.status.encoder_singleturn = st
            self.status.encoder_multiturn = reading.multiturn
            if self.status.comm_error:
                self.status.comm_error = False
            if not strict:
                return self.status.dial
            self.status.dial = self._counts_to_dial(total)
            self.status.valid = True
            self._update_limit_flags()
            dial = self.status.dial
        self._notify()
        return dial

    def _relative_turns(self, raw_mt: int) -> int:
        """Turn count relative to ``multiturn_reference``, as a signed value."""
        span = 1 << self.enc.fmt.multiturn_bits
        mt = (raw_mt - self.cfg.multiturn_reference) % span
        if mt >= span // 2:
            mt -= span
        return mt

    def _read_steps(self) -> int:
        steps = self.pm.position(self.cfg.picomotor_axis)
        with self._lock:
            self.status.steps = steps
            if self._ol_last_steps is not None:
                delta = steps - self._ol_last_steps
                if delta:
                    self._ol_dial += delta / self.spd[1 if delta > 0 else -1]
            self._ol_last_steps = steps
        self._last_tp_read = time.monotonic()
        return steps

    def _idle_poll(self) -> None:
        self._poll_detailed_status()  # first: most useful exactly when the position read fails
        try:
            self._poll_position()
            if time.monotonic() - self._last_tp_read > 1.0:
                self._read_steps()
            with self._lock:
                if self._last_fault is not None and not (self.status.comm_error or self.status.encoder_error):
                    log.warning("recovered from: %s", self._last_fault)
                    self._last_fault = None
                    if self.status.message.startswith("comm error") or "nE bit" in self.status.message:
                        self.status.message = "idle (fault cleared)"
        except HW_ERRORS as exc:
            self._comm_fault(exc)

    def _poll_detailed_status(self) -> None:
        period = self.cfg.detailed_status_period_s
        if not self._detailed_status_ok or period <= 0:
            return
        if time.monotonic() - self._last_detailed_read < period:
            return
        self._last_detailed_read = time.monotonic()
        try:
            self.read_detailed_status()
        except HW_ERRORS as exc:
            # Register access is optional; don't let it disturb position polling.
            log.warning("detailed status register not readable, polling disabled: %s", exc)
            self._detailed_status_ok = False

    def _update_limit_flags(self) -> None:
        p = self.params
        d = self.status.dial
        self.status.at_high_limit = p.high_limit_deg is not None and math.isfinite(d) and d >= p.high_limit_deg
        self.status.at_low_limit = p.low_limit_deg is not None and math.isfinite(d) and d <= p.low_limit_deg

    def _comm_fault(self, exc: Exception) -> None:
        text = str(exc)
        if text != self._last_fault:  # log once per distinct fault, not on every poll
            self._last_fault = text
            kind = "encoder fault" if isinstance(exc, EncoderError) and "CRC" not in text else "hardware error"
            log.warning("%s: %s", kind, text)
        with self._lock:
            if not isinstance(exc, EncoderError) or "CRC" in str(exc):
                self.status.comm_error = True
            self.status.problem = True
            self.status.message = f"comm error: {exc}" if self.status.comm_error else str(exc)
        self._notify()

    def _set_message(self, msg: str) -> None:
        with self._lock:
            self.status.message = msg
        self._notify()

    def _notify(self) -> None:
        if not self._listeners:
            return
        st = self.get_status()
        for fn in self._listeners:
            try:
                fn(st)
            except Exception:  # pragma: no cover
                log.exception("status listener failed")

    def _publish_spd(self) -> None:
        self.status.steps_per_deg_fwd = self.spd[1]
        self.status.steps_per_deg_rev = self.spd[-1]

    # ------------------------------------------------------------------ internals: motion
    def _apply_speed(self, deg_per_s: float, accel_time_s: float) -> int:
        spd = 0.5 * (self.spd[1] + self.spd[-1])
        v = int(max(1, min(MAX_VELOCITY, round(abs(deg_per_s) * spd))))
        a = int(max(1, min(200_000, round(v / max(accel_time_s, 1e-3)))))
        ax = self.cfg.picomotor_axis
        if v != self._last_velocity_steps:
            self.pm.set_velocity(ax, v)
            self._last_velocity_steps = v
        if a != self._last_accel_steps:
            self.pm.set_acceleration(ax, a)
            self._last_accel_steps = a
        return v

    def _wait_motion(self, cmd: _Cmd, expected_s: float, early: Optional[Callable[[float], bool]] = None) -> str:
        """Poll until the 8742 reports done; returns 'done', 'early', 'stopped' or 'retarget'."""
        ax = self.cfg.picomotor_axis
        deadline = time.monotonic() + 2.0 * expected_s + 3.0
        outcome = "done"
        while True:
            time.sleep(self.cfg.poll_period_s)
            dial = self._poll_position()
            if self._stop_requested(cmd):
                outcome = "stopped"
            elif self._retargeted:
                outcome = "retarget"
            elif early is not None and early(dial):
                outcome = "early"
            elif self._outside_limits(dial):
                self.pm.stop(ax)
                self._drain_motion(cmd)
                raise AxisError(f"moved past soft limit (dial {dial:.4f})")
            if outcome != "done":
                self.pm.stop(ax)
                self._drain_motion(cmd)
                return outcome
            if self.pm.motion_done(ax):
                return "done"
            if time.monotonic() > deadline:
                self.pm.stop(ax)
                raise AxisError("8742 did not report motion done in time")

    def _drain_motion(self, cmd: _Cmd) -> None:
        ax = self.cfg.picomotor_axis
        deadline = time.monotonic() + 3.0
        while not self.pm.motion_done(ax):
            if time.monotonic() > deadline:
                raise AxisError("8742 did not stop")
            time.sleep(self.cfg.poll_period_s)
            self._poll_position()

    def _outside_limits(self, dial: float) -> bool:
        p, m = self.params, self.cfg.limit_margin_deg
        return (p.high_limit_deg is not None and dial > p.high_limit_deg + m) or (
            p.low_limit_deg is not None and dial < p.low_limit_deg - m
        )

    def _chunk(self, cmd: _Cmd, steps: int, early: Optional[Callable[[float], bool]] = None):
        """Run one relative move; returns (outcome, steps_done, moved_deg)."""
        ax = self.cfg.picomotor_axis
        with self._lock:
            self.status.moving = True
            self.status.direction = 1 if steps > 0 else -1
        dial0 = self._poll_position()
        tp0 = self._read_steps()
        v = self._last_velocity_steps or MAX_VELOCITY
        self.pm.move_relative(ax, steps)
        outcome = self._wait_motion(cmd, abs(steps) / v, early)
        with self._lock:
            self.status.moving = False
        if self.cfg.settle_time_s > 0:
            time.sleep(self.cfg.settle_time_s)
        dial1 = self._poll_position()
        tp1 = self._read_steps()
        return outcome, tp1 - tp0, dial1 - dial0

    def _learn(self, direction: int, steps_done: int, moved: float) -> None:
        if not self.cfg.adapt or not self.params.closed_loop:
            return
        if abs(steps_done) < self.cfg.min_learn_steps or moved * direction <= 0:
            return
        if abs(moved) < 20 * self.eres_deg:
            return
        obs = abs(steps_done) / abs(moved)
        lo, hi = self.cfg.adapt_ratio_limits
        base = self._spd0[direction]
        obs = min(max(obs, base * lo), base * hi)
        g = self.cfg.adapt_gain
        with self._lock:
            self.spd[direction] = (1 - g) * self.spd[direction] + g * obs
            self._publish_spd()

    def _servo(self, cmd: _Cmd, goal: float, deadband: float, final: bool) -> str:
        """Drive to ``goal``; returns 'done', 'stopped', 'retarget' or 'missed'."""
        p = self.params
        stalls = 0
        wrong_way = 0
        attempts = 0  # moves predicted to reach the goal
        while True:
            if self._stop_requested(cmd):
                return "stopped"
            if self._retargeted:
                return "retarget"
            dial = self._poll_position()
            err = goal - dial
            if abs(err) <= deadband:
                return "done"
            if not p.closed_loop and attempts >= 1:
                return "done"
            if final and attempts > p.max_retries:
                with self._lock:
                    self.status.missed = True
                return "missed"
            direction = 1 if err > 0 else -1
            spd = self.spd[direction]
            fine = abs(err) < self.cfg.fine_zone_deg
            want = abs(err) * spd * (self.cfg.fine_gain if (fine and p.closed_loop) else 1.0)
            nsteps = max(1, int(round(want)))
            v = self._apply_speed(p.velocity_deg_s, p.accel_time_s)
            max_chunk = max(1, int(v * self.cfg.chunk_time_s)) if p.closed_loop else nsteps
            clamped = nsteps > max_chunk
            nsteps = min(nsteps, max_chunk)
            if not clamped:
                attempts += 1
                if final and attempts > 1:
                    with self._lock:
                        self.status.retries = attempts - 1
            early = None
            if p.closed_loop and not fine:
                zone = 0.5 * self.cfg.fine_zone_deg

                def early(d, goal=goal, direction=direction, zone=zone):
                    return (goal - d) * direction < zone

            outcome, steps_done, moved = self._chunk(cmd, direction * nsteps, early)
            if outcome in ("stopped", "retarget"):
                return outcome
            self._learn(direction, steps_done, moved)

            expected = abs(steps_done) / spd if spd else 0.0
            if p.closed_loop and expected > 10 * self.eres_deg and outcome == "done":
                if moved * direction < -max(5 * self.eres_deg, 0.3 * expected):
                    wrong_way += 1
                    if wrong_way >= 2:
                        raise AxisError(
                            "encoder moves opposite to the picomotor; flip encoder_sign in the config"
                        )
                else:
                    wrong_way = 0
                if abs(moved) < max(3 * self.eres_deg, 0.1 * expected):
                    stalls += 1
                    if stalls >= self.cfg.stall_chunks:
                        with self._lock:
                            self.status.stalled = True
                        raise AxisError("actuator stalled: encoder not following picomotor steps")
                else:
                    stalls = 0

    def _do_move(self, cmd: _Cmd) -> None:
        while True:
            with self._lock:
                self._retargeted = False
                target = cmd.args["target"]
                self.status.target = target
                self.status.retries = 0
                self.status.missed = False
            self._set_message(f"moving to {target:.4f}")
            p = self.params
            if p.high_limit_deg is not None and target > p.high_limit_deg + 1e-12:
                raise AxisError(f"target {target} above high limit {p.high_limit_deg}")
            if p.low_limit_deg is not None and target < p.low_limit_deg - 1e-12:
                raise AxisError(f"target {target} below low limit {p.low_limit_deg}")
            dial = self._poll_position()
            bd = p.backlash_deg if p.closed_loop else 0.0
            outcome = "done"
            if bd and (math.copysign(1, target - dial) != math.copysign(1, bd) or abs(target - dial) > abs(bd)):
                outcome = self._servo(cmd, target - bd, max(p.deadband_deg, abs(bd) / 4), final=False)
            if outcome == "done":
                outcome = self._servo(cmd, target, p.deadband_deg, final=True)
            if outcome == "retarget":
                continue
            if outcome == "stopped":
                self._set_message("stopped")
            elif outcome == "missed":
                st = self.get_status()
                self._set_message(f"missed target by {st.dial - target:+.5f} deg after {p.max_retries} retries")
            else:
                st = self.get_status()
                self._set_message(f"in position ({st.dial - target:+.5f} deg, {st.retries} retries)")
            return

    def _do_jog(self, cmd: _Cmd) -> None:
        p = self.params
        ax = self.cfg.picomotor_axis
        direction = cmd.args["direction"]
        dial = self._poll_position()
        lim = p.high_limit_deg if direction > 0 else p.low_limit_deg
        if lim is not None and (dial - lim) * direction >= 0:
            raise AxisError("already at soft limit")
        self._apply_speed(p.jog_velocity_deg_s, p.accel_time_s)
        with self._lock:
            self.status.moving = True
            self.status.direction = direction
            self.status.target = None
        self._set_message("jogging " + ("+" if direction > 0 else "-"))
        self.pm.move_indefinite(ax, direction)
        try:
            while True:
                time.sleep(self.cfg.poll_period_s)
                dial = self._poll_position()
                if self._stop_requested(cmd):
                    break
                if lim is not None and (dial - lim) * direction >= 0:
                    self._set_message("jog stopped at soft limit")
                    break
        finally:
            self.pm.stop(ax)
            self._drain_motion(cmd)
            self._read_steps()
        with self._lock:
            if not self.status.message.startswith("jog stopped"):
                self.status.message = "jog stopped"

    def _do_reset_multiturn(self, cmd: _Cmd) -> None:
        with self._lock:
            keep = self.params.closed_loop and self.status.valid and math.isfinite(self.status.dial)
            dial_before = self.status.dial
        self._set_message("resetting AksIM-2 multiturn counter")
        result = self.enc.reset_multiturn(cmd.args["value"]) or "multiturn reset"
        time.sleep(0.2)
        with self._lock:
            # The turn count changed on purpose: rebuild the unwrapped count from scratch.
            self._total_counts = None
            self._last_st = None
            self._persisted_counts = None
        try:
            self._poll_detailed_status_now()
            dial = self._poll_position()
        except EncoderError as exc:
            raise AxisError(f"multiturn reset sent, but the encoder still reports an error: {exc}") from None
        with self._lock:
            if self.status.encoder_error:
                raise AxisError("multiturn reset sent, but the encoder still reports an error (nE)")
            if keep and self.params.closed_loop:
                self.zero_offset_deg += dial_before - dial
                self.status.dial = self._counts_to_dial(self._total_counts)
            self.status.homed = False
        self._save_state(force=True)
        self._set_message(
            f"{result}; encoder OK"
            + (" (dial kept)" if keep else " (re-reference the position with SET if needed)")
        )

    def _poll_detailed_status_now(self) -> None:
        if self._detailed_status_ok:
            try:
                self.read_detailed_status()
            except HW_ERRORS:
                pass

    def _do_calibrate(self, cmd: _Cmd) -> None:
        n = max(self.cfg.min_learn_steps, cmd.args["steps"])
        p = self.params
        self._apply_speed(p.velocity_deg_s, p.accel_time_s)
        results = {}
        for direction in (1, -1):
            self._set_message(f"calibrating ({'+' if direction > 0 else '-'})")
            outcome, steps_done, moved = self._chunk(cmd, direction * n)
            if outcome != "done":
                self._set_message("calibration interrupted")
                return
            if moved * direction <= 0:
                raise AxisError(
                    f"calibration: {direction * n:+d} steps moved the encoder {moved:+.5f} deg; "
                    "check encoder_sign / coupling"
                )
            if abs(moved) < 20 * self.eres_deg:
                raise AxisError("calibration: motion too small to measure; increase calibration steps")
            results[direction] = abs(steps_done) / abs(moved)
        with self._lock:
            self.spd = dict(results)
            self._spd0 = dict(results)
            self._publish_spd()
        self._save_state(force=True)
        self._set_message(f"calibrated: {results[1]:.1f} steps/deg (+), {results[-1]:.1f} steps/deg (-)")

    # ------------------------------------------------------------------ persistence
    def _load_state(self) -> None:
        path = self.cfg.state_file
        if not path or not os.path.exists(path):
            return
        try:
            with open(path) as f:
                st = json.load(f)
            self.zero_offset_deg = float(st.get("zero_offset_deg", 0.0))
            same_source = st.get("count_source", "encoder-multiturn" if self._has_multiturn else "software-unwrap")
            if "last_counts" in st and st.get("counts_per_rev") == self.cpr and same_source == self._count_source:
                self._persisted_counts = int(st["last_counts"])
            elif "last_counts" in st:
                log.warning("state file turn count ignored (encoder resolution or multiturn setting changed); "
                            "re-reference the position with SET")
            if "steps_per_deg_fwd" in st and "steps_per_deg_rev" in st:
                self.spd = {1: float(st["steps_per_deg_fwd"]), -1: float(st["steps_per_deg_rev"])}
            if "open_loop_dial" in st:
                self._ol_dial = float(st["open_loop_dial"])
            if "calibrated_fwd" in st and "calibrated_rev" in st:
                self._spd0 = {1: float(st["calibrated_fwd"]), -1: float(st["calibrated_rev"])}
            log.info("loaded axis state from %s", path)
        except (OSError, ValueError, TypeError) as exc:
            log.warning("could not read state file %s: %s", path, exc)

    def _save_state(self, force: bool = False) -> None:
        path = self.cfg.state_file
        if not path:
            return
        now = time.monotonic()
        if not force and now - self._last_state_save < 5.0:
            return
        with self._lock:
            if self._total_counts is None and self.params.closed_loop and not force:
                return
            st = {
                "zero_offset_deg": self.zero_offset_deg,
                "counts_per_rev": self.cpr,
                "count_source": self._count_source,
                "steps_per_deg_fwd": self.spd[1],
                "steps_per_deg_rev": self.spd[-1],
                "calibrated_fwd": self._spd0[1],
                "calibrated_rev": self._spd0[-1],
                "open_loop_dial": self._ol_dial,
            }
            if self._total_counts is not None:
                st["last_counts"] = self._total_counts
        tmp = path + ".tmp"
        try:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            with open(tmp, "w") as f:
                json.dump(st, f, indent=2)
            os.replace(tmp, path)
            self._last_state_save = now
        except OSError as exc:
            log.warning("could not write state file %s: %s", path, exc)
