"""Channel Access IOC that presents the closed-loop axis as an EPICS motor record.

The PV ``$(P)$(M)`` is served with the full motor-record field set (VAL, RBV,
DMOV, STOP, TWF, HLM, SET, ...) so standard clients work unchanged: MEDM/caQtDM
``motorx*.adl`` screens, ``ophyd.EpicsMotor`` / Bluesky, SPEC, pyepics and
``caput -c`` (put-callback completes when the move finishes, as with the
real record).

Extra diagnostic/commissioning PVs live under ``$(P)$(M):`` (see README).
"""

from __future__ import annotations

import argparse
import asyncio
import copy
import functools
import logging
import math
import sys
import textwrap
from typing import Optional

from caproto import AlarmSeverity, AlarmStatus, ChannelType, SkipWrite
from caproto.server import PVGroup, pvproperty, run
from caproto.server.records import MotorFields

from .axis import AxisError, AxisStatus, ClosedLoopRotaryAxis
from .config import Config, load_config

log = logging.getLogger(__name__)

# MSTA bits (motor record, motor.h)
MSTA_DIRECTION = 1 << 0
MSTA_DONE = 1 << 1
MSTA_PLUS_LS = 1 << 2
MSTA_HOMELS = 1 << 3
MSTA_POSITION = 1 << 5  # closed-loop position control enabled
MSTA_SLIP_STALL = 1 << 6
MSTA_HOME = 1 << 7
MSTA_PRESENT = 1 << 8  # encoder present
MSTA_PROBLEM = 1 << 9
MSTA_MOVING = 1 << 10
MSTA_GAIN_SUPPORT = 1 << 11
MSTA_COMM_ERR = 1 << 12
MSTA_MINUS_LS = 1 << 13
MSTA_HOMED = 1 << 14

DSIGN = {"Pos": 1, "Neg": -1}


def _delegate(prop: pvproperty, key: str) -> pvproperty:
    """Copy a motor-record field definition and route client puts to the IOC group."""
    p = copy.copy(prop)

    async def put(fields, instance, value):
        return await fields.parent.group.field_put(key, instance, value)

    return p.putter(put)


class RotaryMotorFields(MotorFields):
    """Motor-record fields with the active ones wired to the controller."""

    stop = _delegate(MotorFields.stop, "STOP")
    stop_pause_move_go = _delegate(MotorFields.stop_pause_move_go, "SPMG")
    tweak_motor_forward = _delegate(MotorFields.tweak_motor_forward, "TWF")
    tweak_motor_reverse = _delegate(MotorFields.tweak_motor_reverse, "TWR")
    relative_value = _delegate(MotorFields.relative_value, "RLV")
    home_forward = _delegate(MotorFields.home_forward, "HOMF")
    home_reverse = _delegate(MotorFields.home_reverse, "HOMR")
    jog_motor_forward = _delegate(MotorFields.jog_motor_forward, "JOGF")
    jog_motor_reverse = _delegate(MotorFields.jog_motor_reverse, "JOGR")
    dial_desired_value = _delegate(MotorFields.dial_desired_value, "DVAL")
    raw_desired_value = _delegate(MotorFields.raw_desired_value, "RVAL")
    user_high_limit = _delegate(MotorFields.user_high_limit, "HLM")
    user_low_limit = _delegate(MotorFields.user_low_limit, "LLM")
    dial_high_limit = _delegate(MotorFields.dial_high_limit, "DHLM")
    dial_low_limit = _delegate(MotorFields.dial_low_limit, "DLLM")
    user_offset = _delegate(MotorFields.user_offset, "OFF")
    user_direction = _delegate(MotorFields.user_direction, "DIR")
    set_use_switch = _delegate(MotorFields.set_use_switch, "SET")
    set_set_mode = _delegate(MotorFields.set_set_mode, "SSET")
    set_use_mode = _delegate(MotorFields.set_use_mode, "SUSE")
    velocity = _delegate(MotorFields.velocity, "VELO")
    jog_velocity = _delegate(MotorFields.jog_velocity, "JVEL")
    seconds_to_velocity = _delegate(MotorFields.seconds_to_velocity, "ACCL")
    motor_step_size = _delegate(MotorFields.motor_step_size, "MRES")
    enable_control = _delegate(MotorFields.enable_control, "CNEN")
    use_encoder_if_present = _delegate(MotorFields.use_encoder_if_present, "UEIP")
    sync_position = _delegate(MotorFields.sync_position, "SYNC")
    display_precision = _delegate(MotorFields.display_precision, "PREC")


class RotaryMotorIOC(PVGroup):
    """
    EPICS motor record for a picomotor rotary actuator (New Focus 8742) servoed
    on a Renishaw/RLS AksIM-2 absolute encoder read through an E201-9B.
    """

    motor = pvproperty(value=0.0, name="", record=RotaryMotorFields, precision=4,
                       doc="Closed-loop rotary picomotor")

    status_msg = pvproperty(name=":Status", value="starting", dtype=ChannelType.CHAR,
                            max_length=256, report_as_string=False, read_only=True,
                            doc="Controller status message (char waveform; caget -S)")
    steps_per_deg_fwd = pvproperty(name=":StepsPerDegFwd", value=0.0, precision=2,
                                   doc="Learned picomotor steps per degree, positive direction")
    steps_per_deg_rev = pvproperty(name=":StepsPerDegRev", value=0.0, precision=2,
                                   doc="Learned picomotor steps per degree, negative direction")
    calibrate = pvproperty(name=":Calibrate", value=0, doc="1: measure steps/degree with a +/- move")
    calibrate_steps = pvproperty(name=":CalibrateSteps", value=2000, doc="Steps per calibration move")
    reset_learning = pvproperty(name=":ResetLearning", value=0,
                                doc="1: reset steps/degree to the last calibration")
    enc_counts = pvproperty(name=":EncCounts", value=0, read_only=True, doc="Unwrapped encoder counts")
    enc_singleturn = pvproperty(name=":EncSingleTurn", value=0, read_only=True, doc="Raw single-turn counts")
    step_count = pvproperty(name=":StepCount", value=0, read_only=True, doc="8742 step counter (TP?)")
    enc_error = pvproperty(name=":EncError", value=0, read_only=True, doc="AksIM-2 error bit (position invalid)")
    enc_warning = pvproperty(name=":EncWarning", value=0, read_only=True, doc="AksIM-2 warning bit")
    comm_error = pvproperty(name=":CommError", value=0, read_only=True, doc="Communication error")
    crc_errors = pvproperty(name=":CRCErrors", value=0, read_only=True, doc="BiSS CRC failures since start")
    enc_detailed_status = pvproperty(name=":EncDetailedStatus", value=-1, read_only=True,
                                     doc="AksIM-2 detailed status register (-1 = not read)")
    enc_blips = pvproperty(name=":EncBlips", value=0, read_only=True,
                           doc="Encoder error blips ridden through (dead reckoning)")
    pos_estimated = pvproperty(name=":PosEstimated", value=0, read_only=True,
                               doc="1 while the readback is dead-reckoned through an encoder blip")
    reset_multiturn = pvproperty(name=":ResetMultiturn", value=0,
                                 doc="1: reset the AksIM-2 multiturn counter to :MultiturnValue")
    multiturn_value = pvproperty(name=":MultiturnValue", value=0, doc="Turn count written by :ResetMultiturn")
    zero_preset = pvproperty(name=":ZeroPreset", value=0,
                             doc="1: store the current position as AksIM-2 zero (writes encoder flash)")

    def __init__(self, axis: ClosedLoopRotaryAxis, cfg: Config, **kwargs):
        kwargs.setdefault("prefix", f"{cfg.ioc.prefix}{cfg.ioc.motor}")
        super().__init__(**kwargs)
        self.axis = axis
        self.cfg = cfg
        self._gen = 0
        self._done_gen = 0
        self._dmov_pending = False
        self._sync_val_on_done = False
        self._spmg_move_once = False
        self._last_alarm = (None, None)
        self._status: AxisStatus = axis.get_status()

    # ----------------------------------------------------------------- helpers
    @property
    def F(self) -> RotaryMotorFields:
        return self.motor.field_inst

    async def _call(self, fn, *args, **kwargs):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, functools.partial(fn, *args, **kwargs))

    @staticmethod
    async def _w(field, value):
        """Write a field internally (no putter) only if it changed."""
        cur = field.value
        if isinstance(value, float) and isinstance(cur, float):
            if cur == value or (math.isnan(cur) and math.isnan(value)):
                return
        elif cur == value:
            return
        await field.write(value, verify_value=False)

    def _dsign(self) -> int:
        return DSIGN.get(self.F.user_direction.value, 1)

    def _user_to_dial(self, user: float) -> float:
        return (user - self.F.user_offset.value) * self._dsign()

    def _dial_to_user(self, dial: float) -> float:
        return self._dsign() * dial + self.F.user_offset.value

    def _dial_limits(self) -> tuple[Optional[float], Optional[float]]:
        lo, hi = self.F.dial_low_limit.value, self.F.dial_high_limit.value
        if lo == 0 and hi == 0:
            return None, None
        return lo, hi

    async def _sync_params(self) -> None:
        F = self.F
        lo, hi = self._dial_limits()
        await self._call(
            self.axis.set_params,
            deadband_deg=max(abs(F.retry_deadband.value), 1e-9),
            max_retries=int(F.max_retry_count.value),
            velocity_deg_s=max(F.velocity.value, 1e-6),
            accel_time_s=max(F.seconds_to_velocity.value, 1e-3),
            backlash_deg=F.bl_distance.value,
            jog_velocity_deg_s=max(F.jog_velocity.value, 1e-6),
            closed_loop=F.use_encoder_if_present.value == "Yes",
            enabled=F.enable_control.value == "Enable",
            low_limit_deg=lo,
            high_limit_deg=hi,
        )

    async def _start(self, fn, *args) -> Optional[int]:
        """Issue an axis motion command; marks DMOV=0. Returns the generation, None if refused."""
        await self._sync_params()
        try:
            await self._call(fn, *args)
        except AxisError as exc:
            await self._w(self.status_msg, f"refused: {exc}"[:255])
            return None
        self._gen += 1
        self._dmov_pending = True
        await self._w(self.F.done_moving_to_value, 0)
        await self._w(self.F.motion_in_progress, 1)
        return self._gen

    async def _wait_gen(self, gen: int) -> None:
        while self._done_gen < gen:
            await asyncio.sleep(0.02)

    async def _put_user_target(self, user: float):
        """Common VAL logic. Returns ``(value_to_store, generation_or_None)``."""
        F = self.F
        if F.set_use_switch.value == "Set":
            await self._set_position_user(user)
            return user, None
        dial = self._user_to_dial(user)
        lo, hi = self._dial_limits()
        if (hi is not None and dial > hi) or (lo is not None and dial < lo):
            await self._w(F.limit_violation, 1)
            await self._w(self.status_msg, f"limit violation: dial {dial:.6g} outside [{lo}, {hi}]")
            return self.motor.value, None
        await self._w(F.limit_violation, 0)
        await F.last_user_des_val.write(self.motor.value, verify_value=False)
        await self._w(F.dial_desired_value, dial)
        await self._w(F.raw_desired_value, int(round(dial / self.axis.eres_deg)))
        await self._w(F.last_dial_des_val, dial)
        if F.stop_pause_move_go.value in ("Stop", "Pause") or F.enable_control.value != "Enable":
            return user, None
        self._sync_val_on_done = False
        gen = await self._start(self.axis.move_to, dial)
        return user, gen

    async def _move_user(self, user: float) -> Optional[int]:
        store, gen = await self._put_user_target(user)
        await self.motor.write(store, verify_value=False)
        return gen

    async def _set_position_user(self, user: float) -> None:
        """SET mode put to VAL."""
        F = self.F
        if F.offset_freeze_switch.value == "Variable":
            new_off = user - self._dsign() * F.dial_readback_value.value
            await F.user_offset.write(new_off, verify_value=False)
            await self._recompute_user_values()
        else:
            await self._set_position_dial(self._user_to_dial(user))

    async def _set_position_dial(self, dial: float) -> None:
        try:
            await self._call(self.axis.set_dial_position, dial)
        except AxisError as exc:
            await self._w(self.status_msg, f"set refused: {exc}"[:255])
            return
        st = await self._call(self.axis.get_status)
        await self._publish_status(st)
        await self._w(self.F.dial_desired_value, st.dial)
        await self.motor.write(self._dial_to_user(st.dial), verify_value=False)
        await self._w(self.status_msg, f"position redefined: dial {st.dial:.6g}")

    async def _recompute_user_values(self) -> None:
        """After OFF/DIR change: user values follow from dial values."""
        F = self.F
        s = self._dsign()
        off = F.user_offset.value
        await self.motor.write(self._dial_to_user(F.dial_desired_value.value), verify_value=False)
        await self._w(F.user_readback_value, self._dial_to_user(F.dial_readback_value.value))
        dhi, dlo = F.dial_high_limit.value, F.dial_low_limit.value
        if s > 0:
            await self._w(F.user_high_limit, dhi + off)
            await self._w(F.user_low_limit, dlo + off)
        else:
            await self._w(F.user_high_limit, -dlo + off)
            await self._w(F.user_low_limit, -dhi + off)

    async def _set_dial_limit(self, high: Optional[float] = None, low: Optional[float] = None) -> None:
        F = self.F
        if high is not None:
            await self._w(F.dial_high_limit, float(high))
        if low is not None:
            await self._w(F.dial_low_limit, float(low))
        await self._recompute_user_values()
        await self._sync_params()

    async def _sync_val_to_rbv(self) -> None:
        F = self.F
        await self._w(F.dial_desired_value, F.dial_readback_value.value)
        await self._w(F.raw_desired_value, F.raw_readback_value.value)
        await self.motor.write(F.user_readback_value.value, verify_value=False)

    # ----------------------------------------------------------------- VAL
    @motor.putter
    async def motor(self, instance, value):
        store, gen = await self._put_user_target(float(value))
        if gen is None:
            return store
        await instance.write(store, verify_value=False)
        await self._wait_gen(gen)  # put-callback (caput -c) completes when the move is done
        raise SkipWrite()

    # ----------------------------------------------------------------- other fields
    async def field_put(self, key: str, instance, value):
        F = self.F
        if key == "STOP":
            if value:
                await self._do_stop(sync_val=True)
            return 0
        if key == "SPMG":
            await self._w(F.last_spmg, F.stop_pause_move_go.value)
            if value == "Stop":
                await self._do_stop(sync_val=True)
            elif value == "Pause":
                await self._do_stop(sync_val=False)
            else:
                self._spmg_move_once = value == "Move"
                dval = F.dial_desired_value.value
                if abs(dval - F.dial_readback_value.value) > abs(F.retry_deadband.value):
                    await instance.write(value, verify_value=False)  # so _put_user_target sees Go/Move
                    self._sync_val_on_done = False
                    gen = await self._start(self.axis.move_to, dval)
                    if gen is None:
                        self._spmg_move_once = False
                elif value == "Move":
                    self._spmg_move_once = False
                    return "Pause"
            return value
        if key in ("TWF", "TWR"):
            if value:
                step = F.tweak_step_size.value * (1 if key == "TWF" else -1)
                await self._move_user(self.motor.value + step)
            return 0
        if key == "RLV":
            if value:
                await self._move_user(self.motor.value + float(value))
                await F.last_rel_value.write(float(value), verify_value=False)
            return 0
        if key in ("HOMF", "HOMR"):
            if value:
                self._sync_val_on_done = True
                if await self._start(self.axis.home) is None:
                    return 0
                return 1
            return 0
        if key in ("JOGF", "JOGR"):
            if value:
                if F.stop_pause_move_go.value in ("Stop", "Pause") or F.enable_control.value != "Enable":
                    return 0
                direction = (1 if key == "JOGF" else -1) * self._dsign()
                self._sync_val_on_done = True
                if await self._start(self.axis.jog, direction) is None:
                    return 0
                return 1
            await self._do_stop(sync_val=True)
            return 0
        if key == "DVAL":
            dval = float(value)
            if F.set_use_switch.value == "Set":
                await self._set_position_dial(dval)
                return self.F.dial_desired_value.value
            gen = await self._move_user(self._dial_to_user(dval))
            if gen is not None:
                await self._wait_gen(gen)
            return F.dial_desired_value.value
        if key == "RVAL":
            dval = int(value) * self.axis.eres_deg
            if F.set_use_switch.value == "Set":
                await self._set_position_dial(dval)
            else:
                gen = await self._move_user(self._dial_to_user(dval))
                if gen is not None:
                    await self._wait_gen(gen)
            return F.raw_desired_value.value
        if key in ("HLM", "LLM"):
            v = float(value) - F.user_offset.value
            s = self._dsign()
            if (key == "HLM") == (s > 0):
                await self._set_dial_limit(high=v * s)
            else:
                await self._set_dial_limit(low=v * s)
            raise SkipWrite()  # value written by _recompute_user_values
        if key == "DHLM":
            await self._set_dial_limit(high=float(value))
            raise SkipWrite()
        if key == "DLLM":
            await self._set_dial_limit(low=float(value))
            raise SkipWrite()
        if key in ("OFF", "DIR"):
            await instance.write(value, verify_value=False)
            await self._recompute_user_values()
            raise SkipWrite()
        if key == "SET":
            return value
        if key == "SSET":
            if value:
                await F.set_use_switch.write("Set", verify_value=False)
            return 0
        if key == "SUSE":
            if value:
                await F.set_use_switch.write("Use", verify_value=False)
            return 0
        if key in ("VELO", "JVEL"):
            v = abs(float(value))
            vmax = F.max_velocity.value
            if vmax > 0:
                v = min(v, vmax)
            if key == "VELO":
                await self._w(F.speed, v / 360.0)
            await instance.write(v, verify_value=False)
            await self._sync_params()
            raise SkipWrite()
        if key == "ACCL":
            await instance.write(max(float(value), 1e-3), verify_value=False)
            await self._sync_params()
            raise SkipWrite()
        if key == "MRES":
            v = abs(float(value))
            if v > 0:
                await self._call(self.axis.set_steps_per_degree, 1.0 / v)
            return v
        if key == "CNEN":
            if value != "Enable":
                await self._do_stop(sync_val=True)
            await instance.write(value, verify_value=False)
            await self._sync_params()
            raise SkipWrite()
        if key == "UEIP":
            await instance.write(value, verify_value=False)
            await self._sync_params()
            actual = "Yes" if self.axis.params.closed_loop else "No"
            if actual != value:
                await self._w(self.status_msg, "no encoder configured: open loop only")
            await instance.write(actual, verify_value=False)
            st = await self._call(self.axis.get_status)
            await self._publish_status(st)
            await self._sync_val_to_rbv()  # readback source changed; avoid an unintended move
            raise SkipWrite()
        if key == "SYNC":
            if value:
                await self._sync_val_to_rbv()
            return 0
        if key == "PREC":
            await self._broadcast_precision(int(value))
            return value
        raise ValueError(f"unhandled field {key}")  # pragma: no cover

    async def _do_stop(self, sync_val: bool) -> None:
        self._spmg_move_once = False
        if sync_val:
            self._sync_val_on_done = True
        await self._call(self.axis.stop)
        st = await self._call(self.axis.get_status)
        if not st.busy and sync_val:
            await self._publish_status(st)
            await self._sync_val_to_rbv()

    async def _broadcast_precision(self, prec: int) -> None:
        await self.motor.write_metadata(precision=prec)
        for prop in self.F.pvdb.values():
            if hasattr(prop, "precision"):
                await prop.write_metadata(precision=prec)

    # ----------------------------------------------------------------- extra PVs
    @steps_per_deg_fwd.putter
    async def steps_per_deg_fwd(self, instance, value):
        await self._call(self.axis.set_steps_per_degree, float(value), self.axis.spd[-1])
        return value

    @steps_per_deg_rev.putter
    async def steps_per_deg_rev(self, instance, value):
        await self._call(self.axis.set_steps_per_degree, self.axis.spd[1], float(value))
        return value

    @calibrate.putter
    async def calibrate(self, instance, value):
        if value:
            self._sync_val_on_done = True
            if await self._start(self.axis.calibrate, int(self.calibrate_steps.value)) is None:
                return 0
            return 1
        return 0

    @reset_multiturn.putter
    async def reset_multiturn(self, instance, value):
        if value:
            self._sync_val_on_done = True
            if await self._start(self.axis.reset_multiturn, int(self.multiturn_value.value)) is None:
                return 0
            return 1
        return 0

    @zero_preset.putter
    async def zero_preset(self, instance, value):
        if value:
            self._sync_val_on_done = True
            if await self._start(self.axis.zero_preset) is None:
                return 0
            return 1
        return 0

    @reset_learning.putter
    async def reset_learning(self, instance, value):
        if value:
            await self._call(self.axis.reset_learning)
        return 0

    # ----------------------------------------------------------------- startup / polling
    @motor.startup
    async def motor(self, instance, async_lib):
        await self._initialize_fields()
        period = 1.0 / max(self.cfg.ioc.update_rate_hz, 0.5)
        while True:
            try:
                st = await self._call(self.axis.get_status)
                await self._publish_status(st)
                await self._check_done(st)
            except Exception:  # pragma: no cover - keep the IOC alive
                log.exception("status update failed")
            await asyncio.sleep(period)

    async def _initialize_fields(self) -> None:
        F = self.F
        cfg, m = self.cfg, self.cfg.motor
        await self._broadcast_precision(cfg.ioc.precision)
        await F.engineering_units.write(cfg.ioc.egu, verify_value=False)
        await F.description.write(cfg.ioc.description[:40])  # putter updates the linked record doc
        for prop in (self.motor, F.user_readback_value, F.dial_readback_value, F.dial_desired_value,
                     F.user_high_limit, F.user_low_limit, F.dial_high_limit, F.dial_low_limit,
                     F.user_offset, F.tweak_step_size, F.retry_deadband, F.bl_distance,
                     F.relative_value, F.difference_dval_drbv):
            await prop.write_metadata(units=cfg.ioc.egu)
        await F.velocity.write(m.velocity, verify_value=False)
        await F.speed.write(m.velocity / 360.0, verify_value=False)
        await F.base_velocity.write(0.0, verify_value=False)
        await F.seconds_to_velocity.write(m.accel, verify_value=False)
        await F.retry_deadband.write(m.retry_deadband, verify_value=False)
        await F.max_retry_count.write(m.max_retries, verify_value=False)
        await F.bl_distance.write(m.backlash, verify_value=False)
        await F.jog_velocity.write(m.jog_velocity, verify_value=False)
        await F.tweak_step_size.write(m.tweak, verify_value=False)
        await F.home_velocity.write(m.velocity, verify_value=False)
        await F.user_offset.write(m.user_offset, verify_value=False)
        await F.user_direction.write(m.user_direction, verify_value=False)
        await F.dial_high_limit.write(m.dial_high_limit, verify_value=False)
        await F.dial_low_limit.write(m.dial_low_limit, verify_value=False)
        await F.encoder_step_size.write(self.axis.eres_deg, verify_value=False)
        await F.readback_step_size.write(self.axis.eres_deg, verify_value=False)
        await F.use_encoder_if_present.write("Yes" if self.axis.params.closed_loop else "No", verify_value=False)
        await F.enable_control.write("Enable", verify_value=False)
        await F.stop_pause_move_go.write("Go", verify_value=False)
        await F.set_use_switch.write("Use", verify_value=False)
        await F.offset_freeze_switch.write("Variable", verify_value=False)
        await F.new_target_monitor.write("Yes", verify_value=False)
        await F.egu_s_per_revolution.write(360.0, verify_value=False)
        await F.done_moving_to_value.write(1, verify_value=False)
        await F.card_number.write(0, verify_value=False)
        st = await self._call(self.axis.get_status)
        await self._publish_status(st)
        await self._w(F.dial_desired_value, st.dial)
        await self.motor.write(self._dial_to_user(st.dial), verify_value=False)
        await self._recompute_user_values()
        await self._sync_params()

    async def _check_done(self, st: AxisStatus) -> None:
        if not self._dmov_pending or st.busy:
            return
        F = self.F
        self._dmov_pending = False
        if self._sync_val_on_done:
            self._sync_val_on_done = False
            await self._sync_val_to_rbv()
        for fld in (F.home_forward, F.home_reverse, F.jog_motor_forward, F.jog_motor_reverse):
            await self._w(fld, 0)
        await self._w(self.calibrate, 0)
        await self._w(self.reset_multiturn, 0)
        await self._w(self.zero_preset, 0)
        if self._spmg_move_once:
            self._spmg_move_once = False
            await F.stop_pause_move_go.write("Pause", verify_value=False)
        await self._w(F.motion_in_progress, 0)
        await self._w(F.done_moving_to_value, 1)
        self._done_gen = self._gen

    async def _publish_status(self, st: AxisStatus) -> None:
        self._status = st
        F = self.F
        eres = self.axis.eres_deg
        dial = st.dial
        user = self._dial_to_user(dial) if math.isfinite(dial) else math.nan
        rraw = int(round(dial / eres)) if math.isfinite(dial) else 0
        await self._w(F.dial_readback_value, dial)
        await self._w(F.user_readback_value, user)
        await self._w(F.raw_readback_value, rraw)
        await self._w(F.raw_encoder_position, int(st.encoder_counts))
        await self._w(F.raw_motor_position, int(st.steps))
        dval = F.dial_desired_value.value
        await self._w(F.difference_dval_drbv, dval - dial if math.isfinite(dial) else 0.0)
        await self._w(F.difference_rval_rrbv, int(F.raw_desired_value.value) - rraw)
        await self._w(F.motor_is_moving, int(st.moving))
        tdir = 1 if st.direction * self._dsign() > 0 else 0
        await self._w(F.direction_of_travel, tdir)
        await self._w(F.raw_cmnd_direction, 1 if st.direction > 0 else 0)
        await self._w(F.ran_out_of_retries, int(st.missed))
        await self._w(F.retry_count, int(st.retries))
        rdbd = abs(F.retry_deadband.value)
        home = self.axis.params.home_position_deg
        await self._w(F.at_home, int(math.isfinite(dial) and abs(dial - home) <= rdbd))
        await self._w(F.user_high_limit_switch, 0)
        await self._w(F.user_low_limit_switch, 0)
        spd_avg = 0.5 * (st.steps_per_deg_fwd + st.steps_per_deg_rev)
        if spd_avg > 0:
            await self._w(F.motor_step_size, 1.0 / spd_avg)
            await self._w(F.steps_per_revolution, int(round(360.0 * spd_avg)))
            await self._w(F.max_velocity, 2000.0 / max(st.steps_per_deg_fwd, st.steps_per_deg_rev))

        closed = self.axis.params.closed_loop
        msta = MSTA_GAIN_SUPPORT
        if self.axis.has_encoder:
            msta |= MSTA_PRESENT
        if st.direction > 0:
            msta |= MSTA_DIRECTION
        if not st.busy:
            msta |= MSTA_DONE
        if closed:
            msta |= MSTA_POSITION
        if st.stalled:
            msta |= MSTA_SLIP_STALL
        if st.problem or (st.encoder_error and closed and not st.position_estimated):
            msta |= MSTA_PROBLEM
        if st.moving:
            msta |= MSTA_MOVING
        if st.comm_error:
            msta |= MSTA_COMM_ERR
        if st.homed:
            msta |= MSTA_HOMED
        await self._w(F.motor_status, msta)

        await self._w(self.status_msg, st.message[:255])
        await self._w(self.steps_per_deg_fwd, float(st.steps_per_deg_fwd))
        await self._w(self.steps_per_deg_rev, float(st.steps_per_deg_rev))
        await self._w(self.enc_counts, int(st.encoder_counts))
        await self._w(self.enc_singleturn, int(st.encoder_singleturn))
        await self._w(self.step_count, int(st.steps))
        await self._w(self.enc_error, int(st.encoder_error))
        await self._w(self.enc_warning, int(st.encoder_warning))
        await self._w(self.comm_error, int(st.comm_error))
        await self._w(self.crc_errors, int(st.crc_errors))
        await self._w(self.enc_blips, int(st.encoder_blips))
        await self._w(self.pos_estimated, int(st.position_estimated))
        if st.encoder_detailed_status is not None:
            await self._w(self.enc_detailed_status, int(st.encoder_detailed_status))

        if st.comm_error:
            alarm = (AlarmStatus.COMM, AlarmSeverity.INVALID_ALARM)
        elif (st.encoder_error and closed and not st.position_estimated) or st.stalled or st.problem:
            alarm = (AlarmStatus.STATE, AlarmSeverity.MAJOR_ALARM)
        elif st.missed or st.encoder_warning or st.encoder_error or not closed:
            alarm = (AlarmStatus.STATE, AlarmSeverity.MINOR_ALARM)
        else:
            alarm = (AlarmStatus.NO_ALARM, AlarmSeverity.NO_ALARM)
        if alarm != self._last_alarm:
            self._last_alarm = alarm
            await self.motor.alarm.write(status=alarm[0], severity=alarm[1])


def build_ioc(cfg: Config) -> tuple[RotaryMotorIOC, ClosedLoopRotaryAxis]:
    from .config import build_hardware

    pm, enc = build_hardware(cfg)
    axis = ClosedLoopRotaryAxis(pm, enc, cfg.axis, cfg.axis_params())
    axis.start()
    return RotaryMotorIOC(axis, cfg), axis


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        prog="newfocus-rotary-ioc",
        description=textwrap.dedent(RotaryMotorIOC.__doc__).strip(),
    )
    parser.add_argument("-c", "--config", help="YAML configuration file")
    parser.add_argument("--prefix", help="PV prefix $(P), e.g. '29idb:'")
    parser.add_argument("--motor", help="motor name $(M), e.g. 'm1'")
    parser.add_argument("--sim", action="store_true", help="use the built-in hardware simulator")
    parser.add_argument("--interfaces", nargs="+", default=None, help="interfaces to bind (default all)")
    parser.add_argument("--list-pvs", action="store_true", help="log PV names at startup")
    parser.add_argument("-v", "--verbose", action="count", default=0)
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose > 1 else logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    overrides = {}
    if args.prefix is not None:
        overrides["ioc.prefix"] = args.prefix
    if args.motor is not None:
        overrides["ioc.motor"] = args.motor
    if args.sim:
        overrides["picomotor.transport"] = "sim"
        overrides["encoder.transport"] = "sim"
    cfg = load_config(args.config, overrides)
    try:
        ioc, axis = build_ioc(cfg)
    except Exception as exc:
        print(f"Hardware initialisation failed: {exc}", file=sys.stderr)
        sys.exit(1)
    simulated = "sim" in (cfg.picomotor.transport, cfg.encoder.transport)
    print(f"Serving motor record {ioc.prefix} ({'SIMULATION' if simulated else 'hardware'})", file=sys.stderr)
    try:
        run(ioc.pvdb, interfaces=args.interfaces, log_pv_names=args.list_pvs)
    finally:
        axis.close()


if __name__ == "__main__":  # pragma: no cover
    main()
