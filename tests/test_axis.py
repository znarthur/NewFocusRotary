import json
import time

import pytest

from newfocus_rotary.axis import AxisConfig, AxisParams, ClosedLoopRotaryAxis
from newfocus_rotary.e201 import BissFrameFormat
from newfocus_rotary.sim import SimE201, SimPicomotor8742, SimRotaryStage, SimStageParams


def make_axis(tmp_path=None, stage_kw=None, cfg_kw=None, params_kw=None, fmt=None):
    stage = SimRotaryStage(SimStageParams(speedup=20, **(stage_kw or {})))
    cfg = AxisConfig(steps_per_degree=1000, poll_period_s=0.01, settle_time_s=0.0, **(cfg_kw or {}))
    if tmp_path is not None:
        cfg.state_file = str(tmp_path / "state.json")
    axis = ClosedLoopRotaryAxis(
        SimPicomotor8742(stage), SimE201(stage, fmt or BissFrameFormat()), cfg, AxisParams(**(params_kw or {}))
    )
    axis.start()
    return axis, stage


@pytest.fixture
def axis():
    ax, stage = make_axis()
    yield ax
    ax.close()


def test_closed_loop_reaches_target_both_directions(axis):
    start = axis.get_status().dial
    for target in (start + 7.5, start + 7.503, start - 4.0, start - 3.99):
        axis.move_to(target)
        st = axis.wait_idle(60)
        assert abs(st.dial - target) <= axis.params.deadband_deg
        assert not st.missed and not st.problem
    st = axis.get_status()
    # learned steps/degree reflect the simulated asymmetric stage (1111 fwd, 909 rev nominal)
    assert st.steps_per_deg_fwd > st.steps_per_deg_rev


def test_open_loop_is_worse_than_closed_loop():
    ax, stage = make_axis(params_kw={"closed_loop": False})
    try:
        start = ax.get_status().dial
        angle0 = stage.angle
        ax.move_to(start + 10)
        st = ax.wait_idle(60)
        assert st.dial == pytest.approx(start + 10, abs=0.002)  # reported: integrated steps
        assert abs((stage.angle - angle0) - 10) > 0.05  # real motion: 1000 steps/deg estimate is wrong
    finally:
        ax.close()


def test_retarget_while_moving(axis):
    start = axis.get_status().dial
    axis.move_to(start + 30)
    time.sleep(0.3)
    assert axis.get_status().busy
    axis.move_to(start + 2)
    st = axis.wait_idle(60)
    assert abs(st.dial - (start + 2)) <= axis.params.deadband_deg


def test_stop(axis):
    start = axis.get_status().dial
    axis.move_to(start + 50)
    time.sleep(0.3)
    axis.stop()
    st = axis.wait_idle(5)
    assert st.message == "stopped"
    assert start < st.dial < start + 50


def test_jog_stops_at_soft_limit():
    ax, stage = make_axis(params_kw={"jog_velocity_deg_s": 2.0})
    try:
        start = ax.get_status().dial
        ax.set_params(high_limit_deg=start + 1.0, low_limit_deg=start - 100)
        ax.jog(+1)
        st = ax.wait_idle(30)
        assert "soft limit" in st.message
        assert start + 1.0 <= st.dial < start + 1.5
    finally:
        ax.close()


def test_move_outside_limit_refused(axis):
    start = axis.get_status().dial
    axis.set_params(high_limit_deg=start + 1, low_limit_deg=start - 1)
    axis.move_to(start + 5)
    st = axis.wait_idle(5)
    assert st.problem and "above high limit" in st.message


def test_stall_detected():
    ax, stage = make_axis()
    try:
        stage.stuck = True
        ax.move_to(ax.get_status().dial + 5)
        st = ax.wait_idle(60)
        assert st.stalled and st.problem
    finally:
        ax.close()


def test_wrong_encoder_sign_detected():
    ax, stage = make_axis(stage_kw={"encoder_sign": -1})
    try:
        ax.move_to(ax.get_status().dial + 5)
        st = ax.wait_idle(60)
        assert st.problem and "encoder_sign" in st.message
    finally:
        ax.close()


def test_encoder_error_bit_aborts():
    ax, stage = make_axis()
    try:
        stage.fail_encoder = True
        time.sleep(0.3)
        st = ax.get_status()
        assert st.encoder_error and st.problem
        stage.fail_encoder = False
        ax.move_to(st.dial + 1)
        st = ax.wait_idle(30)
        assert not st.problem and not st.encoder_error
    finally:
        ax.close()


def test_set_position_and_unwrap_persistence(tmp_path):
    ax, stage = make_axis(tmp_path, stage_kw={"encoder_offset_deg": 350.0, "initial_angle_deg": 0.0})
    try:
        ax.set_dial_position(100.0)
        assert abs(ax.get_status().dial - 100.0) < 1e-3
        ax.move_to(115.0)  # crosses the encoder's 360 -> 0 wrap
        st = ax.wait_idle(60)
        assert abs(st.dial - 115.0) <= ax.params.deadband_deg
        angle = stage.angle
    finally:
        ax.close()
    saved = json.loads((tmp_path / "state.json").read_text())
    assert saved["last_counts"] > 1 << 20  # unwrapped past one turn

    # Restart on the same (moved) stage: position must come back unchanged.
    stage2 = SimRotaryStage(SimStageParams(speedup=20, encoder_offset_deg=350.0, initial_angle_deg=angle))
    cfg = AxisConfig(steps_per_degree=1000, poll_period_s=0.01, state_file=str(tmp_path / "state.json"))
    ax2 = ClosedLoopRotaryAxis(SimPicomotor8742(stage2), SimE201(stage2), cfg)
    ax2.start()
    try:
        assert abs(ax2.get_status().dial - 115.0) < 0.01
    finally:
        ax2.close()


def test_multiturn_encoder():
    ax, stage = make_axis(fmt=BissFrameFormat(singleturn_bits=20, multiturn_bits=16),
                          stage_kw={"encoder_offset_deg": -10.0, "initial_angle_deg": 5.0})
    try:
        assert ax.get_status().dial == pytest.approx(-5.0, abs=1e-3)
        ax.move_to(-12.0)
        st = ax.wait_idle(60)
        assert abs(st.dial + 12.0) <= ax.params.deadband_deg
    finally:
        ax.close()


def test_backlash_final_approach_direction():
    ax, stage = make_axis(params_kw={"backlash_deg": 0.5})
    try:
        start = ax.get_status().dial
        ax.move_to(start - 3)
        st = ax.wait_idle(60)
        assert abs(st.dial - (start - 3)) <= ax.params.deadband_deg
        assert st.direction == 1  # final approach in +BDST direction
    finally:
        ax.close()


def test_calibration_measures_asymmetry(axis):
    axis.calibrate(3000)
    st = axis.wait_idle(60)
    assert "calibrated" in st.message
    # nominal sim: 1/0.0009 = 1111 (fwd), 1/0.0011 = 909 (rev), +/-15% load ripple
    assert 900 < st.steps_per_deg_fwd < 1350
    assert 750 < st.steps_per_deg_rev < 1100


def test_disabled_axis_refuses_motion(axis):
    from newfocus_rotary.axis import AxisError

    axis.set_params(enabled=False)
    with pytest.raises(AxisError):
        axis.move_to(0.0)


def test_open_loop_moves_despite_encoder_error():
    ax, stage = make_axis()
    try:
        stage.fail_encoder = True
        time.sleep(0.2)
        start_angle = stage.angle
        dial = ax.get_status().dial
        ax.move_to(dial + 5)
        st = ax.wait_idle(10)
        assert st.problem  # closed loop refuses to move on a faulty encoder
        assert "UEIP=No" in st.message
        assert stage.angle == pytest.approx(start_angle, abs=1e-6)

        ax.set_params(closed_loop=False)
        st = ax.get_status()
        assert not st.problem and st.dial == pytest.approx(dial, abs=1e-3)  # no readback jump
        ax.move_to(dial + 5)
        st = ax.wait_idle(30)
        assert not st.problem and not st.missed
        assert st.dial == pytest.approx(dial + 5, abs=0.002)  # integrated from steps
        assert st.encoder_error  # still reported
        moved = stage.angle - start_angle
        assert 3.5 < moved < 6.5  # really moved; open-loop accuracy only
        ax.move_to(dial + 2)  # reverse direction uses the reverse steps/degree
        st = ax.wait_idle(30)
        assert st.dial == pytest.approx(dial + 2, abs=0.002)
    finally:
        ax.close()


def test_no_encoder_at_all():
    stage = SimRotaryStage(SimStageParams(speedup=20))
    cfg = AxisConfig(steps_per_degree=1000, poll_period_s=0.01, settle_time_s=0.0)
    ax = ClosedLoopRotaryAxis(SimPicomotor8742(stage), None, cfg)
    ax.start()
    try:
        assert not ax.params.closed_loop and not ax.has_encoder
        ax.set_params(closed_loop=True)  # cannot be enabled without an encoder
        assert not ax.params.closed_loop
        ax.set_dial_position(10.0)
        ax.move_to(12.0)
        st = ax.wait_idle(30)
        assert st.dial == pytest.approx(12.0, abs=0.002)
        assert stage.steps == 2000
        ax.jog(-1)
        time.sleep(0.2)
        ax.stop()
        st = ax.wait_idle(5)
        assert st.dial < 12.0
    finally:
        ax.close()


def test_open_loop_position_persists(tmp_path):
    ax, stage = make_axis(tmp_path, params_kw={"closed_loop": False})
    try:
        ax.set_dial_position(30.0)
        ax.move_to(31.5)
        ax.wait_idle(30)
    finally:
        ax.close()
    ax2, _ = make_axis(tmp_path, params_kw={"closed_loop": False})
    try:
        assert ax2.get_status().dial == pytest.approx(31.5, abs=0.002)
    finally:
        ax2.close()


def test_reset_multiturn_clears_encoder_error():
    ax, stage = make_axis()
    try:
        stage.fail_encoder = True
        time.sleep(0.3)
        st = ax.get_status()
        assert st.encoder_error and st.encoder_detailed_status == 1
        ax.reset_multiturn(0)
        st = ax.wait_idle(10)
        assert not st.encoder_error and not st.problem, st.message
        assert "multiturn counter reset" in st.message
        ax.move_to(st.dial + 1.0)  # closed loop works again
        st2 = ax.wait_idle(30)
        assert abs(st2.dial - (st.dial + 1.0)) <= ax.params.deadband_deg
    finally:
        ax.close()


def test_reset_multiturn_keeps_dial_when_valid():
    ax, stage = make_axis(fmt=BissFrameFormat(singleturn_bits=18, multiturn_bits=16),
                          stage_kw={"encoder_offset_deg": 3 * 360 + 50.0, "initial_angle_deg": 0.0})
    try:
        before = ax.get_status().dial
        assert before == pytest.approx(3 * 360 + 50.0, abs=0.01)
        ax.reset_multiturn(0)
        st = ax.wait_idle(10)
        assert st.dial == pytest.approx(before, abs=0.01)  # turn count changed, dial did not
        assert st.encoder_counts < 1 << 18
    finally:
        ax.close()
