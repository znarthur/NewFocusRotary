"""End-to-end tests: run the IOC (simulated hardware) and talk Channel Access to it."""

import os
import socket
import subprocess
import sys
import textwrap
import time

import pytest

PREFIX = "TST:"
M = PREFIX + "m1"


def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        udp = s.getsockname()[1]
    return udp


@pytest.fixture(scope="module")
def ioc(tmp_path_factory):
    port = _free_port()
    env = dict(os.environ)
    env.update(
        EPICS_CA_SERVER_PORT=str(port),
        EPICS_CAS_SERVER_PORT=str(port),
        EPICS_CA_REPEATER_PORT=str(port + 1),
        EPICS_CAS_BEACON_PORT=str(port + 1),
        EPICS_CA_ADDR_LIST="127.0.0.1",
        EPICS_CA_AUTO_ADDR_LIST="NO",
        EPICS_CAS_INTF_ADDR_LIST="127.0.0.1",
    )
    cfg = tmp_path_factory.mktemp("ioc") / "cfg.yaml"
    cfg.write_text(textwrap.dedent(f"""
        ioc: {{prefix: "{PREFIX}", motor: m1, update_rate_hz: 20}}
        picomotor: {{transport: sim}}
        encoder: {{transport: sim}}
        axis: {{steps_per_degree: 1000, poll_period_s: 0.01, settle_time_s: 0.0}}
        motor: {{velocity: 1.0, retry_deadband: 0.002, tweak: 0.5}}
        sim: {{speedup: 20, initial_angle_deg: 0.0, encoder_offset_deg: 20.0}}
    """))
    proc = subprocess.Popen(
        [sys.executable, "-m", "newfocus_rotary.ioc", "-c", str(cfg)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    old = {k: os.environ.get(k) for k in env}
    os.environ.update(env)
    from caproto.sync import client

    deadline = time.time() + 20
    while True:
        try:
            client.read(M + ".RBV", timeout=0.5)
            break
        except Exception:
            if time.time() > deadline or proc.poll() is not None:
                proc.kill()
                out = proc.stdout.read().decode()
                raise RuntimeError("IOC did not start:\n" + out)
            time.sleep(0.2)
    yield client
    proc.terminate()
    try:
        proc.wait(5)
    except subprocess.TimeoutExpired:
        proc.kill()
    for k, v in old.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v


def get(client, pv):
    data = client.read(pv, timeout=2).data
    v = data[0] if len(data) == 1 else data
    if isinstance(v, bytes):
        return v.decode()
    return v


def get_str(client, pv):
    data = client.read(pv, timeout=2).data
    if len(data) and isinstance(data[0], bytes):
        return b"".join(data).rstrip(b"\x00").decode()
    return bytes(bytearray(int(x) for x in data)).rstrip(b"\x00").decode()


def put(client, pv, value, wait=False, timeout=30):
    client.write(pv, value, notify=wait, timeout=timeout)


def wait_dmov(client, timeout=30):
    t0 = time.time()
    time.sleep(0.15)
    while get(client, M + ".DMOV") != 1:
        assert time.time() - t0 < timeout, "move did not finish"
        time.sleep(0.05)


def test_initial_state(ioc):
    assert get(ioc, M + ".DMOV") == 1
    assert get(ioc, M + ".EGU") == "deg"
    rbv = get(ioc, M + ".RBV")
    assert abs(rbv - 20.0) < 0.01  # encoder_offset_deg
    assert abs(get(ioc, M) - rbv) < 0.002  # VAL initialised to RBV (+- encoder noise)
    msta = get(ioc, M + ".MSTA")
    assert msta & (1 << 1) and msta & (1 << 8) and msta & (1 << 5)  # DONE, PRESENT, POSITION
    assert get(ioc, M + ".ERES") == pytest.approx(360 / 2**20)


def test_move_with_put_callback(ioc):
    t0 = time.time()
    put(ioc, M, 23.0, wait=True)
    assert time.time() - t0 > 0.1  # put-callback waited for the move
    assert get(ioc, M + ".DMOV") == 1
    assert abs(get(ioc, M + ".RBV") - 23.0) <= 0.002
    assert abs(get(ioc, M + ".DRBV") - 23.0) <= 0.002
    assert get(ioc, M + ".MISS") == 0


def test_tweak_and_relative(ioc):
    base = get(ioc, M)
    put(ioc, M + ".TWF", 1)
    wait_dmov(ioc)
    assert get(ioc, M) == pytest.approx(base + 0.5)
    assert abs(get(ioc, M + ".RBV") - (base + 0.5)) <= 0.002
    put(ioc, M + ".TWR", 1)
    wait_dmov(ioc)
    put(ioc, M + ".RLV", -1.25)
    wait_dmov(ioc)
    assert abs(get(ioc, M + ".RBV") - (base - 1.25)) <= 0.002


def test_stop_syncs_val_to_rbv(ioc):
    start = get(ioc, M + ".RBV")
    put(ioc, M, start + 40)
    time.sleep(0.5)
    assert get(ioc, M + ".DMOV") == 0
    assert get(ioc, M + ".MOVN") in (0, 1)
    put(ioc, M + ".STOP", 1)
    wait_dmov(ioc)
    rbv = get(ioc, M + ".RBV")
    assert start < rbv < start + 40
    assert get(ioc, M) == pytest.approx(rbv, abs=0.002)


def test_soft_limits_and_lvio(ioc):
    rbv = get(ioc, M + ".RBV")
    put(ioc, M + ".HLM", rbv + 1)
    put(ioc, M + ".LLM", rbv - 1)
    assert get(ioc, M + ".DHLM") == pytest.approx(rbv + 1)
    old_val = get(ioc, M)
    put(ioc, M, rbv + 5)
    time.sleep(0.2)
    assert get(ioc, M + ".LVIO") == 1
    assert get(ioc, M) == pytest.approx(old_val)
    assert get(ioc, M + ".DMOV") == 1
    put(ioc, M, rbv + 0.5, wait=True)
    assert get(ioc, M + ".LVIO") == 0
    put(ioc, M + ".HLM", 0)
    put(ioc, M + ".LLM", 0)


def test_set_mode_variable_offset(ioc):
    # ophyd EpicsMotor.set_current_position: SET=Set, VAL=x, SET=Use
    drbv = get(ioc, M + ".DRBV")
    put(ioc, M + ".SET", 1)
    put(ioc, M, 100.0)
    put(ioc, M + ".SET", 0)
    assert get(ioc, M + ".RBV") == pytest.approx(100.0, abs=0.002)
    assert get(ioc, M + ".DRBV") == pytest.approx(drbv, abs=0.002)  # dial unchanged
    assert get(ioc, M + ".OFF") == pytest.approx(100.0 - drbv, abs=0.002)
    put(ioc, M, 101.0, wait=True)
    assert get(ioc, M + ".RBV") == pytest.approx(101.0, abs=0.002)
    put(ioc, M + ".OFF", 0.0)


def test_set_mode_frozen_redefines_dial(ioc):
    put(ioc, M + ".FOFF", 1)
    put(ioc, M + ".SET", 1)
    put(ioc, M, 50.0)
    put(ioc, M + ".SET", 0)
    put(ioc, M + ".FOFF", 0)
    time.sleep(0.2)
    assert get(ioc, M + ".DRBV") == pytest.approx(50.0, abs=0.002)
    assert get(ioc, M + ".RBV") == pytest.approx(50.0, abs=0.002)
    put(ioc, M, 49.0, wait=True)
    assert get(ioc, M + ".RBV") == pytest.approx(49.0, abs=0.002)


def test_direction_negative(ioc):
    put(ioc, M + ".DIR", 1)  # Neg
    drbv = get(ioc, M + ".DRBV")
    assert get(ioc, M + ".RBV") == pytest.approx(-drbv, abs=0.002)
    put(ioc, M, -drbv + 1.0, wait=True)  # user +1 => dial -1
    assert get(ioc, M + ".DRBV") == pytest.approx(drbv - 1.0, abs=0.002)
    put(ioc, M + ".DIR", 0)


def test_spmg_pause_and_go(ioc):
    rbv = get(ioc, M + ".RBV")
    put(ioc, M + ".SPMG", 1)  # Pause
    put(ioc, M, rbv + 1.0)
    time.sleep(0.3)
    assert get(ioc, M + ".DMOV") == 1
    assert get(ioc, M + ".RBV") == pytest.approx(rbv, abs=0.002)
    put(ioc, M + ".SPMG", 3)  # Go
    wait_dmov(ioc)
    assert get(ioc, M + ".RBV") == pytest.approx(rbv + 1.0, abs=0.002)


def test_jog(ioc):
    rbv = get(ioc, M + ".RBV")
    put(ioc, M + ".JVEL", 1.0)
    put(ioc, M + ".JOGF", 1)
    time.sleep(0.5)
    assert get(ioc, M + ".DMOV") == 0
    put(ioc, M + ".JOGF", 0)
    wait_dmov(ioc)
    new = get(ioc, M + ".RBV")
    assert new > rbv + 0.1
    assert get(ioc, M) == pytest.approx(new, abs=0.002)


def test_home_moves_to_dial_home(ioc):
    put(ioc, M + ".HOMF", 1)
    wait_dmov(ioc, 60)
    assert get(ioc, M + ".DRBV") == pytest.approx(0.0, abs=0.002)
    assert get(ioc, M + ".HOMF") == 0
    assert get(ioc, M + ".ATHM") == 1
    assert get(ioc, M + ".MSTA") & (1 << 14)  # HOMED


def test_extra_pvs(ioc):
    assert get(ioc, M + ":StepsPerDegFwd") > 0
    put(ioc, M + ":Calibrate", 1)
    wait_dmov(ioc, 60)
    assert "calibrated" in get_str(ioc, M + ":Status")
    assert get(ioc, M + ":Calibrate") == 0
