import random

import pytest

from newfocus_rotary.e201 import (
    E201_9B,
    BissFrameFormat,
    EncoderError,
    biss_crc,
    decode_frame,
    encode_frame,
    guess_data_bits,
)


def rls_crc_check(data: int, data_bits: int, polynomial: int, received_crc: int) -> bool:
    """Reference CRC check copied from RLS's e201_test_platform (returns True on ERROR)."""
    crc_width = polynomial.bit_length() - 1
    reg = (data << crc_width) | received_crc
    total_bits = data_bits + crc_width
    poly = polynomial << (total_bits - crc_width - 1)
    for i in range(data_bits):
        if reg & (1 << (total_bits - 1 - i)):
            reg ^= poly >> i
    return (reg & ((1 << crc_width) - 1)) != 0


def rls_parse(data: bytes, st_bits: int, mt_bits: int, status_bits=2, crc_bits=6):
    """Reference BiSS parse copied from RLS's letp_parsers/biss_parser.py."""
    dut_bytes = len(data)
    dut_frame = int.from_bytes(data, "big")
    frame_bits = dut_bytes * 8
    dut_res = st_bits + mt_bits
    crc_data_bits = dut_res + status_bits
    received_crc = (dut_frame >> (frame_bits - dut_res - status_bits - crc_bits)) & ((1 << crc_bits) - 1)
    received_crc ^= (1 << crc_bits) - 1
    crc_frame = (dut_frame >> (frame_bits - dut_res - status_bits)) & ((1 << crc_data_bits) - 1)
    crc_error = rls_crc_check(crc_frame, crc_data_bits, 0x43, received_crc)
    status = (dut_frame >> (frame_bits - dut_res - status_bits)) & ((1 << status_bits) - 1)
    position = (dut_frame >> (frame_bits - dut_res)) & ((1 << dut_res) - 1)
    return position & ((1 << st_bits) - 1), position >> st_bits, status, crc_error


def test_crc_matches_rls_reference():
    rng = random.Random(0)
    for _ in range(500):
        n = rng.randint(8, 40)
        data = rng.getrandbits(n)
        crc = biss_crc(data, n)
        assert not rls_crc_check(data, n, 0x43, crc)
        assert rls_crc_check(data, n, 0x43, crc ^ 1)


@pytest.mark.parametrize("st,mt", [(18, 0), (19, 0), (20, 0), (20, 16), (17, 12)])
def test_encode_decode_roundtrip_and_reference_parser(st, mt):
    fmt = BissFrameFormat(singleturn_bits=st, multiturn_bits=mt)
    rng = random.Random(st * 100 + mt)
    for _ in range(200):
        pos = rng.getrandbits(st)
        turns = rng.getrandbits(mt) if mt else 0
        raw = encode_frame(pos, fmt, multiturn=turns)
        r = decode_frame(raw, fmt)
        assert (r.singleturn, r.multiturn, r.crc_ok, r.error, r.warning) == (pos, turns, True, False, False)
        ref_st, ref_mt, ref_status, ref_crc_err = rls_parse(raw, st, mt)
        assert (ref_st, ref_mt, ref_status, ref_crc_err) == (pos, turns, 0b11, False)


def test_status_bits_active_low():
    fmt = BissFrameFormat()
    r = decode_frame(encode_frame(1234, fmt, error=True), fmt)
    assert r.error and not r.warning and r.crc_ok
    r = decode_frame(encode_frame(1234, fmt, warning=True), fmt)
    assert r.warning and not r.error


def test_corrupted_frame_fails_crc():
    fmt = BissFrameFormat()
    raw = bytearray(encode_frame(0x5A5A5, fmt))
    raw[1] ^= 0x10
    assert not decode_frame(bytes(raw), fmt).crc_ok


def test_guess_data_bits_finds_frame_length():
    fmt = BissFrameFormat(singleturn_bits=20)
    rng = random.Random(3)
    frames = [encode_frame(rng.getrandbits(20), fmt) for _ in range(20)]
    assert 22 in guess_data_bits(frames)


class ScriptedSerial:
    def __init__(self, replies):
        self.replies = replies
        self.written = []

    def write(self, data):
        self.written.append(data)
        self._next = self.replies[data.decode()]

    def readline(self, timeout=None):
        r = self._next
        return (r() if callable(r) else r).encode() + b"\r"

    def flush_input(self):
        pass

    def close(self):
        pass


def test_e201_driver_protocol():
    fmt = BissFrameFormat(singleturn_bits=20)
    frame = encode_frame(0x40000, fmt)  # quarter turn
    t = ScriptedSerial({"v": "E201-9B V1.2", "N": "Power ON", "4": frame.hex().upper(), "M09": "1000 kHz"})
    enc = E201_9B(t, fmt, clock_khz=1000)
    r = enc.read()
    assert r.singleturn == 0x40000 and r.crc_ok
    assert [w.decode() for w in t.written] == ["v", "N", "M09", "4"]


def test_e201_rejects_other_device():
    t = ScriptedSerial({"v": "P911 something"})
    with pytest.raises(EncoderError):
        E201_9B(t)


def test_e201_crc_retry():
    fmt = BissFrameFormat()
    good = encode_frame(777, fmt).hex()
    bad = bytearray(encode_frame(777, fmt))
    bad[0] ^= 0x80
    seq = iter([bad.hex(), good])
    t = ScriptedSerial({"v": "E201-9B", "N": "ON", "4": lambda: next(seq)})
    enc = E201_9B(t, fmt)
    r = enc.read()
    assert r.crc_ok and r.singleturn == 777


def test_aksim2_multiturn_reset_sequence():
    t = ScriptedSerial({"v": "E201-9B", "N": "ON"})
    enc = E201_9B(t, BissFrameFormat(singleturn_bits=18, multiturn_bits=16))
    t.replies = {}

    class AnyOK(dict):
        def __getitem__(self, k):
            return "0"

    t.replies = AnyOK()
    enc.reset_multiturn(0)
    cmds = [w.decode() for w in t.written[2:]]
    assert cmds == [
        "Ws003:064",  # select bank 3
        "Ws000:020", "Ws000:021", "Ws000:022", "Ws000:023",  # multiturn = 0 at 0x14..0x17
        "Ws205:072",  # key 0xCD -> 0x48
        "Ws109:073",  # 0x6D ('m', apply multiturn) -> 0x49
    ]


def test_aksim2_detailed_status_read():
    t = ScriptedSerial({"v": "E201-9B", "N": "ON", "Ws000:064": "0", "R02:074": "0:2:8004"})
    enc = E201_9B(t)
    assert enc.read_detailed_status() == 0x8004


def test_register_error_status():
    t = ScriptedSerial({"v": "E201-9B", "N": "ON", "Ws000:064": "4"})
    enc = E201_9B(t)
    with pytest.raises(EncoderError, match="Timeout"):
        enc.read_detailed_status()
