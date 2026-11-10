import pytest

from newfocus_rotary.picomotor import Picomotor8742, PicomotorError


class FakeLink:
    def __init__(self, replies, prefix=""):
        self.replies = replies
        self.prefix = prefix
        self.written = []
        self._pending = []

    def write(self, data):
        cmd = data.decode()
        assert cmd.endswith("\r")
        cmd = cmd[:-1]
        self.written.append(cmd)
        if cmd.endswith("?"):
            body = cmd[len(self.prefix):]
            self._pending.append(self.prefix + self.replies[body] + "\r\n")

    def readline(self, timeout=None):
        return self._pending.pop(0).encode()

    def flush_input(self):
        pass

    def close(self):
        pass


def test_command_formatting():
    link = FakeLink({"*IDN?": "New_Focus 8742 v2.2 08/01/13 13991", "1TP?": "-1234", "1MD?": "1", "TB?": "0, NO ERROR DETECTED"})
    pm = Picomotor8742(link)
    pm.move_relative(1, 500)
    pm.move_relative(1, -20)
    pm.set_velocity(1, 5000)  # clamped to 2000
    pm.set_acceleration(1, 1e9)  # clamped
    pm.move_indefinite(1, -1)
    pm.stop(1)
    assert pm.position(1) == -1234
    assert pm.motion_done(1)
    assert pm.error() == (0, "NO ERROR DETECTED")
    assert link.written[1:7] == ["1PR+500", "1PR-20", "1VA2000", "1AC200000", "1MV-", "1ST"]


def test_daisy_chain_address_prefix():
    link = FakeLink({"*IDN?": "New_Focus 8742 v2.2", "2TP?": "42"}, prefix="3>")
    pm = Picomotor8742(link, address=3)
    assert pm.position(2) == 42
    pm.move_relative(2, 7)
    assert link.written[-1] == "3>2PR+7"


def test_wrong_device():
    with pytest.raises(PicomotorError):
        Picomotor8742(FakeLink({"*IDN?": "Something else"}))
