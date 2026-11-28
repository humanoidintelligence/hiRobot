"""``hirobot.testing.assert_forwards``: a wrapper passes the robot's declarations up."""

import pytest

from hirobot import Layer, Robot, make_robot
from hirobot.core.types import ControlMode
from hirobot.testing import FORWARDED, assert_forwards


def _inner():
    robot = make_robot("ur_ur5e")
    robot.connect()
    return robot


def test_a_plain_layer_forwards_everything():
    inner = _inner()
    assert_forwards(Layer(inner), inner)


def test_a_layer_over_an_assembly_forwards_everything():
    from tests.test_part_layers import _cfg

    inner = make_robot(_cfg([]))
    assert inner.accepts_parts
    assert_forwards(Layer(inner), inner)


class Narrow(Layer):
    action_modes = (ControlMode.POSITION,)


def test_a_changed_declaration_fails_unless_named():
    inner = _inner()
    with pytest.raises(AssertionError, match="action_modes"):
        assert_forwards(Narrow(inner), inner)
    assert_forwards(Narrow(inner), inner, changes=["action_modes"])
    with pytest.raises(AssertionError, match="not in"):
        assert_forwards(Narrow(inner), inner, changes=["nope"])


class HandWritten(Robot):
    """A wrapper that does not subclass Layer and forgets most of it."""

    def __init__(self, inner):
        self._r = inner

    @property
    def embodiment(self):
        return self._r.embodiment

    def connect(self): ...
    def disconnect(self): ...
    def reset(self, seed=None): ...
    def get_observation(self): return self._r.get_observation()
    def _send_action(self, action): ...
    def set_gains(self, kp, kd): ...


def test_a_hand_written_wrapper_that_misses_things_fails():
    inner = _inner()
    with pytest.raises(AssertionError) as err:
        assert_forwards(HandWritten(inner), inner)
    assert "simulated" in str(err.value) and "now()" in str(err.value)
    assert "embodiment" not in str(err.value)


def test_forwarded_covers_every_public_declaration_of_robot():
    import inspect

    declared = {n for n, v in vars(Robot).items() if not n.startswith("_")
                and not inspect.isfunction(v) and n not in ("safety", "mcp_config", "fault",
                                                            "layer_skill_running",
                                                            "robot_skill_running")}
    assert declared <= set(FORWARDED), sorted(declared - set(FORWARDED))
