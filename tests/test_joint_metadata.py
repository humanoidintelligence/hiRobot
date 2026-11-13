"""Joint metadata: position_limit, torque_limit, axis, parent_link (checks and the G1's values)."""

import pytest
from pydantic import ValidationError

from hirobot import make_robot
from hirobot.core.embodiment import Joint
from hirobot.loading.tree import _joints


def test_joint_fields_default_to_none():
    j = Joint(name="a")
    assert (j.position_limit, j.torque_limit, j.axis, j.parent_link) == (None, None, None, None)


@pytest.mark.parametrize("bad", [
    {"position_limit": (1.0, -1.0)},
    {"torque_limit": 0.0},
    {"axis": (1.0, 1.0, 0.0)},
])
def test_joint_refuses_bad_values(bad):
    with pytest.raises(ValidationError):
        Joint(name="a", **bad)


def test_config_accepts_axis_and_parent_link_and_refuses_unknown_keys():
    out = _joints([{"name": "a", "axis": [0, 0, 1], "parent_link": "base"}], "body")
    assert Joint(**out[0]).axis == (0.0, 0.0, 1.0)
    with pytest.raises(ValueError, match="unknown key"):
        _joints([{"name": "a", "range": [0, 1]}], "body")


@pytest.mark.parametrize("config", ["unitree_g1", "unitree_g1_mujoco"])
def test_g1_joints_carry_limits_torque_axis_and_parent(config):
    joints = make_robot(config).embodiment.joints[:29]
    assert all(j.position_limit and j.torque_limit and j.axis and j.parent_link for j in joints)
    by = {j.name: j for j in joints}
    assert by["left_hip_roll"].axis == (1.0, 0.0, 0.0)
    assert by["left_hip_yaw"].axis == (0.0, 0.0, 1.0)
    assert by["left_hip_pitch"].parent_link == "pelvis"
    assert by["left_ankle_roll"].position_limit == (-0.2618, 0.2618)
    assert by["left_knee"].torque_limit == 139.0
