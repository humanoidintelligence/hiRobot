"""RobotGroup with a MuJoCo G1 member: what real physics adds to the toy-sim tests.

``tests/test_group.py`` covers the group logic with toy sims. Here one member is
the MuJoCo G1 (unitree_g1_mujoco) and one is a toy arm, to check that physics time
advances for a member the action does not mention (``idle_step``), that the G1
holds its pose while only the arm moves, that a ``members`` action drives both,
and that a free-base G1's base state reaches the group observation.

Needs ``mujoco`` and the G1 MJCF; skipped otherwise.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from hirobot import Action, ControlMode, RobotGroup, make_robot
from hirobot.config import load_config
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.embodiment import Embodiment

CFG = "unitree_g1_mujoco"
MODEL = os.path.expanduser(load_config(CFG)["body"]["model_path"])

pytest.importorskip("mujoco")
pytestmark = pytest.mark.skipif(not os.path.isfile(MODEL), reason=f"G1 MJCF not found at {MODEL}")


class _Arm(ToyRobot):
    def __init__(self):
        super().__init__(Embodiment(name="toy-arm", kind="arm", floating_base=False, control_hz=50,
                                    joints=[{"name": f"j{i}"} for i in range(3)],
                                    end_effectors=[{"name": "tcp"}]))
        self.sent = 0

    def _send_action(self, action):
        self.sent += 1
        super()._send_action(action)


def _group(**g1_overrides):
    g1 = make_robot(CFG, body_overrides={"realtime": False, "viewer": False, **g1_overrides})
    arm = _Arm()
    group = RobotGroup({"g1": g1, "arm": arm})
    group.connect()
    group.reset()
    return group, g1, arm


def _arm_action(group, target):
    """An action for the arm only; the G1 is not mentioned."""
    return Action(members={"arm": Action(mode=ControlMode.POSITION,
                                         joint_target=np.asarray(target, dtype=float))})


def test_unmentioned_g1_steps_physics_and_holds_its_pose():
    group, g1, arm = _group()
    try:
        start_t = g1.data.time
        start_q = group.get_observation().joint_pos[group._ranges["g1"]].copy()
        steps = 50
        for _ in range(steps):
            group.send_action(_arm_action(group, [0.3, -0.2, 0.1]))
        # One control period of physics per group send, though the action never named the G1.
        assert g1.data.time - start_t == pytest.approx(steps * g1.control_period, rel=1e-6)
        obs = group.get_observation()
        np.testing.assert_allclose(obs.joint_pos[group._ranges["arm"]], [0.3, -0.2, 0.1])
        # Fixed base under the held PD command: the G1 stays where it was.
        assert np.max(np.abs(obs.joint_pos[group._ranges["g1"]] - start_q)) < 0.05
        assert arm.sent == steps
    finally:
        group.disconnect()


def test_members_action_drives_g1_and_arm_together():
    group, g1, arm = _group()
    try:
        emb = g1.embodiment
        j = emb.joint_names.index("left_elbow")
        q = group.get_observation().joint_pos[group._ranges["g1"]].copy()
        q[j] += 0.4
        action = Action(members={
            "g1": Action(mode=ControlMode.PD, joint_target=q),
            "arm": Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2, 0.3])),
        })
        for _ in range(100):  # 2 s at 50 Hz
            group.send_action(action)
        obs = group.get_observation()
        assert obs.joint_pos[group._ranges["g1"]][j] == pytest.approx(q[j], abs=0.05)
        np.testing.assert_allclose(obs.joint_pos[group._ranges["arm"]], [0.1, 0.2, 0.3])
    finally:
        group.disconnect()


def test_free_base_g1_base_state_is_the_group_base():
    # The only floating member keeps the group's top-level base fields.
    group, g1, _ = _group(fixed_base=False, base_height=0.8)
    try:
        assert group.embodiment.floating_base
        z0 = group.get_observation().base_position[2]
        for _ in range(25):
            group.send_action(_arm_action(group, [0.0, 0.0, 0.0]))
        obs = group.get_observation()
        assert obs.base_position is not None and obs.base_orientation is not None
        np.testing.assert_allclose(obs.base_position, g1.data.qpos[0:3])
        assert np.isclose(np.linalg.norm(obs.base_orientation), 1.0, atol=1e-6)
        # Not welded: gravity moved it.
        assert obs.base_position[2] != pytest.approx(z0, abs=1e-4)
    finally:
        group.disconnect()
