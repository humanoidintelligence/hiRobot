"""Collision check between robot-group members (hirobot.collision, RobotGroup)."""

from __future__ import annotations

import os

import numpy as np
import pytest
import yaml

from hirobot import Action, ControlMode, RobotGroup, SafetyError, make_robot
from hirobot.collision import CollisionCheck, member_model, segment_distances
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.embodiment import Embodiment
from hirobot.kinematics import Kinematics

# A 3-joint arm standing on the floor: base -> j0 (yaw, z) -> 0.3 m up -> j1 (pitch, y)
# -> 0.3 m -> j2 (pitch, y) -> 0.2 m tip. Upright at zero, reaching along +x when
# j1 = pi/2.
ARM_XML = """
<mujoco>
  <worldbody>
    <body name="base">
      <body name="link0" pos="0 0 0.1">
        <joint name="j0" axis="0 0 1"/>
        <body name="link1" pos="0 0 0.3">
          <joint name="j1" axis="0 1 0"/>
          <body name="link2" pos="0 0 0.3">
            <joint name="j2" axis="0 1 0"/>
            <body name="tip" pos="0 0 0.2"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def _arm(name="arm"):
    return Embodiment(name=name, kind="arm", floating_base=False, control_hz=30,
                      joints=[{"name": f"j{i}"} for i in range(3)],
                      end_effectors=[{"name": "tip"}])


class Arm(ToyRobot):
    action_modes = (ControlMode.POSITION, ControlMode.PD, ControlMode.CARTESIAN)

    def __init__(self):
        super().__init__(_arm())
        self.sent = []

    def _send_action(self, action):
        self.sent.append(action)
        if action.mode != ControlMode.CARTESIAN:
            super()._send_action(action)


@pytest.fixture
def arm_xml(tmp_path):
    path = tmp_path / "arm.xml"
    path.write_text(ARM_XML)
    return str(path)


def _pair(arm_xml, gap_x=0.8, min_distance=0.05, radius=0.04):
    kin = Kinematics.from_mjcf(arm_xml)
    names = _arm().joint_names
    check = CollisionCheck({"left": member_model(kin, names, radius),
                            "right": member_model(kin, names, radius)}, min_distance)
    left, right = Arm(), Arm()
    # Right arm faces the left one: placed at +x, turned 180 degrees about z.
    group = RobotGroup({"left": left, "right": right},
                       base_poses={"left": [0, 0, 0, 0, 0, 0, 1],
                                   "right": [gap_x, 0, 0, 0, 0, 1, 0]},
                       collision=check)
    group.connect()
    return group, left, right


def _reach(left, right):
    # j1 = pi/2 tips the arm over towards +x of its own base.
    return Action(members={
        "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0, left, 0.0])),
        "right": Action(mode=ControlMode.POSITION, joint_target=np.array([0, right, 0.0])),
    })


# ---------------------------------------------------------------- geometry


def test_segment_distances_cases():
    p = np.array
    # Parallel, 1 m apart; crossing; point to segment; disjoint ends.
    a0 = p([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0.0]])
    a1 = p([[1, 0, 0], [1, 0, 0], [0, 0, 0], [1, 0, 0.0]])
    b0 = p([[0, 1, 0], [0.5, -1, 1], [0.5, 2, 0], [2, 0, 0.0]])
    b1 = p([[1, 1, 0], [0.5, 1, 1], [0.5, -2, 0], [3, 0, 0.0]])
    d = segment_distances(a0, a1, b0, b1)
    np.testing.assert_allclose(np.diag(d), [1.0, 1.0, 0.5, 1.0], atol=1e-9)


def test_segment_distances_match_brute_force():
    rng = np.random.default_rng(0)
    a0, a1, b0, b1 = (rng.normal(size=(20, 3)) for _ in range(4))
    d = segment_distances(a0, a1, b0, b1)
    s = np.linspace(0, 1, 201)
    for i in range(0, 20, 4):
        for j in range(0, 20, 5):
            pa = a0[i] + (a1[i] - a0[i]) * s[:, None]
            pb = b0[j] + (b1[j] - b0[j]) * s[:, None]
            brute = np.min(np.linalg.norm(pa[:, None] - pb[None], axis=-1))
            assert d[i, j] <= brute + 1e-9 and d[i, j] == pytest.approx(brute, abs=5e-3)


# ---------------------------------------------------------------- group


def test_far_apart_moves_pass(arm_xml):
    group, left, right = _pair(arm_xml)
    group.send_action(_reach(0.3, 0.3))
    assert len(left.sent) == 1 and len(right.sent) == 1


def test_arms_reaching_into_each_other_are_refused(arm_xml):
    group, left, right = _pair(arm_xml)
    # Both tipped fully forward: tips 0.8 m from each base -> they overlap in the middle.
    with pytest.raises(SafetyError, match="collision check: .* cm apart"):
        group.send_action(_reach(np.pi / 2, np.pi / 2))
    assert left.sent == [] and right.sent == []  # nothing reached either arm


def test_one_arm_alone_can_reach(arm_xml):
    group, left, _ = _pair(arm_xml)
    group.send_action(_reach(np.pi / 2, 0.0))
    assert len(left.sent) == 1


def test_flat_joint_target_is_checked_too(arm_xml):
    group, _, _ = _pair(arm_xml)
    with pytest.raises(SafetyError, match="collision check"):
        group.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.array([0, np.pi / 2, 0, 0, np.pi / 2, 0])))


def test_too_close_already_may_only_move_apart(arm_xml):
    # Bases 0.05 m apart: upright, the capsules (r = 0.04) already overlap.
    group, left, _ = _pair(arm_xml, gap_x=0.05)
    group.send_action(_reach(-0.5, -0.5))  # both lean away: the gap grows
    assert len(left.sent) == 1
    with pytest.raises(SafetyError, match="collision check"):
        group.send_action(_reach(0.3, 0.3))  # back towards each other


def test_cartesian_target_inside_the_other_arm_is_refused(arm_xml):
    group, left, _ = _pair(arm_xml)
    # Right arm's link at x = 0.8 (world) is upright from z = 0.1 to 0.9.
    inside = np.array([0.8, 0.0, 0.5, 0, 0, 0, 1.0])
    clear = np.array([0.3, 0.0, 0.5, 0, 0, 0, 1.0])
    with pytest.raises(SafetyError, match="left/target and right/"):
        group.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"left/tip": inside}))
    group.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"left/tip": clear}))
    assert left.sent[-1].ee_targets.keys() == {"tip"}


def test_member_without_a_model_is_not_checked(arm_xml):
    kin = Kinematics.from_mjcf(arm_xml)
    names = _arm().joint_names
    with pytest.raises(ValueError, match="at least two members"):
        CollisionCheck({"left": member_model(kin, names, 0.04)}, 0.05)
    check = CollisionCheck({"left": member_model(kin, names, 0.04),
                            "right": member_model(kin, names, 0.04)}, 0.05)
    with pytest.raises(ValueError, match="no member"):
        RobotGroup({"left": Arm(), "other": Arm()}, collision=check)
    # A third arm with no model sits on top of the left one; it is not checked.
    third = Arm()
    group = RobotGroup({"left": Arm(), "right": Arm(), "third": third},
                       base_poses={"right": [0.8, 0, 0, 0, 0, 1, 0]}, collision=check)
    group.connect()
    group.send_action(Action(members={
        "third": Action(mode=ControlMode.POSITION, joint_target=np.array([0, np.pi / 2, 0]))}))
    assert len(third.sent) == 1


def test_floating_member_needs_a_base_pose(arm_xml):
    floating = ToyRobot(Embodiment(name="h", control_hz=30, joints=[{"name": f"j{i}"} for i in range(3)]))
    kin = Kinematics.from_mjcf(arm_xml)
    names = [f"j{i}" for i in range(3)]
    check = CollisionCheck({"a": member_model(kin, names, 0.04), "h": member_model(kin, names, 0.04)}, 0.05)
    with pytest.raises(ValueError, match="need a base_pose"):
        RobotGroup({"a": Arm(), "h": floating}, collision=check)


def test_group_config_collision_section(tmp_path, arm_xml):
    for name in ("a", "b"):
        (tmp_path / f"{name}.yaml").write_text(yaml.safe_dump({
            "embodiment": _arm(name).model_dump(mode="json"), "body": {"driver": "toy"}}))
    (tmp_path / "models").mkdir()
    (tmp_path / "models" / "arm.xml").write_text(ARM_XML)
    cfg = {"group": {
        "members": {"left": {"config": "./a.yaml"},
                    "right": {"config": "./b.yaml", "base_pose": [0.8, 0, 0, 0, 0, 1, 0]}},
        "collision": {"min_distance": 0.05, "radius": 0.04,
                      "models": {"left": {"mjcf": "models/arm.xml"},
                                 "right": {"mjcf": "models/arm.xml", "skip": ["link0"]}}}}}
    path = tmp_path / "dual.yaml"
    path.write_text(yaml.safe_dump(cfg))
    group = make_robot(str(path))
    assert group.collision is not None and group.collision.min_distance == 0.05
    group.connect()
    with pytest.raises(SafetyError, match="collision check"):
        group.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.array([0, np.pi / 2, 0, 0, np.pi / 2, 0])))
    cfg["group"]["collision"]["models"]["left"]["oops"] = 1
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="unknown key"):
        make_robot(str(path))


# ---------------------------------------------------------------- real model

G1 = os.path.expanduser("~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")


@pytest.mark.skipif(not os.path.isfile(G1), reason="G1 MJCF not found")
def test_two_g1s_side_by_side(tmp_path):
    """Two fixed-base G1 models 0.6 m apart: arms at the sides pass, arms out towards each other do not."""
    from hirobot import load_embodiment

    emb = load_embodiment("unitree_g1")
    kin = Kinematics.from_mjcf(G1)
    model = member_model(kin, emb.joint_names, radius=0.05)
    check = CollisionCheck({"a": model, "b": model}, min_distance=0.03)
    seg = {"a": model.segments(np.zeros(emb.num_dofs), np.array([0, 0, 0.8, 0, 0, 0, 1.0])),
           "b": model.segments(np.zeros(emb.num_dofs), np.array([0, 0.6, 0.8, 0, 0, 0, 1.0]))}
    gap = check.gap(seg)
    assert gap.distance > check.min_distance
    q = np.zeros(emb.num_dofs)
    q[emb.joint_names.index("left_shoulder_roll")] = 1.5   # a's left arm out to +y
    q_b = np.zeros(emb.num_dofs)
    q_b[emb.joint_names.index("right_shoulder_roll")] = -1.5  # b's right arm out to -y
    seg = {"a": model.segments(q, np.array([0, 0, 0.8, 0, 0, 0, 1.0])),
           "b": model.segments(q_b, np.array([0, 0.6, 0.8, 0, 0, 0, 1.0]))}
    gap = check.gap(seg)
    assert gap.distance < check.min_distance
    assert {gap.members[0], gap.members[1]} == {"a", "b"}
