"""The ``toy`` body as a mobile base (``mobile_base=True``): odometry from ``BASE_VELOCITY``.

Also the two assembly shapes it is meant for, built with ``RobotAssembly``
directly: a base carrying an arm (``mobile_ur5e``) and a humanoid beside a
fixed arm (``g1_ur5e_cell``). The rule for a moving part without odometry is in
``tests/test_assembly_parts.py`` (``test_floating_part_without_odometry_*``).
"""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import Action, ControlMode, RobotAssembly, SafetyError, make_robot
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.core.embodiment import Embodiment
from hirobot.geometry.transforms import pose_compose

HZ = 50
DT = 1.0 / HZ


def _base(joints=(), ees=()) -> Embodiment:
    return Embodiment(name="base", floating_base=True, control_hz=HZ,
                      joints=[{"name": j} for j in joints],
                      end_effectors=[{"name": e} for e in ees])


def _arm() -> Embodiment:
    return Embodiment(name="arm", kind="arm", floating_base=False, control_hz=HZ,
                      joints=[{"name": f"j{i}"} for i in range(3)],
                      end_effectors=[{"name": "tcp"}])


class _WithEe(ToyRobot):
    """A toy that reports fixed end-effector poses in its base frame."""

    def __init__(self, emb, ee_poses, **kw):
        super().__init__(emb, **kw)
        self.ee = {k: np.asarray(v, dtype=float) for k, v in ee_poses.items()}

    def get_observation(self):
        obs = super().get_observation()
        obs.ee_poses = {k: v.copy() for k, v in self.ee.items()}
        return obs


def _yaw(obs) -> float:
    qx, qy, qz, qw = obs.base_orientation
    assert qx == pytest.approx(0) and qy == pytest.approx(0)
    return float(2 * np.arctan2(qz, qw))


def _drive(robot, v, steps):
    for _ in range(steps):
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.asarray(v)))


# ------------------------------------------------------------- the toy itself


def test_mobile_base_with_no_joints():
    robot = ToyRobot(_base(), mobile_base=True)
    assert ControlMode.BASE_VELOCITY in robot.action_modes
    robot.connect()
    obs = robot.get_observation()
    assert obs.joint_pos.shape == (0,) and obs.joint_vel.shape == (0,)
    assert np.allclose(obs.base_position, 0) and np.allclose(obs.base_orientation, [0, 0, 0, 1])
    _drive(robot, [0.5, 0, 0], 10)
    assert robot._ticks == 10


def test_drive_forward_then_turn_matches_the_closed_form():
    robot = ToyRobot(_base(), mobile_base=True)
    robot.connect()
    _drive(robot, [0.5, 0, 0], 50)  # 1 s straight ahead
    obs = robot.get_observation()
    assert np.allclose(obs.base_position, [0.5, 0, 0]) and _yaw(obs) == pytest.approx(0)
    # A quarter circle of radius v / w = 1 m, counter-clockwise, in 1 s.
    v, w = np.pi / 2, np.pi / 2
    _drive(robot, [v, 0, w], 50)
    obs = robot.get_observation()
    assert np.allclose(obs.base_position, [1.5, 1.0, 0], atol=1e-9)
    assert _yaw(obs) == pytest.approx(np.pi / 2)
    assert np.allclose(obs.base_angular_velocity, [0, 0, w])
    assert np.allclose(obs.base_linear_velocity, [0, v, 0], atol=1e-9)  # facing +y now
    # Sideways (body +y) while facing +y moves along world -x.
    _drive(robot, [0, 0.2, 0], 50)
    assert np.allclose(robot.get_observation().base_position, [1.3, 1.0, 0], atol=1e-9)
    robot.reset()
    obs = robot.get_observation()
    assert np.allclose(obs.base_position, 0) and np.allclose(obs.base_orientation, [0, 0, 0, 1])
    assert np.allclose(obs.base_linear_velocity, 0)


def test_idle_steps_keep_following_until_the_watchdog_stops_the_base():
    robot = ToyRobot(_base(), mobile_base=True, command_timeout_s=0.1)
    robot.connect()
    _drive(robot, [1.0, 0, 0], 1)
    for _ in range(20):
        robot._idle_step()
    # 1 commanded step, then idle steps until 0.1 s without a new command.
    x = robot.get_observation().base_position[0]
    assert 0.1 <= x <= 0.14
    assert np.allclose(robot.get_observation().base_linear_velocity, 0)


def test_mobile_base_with_joints_keeps_the_joint_modes():
    robot = ToyRobot(_base(joints=["a", "b"]), mobile_base=True)
    robot.connect()
    _drive(robot, [1.0, 0, 0], 5)
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2])))
    obs = robot.get_observation()
    assert np.allclose(obs.joint_pos, [0.1, 0.2])
    assert np.allclose(obs.base_position, [0.1, 0, 0])
    assert np.allclose(obs.base_linear_velocity, 0)  # a joint command stops the base
    robot._idle_step()
    assert np.allclose(robot.get_observation().base_position, [0.1, 0, 0])


def test_without_the_flag_nothing_changes():
    robot = ToyRobot(_base(joints=["a"]))
    assert ControlMode.BASE_VELOCITY not in robot.action_modes
    robot.connect()
    obs = robot.get_observation()
    assert obs.base_position is None and np.allclose(obs.base_orientation, [0, 0, 0, 1])
    with pytest.raises(ValueError, match="floating_base"):
        ToyRobot(_arm(), mobile_base=True)


def test_mobile_base_from_a_config():
    robot = make_robot({"assembly": {"name": "base", "parts": {"body": {
        "device": "body", "floating_base": True, "control_hz": HZ, "joints": [],
        "driver": {"type": "toy", "mobile_base": True}}}}})
    robot.connect()
    _drive(robot, [0.5, 0, 0], 50)
    assert np.allclose(robot.get_observation().base_position, [0.5, 0, 0])
    with pytest.raises(SafetyError, match="non-finite"):
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY,
                                 base_velocity=np.array([np.nan, 0, 0])))


# ---------------------------------------------------------------- assemblies


def test_mobile_ur5e_arm_poses_stay_in_the_base_frame():
    # The node's frame is the base. node.py has no `on:` placement yet, so the
    # arm's mount on the base is written as a fixed `pose:` in the node; the
    # base has no placement (no frame conversion for it, and it has no end
    # effectors). Driving moves the base's odometry, which the node reports at
    # the top level, but not the arm's ee_poses.
    tcp = [0.4, 0.0, 0.3, 0, 0, 0, 1]
    base = ToyRobot(_base(), mobile_base=True)
    arm = _WithEe(_arm(), {"tcp": tcp})
    mount = [0.1, 0.0, 0.5, 0, 0, 0, 1]
    robot = RobotAssembly(None, parts={"base": base, "arm": arm},
                          placement={"arm": {"pose": mount}})
    assert robot.embodiment.floating_base and robot.embodiment.num_dofs == 3
    robot.connect()
    before = robot.get_observation().ee_poses["arm/tcp"]
    assert np.allclose(before, pose_compose(mount, tcp))
    _drive(robot, [0.5, 0, np.pi / 2], 50)  # flat action: base_velocity goes to the base
    robot.send_action(Action(parts={"base": Action(mode=ControlMode.BASE_VELOCITY,
                                                   base_velocity=np.array([0.2, 0, 0]))}))
    obs = robot.get_observation()
    assert np.linalg.norm(obs.base_position) > 0.3 and _yaw(obs) == pytest.approx(np.pi / 2)
    assert np.allclose(obs.ee_poses["arm/tcp"], before)
    robot.reset()
    assert np.allclose(robot.get_observation().base_position, 0)


def test_g1_ur5e_cell_humanoid_poses_follow_origin_and_odometry():
    hand = [0.3, 0.2, 0.0, 0, 0, 0, 1]
    g1 = _WithEe(_base(joints=["hip", "knee"], ees=["left_hand"]), {"left_hand": hand},
                 mobile_base=True)
    tcp = [0.4, 0.0, 0.3, 0, 0, 0, 1]
    arm = _WithEe(_arm(), {"tcp": tcp})
    origin = [2.0, 1.0, 0.0, 0, 0, 0, 1]
    arm_pose = [0.0, -1.0, 0.8, 0, 0, 0, 1]
    robot = RobotAssembly(None, parts={"g1": g1, "arm": arm},
                          placement={"g1": {"origin": origin}, "arm": {"pose": arm_pose}})
    robot.connect()
    obs = robot.get_observation()
    assert np.allclose(obs.ee_poses["g1/left_hand"], pose_compose(origin, hand))
    _drive(robot, [0.5, 0, 0], 50)                     # 0.5 m ahead
    _drive(robot, [np.pi / 4, 0, np.pi / 2], 50)        # quarter circle of radius 0.5 m
    obs = robot.get_observation()
    odom = np.concatenate([obs.base_position, obs.base_orientation])
    assert np.allclose(odom[:3], [1.0, 0.5, 0], atol=1e-9)
    assert np.allclose(obs.ee_poses["g1/left_hand"], pose_compose(origin, pose_compose(odom, hand)))
    # [2, 1] + [1, 0.5] + the hand (0.3 ahead, 0.2 left) turned to face +y
    assert np.allclose(obs.ee_poses["g1/left_hand"][:3], [2.8, 1.8, 0], atol=1e-9)
    assert np.allclose(obs.ee_poses["arm/tcp"], pose_compose(arm_pose, tcp))
    # Joints of the humanoid still take joint modes beside its base.
    robot.send_action(Action(parts={"g1": Action(mode=ControlMode.POSITION,
                                                 joint_target=np.array([0.1, 0.2]))}))
    assert np.allclose(robot.get_observation().joint_pos[:2], [0.1, 0.2])


def test_a_floating_toy_without_the_flag_has_no_odometry():
    # The toy without mobile_base reports no base_position: its end effectors
    # leave the node frame (the rule itself is tested in test_assembly_parts.py).
    g1 = _WithEe(_base(joints=["hip"], ees=["left_hand"]), {"left_hand": [0.3, 0, 0, 0, 0, 0, 1]})
    robot = RobotAssembly(None, parts={"g1": g1, "arm": _WithEe(_arm(), {"tcp": [0] * 6 + [1]})})
    robot.connect()
    obs = robot.get_observation()
    assert "g1/left_hand" not in obs.ee_poses and "g1/ee_poses" in obs.extra
