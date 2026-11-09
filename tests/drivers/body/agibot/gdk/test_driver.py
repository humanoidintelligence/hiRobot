"""AgibotG1Robot: reading, gripper scale, mode checks, group guard, chassis (fake cosine_bus)."""

import time

import numpy as np
import pytest

from hirobot import Action, ControlMode, RobotFaultError, get_body, load_embodiment
from tests.drivers.body.agibot.gdk import fake_sdk

EMB = load_embodiment("agibot_g1")


def _robot(monkeypatch, start_mode=1, **kw):
    gdk = fake_sdk.install(monkeypatch, start_mode=start_mode)
    robot = get_body("agibot_gdk")(EMB, **kw)
    robot.connect()
    return gdk, robot


def _idx(name):
    return EMB.joint_names.index(name)


def test_observation_maps_every_joint_and_gripper(monkeypatch):
    gdk, robot = _robot(monkeypatch)
    obs = robot.get_observation()
    assert obs.joint_pos[_idx("left_arm_joint1")] == pytest.approx(fake_sdk.ARM_POS[0])
    assert obs.joint_pos[_idx("right_arm_joint7")] == pytest.approx(fake_sdk.ARM_POS[13])
    assert obs.joint_pos[_idx("waist_lift_joint")] == pytest.approx(0.289)
    assert obs.joint_pos[_idx("head_pitch_joint")] == pytest.approx(0.436)
    # raw 120 = closed, 0 = open; hiRobot: 0 closed, 1 open
    assert obs.gripper == {"left_gripper": 0.0, "right_gripper": 1.0}
    assert set(obs.ee_poses) == {"left_gripper", "right_gripper"}
    assert obs.base_position is None  # no chassis_host
    robot.disconnect()


def test_command_carries_only_the_arm_groups_and_grippers(monkeypatch):
    gdk, robot = _robot(monkeypatch)
    q = robot.get_observation().joint_pos.copy()
    q[_idx("left_arm_joint2")] += 0.05
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q,
                             gripper={"left_gripper": 1.0, "right_gripper": 0.25}))
    msg = gdk.joint_commands()[-1]
    assert msg.control_group == 4 | 8 | 128 | 256
    assert msg.left_arm_joint_positions[1] == pytest.approx(fake_sdk.ARM_POS[1] + 0.05)
    assert msg.right_arm_joint_positions == pytest.approx(fake_sdk.ARM_POS[7:])
    assert msg.left_tool_joint_positions == [0.0]  # open -> GDK 0
    assert msg.right_tool_joint_positions == [0.75]
    assert msg.lifetime == pytest.approx(0.2)
    robot.disconnect()


def test_gripper_only_action(monkeypatch):
    gdk, robot = _robot(monkeypatch)
    robot.send_action(Action(mode=ControlMode.POSITION, gripper={"left_gripper": 0.0}))
    msg = gdk.joint_commands()[-1]
    assert msg.control_group == 128 and msg.left_tool_joint_positions == [1.0]
    robot.disconnect()


def test_target_for_an_uncommanded_joint_is_refused(monkeypatch):
    gdk, robot = _robot(monkeypatch)
    q = robot.get_observation().joint_pos.copy()
    q[_idx("waist_lift_joint")] += 0.1
    with pytest.raises(ValueError, match="waist_lift_joint"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    assert not gdk.joint_commands()
    robot.disconnect()


def test_waist_group_when_enabled(monkeypatch):
    gdk, robot = _robot(monkeypatch, groups=["left_arm", "waist_lift"])
    q = robot.get_observation().joint_pos.copy()
    q[_idx("waist_lift_joint")] = 0.3
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    msg = gdk.joint_commands()[-1]
    assert msg.control_group == 4 | 16 and msg.waist_lift_joint_position == pytest.approx(0.3)
    assert msg.right_arm_joint_positions == []
    robot.disconnect()


def test_stop_mode_refuses_until_reset(monkeypatch):
    gdk, robot = _robot(monkeypatch, start_mode=0)
    q = robot.get_observation().joint_pos
    with pytest.raises(RobotFaultError, match="STOP"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    robot.reset()
    modes = [m for t, m in gdk.published if t == "/wbc/set_control_mode"]
    assert len(modes) == 1 and modes[0].control_mode == 1 and modes[0].input_type == 54
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    assert len(gdk.joint_commands()) == 1
    robot.disconnect()


def test_reset_names_a_controller_that_stays_in_stop(monkeypatch):
    gdk, robot = _robot(monkeypatch, start_mode=0)
    gdk.servo_on_request = False
    with pytest.raises(RuntimeError, match="stayed in STOP"):
        robot.reset()
    robot.disconnect()


def test_estop_and_motion_error_refuse(monkeypatch):
    gdk, robot = _robot(monkeypatch)
    q = robot.get_observation().joint_pos
    gdk.estop = True
    gdk.publish_state()
    with pytest.raises(RobotFaultError, match="e-stop"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    gdk.estop, gdk.motion_error = False, 7
    gdk.publish_state()
    with pytest.raises(RobotFaultError, match="error 7"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    assert not gdk.joint_commands()
    robot.disconnect()


def test_stale_arm_state_refuses(monkeypatch):
    gdk, robot = _robot(monkeypatch, max_state_age=0.05)
    time.sleep(0.08)
    with pytest.raises(RobotFaultError, match="arm_joint_state"):
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=robot.get_observation().joint_pos))
    robot.disconnect()


def test_read_only_never_publishes(monkeypatch):
    gdk, robot = _robot(monkeypatch, read_only=True)
    robot.get_observation()
    with pytest.raises(RuntimeError, match="read_only"):
        robot.reset()
    with pytest.raises(RuntimeError, match="read_only"):
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=robot.get_observation().joint_pos))
    robot.disconnect()
    assert gdk.published == []


def test_connect_names_missing_topics(monkeypatch):
    gdk = fake_sdk.install(monkeypatch)
    real = gdk.messages
    gdk.messages = lambda: {k: v for k, v in real().items() if k != "/hal/waist_state"} | {
        "/hal/waist_state": None}
    robot = get_body("agibot_gdk")(EMB, connect_timeout=0.05)
    gdk_sub = gdk.subs
    del gdk_sub  # subscriber callbacks fire with None; the driver treats it as a message
    # Simulate the topic being silent: drop the callback result.
    robot._make_callback = lambda key: (lambda msg: None) if key == "waist" else \
        type(robot)._make_callback(robot, key)
    with pytest.raises(TimeoutError, match="/hal/waist_state"):
        robot.connect()


def test_status_reports_mode_and_hal_errors(monkeypatch):
    gdk, robot = _robot(monkeypatch)
    status = robot.get_status()
    assert status.extra["motion_mode"] == "SERVO"
    assert status.extra["hal.neck_error"] == 4294967295
    robot.disconnect()


# ------------------------------------------------------------------ chassis


def _with_chassis(monkeypatch, soft_manual=True, **kw):
    chassis = fake_sdk.Chassis(soft_manual=soft_manual)
    fake_sdk.install_chassis(monkeypatch, chassis)
    gdk, robot = _robot(monkeypatch, chassis_host="10.0.0.1", **kw)
    return gdk, robot, chassis


def test_chassis_pose_and_battery(monkeypatch):
    _, robot, chassis = _with_chassis(monkeypatch)
    assert ControlMode.BASE_VELOCITY in robot.action_modes
    obs = robot.get_observation()
    assert obs.base_position == pytest.approx([1.0, -0.5, 0.0])
    assert obs.base_orientation == pytest.approx([0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    assert robot.get_status().battery.soc == 100
    robot.disconnect()
    assert chassis.closed and chassis.moves == []  # never told to move, never sent anything


def test_base_velocity_is_resent_then_zeroed(monkeypatch):
    _, robot, chassis = _with_chassis(monkeypatch, command_timeout=0.25)
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.1, 0.0, 0.2], float)))
    time.sleep(0.6)
    sent = list(chassis.moves)
    assert sent[0] == pytest.approx((100.0, np.degrees(0.2)))
    assert len([m for m in sent if m[0] == 100.0]) >= 2  # resent at 10 Hz
    assert sent[-1] == (0.0, 0.0)  # stopped after command_timeout
    robot.disconnect()


def test_base_velocity_limits_and_mode(monkeypatch):
    _, robot, _ = _with_chassis(monkeypatch)
    for bad in ([0.5, 0, 0], [0, 0.1, 0], [0, 0, 2.0]):
        with pytest.raises(ValueError):
            robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array(bad, float)))
    robot.disconnect()
    _, robot, chassis = _with_chassis(monkeypatch, soft_manual=False)
    with pytest.raises(RobotFaultError, match="soft-manual"):
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.1, 0, 0], float)))
    robot.disconnect()
    assert chassis.moves == []


def test_no_chassis_no_base_velocity(monkeypatch):
    _, robot = _robot(monkeypatch)
    assert robot.action_modes == (ControlMode.POSITION,)
    robot.disconnect()
