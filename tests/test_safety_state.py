"""The gate's checks that read the robot: step and error limits, battery, temperature."""

from __future__ import annotations

import numpy as np
import pytest

import hirobot.drivers  # noqa: F401 (registers built-in drivers)
from hirobot import (
    Action,
    BatteryState,
    ControlMode,
    RobotStatus,
    SafetyError,
    SafetyGate,
    get_body,
    load_embodiment,
    make_robot,
)
from hirobot.loading.config import load_config

N = 6  # ur_ur5e


def _robot(**gate):
    robot = get_body("toy")(load_embodiment("ur_ur5e"))
    robot._set_safety(SafetyGate(num_dofs=N, **gate))
    robot.connect()
    return robot


def _pos(*values):
    return Action(mode=ControlMode.POSITION, joint_target=np.array(values, dtype=float))


def test_step_limit_compares_with_the_last_accepted_target():
    robot = _robot(max_joint_step=0.1)
    robot.send_action(_pos(0.1, 0, 0, 0, 0, 0))  # first command: from the measured 0
    robot.send_action(_pos(0.2, 0, 0, 0, 0, 0))
    with pytest.raises(SafetyError, match=r"max_joint_step 0.100 rad from the last target; "
                                          r"largest: 'shoulder_lift' by \+0.300"):
        robot.send_action(_pos(0.2, 0.3, 0, 0, 0, 0))
    robot.send_action(_pos(0.3, 0.05, 0, 0, 0, 0))  # the refused one is not the reference
    assert np.allclose(robot.get_observation().joint_pos[:2], [0.3, 0.05])


def test_first_command_after_connect_reset_and_hold_is_measured_from_the_pose():
    robot = _robot(max_joint_step=0.1)
    with pytest.raises(SafetyError, match="max_joint_step"):
        robot.send_action(_pos(0.5, 0, 0, 0, 0, 0))
    for _ in range(5):
        robot.send_action(_pos(*([0.1 * (_ + 1)] + [0] * 5)))
    robot._joint_pos[0] = 0.9  # the arm was pushed; hold() keeps it there
    robot.hold()  # not step-checked: 0.4 rad from the last target
    robot.send_action(_pos(0.95, 0, 0, 0, 0, 0))
    robot.reset()  # toy: back to zero
    with pytest.raises(SafetyError, match="max_joint_step"):
        robot.send_action(_pos(0.9, 0, 0, 0, 0, 0))
    robot.send_action(_pos(0.05, 0, 0, 0, 0, 0))


def test_error_limit_catches_a_target_that_drifts_away_from_a_blocked_arm():
    robot = _robot(max_joint_step=0.1, max_joint_error=0.3)
    for i in range(1, 4):
        robot.send_action(_pos(0.1 * i, 0, 0, 0, 0, 0))
        robot._joint_pos[0] = 0.0  # blocked: the arm does not follow
    with pytest.raises(SafetyError, match="max_joint_error 0.300 rad from the measured position"):
        robot.send_action(_pos(0.4, 0, 0, 0, 0, 0))


def test_clamp_clips_steps_instead_of_refusing():
    robot = _robot(max_joint_step=0.1, clamp=True)
    robot.send_action(_pos(0.5, -0.5, 0.05, 0, 0, 0))
    assert np.allclose(robot.get_observation().joint_pos[:3], [0.1, -0.1, 0.05])


def test_velocity_commands_are_not_step_checked_and_leave_no_reference():
    robot = _robot(max_joint_step=0.1)
    robot.send_action(Action(mode=ControlMode.VELOCITY, joint_target=np.full(N, 5.0)))
    assert robot.get_observation().joint_pos[0] > 0.0
    measured = robot.get_observation().joint_pos.copy()
    robot.send_action(_pos(*(measured + 0.05)))  # from the measured pose, not from 0


def test_named_joints_beside_a_non_joint_mode_are_checked_on_their_own():
    robot = make_robot({**load_config("unitree_g1"),
                        "safety": {"max_joint_step": 0.1}})
    robot.direct_joints = ("left_elbow",)
    robot.connect()
    arm = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                 joint_names=["left_elbow"], joint_target=np.array([0.08]))
    robot.check_action(arm)
    with pytest.raises(SafetyError, match="left_elbow"):
        robot.check_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                                  joint_names=["left_elbow"], joint_target=np.array([0.5])))


class _Status:
    def __init__(self, robot):
        self.robot, self.status = robot, RobotStatus()

    def __call__(self):
        return self.status


def test_low_battery_and_hot_joints_refuse_commands_but_not_hold(monkeypatch):
    import hirobot.control.safety as safety

    monkeypatch.setattr(safety, "STATUS_PERIOD_S", 0.0)
    robot = _robot(min_battery_soc=20, max_joint_temperature=70)
    status = _Status(robot)
    robot.get_status = status
    robot.send_action(_pos(0, 0, 0, 0, 0, 0))  # no readings: nothing to refuse on
    status.status = RobotStatus(battery=BatteryState(soc=15))
    with pytest.raises(SafetyError, match="battery at 15 %, below min_battery_soc 20 %"):
        robot.send_action(_pos(0, 0, 0, 0, 0, 0))
    robot.hold()
    temps = np.full(N, 40.0)
    temps[2] = 75.0
    status.status = RobotStatus(battery=BatteryState(soc=80), joint_temperature=temps)
    with pytest.raises(SafetyError, match="joint 'elbow' at 75 degC"):
        robot.send_action(_pos(0, 0, 0, 0, 0, 0))
    robot.hold()


def test_status_is_read_at_most_once_a_second():
    robot = _robot(min_battery_soc=20)
    calls = []
    robot.get_status = lambda: calls.append(1) or RobotStatus()
    for _ in range(10):
        robot.send_action(_pos(0, 0, 0, 0, 0, 0))
    assert len(calls) == 1


def test_bad_values_are_rejected_at_build_time():
    with pytest.raises(ValueError, match="max_joint_step must be > 0"):
        SafetyGate(max_joint_step=0)
    with pytest.raises(ValueError, match="max_joint_stp"):
        make_robot({**load_config("ur_ur5e"), "safety": {"max_joint_stp": 0.1}})


def test_config_keys_reach_the_gate_on_both_gates_of_a_layered_stack():
    robot = make_robot({**load_config("ur_ur5e"), "safety": {"max_joint_step": 0.2,
                                                             "max_joint_error": 0.5}})
    assert robot.safety.max_joint_step == 0.2 and robot.safety.max_joint_error == 0.5
