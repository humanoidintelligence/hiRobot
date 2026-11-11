"""The safety gate's simulation check (hirobot.sim.sim_check)."""

from __future__ import annotations

import os
import time

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, RobotFaultError, load_embodiment, make_robot
from hirobot.loading.config import load_config
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.control.safety import SafetyError, SafetyGate
from hirobot.sim.sim_check import SimCheck
from tests.conftest import mjcf_of


class Recorder(ToyRobot):
    """A toy robot that records what reached the driver."""

    def __init__(self, emb):
        super().__init__(emb)
        self.sent = []

    def _send_action(self, action):
        self.sent.append(action)
        super()._send_action(action)


class FakeSim:
    """Stands in for the MuJoCo prediction: returns a reason, raises, or sleeps."""

    def __init__(self, reason=None, error=None, sleep=0.0):
        self.reason, self.error, self.sleep, self.calls = reason, error, sleep, 0

    def predict(self, action, obs, check):
        self.calls += 1
        if self.sleep:
            time.sleep(self.sleep)
        if self.error is not None:
            raise self.error
        return self.reason


def _robot(sim=None, **kw):
    emb = Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(3)], control_hz=50)
    robot = Recorder(emb)
    gate = SafetyGate.from_embodiment(emb)
    gate.sim_check = SimCheck(model="fake", **kw)
    gate.sim_check._sim = sim if sim is not None else FakeSim()
    robot._set_safety(gate)
    robot.connect()
    return robot, gate.sim_check


def _pos(v=0.1, n=3):
    return Action(mode=ControlMode.POSITION, joint_target=np.full(n, v))


def test_a_refused_prediction_never_reaches_the_driver():
    robot, check = _robot(FakeSim(reason="joint 'j1' would need 99 Nm"))
    with pytest.raises(SafetyError, match="simulation check: joint 'j1' would need 99 Nm"):
        robot.send_action(_pos())
    assert robot.sent == []
    assert check.stats()["rejected"] == 1 and robot.fault is None


def test_a_passing_prediction_sends_and_is_counted():
    robot, check = _robot()
    robot.send_action(_pos())
    assert len(robot.sent) == 1
    status = robot.get_status().safety["sim_check"]
    assert status["checked"] == 1 and status["rejected"] == 0 and status["p99_ms"] is not None


def test_a_failing_prediction_refuses_saying_the_check_failed():
    robot, check = _robot(FakeSim(error=RuntimeError("mj_step blew up")))
    with pytest.raises(SafetyError, match=r"simulation check failed \(not the command\): "
                                          "mj_step blew up"):
        robot.send_action(_pos())
    assert robot.sent == [] and check.errors == 1


def test_a_prediction_over_budget_refuses():
    robot, check = _robot(FakeSim(sleep=0.02), budget_s=0.005)
    with pytest.raises(SafetyError, match="prediction took .* budget 5.0 ms"):
        robot.send_action(_pos())
    assert robot.sent == []


def test_three_failures_in_a_row_fault_the_robot_and_a_success_resets_the_count():
    sim = FakeSim(error=RuntimeError("boom"))
    robot, check = _robot(sim)
    for _ in range(2):
        with pytest.raises(SafetyError):
            robot.send_action(_pos())
    sim.error = None
    robot.send_action(_pos())  # a check that ran to the end
    assert check.failures_in_row == 0 and robot.fault is None
    sim.error = RuntimeError("boom")
    for _ in range(3):
        with pytest.raises(SafetyError):
            robot.send_action(_pos())
    assert "sim_check failed 3 times in a row" in robot.fault
    with pytest.raises(RobotFaultError):
        robot.send_action(_pos())
    sim.error = None
    robot.reset()
    robot.send_action(_pos())


def test_refusals_of_the_command_are_not_failures():
    robot, check = _robot(FakeSim(reason="self-collision a / b"))
    for _ in range(5):
        with pytest.raises(SafetyError):
            robot.send_action(_pos())
    assert robot.fault is None and check.failures_in_row == 0


def test_on_error_warn_lets_the_command_through(caplog):
    robot, check = _robot(FakeSim(error=RuntimeError("boom")), on_error="warn")
    robot.send_action(_pos())
    assert len(robot.sent) == 1 and "on_error: warn" in caplog.text


def test_a_model_that_does_not_load_refuses_every_command():
    emb = Embodiment(name="toy", joints=[{"name": "j0"}], control_hz=50)
    robot = Recorder(emb)
    gate = SafetyGate.from_embodiment(emb)
    gate.sim_check = SimCheck(model="/nonexistent/model.xml")
    gate.sim_check.prepare(emb)  # logged, not raised
    robot._set_safety(gate)
    robot.connect()
    with pytest.raises(SafetyError, match="model did not load"):
        robot.send_action(_pos(n=1))
    assert robot.sent == []


def test_hold_and_commands_it_cannot_predict_are_not_checked():
    sim = FakeSim(reason="always no")
    robot, check = _robot(sim)
    robot.hold()  # stops where it is: never refused by a prediction
    assert len(robot.sent) == 1
    # No joint targets to predict (a base command, joints named beside it).
    check.check(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)), robot)
    check.check(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                       joint_target=np.array([0.1]), joint_names=["j0"]), robot)
    assert sim.calls == 0 and check.skipped == 3


def test_config_is_checked():
    with pytest.raises(ValueError, match="unknown key"):
        SimCheck.from_config({"horizon": 5})
    with pytest.raises(ValueError, match="on_error"):
        SimCheck(on_error="ignore")


def test_make_robot_refuses_sim_check_in_sim_a_body_without_a_model_and_model_key():
    with pytest.raises(ValueError, match="already is MuJoCo"):
        make_robot({**load_config("unitree_g1_mujoco"), "safety": {"sim_check": True}})
    with pytest.raises(ValueError, match=r"\['body'\] have no model"):
        make_robot({**load_config("ur_ur5e"), "safety": {"sim_check": True}})
    with pytest.raises(ValueError, match="model is gone"):
        make_robot({**load_config("unitree_g1"),
                    "safety": {"sim_check": {"model": "unitree_g1_mujoco"}}})


def test_a_hand_made_check_needs_an_mjcf_path_not_a_config_name():
    emb = Embodiment(name="toy", joints=[{"name": "j0"}], control_hz=50)
    check = SimCheck(model="unitree_g1_mujoco")
    check.prepare(emb)
    assert "give an MJCF .xml path" in str(check._load_error)


# ------------------------------------------------------- with the G1 MuJoCo model

def _g1_model():
    pytest.importorskip("mujoco")
    path = mjcf_of("unitree_g1_mujoco")
    if not os.path.isfile(path):
        pytest.skip("G1 MJCF not found (docs/setup/unitree.md)")


def _g1(**kw):
    from hirobot.loading.tree import load_robot

    _g1_model()
    emb = load_embodiment("unitree_g1").model_copy(update={"cameras": []})
    robot = Recorder(emb)
    gate = SafetyGate.from_embodiment(emb)
    # The tree of the welded G1 (its sim: section welds the pelvis at 1 m).
    gate.sim_check = SimCheck(model=load_robot("unitree_g1_mujoco"), **kw)
    gate.sim_check.prepare(emb)
    robot._set_safety(gate)
    robot.connect()
    return robot, emb.joint_names


def _g1_pose(names, **joints):
    q = np.zeros(len(names))
    for k, v in joints.items():
        q[names.index(k)] = v
    return Action(mode=ControlMode.POSITION, joint_target=q)


def test_g1_small_steps_pass_and_a_jump_needs_too_much_torque():
    robot, names = _g1()
    robot.send_action(_g1_pose(names))
    robot.send_action(_g1_pose(names, left_elbow=0.3))
    with pytest.raises(SafetyError, match="'left_elbow' would need .* Nm, limit 25.0 Nm"):
        robot.send_action(_g1_pose(names, left_elbow=3.0))
    assert len(robot.sent) == 2


def test_g1_arm_into_the_torso_is_a_self_collision():
    # torque_ratio high: only the contact can refuse it.
    robot, names = _g1(torque_ratio=1000, horizon_steps=25, budget_s=1.0)
    with pytest.raises(SafetyError, match="self-collision torso_link / left_shoulder_yaw_link"):
        robot.send_action(_g1_pose(names, left_shoulder_roll=-1.2))
    assert robot.sent == []
    robot.send_action(_g1_pose(names, left_shoulder_roll=0.3))


def test_g1_prediction_fits_in_half_a_control_period():
    robot, names = _g1()
    for k in range(20):
        robot.send_action(_g1_pose(names, left_elbow=0.01 * k))
    stats = robot.get_status().safety["sim_check"]
    assert stats["checked"] == 20 and stats["p99_ms"] < 10.0, stats


def test_make_robot_attaches_the_check_to_the_hardware_gate_only():
    _g1_model()
    config = {**load_config("unitree_g1"),
              "safety": {"sim_check": {"fixed_base": ["body"], "base_height": {"body": 1.0}}}}
    robot = make_robot(config)  # body: toy, standing in for the real G1
    check = robot.safety.sim_check
    assert check is not None and check._sim is not None  # loaded by make_robot
    robot.connect()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(29)))
    assert robot.get_status().safety["sim_check"]["checked"] == 1
