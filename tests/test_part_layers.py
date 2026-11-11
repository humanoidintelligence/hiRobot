"""Layers on one part of an assembly (``layers: [{type: ..., part: ...}]``)."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from hirobot import Action, ControlMode, Priority, RobotBusyError, make_robot
from hirobot.loading import registry
from hirobot.control.layer import Layer
from hirobot.assembly.part_layers import PartLayers, PartPort
from hirobot.core.robot import RobotFaultError
from hirobot.core.robot_skills import RobotSkillSpec
from hirobot.control.safety import SafetyError

_ARM = {"device": "body", "control_hz": 100, "joints": [{"name": f"j{i}"} for i in range(3)],
        "driver": {"type": "toy"}}


class Walker(Layer):
    """Passive: takes BASE_VELOCITY and turns it into joint targets (vx on every joint)."""

    action_modes = (ControlMode.BASE_VELOCITY, ControlMode.POSITION)

    def __init__(self, inner, gain: float = 1.0) -> None:
        super().__init__(inner)
        self.gain = gain
        self.received = []
        self.holds = 0
        self.resets = 0

    @property
    def direct_joints(self):  # type: ignore[override]
        return ()

    def _send_action(self, action):
        self.received.append(action)
        n = self.inner.embodiment.num_dofs
        if action.mode == ControlMode.BASE_VELOCITY:
            target = np.full(n, float(action.base_velocity[0]) * self.gain)
        else:
            target = np.asarray(action.joint_target, dtype=float)
        self.inner.send_action(Action(mode=ControlMode.POSITION, joint_target=target))

    def hold(self):
        self.holds += 1
        self.inner.hold()

    def reset(self, seed=None):
        self.resets += 1
        self.inner.reset(seed)

    def _layer_skills(self):
        return [RobotSkillSpec(name="wave", description="wave")]

    def _start_layer_skill(self, spec, args):
        self.inner.send_action(Action(mode=ControlMode.POSITION,
                                      joint_target=np.full(self.inner.embodiment.num_dofs, 0.3)))


class Streamer(Layer):
    """Has its own thread: sends its target to its part at its own rate."""

    action_modes = (ControlMode.POSITION,)

    def __init__(self, inner, hz: float = 200.0) -> None:
        super().__init__(inner)
        self.hz = hz
        self.target = None
        self.sent = 0
        self.error = None
        self._stop = threading.Event()
        self._thread = None

    def connect(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0)

    def _run(self):
        while not self._stop.is_set():
            target = self.target
            if target is not None and self._fault is None:
                try:
                    self.inner.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
                    self.sent += 1
                except Exception as exc:  # noqa: BLE001
                    self.error = exc
            time.sleep(1.0 / self.hz)

    def _send_action(self, action):
        self.target = np.asarray(action.joint_target, dtype=float).copy()

    def fail(self, reason="controller diverged"):
        self.target = None
        self._set_fault(reason)


@pytest.fixture(autouse=True)
def _layers(monkeypatch):
    monkeypatch.setitem(registry.LAYERS._entries, "walker", Walker)
    monkeypatch.setitem(registry.LAYERS._entries, "streamer", Streamer)


def _cfg(layers, **extra):
    arm = {"assembly": {"parts": {"body": _ARM}}}
    cfg = {"assembly": {"parts": {"left": arm, "right": arm,
                                  "cell": {"assembly": {"parts": {"a": arm, "b": arm}}}}},
           "layers": layers, "safety": {"joint_limit": 2.0}}
    cfg.update(extra)
    return cfg


def _robot(layers, **extra):
    robot = make_robot(_cfg(layers, **extra))
    robot.connect()
    robot.reset()
    return robot


def _q(robot, part):
    return robot.part(part).get_observation().joint_pos


def _layer(robot, part):
    stack = robot
    while not isinstance(stack, PartLayers):
        stack = stack._inner
    return stack.layers[part]


def test_the_layer_sees_only_its_part_in_its_own_names():
    robot = _robot([{"type": "walker", "part": "left"}])
    layer = _layer(robot, "left")
    assert isinstance(layer.inner, PartPort)
    assert layer.inner.embodiment.joint_names == ["j0", "j1", "j2"]
    assert layer.inner.get_observation().joint_pos.shape == (3,)
    # The root takes the layer's modes for its part (BASE_VELOCITY on left only: not common).
    assert ControlMode.BASE_VELOCITY not in robot.action_modes
    assert ControlMode.POSITION in robot.action_modes


def test_a_part_action_goes_to_the_layer_and_the_others_pass_through():
    robot = _robot([{"type": "walker", "part": "left", "gain": 2.0}])
    layer = _layer(robot, "left")
    robot.send_action(Action(parts={
        "left": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.25, 0, 0])),
        "right": Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.1))}))
    assert len(layer.received) == 1 and layer.received[0].mode == ControlMode.BASE_VELOCITY
    assert np.allclose(_q(robot, "left"), 0.5)
    assert np.allclose(_q(robot, "right"), 0.1)
    assert np.allclose(_q(robot, "cell/a"), 0.0)


def test_a_flat_action_is_split_and_the_layer_gets_its_piece():
    robot = _robot([{"type": "walker", "part": "cell/a"}])
    layer = _layer(robot, "cell/a")
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.arange(12) * 0.1))
    assert len(layer.received) == 1
    assert np.allclose(layer.received[0].joint_target, [0.6, 0.7, 0.8])  # its own names
    assert np.allclose(_q(robot, "cell/a"), [0.6, 0.7, 0.8])
    assert np.allclose(_q(robot, "left"), [0.0, 0.1, 0.2])


def test_the_gate_checks_the_layer_output_with_the_rest_and_nothing_goes_out():
    robot = _robot([{"type": "walker", "part": "left", "gain": 10.0}])
    with pytest.raises(SafetyError, match="exceed"):
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.5, 0, 0])),
            "right": Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.1))}))
    assert np.allclose(_q(robot, "left"), 0.0) and np.allclose(_q(robot, "right"), 0.0)


def test_a_layer_with_its_own_thread_sends_its_part_alone():
    robot = _robot([{"type": "streamer", "part": "right"}])
    layer = _layer(robot, "right")
    robot.send_action(Action(parts={
        "left": Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.2)),
        "right": Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.4))}))
    deadline = time.monotonic() + 2.0
    while layer.sent < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert layer.sent >= 3 and layer.error is None
    assert np.allclose(_q(robot, "right"), 0.4)
    assert np.allclose(_q(robot, "left"), 0.2)  # kept its target
    robot.disconnect()


def test_hold_goes_through_the_layer_for_its_part():
    robot = _robot([{"type": "walker", "part": "cell"}])
    layer = _layer(robot, "cell")
    robot.hold()
    assert layer.holds == 1
    robot.hold_parts(["left"]) if hasattr(robot, "hold_parts") else None
    assert layer.holds == 1


def test_an_estop_stops_the_tree_and_every_layer():
    robot = _robot([{"type": "streamer", "part": "left"}, {"type": "walker", "part": "right"}])
    left, right = _layer(robot, "left"), _layer(robot, "right")
    robot.estop()
    assert robot.fault and left.fault and right.fault
    with pytest.raises(RobotFaultError):
        robot.send_action(Action(parts={"right": Action(mode=ControlMode.POSITION,
                                                        joint_target=np.zeros(3))}))
    robot.reset()
    assert robot.fault is None and left.fault is None and right.fault is None
    robot.disconnect()


def test_a_layer_fault_stops_the_whole_tree_until_reset():
    robot = _robot([{"type": "streamer", "part": "left"}])
    layer = _layer(robot, "left")
    layer.fail()
    assert robot.fault and "left" in robot.fault
    with pytest.raises(RobotFaultError):
        robot.send_action(Action(parts={"right": Action(mode=ControlMode.POSITION,
                                                        joint_target=np.zeros(3))}))
    robot.reset()
    assert robot.fault is None
    robot.send_action(Action(parts={"right": Action(mode=ControlMode.POSITION,
                                                    joint_target=np.full(3, 0.1))}))
    assert np.allclose(_q(robot, "right"), 0.1)
    robot.disconnect()


def test_layer_skills_are_prefixed_and_claim_only_their_part():
    robot = _robot([{"type": "walker", "part": "left"}])
    assert [s.name for s in robot.list_layer_skills()] == ["left/wave"]
    robot.run_layer_skill("left/wave")
    assert np.allclose(_q(robot, "left"), 0.3)
    with pytest.raises(RobotBusyError):
        robot.send_action(Action(parts={"left": Action(mode=ControlMode.POSITION,
                                                       joint_target=np.zeros(3))}))
    # The other parts stay commandable.
    robot.send_action(Action(parts={"right": Action(mode=ControlMode.POSITION,
                                                    joint_target=np.full(3, 0.2))}))
    assert np.allclose(_q(robot, "right"), 0.2)
    robot.stop_layer_skill()
    lease = robot.claim("app", Priority.CALLER, parts=["left"])
    lease.send_action(Action(parts={"left": Action(mode=ControlMode.POSITION,
                                                   joint_target=np.zeros(3))}))
    lease.release()


def test_a_whole_robot_layer_sits_outside_the_part_layers():
    robot = _robot([{"type": "walker", "part": "left"}, {"type": "walker"}])
    assert isinstance(robot, Walker) and isinstance(robot._inner, PartLayers)
    assert robot.inner.embodiment.joint_names[:3] == ["left/j0", "left/j1", "left/j2"]


@pytest.mark.parametrize("layers, match", [
    ([{"type": "walker", "part": "left"}, {"type": "streamer", "part": "left"}], "same part"),
    ([{"type": "walker", "part": "cell"}, {"type": "walker", "part": "cell/a"}], "inside"),
    ([{"type": "walker", "part": "nope"}], "not a part"),
    ([{"type": "walker", "part": "left/body"}], "not a part"),
])
def test_bad_layer_parts_fail_at_load(layers, match):
    with pytest.raises(ValueError, match=match):
        make_robot(_cfg(layers))


def test_part_on_a_single_robot_is_refused():
    with pytest.raises(ValueError, match="built from parts"):
        make_robot({"assembly": {"parts": {"body": _ARM}}, "layers": [{"type": "walker",
                                                                       "part": "body"}]})
