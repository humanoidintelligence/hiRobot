"""Commands for robots that move themselves, and the ``layers:`` extension point.

The layer here is a stand-in written for the test (hiRobot ships none): it takes
``BASE_VELOCITY`` / ``MOTION_CLIP`` / ``CARTESIAN`` from the caller and sends
the joints it drives as PD targets to the robot underneath, passing the joints
it leaves free (``direct_joints``) through from ``Action.joint_names``.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from hirobot import (
    Action,
    ActionChunk,
    ControlMode,
    Layer,
    MotionClip,
    Robot,
    RobotAssembly,
    RobotFaultError,
    SafetyError,
    SafetyGate,
    make_robot,
)
from hirobot.loading import registry
from hirobot.loading.config import load_config
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.core.embodiment import Embodiment
from hirobot.control.layer import ESTOP_LAYER_TIMEOUT_S
from hirobot.testing import assert_layer_uses_public_api
from tests.conftest import toy_config


def _emb(n: int) -> Embodiment:
    return Embodiment(name="toy", joints=[{"name": f"joint_{i}"} for i in range(n)],
                      end_effectors=[{"name": "left_hand"}], control_hz=50)


# ---------------------------------------------------------------- MotionClip


def test_joint_clip_checks_shapes_and_reports_duration():
    clip = MotionClip(fps=50, joint_names=["a", "b"], joint_pos=np.zeros((25, 2)))
    assert clip.num_frames == 25 and clip.duration_s == 0.5
    with pytest.raises(ValueError, match=r"shape \(T, 2\)"):
        MotionClip(fps=50, joint_names=["a", "b"], joint_pos=np.zeros((25, 3)))
    with pytest.raises(ValueError, match="same number of frames"):
        MotionClip(fps=50, joint_names=["a"], joint_pos=np.zeros((5, 1)), root_quat=np.zeros((4, 4)))
    with pytest.raises(ValueError, match="needs joint_names"):
        MotionClip(fps=50, joint_pos=np.zeros((5, 1)))
    with pytest.raises(ValueError, match="fps"):
        MotionClip(fps=0, joint_names=["a"], joint_pos=np.zeros((5, 1)))


def test_smpl_clip():
    clip = MotionClip(fps=30, space="smpl", smpl_joints=np.zeros((10, 24, 3)),
                      root_quat=np.tile([0, 0, 0, 1.0], (10, 1)))
    assert clip.num_frames == 10
    with pytest.raises(ValueError, match="space must be"):
        MotionClip(fps=30, space="bvh", smpl_joints=np.zeros((10, 24, 3)))


# ------------------------------------------------------------------- Action


def test_action_checks_the_new_modes():
    with pytest.raises(ValueError, match="MOTION_CLIP needs motion"):
        Action(mode=ControlMode.MOTION_CLIP)
    with pytest.raises(ValueError, match="BASE_POSE needs base_target"):
        Action(mode=ControlMode.BASE_POSE)
    with pytest.raises(ValueError, match="non-joint modes"):
        Action(mode=ControlMode.PD, joint_target=np.zeros(1), joint_names=["a"])
    a = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
               joint_names=["f1"], joint_target=np.zeros(1))
    assert a.joint_names == ["f1"]


def test_chunk_carries_base_target_and_joint_names():
    chunk = ActionChunk(mode=ControlMode.BASE_POSE, base_target=np.arange(6.0).reshape(2, 3),
                        joint_names=["f1"], joint_targets=np.ones((2, 1)))
    a = chunk.step(1)
    assert np.array_equal(a.base_target, [3, 4, 5]) and a.joint_names == ["f1"]


def test_gate_checks_named_joints_and_new_fields():
    emb = _emb(3)
    gate = SafetyGate.from_embodiment(emb, joint_limit=1.0)
    ok = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                joint_names=["joint_2"], joint_target=np.array([0.5]))
    assert gate.check(ok, emb).allowed
    unknown = ok.model_copy(update={"joint_names": ["wrist"]})
    assert "unknown joint" in gate.check(unknown, emb).reason
    short = ok.model_copy(update={"joint_target": np.zeros(2)})
    assert "for 1 joint_names" in gate.check(short, emb).reason
    over = ok.model_copy(update={"joint_target": np.array([2.0])})
    assert "exceed" in gate.check(over, emb).reason
    nan_vel = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([np.nan, 0, 0]))
    assert "base_velocity" in gate.check(nan_vel, emb).reason
    nan_ee = Action(mode=ControlMode.CARTESIAN, ee_targets={"tcp": np.full(7, np.inf)})
    assert "ee_targets" in gate.check(nan_ee, emb).reason
    pos = np.zeros((4, 1))
    pos[2, 0] = np.nan
    clip = MotionClip(fps=50, joint_names=["joint_0"], joint_pos=pos)
    nan_clip = Action(mode=ControlMode.MOTION_CLIP, motion=clip)
    assert "motion.joint_pos" in gate.check(nan_clip, emb).reason


def test_capability_defaults():
    robot = ToyRobot(_emb(3))
    assert robot.motion_spaces == () and robot.direct_joints == ()
    assert robot.cartesian_targets == ()  # ToyRobot takes no CARTESIAN


# ------------------------------------------------------------- a fake layer


class FakeWbc(Layer):
    """Drives ``driven`` joints itself; the others are left to the caller."""

    action_modes = (ControlMode.BASE_VELOCITY, ControlMode.MOTION_CLIP, ControlMode.CARTESIAN)
    motion_spaces = ("joint",)

    def __init__(self, inner: Robot, driven=None, gain: float = 40.0) -> None:
        super().__init__(inner)
        names = inner.embodiment.joint_names
        self.driven = list(driven) if driven is not None else list(names)
        self.gain = gain
        self.received = []

    @property
    def direct_joints(self):  # type: ignore[override]
        return tuple(n for n in self.inner.embodiment.joint_names if n not in self.driven)

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return ("left_hand",)

    def _send_action(self, action):
        self.received.append(action)
        names = self.inner.embodiment.joint_names
        obs = self.inner.get_observation()
        target = np.nan_to_num(obs.joint_pos.copy())
        free = dict(zip(action.joint_names or [], np.asarray(action.joint_target if
                                                              action.joint_names else [])))
        for i, n in enumerate(names):
            if n in free:
                target[i] = free[n]
        self.inner.send_action(Action(mode=ControlMode.PD, joint_target=target,
                                      kp=np.full(len(names), self.gain)))


@pytest.fixture
def fake_layer(monkeypatch):
    monkeypatch.setitem(registry.LAYERS._entries, "fake_wbc", FakeWbc)
    return FakeWbc


def _cfg(n=4, **extra):
    cfg = toy_config(_emb(n))
    cfg.update(extra)
    return cfg


def test_make_robot_stacks_layers_with_a_gate_under_and_over(fake_layer):
    robot = make_robot(_cfg(layers=[{"type": "fake_wbc", "gain": 12.0}]))
    assert isinstance(robot, FakeWbc) and isinstance(robot.inner, ToyRobot)
    assert robot.gain == 12.0
    assert robot.safety is not None and robot.inner.safety is not None
    robot.connect()
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)))
    assert robot.received[-1].mode == ControlMode.BASE_VELOCITY
    with pytest.raises(SafetyError):  # the gate on top sees what the caller sends
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY,
                                 base_velocity=np.array([np.inf, 0, 0])))


def test_layer_joint_commands_clear_the_gate_underneath(fake_layer):
    layer = {"type": "fake_wbc", "driven": ["joint_1", "joint_2", "joint_3"]}
    robot = make_robot(_cfg(layers=[layer], safety={"joint_limit": 0.1}))
    robot.connect()
    free = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                  joint_names=["joint_0"], joint_target=np.array([0.05]))
    robot.send_action(free)  # within 0.1 rad on both gates
    # A tighter gate underneath refuses the PD command the layer sends, even
    # though the caller's action cleared the gate on top.
    robot._inner._set_safety(SafetyGate(num_dofs=4, joint_limit=0.01))
    with pytest.raises(SafetyError, match="exceed"):
        robot.send_action(free)


def test_unknown_layer_and_bad_spec(fake_layer):
    with pytest.raises(KeyError, match="plugin package"):
        make_robot(_cfg(layers=[{"type": "nope"}]))
    with pytest.raises(ValueError, match="needs a 'type'"):
        make_robot(_cfg(layers=[{"gain": 1}]))
    with pytest.raises(ValueError, match="must be a list"):
        make_robot(_cfg(layers={"type": "fake_wbc"}))


def test_layer_wraps_body_and_hands_so_it_sees_every_joint(fake_layer, monkeypatch):
    from tests.test_end_effectors import CFG, _hands

    _hands(monkeypatch)  # installs the fake Dex3 SDK
    robot = make_robot(CFG, overrides={"body": {"driver": {"type": "unitree_sdk2", "network_interface": "eth0"}}})
    assert isinstance(robot, RobotAssembly)  # no layers: unchanged
    cfg = load_config(CFG)
    cfg["layers"] = [{"type": "fake_wbc"}]
    robot = make_robot(cfg, overrides={"body": {"driver": {"type": "unitree_sdk2", "network_interface": "eth0"}}})
    # One assembly under the layer joins the hands, cameras and audio.
    layer = robot
    assert isinstance(layer, FakeWbc)
    assert isinstance(layer.inner, RobotAssembly) and set(layer._inner.end_effectors) == {
        "left_hand", "right_hand"}
    assert layer.cameras is layer.inner.cameras
    assert layer.embodiment.num_dofs == 43  # the layer can drive the fingers too
    assert robot.action_modes == FakeWbc.action_modes
    assert robot.motion_spaces == ("joint",) and robot.cartesian_targets == ("left_hand",)


def test_layer_drives_the_body_and_the_caller_drives_the_fingers(monkeypatch):
    from hirobot.devices.end_effector import body_embodiment
    from tests.test_end_effectors import LEFT_Q, _hands

    emb, bus, hands = _hands(monkeypatch)
    inner = RobotAssembly(ToyRobot(body_embodiment(emb)), end_effectors=hands, embodiment=emb)
    body = body_embodiment(emb).joint_names
    robot = FakeWbc(inner, driven=body)
    robot._set_safety(SafetyGate.from_embodiment(emb))
    left, right = emb.end_effectors[0].joints, emb.end_effectors[1].joints
    assert set(robot.direct_joints) == set(left) | set(right)
    robot.connect()
    q = np.array(LEFT_Q)  # within the Dex3 limits, which the hand clips to
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                             joint_names=left, joint_target=q))
    cmd = bus.sent["rt/dex3/left/cmd"][-1]
    assert np.allclose([m[1] for m in cmd], q)
    # A body joint belongs to the controller: the caller may not command it.
    with pytest.raises(SafetyError, match="not free to command"):
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                                 joint_names=["left_elbow"], joint_target=np.zeros(1)))


# ------------------------------------------- hands commanded beside a controller


class _SelfBalancing(ToyRobot):
    """A body that balances itself: takes BASE_VELOCITY plus PD for its arms."""

    action_modes = (ControlMode.PD, ControlMode.BASE_VELOCITY)

    def __init__(self, emb):
        super().__init__(emb)
        self.sent = []

    @property
    def direct_joints(self):  # type: ignore[override]
        return tuple(self.embodiment.joint_names[15:29])  # arms

    def _send_action(self, action):
        self.sent.append(action)
        if action.mode == ControlMode.PD:
            super()._send_action(action)

    def hold(self):
        self.held = True


def _balancing_assembly(monkeypatch):
    from hirobot.devices.end_effector import body_embodiment
    from tests.test_end_effectors import _hands

    emb, bus, hands = _hands(monkeypatch)
    body = _SelfBalancing(body_embodiment(emb))
    return RobotAssembly(body, end_effectors=hands, embodiment=emb), body, bus, emb


def test_assembly_routes_finger_targets_beside_a_non_joint_mode(monkeypatch):
    robot, body, bus, emb = _balancing_assembly(monkeypatch)
    assert ControlMode.BASE_VELOCITY in robot.action_modes
    fingers = [n for ee in emb.end_effectors for n in ee.joints]
    assert set(fingers) <= set(robot.direct_joints)
    assert "left_elbow" in robot.direct_joints
    robot.connect()
    left = fingers[:7]
    names = ["left_elbow", *left]
    target = np.array([0.3, 0.1, 0.2, 0.3, -0.4, -0.5, -0.6, -0.7])
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.2, 0, 0]),
                             joint_names=names, joint_target=target, kp=np.full(8, 5.0)))
    sent = body.sent[-1]
    assert sent.mode == ControlMode.BASE_VELOCITY and sent.joint_names == ["left_elbow"]
    assert np.allclose(sent.joint_target, [0.3]) and np.allclose(sent.kp, [5.0])
    cmd = bus.sent["rt/dex3/left/cmd"][-1]
    assert np.allclose([m[1] for m in cmd], target[1:]) and [m[2] for m in cmd] == [5.0] * 7
    assert not bus.sent["rt/dex3/right/cmd"]


def test_assembly_needs_a_whole_hand(monkeypatch):
    robot, body, bus, emb = _balancing_assembly(monkeypatch)
    robot.connect()
    finger = emb.end_effectors[0].joints[0]
    with pytest.raises(ValueError, match="all its joints"):
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                                 joint_names=[finger], joint_target=np.zeros(1)))
    assert not body.sent


def test_finger_only_targets_still_send_the_body_command(monkeypatch):
    robot, body, bus, emb = _balancing_assembly(monkeypatch)
    robot.connect()
    right = emb.end_effectors[1].joints
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                             joint_names=right, joint_target=np.zeros(7)))
    sent = body.sent[-1]
    assert sent.joint_names is None and sent.joint_target is None
    assert bus.sent["rt/dex3/right/cmd"]


def test_joint_assembly_keeps_refusing_torque(monkeypatch):
    robot, body, bus, emb = _balancing_assembly(monkeypatch)
    assert ControlMode.PD in robot.action_modes
    assert ControlMode.TORQUE not in robot.action_modes


def test_hold_beside_a_self_balancing_body(monkeypatch):
    from tests.test_end_effectors import LEFT_Q

    robot, body, bus, emb = _balancing_assembly(monkeypatch)
    robot.connect()
    robot.hold()
    assert body.held and not body.sent  # the body holds itself, no joint freeze
    cmd = bus.sent["rt/dex3/left/cmd"][-1]
    assert np.allclose([m[1] for m in cmd], LEFT_Q)


def test_one_assembly_under_the_layers_and_devices_reach_the_caller(fake_layer):
    cfg = _cfg(layers=[{"type": "fake_wbc"}])
    cfg["assembly"]["parts"]["head"] = {"device": "camera", "width": 64, "height": 64,
                                        "driver": {"type": "array", "fps": 10}}
    robot = make_robot(cfg)
    # caller -> layer -> (gate) assembly -> driver, with one assembly only
    assert isinstance(robot, FakeWbc)
    assembly = robot._inner
    assert isinstance(assembly, RobotAssembly) and isinstance(assembly.body, ToyRobot)
    assert assembly.safety is not None and assembly.body.safety is None
    assert set(robot.cameras) == {"head"} and robot.cameras is assembly.cameras
    assert dict(robot.microphones) == {} and dict(robot.speakers) == {}
    robot.connect()
    try:
        assert "head" in robot.get_observation().images
    finally:
        robot.disconnect()


def test_a_bare_driver_has_no_devices():
    robot = make_robot(_cfg())
    assert isinstance(robot, ToyRobot)
    assert dict(robot.cameras) == {} and dict(robot.microphones) == {}
    assert dict(robot.speakers) == {}


def test_layer_base_forwards_to_inner():
    inner = ToyRobot(_emb(4))
    layer = Layer(inner)
    assert layer.embodiment is inner.embodiment
    assert layer.action_modes == inner.action_modes and layer.simulated
    layer.connect()
    layer.reset()
    layer.send_action(Action(mode=ControlMode.POSITION, joint_target=np.full(4, 0.1)))
    assert np.allclose(inner.get_observation().joint_pos, 0.1, atol=0.05)
    inner._set_fault("driver damped")
    assert layer.fault == "driver damped" and layer.get_status().fault == "driver damped"
    layer.reset()
    assert layer.fault is None
    layer.estop()
    assert inner.fault is not None  # the stop reached the robot underneath
    layer.disconnect()


# ------------------------------------------------- e-stop order and inner proxy


class _StuckLayer(Layer):
    """A layer whose own _estop hangs or raises."""

    def __init__(self, inner, how="hang"):
        super().__init__(inner)
        self.how = how
        self.estopped = False

    def _estop(self):
        self.estopped = True
        if self.how == "hang":
            time.sleep(5)
        raise RuntimeError("layer stop failed")


@pytest.mark.parametrize("how", ["hang", "raise"])
def test_estop_reaches_the_bottom_even_if_a_layer_hangs(how):
    bottom = ToyRobot(_emb(4))
    calls = []
    bottom_estop = bottom.estop
    bottom.estop = lambda: (calls.append(time.monotonic()), bottom_estop())
    middle = _StuckLayer(bottom, how)
    outer = _StuckLayer(middle, "raise")
    outer.connect()
    t0 = time.monotonic()
    outer.estop()
    assert calls and calls[0] - t0 < 0.05  # the hardware first, before any layer
    assert time.monotonic() - t0 < 0.25 + ESTOP_LAYER_TIMEOUT_S
    assert middle.estopped and outer.estopped  # every layer still got its turn
    for robot in (outer, middle):
        assert robot.fault is not None
        with pytest.raises(RobotFaultError):
            robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(4)))
    assert bottom.fault is not None


def test_estop_from_a_middle_layer_still_hits_the_bottom():
    bottom = ToyRobot(_emb(4))
    middle = Layer(bottom)
    outer = Layer(middle)
    middle.estop()
    assert bottom.fault is not None and middle.fault is not None
    assert outer.fault is not None  # reported from below


def test_inner_is_public_api_only():
    bottom = ToyRobot(_emb(4))
    layer = Layer(bottom)
    assert isinstance(layer.inner, ToyRobot) and layer.inner.embodiment is bottom.embodiment
    for name in ("_send_action", "_set_safety", "_estop", "_fault"):
        with pytest.raises(AttributeError, match="public API"):
            getattr(layer.inner, name)
    with pytest.raises(AttributeError):
        layer.inner.safety = None
    assembly = RobotAssembly(bottom, cameras={})
    with pytest.raises(AttributeError, match="public API"):
        Layer(assembly).inner.body
    # Stacking layers does not wrap a proxy in a proxy.
    assert type(Layer(layer.inner).inner) is type(layer.inner)


def test_assert_layer_uses_public_api(fake_layer):
    used = assert_layer_uses_public_api(FakeWbc)
    assert {"send_action", "get_observation"} <= used

    class Sneaky(Layer):
        def _send_action(self, action):
            try:
                self.inner._send_action(action)
            except AttributeError:
                pass

    with pytest.raises(AssertionError, match="_send_action"):
        assert_layer_uses_public_api(Sneaky)
    with pytest.raises(AssertionError, match="does not subclass"):
        assert_layer_uses_public_api(lambda inner: ToyRobot(inner.embodiment))
