"""RobotGroup: several complete robots driven as one.

Members are toy sims (and a fake whole-body-control layer for the fault and
ownership cases), plus an array scene camera.
"""

from __future__ import annotations

import threading

import numpy as np
import pytest
import yaml

from hirobot import (
    Action,
    ControlMode,
    Robot,
    RobotFaultError,
    RobotGroup,
    RobotStatus,
    SafetyError,
    make_robot,
)
from hirobot.config import available_configs, load_config
from hirobot.drivers.camera.generic.array import ArrayCamera
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.embodiment import Embodiment
from hirobot.robot_skills import RobotSkillSpec
from tests.drivers.contract import check_robot_behaviour


def _arm(name: str, n: int, ee: str = "ee", hz: int = 30) -> Embodiment:
    return Embodiment(name=name, kind="arm", floating_base=False, control_hz=hz,
                      joints=[{"name": f"j{i}"} for i in range(n)],
                      end_effectors=[{"name": ee, "has_gripper": True}],
                      cameras=[{"name": "wrist", "width": 8, "height": 8}])


def _humanoid(n: int = 3) -> Embodiment:
    return Embodiment(name="toy-humanoid", control_hz=30,
                      joints=[{"name": f"leg{i}"} for i in range(n)] + [{"name": "finger"}],
                      end_effectors=[{"name": "left_hand"}])


class Recorder(ToyRobot):
    """A toy arm that records what reaches it and can be told to fail."""

    action_modes = (ControlMode.POSITION, ControlMode.PD, ControlMode.CARTESIAN)

    def __init__(self, emb, **kw):
        super().__init__(emb, **kw)
        self.sent = []
        self.held = 0
        self.resets = 0
        self.fail_send = False
        self.fail_obs = False

    def _send_action(self, action):
        if self.fail_send:
            raise ConnectionError("SDK lost the arm")
        self.sent.append(action)
        super()._send_action(action)

    def get_observation(self):
        if self.fail_obs:
            raise ConnectionError("no state")
        obs = super().get_observation()
        ee = self.embodiment.end_effectors[0].name
        obs.ee_poses = {ee: np.array([0, 0, 0, 0, 0, 0, 1.0])}
        obs.gripper = {ee: 0.25}
        obs.depth = {"wrist": np.ones((8, 8), dtype=np.float32)}
        obs.extra = {"raw": 1}
        return obs

    def hold(self):
        self.held += 1

    def reset(self, seed=None):
        self.resets += 1
        super().reset(seed)
        self.clear_fault()


class FakeWbc(Robot):
    """A balancing layer over a toy body: a thread of its own commands ``inner``."""

    action_modes = (ControlMode.BASE_VELOCITY, ControlMode.CARTESIAN)
    cartesian_targets = ("left_hand",)
    direct_joints = ("finger",)

    def __init__(self, inner):
        self.inner = inner
        self.received = []
        self.held = 0

    @property
    def embodiment(self):
        return self.inner.embodiment

    @property
    def simulated(self):  # type: ignore[override]
        return True

    def connect(self):
        self.inner.connect()

    def reset(self, seed=None):
        self.inner.reset(seed)
        self.clear_fault()

    #: Odometry the fake reports; ``None`` = no base_position (a real robot
    #: without odometry).
    odom_position = np.zeros(3)
    odom_yaw = 0.0

    def get_observation(self):
        obs = self.inner.get_observation()
        if self.odom_position is not None:
            obs.base_position = np.asarray(self.odom_position, dtype=float)
        h = self.odom_yaw / 2
        obs.base_orientation = np.array([0.0, 0.0, np.sin(h), np.cos(h)])
        # Left hand 0.3 m in front of the pelvis, in the body frame.
        obs.ee_poses = {"left_hand": np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])}
        return obs

    def _send_action(self, action):
        self.received.append(action)

    def tick(self):
        """What the controller thread does each tick: joint targets to the body."""
        n = self.inner.embodiment.num_dofs
        self.inner.send_action(Action(mode=ControlMode.PD, joint_target=np.zeros(n)))

    def damp_on_error(self, why):
        self.set_fault(f"damped: {why}")

    def hold(self):
        self.held += 1

    def list_robot_skills(self):
        return [RobotSkillSpec(name="wave", description="wave a hand")]

    @property
    def has_robot_skills(self):
        return True

    def _run_robot_skill(self, spec, args):
        self.received.append(("skill", spec.name))


def _pair(**kw):
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 3, ee="tcp"))
    group = RobotGroup({"left": left, "right": right}, **kw)
    group.connect()
    return group, left, right


# -------------------------------------------------------------- contract


def test_group_passes_the_robot_contract():
    """A group is a Robot: same lifecycle, shapes and gate rules as a single driver."""
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 3, ee="tcp"))
    group = RobotGroup({"left": left, "right": right}, cameras={"scene": ArrayCamera("scene")})
    check_robot_behaviour(group, lambda: len(left.sent) + len(right.sent), label="group")


# -------------------------------------------------------------- structure


def test_names_are_prefixed_and_joints_concatenated():
    cam = ArrayCamera("scene")
    group, left, right = _pair(cameras={"scene": cam})
    emb = group.embodiment
    assert emb.joint_names == ["left/j0", "left/j1", "right/j0", "right/j1", "right/j2"]
    assert [e.name for e in emb.end_effectors] == ["left/ee", "right/tcp"]
    assert [c.name for c in emb.cameras] == ["left/wrist", "right/wrist", "scene"]
    assert emb.kind.value == "arm" and not emb.floating_base and emb.control_hz == 30
    assert group.simulated


def test_observation_concatenates_and_prefixes():
    group, left, right = _pair(cameras={"scene": ArrayCamera("scene")})
    left._joint_pos = np.array([1.0, 2.0])
    obs = group.get_observation()
    assert np.array_equal(obs.joint_pos, [1, 2, 0, 0, 0])
    assert set(obs.ee_poses) == {"left/ee", "right/tcp"}
    assert obs.gripper == {"left/ee": 0.25, "right/tcp": 0.25}
    assert set(obs.depth) == {"left/wrist", "right/wrist"}
    assert "scene" in obs.images
    assert obs.extra["left/raw"] == 1 and "timestamps" not in obs.extra
    assert {"left/body", "right/body", "scene"} <= set(obs.stamps)
    assert obs.base_orientation is None


def test_member_view_reads_but_cannot_command():
    group, left, _ = _pair()
    view = group.member("left")
    assert view.embodiment is left.embodiment and view.get_observation().joint_pos.shape == (2,)
    assert not hasattr(view, "send_action")
    with pytest.raises(KeyError):
        group.member("nope")


# ---------------------------------------------------------------- actions


def test_flat_joint_target_is_split_by_member():
    group, left, right = _pair()
    group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.arange(5.0)))
    assert np.array_equal(left.sent[-1].joint_target, [0, 1])
    assert np.array_equal(right.sent[-1].joint_target, [2, 3, 4])


def test_one_refusal_sends_nothing():
    group, left, right = _pair()
    from hirobot import SafetyGate

    right.set_safety(SafetyGate.from_embodiment(right.embodiment, joint_limit=1.0))
    with pytest.raises(SafetyError, match="member 'right'"):
        group.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.array([0, 0, 0, 0, 5.0])))
    assert not left.sent and not right.sent
    assert group.fault is None  # a refusal is not a failure


def test_member_gets_the_clamped_action():
    from hirobot import SafetyGate

    group, left, right = _pair()
    right.set_safety(SafetyGate.from_embodiment(right.embodiment, joint_limit=1.0, clamp=True))
    group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array([0, 0, 0, 0, 5.0])))
    assert np.array_equal(right.sent[-1].joint_target, [0, 0, 1.0])


def test_unmentioned_member_keeps_its_target_and_idles():
    group, left, right = _pair()
    group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.arange(5.0)))
    before = right.idle_steps
    group.send_action(Action(mode=ControlMode.CARTESIAN,
                             ee_targets={"left/ee": np.array([0, 0, 0, 0, 0, 0, 1.0])}))
    assert left.sent[-1].ee_targets.keys() == {"ee"}
    assert len(right.sent) == 1  # nothing new for the arm not named
    assert right.idle_steps == before + 1 and left.idle_steps == 0


def test_bad_names_and_lengths_are_refused():
    group, left, right = _pair()
    with pytest.raises(SafetyError, match="no member 'mid'"):
        group.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"mid/ee": np.zeros(7)}))
    with pytest.raises(SafetyError, match="expected '<member>/<name>'"):
        group.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"ee": np.zeros(7)}))
    with pytest.raises(SafetyError):  # the group's own gate: 5 joints
        group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(4)))
    with pytest.raises(SafetyError, match="floating-base"):
        group.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)))
    assert not left.sent and not right.sent


def test_action_modes_are_the_intersection():
    left, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    group = RobotGroup({"left": left, "g1": wbc})
    assert group.action_modes == (ControlMode.CARTESIAN,)  # the only mode both take
    assert group.direct_joints == ("g1/finger",)


# --------------------------------------------------------------- errors


def test_send_failure_holds_the_others_and_refuses_until_reset():
    group, left, right = _pair()
    right.fail_send = True
    with pytest.raises(ConnectionError):
        group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))
    assert left.held == 1 and right.held == 0
    assert "right" in group.fault and "right" in group.get_status().fault
    right.fail_send = False
    with pytest.raises(RobotFaultError):
        group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))
    group.reset()
    assert group.fault is None and left.resets == right.resets == 1
    group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))


def test_observation_failure_also_stops_the_group():
    group, left, right = _pair()
    left.fail_obs = True
    with pytest.raises(ConnectionError):
        group.get_observation()
    assert right.held == 1 and group.fault is not None


def test_member_fault_between_sends_stops_the_group():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    group = RobotGroup({"arm": arm, "g1": wbc})
    group.connect()
    wbc.damp_on_error("SafetyError on the joint command")  # its own thread hit the inner gate
    assert group.get_status().fault.startswith("member 'g1'")
    with pytest.raises(RobotFaultError, match="g1"):
        group.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.zeros(group.embodiment.num_dofs)))
    assert arm.held == 1 and not arm.sent


def test_fault_state_refuses_on_a_single_robot_too():
    robot = Recorder(_arm("a", 2))
    robot.set_fault("estop")
    assert robot.get_status().fault == "estop"
    with pytest.raises(RobotFaultError, match="estop"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))
    robot.reset()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))


# ------------------------------------------------------------- ownership


def test_member_cannot_be_commanded_around_the_group():
    group, left, _ = _pair()
    with pytest.raises(RuntimeError, match="member of a RobotGroup"):
        left.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))
    other = []

    def from_other_thread():
        try:
            left.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))
        except RuntimeError as exc:
            other.append(exc)

    # A group call in progress on one thread does not open the member to others.
    with group._calling():
        t = threading.Thread(target=from_other_thread)
        t.start()
        t.join()
    assert other and not left.sent
    with pytest.raises(ValueError, match="another RobotGroup"):
        RobotGroup({"x": left})


def test_wbc_member_thread_still_commands_its_inner_robot():
    body = ToyRobot(_humanoid())
    wbc = FakeWbc(body)
    group = RobotGroup({"g1": wbc, "arm": Recorder(_arm("a", 2))})
    group.connect()
    t = threading.Thread(target=wbc.tick)  # the controller thread, outside any group call
    t.start()
    t.join()
    assert body._last_mode == ControlMode.PD  # the inner robot carries no group mark
    with pytest.raises(RuntimeError):
        wbc.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)))


def test_skills_are_prefixed_and_routed():
    wbc = FakeWbc(ToyRobot(_humanoid()))
    group = RobotGroup({"g1": wbc, "arm": Recorder(_arm("a", 2))})
    assert [s.name for s in group.list_robot_skills()] == ["g1/wave"]
    group.run_robot_skill("g1/wave")
    assert wbc.received[-1] == ("skill", "wave")
    with pytest.raises(RuntimeError, match="member of a RobotGroup"):
        wbc.run_robot_skill("wave")


# ----------------------------------------------------------- floating base


def test_single_floating_member_keeps_top_level_base_and_takes_base_commands():
    wbc = FakeWbc(ToyRobot(_humanoid()))
    arm = Recorder(_arm("a", 2))
    group = RobotGroup({"g1": wbc, "arm": arm})
    group.connect()
    assert group.embodiment.floating_base and group.embodiment.kind.value == "humanoid"
    obs = group.get_observation()
    assert obs.base_position is not None and "g1/base_position" not in obs.extra
    group.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.2, 0, 0]),
                             joint_names=["g1/finger"], joint_target=np.array([0.3])))
    sent = wbc.received[-1]
    assert np.allclose(sent.base_velocity, [0.2, 0, 0])
    assert sent.joint_names == ["finger"] and np.allclose(sent.joint_target, [0.3])
    assert arm.idle_steps == 1


def test_status_is_merged():
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 1))
    left.get_status = lambda: RobotStatus(joint_temperature=np.array([30.0, 31.0]),
                                          joint_faults={"j1": 7},
                                          battery={"soc": 80.0})
    group = RobotGroup({"left": left, "right": right})
    status = group.get_status()
    assert np.allclose(status.joint_temperature[:2], [30, 31]) and np.isnan(status.joint_temperature[2])
    assert status.joint_faults == {"left/j1": 7} and status.battery.soc == 80.0


# ------------------------------------------------------------------ config


def _member_file(tmp_path, name, emb):
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump({"embodiment": emb.model_dump(mode="json"),
                                    "body": {"driver": "toy"}}))
    return path


def test_group_config_builds_members_and_scene_cameras(tmp_path):
    _member_file(tmp_path, "a", _arm("a", 2))
    _member_file(tmp_path, "b", _arm("b", 3, ee="tcp"))
    cfg = {"group": {"name": "dual", "control_hz": 20,
                     "members": {"left": {"config": "./a.yaml"}, "right": {"config": "./b.yaml"}},
                     "cameras": [{"name": "scene", "width": 64, "height": 64,
                                  "driver": {"type": "array"}}]},
           "safety": {"joint_limit": 2.0}}
    path = tmp_path / "dual.yaml"
    path.write_text(yaml.safe_dump(cfg))
    group = make_robot(str(path))
    assert isinstance(group, RobotGroup) and group.embodiment.name == "dual"
    assert group.embodiment.control_hz == 20 and set(group.cameras) == {"scene"}
    assert group.safety.joint_limit == 2.0 and group.members["left"].safety is not None
    group.connect()
    with pytest.raises(SafetyError):  # the group's gate
        group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.full(5, 3.0)))
    group.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))


@pytest.mark.parametrize("name", available_configs("groups"))
def test_builtin_group_configs_run(name):
    group = make_robot(name)
    assert isinstance(group, RobotGroup)
    sent = []
    for robot in group.members.values():  # count what reaches each member
        inner = robot._send_action
        robot._send_action = lambda a, inner=inner: (sent.append(a), inner(a))
    check_robot_behaviour(group, lambda: len(sent), label=name)


def test_example_group_names_and_overrides(tmp_path):
    group = make_robot("groups/franka_ur5e_dual")
    emb = group.embodiment
    assert [e.name for e in emb.end_effectors] == ["left_arm", "right_arm"]
    assert [c.name for c in emb.cameras] == ["agentview"]
    assert emb.num_dofs == 13 and emb.control_hz == 30
    mine = tmp_path / "mine.yaml"
    mine.write_text(yaml.safe_dump({"defaults": ["groups/franka_ur5e_dual"],
                                    "group": {"members": {"left": {"overrides": {"seed": 3}}}}}))
    cfg = load_config(str(mine))["group"]["members"]["left"]
    assert cfg == {"config": "franka_fr3", "base_pose": [0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 1.0],
                   "overrides": {"seed": 3}}


def test_group_config_errors(tmp_path):
    _member_file(tmp_path, "a", _arm("a", 2))
    base = {"members": {"left": {"config": str(tmp_path / "a.yaml")}}}
    with pytest.raises(ValueError, match="not both"):
        make_robot({"group": base, "body": {"driver": "toy"}})
    with pytest.raises(ValueError, match="on_error"):
        make_robot({"group": {**base, "on_error": "continue"}})
    with pytest.raises(ValueError, match="base_poses"):
        make_robot({"group": {"members": {"left": {"config": str(tmp_path / "a.yaml"),
                                                   "base_pose": [0, 0, 0]}}}})
    with pytest.raises(ValueError, match="expose.ee"):
        make_robot({"group": {**base, "expose": {"ee": {"left/nope": "x"}}}})
    with pytest.raises(ValueError, match="keyed by member"):
        make_robot({"group": base}, body_overrides={"ip": "1.2.3.4"})


def test_body_overrides_go_to_the_named_member(tmp_path):
    _member_file(tmp_path, "a", _arm("a", 2))
    cfg = {"group": {"members": {"left": {"config": str(tmp_path / "a.yaml"),
                                          "overrides": {"dt": 0.5}}}}}
    group = make_robot(cfg, body_overrides={"left": {"dt": 0.25}})
    assert group.members["left"]._dt == 0.25


def test_cli_info_shows_a_group(tmp_path, capsys):
    from hirobot.cli import main

    _member_file(tmp_path, "a", _arm("a", 2))
    path = tmp_path / "g.yaml"
    path.write_text(yaml.safe_dump({"group": {"members": {"left": {"config": "./a.yaml"}}}}))
    assert main(["info", "--config", str(path)]) == 0
    out = capsys.readouterr().out
    assert "Member left" in out and "left/j0" in out


# ------------------------------------------------- Action.members (step 2)


def _mixed():
    arm, body = Recorder(_arm("a", 2)), ToyRobot(_humanoid())
    wbc = FakeWbc(body)
    group = RobotGroup({"left": arm, "g1": wbc})
    group.connect()
    return group, arm, wbc


def test_members_action_gives_each_member_its_own_mode():
    group, arm, wbc = _mixed()
    group.send_action(Action(members={
        "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2])),
        "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.2, 0, 0]),
                     joint_names=["finger"], joint_target=np.array([0.5])),
    }))
    assert np.allclose(arm.sent[-1].joint_target, [0.1, 0.2])
    sent = wbc.received[-1]
    assert sent.mode == ControlMode.BASE_VELOCITY and sent.joint_names == ["finger"]


def test_members_action_is_checked_per_member_before_sending():
    group, arm, wbc = _mixed()
    with pytest.raises(SafetyError, match="member 'g1'"):
        group.send_action(Action(members={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)),
            # leg0 belongs to the controller, not free to command
            "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                         joint_names=["leg0"], joint_target=np.zeros(1)),
        }))
    assert not arm.sent and not wbc.received
    with pytest.raises(SafetyError, match="does not take mode"):
        group.send_action(Action(members={
            "left": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3))}))
    with pytest.raises(SafetyError, match="no member"):
        group.send_action(Action(members={"mid": Action(joint_target=np.zeros(2))}))


def test_members_action_rules():
    with pytest.raises(ValueError, match="no other command fields"):
        Action(members={"left": Action()}, joint_target=np.zeros(2))
    from hirobot import SafetyGate

    group, arm, _ = _mixed()
    group.set_safety(SafetyGate.from_embodiment(group.embodiment))  # as make_robot does
    with pytest.raises(SafetyError, match="non-finite"):  # the group's gate
        group.send_action(Action(members={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.array([np.nan, 0]))}))
    with pytest.raises(SafetyError, match="single robot"):
        arm._group = None
        arm.send_action(Action(members={"x": Action()}))


def test_members_chunk_steps_into_members_actions():
    from hirobot import ActionChunk

    chunk = ActionChunk(members={
        "left": ActionChunk(mode=ControlMode.POSITION, joint_targets=np.arange(8.0).reshape(4, 2)),
        "g1": ActionChunk(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros((4, 3)))})
    assert chunk.horizon == 4
    group, arm, wbc = _mixed()
    group.send_action(chunk.step(2))
    assert np.allclose(arm.sent[-1].joint_target, [4, 5])
    with pytest.raises(ValueError, match="different horizons"):
        ActionChunk(members={"a": ActionChunk(joint_targets=np.zeros((2, 2))),
                             "b": ActionChunk(joint_targets=np.zeros((3, 2)))})


def test_wbc_fault_between_sends_holds_the_arm():
    group, arm, wbc = _mixed()
    wbc.damp_on_error("SafetyError in its own thread")
    with pytest.raises(RobotFaultError, match="damped"):
        group.send_action(Action(members={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.zeros(2))}))
    assert arm.held == 1 and not arm.sent
    assert "damped" in group.get_status().fault


# ------------------------------------------------- frames and expose (step 3)

YAW90 = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]


def test_base_poses_put_ee_poses_and_targets_in_the_world_frame():
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 3, ee="tcp"))
    group = RobotGroup({"left": left, "right": right},
                       base_poses={"left": [0, 0.4, 0, 0, 0, 0, 1], "right": [1, 0, 0, *YAW90]})
    group.connect()
    obs = group.get_observation()  # each member reports its ee at its own base origin
    assert np.allclose(obs.ee_poses["left/ee"][:3], [0, 0.4, 0])
    assert np.allclose(obs.ee_poses["right/tcp"], [1, 0, 0, *YAW90])
    # A world target 0.5 m along +y from the right base is +x in the right arm's frame.
    group.send_action(Action(mode=ControlMode.CARTESIAN,
                             ee_targets={"right/tcp": np.array([1, 0.5, 0, *YAW90])}))
    assert np.allclose(right.sent[-1].ee_targets["tcp"], [0.5, 0, 0, 0, 0, 0, 1])


def test_cartesian_delta_is_rotated_into_the_member_frame():
    right = Recorder(_arm("b", 3, ee="tcp"))
    right.action_modes = (ControlMode.CARTESIAN_DELTA,)
    group = RobotGroup({"right": right}, base_poses={"right": [5, 5, 0, *YAW90]})
    group.send_action(Action(mode=ControlMode.CARTESIAN_DELTA,
                             ee_targets={"right/tcp": np.array([0, 0.1, 0, 0, 0, 0.2])}))
    # +y in the world is +x for a base turned 90 deg; the z rotation is unchanged.
    assert np.allclose(right.sent[-1].ee_targets["tcp"], [0.1, 0, 0, 0, 0, 0.2])


def test_floating_member_frame_follows_its_odometry():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    group = RobotGroup({"left": arm, "g1": wbc}, base_poses={"g1": [10, 0, 0, 0, 0, 0, 1]})
    wbc.odom_position, wbc.odom_yaw = np.array([1.0, 0, 0]), np.pi / 2
    obs = group.get_observation()
    # hand 0.3 m ahead of a pelvis at world x=11 facing +y
    assert np.allclose(obs.ee_poses["g1/left_hand"][:3], [11, 0.3, 0])
    group.send_action(Action(members={"g1": Action(
        mode=ControlMode.CARTESIAN, ee_targets={"left_hand": np.array([11, 0.3, 0, *YAW90])})}))
    assert np.allclose(wbc.received[-1].ee_targets["left_hand"], [0.3, 0, 0, 0, 0, 0, 1])
    # body-frame targets pass untouched with frame: base
    group.send_action(Action(members={"g1": Action(
        mode=ControlMode.CARTESIAN, ee_targets={"left_hand": np.array([0.2, 0, 0, 0, 0, 0, 1])},
        extra={"frame": "base"})}))
    assert np.allclose(wbc.received[-1].ee_targets["left_hand"][:3], [0.2, 0, 0])


def test_floating_member_without_odometry_has_no_world_frame():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    wbc.odom_position = None
    group = RobotGroup({"left": arm, "g1": wbc}, base_poses={"g1": [0, 0, 0, 0, 0, 0, 1]})
    obs = group.get_observation()
    assert "g1/left_hand" not in obs.ee_poses and "g1/ee_poses" in obs.extra
    assert "g1/left_hand" not in group.cartesian_targets
    with pytest.raises(SafetyError, match="no base_position"):
        group.send_action(Action(members={"g1": Action(
            mode=ControlMode.CARTESIAN, ee_targets={"left_hand": np.array([0, 0, 0, 0, 0, 0, 1.0])})}))


def test_floating_member_without_base_pose_keeps_its_own_frame():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    wbc.odom_position = None  # no odometry, but none needed without a base_pose
    group = RobotGroup({"left": arm, "g1": wbc})
    assert np.allclose(group.get_observation().ee_poses["g1/left_hand"][:3], [0.3, 0, 0])
    assert "g1/left_hand" in group.cartesian_targets


def test_expose_renames_ee_and_cameras_everywhere():
    group, left, right = _pair(cameras={"scene": ArrayCamera("scene")},
                               expose={"ee": {"left/ee": "left_arm", "right/tcp": "right_arm"},
                                       "cameras": {"scene": "agentview",
                                                   "left/wrist": "left_wrist"}})
    emb = group.embodiment
    assert [e.name for e in emb.end_effectors] == ["left_arm", "right_arm"]
    assert [c.name for c in emb.cameras] == ["left_wrist", "right/wrist", "agentview"]
    assert "agentview" in group.get_observation().images
    assert set(group.get_observation().ee_poses) == {"left_arm", "right_arm"}
    assert set(group.get_observation().gripper) == {"left_arm", "right_arm"}
    assert set(group.get_observation().depth) == {"left_wrist", "right/wrist"}
    group.send_action(Action(mode=ControlMode.CARTESIAN,
                             ee_targets={"right_arm": np.array([0, 0, 0, 0, 0, 0, 1.0])},
                             gripper={"left_arm": 0.5}))
    assert set(right.sent[-1].ee_targets) == {"tcp"} and left.sent[-1].gripper == {"ee": 0.5}
    with pytest.raises(SafetyError, match="no end-effector"):
        group.send_action(Action(mode=ControlMode.CARTESIAN,
                                 ee_targets={"right/tcp": np.array([0, 0, 0, 0, 0, 0, 1.0])}))


def test_expose_errors():
    a, b = Recorder(_arm("a", 2)), Recorder(_arm("b", 3))
    with pytest.raises(ValueError, match="no such name"):
        RobotGroup({"left": a, "right": b}, expose={"ee": {"mid/ee": "x"}})
    with pytest.raises(ValueError, match="more than once"):
        RobotGroup({"left": a, "right": b}, expose={"ee": {"left/ee": "right/ee"}})
    with pytest.raises(ValueError, match="base_poses"):
        RobotGroup({"left": a}, base_poses={"left": [0, 0, 0]})


def test_group_config_with_base_pose_and_expose(tmp_path):
    _member_file(tmp_path, "a", _arm("a", 2))
    cfg = {"group": {
        "members": {"left": {"config": "a.yaml", "base_pose": [0, 0.4, 0, 0, 0, 0, 1]}},
        "cameras": [{"name": "scene", "width": 64, "height": 64, "driver": {"type": "array"}}],
        "expose": {"ee": {"left/ee": "left_arm"}, "cameras": {"scene": "agentview"}}}}
    path = tmp_path / "g.yaml"
    path.write_text(yaml.safe_dump(cfg))
    group = make_robot(str(path))
    assert [e.name for e in group.embodiment.end_effectors] == ["left_arm"]
    assert np.allclose(group.base_poses["left"], [0, 0.4, 0, 0, 0, 0, 1])
    group.connect()
    assert "agentview" in group.get_observation().images  # the toy sim reports no ee_poses
