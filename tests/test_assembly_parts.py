"""RobotAssembly with child parts: several complete robots driven as one.

Parts are toy sims (and a fake whole-body-control layer for the fault and
ownership cases), plus an array scene camera.
"""

from __future__ import annotations

import threading

import numpy as np
import pytest

from hirobot import (
    Action,
    CommandProgress,
    ControlMode,
    Robot,
    RobotAssembly,
    RobotBusyError,
    RobotFaultError,
    RobotStatus,
    SafetyError,
    make_robot,
)
from hirobot.drivers.camera.generic.array import ArrayCamera
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.core.embodiment import Embodiment
from hirobot.core.robot_skills import RobotSkillSpec
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
        self._clear_fault()


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
        self._clear_fault()

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
        self._set_fault(f"damped: {why}")

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
    robot = RobotAssembly(None, parts={"left": left, "right": right}, **kw)
    robot.connect()
    return robot, left, right


# -------------------------------------------------------------- contract


def test_assembly_with_parts_passes_the_robot_contract():
    """An assembly of parts is a Robot: same lifecycle, shapes and gate rules as one driver."""
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 3, ee="tcp"))
    robot = RobotAssembly(None, parts={"left": left, "right": right},
                          cameras={"scene": ArrayCamera("scene")})
    check_robot_behaviour(robot, lambda: len(left.sent) + len(right.sent), label="assembly")


# -------------------------------------------------------------- structure


def test_names_are_prefixed_and_joints_concatenated():
    cam = ArrayCamera("scene")
    robot, left, right = _pair(cameras={"scene": cam})
    emb = robot.embodiment
    assert emb.joint_names == ["left/j0", "left/j1", "right/j0", "right/j1", "right/j2"]
    assert [e.name for e in emb.end_effectors] == ["left/ee", "right/tcp"]
    assert [c.name for c in emb.cameras] == ["left/wrist", "right/wrist", "scene"]
    assert emb.kind.value == "arm" and not emb.floating_base and emb.control_hz == 30
    assert robot.simulated


def test_observation_concatenates_and_prefixes():
    robot, left, right = _pair(cameras={"scene": ArrayCamera("scene")})
    left._joint_pos = np.array([1.0, 2.0])
    obs = robot.get_observation()
    assert np.array_equal(obs.joint_pos, [1, 2, 0, 0, 0])
    assert set(obs.ee_poses) == {"left/ee", "right/tcp"}
    assert obs.gripper == {"left/ee": 0.25, "right/tcp": 0.25}
    assert set(obs.depth) == {"left/wrist", "right/wrist"}
    assert "scene" in obs.images
    assert obs.extra["left/raw"] == 1 and "timestamps" not in obs.extra
    assert {"left/body", "right/body", "scene"} <= set(obs.stamps)
    assert obs.base_orientation is None


def test_part_view_reads_but_cannot_command():
    robot, left, _ = _pair()
    view = robot.part("left")
    assert view.embodiment is left.embodiment and view.get_observation().joint_pos.shape == (2,)
    assert not hasattr(view, "send_action")
    with pytest.raises(KeyError):
        robot.part("nope")
    with pytest.raises(KeyError):
        robot.part("left/deeper")  # left has no parts of its own


# ---------------------------------------------------------------- actions


def test_flat_joint_target_is_split_by_part():
    robot, left, right = _pair()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.arange(5.0)))
    assert np.array_equal(left.sent[-1].joint_target, [0, 1])
    assert np.array_equal(right.sent[-1].joint_target, [2, 3, 4])


def test_one_refusal_sends_nothing():
    robot, left, right = _pair()
    from hirobot import SafetyGate

    right._set_safety(SafetyGate.from_embodiment(right.embodiment, joint_limit=1.0))
    with pytest.raises(SafetyError, match="part 'right'"):
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.array([0, 0, 0, 0, 5.0])))
    assert not left.sent and not right.sent
    assert robot.fault is None  # a refusal is not a failure


def test_part_gets_the_clamped_action():
    from hirobot import SafetyGate

    robot, left, right = _pair()
    right._set_safety(SafetyGate.from_embodiment(right.embodiment, joint_limit=1.0, clamp=True))
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array([0, 0, 0, 0, 5.0])))
    assert np.array_equal(right.sent[-1].joint_target, [0, 0, 1.0])


def test_unmentioned_part_keeps_its_target_and_idles():
    robot, left, right = _pair()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.arange(5.0)))
    before = right.idle_steps
    robot.send_action(Action(mode=ControlMode.CARTESIAN,
                             ee_targets={"left/ee": np.array([0, 0, 0, 0, 0, 0, 1.0])}))
    assert left.sent[-1].ee_targets.keys() == {"ee"}
    assert len(right.sent) == 1  # nothing new for the arm not named
    assert right.idle_steps == before + 1 and left.idle_steps == 0


def test_bad_names_and_lengths_are_refused():
    robot, left, right = _pair()
    with pytest.raises(SafetyError, match="no part 'mid'"):
        robot.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"mid/ee": np.zeros(7)}))
    with pytest.raises(SafetyError, match="expected '<part>/<name>'"):
        robot.send_action(Action(mode=ControlMode.CARTESIAN, ee_targets={"ee": np.zeros(7)}))
    with pytest.raises(SafetyError):  # the split: 5 joints
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(4)))
    with pytest.raises(SafetyError, match="floating-base"):
        robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3)))
    assert not left.sent and not right.sent


def test_action_modes_are_the_intersection():
    left, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    robot = RobotAssembly(None, parts={"left": left, "g1": wbc})
    assert robot.action_modes == (ControlMode.CARTESIAN,)  # the only mode both take
    assert robot.direct_joints == ("g1/finger",)


# --------------------------------------------------------------- errors


def test_send_failure_holds_the_others_and_refuses_until_reset():
    robot, left, right = _pair()
    right.fail_send = True
    with pytest.raises(ConnectionError):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))
    assert left.held == 1 and right.held == 0
    assert "right" in robot.fault and "right" in robot.get_status().fault
    right.fail_send = False
    with pytest.raises(RobotFaultError):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))
    robot.reset()
    assert robot.fault is None and left.resets == right.resets == 1
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(5)))


def test_observation_failure_also_stops_the_assembly():
    robot, left, right = _pair()
    left.fail_obs = True
    with pytest.raises(ConnectionError):
        robot.get_observation()
    assert right.held == 1 and robot.fault is not None


def test_part_fault_between_sends_stops_the_assembly():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    robot = RobotAssembly(None, parts={"arm": arm, "g1": wbc})
    robot.connect()
    wbc.damp_on_error("SafetyError on the joint command")  # its own thread hit the inner gate
    assert robot.get_status().fault.startswith("part 'g1'")
    with pytest.raises(RobotFaultError, match="g1"):
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.zeros(robot.embodiment.num_dofs)))
    assert arm.held == 1 and not arm.sent


def test_fault_state_refuses_on_a_single_robot_too():
    robot = Recorder(_arm("a", 2))
    robot._set_fault("estop")
    assert robot.get_status().fault == "estop"
    with pytest.raises(RobotFaultError, match="estop"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))
    robot.reset()
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))


# ------------------------------------------------------- wbc and skills


def test_wbc_part_thread_still_commands_its_inner_robot():
    body = ToyRobot(_humanoid())
    wbc = FakeWbc(body)
    robot = RobotAssembly(None, parts={"g1": wbc, "arm": Recorder(_arm("a", 2))})
    robot.connect()
    t = threading.Thread(target=wbc.tick)  # the controller thread, outside any assembly call
    t.start()
    t.join()
    assert body._last_mode == ControlMode.PD  # the inner robot carries no assembly mark


def test_skills_are_prefixed_and_routed():
    wbc = FakeWbc(ToyRobot(_humanoid()))
    robot = RobotAssembly(None, parts={"g1": wbc, "arm": Recorder(_arm("a", 2))})
    assert [s.name for s in robot.list_robot_skills()] == ["g1/wave"]
    robot.run_robot_skill("g1/wave")
    assert wbc.received[-1] == ("skill", "wave")


# ----------------------------------------------------------- floating base


def test_single_floating_part_keeps_top_level_base_and_takes_base_commands():
    wbc = FakeWbc(ToyRobot(_humanoid()))
    arm = Recorder(_arm("a", 2))
    robot = RobotAssembly(None, parts={"g1": wbc, "arm": arm})
    robot.connect()
    assert robot.embodiment.floating_base and robot.embodiment.kind.value == "humanoid"
    obs = robot.get_observation()
    assert obs.base_position is not None and "g1/base_position" not in obs.extra
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.2, 0, 0]),
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
    robot = RobotAssembly(None, parts={"left": left, "right": right})
    status = robot.get_status()
    assert np.allclose(status.joint_temperature[:2], [30, 31]) and np.isnan(status.joint_temperature[2])
    assert status.joint_faults == {"left/j1": 7} and status.battery.soc == 80.0
    assert status.command is None


def test_command_progress_is_top_level_for_one_part_else_in_extra():
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 1))
    walk = CommandProgress(mode=ControlMode.BASE_POSE, done=False, goal=[1.0, 0.0, 0.0])
    left.get_status = lambda: RobotStatus(command=walk)
    robot = RobotAssembly(None, parts={"left": left, "right": right})
    assert robot.get_status().command == walk
    clip = CommandProgress(mode=ControlMode.MOTION_CLIP, done=True, clip_time=2.0,
                           clip_duration=2.0)
    right.get_status = lambda: RobotStatus(command=clip)
    status = robot.get_status()
    assert status.command is None
    assert status.extra["left/command"] == walk and status.extra["right/command"] == clip


# ------------------------------------------------------------------ config


@pytest.mark.parametrize("name", ["robots/franka_ur5e_dual", "robots/ur5e_dual"])
def test_builtin_multi_robot_configs_run(name):
    robot = make_robot(name)
    assert isinstance(robot, RobotAssembly)
    sent = []
    for part in robot.parts.values():  # count what reaches each part
        inner = part._send_action
        part._send_action = lambda a, inner=inner: (sent.append(a), inner(a))
    check_robot_behaviour(robot, lambda: len(sent), label=name)


def test_franka_ur5e_dual_names_and_placement():
    robot = make_robot("franka_ur5e_dual")
    emb = robot.embodiment
    assert [e.name for e in emb.end_effectors] == ["left/ee", "right/tcp"]
    assert [c.name for c in emb.cameras] == ["scene"]
    assert emb.num_dofs == 13 and emb.control_hz == 30
    assert set(emb.parts) == {"", "left", "right"}
    assert np.allclose(robot._node.fixed["left"], [0, 0.4, 0, 0, 0, 0, 1])
    assert np.allclose(robot._node.fixed["right"], [0, -0.4, 0, 0, 0, 0, 1])


def test_three_level_example_splits_nests_and_leases_per_part():
    """examples/lab_two_cells.yaml: a lab of two cells of two arms, all on toy drivers."""
    robot = make_robot("examples/lab_two_cells.yaml")
    emb = robot.embodiment
    # Names carry the whole path; each cell is 2 x 6-dof UR5e.
    assert emb.num_dofs == 24
    assert emb.joint_names[0] == "cell_a/left/shoulder_pan"
    assert {"cell_a", "cell_a/left", "cell_b/right"} <= set(emb.parts)
    assert emb.parts[""].children == ["cell_a", "cell_b"]

    robot.connect()
    try:
        # Split: a flat joint target over all 24 joints reaches each leaf arm's six.
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(24)))
        # Nested parts action routes to one deep leaf; the rest idle.
        robot.send_action(Action(parts={"cell_b": Action(parts={
            "right": Action(mode=ControlMode.POSITION, joint_target=np.full(6, 0.05))})}))
        q = robot.get_observation().joint_pos
        names = list(emb.joint_names)
        assert np.allclose(q[names.index("cell_b/right/shoulder_pan")], 0.05)

        # Per-part leases: a whole cell, and a single arm two levels down.
        with robot.claim("cell_runner", parts=["cell_a"]) as cell:
            with pytest.raises(RobotBusyError):
                robot.claim("clash", parts=["cell_a/left"])  # overlaps the held cell
            arm = robot.claim("arm_runner", parts=["cell_b/left"])  # disjoint: allowed
            arm.release()
            cell.send_action(Action(parts={"cell_a": Action(parts={
                "left": Action(mode=ControlMode.POSITION, joint_target=np.zeros(6))})}))
    finally:
        robot.disconnect()


# ---------------------------------------------------------- Action.parts


def _mixed():
    arm, body = Recorder(_arm("a", 2)), ToyRobot(_humanoid())
    wbc = FakeWbc(body)
    robot = RobotAssembly(None, parts={"left": arm, "g1": wbc})
    robot.connect()
    return robot, arm, wbc


def test_parts_action_gives_each_part_its_own_mode():
    robot, arm, wbc = _mixed()
    robot.send_action(Action(parts={
        "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2])),
        "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.2, 0, 0]),
                     joint_names=["finger"], joint_target=np.array([0.5])),
    }))
    assert np.allclose(arm.sent[-1].joint_target, [0.1, 0.2])
    sent = wbc.received[-1]
    assert sent.mode == ControlMode.BASE_VELOCITY and sent.joint_names == ["finger"]


def test_parts_action_is_checked_per_part_before_sending():
    robot, arm, wbc = _mixed()
    with pytest.raises(SafetyError, match="part 'g1'"):
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)),
            # leg0 belongs to the controller, not free to command
            "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3),
                         joint_names=["leg0"], joint_target=np.zeros(1)),
        }))
    assert not arm.sent and not wbc.received
    with pytest.raises(SafetyError, match="does not take mode"):
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3))}))
    with pytest.raises(SafetyError, match="no part"):
        robot.send_action(Action(parts={"mid": Action(joint_target=np.zeros(2))}))


def test_parts_action_rules():
    with pytest.raises(ValueError, match="no other command fields"):
        Action(parts={"left": Action()}, joint_target=np.zeros(2))
    from hirobot import SafetyGate

    robot, arm, _ = _mixed()
    robot._set_safety(SafetyGate.from_embodiment(robot.embodiment))  # as make_robot does
    with pytest.raises(SafetyError, match="non-finite"):  # the root gate
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.array([np.nan, 0]))}))
    with pytest.raises(SafetyError, match="has no parts"):  # a robot without parts refuses it
        arm.send_action(Action(parts={"x": Action()}))
    assert robot.accepts_parts and not arm.accepts_parts
    with pytest.raises(ValueError, match="now called Action.parts"):
        Action(members={"left": Action()})
    from hirobot import ActionChunk

    with pytest.raises(ValueError, match="now called ActionChunk.parts"):
        ActionChunk(members={"left": ActionChunk(joint_targets=np.zeros((2, 2)))})


def test_parts_chunk_steps_into_parts_actions():
    from hirobot import ActionChunk

    chunk = ActionChunk(parts={
        "left": ActionChunk(mode=ControlMode.POSITION, joint_targets=np.arange(8.0).reshape(4, 2)),
        "g1": ActionChunk(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros((4, 3)))})
    assert chunk.horizon == 4
    robot, arm, wbc = _mixed()
    robot.send_action(chunk.step(2))
    assert np.allclose(arm.sent[-1].joint_target, [4, 5])
    with pytest.raises(ValueError, match="different horizons"):
        ActionChunk(parts={"a": ActionChunk(joint_targets=np.zeros((2, 2))),
                             "b": ActionChunk(joint_targets=np.zeros((3, 2)))})


def test_root_gate_refuses_a_parts_leaf_and_sends_nothing():
    from hirobot import SafetyGate

    robot, arm, wbc = _mixed()
    n = robot.embodiment.num_dofs
    robot._set_safety(SafetyGate.from_embodiment(robot.embodiment, joint_limit=[1.0] * n,
                                                 max_joint_step=0.5))
    with pytest.raises(SafetyError, match="'left/j1'"):
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0.0, 2.0])),
            "g1": Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3))}))
    with pytest.raises(SafetyError, match="max_joint_step"):  # 0.9 rad from the measured 0
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0.9, 0.0]))}))
    assert not arm.sent and not wbc.received
    robot.send_action(Action(parts={
        "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0.4, 0.0]))}))
    assert np.allclose(arm.sent[-1].joint_target, [0.4, 0.0])


def _cell():
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 3, ee="tcp"))
    cell = RobotAssembly(None, parts={"left": left, "right": right},
                         placement={"left": {"pose": [0, 0.4, 0, 0, 0, 0, 1]}})
    extra = Recorder(_arm("c", 1, ee="grip"))
    root = RobotAssembly(None, parts={"cell": cell, "solo": extra},
                         cameras={"scene": ArrayCamera("scene")},
                         placement={"cell": {"pose": [1, 0, 0, 0, 0, 0, 1]}})
    root.connect()
    return root, cell, left, right, extra


def test_nested_assembly_names_parts_and_routes_to_the_leaf():
    root, cell, left, right, extra = _cell()
    emb = root.embodiment
    assert emb.joint_names == ["cell/left/j0", "cell/left/j1", "cell/right/j0", "cell/right/j1",
                               "cell/right/j2", "solo/j0"]
    assert set(emb.parts) == {"", "cell", "cell/left", "cell/right", "solo"}
    assert emb.parts["cell/right"].joints == (2, 5) and emb.parts[""].children == ["cell", "solo"]
    assert root.part("cell/left").embodiment is left.embodiment
    obs = root.get_observation()
    assert np.allclose(obs.ee_poses["cell/left/ee"][:3], [1, 0.4, 0])  # both placements
    assert "scene" in obs.images and obs.joint_pos.shape == (6,)
    root.send_action(Action(parts={"cell": Action(parts={
        "left": Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2]))})}))
    assert np.allclose(left.sent[-1].joint_target, [0.1, 0.2])
    assert not right.sent and right.idle_steps == 1 and extra.idle_steps == 1
    root.send_action(Action(mode=ControlMode.POSITION, joint_target=np.arange(6.0)))
    assert np.allclose(right.sent[-1].joint_target, [2, 3, 4])
    assert np.allclose(extra.sent[-1].joint_target, [5])
    with pytest.raises(SafetyError, match="has no parts"):
        root.send_action(Action(parts={"solo": Action(parts={"x": Action()})}))


def test_nested_root_gate_checks_deep_leaves():
    from hirobot import SafetyGate

    root, cell, left, right, extra = _cell()
    root._set_safety(SafetyGate.from_embodiment(root.embodiment,
                                                joint_limit=[1.0] * root.embodiment.num_dofs))
    with pytest.raises(SafetyError, match="'cell/right/j2'"):
        root.send_action(Action(parts={
            "cell": Action(parts={"right": Action(mode=ControlMode.POSITION,
                                                  joint_target=np.array([0, 0, 3.0]))}),
            "solo": Action(mode=ControlMode.POSITION, joint_target=np.zeros(1))}))
    assert not left.sent and not right.sent and not extra.sent


def test_wbc_fault_between_sends_holds_the_arm():
    robot, arm, wbc = _mixed()
    wbc.damp_on_error("SafetyError in its own thread")
    with pytest.raises(RobotFaultError, match="damped"):
        robot.send_action(Action(parts={
            "left": Action(mode=ControlMode.POSITION, joint_target=np.zeros(2))}))
    assert arm.held == 1 and not arm.sent
    assert "damped" in robot.get_status().fault


# ---------------------------------------------------------------- frames

YAW90 = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]


def test_placement_puts_ee_poses_and_targets_in_the_node_frame():
    left, right = Recorder(_arm("a", 2)), Recorder(_arm("b", 3, ee="tcp"))
    robot = RobotAssembly(None, parts={"left": left, "right": right},
                          placement={"left": {"pose": [0, 0.4, 0, 0, 0, 0, 1]},
                                     "right": {"pose": [1, 0, 0, *YAW90]}})
    robot.connect()
    obs = robot.get_observation()  # each part reports its ee at its own base origin
    assert np.allclose(obs.ee_poses["left/ee"][:3], [0, 0.4, 0])
    assert np.allclose(obs.ee_poses["right/tcp"], [1, 0, 0, *YAW90])
    # A world target 0.5 m along +y from the right base is +x in the right arm's frame.
    robot.send_action(Action(mode=ControlMode.CARTESIAN,
                             ee_targets={"right/tcp": np.array([1, 0.5, 0, *YAW90])}))
    assert np.allclose(right.sent[-1].ee_targets["tcp"], [0.5, 0, 0, 0, 0, 0, 1])


def test_cartesian_delta_is_rotated_into_the_part_frame():
    right = Recorder(_arm("b", 3, ee="tcp"))
    right.action_modes = (ControlMode.CARTESIAN_DELTA,)
    robot = RobotAssembly(None, parts={"right": right},
                          placement={"right": {"pose": [5, 5, 0, *YAW90]}})
    robot.send_action(Action(mode=ControlMode.CARTESIAN_DELTA,
                             ee_targets={"right/tcp": np.array([0, 0.1, 0, 0, 0, 0.2])}))
    # +y in the world is +x for a base turned 90 deg; the z rotation is unchanged.
    assert np.allclose(right.sent[-1].ee_targets["tcp"], [0.1, 0, 0, 0, 0, 0.2])


def test_floating_part_frame_follows_its_odometry():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    robot = RobotAssembly(None, parts={"left": arm, "g1": wbc},
                          placement={"g1": {"origin": [10, 0, 0, 0, 0, 0, 1]}})
    wbc.odom_position, wbc.odom_yaw = np.array([1.0, 0, 0]), np.pi / 2
    obs = robot.get_observation()
    # hand 0.3 m ahead of a pelvis at world x=11 facing +y
    assert np.allclose(obs.ee_poses["g1/left_hand"][:3], [11, 0.3, 0])
    robot.send_action(Action(parts={"g1": Action(
        mode=ControlMode.CARTESIAN, ee_targets={"left_hand": np.array([11, 0.3, 0, *YAW90])})}))
    assert np.allclose(wbc.received[-1].ee_targets["left_hand"], [0.3, 0, 0, 0, 0, 0, 1])
    # body-frame targets pass untouched with frame: base
    robot.send_action(Action(parts={"g1": Action(
        mode=ControlMode.CARTESIAN, ee_targets={"left_hand": np.array([0.2, 0, 0, 0, 0, 0, 1])},
        extra={"frame": "base"})}))
    assert np.allclose(wbc.received[-1].ee_targets["left_hand"][:3], [0.2, 0, 0])


def test_floating_part_without_odometry_has_no_node_frame():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    wbc.odom_position = None
    robot = RobotAssembly(None, parts={"left": arm, "g1": wbc},
                          placement={"g1": {"origin": [0, 0, 0, 0, 0, 0, 1]}})
    obs = robot.get_observation()
    assert "g1/left_hand" not in obs.ee_poses and "g1/ee_poses" in obs.extra
    assert "g1/left_hand" not in robot.cartesian_targets
    with pytest.raises(SafetyError, match="no base_position"):
        robot.send_action(Action(parts={"g1": Action(
            mode=ControlMode.CARTESIAN,
            ee_targets={"left_hand": np.array([0, 0, 0, 0, 0, 0, 1.0])})}))


def test_floating_part_without_placement_has_its_origin_at_the_node_origin():
    arm, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    robot = RobotAssembly(None, parts={"left": arm, "g1": wbc})
    assert np.allclose(robot._node.origin["g1"], [0, 0, 0, 0, 0, 0, 1])
    wbc.odom_position, wbc.odom_yaw = np.array([2.0, 0, 0]), 0.0
    assert np.allclose(robot.get_observation().ee_poses["g1/left_hand"][:3], [2.3, 0, 0])
    assert "g1/left_hand" in robot.cartesian_targets
    wbc.odom_position = None  # odometry lost: no node frame for its end effectors
    obs = robot.get_observation()
    assert "g1/left_hand" not in obs.ee_poses and "g1/ee_poses" in obs.extra
    assert "g1/left_hand" not in robot.cartesian_targets


def test_placement_errors():
    a, wbc = Recorder(_arm("a", 2)), FakeWbc(ToyRobot(_humanoid()))
    with pytest.raises(ValueError, match="pose must be a finite"):
        RobotAssembly(None, parts={"left": a}, placement={"left": {"pose": [0, 0, 0]}})
    with pytest.raises(ValueError, match="no child part 'mid'"):
        RobotAssembly(None, parts={"left": a}, placement={"mid": {"pose": [0] * 6 + [1]}})
    with pytest.raises(ValueError, match="origin: is for a part that moves"):
        RobotAssembly(None, parts={"left": a}, placement={"left": {"origin": [0] * 6 + [1]}})
    with pytest.raises(ValueError, match="moves by itself"):
        RobotAssembly(None, parts={"g1": wbc}, placement={"g1": {"pose": [0] * 6 + [1]}})
    with pytest.raises(ValueError, match="quaternion is zero"):
        RobotAssembly(None, parts={"left": a}, placement={"left": {"pose": [0] * 7}})
    with pytest.raises(ValueError, match="must be non-empty"):
        RobotAssembly(None, parts={"le/ft": a})
