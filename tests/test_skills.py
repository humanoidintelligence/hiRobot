"""Skills: robot skills and layer skills as one list (list_skills / run_skill / stop_skill)."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, Layer, PreemptedError, RobotBusyError
from hirobot.control.arbiter import CommandArbiter
from hirobot.core.robot_skills import RobotSkillError, RobotSkillSpec
from hirobot.drivers.sim.toy import ToyRobot


def _spec(name, desc="", **props):
    params = {"type": "object", "properties": props} if props else {}
    return RobotSkillSpec(name=name, description=desc or name, parameters=params)


class Vendor(ToyRobot):
    """A driver with robot skills ``sit`` and ``walk``; records what ran and stopped."""

    def __init__(self):
        super().__init__(Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(3)],
                                    control_hz=50))
        self.ran, self.stops = [], 0

    @property
    def has_robot_skills(self):
        return True

    def list_robot_skills(self):
        return [_spec("sit", "vendor sit"),
                _spec("walk", "vendor walk", vx={"type": "number", "maximum": 0.5})]

    def _run_robot_skill(self, spec, args):
        self.ran.append((spec.name, dict(args)))

    def stop_robot_skill(self):
        self.stops += 1


class Walker(Layer):
    """A layer whose own ``walk`` hides the vendor's, plus ``wave``; runs until stopped."""

    def __init__(self, inner):
        super().__init__(inner)
        self.started, self.stops, self.finish = [], 0, threading.Event()

    def _send_action(self, action):
        self.inner.send_action(action)

    def _layer_skills(self):
        return [_spec("walk", "layer walk", vx={"type": "number", "minimum": 0}),
                _spec("wave")]

    def _start_layer_skill(self, spec, args):
        self.started.append((spec.name, dict(args)))
        self.finish.clear()

    def _stop_layer_skill(self):
        self.stops += 1
        self.finish.set()


def _stack():
    vendor = Vendor()
    walker = Walker(vendor)
    arbiter = CommandArbiter(walker)  # what make_robot does
    walker._arbiter = arbiter
    walker._stack_arbiter = arbiter
    walker.connect()
    return walker, vendor


def _pos():
    return Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.1))


def test_one_list_layer_skills_first_and_a_layer_skill_hides_a_robot_skill_of_its_name():
    robot, _ = _stack()
    skills = robot.list_skills()
    assert [s.name for s in skills] == ["walk", "wave", "sit"]
    assert skills[0].description == "layer walk"
    assert all(isinstance(s, RobotSkillSpec) for s in skills)


def test_run_skill_goes_to_whoever_runs_it():
    robot, vendor = _stack()
    result = robot.run_skill("walk", {"vx": 0.8})  # the layer's: no maximum
    assert robot.started == [("walk", {"vx": 0.8})] and vendor.ran == []
    assert robot.skill_running and result.name == "walk"
    robot.stop_skill()
    robot.run_skill("sit")
    assert vendor.ran == [("sit", {})]
    assert not robot.skill_running  # robot skills are not tracked


def test_run_skill_checks_arguments_and_refuses_unknown_names():
    robot, _ = _stack()
    with pytest.raises(RobotSkillError, match="vx"):
        robot.run_skill("walk", {"vx": -1})
    with pytest.raises(RobotSkillError, match=r"unknown skill 'dance'; available: \['walk', 'wave', 'sit'\]"):
        robot.run_skill("dance")
    assert robot._arbiter.holder is None


def test_stop_skill_stops_both_kinds():
    robot, vendor = _stack()
    robot.run_skill("wave")
    robot.stop_skill()
    assert robot.stops == 1 and vendor.stops == 1 and not robot.skill_running
    robot.send_action(_pos())  # the stream is back


def test_a_running_skill_refuses_direct_commands_like_a_layer_skill():
    robot, _ = _stack()
    robot.run_skill("wave")
    with pytest.raises(RobotBusyError, match="Walker:wave"):
        robot.send_action(_pos())
    robot.stop_skill()


def test_through_a_lease_a_robot_skill_runs_and_a_layer_skill_preempts_it():
    robot, vendor = _stack()
    lease = robot.claim("agent")
    lease.run_skill("sit")
    assert vendor.ran == [("sit", {})]
    lease.run_skill("wave")
    assert robot.skill_running
    with pytest.raises(PreemptedError, match="Walker:wave"):
        lease.send_action(_pos())
    robot.stop_skill()
    with pytest.raises(RobotSkillError, match="unknown skill 'dance'"):
        robot.claim("agent2").run_skill("dance")


def test_a_robot_without_skills_has_the_four_methods():
    toy = ToyRobot(Embodiment(name="toy", joints=[{"name": "j0"}], control_hz=50))
    assert toy.list_skills() == [] and toy.skill_running is False
    toy.stop_skill()
    with pytest.raises(RobotSkillError, match="unknown skill 'wave'; available: none"):
        toy.run_skill("wave")


def test_assembly_names_carry_the_part_path():
    from hirobot import RobotAssembly

    robot = RobotAssembly(None, parts={"g1": Vendor(), "arm": ToyRobot(Embodiment(
        name="arm", joints=[{"name": "a0"}], control_hz=50))})
    robot.connect()
    assert [s.name for s in robot.list_skills()] == ["g1/sit", "g1/walk"]
    robot.run_skill("g1/sit")
    assert robot.parts["g1"].ran == [("sit", {})]
