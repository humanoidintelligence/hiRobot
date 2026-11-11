"""Layer skills: run by a layer, holding the command stream with LAYER_SKILL."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from hirobot import (
    Action,
    ControlMode,
    Embodiment,
    Layer,
    PreemptedError,
    Priority,
    RobotBusyError,
)
from hirobot.control.arbiter import CommandArbiter
from hirobot.drivers.sim.toy import ToyRobot
from hirobot.core.robot_skills import RobotSkillError, RobotSkillSpec

WAVE = RobotSkillSpec(name="wave", description="Wave.",
                      parameters={"type": "object",
                                  "properties": {"times": {"type": "integer", "minimum": 1,
                                                           "default": 2}}})


def _pos(v=0.1):
    return Action(mode=ControlMode.POSITION, joint_target=np.full(3, v))


class Waver(Layer):
    """Runs ``wave`` on its own thread until stopped (or ``finish`` is set)."""

    def __init__(self, inner):
        super().__init__(inner)
        self.started, self.stopped, self.finish = [], 0, threading.Event()
        self._thread = None

    def _send_action(self, action):
        self.inner.send_action(action)

    def _layer_skills(self):
        return [WAVE]

    def _start_layer_skill(self, spec, args):
        self.started.append((spec.name, dict(args)))
        self.finish.clear()

        def run():
            # The layer commands inner itself; that is below the arbiter.
            self.inner.send_action(_pos(0.5))
            if self.finish.wait(2.0):
                self._layer_skill_finished()

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()

    def _stop_layer_skill(self):
        self.stopped += 1
        self.finish.set()


class Outer(Layer):
    """A plain layer above the waver: layer skills below it show through."""


def _stack():
    toy = ToyRobot(Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(3)],
                              control_hz=50))
    waver = Waver(toy)
    outer = Outer(waver)
    arbiter = CommandArbiter(outer)  # what make_robot does
    outer._arbiter = arbiter
    waver._stack_arbiter = outer._stack_arbiter = arbiter
    outer.connect()
    return outer, waver, toy


def test_skills_are_listed_through_the_layers_above():
    outer, waver, _ = _stack()
    assert [s.name for s in outer.list_layer_skills()] == ["wave"]
    assert outer.list_robot_skills() == []  # layer skills are not robot skills


def test_caller_is_refused_while_a_skill_runs_and_can_claim_after_stop():
    outer, waver, toy = _stack()
    result = outer.run_layer_skill("wave")
    assert result.args == {"times": 2} and waver.started == [("wave", {"times": 2})]
    assert waver.layer_skill_running
    with pytest.raises(RobotBusyError, match="Waver:wave"):
        outer.send_action(_pos())
    with pytest.raises(RobotBusyError):
        outer.claim("runner")
    outer.get_observation()  # reads are never checked
    outer.stop_layer_skill()
    assert waver.stopped == 1 and not waver.layer_skill_running
    with outer.claim("runner") as r:
        r.send_action(_pos(0.2))
    outer.send_action(_pos(0.3))


def test_a_skill_that_ends_on_its_own_gives_the_stream_back():
    outer, waver, _ = _stack()
    outer.run_layer_skill("wave")
    waver.finish.set()
    waver._thread.join(1.0)
    assert not waver.layer_skill_running
    outer.send_action(_pos())


def test_a_skill_preempts_the_caller_and_a_takeover_preempts_the_skill():
    outer, waver, _ = _stack()
    runner = outer.claim("runner")
    outer.run_layer_skill("wave", {"times": 3})
    with pytest.raises(PreemptedError, match="Waver:wave"):
        runner.send_action(_pos())
    teleop = outer.claim("teleop", Priority.TAKEOVER)
    assert waver.stopped == 1 and not waver.layer_skill_running
    with pytest.raises(RobotBusyError):
        outer.run_layer_skill("wave")  # a takeover outranks a layer skill
    teleop.send_action(_pos(0.2))
    teleop.release()


def test_bad_arguments_and_unknown_skills_claim_nothing():
    outer, waver, _ = _stack()
    with pytest.raises(RobotSkillError, match="times"):
        outer.run_layer_skill("wave", {"times": 0})
    with pytest.raises(RobotSkillError, match="no layer skill 'dance'"):
        outer.run_layer_skill("dance")
    assert outer._arbiter.holder is None


def test_a_failing_start_gives_the_stream_back():
    outer, waver, _ = _stack()

    def boom(spec, args):
        raise RuntimeError("motor cold")

    waver._start_layer_skill = boom
    with pytest.raises(RuntimeError, match="motor cold"):
        outer.run_layer_skill("wave")
    assert outer._arbiter.holder is None


def test_estop_ends_the_skill_so_the_caller_can_reset():
    outer, waver, toy = _stack()
    outer.run_layer_skill("wave")
    outer.estop()
    assert not waver.layer_skill_running and outer.fault is not None
    outer.reset()
    assert outer.fault is None


def test_a_hand_built_layer_without_make_robot_uses_its_own_arbiter():
    toy = ToyRobot(Embodiment(name="toy", joints=[{"name": f"j{i}"} for i in range(3)],
                              control_hz=50))
    waver = Waver(toy)
    waver.connect()
    waver.run_layer_skill("wave")
    with pytest.raises(RobotBusyError):
        waver.send_action(_pos())
    waver.stop_layer_skill()
    waver.send_action(_pos())
