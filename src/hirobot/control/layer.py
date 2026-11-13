"""Base class for layers: a :class:`~hirobot.core.robot.Robot` that wraps another one.

A layer sits between the caller and the assembled robot (``layers:`` in a
config, first entry innermost). It sees the robot below as :attr:`Layer.inner`
and is itself a ``Robot``, so callers do not change when a layer is added.
Whole-body control (hiWBC), IK, filters and teleop takeover are layers; hiRobot
ships none, they come from plugin packages (:func:`~hirobot.loading.registry.register_layer`).

:class:`Layer` forwards everything to ``inner`` by default: the capability
declarations (``action_modes``, ``motion_spaces``, ``direct_joints``,
``cartesian_targets``), the devices (``cameras``, ``microphones``,
``speakers``), the lifecycle (``connect``, ``reset``, ``disconnect``),
observations, status, ``hold``, gains and onboard skills. A subclass overrides
only what it changes.

Two kinds of layer:

* **Passive** (the default): each ``send_action`` computes once and sends to
  ``inner``; it runs at the caller's rate. Override :meth:`_send_action`.
* **With its own thread** (whole-body control at 50 Hz, a filter, IK
  smoothing): ``send_action`` only updates a target, and the thread calls
  ``inner.send_action`` at its own rate. Such a layer also overrides
  :meth:`hold` (keep the robot standing when the caller stops), :meth:`_estop`
  (stop its thread and drop the target; the robot below is stopped first by
  :meth:`estop`) and calls :meth:`_set_fault` when its thread fails.

Two rules hold for every layer:

* **E-stop reaches the hardware first.** :meth:`Layer.estop` faults every layer
  of the stack, e-stops the robot at the bottom (the assembly or driver), and
  only then runs each layer's own :meth:`_estop`, outermost first, each with
  :data:`ESTOP_LAYER_TIMEOUT_S`. A layer that raises or hangs there is logged
  and skipped, so it cannot hold the stop back. A layer overrides
  :meth:`_estop` to stop itself only; it never calls ``inner.estop()``.
* **Layers use the public API of ``inner``.** A layer is trusted code, like a
  kernel module: the gate below it holds only while it commands through
  ``inner.send_action`` and the other public methods. :attr:`Layer.inner` is a
  proxy that refuses ``_``-prefixed names (``_send_action``, ``_set_safety``,
  ...) and the parts of an assembly (``body``, ``end_effectors``). This catches
  mistakes, not deliberate bypasses. :func:`hirobot.testing.assert_layer_uses_public_api`
  checks a layer class.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, List, Mapping, Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.core.robot import Robot
from hirobot.core.robot_skills import RobotSkillError, RobotSkillResult, RobotSkillSpec, validate_args
from hirobot.core.types import Action, Observation, RobotStatus

logger = logging.getLogger(__name__)

#: Seconds :meth:`Layer.estop` gives each layer's own ``_estop`` before it
#: logs an error and moves on to the next layer.
ESTOP_LAYER_TIMEOUT_S = 0.2

#: Attributes of the robot below that a layer may not reach, besides every
#: ``_``-prefixed name: the parts an assembly is built from.
PRIVATE_PARTS = ("body", "end_effectors")


class InnerProxy:
    """The robot below a layer, with only its public API.

    Attribute reads go to the robot, except ``_``-prefixed names and
    :data:`PRIVATE_PARTS`, which raise ``AttributeError``. ``isinstance`` sees
    the robot's class. Every name read is recorded in :attr:`accessed` (a set),
    so a test can see what a layer used.
    """

    __slots__ = ("_target", "_accessed", "_denied")

    def __init__(self, target: Robot) -> None:
        object.__setattr__(self, "_target", target)
        object.__setattr__(self, "_accessed", set())
        object.__setattr__(self, "_denied", set())

    @property
    def __class__(self):  # type: ignore[override]
        return type(object.__getattribute__(self, "_target"))

    @property
    def accessed(self) -> set:
        """Public names read through this proxy."""
        return object.__getattribute__(self, "_accessed")

    @property
    def denied(self) -> set:
        """Names this proxy refused."""
        return object.__getattribute__(self, "_denied")

    def __getattr__(self, name: str) -> Any:
        target = object.__getattribute__(self, "_target")
        if name.startswith("_") or name in PRIVATE_PARTS:
            object.__getattribute__(self, "_denied").add(name)
            raise AttributeError(
                f"a layer may only use the public API of the robot below it; "
                f"{type(target).__name__}.{name} is not part of it (commands must "
                "go through inner.send_action so the safety gate below checks them)")
        object.__getattribute__(self, "_accessed").add(name)
        return getattr(target, name)

    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError(f"a layer may not set attributes on the robot below it ({name!r})")

    def __repr__(self) -> str:
        return f"InnerProxy({object.__getattribute__(self, '_target')!r})"


def _unwrap(robot: Any) -> Robot:
    if type(robot) is InnerProxy:  # type() ignores the proxy's __class__
        return object.__getattribute__(robot, "_target")
    return robot


class Layer(Robot):
    """A :class:`Robot` that wraps ``inner`` and forwards to it by default.

    Args:
        inner: The robot underneath: the assembled robot, or the next layer in.
    """

    #: The stack's :class:`~hirobot.control.arbiter.CommandArbiter`, set by
    #: :func:`~hirobot.make_robot`. A layer skill claims it with ``LAYER_SKILL``.
    _stack_arbiter: Optional[Any] = None

    #: For a layer on one part (``layers: [{part: ...}]``): the part paths its
    #: layer skills claim (``None``: the whole robot).
    _skill_parts: Optional[List[str]] = None

    #: Called with ``(layer, reason)`` when this layer enters the fault state
    #: (not on an e-stop); a layer on one part tells the root so (see
    #: :mod:`hirobot.assembly.part_layers`).
    _fault_listener: Optional[Any] = None

    def __init__(self, inner: Robot) -> None:
        self.inner = inner

    @property
    def inner(self) -> Robot:
        """The robot below, through an :class:`InnerProxy` (public API only)."""
        return self._inner_proxy

    @inner.setter
    def inner(self, robot: Robot) -> None:
        raw = _unwrap(robot)
        self._inner = raw
        self._inner_proxy = robot if raw is not robot else InnerProxy(raw)

    # ----------------------------------------------------------- declarations
    # Robot declares these as class attributes, so a subclass that does not
    # override them must still see the inner robot's values: forward them here.

    @property
    def embodiment(self) -> Embodiment:
        return self.inner.embodiment

    @property
    def action_modes(self):  # type: ignore[override]
        return self.inner.action_modes

    @property
    def motion_spaces(self):  # type: ignore[override]
        return self.inner.motion_spaces

    @property
    def direct_joints(self):  # type: ignore[override]
        return self.inner.direct_joints

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return self.inner.cartesian_targets

    @property
    def simulated(self) -> bool:  # type: ignore[override]
        return bool(self.inner.simulated)

    @property
    def reward(self):  # type: ignore[override]
        return self.inner.reward

    @property
    def success(self):  # type: ignore[override]
        return self.inner.success

    @property
    def done(self):  # type: ignore[override]
        return self.inner.done

    @property
    def fault(self):  # type: ignore[override]
        """This layer's fault, else the one below it (a driver that damped itself)."""
        return self._fault or self.inner.fault

    def _set_fault(self, reason: str) -> None:
        super()._set_fault(reason)
        listener = self._fault_listener
        if listener is not None:
            try:
                listener(self, self._fault)
            except Exception:  # noqa: BLE001 - the layer is in the fault state regardless
                logger.exception("%s: reporting the fault failed", type(self).__name__)

    def _part_layers(self) -> List["Layer"]:
        """Layers on parts of the robot below this one (:mod:`hirobot.assembly.part_layers`)."""
        return []

    # ----------------------------------------------------------------- devices

    @property
    def cameras(self) -> Mapping[str, Any]:  # type: ignore[override]
        return self.inner.cameras

    @property
    def microphones(self) -> Mapping[str, Any]:  # type: ignore[override]
        return self.inner.microphones

    @property
    def speakers(self) -> Mapping[str, Any]:  # type: ignore[override]
        return self.inner.speakers

    # --------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        self.inner.connect()

    def reset(self, seed: Optional[int] = None) -> None:
        self.inner.reset(seed)

    def disconnect(self) -> None:
        self.inner.disconnect()

    def now(self) -> float:
        return self.inner.now()

    # ---------------------------------------------------------------- sensing

    def get_observation(self) -> Observation:
        return self.inner.get_observation()

    def get_status(self) -> RobotStatus:
        status = self.inner.get_status()
        status.fault = self.fault
        return status

    # --------------------------------------------------------------- commands

    def _send_action(self, action: Action) -> None:
        """Pass the action down; ``inner.send_action`` runs the gate below."""
        self.inner.send_action(action)

    def hold(self) -> None:
        self.inner.hold()

    def _idle_step(self) -> None:
        self.inner._idle_step()

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        self.inner.set_gains(kp, kd)

    # --------------------------------------------------------------- e-stop

    def estop(self) -> None:
        """Stop the robot now: the hardware first, then each layer.

        1. Every layer from this one down enters the fault state, and its long
           calls give up (:class:`~hirobot.core.robot.RobotStoppedError`).
        2. The robot at the bottom of the stack (the assembly or driver) is
           e-stopped directly, without going through the layers.
        3. Each layer's :meth:`_estop` runs, outermost first. One that raises,
           or takes longer than :data:`ESTOP_LAYER_TIMEOUT_S`, is logged and
           the next one runs.

        An error from step 2 is raised after step 3.
        """
        layers: List[Layer] = []
        robot: Robot = self
        while isinstance(robot, Layer):
            layers.append(robot)
            robot = robot._inner
        # Layers on parts sit under the others; they stop last.
        layers += [p for layer in layers for p in layer._part_layers()]
        for layer in layers:
            layer._bump_stop()
            if layer._fault is None:
                Robot._set_fault(layer, "estop()")  # no fault listener: everything stops
        error: Optional[BaseException] = None
        try:
            robot.estop()
        except Exception as exc:  # noqa: BLE001 - the layers still stop themselves
            logger.exception("estop on %s failed", type(robot).__name__)
            error = exc
        for layer in layers:
            layer._run_own_estop()
            layer._end_layer_skill()  # the caller must be able to reset()
        if error is not None:
            raise error

    def _run_own_estop(self) -> None:
        if type(self)._estop is Layer._estop:
            return
        name = type(self).__name__
        failure: List[BaseException] = []

        def run() -> None:
            try:
                self._estop()
            except BaseException as exc:  # noqa: BLE001 - reported below
                failure.append(exc)

        worker = threading.Thread(target=run, name=f"estop-{name}", daemon=True)
        worker.start()
        worker.join(ESTOP_LAYER_TIMEOUT_S)
        if worker.is_alive():
            logger.error("%s._estop() did not return within %.2f s; e-stop went on without it",
                         name, ESTOP_LAYER_TIMEOUT_S)
        elif failure:
            logger.error("%s._estop() failed: %r", name, failure[0])

    def _estop(self) -> None:
        """Stop this layer only: its thread, its targets. Does nothing by default.

        The robot below is already stopped when this runs (:meth:`estop`). Do
        not call ``inner.estop()`` here.
        """

    # ----------------------------------------------------------- onboard skills

    @property
    def has_onboard_skills(self) -> bool:
        return self.inner.has_onboard_skills

    def list_onboard_skills(self) -> List[RobotSkillSpec]:
        return self.inner.list_onboard_skills()

    def run_onboard_skill(
        self, name: str, args: Optional[Mapping[str, Any]] = None
    ) -> RobotSkillResult:
        return self.inner.run_onboard_skill(name, args)

    def stop_onboard_skill(self) -> None:
        self.inner.stop_onboard_skill()

    # ----------------------------------------------------------- layer skills
    #
    # A layer skill is run by the layer itself over many control cycles (a
    # behavior, a macro). While it runs the layer holds the stack's command
    # stream with Priority.LAYER_SKILL, so a caller's send_action raises
    # RobotBusyError; stop_layer_skill() ends it. A subclass declares its
    # skills in _layer_skills(), starts one in _start_layer_skill(), stops it
    # in _stop_layer_skill(), and calls _layer_skill_finished() when one ends
    # on its own.

    def list_layer_skills(self) -> List[RobotSkillSpec]:
        """Skills this layer and the layers below it run (not the vendor's onboard skills)."""
        return list(self._layer_skills()) + list(self.inner.list_layer_skills())

    def run_layer_skill(
        self, name: str, args: Optional[Mapping[str, Any]] = None
    ) -> RobotSkillResult:
        """Start layer skill ``name``; returns once it has started.

        The layer that owns the skill claims the stack's command stream with
        :attr:`~hirobot.control.arbiter.Priority.LAYER_SKILL` first: a caller's lease
        is preempted (it holds the robot, its ``on_preempt`` runs; claim again
        after the skill), and direct commands raise
        :class:`~hirobot.control.arbiter.RobotBusyError` until the skill ends or
        :meth:`stop_layer_skill` is called. Raises ``RobotBusyError`` while
        another layer skill runs or a ``TAKEOVER`` holds the robot, and
        :class:`~hirobot.core.robot_skills.RobotSkillError` for an unknown skill or
        bad arguments.
        """
        specs = {s.name: s for s in self._layer_skills()}
        if name not in specs:
            if any(s.name == name for s in self.inner.list_layer_skills()):
                return self.inner.run_layer_skill(name, args)
            raise RobotSkillError(f"{type(self).__name__}: no layer skill {name!r}; "
                                  f"available: {[s.name for s in self.list_layer_skills()]}")
        spec = specs[name]
        checked = validate_args(spec, args)
        if self.fault is not None:
            raise RobotSkillError(f"{type(self).__name__} is in a fault state ({self.fault}); "
                                  "reset() first")
        lease = self._layer_skill_arbiter().claim(
            f"{type(self).__name__}:{name}", _priority().LAYER_SKILL,
            on_preempt=self._on_layer_skill_preempted, parts=self._skill_parts)
        with self._layer_skill_lock:
            self._layer_skill_lease = lease
        try:
            with lease._admitted("run_layer_skill"):  # self.hold() etc. pass in the hook
                self._start_layer_skill(spec, checked)
        except BaseException:
            self._end_layer_skill()
            raise
        return RobotSkillResult(name=name, args=checked, duration_s=spec.duration_s)

    def stop_layer_skill(self) -> None:
        """Stop the running layer skill (this layer's or one below) and give the stream back."""
        lease = self._layer_skill_lease
        if lease is not None:
            try:
                with lease._admitted("stop_layer_skill"):
                    self._stop_layer_skill()
            finally:
                self._end_layer_skill()
        self.inner.stop_layer_skill()

    @property
    def layer_skill_running(self) -> bool:
        """Whether a skill of this layer or of a layer below holds the command stream."""
        return self._layer_skill_lease is not None or self.inner.layer_skill_running

    def _layer_skills(self) -> List[RobotSkillSpec]:
        """This layer's own skills. Default: none."""
        return []

    def _start_layer_skill(self, spec: RobotSkillSpec, args: Mapping[str, Any]) -> None:
        """Start ``spec`` (``args`` already checked) and return; it runs on the layer's thread."""
        raise RobotSkillError(f"{type(self).__name__} has no layer skills")

    def _stop_layer_skill(self) -> None:
        """Stop the running skill and leave the robot standing still. Default: nothing."""

    def _layer_skill_finished(self) -> None:
        """Call when a skill ends on its own: gives the command stream back."""
        self._end_layer_skill()

    _layer_skill_lease: Optional[Any] = None

    @property
    def _layer_skill_lock(self) -> threading.Lock:
        lock = self.__dict__.get("_layer_skill_lock_")
        if lock is None:
            lock = self.__dict__.setdefault("_layer_skill_lock_", threading.Lock())
        return lock

    def _end_layer_skill(self) -> None:
        with self._layer_skill_lock:
            lease, self._layer_skill_lease = self._layer_skill_lease, None
        if lease is not None:
            lease.release()

    def _on_layer_skill_preempted(self, lease: Any) -> None:
        # A takeover: stop the skill; the arbiter then holds the robot.
        with self._layer_skill_lock:
            if self._layer_skill_lease is lease:
                self._layer_skill_lease = None
        try:
            self._stop_layer_skill()
        except Exception:  # noqa: BLE001 - the takeover goes ahead
            logger.exception("%s: stopping the layer skill on a takeover failed",
                             type(self).__name__)

    def _layer_skill_arbiter(self) -> Any:
        """The stack's arbiter; a layer built by hand (no ``make_robot``) uses its own."""
        if self._stack_arbiter is not None:
            return self._stack_arbiter
        if self._arbiter is None:
            from hirobot.control.arbiter import CommandArbiter

            self._arbiter = CommandArbiter(self)
        return self._arbiter


def _priority() -> Any:
    from hirobot.control.arbiter import Priority

    return Priority


__all__ = ["ESTOP_LAYER_TIMEOUT_S", "InnerProxy", "Layer"]
