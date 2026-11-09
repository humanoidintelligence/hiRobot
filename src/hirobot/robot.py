"""The unified robot-driver contract.

A :class:`Robot` is the single interface a model or agent talks to in order to
observe and control a humanoid or an arm, regardless of the underlying hardware,
SDK, or simulator. Concrete drivers (real hardware, a simulator, a teleop rig)
implement this contract so that control code written against it is portable
across embodiments *and* across sim/real.

hiRobot is about *this* abstraction — standardizing robot I/O. Model loading
and inference optimization live in a separate serving layer (see hiVLA).
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Iterator, List, Mapping, Optional, Tuple

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.robot_skills import (
    RobotSkillError,
    RobotSkillResult,
    RobotSkillSpec,
    validate_args,
)
from hirobot.safety import SafetyError, SafetyGate
from hirobot.types import Action, ControlMode, Observation, RobotStatus

logger = logging.getLogger(__name__)


class RobotFaultError(RuntimeError):
    """Raised by ``send_action`` while a robot is in a fault state (see :attr:`Robot.fault`).

    A robot enters one when it damps or stops itself (a whole-body controller
    whose loop failed, a robot group that stopped every member after one
    failed). Commands are refused until :meth:`Robot.reset` succeeds.
    """


class RobotStoppedError(RobotFaultError):
    """A long call (``connect``, ``reset``) gave up because ``estop()`` ran."""


#: Seconds ``estop()`` waits for a command already being sent before it
#: sends the stop anyway.
ESTOP_LOCK_TIMEOUT_S = 0.2

_SIGNAL_LOCK = threading.Lock()


class _StopSignal:
    """E-stop bookkeeping shared by one device's threads.

    ``count`` goes up on every e-stop; a long call takes it at the start
    (:meth:`Stoppable._stop_token`) and gives up once it changes. ``lock``
    orders commands against an e-stop: a command already being sent finishes
    first, and none goes out after the stop. ``wake`` (its own lock) guards
    ``count`` and wakes :meth:`Stoppable._wait`.
    """

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.wake = threading.Condition(threading.Lock())
        self.count = 0


class Stoppable:
    """What lets an e-stop from another thread cut a device's long calls short.

    Used by :class:`Robot` and :class:`~hirobot.end_effector.EndEffectorDriver`. A driver
    loop that runs for a while (a reset ramp, a wait for the first state)
    takes a token with :meth:`_stop_token`, sleeps with :meth:`_wait` and
    sends each step inside :meth:`_commanding`; all three raise
    :class:`RobotStoppedError` once an e-stop came after the token.
    """

    @property
    def _estop_signal(self) -> _StopSignal:
        sig = self.__dict__.get("_estop_signal_")
        if sig is None:
            with _SIGNAL_LOCK:
                sig = self.__dict__.setdefault("_estop_signal_", _StopSignal())
        return sig

    def _stop_token(self) -> int:
        """The e-stop count now; pass it to the calls below."""
        sig = self._estop_signal
        with sig.wake:
            return sig.count

    def _bump_stop(self) -> None:
        sig = self._estop_signal
        with sig.wake:
            sig.count += 1
            sig.wake.notify_all()

    def _check_stop(self, token: int) -> None:
        """Raise :class:`RobotStoppedError` if an e-stop came since ``token``."""
        if self._stop_token() != token:
            raise RobotStoppedError(f"{type(self).__name__}: stopped by estop()")

    def _wait(self, seconds: float, token: int) -> None:
        """Sleep ``seconds``, waking at once (with :class:`RobotStoppedError`) on an e-stop."""
        sig = self._estop_signal
        with sig.wake:
            if sig.count == token and seconds > 0:
                sig.wake.wait(seconds)
            stopped = sig.count != token
        if stopped:
            raise RobotStoppedError(f"{type(self).__name__}: stopped by estop()")

    @contextlib.contextmanager
    def _commanding(self, token: Optional[int] = None) -> Iterator[None]:
        """Hold the command lock; with ``token``, refuse if an e-stop came since."""
        with self._estop_signal.lock:
            if token is not None:
                self._check_stop(token)
            yield


def _clears_fault(reset):
    """Wrap a driver's ``reset``: leave the fault state when it returns, unless
    an e-stop came while it ran."""
    if getattr(reset, "_clears_fault", False) or getattr(reset, "__isabstractmethod__", False):
        return reset

    def wrapped(self, *args, **kwargs):
        token = self._stop_token()
        result = reset(self, *args, **kwargs)
        if self._stop_token() == token:
            self.clear_fault()
        return result

    wrapped.__name__, wrapped.__qualname__ = reset.__name__, reset.__qualname__
    wrapped.__doc__, wrapped.__wrapped__ = reset.__doc__, reset
    wrapped._clears_fault = True  # type: ignore[attr-defined]
    return wrapped


class Robot(Stoppable, ABC):
    """Common interface for observing and commanding a robot.

    A driver maps hiRobot's embodiment-aware :class:`~hirobot.types.Observation`
    and :class:`~hirobot.types.Action` schemas onto a specific robot's native
    sensor reads and command API. Control stacks, policies, and agents only ever
    see this interface, so the hardware — or a simulator standing in for it — can
    be swapped without touching callers.

    The surveyed SDKs (Unitree, Fourier GRx, AgiBot X1, Franka, UR) all reduce to
    the same core: connect, read joint (and optionally base/task) state, send a
    joint-or-Cartesian command, and set PD gains. That is exactly this contract.

    Safety is enforced here, not in callers. :meth:`send_action` runs an optional
    :class:`~hirobot.safety.SafetyGate` and then dispatches to the driver's
    :meth:`_send_action`; a driver implements ``_send_action`` and never needs to
    touch the gate. Attach a gate with :meth:`set_safety` (``make_robot`` does
    this from the embodiment and the config's ``safety:`` section), so every path
    that reaches the robot — the control loop, eval scripts, an agent's tools —
    clears the same check and cannot skip it.
    """

    #: The gate applied to every action in ``send_action``; ``None`` disables it.
    safety: Optional[SafetyGate] = None

    #: True for simulator drivers. ``make_robot`` attaches hardware camera
    #: sources only to real drivers, so a config's ``toy`` default runs offline.
    simulated: bool = False

    #: Control modes ``_send_action`` accepts. Callers that build actions
    #: themselves (an agent's tools) read this instead of discovering a mode
    #: is unsupported when the driver raises. Drivers override it.
    action_modes: Tuple[ControlMode, ...] = (
        ControlMode.POSITION,
        ControlMode.PD,
        ControlMode.VELOCITY,
        ControlMode.TORQUE,
    )

    #: Kinds of :class:`~hirobot.types.MotionClip` (``space``) the robot can
    #: track under ``MOTION_CLIP``; empty means it takes no clips.
    motion_spaces: Tuple[str, ...] = ()

    #: Joints the caller may command by name (``Action.joint_names``) alongside
    #: a non-joint mode, because whatever runs the robot leaves them free: the
    #: fingers under a body-only whole-body controller, say. Empty by default.
    direct_joints: Tuple[str, ...] = ()

    #: Why the robot is in a fault state, or ``None``. Set with :meth:`set_fault`.
    _fault: Optional[str] = None

    #: Whether ``send_action`` takes ``Action.members`` (only a robot group does).
    accepts_members: bool = False

    #: The :class:`~hirobot.group.RobotGroup` holding this robot, if any. Only
    #: the group may command it then (see :meth:`send_action`).
    _group: Optional[Any] = None

    #: Episode outcome, for simulators that judge one (benchmark environments).
    #: ``None`` means the driver does not report it. Drivers that do set these
    #: in ``reset`` and update them in ``_send_action``.
    reward: Optional[float] = None
    success: Optional[bool] = None
    done: Optional[bool] = None

    @property
    @abstractmethod
    def embodiment(self) -> Embodiment:
        """The declarative I/O contract this driver exposes."""

    @abstractmethod
    def connect(self) -> None:
        """Establish the connection to the robot. Called once before control."""

    @abstractmethod
    def reset(self, seed: Optional[int] = None) -> None:
        """Bring the robot to a safe/initial state (home pose, clear faults).

        Simulators that sample a scene take ``seed`` to pick it (e.g. an env
        seed or an init-state index). Other drivers ignore it.
        """

    @abstractmethod
    def get_observation(self) -> Observation:
        """Read the latest sensors and return a normalized observation.

        Sets ``stamps["body"]`` to when the body state in it arrived, on the
        :meth:`now` clock (see :mod:`hirobot.stamps`).
        """

    def now(self) -> float:
        """The robot's clock, in seconds: what ``Observation.stamps`` times are on.

        ``time.monotonic()`` for hardware (the default); a simulator returns its
        simulated time, so ``robot.now() - stamp.t`` is a reading's age on
        both. Wrappers return their inner robot's clock.
        """
        return time.monotonic()

    @property
    def cartesian_targets(self) -> Tuple[str, ...]:
        """End-effectors that take a pose target under ``CARTESIAN``.

        The default is every end-effector of the embodiment when the driver lists
        ``CARTESIAN``, and none otherwise. A robot that tracks only some (a
        whole-body controller that follows the wrists and head) overrides it.
        """
        if ControlMode.CARTESIAN not in self.action_modes:
            return ()
        return tuple(ee.name for ee in self.embodiment.end_effectors)

    def set_safety(self, gate: Optional[SafetyGate]) -> None:
        """Attach (or clear) the :class:`SafetyGate` enforced by ``send_action``."""
        self.safety = gate

    # ------------------------------------------------------------- faults

    @property
    def fault(self) -> Optional[str]:
        """Why the robot refuses commands, or ``None`` when it is fine.

        A driver or layer that damps or stops itself (its control loop failed,
        an e-stop) calls :meth:`set_fault`; from then on :meth:`send_action`
        raises :class:`RobotFaultError`. Its :meth:`reset` calls
        :meth:`clear_fault` once the robot is commandable again. Wrappers
        report their inner robot's fault too. Also in ``get_status().fault``.
        """
        return self._fault

    def set_fault(self, reason: str) -> None:
        """Enter the fault state: commands are refused until :meth:`clear_fault`."""
        self._fault = str(reason) or "fault"

    def clear_fault(self) -> None:
        """Leave the fault state.

        A driver's :meth:`reset` does this itself when it returns: every
        subclass's ``reset`` is wrapped so that it clears the fault, unless an
        :meth:`estop` came while it ran.
        """
        self._fault = None

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if "reset" in cls.__dict__:
            cls.reset = _clears_fault(cls.__dict__["reset"])  # type: ignore[method-assign]

    # ------------------------------------------------------------- e-stop

    def estop(self) -> None:
        """Stop the robot now. Safe to call from any thread, at any time.

        In order: a long call running on another thread (a ``reset`` ramp, a
        wait in ``connect``) is told to give up, and raises
        :class:`RobotStoppedError` at its next step; the robot enters the fault
        state, so later commands raise :class:`RobotFaultError` until
        :meth:`reset`; a command already being sent is let finish (at most
        ``ESTOP_LOCK_TIMEOUT_S``); then the driver's :meth:`_estop` runs. It
        does not go through the safety gate or a robot group's owner check.

        What stopping means depends on the robot (see :meth:`_estop`): damping
        on Unitree and MuJoCo, ``MotionStop`` on AgiBot G1, a controlled stop on
        UR, ending the controller on Franka. An error from the driver is raised
        after the fault state is set.
        """
        self._bump_stop()
        if self._fault is None:
            self.set_fault("estop()")
        lock = self._estop_signal.lock
        got = lock.acquire(timeout=ESTOP_LOCK_TIMEOUT_S)
        if not got:
            logger.warning("%s.estop(): a command did not finish within %.2f s; stopping anyway",
                           type(self).__name__, ESTOP_LOCK_TIMEOUT_S)
        try:
            self._estop()
        finally:
            if got:
                lock.release()

    def _estop(self) -> None:
        """Driver primitive behind :meth:`estop`: put the robot in its safe state.

        The default calls the driver's ``damp()`` when it has one, and does
        nothing otherwise (the fault state still refuses every command, and
        the robot keeps its last target). It may run while another thread is
        inside a driver call, so it must not wait on that call.
        """
        damp = getattr(self, "damp", None)
        if callable(damp):
            damp()

    # ------------------------------------------------------------ commands

    def send_action(self, action: Action) -> None:
        """Command the robot, enforcing the safety gate first.

        Runs :meth:`check_action` (fault state, ``joint_names`` outside
        :attr:`direct_joints`, the attached :class:`~hirobot.safety.SafetyGate`)
        and passes the possibly-clamped action to the driver's
        :meth:`_send_action`. A refusal raises :class:`~hirobot.safety.SafetyError`
        (or :class:`RobotFaultError`) and nothing reaches the robot. Drivers
        override ``_send_action``, not this method, so the gate is never bypassed.

        A robot held by a :class:`~hirobot.group.RobotGroup` refuses commands
        that do not come through the group (``RuntimeError``), so the group's
        checks and its stop-everything-on-error rule cannot be skipped.
        """
        self._check_owner("send_action")
        self._send_checked(self.check_action(action))

    def check_action(self, action: Action) -> Action:
        """Run every check :meth:`send_action` runs, without sending.

        Returns the action to send (the gate may clamp it). Raises
        :class:`RobotFaultError` in the fault state and
        :class:`~hirobot.safety.SafetyError` on a refusal. A robot group calls
        this on every member before sending anything to any of them.
        """
        if self.fault is not None:
            raise RobotFaultError(f"{type(self).__name__} is in a fault state "
                                  f"({self.fault}); call reset() first")
        if action.members and not self.accepts_members:
            raise SafetyError(f"{type(self).__name__} is a single robot; Action.members is "
                              "for a RobotGroup")
        if action.joint_names is not None:
            # Joints named beside a non-joint mode must be ones the robot leaves
            # free; the rest belong to whatever runs it (a whole-body controller).
            owned = [n for n in action.joint_names if n not in self.direct_joints]
            if owned:
                raise SafetyError(
                    f"joint(s) {owned} are not free to command directly; "
                    f"direct_joints: {list(self.direct_joints) or 'none'}")
        if self.safety is not None:
            verdict = self.safety.check(action, self.embodiment)
            if not verdict.allowed:
                raise SafetyError(verdict.reason)
            action = verdict.action
        return action

    def _send_checked(self, action: Action) -> None:
        """Send an action that already passed :meth:`check_action`.

        Only the fault state is checked again (it may have changed since). For
        the group, which checks every member first and then sends.
        """
        with self._commanding():
            # Checked under the command lock: an estop() that set the fault
            # before this point also sends its stop before or after this, never
            # a command after the stop.
            if self.fault is not None:
                raise RobotFaultError(f"{type(self).__name__} is in a fault state "
                                      f"({self.fault}); call reset() first")
            self._send_action(action)

    def _check_owner(self, what: str) -> None:
        group = self._group
        if group is not None and not group._in_call():
            raise RuntimeError(
                f"{type(self).__name__} is a member of a RobotGroup; {what} goes "
                "through the group")

    @abstractmethod
    def _send_action(self, action: Action) -> None:
        """Command the robot with an action in its embodiment's action space.

        Driver-specific primitive called by :meth:`send_action` after the safety
        gate has passed. Implement this, not ``send_action``.
        """

    def hold(self) -> None:
        """Keep the robot where it is, e.g. after a policy is stopped.

        The default commands the current joint positions when the driver takes
        POSITION actions, and does nothing otherwise. Drivers override it when
        holding means something else (a simulator that takes only Cartesian deltas
        holds by not sending; a Unitree robot under its built-in motion service
        keeps itself). Goes through :meth:`send_action`, so the safety gate applies.

        A driver with its own balance loop (a whole-body-controlled humanoid) must
        override this to switch its reference to standing still (base velocity
        ``[0, 0, 0]``). Sending the current ``joint_pos`` freezes the joints and
        turns off balancing, and the humanoid falls.
        """
        if ControlMode.POSITION not in self.action_modes:
            return
        obs = self.get_observation()
        if obs.joint_pos is None:
            return
        self.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos.copy()))

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        """Set per-joint PD gains. Override on drivers that support PD control."""

    def get_status(self) -> RobotStatus:
        """Read health and housekeeping signals: battery, temperatures, faults.

        Separate from :meth:`get_observation` so these never ride along to a
        model server. The default reports nothing (every field empty); drivers
        whose robot exposes such signals override it.
        """
        return RobotStatus(timestamp=time.time(), fault=self.fault)

    def idle_step(self) -> None:
        """Let simulated time pass for one control period without a new command.

        A robot group calls this on its simulated members that the current
        action did not mention, so every member's simulation advances by the
        same amount. Simulators that advance only on ``send_action`` implement
        it by stepping under the last command; the default does nothing
        (real robots keep running on their own).
        """

    @property
    def has_robot_skills(self) -> bool:
        """Whether this driver can offer robot skills at all.

        Callers use it to decide whether to offer skill tools.
        """
        return False

    def list_robot_skills(self) -> List[RobotSkillSpec]:
        """Skills the robot's own controller can run (walk, sit, wave, ...).

        Declared by the driver, so it differs between robots; callers show it to
        the model instead of knowing skills in advance. The default is empty.
        """
        return []

    def run_robot_skill(
        self, name: str, args: Optional[Mapping[str, Any]] = None
    ) -> RobotSkillResult:
        """Start robot skill ``name``; returns once the robot accepts it.

        Arguments are checked against the skill's schema first. Drivers with
        skills override :meth:`_run_robot_skill`, not this method.

        Raises:
            RobotSkillError: Unknown skill, bad arguments, or the robot refused.
        """
        self._check_owner("run_robot_skill")
        specs = {s.name: s for s in self.list_robot_skills()}
        if name not in specs:
            raise RobotSkillError(
                f"unknown robot skill {name!r}; available: {sorted(specs) or 'none'}")
        spec = specs[name]
        checked = validate_args(spec, args)
        self._run_robot_skill(spec, checked)
        return RobotSkillResult(name=name, args=checked, duration_s=spec.duration_s)

    def _run_robot_skill(self, spec: RobotSkillSpec, args: Dict[str, Any]) -> None:
        """Driver primitive behind :meth:`run_robot_skill`; ``args`` are already checked."""
        raise RobotSkillError(f"{type(self).__name__} has no robot skills")

    def stop_robot_skill(self) -> None:
        """Stop whatever robot skill is running and leave the robot standing still.

        The default does nothing (there are no skills to stop).
        """

    def disconnect(self) -> None:
        """Release the connection and stop the robot. Override if needed."""
