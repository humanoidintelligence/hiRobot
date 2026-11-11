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
from types import MappingProxyType
from typing import Any, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.core.robot_skills import (
    RobotSkillError,
    RobotSkillResult,
    RobotSkillSpec,
    validate_args,
)
from hirobot.control.safety import SafetyError, SafetyGate
from hirobot.core.types import Action, ControlMode, Observation, RobotStatus

logger = logging.getLogger(__name__)


class RobotFaultError(RuntimeError):
    """Raised by ``send_action`` while a robot is in a fault state (see :attr:`Robot.fault`).

    A robot enters one when it damps or stops itself (a whole-body controller
    whose loop failed, an assembly that stopped every part after one
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

    Used by :class:`Robot` and :class:`~hirobot.devices.end_effector.EndEffectorDriver`. A driver
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
        _forget_targets(self)
        if self._stop_token() == token:
            self._clear_fault()
        return result

    wrapped.__name__, wrapped.__qualname__ = reset.__name__, reset.__qualname__
    wrapped.__doc__, wrapped.__wrapped__ = reset.__doc__, reset
    wrapped._clears_fault = True  # type: ignore[attr-defined]
    return wrapped


def _forget_targets(robot) -> None:
    """Drop the gate's step reference: the next command is compared with the measured pose."""
    forget = getattr(getattr(robot, "safety", None), "forget", None)
    if forget is not None:
        forget()


def _holds(hold):
    """Wrap a ``hold``: its commands skip the state checks, and it drops the step reference."""
    if getattr(hold, "_holds", False) or getattr(hold, "__isabstractmethod__", False):
        return hold

    def wrapped(self, *args, **kwargs):
        from hirobot.control.safety import holding

        with holding():
            result = hold(self, *args, **kwargs)
        _forget_targets(self)
        return result

    wrapped.__name__, wrapped.__qualname__ = hold.__name__, hold.__qualname__
    wrapped.__doc__, wrapped.__wrapped__ = hold.__doc__, hold
    wrapped._holds = True  # type: ignore[attr-defined]
    return wrapped


def _reports_safety(get_status):
    """Wrap a ``get_status``: add the gate's simulation-check counters, if it has one."""
    if getattr(get_status, "_reports_safety", False):
        return get_status

    def wrapped(self, *args, **kwargs):
        status = get_status(self, *args, **kwargs)
        if status is None:
            return status
        sim_check = getattr(getattr(self, "safety", None), "sim_check", None)
        if sim_check is not None:
            status.safety["sim_check"] = sim_check.stats()
        arbiter = getattr(self, "_arbiter", None)
        if arbiter is not None:
            status.safety["audit"] = arbiter.audit()
        return status

    wrapped.__name__, wrapped.__qualname__ = get_status.__name__, get_status.__qualname__
    wrapped.__doc__, wrapped.__wrapped__ = get_status.__doc__, get_status
    wrapped._reports_safety = True  # type: ignore[attr-defined]
    return wrapped


#: Calls the command arbiter checks (:mod:`hirobot.control.arbiter`).
ARBITRATED = ("send_action", "hold", "reset", "run_robot_skill")


def _arbitrated(method, what: str):
    """Wrap a command method: refuse it while another lease holds the robot."""
    if getattr(method, "_arbitrated", False) or getattr(method, "__isabstractmethod__", False):
        return method

    def wrapped(self, *args, **kwargs):
        arbiter = self._arbiter
        if arbiter is None:
            return method(self, *args, **kwargs)
        from hirobot.control.arbiter import _touched

        with arbiter._admit(None, what, touched=lambda: _touched(self, what, args)):
            return method(self, *args, **kwargs)

    wrapped.__name__, wrapped.__qualname__ = method.__name__, method.__qualname__
    wrapped.__doc__, wrapped.__wrapped__ = method.__doc__, method
    wrapped._arbitrated = True  # type: ignore[attr-defined]
    return wrapped


class Robot(Stoppable, ABC):
    """Common interface for observing and commanding a robot.

    A driver maps hiRobot's embodiment-aware :class:`~hirobot.core.types.Observation`
    and :class:`~hirobot.core.types.Action` schemas onto a specific robot's native
    sensor reads and command API. Control stacks, policies, and agents only ever
    see this interface, so the hardware — or a simulator standing in for it — can
    be swapped without touching callers.

    The surveyed SDKs (Unitree, Fourier GRx, AgiBot X1, Franka, UR) all reduce to
    the same core: connect, read joint (and optionally base/task) state, send a
    joint-or-Cartesian command, and set PD gains. That is exactly this contract.

    Safety is enforced here, not in callers. :meth:`send_action` runs an optional
    :class:`~hirobot.control.safety.SafetyGate` and then dispatches to the driver's
    :meth:`_send_action`; a driver implements ``_send_action`` and never needs to
    touch the gate. Attach a gate with :meth:`_set_safety` (``make_robot`` does
    this from the embodiment and the config's ``safety:`` section), so every path
    that reaches the robot — the control loop, eval scripts, an agent's tools —
    clears the same check and cannot skip it.
    """

    #: The gate applied to every action in ``send_action``; ``None`` disables it.
    safety: Optional[SafetyGate] = None

    #: Seconds without a new ``VELOCITY`` / ``BASE_VELOCITY`` command before
    #: the driver stops the motion (:mod:`hirobot.control.watchdog`); ``None`` when the
    #: driver has no watchdog. Drivers that stream velocities set it from
    #: their ``command_timeout_s`` argument (config ``body.command_timeout_s``,
    #: ``null`` turns it off).
    command_timeout_s: Optional[float] = None

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

    #: Kinds of :class:`~hirobot.core.types.MotionClip` (``space``) the robot can
    #: track under ``MOTION_CLIP``; empty means it takes no clips.
    motion_spaces: Tuple[str, ...] = ()

    #: Joints the caller may command by name (``Action.joint_names``) alongside
    #: a non-joint mode, because whatever runs the robot leaves them free: the
    #: fingers under a body-only whole-body controller, say. Empty by default.
    direct_joints: Tuple[str, ...] = ()

    #: Devices attached to the robot, by name: camera drivers
    #: (:class:`~hirobot.devices.camera.CameraDriver`), microphones and speakers
    #: (:mod:`hirobot.devices.audio`). Empty for a bare driver; the assembly that
    #: ``make_robot`` builds fills them, and every layer above passes them up,
    #: so the outermost object has them too.
    cameras: Mapping[str, Any] = MappingProxyType({})
    microphones: Mapping[str, Any] = MappingProxyType({})
    speakers: Mapping[str, Any] = MappingProxyType({})

    #: Why the robot is in a fault state, or ``None``. Set with :meth:`_set_fault`.
    _fault: Optional[str] = None

    #: Whether ``send_action`` takes ``Action.parts`` (an assembly with child parts does).
    accepts_parts: bool = False

    #: The :class:`~hirobot.control.arbiter.CommandArbiter` checking this robot's
    #: commands; set by :func:`~hirobot.make_robot` on the outermost robot, or
    #: by the first :meth:`claim`.
    _arbiter: Optional[Any] = None

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
        :meth:`now` clock (see :mod:`hirobot.core.stamps`).
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

    def _set_safety(self, gate: Optional[SafetyGate]) -> None:
        """Attach (or clear) the :class:`SafetyGate` enforced by ``send_action``."""
        self.safety = gate

    # ------------------------------------------------------------- faults

    @property
    def fault(self) -> Optional[str]:
        """Why the robot refuses commands, or ``None`` when it is fine.

        A driver or layer that damps or stops itself (its control loop failed,
        an e-stop) calls :meth:`_set_fault`; from then on :meth:`send_action`
        raises :class:`RobotFaultError`. Its :meth:`reset` calls
        :meth:`_clear_fault` once the robot is commandable again. Wrappers
        report their inner robot's fault too. Also in ``get_status().fault``.
        """
        return self._fault

    def _set_fault(self, reason: str) -> None:
        """Enter the fault state: commands are refused until :meth:`_clear_fault`."""
        self._fault = str(reason) or "fault"

    def _clear_fault(self) -> None:
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
        if "get_status" in cls.__dict__:
            cls.get_status = _reports_safety(cls.__dict__["get_status"])  # type: ignore[method-assign]
        if "hold" in cls.__dict__:
            cls.hold = _holds(cls.__dict__["hold"])  # type: ignore[method-assign]
        for what in ARBITRATED:
            if what in cls.__dict__:
                setattr(cls, what, _arbitrated(cls.__dict__[what], what))

    # ------------------------------------------------------------ arbiter

    def claim(self, name: str, priority: Any = None, on_preempt: Any = None,
              envelope: Optional[Mapping[str, Any]] = None,
              parts: Optional[Sequence[str]] = None) -> Any:
        """Take this robot's command stream; returns a :class:`~hirobot.control.arbiter.CommandLease`.

        ``priority`` is a :class:`~hirobot.control.arbiter.Priority` (``CALLER`` by
        default). While the lease is held, ``send_action`` / ``hold`` /
        ``reset`` / ``run_robot_skill`` called on the robot directly raise
        :class:`~hirobot.control.arbiter.RobotBusyError`; call them on the lease. A
        higher priority preempts a lower one (the robot holds, the old lease's
        ``on_preempt`` runs, its next call raises
        :class:`~hirobot.control.arbiter.PreemptedError`). See :mod:`hirobot.control.arbiter`.

        ``envelope`` narrows the limits for this lease's ``send_action``:
        ``joint_limit``, ``max_joint_step``, ``max_joint_error``,
        ``gripper_range``, ``allowed_modes``. Only items stricter than the
        robot's gate count; the rest are ignored with a warning.

        ``parts`` (part paths, a robot built from an ``assembly:`` config)
        takes only those parts: leases on parts that do not overlap are held
        at once. See :mod:`hirobot.control.arbiter`.
        """
        from hirobot.control.arbiter import CommandArbiter, Priority

        if self._arbiter is None:
            self._arbiter = CommandArbiter(self)
        return self._arbiter.claim(name, Priority.CALLER if priority is None else priority,
                                   on_preempt, envelope=envelope, parts=parts)

    # ------------------------------------------------------------- e-stop

    def command_events(self) -> List[Any]:
        """The command arbiter's audit events, oldest first (claims, releases,
        preemptions, refusals, busy calls); empty without an arbiter. See
        :class:`~hirobot.control.arbiter.CommandEvent`."""
        return [] if self._arbiter is None else self._arbiter.events()

    def estop(self) -> None:
        """Stop the robot now. Safe to call from any thread, at any time.

        In order: a long call running on another thread (a ``reset`` ramp, a
        wait in ``connect``) is told to give up, and raises
        :class:`RobotStoppedError` at its next step; the robot enters the fault
        state, so later commands raise :class:`RobotFaultError` until
        :meth:`reset`; a command already being sent is let finish (at most
        ``ESTOP_LOCK_TIMEOUT_S``); then the driver's :meth:`_estop` runs. It
        does not go through the safety gate or the command arbiter.

        What stopping means depends on the robot (see :meth:`_estop`): damping
        on Unitree and MuJoCo, ``MotionStop`` on AgiBot G1, a controlled stop on
        UR, ending the controller on Franka. An error from the driver is raised
        after the fault state is set.
        """
        self._bump_stop()
        if self._fault is None:
            self._set_fault("estop()")
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
        :attr:`direct_joints`, the attached :class:`~hirobot.control.safety.SafetyGate`)
        and passes the possibly-clamped action to the driver's
        :meth:`_send_action`. A refusal raises :class:`~hirobot.control.safety.SafetyError`
        (or :class:`RobotFaultError`) and nothing reaches the robot. Drivers
        override ``_send_action``, not this method, so the gate is never bypassed.

        A robot whose command stream someone holds refuses a direct call with
        :class:`~hirobot.control.arbiter.RobotBusyError`; see :meth:`claim`.
        """
        self._send_checked(self.check_action(action))

    def check_action(self, action: Action) -> Action:
        """Run every check :meth:`send_action` runs, without sending.

        Returns the action to send (the gate may clamp it). Raises
        :class:`RobotFaultError` in the fault state and
        :class:`~hirobot.control.safety.SafetyError` on a refusal.
        """
        if self.fault is not None:
            raise RobotFaultError(f"{type(self).__name__} is in a fault state "
                                  f"({self.fault}); call reset() first")
        if action.parts and not self.accepts_parts:
            raise SafetyError(f"{type(self).__name__} has no parts; Action.parts is for a robot "
                              "assembled from parts (an assembly: config)")
        if action.joint_names is not None:
            # Joints named beside a non-joint mode must be ones the robot leaves
            # free; the rest belong to whatever runs it (a whole-body controller).
            owned = [n for n in action.joint_names if n not in self.direct_joints]
            if owned:
                raise SafetyError(
                    f"joint(s) {owned} are not free to command directly; "
                    f"direct_joints: {list(self.direct_joints) or 'none'}")
        if self.safety is not None:
            from hirobot.control.safety import is_holding

            # The chain: checks on the action alone, then the ones that read
            # the robot (not for a hold), then the simulation check, last.
            verdict = self.safety.check(action, self.embodiment)
            if not verdict.allowed:
                raise SafetyError(verdict.reason)
            action = verdict.action
            check_state = getattr(self.safety, "check_state", None)
            if check_state is not None and not is_holding():
                action = check_state(action, self)
            sim_check = getattr(self.safety, "sim_check", None)
            if sim_check is not None:  # last: only actions every other check passed
                sim_check.check(action, self)
        return action

    def _send_checked(self, action: Action) -> None:
        """Send an action that already passed :meth:`check_action`.

        Only the fault state is checked again (it may have changed since). An
        assembly with child parts checks every part first, then sends.
        """
        with self._commanding():
            # Checked under the command lock: an estop() that set the fault
            # before this point also sends its stop before or after this, never
            # a command after the stop.
            if self.fault is not None:
                raise RobotFaultError(f"{type(self).__name__} is in a fault state "
                                      f"({self.fault}); call reset() first")
            self._send_action(action)
            commit = getattr(self.safety, "commit", None)
            if commit is not None:  # the step reference for the next command
                commit(action)

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
        # Marked as a hold (see _holds): the state checks and the simulation
        # check do not run for it.
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

    def _idle_step(self) -> None:
        """Let simulated time pass for one control period without a new command.

        An assembly calls this on its simulated parts that the current action
        did not mention, so every part's simulation advances by the same
        amount. Simulators that advance only on ``send_action`` implement
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


Robot.get_status = _reports_safety(Robot.__dict__["get_status"])  # type: ignore[method-assign]
Robot.hold = _holds(Robot.__dict__["hold"])  # type: ignore[method-assign]
for _what in ARBITRATED:
    if _what in Robot.__dict__:
        setattr(Robot, _what, _arbitrated(Robot.__dict__[_what], _what))
del _what
