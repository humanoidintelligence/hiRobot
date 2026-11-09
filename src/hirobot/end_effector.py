"""The end-effector driver contract and its registry.

An :class:`EndEffectorDriver` drives one dexterous hand or gripper that has a
driver of its own, separate from the body driver. How it shows up in
``Observation`` / ``Action`` is its **interface**, chosen by the config:

* ``joints``: the end-effector lists joints. They are ordinary entries of
  ``Embodiment.joints``, read in ``joint_pos`` / ``joint_vel`` /
  ``joint_torque`` and commanded through ``joint_target`` / ``kp`` / ``kd``::

      end_effectors:
        - name: left_hand
          joints: [left_hand_thumb_0, ...]   # the hand's motors, in its order
          driver:
            type: unitree_handcmd            # a registered end-effector driver
            side: left                       # remaining keys go to the driver

* ``opening``: no joints and ``has_gripper: true``. The reading is
  ``Observation.gripper[<name>]`` and the command ``Action.gripper[<name>]``,
  both in [0, 1] (0 closed, 1 open), sent in any control mode::

      end_effectors:
        - name: gripper
          has_gripper: true
          driver: {type: ..., port: /dev/ttyUSB0}

A driver class lists the interfaces it supports in :attr:`EndEffectorDriver.interfaces`.

With a real body driver, ``make_robot`` builds one driver per end-effector with
a ``driver:`` and wraps everything in a :class:`~hirobot.assembly.RobotAssembly`,
which gives the body only its own joints and routes each end-effector's part
of the observation and action to its driver. A simulator models the fingers as
joints of its own and ignores ``driver``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from hirobot.embodiment import Joint
from hirobot.registry import Registry
from hirobot.robot import ESTOP_LOCK_TIMEOUT_S, Stoppable
from hirobot.stamps import Stamp

#: The ways an end-effector can appear in ``Observation`` / ``Action``.
INTERFACES = ("joints", "opening")


@dataclass
class JointsState:
    """One reading of an end-effector's joints (``joints`` interface), in its joint order.

    Attributes:
        q: Joint positions (rad).
        dq: Joint velocities (rad/s).
        tau: Joint torques (Nm); NaN where the device reports none it can trust.
        stamp: When the reading arrived (``time.monotonic()``) and its number;
            goes to ``Observation.stamps`` under the end-effector's name.
        extra: Additional signals (e.g. tactile arrays), merged into
            ``Observation.extra`` under ``<end-effector name>_<key>``.
    """

    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    stamp: Stamp
    extra: Dict[str, np.ndarray] = field(default_factory=dict)


@dataclass
class OpeningState:
    """One reading of a gripper's opening (``opening`` interface).

    Attributes:
        opening: Measured opening in [0, 1], 0 closed and 1 open; goes to
            ``Observation.gripper`` under the end-effector's name.
        stamp: When the reading arrived (``time.monotonic()``) and its number;
            goes to ``Observation.stamps`` under the end-effector's name.
        extra: Additional signals (contact, current, the echoed target), merged
            into ``Observation.extra`` under ``<end-effector name>_<key>``.
    """

    opening: float
    stamp: Stamp
    extra: Dict[str, Any] = field(default_factory=dict)


def check_opening(value: Any) -> float:
    """``value`` as a float, or ``ValueError`` if it is not finite or outside [0, 1]."""
    v = float(np.asarray(value, dtype=float))
    if not np.isfinite(v) or not 0.0 <= v <= 1.0:
        raise ValueError(f"gripper opening must be in [0, 1] (0 closed, 1 open); got {value!r}")
    return v


class EndEffectorDriver(Stoppable, ABC):
    """One dexterous hand or gripper with a driver of its own.

    A driver implements the methods of the interfaces it lists in
    :attr:`interfaces`: :meth:`command` (and :meth:`set_gains`) for
    ``joints``, :meth:`command_opening` for ``opening``. :meth:`get_state`
    returns a :class:`JointsState` or an :class:`OpeningState` to match
    :attr:`interface`.

    Args:
        name: The end-effector name (``left_hand``).
        joints: The embodiment joints this end-effector drives, in its motor
            order; empty for the ``opening`` interface. A ``position_limit`` on
            a joint is enforced: targets outside it are clipped before they
            are sent.
        read_only: Never create a command channel; any command raises.
    """

    #: Interfaces this driver supports (a subset of :data:`INTERFACES`).
    interfaces: Tuple[str, ...] = ("joints",)

    def __init__(self, name: str, joints: Sequence[Joint], read_only: bool = False) -> None:
        self.name = name
        self.joints = list(joints)
        self.read_only = bool(read_only)
        #: The interface in use: ``joints`` when the config lists joints, else ``opening``.
        self.interface = "joints" if self.joints else "opening"
        if self.interface not in self.interfaces:
            raise ValueError(f"end-effector {name!r}: {type(self).__name__} supports "
                             f"{list(self.interfaces)}, not {self.interface!r} "
                             f"({'joints listed' if self.joints else 'no joints listed'})")
        n = len(self.joints)
        lo = [j.position_limit[0] if j.position_limit else -np.inf for j in self.joints]
        hi = [j.position_limit[1] if j.position_limit else np.inf for j in self.joints]
        self._lo = np.asarray(lo, dtype=float).reshape(n)
        self._hi = np.asarray(hi, dtype=float).reshape(n)

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    @property
    def num_joints(self) -> int:
        return len(self.joints)

    def clip(self, q: np.ndarray) -> np.ndarray:
        """Clip position targets into the joints' ``position_limit``."""
        return np.clip(np.asarray(q, dtype=float), self._lo, self._hi)

    @abstractmethod
    def connect(self) -> None:
        """Open the device and wait for its first state."""

    @abstractmethod
    def get_state(self) -> Union[JointsState, OpeningState]:
        """The latest state for :attr:`interface`. Raises if none arrived or it is stale."""

    def command(self, q: np.ndarray, kp: Optional[np.ndarray] = None,
                kd: Optional[np.ndarray] = None) -> None:
        """``joints``: send position targets (clipped to limits) with optional per-joint gains."""
        raise NotImplementedError(f"{type(self).__name__} has no 'joints' interface")

    def command_opening(self, opening: float) -> None:
        """``opening``: send an opening target in [0, 1] (0 closed, 1 open).

        Returns without waiting for the motion; a new target replaces the old
        one. Raises ``ValueError`` outside [0, 1] (see :func:`check_opening`).
        """
        raise NotImplementedError(f"{type(self).__name__} has no 'opening' interface")

    def reset(self) -> None:
        """Bring the device under control holding its measured pose, without a jump."""

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        """``joints``: set the default gains used when :meth:`command` gets none."""

    @abstractmethod
    def damp(self) -> None:
        """Leave the device in its safe passive state. A no-op when read-only."""

    def estop(self) -> None:
        """Damp now, from any thread: a running :meth:`reset` ramp gives up
        (:class:`~hirobot.robot.RobotStoppedError`) and no ramp step goes out after it."""
        self._bump_stop()
        lock = self._estop_signal.lock
        got = lock.acquire(timeout=ESTOP_LOCK_TIMEOUT_S)
        try:
            self.damp()
        finally:
            if got:
                lock.release()

    def disconnect(self) -> None:
        """Damp the device and release it. Safe to call more than once."""
        self.damp()


END_EFFECTORS: Registry = Registry("end-effector driver", EndEffectorDriver)
#: Class decorator that registers an :class:`EndEffectorDriver` under a name.
register_end_effector = END_EFFECTORS.register
#: Look up a registered end-effector driver class by name.
get_end_effector = END_EFFECTORS.get
#: Names of all registered end-effector drivers.
available_end_effectors = END_EFFECTORS.available


def end_effector_joint_names(embodiment: Any) -> List[str]:
    """Joints driven by an end-effector driver (end-effectors with a ``driver:``)."""
    return [j for ee in embodiment.end_effectors if ee.driver for j in ee.joints]


def has_end_effector_drivers(embodiment: Any) -> bool:
    """Whether any end-effector has a ``driver:`` (on either interface)."""
    return any(ee.driver for ee in embodiment.end_effectors)


def body_embodiment(embodiment: Any) -> Any:
    """The embodiment minus what its end-effector drivers own, for the body driver.

    Joint order is kept, so the body joints must be listed in the body's motor
    order. End-effectors keep their names but lose their joints and drivers; one
    on the ``opening`` interface also loses ``has_gripper``, since its own
    driver, not the body, drives that gripper.
    """
    if not has_end_effector_drivers(embodiment):
        return embodiment
    owned = set(end_effector_joint_names(embodiment))
    return embodiment.model_copy(update={
        "joints": [j for j in embodiment.joints if j.name not in owned],
        "end_effectors": [
            ee.model_copy(update={"joints": [], "driver": None,
                                  "has_gripper": ee.has_gripper and not (ee.driver and not ee.joints)})
            for ee in embodiment.end_effectors],
    })


def build_end_effectors(embodiment: Any, network_interface: Optional[str] = None,
                        read_only: bool = False) -> Dict[str, "EndEffectorDriver"]:
    """One driver per end-effector with a ``driver:`` spec, keyed by name.

    ``network_interface`` fills in the spec's NIC when it sets none (Unitree
    hands share the body's DDS NIC); ``read_only`` forces every driver read-only.
    Raises ``ValueError`` before building anything when a driver does not
    support the interface its end-effector asks for.
    """
    joints = {j.name: j for j in embodiment.joints}
    plan = []
    for ee in embodiment.end_effectors:
        if not ee.driver:
            continue
        spec = dict(ee.driver)
        if "type" not in spec:
            raise ValueError(f"end-effector {ee.name!r}: driver needs a 'type'")
        cls = get_end_effector(spec.pop("type"))
        interface = "joints" if ee.joints else "opening"
        if interface not in cls.interfaces:
            hint = ("list its joints" if "joints" in cls.interfaces
                    else "drop joints: and set has_gripper: true")
            raise ValueError(f"end-effector {ee.name!r}: {cls.__name__} supports "
                             f"{list(cls.interfaces)}, not {interface!r}; {hint}")
        if network_interface is not None:
            spec.setdefault("network_interface", network_interface)
        if read_only:
            spec["read_only"] = True
        plan.append((ee.name, cls, [joints[n] for n in ee.joints], spec))
    return {name: cls(name, ee_joints, **spec) for name, cls, ee_joints, spec in plan}
