"""The safety gate: last check before any action reaches the robot.

A policy (or a VLA model it calls) can emit an action that is malformed or out
of range. On hardware that is how things break. The :class:`SafetyGate` runs on
every action inside :meth:`~hirobot.core.robot.Robot.send_action`, like a permission
check for motor commands. It is intentionally simple and conservative: validate
shape and bounds, and refuse what it cannot make safe.

Enforcement lives at the robot boundary, not in the caller. ``send_action``
applies the gate itself and raises :class:`SafetyError` on a refusal, so every
consumer that drives a robot — the control loop, eval scripts, or an agent's
tools — is held to the same check through one choke point and cannot skip it by
forgetting to call the gate. By default a refusal *raises* rather than silently
rewriting the command (``clamp=False``); set ``clamp=True`` for robots that
genuinely want out-of-range joint targets clipped into bounds.

The checks run in a fixed order, as a chain. :meth:`SafetyGate.check` holds
the ones that need only the action (shape, finite values, ``joint_limit``,
``gripper_range``). :meth:`SafetyGate.check_state` holds the ones that need the
robot: ``max_joint_step`` (against the last target the robot accepted),
``max_joint_error`` (against the measured position), ``min_battery_soc`` and
``max_joint_temperature``. The optional simulation check runs last.
:meth:`~hirobot.core.robot.Robot.check_action` runs them in that order.
"""

from __future__ import annotations

import contextlib
import contextvars
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple, Union

import numpy as np


_HOLDING: contextvars.ContextVar[bool] = contextvars.ContextVar("hirobot_holding", default=False)


@contextlib.contextmanager
def holding() -> Any:
    """Mark the commands sent inside as :meth:`~hirobot.core.robot.Robot.hold`.

    A hold keeps the robot where it is. The checks that compare a command with
    the robot's state (``max_joint_step``, ``max_joint_error``, battery,
    temperature) and the simulation check do not run for it: refusing to stop
    would be worse than the command.
    """
    token = _HOLDING.set(True)
    try:
        yield
    finally:
        _HOLDING.reset(token)


def is_holding() -> bool:
    """Whether the current call is inside :func:`holding`."""
    return _HOLDING.get()


class SafetyError(RuntimeError):
    """Raised when the :class:`SafetyGate` refuses an action at ``send_action``."""


@dataclass
class SafetyVerdict:
    """Result of a gate check.

    Attributes:
        allowed: Whether the action may be sent.
        action: The action to send (possibly clamped); only valid if allowed.
        reason: Human-readable explanation when not allowed.
    """

    allowed: bool
    action: Any = None
    reason: str = ""


#: A joint limit: one number for every joint, or one per joint (``num_dofs`` long).
Limit = Union[float, np.ndarray]


class SafetyGate:
    """Validate (and optionally clamp) actions before they reach the robot.

    ``joint_limit``, ``max_joint_step`` and ``max_joint_error`` each take one
    number for every joint or a list with one value per joint (in
    ``embodiment.joint_names`` order; needs ``num_dofs``). A list keeps its
    values as an array; a number stays a number.

    Args:
        num_dofs: Expected joint-target length; ``None`` skips the length check.
        joint_limit: Symmetric per-joint position bound (radians), or ``None`` to
            skip the bound check. Targets outside ``[-joint_limit, joint_limit]``
            are clamped when ``clamp`` is set, otherwise the action is refused.
        clamp: Clamp out-of-range joint targets instead of refusing them. Off by
            default: an out-of-range command is refused (and ``send_action``
            raises) rather than silently rewritten.
        gripper_range: Allowed ``[low, high]`` for gripper commands, or a mapping
            end-effector name -> ``[low, high]`` (a gripper not listed is not
            checked); ``None`` skips the gripper check. Values are clamped into
            the range.
        max_joint_step: Largest change (rad) of a position target from the last
            one the robot accepted, per joint, per command. ``None`` skips it.
            Where there is no last target (after connect, ``reset``, a
            non-position command, for joints a command did not name) the
            measured position is the reference. ``hold()`` is not checked;
            after it, and after ``reset()``, the measured position is the
            reference again.
        max_joint_error: Largest distance (rad) of a position target from the
            measured position, per joint. ``None`` skips it. Reads the
            observation on every position command.
        min_battery_soc: Refuse commands (not ``hold()``) while the battery is
            below this charge, percent. ``None`` skips it.
        max_joint_temperature: Refuse commands (not ``hold()``) while a joint
            motor is hotter than this, degC. ``None`` skips it.
        sim_check: Optional :class:`~hirobot.sim.sim_check.SimCheck` that predicts
            each command in MuJoCo. It runs last, after every other check,
            from :meth:`~hirobot.core.robot.Robot.check_action` (it needs the robot's
            state), and only refuses.
    """

    def __init__(
        self,
        num_dofs: Optional[int] = None,
        joint_limit: Optional[Union[float, Sequence[float]]] = None,
        clamp: bool = False,
        gripper_range: Optional[Union[Sequence[float], Mapping[str, Sequence[float]]]] = None,
        max_joint_step: Optional[Union[float, Sequence[float]]] = None,
        max_joint_error: Optional[Union[float, Sequence[float]]] = None,
        min_battery_soc: Optional[float] = None,
        max_joint_temperature: Optional[float] = None,
        sim_check: Any = None,
    ) -> None:
        self.sim_check = sim_check
        self.max_joint_step = _limit_value("safety.max_joint_step", max_joint_step, num_dofs)
        self.max_joint_error = _limit_value("safety.max_joint_error", max_joint_error, num_dofs)
        self.min_battery_soc = None if min_battery_soc is None else float(min_battery_soc)
        self.max_joint_temperature = (None if max_joint_temperature is None
                                      else float(max_joint_temperature))
        self._lock = threading.Lock()
        self._last: Optional[np.ndarray] = None  # last accepted position targets, NaN = none
        # (id(action), n, idx, targets) of the action check_state last passed.
        self._pending: Optional[Tuple[int, int, np.ndarray, np.ndarray]] = None
        self._status: Optional[Tuple[float, Any]] = None  # (perf_counter, RobotStatus)
        #: Set on a lease's envelope gate (:func:`envelope_gate`): modes it allows.
        self.allowed_modes: Optional[Tuple[Any, ...]] = None
        self.lease_name: Optional[str] = None
        self.num_dofs = num_dofs
        self.joint_limit = _limit_value("safety.joint_limit", joint_limit, num_dofs)
        self.clamp = clamp
        self.gripper_range = _gripper_range(gripper_range)

    @classmethod
    def from_embodiment(cls, embodiment: Any, **kwargs: Any) -> "SafetyGate":
        """Build a gate whose joint-count check matches the embodiment."""
        return cls(num_dofs=getattr(embodiment, "num_dofs", None), **kwargs)

    def check(self, action: Any, embodiment: Any = None) -> SafetyVerdict:
        """Return a verdict for ``action``, clamping in place where configured.

        ``joint_target`` normally has one value per joint. With
        ``action.joint_names`` set (a non-joint mode carrying targets for some
        joints) it has one value per name instead, and every name must be a
        joint of ``embodiment`` when one is given. The other numeric command
        fields (``base_velocity``, ``base_target``, ``latent``, ``ee_targets``,
        the arrays of ``motion``) must be finite.
        """
        if getattr(action, "parts", None):
            # Per-part actions: each leaf is checked in the robot's names, so
            # the same limits apply as to a flat action (hirobot.assembly.parts).
            from hirobot.assembly.parts import check_parts

            return check_parts(self, action, embodiment)
        bad = _non_finite(action)
        if bad:
            return SafetyVerdict(False, reason=f"{bad} contains non-finite values")
        names = getattr(action, "joint_names", None)
        jt = getattr(action, "joint_target", None)
        if jt is not None:
            jt = np.asarray(jt, dtype=float)
            if jt.ndim != 1:
                return SafetyVerdict(False, reason=f"joint_target must be 1-D, got shape {jt.shape}")
            if names is not None:
                if jt.shape[0] != len(names):
                    return SafetyVerdict(
                        False,
                        reason=f"joint_target has {jt.shape[0]} values for {len(names)} joint_names",
                    )
                known = getattr(embodiment, "joint_names", None)
                unknown = [n for n in names if known is not None and n not in known]
                if unknown:
                    return SafetyVerdict(False, reason=f"unknown joint(s) in joint_names: {unknown}")
            elif self.num_dofs is not None and jt.shape[0] != self.num_dofs:
                return SafetyVerdict(
                    False,
                    reason=f"joint_target has {jt.shape[0]} values, embodiment expects {self.num_dofs}",
                )
            if not np.all(np.isfinite(jt)):
                return SafetyVerdict(False, reason="joint_target contains non-finite values")
            if self.joint_limit is not None:
                limit = self._joint_limit_for(names, embodiment, jt.shape[0])
                over = np.abs(jt) > limit
                if np.any(over):
                    if not self.clamp:
                        return SafetyVerdict(False, reason=self._exceed_reason(
                            jt, limit, over, names, embodiment,
                            isinstance(self.joint_limit, np.ndarray)))
                    jt = np.clip(jt, -limit, limit)
            action.joint_target = jt

        for name, cmd in dict(getattr(action, "gripper", {}) or {}).items():
            val = np.asarray(cmd, dtype=float)
            if not np.all(np.isfinite(val)):
                return SafetyVerdict(False, reason=f"gripper {name!r} has non-finite value")
            bounds = self.gripper_bounds(name)
            if bounds is not None:
                action.gripper[name] = np.clip(val, bounds[0], bounds[1])

        return SafetyVerdict(True, action=action)

    def gripper_bounds(self, name: str) -> Optional[Tuple[float, float]]:
        """The ``[low, high]`` gripper ``name`` is clamped into, or ``None``."""
        rng = self.gripper_range
        if rng is None or isinstance(rng, tuple):
            return rng
        return rng.get(name)

    def _joint_limit_for(self, names: Optional[Sequence[str]], embodiment: Any,
                         n: int) -> np.ndarray:
        """``joint_limit`` for the ``n`` targets of an action (``joint_names`` order if set)."""
        limit = self.joint_limit
        if not isinstance(limit, np.ndarray):
            return np.full(n, float(limit))
        if names is None:
            return limit
        known = list(embodiment.joint_names)
        return limit[[known.index(name) for name in names]]

    @staticmethod
    def _exceed_reason(jt: np.ndarray, limit: np.ndarray, over: np.ndarray,
                       names: Optional[Sequence[str]], embodiment: Any, per_joint: bool) -> str:
        if not per_joint:
            return f"{int(over.sum())} joint target(s) exceed ±{limit[0]:.3f} rad"
        excess = np.where(over, np.abs(jt) - limit, -np.inf)
        worst = int(np.argmax(excess))
        labels = list(names) if names is not None else list(
            getattr(embodiment, "joint_names", None) or range(len(jt)))
        return (f"{int(over.sum())} joint target(s) exceed joint_limit; largest: "
                f"{labels[worst]!r} at {jt[worst]:+.3f} rad, limit ±{limit[worst]:.3f} rad")

    # ------------------------------------------------------ checks on the robot

    @property
    def checks_state(self) -> bool:
        """Whether any check needs the robot (:meth:`check_state` does something)."""
        return any(v is not None for v in (self.max_joint_step, self.max_joint_error,
                                           self.min_battery_soc, self.max_joint_temperature))

    def check_state(self, action: Any, robot: Any) -> Any:
        """Run the checks that read ``robot``; return the action, raise :class:`SafetyError`.

        Called by :meth:`~hirobot.core.robot.Robot.check_action` after :meth:`check`.
        With ``clamp`` the step and error limits clip the targets instead of
        refusing. :meth:`commit` makes the targets the step reference once the
        robot accepted the command. Not run for ``hold()``.
        """
        if not self.checks_state:
            return action
        with self._lock:
            self._pending = None
            if self.min_battery_soc is not None or self.max_joint_temperature is not None:
                self._check_status(robot)
            if self.max_joint_step is None and self.max_joint_error is None:
                return action
            write = None
            if getattr(action, "parts", None):
                from hirobot.assembly.parts import parts_position_targets

                idx, target, write = parts_position_targets(action, robot.embodiment)
            else:
                idx = self._joint_index(action, robot.embodiment)
                target = None if idx is None else np.asarray(action.joint_target, dtype=float)
            if idx is None:
                return action
            n = robot.embodiment.num_dofs
            last = self._last if self._last is not None else np.full(n, np.nan)
            measured = None
            if self.max_joint_error is not None or np.any(np.isnan(last[idx])):
                obs = robot.get_observation()
                measured = (np.full(n, np.nan) if obs.joint_pos is None
                            else np.asarray(obs.joint_pos, dtype=float))
            if self.max_joint_step is not None:
                ref = last[idx]
                if measured is not None:
                    ref = np.where(np.isnan(ref), measured[idx], ref)
                target = self._limit(target, ref, self.max_joint_step, "max_joint_step",
                                     "from the last target", idx, robot)
            if self.max_joint_error is not None:
                target = self._limit(target, measured[idx], self.max_joint_error,
                                     "max_joint_error", "from the measured position", idx, robot)
            if write is not None:
                write(target)
            else:
                action.joint_target = target
            self._pending = (id(action), n, idx, np.asarray(target, dtype=float))
            return action

    def commit(self, action: Any) -> None:
        """The robot accepted ``action``: its position targets become the step reference.

        A command without position targets (velocities, torques, a base or
        Cartesian command) drops the reference, so the next position command is
        compared with the measured pose.
        """
        if self.max_joint_step is None:
            return
        with self._lock:
            pending, self._pending = self._pending, None
            if pending is None or pending[0] != id(action):
                self._last = None
                return
            _, n, idx, target = pending
            # Joints a command did not name (beside a non-joint mode) are moved
            # by whatever runs the robot: they have no reference.
            last = np.full(n, np.nan)
            last[idx] = target
            self._last = last

    def forget(self, joints: Optional[Sequence[int]] = None) -> None:
        """Drop the step reference (after ``reset`` and ``hold``): the next
        command is compared with the measured position.

        With ``joints`` (indices), only those joints lose their reference
        (a part held or reset on its own); the others keep theirs.
        """
        with self._lock:
            self._pending = None
            if joints is None or self._last is None:
                self._last = None
                return
            last = self._last.copy()
            last[np.asarray(joints, dtype=int)] = np.nan
            self._last = None if np.all(np.isnan(last)) else last

    def _joint_index(self, action: Any, embodiment: Any) -> Optional[np.ndarray]:
        """Joint indices of the position targets in ``action``, or ``None`` if it has none."""
        jt = getattr(action, "joint_target", None)
        if jt is None or getattr(action, "parts", None):
            return None
        names = getattr(action, "joint_names", None)
        if names is not None:
            known = list(embodiment.joint_names)
            return np.array([known.index(name) for name in names], dtype=int)
        mode = getattr(getattr(action, "mode", None), "value", None)
        if mode not in _POSITION_MODES:
            return None
        return np.arange(embodiment.num_dofs)

    def _limit(self, target: np.ndarray, ref: np.ndarray, limit: Limit, key: str,
               what: str, idx: np.ndarray, robot: Any) -> np.ndarray:
        per_joint = isinstance(limit, np.ndarray)
        lim = limit[idx] if per_joint else np.full(len(idx), float(limit))
        known = ~np.isnan(ref)
        delta = np.where(known, target - ref, 0.0)
        over = np.abs(delta) > lim + _TOLERANCE
        if not np.any(over):
            return target
        if self.clamp:
            return np.where(over, ref + np.clip(delta, -lim, lim), target)
        names = list(robot.embodiment.joint_names)
        if not per_joint:
            worst = int(np.argmax(np.abs(delta)))
            raise SafetyError(
                f"{int(over.sum())} joint target(s) move more than {key} {float(limit):.3f} rad "
                f"{what}; largest: {names[int(idx[worst])]!r} by {delta[worst]:+.3f} rad")
        worst = int(np.argmax(np.where(over, np.abs(delta) - lim, -np.inf)))
        raise SafetyError(
            f"{int(over.sum())} joint target(s) move more than {key} {what}; largest: "
            f"{names[int(idx[worst])]!r} by {delta[worst]:+.3f} rad, limit {lim[worst]:.3f} rad")

    def _check_status(self, robot: Any) -> None:
        now = time.perf_counter()
        if self._status is None or now - self._status[0] > STATUS_PERIOD_S:
            self._status = (now, robot.get_status())
        status = self._status[1]
        battery = getattr(status, "battery", None)
        soc = getattr(battery, "soc", None)
        if self.min_battery_soc is not None and soc is not None and soc < self.min_battery_soc:
            raise SafetyError(f"battery at {soc:.0f} %, below min_battery_soc "
                              f"{self.min_battery_soc:.0f} %; hold() still works")
        temps = getattr(status, "joint_temperature", None)
        if self.max_joint_temperature is not None and temps is not None:
            temps = np.asarray(temps, dtype=float)
            hot = np.nan_to_num(temps, nan=-np.inf) > self.max_joint_temperature
            if np.any(hot):
                names = list(robot.embodiment.joint_names)
                worst = int(np.nanargmax(temps))
                name = names[worst] if worst < len(names) else str(worst)
                raise SafetyError(f"joint {name!r} at {temps[worst]:.0f} degC, above "
                                  f"max_joint_temperature {self.max_joint_temperature:.0f} degC; "
                                  "hold() still works")


#: Keys a lease's envelope may set (``robot.claim(..., envelope=)``).
ENVELOPE_KEYS = ("joint_limit", "max_joint_step", "max_joint_error", "gripper_range",
                 "allowed_modes")


def envelope_gate(base: Optional["SafetyGate"], envelope: Any, robot: Any,
                  lease_name: str, joints: Optional[Sequence[int]] = None,
                  action_modes: Optional[Sequence[Any]] = None
                  ) -> Tuple[Optional["SafetyGate"], dict]:
    """The extra gate a lease's commands clear, and the limits it enforces.

    Only items stricter than ``base`` are kept: a looser one is dropped, with
    one warning for the claim. The extra gate runs after ``base``, so the
    stricter of the two always applies. Returns ``(None, {})`` when nothing in
    the envelope is stricter. Raises ``ValueError`` for an unknown key or a
    value that cannot be used.

    ``joints`` (indices) are the joints of a lease on some parts only: a
    joint limit is compared with ``base`` on those joints, and a list may
    have one value per lease joint instead of one per robot joint (the other
    joints get ``base``'s value, or no limit). ``action_modes`` are the modes
    the lease's parts take (default: the robot's).
    """
    import logging

    from hirobot.core.types import ControlMode

    env = dict(envelope or {})
    unknown = sorted(set(env) - set(ENVELOPE_KEYS))
    if unknown:
        raise ValueError(f"envelope: unknown key(s) {unknown}; it takes {list(ENVELOPE_KEYS)}")
    looser: list = []
    kept: dict = {}
    num_dofs = getattr(getattr(robot, "embodiment", None), "num_dofs", None)
    for key in ("joint_limit", "max_joint_step", "max_joint_error"):
        if env.get(key) is None:
            continue
        current = getattr(base, key, None)
        if joints is None:
            value = _limit_value(f"envelope.{key}", env[key], num_dofs)
            if current is not None and np.all(np.asarray(value) >= np.asarray(current)):
                looser.append(f"{key} {_fmt(value)} (robot: {_fmt(current)})")
            else:
                kept[key] = value
            continue
        value = _part_limit(f"envelope.{key}", env[key], joints, num_dofs)
        if current is not None and np.all(value[joints] >= np.broadcast_to(
                np.asarray(current, dtype=float), (num_dofs,))[joints]):
            looser.append(f"{key} {_fmt(value[joints])} on the lease's joints "
                          f"(robot: {_fmt(current)})")
            continue
        kept[key] = value  # no limit (inf) on the joints outside the lease
    if env.get("gripper_range") is not None:
        kept_range = _envelope_gripper(env["gripper_range"], base, looser)
        if kept_range is not None:
            kept["gripper_range"] = kept_range
    if env.get("allowed_modes") is not None:
        try:
            modes = tuple(ControlMode(m) for m in env["allowed_modes"])
        except ValueError as exc:
            raise ValueError(f"envelope.allowed_modes: {exc}") from None
        takes = tuple(robot.action_modes if action_modes is None else action_modes)
        usable = tuple(m for m in takes if m in modes)
        if not usable:
            raise ValueError(f"envelope.allowed_modes {[m.value for m in modes]}: the robot takes "
                             f"none of them; it takes {[m.value for m in takes]}")
        if set(usable) != set(takes):
            kept["allowed_modes"] = usable
    if looser:
        logging.getLogger(__name__).warning(
            "lease %r: envelope item(s) not stricter than the robot's gate, ignored: %s",
            lease_name, "; ".join(looser))
    if not kept:
        return None, {}
    gate = SafetyGate(
        num_dofs=num_dofs, joint_limit=kept.get("joint_limit"),
        clamp=bool(getattr(base, "clamp", False)), gripper_range=kept.get("gripper_range"),
        max_joint_step=kept.get("max_joint_step"), max_joint_error=kept.get("max_joint_error"))
    gate.allowed_modes = kept.get("allowed_modes")
    gate.lease_name = lease_name
    report = {k: _report(k, v) for k, v in kept.items()}
    return gate, report


def _report(key: str, value: Any) -> Any:
    """An envelope item as plain Python (lists, floats) for the lease and the audit."""
    if key == "allowed_modes":
        return [m.value for m in value]
    if key == "gripper_range":
        if isinstance(value, tuple):
            return list(value)
        return {name: list(bounds) for name, bounds in value.items()}
    if isinstance(value, np.ndarray):
        return [float(v) for v in value]
    return value


def _fmt(value: Any) -> str:
    if isinstance(value, np.ndarray):
        return "[" + ", ".join(f"{v:g}" for v in value) + "]"
    return f"{value:g}"


def _envelope_gripper(value: Any, base: Optional["SafetyGate"], looser: list) -> Any:
    """The part of an envelope's ``gripper_range`` stricter than ``base``, or ``None``."""
    wanted = _gripper_range(value, "envelope.gripper_range")
    if isinstance(wanted, tuple):
        if base is None or base.gripper_range is None:
            return wanted
        if isinstance(base.gripper_range, tuple):
            return _tighter_range("", wanted, base.gripper_range, looser)
        # One range for every gripper over a base with one range per gripper:
        # keep it whole. The envelope gate clamps after the base gate, so each
        # listed gripper ends up in the intersection and the others in ``wanted``.
        return wanted
    kept: Dict[str, Tuple[float, float]] = {}
    for name, bounds in wanted.items():
        current = base.gripper_bounds(name) if base is not None else None
        tight = bounds if current is None else _tighter_range(f"[{name!r}]", bounds, current,
                                                              looser)
        if tight is not None:
            kept[name] = tight
    return kept or None


def _tighter_range(label: str, wanted: Tuple[float, float], current: Tuple[float, float],
                   looser: list) -> Optional[Tuple[float, float]]:
    low, high = wanted
    if low < current[0] or high > current[1]:
        looser.append(f"gripper_range{label} {[low, high]} (robot: {list(current)})")
    low, high = max(low, current[0]), min(high, current[1])
    return None if (low, high) == tuple(current) else (low, high)


def _part_limit(key: str, value: Any, joints: Sequence[int], num_dofs: int) -> np.ndarray:
    """An envelope limit for the ``joints`` of a lease on some parts, as one value per
    robot joint (``inf`` on the others). A list has one value per lease joint or per robot joint.
    """
    idx = np.asarray(joints, dtype=int)
    out = np.full(num_dofs, np.inf)
    if np.ndim(value) == 0:
        out[idx] = _positive(key, value)
        return out
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 1 and arr.shape[0] == len(idx) != num_dofs:
        if not np.all(arr > 0):
            raise ValueError(f"{key} must be > 0 for every joint, got {list(arr)}")
        out[idx] = arr
        return out
    full = _limit_value(key, value, num_dofs)
    out[idx] = np.asarray(full)[idx]
    return out


def _limit_value(key: str, value: Any, num_dofs: Optional[int]) -> Optional[Limit]:
    """A joint limit from config: a positive number, or one positive number per joint."""
    if value is None:
        return None
    if np.ndim(value) == 0:
        return _positive(key, value)
    arr = np.asarray(value, dtype=float)
    if arr.ndim != 1:
        raise ValueError(f"{key} must be a number or a list of numbers, got {value!r}")
    if num_dofs is None:
        raise ValueError(f"{key} as a list needs the joint count (num_dofs)")
    if arr.shape[0] != num_dofs:
        raise ValueError(f"{key} has {arr.shape[0]} values for {num_dofs} joints")
    if not np.all(arr > 0):
        raise ValueError(f"{key} must be > 0 for every joint, got {list(arr)}")
    return arr


def _gripper_range(value: Any, key: str = "safety.gripper_range") -> Any:
    """``[low, high]`` as a tuple, or ``{name: [low, high]}`` as a dict of tuples."""
    if value is None:
        return None
    if isinstance(value, Mapping):
        return {str(name): _bounds(f"{key}[{name!r}]", b) for name, b in value.items()}
    return _bounds(key, value)


def _bounds(key: str, value: Any) -> Tuple[float, float]:
    try:
        low, high = (float(v) for v in value)
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be [low, high], got {value!r}") from None
    if not low <= high:
        raise ValueError(f"{key} must be [low, high], got {[low, high]}")
    return low, high


#: Seconds a battery / temperature reading is reused before ``get_status`` is read again.
STATUS_PERIOD_S = 1.0

_POSITION_MODES = ("position", "pd")

#: Slack (rad) on the step and error limits: a step of exactly the limit,
#: computed in floating point, passes.
_TOLERANCE = 1e-9


def _positive(key: str, value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    value = float(value)
    if not value > 0:
        raise ValueError(f"{key} must be > 0, got {value}")
    return value


def _non_finite(action: Any, joints: bool = False) -> str:
    """Name of the first numeric command field holding NaN / inf, or ``""``.

    With ``joints`` the joint targets, gains and gripper commands are checked too
    (the main check covers them itself).
    """
    fields = ("base_velocity", "base_target", "latent")
    if joints:
        fields += ("joint_target", "kp", "kd")
        for name, cmd in dict(getattr(action, "gripper", {}) or {}).items():
            if not np.all(np.isfinite(np.asarray(cmd, dtype=float))):
                return f"gripper[{name!r}]"
    for field in fields:
        value = getattr(action, field, None)
        if value is not None and not np.all(np.isfinite(np.asarray(value, dtype=float))):
            return field
    for name, pose in dict(getattr(action, "ee_targets", {}) or {}).items():
        if not np.all(np.isfinite(np.asarray(pose, dtype=float))):
            return f"ee_targets[{name!r}]"
    motion = getattr(action, "motion", None)
    if motion is not None:
        for field, arr in motion.arrays().items():
            if not np.all(np.isfinite(arr)):
                return f"motion.{field}"
    for m, sub in dict(getattr(action, "parts", {}) or {}).items():
        bad = _non_finite(sub, joints=True)
        if bad:
            return f"parts[{m!r}].{bad}"
    return ""
