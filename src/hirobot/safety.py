"""The safety gate: last check before any action reaches the robot.

A policy (or a VLA model it calls) can emit an action that is malformed or out
of range. On hardware that is how things break. The :class:`SafetyGate` runs on
every action inside :meth:`~hirobot.robot.Robot.send_action`, like a permission
check for motor commands. It is intentionally simple and conservative: validate
shape and bounds, and refuse what it cannot make safe.

Enforcement lives at the robot boundary, not in the caller. ``send_action``
applies the gate itself and raises :class:`SafetyError` on a refusal, so every
consumer that drives a robot — the control loop, eval scripts, or an agent's
tools — is held to the same check through one choke point and cannot skip it by
forgetting to call the gate. By default a refusal *raises* rather than silently
rewriting the command (``clamp=False``); set ``clamp=True`` for robots that
genuinely want out-of-range joint targets clipped into bounds.

This is where to add workspace limits, velocity caps, or an e-stop interlock.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


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


class SafetyGate:
    """Validate (and optionally clamp) actions before they reach the robot.

    Args:
        num_dofs: Expected joint-target length; ``None`` skips the length check.
        joint_limit: Symmetric per-joint position bound (radians), or ``None`` to
            skip the bound check. Targets outside ``[-joint_limit, joint_limit]``
            are clamped when ``clamp`` is set, otherwise the action is refused.
        clamp: Clamp out-of-range joint targets instead of refusing them. Off by
            default: an out-of-range command is refused (and ``send_action``
            raises) rather than silently rewritten.
        gripper_range: Allowed ``[low, high]`` for gripper commands, or ``None``
            to skip the gripper check; values are clamped into this range.
    """

    def __init__(
        self,
        num_dofs: Optional[int] = None,
        joint_limit: Optional[float] = None,
        clamp: bool = False,
        gripper_range: Optional[tuple] = None,
    ) -> None:
        self.num_dofs = num_dofs
        self.joint_limit = None if joint_limit is None else float(joint_limit)
        self.clamp = clamp
        self.gripper_range = (
            None if gripper_range is None
            else (float(gripper_range[0]), float(gripper_range[1]))
        )

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
        members = getattr(action, "members", None)
        if members:
            # A robot group's per-member action: each member's own gate checks
            # its part (shape, limits); here only that every number is finite.
            for m, sub in members.items():
                bad = _non_finite(sub, joints=True)
                if bad:
                    return SafetyVerdict(False, reason=f"members[{m!r}].{bad} contains "
                                                       "non-finite values")
            return SafetyVerdict(True, action=action)
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
                over = np.abs(jt) > self.joint_limit
                if np.any(over):
                    if not self.clamp:
                        return SafetyVerdict(
                            False,
                            reason=f"{int(over.sum())} joint target(s) exceed "
                            f"±{self.joint_limit:.3f} rad",
                        )
                    jt = np.clip(jt, -self.joint_limit, self.joint_limit)
            action.joint_target = jt

        for name, cmd in dict(getattr(action, "gripper", {}) or {}).items():
            val = np.asarray(cmd, dtype=float)
            if not np.all(np.isfinite(val)):
                return SafetyVerdict(False, reason=f"gripper {name!r} has non-finite value")
            if self.gripper_range is not None:
                action.gripper[name] = np.clip(val, self.gripper_range[0], self.gripper_range[1])

        return SafetyVerdict(True, action=action)


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
    for m, sub in dict(getattr(action, "members", {}) or {}).items():
        bad = _non_finite(sub, joints=True)
        if bad:
            return f"members[{m!r}].{bad}"
    return ""
