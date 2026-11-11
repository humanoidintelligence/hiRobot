"""The parts of an assembly node that are robots of their own.

:class:`NodeParts` is what :class:`~hirobot.assembly.RobotAssembly` holds
when it has child parts (an ``assembly:`` config with nodes in it, such as two
UR5e arms and a scene camera). It holds no command logic of the assembly's
own and is not a :class:`~hirobot.core.robot.Robot`; the assembly calls it.

**Names.** A child part's joints, end effectors, cameras, microphones and
speakers are ``<part>/<name>`` in the assembly (``left/shoulder_pan``). The
node's own body (if it has one) keeps its names. Joints are concatenated: the
own body's first, then each child in order.

**Observations** concatenate the joint vectors and prefix every keyed field
(``ee_poses``, ``gripper``, ``images``, ``depth``, ``stamps``, ``extra``):
``stamps["left/body"]`` is when part ``left``'s body state arrived. With
exactly one floating-base part its base signals stay in the top-level fields;
with more they go to ``extra["<part>/base_*"]``.

**Frames.** ``ee_poses`` and ``ee_targets`` are in the node's frame. A child
placed with ``pose`` (fixed) is converted with it; a moving child (``origin``)
with ``origin ∘`` its current odometry (``base_position`` /
``base_orientation``). While a moving child reports no ``base_position`` its
poses go to ``extra["<part>/ee_poses"]`` and its end effectors drop out of
``cartesian_targets``. A child without a placement is at the node's origin.

**Actions** are flat (one mode for every part) or per part (``Action.parts``:
part name -> its own action, own names, own mode). In a flat action
``joint_target`` / ``kp`` / ``kd`` are split by the parts' joint counts (or by
name with ``joint_names``), ``ee_targets`` / ``gripper`` by name prefix.
``base_velocity`` / ``base_target`` / ``latent`` / ``motion`` go to the one
floating-base part. A part the action does not mention gets nothing and keeps
its last target; its simulation (if it has one) still advances one period.

**Checks before sending.** Every part's mode and ``direct_joints`` are checked
on its piece, then the collision check runs (when configured); if any refuses,
nothing is sent to any part. Then the pieces go out one part after another.

**Errors.** When a part fails to read or send, or reports a fault, the
assembly holds every other part, enters its own fault state and re-raises. It
refuses commands until :meth:`~hirobot.core.robot.Robot.reset`.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

from hirobot.assembly.parts import BASE_COMMANDS, BASE_FIELDS, SEP, frame_targets, split_name
from hirobot.control.safety import SafetyError
from hirobot.core.stamps import Stamp
from hirobot.geometry.transforms import IDENTITY_POSE, pose_compose
from hirobot.core.types import JOINT_MODES, Action, ControlMode, Observation

if TYPE_CHECKING:
    from hirobot.geometry.collision import CollisionCheck
    from hirobot.core.robot import Robot

logger = logging.getLogger(__name__)

#: Key of the node's own body in the joint layout and the frame tables.
OWN = ""


def _pose(value: Any, where: str) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{where} must be a finite [x, y, z, qx, qy, qz, qw]")
    norm = np.linalg.norm(pose[3:])
    if norm < 1e-9:
        raise ValueError(f"{where}: the quaternion is zero")
    return np.concatenate([pose[:3], pose[3:] / norm])


class NodeParts:
    """The child robots of an assembly node, their joint ranges and frames.

    Args:
        owner: The :class:`~hirobot.assembly.RobotAssembly` they belong to.
        children: Part name -> robot, in order.
        own: The node's own body robot (with its end effectors), or ``None``.
        placement: Part name -> ``{"pose": [...]}`` (fixed in the node) or
            ``{"origin": [...]}`` (moves by itself; its odometry origin).
        collision: Distance check between parts, or ``None``.
    """

    def __init__(self, owner: "Robot", children: Mapping[str, "Robot"], own: Optional["Robot"],
                 placement: Optional[Mapping[str, Mapping[str, Any]]] = None,
                 collision: Optional["CollisionCheck"] = None) -> None:
        self.owner = owner
        self.children: Dict[str, Robot] = dict(children)
        self.own = own
        #: Every part with joints, the own body under ``OWN``, in joint order.
        self.robots: Dict[str, Robot] = ({OWN: own} if own is not None else {})
        self.robots.update(self.children)
        self.fixed: Dict[str, np.ndarray] = {}
        self.origin: Dict[str, np.ndarray] = {}
        for m, entry in dict(placement or {}).items():
            if m not in self.children:
                raise ValueError(f"placement: no child part {m!r}; child parts: "
                                 f"{list(self.children)}")
            entry = dict(entry or {})
            unknown = sorted(set(entry) - {"pose", "origin"})
            if unknown or len(entry) > 1:
                raise ValueError(f"placement[{m!r}] is {{pose: ...}} or {{origin: ...}}, got "
                                 f"{entry!r}")
            floating = self.children[m].embodiment.floating_base
            if "pose" in entry:
                if floating:
                    raise ValueError(f"placement[{m!r}]: {m!r} moves by itself (floating base); "
                                     "give its odometry origin (origin:), not pose:")
                self.fixed[m] = _pose(entry["pose"], f"placement[{m!r}].pose")
            elif "origin" in entry:
                if not floating:
                    raise ValueError(f"placement[{m!r}]: origin: is for a part that moves by "
                                     "itself; use pose:")
                self.origin[m] = _pose(entry["origin"], f"placement[{m!r}].origin")
        for m, r in self.children.items():
            if r.embodiment.floating_base and m not in self.origin:
                self.origin[m] = IDENTITY_POSE.copy()
        #: Last odometry (``[x, y, z, qx, qy, qz, qw]``) seen per moving child; ``None``
        #: while it reports no base_position.
        self._odom: Dict[str, Optional[np.ndarray]] = {}
        # Where each part's joints sit in the assembly's joint vector, by name
        # (the own body's joints need not come first).
        index = {n: i for i, n in enumerate(owner.embodiment.joint_names)}
        self.ranges: Dict[str, np.ndarray] = {}
        for m, r in self.robots.items():
            names = [self.out_name(m, j) for j in r.embodiment.joint_names]
            missing = [n for n in names if n not in index]
            if missing:
                raise ValueError(f"the assembly's embodiment has no joint(s) {missing} of part "
                                 f"{self._label(m)}")
            self.ranges[m] = np.array([index[n] for n in names], dtype=int)
        if sum(len(ix) for ix in self.ranges.values()) != owner.embodiment.num_dofs:
            raise ValueError("the assembly's joints must be exactly its parts' joints")
        self.floating = [m for m, r in self.robots.items() if r.embodiment.floating_base]
        if collision is not None:
            unknown = sorted(set(collision.models) - set(self.children))
            if unknown:
                raise ValueError(f"collision: no child part(s) {unknown}; parts: "
                                 f"{list(self.children)}")
        self.collision = collision
        self._skew_warn_s = 0.5 / max(1, int(owner.embodiment.control_hz))
        self._last_skew_warning = -1e9

    # --------------------------------------------------------------- names

    def out_name(self, m: str, name: str) -> str:
        return name if m == OWN else f"{m}{SEP}{name}"

    def owner_of(self, name: str, what: str) -> Tuple[str, str]:
        """``(part, local name)`` of a name in the assembly."""
        if SEP in name:
            m, local = split_name(name)
            if m in self.children:
                return m, local
            if self.own is None:
                raise SafetyError(f"{what} {name!r}: no part {m!r}; parts: {list(self.children)}")
        if self.own is None:
            raise SafetyError(f"{what} {name!r} is not a part name: expected '<part>{SEP}<name>'")
        return OWN, name

    # -------------------------------------------------------------- frames

    def frame(self, m: str) -> Optional[np.ndarray]:
        """Pose of part ``m``'s frame in the node, or ``None`` while unknown."""
        if m == OWN:
            return IDENTITY_POSE
        if m in self.fixed:
            return self.fixed[m]
        if m not in self.origin:
            return IDENTITY_POSE
        if m not in self._odom:
            self.remember(m, self.children[m].get_observation())
        odom = self._odom[m]
        return None if odom is None else pose_compose(self.origin[m], odom)

    def remember(self, m: str, obs: Observation) -> None:
        if m not in self.origin:
            return
        if obs.base_position is None or obs.base_orientation is None:
            self._odom[m] = None
        else:
            self._odom[m] = np.concatenate([np.asarray(obs.base_position, dtype=float)[:3],
                                            np.asarray(obs.base_orientation, dtype=float)])

    def lost(self, m: str) -> bool:
        """Whether moving part ``m`` reported no odometry last time it was read."""
        return m in self._odom and self._odom[m] is None

    # ----------------------------------------------------------- observation

    def observe(self) -> Tuple[Observation, Dict[str, float]]:
        """Read every part and merge (raises after stopping the rest on a failure)."""
        parts: Dict[str, Observation] = {}
        read_at: Dict[str, float] = {}
        for m, robot in self.robots.items():
            try:
                parts[m] = robot.get_observation()
            except Exception as exc:
                self.owner._fail_parts(m, f"part {self._label(m)} get_observation failed: {exc!r}")
                raise
            read_at[m] = time.monotonic()
            self.remember(m, parts[m])
        obs = Observation()
        n = self.owner.embodiment.num_dofs
        for field in ("joint_pos", "joint_vel", "joint_torque"):
            if all(getattr(p, field) is None for p in parts.values()):
                continue
            full = np.full(n, np.nan)
            for m, p in parts.items():
                value = getattr(p, field)
                if value is not None:
                    full[self.ranges[m]] = value
            setattr(obs, field, full)
        for m, p in parts.items():
            if len(self.floating) == 1 and m == self.floating[0]:
                for field in BASE_FIELDS:
                    setattr(obs, field, getattr(p, field))
            else:
                for field in BASE_FIELDS:
                    if getattr(p, field) is not None:
                        obs.extra[self.out_name(m, field)] = getattr(p, field)
            frame = self.frame(m)
            if frame is None:
                if p.ee_poses:
                    obs.extra[self.out_name(m, "ee_poses")] = dict(p.ee_poses)
            else:
                identity = np.array_equal(frame, IDENTITY_POSE)
                for k, v in p.ee_poses.items():
                    obs.ee_poses[self.out_name(m, k)] = (np.asarray(v, dtype=float) if identity
                                                         else pose_compose(frame, v))
            for key in ("gripper", "images", "depth", "stamps"):
                target = getattr(obs, key)
                for k, v in getattr(p, key).items():
                    target[self.out_name(m, k)] = v
            obs.extra.update({self.out_name(m, k): v for k, v in p.extra.items()})
            if obs.instruction is None:
                obs.instruction = p.instruction
        if len(read_at) > 1:
            skew = max(read_at.values()) - min(read_at.values())
            if skew > self._skew_warn_s and time.monotonic() - self._last_skew_warning > 5.0:
                self._last_skew_warning = time.monotonic()
                logger.warning("assembly read its parts %.1f ms apart (more than %.1f ms)",
                               skew * 1e3, self._skew_warn_s * 1e3)
        return obs, read_at

    def _label(self, m: str) -> str:
        return "(own body)" if m == OWN else repr(m)

    # -------------------------------------------------------------- actions

    def split(self, action: Action) -> Dict[str, Action]:
        """Each part's piece of ``action``, in the part's own names and frame.

        A flat action is split by joint ranges and names; ``Action.parts`` is
        taken as given (an entry naming one of the node's own devices goes to
        the own body). Node-frame ``ee_targets`` are converted to each part's
        frame; a part action with ``extra["frame"] == "base"`` is already in
        the part's frame.

        Raises:
            SafetyError: A name with no part, a vector of the wrong length, a
                base command with no single floating-base part to take it, or
                a target for a part whose position is unknown now.
        """
        if action.parts:
            return self._split_parts(action)
        fields: Dict[str, Dict[str, Any]] = {m: {} for m in self.robots}
        n = self.owner.embodiment.num_dofs
        if action.joint_target is not None and action.joint_names is None:
            target = np.asarray(action.joint_target, dtype=float)
            if target.shape != (n,):
                raise SafetyError(f"joint_target has shape {target.shape}, the robot has {n} "
                                  "joints")
            for m in self.robots:
                fields[m]["joint_target"] = target[self.ranges[m]]
                fields[m]["kp"] = self._slice(action.kp, m)
                fields[m]["kd"] = self._slice(action.kd, m)
        elif action.joint_names is not None:
            target = np.asarray(action.joint_target, dtype=float)
            by_part: Dict[str, List[int]] = {}
            locals_: Dict[str, List[str]] = {}
            for i, name in enumerate(action.joint_names):
                m, local = self.owner_of(name, "joint")
                by_part.setdefault(m, []).append(i)
                locals_.setdefault(m, []).append(local)
            for m, idx in by_part.items():
                ix = np.array(idx, dtype=int)
                fields[m]["joint_names"] = locals_[m]
                fields[m]["joint_target"] = target[ix]
                for gain in ("kp", "kd"):
                    g = getattr(action, gain)
                    if g is not None:
                        g = np.asarray(g, dtype=float)
                        fields[m][gain] = g[ix] if g.ndim == 1 else g
        for key in ("ee_targets", "gripper"):
            for name, value in getattr(action, key).items():
                m, local = self.owner_of(name, "end-effector")
                fields[m].setdefault(key, {})[local] = value
        base = [f for f in BASE_COMMANDS if getattr(action, f) is not None]
        if base:
            if len(self.floating) != 1:
                raise SafetyError(
                    f"{base} need exactly one floating-base part to go to; this robot has "
                    f"{[self._label(m) for m in self.floating] or 'none'}; use Action.parts")
            for f in base:
                fields[self.floating[0]][f] = getattr(action, f)
        pieces = {}
        for m, f in fields.items():
            if not f:
                continue
            if "ee_targets" in f:
                f["ee_targets"] = frame_targets(self.frame(m), action.mode, f["ee_targets"],
                                                f"part {self._label(m)}")
            f.update(mode=action.mode, append=action.append, extra=dict(action.extra))
            pieces[m] = Action(**f)
        if not pieces:
            raise SafetyError("the action commands no part")
        return pieces

    def _split_parts(self, action: Action) -> Dict[str, Action]:
        pieces: Dict[str, Action] = {}
        for m, sub in action.parts.items():
            if m in self.children:
                target = m
            elif self.own is not None:
                target = OWN  # one of the node's own devices: the own body takes it
            else:
                raise SafetyError(f"Action.parts: no part {m!r}; parts: {list(self.children)}")
            if target == OWN:
                if sub.parts:
                    raise SafetyError(f"parts[{m!r}] is a device of this node; it has no parts")
                if OWN in pieces:
                    raise SafetyError("Action.parts: more than one entry for this node's own "
                                      "devices; write them as one entry")
            if sub.extra.get("frame") != "base" and sub.ee_targets:
                sub = sub.model_copy(update={"ee_targets": frame_targets(
                    self.frame(target), sub.mode, sub.ee_targets, f"part {self._label(target)}")})
            pieces[target] = sub
        if not pieces:
            raise SafetyError("the action commands no part")
        return pieces

    def _slice(self, values: Optional[np.ndarray], m: str) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values, dtype=float)
        return arr[self.ranges[m]] if arr.ndim == 1 else arr  # a scalar applies to all

    def check(self, pieces: Mapping[str, Action]) -> Dict[str, Action]:
        """Check every piece against its part; :class:`SafetyError` and nothing sent if one fails."""
        from hirobot.core.robot import RobotFaultError

        for m, robot in self.robots.items():
            fault = robot.fault
            if fault is not None:
                reason = f"part {self._label(m)} fault: {fault}"
                self.owner._fail_parts(m, reason)
                raise RobotFaultError(reason)
        checked: Dict[str, Action] = {}
        for m, piece in pieces.items():
            robot = self.robots[m]
            if piece.parts and not robot.accepts_parts:
                raise SafetyError(f"part {self._label(m)} has no parts of its own")
            if not piece.parts:
                if piece.mode not in robot.action_modes:
                    raise SafetyError(f"part {self._label(m)} does not take mode "
                                      f"{piece.mode.value!r}; it takes "
                                      f"{[x.value for x in robot.action_modes]}")
                if piece.mode not in JOINT_MODES and piece.joint_target is not None \
                        and piece.joint_names is None:
                    raise SafetyError(f"part {self._label(m)}: mode {piece.mode.value!r} needs "
                                      "joint_names for joint targets")
            try:
                # The part's own checks: fault state, direct_joints, and its
                # gate when it was built with one (built by hand).
                piece = robot.check_action(piece)
                if robot.accepts_parts:
                    # A node inside: check its whole subtree now, send later.
                    piece = robot._node.check(robot._node.split(piece))  # type: ignore[attr-defined]
            except SafetyError as exc:
                raise SafetyError(f"part {self._label(m)}: {exc}") from exc
            checked[m] = piece
        if self.collision is not None:
            self._check_collision(checked)
        return checked

    def send(self, checked: Mapping[str, Any]) -> None:
        """Send checked pieces one part after another, then idle the parts not mentioned."""
        for m, piece in checked.items():
            robot = self.robots[m]
            try:
                if isinstance(piece, dict):
                    robot._send_node_checked(piece)  # type: ignore[attr-defined]
                else:
                    robot._send_checked(piece)
            except Exception as exc:
                self.owner._fail_parts(m, f"part {self._label(m)} send failed: {exc!r}")
                raise
        for m, robot in self.robots.items():
            if m in checked:
                continue
            try:
                robot._idle_step()  # its simulation advances with the others
            except Exception as exc:
                self.owner._fail_parts(m, f"part {self._label(m)} _idle_step failed: {exc!r}")
                raise

    def _check_collision(self, checked: Mapping[str, Any]) -> None:
        """Refuse ``checked`` if it brings two parts closer than allowed.

        The predicted gap (joint targets as the next angles, Cartesian target
        points as spheres) is compared with the current one; below
        ``min_distance`` the action passes only if the gap does not shrink.
        """
        check = self.collision
        now: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        nxt: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        points: Dict[str, np.ndarray] = {}
        for m, model in check.models.items():
            obs = self.children[m].get_observation()
            self.remember(m, obs)
            frame = self.frame(m)
            if frame is None or obs.joint_pos is None:
                raise SafetyError(f"collision check: part {m!r} has no known position or joint "
                                  "positions now; nothing sent")
            q = np.asarray(obs.joint_pos, dtype=float)
            now[m] = model.segments(q, frame)
            piece = checked.get(m)
            q_next = q
            if isinstance(piece, Action) and piece.mode in (ControlMode.POSITION, ControlMode.PD) \
                    and piece.joint_target is not None:
                target = np.asarray(piece.joint_target, dtype=float)
                if piece.joint_names is None:
                    q_next = target
                else:
                    q_next = q.copy()
                    names = self.children[m].embodiment.joint_names
                    for name, v in zip(piece.joint_names, target):
                        q_next[names.index(name)] = v
            nxt[m] = model.segments(q_next, frame)
            if isinstance(piece, Action) and piece.mode == ControlMode.CARTESIAN \
                    and piece.ee_targets:
                points[m] = np.array([pose_compose(frame, np.asarray(v, dtype=float))[:3]
                                      for v in piece.ee_targets.values()])
        after = check.gap(nxt, points)
        if after is None or after.distance >= check.min_distance:
            return
        before = check.gap(now)
        if before is not None and after.distance >= before.distance - 1e-6:
            return  # already too close, and this does not make it worse
        raise SafetyError(
            f"collision check: {after.members[0]}/{after.bodies[0]} and "
            f"{after.members[1]}/{after.bodies[1]} would be {after.distance * 100:.1f} cm apart "
            f"(minimum {check.min_distance * 100:.1f} cm); nothing sent")

    # ---------------------------------------------------------------- cameras

    @staticmethod
    def restamp(obs: Observation, names: Any, now: float) -> None:
        for name in names:
            stamp = obs.stamps.get(name)
            if stamp is not None:
                obs.stamps[name] = Stamp(now, stamp.seq)


__all__ = ["NodeParts", "OWN"]
