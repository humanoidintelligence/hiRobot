"""Helpers for robots made of parts: names, embodiments, frames, read-only views.

A :class:`~hirobot.assembly.RobotAssembly` with child robots (parts that are
robots of their own, from an ``assembly:`` config) names their joints, end
effectors, cameras, microphones and speakers ``<part>/<name>``. This module
holds what does not depend on the assembly's state: splitting and joining
those names, the embodiment of a node built from its children
(:func:`merge_embodiments`), moving targets between frames, the leaves of an
``Action.parts`` tree (:func:`action_leaves`, used by the safety gate) and
:class:`PartView`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from hirobot.core.embodiment import Camera, Embodiment, EmbodimentKind, EmbodimentPart
from hirobot.control.safety import SafetyError
from hirobot.geometry.transforms import IDENTITY_POSE, pose_compose, pose_inverse, quat_conjugate, rotate
from hirobot.core.types import ControlMode, Observation, RobotStatus

if TYPE_CHECKING:
    from hirobot.core.robot import Robot
    from hirobot.core.types import Action

SEP = "/"

#: Observation fields of a floating base.
BASE_FIELDS = ("base_orientation", "base_angular_velocity", "base_linear_acceleration",
               "base_position", "base_linear_velocity")
#: Action fields that only a part with a floating base can take.
BASE_COMMANDS = ("base_velocity", "base_target", "latent", "motion")


def split_name(name: str) -> Tuple[str, str]:
    """``("left", "j1")`` for ``"left/j1"``; :class:`SafetyError` without a part prefix."""
    part, sep, local = name.partition(SEP)
    if not sep or not part or not local:
        raise SafetyError(f"{name!r} is not a part name: expected '<part>{SEP}<name>'")
    return part, local


def check_part_name(name: str) -> None:
    """Raise ``ValueError`` unless ``name`` can name a part (non-empty, no ``/`` or ``.``)."""
    if not name or SEP in name or "." in name:
        raise ValueError(f"part name {name!r} must be non-empty and contain no '/' or '.'")


def prefixed(part: str, name: str) -> str:
    """``part/name``; the node's own devices (``part == ""``) keep ``name``."""
    return f"{part}{SEP}{name}" if part else name


def merge_embodiments(name: str, local: Optional[Embodiment], children: Mapping[str, Embodiment],
                      cameras: Sequence[Camera] = (), microphones: Sequence[Any] = (),
                      speakers: Sequence[Any] = (), control_hz: Optional[int] = None) -> Embodiment:
    """The embodiment of a node: its own devices, then each child's parts prefixed.

    Args:
        name: The node's embodiment name.
        local: The embodiment of the node's own body (with its end effectors),
            or ``None``; its names stay as they are.
        children: Child part name -> its embodiment; names get ``<part>/``.
        cameras: The node's own cameras, microphones, speakers.
        control_hz: Default: the slowest of ``local`` and the children.

    Hardware specs (``driver:``) of the children are dropped: the children own
    their devices. ``floating_base`` is true when any part has a floating base.
    ``parts`` lists the node (``""``) and every child node, with their joint ranges.
    """
    for m in children:
        check_part_name(m)
    if local is None and not children:
        raise ValueError("an assembly needs a body or at least one child part")
    joints, ees, cams, mics, spks = [], [], [], [], []
    parts: Dict[str, EmbodimentPart] = {}
    hz: List[int] = []
    floating = False
    if local is not None:
        joints += list(local.joints)
        ees += list(local.end_effectors)
        cams += list(local.cameras)
        mics += list(local.microphones)
        spks += list(local.speakers)
        hz.append(local.control_hz)
        floating = local.floating_base
    for m, emb in children.items():
        start = len(joints)
        pre = f"{m}{SEP}"
        joints += [j.model_copy(update={"name": pre + j.name}) for j in emb.joints]
        ees += [e.model_copy(update={"name": pre + e.name, "driver": None,
                                     "joints": [pre + j for j in e.joints]})
                for e in emb.end_effectors]
        cams += [c.model_copy(update={"name": pre + c.name, "driver": None}) for c in emb.cameras]
        mics += [d.model_copy(update={"name": pre + d.name, "driver": None})
                 for d in emb.microphones]
        spks += [d.model_copy(update={"name": pre + d.name, "driver": None})
                 for d in emb.speakers]
        sub_parts = emb.parts or {"": EmbodimentPart(
            joints=(0, emb.num_dofs), end_effectors=[e.name for e in emb.end_effectors],
            cameras=[c.name for c in emb.cameras], microphones=[d.name for d in emb.microphones],
            speakers=[d.name for d in emb.speakers])}
        for sub, info in sub_parts.items():
            parts[pre + sub if sub else m] = info.model_copy(update={
                "joints": (info.joints[0] + start, info.joints[1] + start),
                "end_effectors": [pre + n for n in info.end_effectors],
                "cameras": [pre + n for n in info.cameras],
                "microphones": [pre + n for n in info.microphones],
                "speakers": [pre + n for n in info.speakers],
                "children": [pre + c for c in info.children]})
        hz.append(emb.control_hz)
        floating = floating or emb.floating_base
    cams += list(cameras)
    mics += list(microphones)
    spks += list(speakers)
    parts[""] = EmbodimentPart(
        joints=(0, len(joints)), end_effectors=[e.name for e in ees], cameras=[c.name for c in cams],
        microphones=[d.name for d in mics], speakers=[d.name for d in spks],
        children=list(children), moving=False)
    return Embodiment(
        name=name, kind=EmbodimentKind.HUMANOID if floating else EmbodimentKind.ARM,
        control_hz=control_hz or min(hz), joints=joints, floating_base=floating,
        end_effectors=ees, cameras=cams, microphones=mics, speakers=spks,
        parts=dict(sorted(parts.items())))


def part_joint_names(embodiment: Embodiment, path: str) -> List[str]:
    """Joint names (in the robot's names) that an ``Action.parts`` leaf at ``path`` covers.

    ``path`` is a child node (``left``, ``cell/left``): every joint under it.
    Any other last component names one of the node's own devices: the node's
    joints that are under none of its child nodes.
    """
    names = embodiment.joint_names
    parts = embodiment.parts
    if path in parts:
        lo, hi = parts[path].joints
        return names[lo:hi]
    node, _, _ = path.rpartition(SEP)
    if node not in parts:
        node = ""
        if SEP in path:
            raise SafetyError(f"Action.parts: no part {path!r}; parts: "
                              f"{sorted(p for p in parts if p)}")
    lo, hi = parts[node].joints
    taken = set()
    for child in parts[node].children:
        clo, chi = parts[child].joints
        taken.update(range(clo, chi))
    own = [names[i] for i in range(lo, hi) if i not in taken]
    if not own:
        children = [c.rpartition(SEP)[2] for c in parts[node].children]
        raise SafetyError(f"Action.parts: no part {path!r}; parts: {children}")
    return own


def leaf_prefix(embodiment: Embodiment, path: str) -> str:
    """Prefix of the names inside an ``Action.parts`` leaf at ``path`` (``left/``, or ``""``)."""
    if path in embodiment.parts:
        return f"{path}{SEP}"
    node, _, _ = path.rpartition(SEP)
    return f"{node}{SEP}" if node else ""


def action_leaves(action: "Action", prefix: str = "") -> List[Tuple[str, "Action"]]:
    """``(path, action)`` for every action in an ``Action.parts`` tree with no parts of its own."""
    if not action.parts:
        return [(prefix.rstrip(SEP), action)]
    out: List[Tuple[str, Action]] = []
    for name, sub in action.parts.items():
        out += action_leaves(sub, f"{prefix}{name}{SEP}")
    return out


def owner_part(embodiment: Embodiment, name: str) -> str:
    """The deepest node (``embodiment.parts`` key) a robot name such as ``left/tcp`` is in."""
    best = ""
    for p in embodiment.parts:
        if p and name.startswith(p + SEP) and len(p) > len(best):
            best = p
    return best


def resolve_part(embodiment: Embodiment, path: str) -> Tuple[str, bool]:
    """``(node, own_only)`` for a part path: a node (its whole subtree), or one of a
    node's own devices (``"base"``, ``"cell/body"``: that node's own robot only).

    Raises:
        ValueError: No such part.
    """
    path = path.strip(SEP)
    if not path or path in embodiment.parts:
        return path, False
    node, _, device = path.rpartition(SEP)
    if node in embodiment.parts and device in embodiment.parts[node].devices:
        return node, True
    try:
        part_joint_names(embodiment, path)  # a device of a node with joints of its own
    except SafetyError as exc:
        raise ValueError(str(exc).replace("Action.parts: ", "")) from None
    return path.rpartition(SEP)[0], True


def scope_joints(embodiment: Embodiment, paths: Sequence[str]) -> List[int]:
    """Indices of the robot joints the part paths cover (a jointless device covers none)."""
    index = {n: i for i, n in enumerate(embodiment.joint_names)}
    out = set()
    for path in paths:
        node, own = resolve_part(embodiment, path)
        if not own:
            out.update(range(*embodiment.parts[node].joints))
            continue
        try:
            out.update(index[n] for n in part_joint_names(embodiment, path.strip(SEP)))
        except SafetyError:
            pass  # a node's own device with no joints of its own (a mobile base, a camera)
    return sorted(out)


def part_covers(part: Tuple[str, bool], node: str) -> bool:
    """Whether a resolved part path covers what node ``node`` itself drives."""
    top, own_only = part
    return node == top or (not own_only and (not top or node.startswith(top + SEP)))


def parts_overlap(a: Tuple[str, bool], b: Tuple[str, bool]) -> bool:
    """Whether two resolved part paths share something."""
    return part_covers(a, b[0]) or part_covers(b, a[0])


def action_owners(embodiment: Embodiment, action: "Action") -> set:
    """The nodes whose own robots ``action`` commands (``""`` is the root's own).

    A joint, end effector or gripper counts for the deepest node it is in. A
    base command goes to the topmost moving node under the node it is sent to,
    else that node's own body. A robot without parts has one node, ``""``.
    """
    if not embodiment.parts:
        return {""}
    index = {n: i for i, n in enumerate(embodiment.joint_names)}
    deepest: Dict[int, str] = {}
    for p, info in sorted(embodiment.parts.items(), key=lambda kv: len(kv[0])):
        for i in range(*info.joints):
            deepest[i] = p
    out: set = set()
    for path, sub in action_leaves(action):
        if action.parts:
            names = _leaf_joints(embodiment, path, sub)
            pre = leaf_prefix(embodiment, path)
            node = path if path in embodiment.parts else None
        else:
            names = (None if sub.joint_target is None else
                     list(sub.joint_names) if sub.joint_names is not None else
                     list(embodiment.joint_names))
            pre, node = "", ""
        for n in names or ():
            if n not in index:
                raise SafetyError(f"unknown joint {n!r}")
            out.add(deepest[index[n]])
        for key in ("ee_targets", "gripper"):
            out.update(owner_part(embodiment, pre + k) for k in getattr(sub, key))
        if any(getattr(sub, f) is not None for f in BASE_COMMANDS):
            if node is None:  # one of a node's own devices: its own body
                out.add(pre.rstrip(SEP))
            else:
                moving = sorted((p for p, info in embodiment.parts.items() if info.moving
                                 and part_covers((node, False), p)), key=len)
                out.add(moving[0] if moving else node)
    return out


def frame_targets(frame: Optional[np.ndarray], mode: ControlMode,
                  targets: Mapping[str, Any], where: str) -> Dict[str, np.ndarray]:
    """``ee_targets`` given in a node's frame, in the frame of a part placed at ``frame``.

    Raises:
        SafetyError: ``frame`` is ``None`` (the part's position is unknown now),
            or a target has the wrong shape.
    """
    if not targets:
        return {}
    if frame is None:
        raise SafetyError(f"{where} reports no base_position, so its position in the assembly is "
                          "unknown; send its ee_targets in its own frame (Action.parts with "
                          "extra={'frame': 'base'})")
    if np.array_equal(frame, IDENTITY_POSE):
        return {k: np.asarray(v, dtype=float) for k, v in targets.items()}
    inv = pose_inverse(frame)
    out = {}
    for name, value in targets.items():
        v = np.asarray(value, dtype=float)
        if mode == ControlMode.CARTESIAN_DELTA:
            if v.shape != (6,):
                raise SafetyError(f"CARTESIAN_DELTA target {name!r} must be 6 values, "
                                  f"got shape {v.shape}")
            q = quat_conjugate(frame[3:])
            out[name] = np.concatenate([rotate(q, v[:3]), rotate(q, v[3:])])
        else:
            if v.shape != (7,):
                raise SafetyError(f"ee target {name!r} must be a 7-vector pose, got shape {v.shape}")
            out[name] = pose_compose(inv, v)
    return out


def _leaf_joints(embodiment: Embodiment, path: str, sub: "Action") -> Optional[List[str]]:
    """Robot joint names of a leaf's ``joint_target`` (``None`` without one)."""
    if sub.joint_target is None:
        return None
    pre = leaf_prefix(embodiment, path)
    if sub.joint_names is not None:
        return [pre + n for n in sub.joint_names]
    names = part_joint_names(embodiment, path)
    n = int(np.asarray(sub.joint_target).reshape(-1).shape[0])
    if n != len(names):
        raise SafetyError(f"parts[{path!r}].joint_target has {n} values, the part has "
                          f"{len(names)} joints")
    return names


def check_parts(gate: Any, action: "Action", embodiment: Embodiment) -> Any:
    """The gate's action checks on an ``Action.parts`` tree, leaf by leaf, in the robot's names.

    Joint targets are checked against ``joint_limit`` for the joints they
    name; gripper commands are clamped by end-effector name. Returns a
    :class:`~hirobot.control.safety.SafetyVerdict` with a copy of ``action``.
    """
    from hirobot.control.safety import SafetyVerdict, _non_finite

    if not embodiment.parts:
        return SafetyVerdict(False, reason="Action.parts is for a robot with parts; this robot "
                                           "has none")
    action = _copy_tree(action)
    known = set(embodiment.joint_names)
    for path, sub in action_leaves(action):
        if sub.parts:
            continue
        bad = _non_finite(sub, joints=True)
        if bad:
            return SafetyVerdict(False, reason=f"parts[{path!r}].{bad} contains non-finite values")
        try:
            names = _leaf_joints(embodiment, path, sub)
        except SafetyError as exc:
            return SafetyVerdict(False, reason=str(exc))
        if names is not None:
            unknown = [n for n in names if n not in known]
            if unknown:
                return SafetyVerdict(False, reason=f"parts[{path!r}]: unknown joint(s) {unknown}")
            jt = np.asarray(sub.joint_target, dtype=float)
            if jt.ndim != 1:
                return SafetyVerdict(False, reason=f"parts[{path!r}].joint_target must be 1-D")
            if gate.joint_limit is not None:
                limit = gate._joint_limit_for(names, embodiment, len(names))
                over = np.abs(jt) > limit
                if np.any(over):
                    if not gate.clamp:
                        return SafetyVerdict(False, reason=gate._exceed_reason(
                            jt, limit, over, names, embodiment,
                            isinstance(gate.joint_limit, np.ndarray)))
                    sub.joint_target = np.clip(jt, -limit, limit)
        pre = leaf_prefix(embodiment, path)
        for name, cmd in dict(sub.gripper).items():
            bounds = gate.gripper_bounds(pre + name)
            if bounds is not None:
                sub.gripper[name] = np.clip(np.asarray(cmd, dtype=float), bounds[0], bounds[1])
    return SafetyVerdict(True, action=action)


def parts_position_targets(action: "Action", embodiment: Embodiment):
    """Position targets of an ``Action.parts`` tree for the gate's step and error checks.

    Returns ``(idx, targets, write)``: the robot joint indices, the targets for
    them, and a function that writes (clamped) targets back into the leaves.
    ``(None, None, None)`` when no leaf has position targets.
    """
    index = {n: i for i, n in enumerate(embodiment.joint_names)}
    pieces = []
    for path, sub in action_leaves(action):
        if sub.joint_target is None:
            continue
        if sub.joint_names is None and sub.mode not in (ControlMode.POSITION, ControlMode.PD):
            continue  # velocities, torques: not positions
        names = _leaf_joints(embodiment, path, sub)
        pieces.append((sub, [index[n] for n in names]))
    if not pieces:
        return None, None, None
    idx = np.array([i for _, ix in pieces for i in ix], dtype=int)
    targets = np.concatenate([np.asarray(s.joint_target, dtype=float) for s, _ in pieces])

    def write(values: np.ndarray) -> None:
        start = 0
        for sub, ix in pieces:
            sub.joint_target = np.asarray(values[start:start + len(ix)], dtype=float)
            start += len(ix)

    return idx, targets, write


def _copy_tree(action: "Action") -> "Action":
    update: Dict[str, Any] = {"gripper": dict(action.gripper)}
    if action.parts:
        update["parts"] = {k: _copy_tree(v) for k, v in action.parts.items()}
    return action.model_copy(update=update)


class PartView:
    """Read-only access to one part of an assembly; it cannot be commanded through this.

    ``robot.part("left")`` returns one. Names are the part's own (no prefix).
    """

    def __init__(self, path: str, robot: "Robot") -> None:
        self.path = path
        self._robot = robot

    @property
    def embodiment(self) -> Embodiment:
        return self._robot.embodiment

    @property
    def simulated(self) -> bool:
        return bool(self._robot.simulated)

    @property
    def fault(self) -> Optional[str]:
        return self._robot.fault

    @property
    def action_modes(self) -> Tuple[ControlMode, ...]:
        return tuple(self._robot.action_modes)

    def get_observation(self) -> Observation:
        return self._robot.get_observation()

    def get_status(self) -> RobotStatus:
        return self._robot.get_status()

    def __repr__(self) -> str:
        return f"PartView({self.path!r}, {self._robot.embodiment.name!r})"


__all__ = ["BASE_COMMANDS", "BASE_FIELDS", "PartView", "SEP", "action_leaves", "action_owners",
           "check_parts", "frame_targets", "owner_part", "part_covers", "parts_overlap",
           "parts_position_targets", "resolve_part", "scope_joints",
           "leaf_prefix", "merge_embodiments", "part_joint_names", "prefixed", "split_name"]
