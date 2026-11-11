"""Where each part of a robot config sits, and the :class:`Embodiment` that follows.

:func:`node_layout` takes a loaded :class:`~hirobot.loading.tree.NodeSpec` and returns
a :class:`NodeLayout`: every part's resolved placement and the node's
embodiment.

**Names.** Joints, end effectors, cameras, microphones and speakers of a child
node get the prefix ``<part>/`` (``left/shoulder_pan``, ``cell/left/tcp``).
The devices directly in a node keep their names: joints and body ``frames`` as
in the device file, cameras, hands and audio devices by their ``parts:`` key.
Joints are concatenated in ``parts:`` order.

**Placement.** A part is one of three kinds:

* *mounted*: ``{on: <part>.<mount>, pose}``; fixed to the link that mount is
  on, at ``mount.pose ∘ pose``. It moves with that link.
* *fixed*: ``{pose}`` or no entry; at ``pose`` in the node's frame.
* *moving*: a body with ``floating_base: true``, or a floating child node;
  ``{origin}`` or no entry. ``origin`` is where its odometry origin sits in the
  node's frame.

A node is **floating** when it has exactly one moving part, that part has no
placement entry, and every other part is mounted on it (directly or through
other mounted parts). Its frame is that part's base frame and moves with it: a
single humanoid is a floating node, and so is a mobile base with an arm on it.
Any other node has a fixed frame.

**Frames in the embodiment.** A camera's :class:`~hirobot.core.embodiment.CameraMount`
and an end effector's :class:`~hirobot.core.embodiment.EndEffectorFrame` name a
link and an offset from it. A mounted part uses the mount's link; a fixed part
in a fixed root node uses ``world``. A fixed part inside a child node placed
with ``pose`` or ``on`` is carried through that placement. A hand's frame is
its placement composed with its ``tcp``. A link of a child node's device gets
the child's prefix (``left/wrist_3_link``), like its joints.

**A hand where the body has a frame of the same name.** The Unitree G1 body
reports ``left_hand`` (at ``left_wrist_yaw_link``); the G1 with Dex3 hands
mounts a part ``left_hand`` on the wrist. When an end-effector part has the
name of a body frame in the same node and is mounted on that body at the
frame's link, the hand replaces the frame. Any other repeated name is an error.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union

import numpy as np

from hirobot.core.embodiment import (
    AudioDevice,
    Camera,
    CameraMount,
    Embodiment,
    EmbodimentKind,
    EmbodimentPart,
    EndEffector,
    EndEffectorFrame,
    Joint,
)
from hirobot.geometry.transforms import IDENTITY_POSE, pose_compose
from hirobot.loading.tree import SEP, DeviceSpec, NodeSpec

#: Link name of the world frame (MuJoCo's world body).
WORLD = "world"


@dataclass
class PartLayout:
    """Where one part of a node sits.

    Attributes:
        kind: ``mounted``, ``fixed`` or ``moving``.
        anchor: The link it is fixed to (mounted, or fixed in a node that has
            one), already prefixed; ``world`` for a fixed root node; ``None``
            when the node's frame moves (a floating node) or for a moving part.
        pose: Offset from ``anchor`` (mounted / fixed); identity for moving.
        origin: Odometry origin in the node's frame (moving parts only).
        on: ``(part, mount)`` for a mounted part.
    """

    kind: str
    anchor: Optional[str] = None
    pose: np.ndarray = field(default_factory=lambda: IDENTITY_POSE.copy())
    origin: Optional[np.ndarray] = None
    on: Optional[Tuple[str, str]] = None


@dataclass
class NodeLayout:
    """A node with every placement resolved.

    Attributes:
        embodiment: The node's embodiment (children's names prefixed).
        parts: Part name -> its :class:`PartLayout`.
        children: Child node name -> its own layout.
        floating: Whether the node's frame is its one moving part's base frame.
        moving: The name of that part in a floating node, else ``None``.
    """

    embodiment: Embodiment
    parts: Dict[str, PartLayout]
    children: Dict[str, "NodeLayout"]
    floating: bool
    moving: Optional[str] = None


def node_layout(node: NodeSpec, name: Optional[str] = None,
                anchor: Optional[Tuple[str, np.ndarray]] = (WORLD, IDENTITY_POSE),
                link_prefix: str = "") -> NodeLayout:
    """Resolve ``node``'s placements and compute its embodiment.

    Args:
        node: The loaded node.
        name: The embodiment's name; default ``node.name`` or ``"robot"``.
        anchor: Link and offset of this node's frame, when it is fixed to one
            (``world`` for the root); ``None`` when it is not. The link is
            already in its final (prefixed) name.
        link_prefix: Prefix of this node's own links (``left/`` for a child
            ``left`` of the root).

    Raises:
        ValueError: A placement on a part or mount that does not exist, a
            cycle, ``pose`` on a moving part or ``origin`` on a fixed one, a
            moving part mounted on something, a name used twice.
    """
    where = f"assembly {node.name or name or 'robot'!r}"
    moving = {p: _is_moving(part) for p, part in node.parts.items()}
    _check_placements(node, moving, where)
    floating_part = _floating_part(node, moving)
    if floating_part is not None:
        anchor = None  # the node's frame moves with its moving part
    layouts: Dict[str, PartLayout] = {}
    for p in node.parts:
        _resolve(p, node, moving, anchor, layouts, [], where, link_prefix)
    children: Dict[str, NodeLayout] = {}
    for p, part in node.parts.items():
        if isinstance(part, NodeSpec):
            lay = layouts[p]
            child_anchor = None if lay.anchor is None else (lay.anchor, lay.pose)
            children[p] = node_layout(part, p, child_anchor, f"{link_prefix}{p}{SEP}")
    emb = _embodiment(node, name, layouts, children, floating_part, where, link_prefix)
    return NodeLayout(embodiment=emb, parts=layouts, children=children,
                      floating=floating_part is not None, moving=floating_part)


def node_embodiment(node: NodeSpec, name: Optional[str] = None) -> Embodiment:
    """The embodiment of ``node`` (:func:`node_layout` without the placements)."""
    return node_layout(node, name).embodiment


# ----------------------------------------------------------------- placement


def _is_moving(part: Union[DeviceSpec, NodeSpec]) -> bool:
    if isinstance(part, DeviceSpec):
        return part.kind == "body" and bool(part.fields.get("floating_base"))
    moving = {p: _is_moving(c) for p, c in part.parts.items()}
    return _floating_part(part, moving) is not None


def _floating_part(node: NodeSpec, moving: Dict[str, bool]) -> Optional[str]:
    """The one moving part of a floating node, or ``None`` (the node's frame is fixed)."""
    movers = [p for p, m in moving.items() if m]
    if len(movers) != 1 or movers[0] in node.placement:
        return None
    root = movers[0]
    for p in node.parts:
        if p == root:
            continue
        seen = set()
        while True:
            pl = node.placement.get(p)
            if pl is None or pl.on is None or p in seen:
                return None
            seen.add(p)
            p = pl.on[0]
            if p == root:
                break
    return root


def _check_placements(node: NodeSpec, moving: Dict[str, bool], where: str) -> None:
    for p, pl in node.placement.items():
        part = node.parts[p]
        at = f"{where}: placement.{p}"
        if moving[p]:
            if pl.on is not None:
                raise ValueError(f"{at}: {p!r} moves by itself (floating base); it cannot be "
                                 "mounted on another part. Mount the other parts on it instead")
            if pl.pose is not None:
                raise ValueError(f"{at}: {p!r} moves by itself; give the origin of its odometry "
                                 "(origin:), not pose:")
        elif pl.origin is not None:
            raise ValueError(f"{at}: origin: is for a part that moves by itself; {p!r} does not "
                             "(use pose:)")
        if pl.on is not None:
            target, mount = pl.on
            if target == p:
                raise ValueError(f"{at}: a part cannot be mounted on itself")
            if target not in node.parts:
                raise ValueError(f"{at}: on: {target}.{mount}: no part {target!r}; parts: "
                                 f"{list(node.parts)}")
            host = node.parts[target]
            if not isinstance(host, DeviceSpec):
                raise ValueError(f"{at}: on: {target}.{mount}: {target!r} is an assembly; mount "
                                 "on one of its devices from inside it, or place it with pose:")
            mounts = host.fields.get("mounts") or {}
            if mount not in mounts:
                raise ValueError(f"{at}: on: {target}.{mount}: {target!r} declares no mount "
                                 f"{mount!r}; its mounts: {sorted(mounts)}")
        del part


def _resolve(p: str, node: NodeSpec, moving: Dict[str, bool],
             anchor: Optional[Tuple[str, np.ndarray]], out: Dict[str, PartLayout],
             chain: List[str], where: str, link_prefix: str) -> PartLayout:
    if p in out:
        return out[p]
    if p in chain:
        cycle = " -> ".join([*chain[chain.index(p):], p])
        raise ValueError(f"{where}: placement forms a cycle: {cycle}")
    pl = node.placement.get(p)
    if moving[p]:
        origin = IDENTITY_POSE.copy() if pl is None or pl.origin is None else pl.origin
        out[p] = PartLayout(kind="moving", origin=origin)
        return out[p]
    if pl is not None and pl.on is not None:
        target, mount = pl.on
        _resolve(target, node, moving, anchor, out, [*chain, p], where, link_prefix)
        m = node.parts[target].fields["mounts"][mount]
        offset = m["pose"] if m["pose"] is not None else IDENTITY_POSE
        if pl.pose is not None:
            offset = pose_compose(offset, pl.pose)
        out[p] = PartLayout(kind="mounted", anchor=link_prefix + m["link"],
                            pose=np.asarray(offset, dtype=float), on=pl.on)
        return out[p]
    pose = IDENTITY_POSE if pl is None or pl.pose is None else pl.pose
    if anchor is None:
        out[p] = PartLayout(kind="fixed", anchor=None, pose=np.asarray(pose, dtype=float))
    else:
        out[p] = PartLayout(kind="fixed", anchor=anchor[0],
                            pose=pose_compose(anchor[1], pose))
    return out[p]


# ---------------------------------------------------------------- embodiment


def _frame(anchor: Optional[str], pose: np.ndarray) -> Optional[Tuple[str, tuple, tuple]]:
    if anchor is None:
        return None
    pose = np.asarray(pose, dtype=float)
    return anchor, tuple(float(v) for v in pose[:3]), tuple(float(v) for v in pose[3:])


def _embodiment(node: NodeSpec, name: Optional[str], layouts: Dict[str, PartLayout],
                children: Dict[str, NodeLayout], floating_part: Optional[str],
                where: str, link_prefix: str) -> Embodiment:
    joints: List[Joint] = []
    ees: List[EndEffector] = []
    cams: List[Camera] = []
    mics: List[AudioDevice] = []
    spks: List[AudioDevice] = []
    parts: Dict[str, EmbodimentPart] = {}
    hz: List[int] = []
    floating = False
    # Body frames a hand of the same name, mounted at the frame's link, replaces.
    replaced = _replaced_frames(node, layouts)
    for p, part in node.parts.items():
        lay = layouts[p]
        if isinstance(part, NodeSpec):
            child = children[p].embodiment
            pre = f"{p}{SEP}"
            start = len(joints)
            joints += [j.model_copy(update={"name": pre + j.name}) for j in child.joints]
            # Links are named in full by the child (link_prefix): not renamed here.
            ees += [e.model_copy(update={"name": pre + e.name, "driver": None,
                                         "joints": [pre + j for j in e.joints]})
                    for e in child.end_effectors]
            cams += [c.model_copy(update={"name": pre + c.name, "driver": None})
                     for c in child.cameras]
            mics += [d.model_copy(update={"name": pre + d.name, "driver": None})
                     for d in child.microphones]
            spks += [d.model_copy(update={"name": pre + d.name, "driver": None})
                     for d in child.speakers]
            for sub, info in child.parts.items():
                path = pre + sub if sub else p  # the child's "" is this part
                parts[path] = info.model_copy(update={
                    "joints": (info.joints[0] + start, info.joints[1] + start),
                    "end_effectors": [pre + n for n in info.end_effectors],
                    "cameras": [pre + n for n in info.cameras],
                    "microphones": [pre + n for n in info.microphones],
                    "speakers": [pre + n for n in info.speakers],
                    "children": [pre + c for c in info.children],
                    "moving": info.moving or (not sub and lay.kind == "moving")})
            hz.append(child.control_hz)
            floating = floating or child.floating_base
            continue
        f = part.fields
        if part.kind == "body":
            joints += [Joint(**j) for j in f["joints"]]
            hz.append(f["control_hz"])
            floating = floating or bool(f["floating_base"])
            for fr in f["frames"]:
                if fr["name"] in replaced:
                    continue
                frame = None
                if fr.get("link") is not None:
                    pose = IDENTITY_POSE if fr["pose"] is None else fr["pose"]
                    frame = EndEffectorFrame(body=link_prefix + fr["link"],
                                             pos=tuple(float(v) for v in pose[:3]),
                                             quat=tuple(float(v) for v in pose[3:]))
                ees.append(EndEffector(name=fr["name"], has_gripper=fr["name"] in f["grippers"],
                                       frame=frame))
        elif part.kind == "end_effector":
            joints += [Joint(**j) for j in f["joints"]]
            pose = lay.pose
            if f.get("tcp") is not None:
                pose = pose_compose(pose, f["tcp"])
            fr = _frame(lay.anchor, pose)
            ees.append(EndEffector(
                name=p, has_gripper=f["interface"] == "opening",
                joints=[j["name"] for j in f["joints"]], driver=part.driver,
                frame=None if fr is None else EndEffectorFrame(body=fr[0], pos=fr[1],
                                                               quat=fr[2])))
        elif part.kind == "camera":
            fr = _frame(lay.anchor, lay.pose)
            mount = None if fr is None else CameraMount(body=fr[0], pos=fr[1], quat=fr[2],
                                                        fovy=f["fovy"])
            cams.append(Camera(name=p, width=f["width"], height=f["height"], driver=part.driver,
                               mount=mount, depth=f["depth"], fps=f.get("fps")))
        elif part.kind == "microphone":
            mics.append(AudioDevice(name=p, driver=part.driver))
        else:
            spks.append(AudioDevice(name=p, driver=part.driver))
    parts[""] = EmbodimentPart(
        joints=(0, len(joints)), end_effectors=[e.name for e in ees],
        cameras=[c.name for c in cams], microphones=[m.name for m in mics],
        speakers=[s.name for s in spks],
        children=[p for p in node.parts if isinstance(node.parts[p], NodeSpec)],
        moving=floating_part is not None,
        devices=[p for p in node.parts if not isinstance(node.parts[p], NodeSpec)])
    try:
        return Embodiment(
            name=name if name is not None else (node.name or "robot"),
            kind=EmbodimentKind.HUMANOID if floating else EmbodimentKind.ARM,
            control_hz=node.control_hz or (min(hz) if hz else 30),
            joints=joints, floating_base=floating, end_effectors=ees, cameras=cams,
            microphones=mics, speakers=spks, parts=dict(sorted(parts.items())))
    except ValueError as exc:
        raise ValueError(f"{where}: {exc}") from None


def _replaced_frames(node: NodeSpec, layouts: Dict[str, PartLayout]) -> set:
    frames: Dict[str, Tuple[str, Optional[str]]] = {}
    for p, part in node.parts.items():
        if isinstance(part, DeviceSpec) and part.kind == "body":
            for fr in part.fields["frames"]:
                frames[fr["name"]] = (p, fr.get("link"))
    out = set()
    for p, part in node.parts.items():
        if not (isinstance(part, DeviceSpec) and part.kind == "end_effector") or p not in frames:
            continue
        body, link = frames[p]
        lay = layouts[p]
        if lay.on is not None and lay.on[0] == body and link is not None \
                and node.parts[body].fields["mounts"][lay.on[1]]["link"] == link:
            out.add(p)
    return out


__all__ = ["NodeLayout", "PartLayout", "WORLD", "node_embodiment", "node_layout"]
