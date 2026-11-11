"""Build a :class:`~hirobot.core.robot.Robot` from an ``assembly:`` config tree.

:func:`make_robot` calls :func:`build_tree` for a config whose top level is
``assembly:`` or ``device:`` (:mod:`hirobot.loading.tree`). The tree is built bottom
up, one robot per node:

* The devices directly in a node form its **own robot**: the body driver, with
  its end-effector drivers in a :class:`~hirobot.assembly.RobotAssembly` when
  they have real drivers (a simulated body models the fingers itself, as in
  the single-robot path). Its names stay unprefixed.
* A node without child nodes is that own robot, with the node's cameras and
  audio devices attached: the same stack a single-robot config gets.
* A node with child nodes is a ``RobotAssembly(None, parts=children, own=...)``
  holding the node's cameras and audio itself. A child's placement comes from
  the config: ``pose`` (fixed), ``origin`` (moves by itself), or ``on`` a mount
  of the node's own body. A child node mounted on a body with joints is refused
  for now: its pose would follow that body's kinematics.

Cameras and audio devices are built only as stand-ins (``simulated`` drivers)
when every body in the tree is simulated, as for a single simulated robot.

**Safety.** One gate around the root checks every joint in the robot's names.
Each per-joint key (``joint_limit``, ``max_joint_step``, ``max_joint_error``)
becomes a list over the robot's joints, from the ``safety:`` that holds for
each device (:func:`hirobot.loading.tree.joint_safety`); a joint no section limits
gets no limit. ``gripper_range`` becomes ``{end effector: [lo, hi]}``. The
other keys (``clamp``, ``min_battery_soc``, ``max_joint_temperature``) hold for
the whole robot; two parts that set different values are an error.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

from hirobot.core.embodiment import Embodiment, EmbodimentKind
from hirobot.loading.layout import NodeLayout, node_layout
from hirobot.geometry.transforms import IDENTITY_POSE, pose_compose
from hirobot.loading.tree import SEP, DeviceSpec, NodeSpec, RobotSpec, joint_safety

if TYPE_CHECKING:
    from hirobot.core.robot import Robot

logger = logging.getLogger(__name__)

#: ``safety:`` keys with one value per joint.
PER_JOINT = ("joint_limit", "max_joint_step", "max_joint_error")
#: ``safety:`` keys that hold for the whole robot.
WHOLE_ROBOT = ("clamp", "min_battery_soc", "max_joint_temperature")


def build_tree(spec: RobotSpec, sim: bool = False) -> Tuple["Robot", Embodiment]:
    """The robot of ``spec`` without gate, layers or arbiter, and its embodiment.

    With ``sim``, every body is a view of one shared MuJoCo world
    (:mod:`hirobot.sim.sim_world`, set up by ``spec.sim``) and every mounted camera
    is rendered by it; no real driver is built.
    """
    layout = node_layout(spec.root)
    world = None
    if sim:
        from hirobot.sim.sim_world import build_world, sim_options

        world = build_world(spec, layout, sim_options(spec.sim))
    sim_only = world is not None or all(_body_class(b).simulated for _, b in _bodies(spec.root))
    robot = _build_node(spec.root, layout, sim_only, "", world)
    return robot, layout.embodiment


def world_views(spec: RobotSpec, options: Mapping[str, Any]) -> Tuple[Any, List[Any], NodeLayout]:
    """The shared world of ``spec`` with one view per body, and nothing else.

    No cameras, audio or assembly: what the simulation check
    (:mod:`hirobot.sim.sim_check`) predicts in. ``options`` is a checked ``sim:``
    section (:func:`hirobot.sim.sim_world.sim_options`). The world is not connected.
    """
    from hirobot.drivers.sim.mujoco.driver import MujocoView
    from hirobot.sim.sim_world import build_world

    layout = node_layout(spec.root)
    tree = build_world(spec, layout, options)
    tree.world.cameras = []  # nothing renders in the check (set before connect builds it)
    views: List[Any] = []

    def walk(node: NodeSpec, lay: NodeLayout, path: str) -> None:
        children = {p for p, d in node.parts.items() if isinstance(d, NodeSpec)}
        for p, d in node.parts.items():
            if isinstance(d, NodeSpec):
                walk(d, lay.children[p], f"{path}{p}{SEP}")
            elif d.kind == "body":
                emb = _without_devices(_own_embodiment(node, lay.embodiment, children))
                views.append(MujocoView(emb, command_timeout_s=None, **tree.views[path + p]))

    walk(spec.root, layout, "")
    return tree.world, views, layout


def tree_safety(spec: RobotSpec, embodiment: Embodiment) -> Dict[str, Any]:
    """The root gate's keyword arguments (see the module docstring)."""
    index = {n: i for i, n in enumerate(embodiment.joint_names)}
    ee_names = {e.name for e in embodiment.end_effectors}
    n = embodiment.num_dofs
    per_joint: Dict[str, np.ndarray] = {}
    grippers: Dict[str, Any] = {}
    whole: Dict[str, Tuple[Any, str]] = {}
    for path, dev in spec.root.walk():
        if not isinstance(dev, DeviceSpec):
            continue
        cfg = joint_safety(spec, path)
        node_prefix = path.rpartition(SEP)[0]
        node_prefix = node_prefix + SEP if node_prefix else ""
        joints = [node_prefix + j["name"] for j in dev.fields.get("joints") or []]
        for key in PER_JOINT:
            value = cfg.get(key)
            if value is None or not joints:
                continue
            values = np.broadcast_to(np.asarray(value, dtype=float), (len(joints),)) \
                if np.ndim(value) == 0 else np.asarray(value, dtype=float)
            if values.shape != (len(joints),):
                raise ValueError(f"safety.{key} for {path!r} has {values.size} values for its "
                                 f"{len(joints)} joints")
            arr = per_joint.setdefault(key, np.full(n, np.inf))
            for name, v in zip(joints, values):
                arr[index[name]] = v
        rng = cfg.get("gripper_range")
        if rng is not None:
            for ee in _gripper_names(dev, path, node_prefix):
                if ee not in ee_names:
                    continue
                grippers[ee] = rng.get(ee.rpartition(SEP)[2]) if isinstance(rng, Mapping) else rng
        for key in WHOLE_ROBOT:
            if key in cfg:
                if key in whole and whole[key][0] != cfg[key]:
                    raise ValueError(f"safety.{key} is one value for the whole robot; "
                                     f"{whole[key][1]!r} sets {whole[key][0]!r} and {path!r} "
                                     f"sets {cfg[key]!r}")
                whole[key] = (cfg[key], path)
    out: Dict[str, Any] = {k: v[0] for k, v in whole.items()}
    for key in WHOLE_ROBOT:
        if key in (spec.safety or {}):
            out[key] = spec.safety[key]
    for key, arr in per_joint.items():
        if np.all(arr == arr[0]):
            out[key] = float(arr[0]) if np.isfinite(arr[0]) else None
        else:
            out[key] = [float(v) for v in arr]
        if out[key] is None:
            del out[key]
    grippers = {k: v for k, v in grippers.items() if v is not None}
    if grippers:
        out["gripper_range"] = grippers
    return out


# ---------------------------------------------------------------- building


def _bodies(node: NodeSpec) -> List[Tuple[str, DeviceSpec]]:
    return [(p, d) for p, d in node.walk() if isinstance(d, DeviceSpec) and d.kind == "body"]


def _body_class(dev: DeviceSpec) -> Any:
    from hirobot.loading.registry import get_body

    return get_body((dev.driver or {"type": "toy"})["type"])


def _gripper_names(dev: DeviceSpec, path: str, node_prefix: str) -> List[str]:
    if dev.kind == "body":
        return [node_prefix + g for g in dev.fields.get("grippers") or []]
    if dev.kind == "end_effector" and dev.fields.get("interface") == "opening":
        return [node_prefix + path.rpartition(SEP)[2]]
    return []


def _build_node(node: NodeSpec, layout: NodeLayout, sim_only: bool, path: str,
                world: Any = None) -> "Robot":
    from hirobot.assembly import RobotAssembly
    from hirobot.devices.audio import build_audio
    from hirobot.devices.camera import build_cameras

    where = f"assembly {path or node.name or 'robot'!r}"
    emb = layout.embodiment
    bodies = [p for p, d in node.parts.items() if isinstance(d, DeviceSpec) and d.kind == "body"]
    children = {p: d for p, d in node.parts.items() if isinstance(d, NodeSpec)}
    if len(bodies) > 1:
        raise ValueError(f"{where}: more than one body directly in one node ({bodies}); put "
                         "each body in a node of its own")
    hands = [p for p, d in node.parts.items()
             if isinstance(d, DeviceSpec) and d.kind == "end_effector"]
    if hands and not bodies:
        raise ValueError(f"{where}: end effector(s) {hands} need a body in the same node")
    own_names = {p for p, d in node.parts.items() if isinstance(d, DeviceSpec)}
    own_emb = _own_embodiment(node, emb, set(children))
    body, hands, nic = (_build_body(node.parts[bodies[0]], own_emb, world, path + bodies[0])
                        if bodies else (None, {}, None))
    own_cameras = [c for c in own_emb.cameras if c.name in own_names]
    if world is not None:
        # The body's view renders its node's mounted cameras; a node without a body
        # gets a camera rendered by the world; any other camera only as a stand-in.
        drawn = {c.name for c in own_cameras if path + c.name in world.rendered}
        cameras = build_cameras([c for c in own_cameras if c.name not in drawn], nic,
                                sim_only=True)
        cameras.update({c.name: world.cameras[path + c.name] for c in own_cameras
                        if path + c.name in world.cameras})
    else:
        cameras = build_cameras(own_cameras, nic, sim_only=sim_only)
    mics, spks = build_audio(own_emb.microphones, own_emb.speakers, nic, sim_only=sim_only)
    if not children:
        assert body is not None  # a node has parts; without children one of them is a body
        if cameras or hands or mics or spks:
            return RobotAssembly(body, cameras=cameras, end_effectors=hands, embodiment=emb,
                                 microphones=mics, speakers=spks)
        return body
    own = body
    if hands:
        own = RobotAssembly(body, end_effectors=hands, embodiment=_without_devices(own_emb))
    parts = {p: _build_node(child, layout.children[p], sim_only, f"{path}{p}{SEP}", world)
             for p, child in children.items()}
    placement = {p: _child_placement(node, layout, p, where) for p in children}
    placement = {p: v for p, v in placement.items() if v is not None}
    collision = _collision(node.collision, node, parts, where)
    return RobotAssembly(None, cameras=cameras, embodiment=emb, microphones=mics, speakers=spks,
                         parts=parts, placement=placement, collision=collision, own=own)


def _own_embodiment(node: NodeSpec, emb: Embodiment, children: set) -> Embodiment:
    """``emb`` without the child nodes' joints and devices: what the node's own robot drives."""
    def own(name: str) -> bool:
        return name.partition(SEP)[0] not in children or SEP not in name

    body = next((d for d in node.parts.values()
                 if isinstance(d, DeviceSpec) and d.kind == "body"), None)
    floating = bool(body.fields.get("floating_base")) if body is not None else False
    return Embodiment(
        name=node.name or "robot",
        kind=EmbodimentKind.HUMANOID if floating else EmbodimentKind.ARM,
        control_hz=body.fields["control_hz"] if body is not None else emb.control_hz,
        joints=[j for j in emb.joints if own(j.name)], floating_base=floating,
        end_effectors=[e for e in emb.end_effectors if own(e.name)],
        cameras=[c for c in emb.cameras if own(c.name)],
        microphones=[m for m in emb.microphones if own(m.name)],
        speakers=[s for s in emb.speakers if own(s.name)])


def _without_devices(emb: Embodiment) -> Embodiment:
    """``emb`` without cameras and audio (the node holds those, not its own body)."""
    return emb.model_copy(update={"cameras": [], "microphones": [], "speakers": []})


def only_body(spec: RobotSpec, what: str) -> Tuple[str, DeviceSpec]:
    """The part path and device of a robot's one body (``what`` names the caller in the error)."""
    bodies = _bodies(spec.root)
    if len(bodies) != 1:
        raise ValueError(f"{what} takes a robot with one body; this one has "
                         f"{[p for p, _ in bodies] or 'none'}")
    return bodies[0]


def body_driver_kwargs(dev: DeviceSpec) -> Dict[str, Any]:
    """The body driver's keyword arguments (its ``driver:`` without ``type``).

    Until the shared simulation (``sim:``) exists, a body set to the ``mujoco``
    driver takes its MJCF, gains, armature and home pose from the device file's
    ``model:`` (what ``driver:`` sets itself wins).
    """
    kwargs = dict(dev.driver or {"type": "toy"})
    if kwargs.pop("type") == "mujoco" and dev.fields.get("model"):
        model = dev.fields["model"]
        kwargs.setdefault("model_path", model["mjcf"])
        for key, arg in (("kp", "default_kp"), ("kd", "default_kd"), ("armature", "armature"),
                         ("home", "initial_joint_pos")):
            if model.get(key) is not None:
                kwargs.setdefault(arg, model[key])
    return kwargs


def _build_body(dev: DeviceSpec, emb: Embodiment, world: Any = None,
                path: str = "") -> Tuple["Robot", Dict[str, Any], Any]:
    """The body driver, its end-effector drivers (when it is real) and its NIC."""
    from hirobot.devices.end_effector import (
        body_embodiment,
        build_end_effectors,
        has_end_effector_drivers,
    )

    if world is not None:
        # The shared simulation: a view of the world drives the body and its fingers.
        from hirobot.drivers.sim.mujoco.driver import MujocoView

        node = path.rpartition(SEP)[0]
        node = node + SEP if node else ""
        view_emb = emb.model_copy(update={
            "cameras": [c for c in emb.cameras if node + c.name in world.rendered],
            "microphones": [], "speakers": []})
        return MujocoView(view_emb, **world.views[path]), {}, None
    kwargs = body_driver_kwargs(dev)
    cls = _body_class(dev)
    nic = kwargs.get("network_interface")
    # A simulator renders the cameras itself (from their mounts), as in the
    # single-robot path, so it gets the whole embodiment.
    body_emb = emb if cls.simulated else _without_devices(emb)
    if not cls.simulated and has_end_effector_drivers(emb):
        # A real body drives only its own joints; the hand drivers take the fingers.
        hands = build_end_effectors(emb, nic, read_only=bool(kwargs.get("read_only")))
        return cls(body_embodiment(body_emb), **kwargs), hands, nic
    return cls(body_emb, **kwargs), {}, nic


def _child_placement(node: NodeSpec, layout: NodeLayout, p: str,
                     where: str) -> Optional[Dict[str, Any]]:
    lay = layout.parts[p]
    pl = node.placement.get(p)
    if lay.kind == "moving":
        return {"origin": lay.origin}
    if pl is None:
        return None
    if pl.on is None:
        return None if pl.pose is None else {"pose": pl.pose}
    host_name, mount = pl.on
    host = node.parts[host_name]
    if host.kind != "body" or host.fields.get("joints"):
        raise ValueError(f"{where}: placement.{p}: a robot mounted on {host_name}.{mount} would "
                         "move with that part's joints; only a mount on a body without joints "
                         "(a base, a pedestal) is supported for a child robot")
    m = host.fields["mounts"][mount]
    offset = m["pose"] if m["pose"] is not None else IDENTITY_POSE
    if pl.pose is not None:
        offset = pose_compose(offset, pl.pose)
    return {"pose": np.asarray(offset, dtype=float)}


def _collision(cfg: Optional[Mapping[str, Any]], node: NodeSpec, parts: Mapping[str, "Robot"],
               where: str) -> Any:
    """``collision:`` as a check between child robots; a model key is ``<child>/<body>``."""
    if not cfg:
        return None
    from hirobot.geometry.collision import CollisionCheck

    cfg = dict(cfg)
    models: Dict[str, Any] = {}
    for key, spec in dict(cfg.get("models") or {}).items():
        child, _, rest = str(key).partition(SEP)
        if child not in parts:
            raise ValueError(f"{where}: collision.models.{key}: no child robot {child!r}; "
                             f"children: {list(parts)}")
        sub = node.parts[child]
        dev = sub.parts.get(rest) if isinstance(sub, NodeSpec) and rest else None
        if rest and not (isinstance(dev, DeviceSpec) and dev.kind == "body"):
            raise ValueError(f"{where}: collision.models.{key}: {rest!r} is not a body directly "
                             f"in {child!r}")
        if child in models:
            raise ValueError(f"{where}: collision.models: two models for {child!r}")
        models[child] = spec
    cfg["models"] = models
    return CollisionCheck.build({m: r.embodiment.joint_names for m, r in parts.items()}, cfg)


__all__ = ["build_tree", "tree_safety"]
