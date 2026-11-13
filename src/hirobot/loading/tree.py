"""Robot configs as a tree: device files and ``assembly:`` nodes.

A **device file** describes one device model; its top level is the device::

    device: body              # body, end_effector, camera, microphone or speaker
    control_hz: 125
    joints: [{name: shoulder_pan}, ...]
    driver: {type: ur_rtde}

A **robot config** has an ``assembly:`` node listing its parts. A part is a
device or another node, referenced with ``from:`` (a built-in name or a path;
a relative path is taken from the file that names it) or written inline::

    assembly:
      name: ur5e_dual
      parts:
        left:  {from: robots/ur_ur5e, overrides: {body: {driver: {ip: 192.168.1.101}}}}
        right: {from: robots/ur_ur5e, safety: {joint_limit: 2.5}}
        scene: {device: camera, width: 640, height: 480, driver: {type: array}}
      placement:
        left:  {pose: [0, 0.4, 0, 0, 0, 0, 1]}
        right: {pose: [0, -0.4, 0, 0, 0, 0, 1]}
    safety: {max_joint_step: 0.2}

``overrides`` on a device reference is merged into the device file
(:func:`hirobot.loading.config.merge`); on a node reference it maps a child part's name
to that part's overrides, level by level. Overrides from further up are merged
over the ones written further down first, and the result is merged into the
device file once, so a driver ``type`` set back to the file's own keeps the
file's other driver options.

:func:`load_robot` returns a :class:`RobotSpec`: the root :class:`NodeSpec` and
the sections only the root may have (``safety``, ``layers``, ``arbiter``,
``sim``). Where each part sits, and the :class:`~hirobot.core.embodiment.Embodiment`
that follows, is worked out by :mod:`hirobot.assembly.parts`.
"""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple, Union

import numpy as np

from hirobot.loading.config import ConfigRef, _load, merge, resolve

SEP = "/"

DEVICE_KINDS = ("body", "end_effector", "camera", "microphone", "speaker")

#: Keys each kind of device file takes (besides ``device``).
DEVICE_FIELDS: Dict[str, Tuple[str, ...]] = {
    "body": ("control_hz", "floating_base", "joints", "frames", "mounts", "grippers", "model",
             "driver"),
    "end_effector": ("interface", "joints", "tcp", "mounts", "model", "driver"),
    "camera": ("width", "height", "fps", "depth", "fovy", "driver"),
    "microphone": ("driver",),
    "speaker": ("driver",),
}
NODE_KEYS = ("name", "control_hz", "parts", "placement", "collision")
#: Keys of a part that references a file.
REF_KEYS = ("from", "overrides", "safety")
#: Top-level keys of a robot config used as the root (``defaults:`` is expanded first).
ROOT_KEYS = ("assembly", "safety", "layers", "arbiter", "sim")
MODEL_KEYS = ("mjcf", "kp", "kd", "armature", "home")
#: ``safety:`` keys that only the root config may set.
ROOT_ONLY_SAFETY = ("sim_check",)
#: Every ``safety:`` key (the gate's arguments, ``sim_check``).
SAFETY_KEYS = ("joint_limit", "max_joint_step", "max_joint_error", "gripper_range", "clamp",
               "min_battery_soc", "max_joint_temperature", "sim_check")


@dataclass
class Placement:
    """Where a part sits in its node (``placement:`` entry).

    Attributes:
        on: ``(part, mount)`` it is mounted on, or ``None``.
        pose: ``[x, y, z, qx, qy, qz, qw]`` in the mount's frame (with ``on``)
            or the node's frame, or ``None`` (identity).
        origin: For a part that moves by itself: where its odometry origin
            sits in the node's frame. ``None`` for the other two kinds.
    """

    on: Optional[Tuple[str, str]] = None
    pose: Optional[np.ndarray] = None
    origin: Optional[np.ndarray] = None


@dataclass
class DeviceSpec:
    """One device, its file merged with every override that reaches it.

    Attributes:
        kind: ``body``, ``end_effector``, ``camera``, ``microphone`` or ``speaker``.
        fields: The validated device fields (no ``device`` key).
        safety: The ``safety:`` of the part entry that names it, or ``{}``
            (see :func:`joint_safety`).
        source: The file it came from, or ``None`` (inline in a dict).
    """

    kind: str
    fields: Dict[str, Any]
    safety: Dict[str, Any] = field(default_factory=dict)
    source: Optional[Path] = None

    @property
    def driver(self) -> Optional[Dict[str, Any]]:
        drv = self.fields.get("driver")
        return None if drv is None else dict(drv)


@dataclass
class NodeSpec:
    """An ``assembly:`` node: its parts in order and where they sit.

    Attributes:
        name: ``assembly.name``, or ``None``.
        control_hz: ``assembly.control_hz``, or ``None`` (the slowest body's).
        parts: Part name -> device or child node, in joint order.
        placement: Part name -> where it sits; a part not listed is at the origin.
        collision: The ``collision:`` section (root only), or ``None``.
        safety: The ``safety:`` of the part entry that names this node, or ``{}``.
        defaults: The top-level ``safety:`` of the robot config this node
            comes from, or ``{}`` (see :func:`joint_safety`).
        source: The file it came from, or ``None``.
    """

    name: Optional[str] = None
    control_hz: Optional[int] = None
    parts: Dict[str, Union[DeviceSpec, "NodeSpec"]] = field(default_factory=dict)
    placement: Dict[str, Placement] = field(default_factory=dict)
    collision: Optional[Dict[str, Any]] = None
    safety: Dict[str, Any] = field(default_factory=dict)
    defaults: Dict[str, Any] = field(default_factory=dict)
    source: Optional[Path] = None

    def walk(self, prefix: str = "") -> List[Tuple[str, Union[DeviceSpec, "NodeSpec"]]]:
        """Every part below this node as ``(path, spec)``, parents before children."""
        out: List[Tuple[str, Union[DeviceSpec, NodeSpec]]] = []
        for name, part in self.parts.items():
            path = f"{prefix}{name}"
            out.append((path, part))
            if isinstance(part, NodeSpec):
                out += part.walk(path + SEP)
        return out

    def find(self, path: str) -> Union[DeviceSpec, "NodeSpec"]:
        """The part at ``path`` (``left/body``); ``""`` is this node."""
        node: Union[DeviceSpec, NodeSpec] = self
        for name in [p for p in path.split(SEP) if p]:
            if not isinstance(node, NodeSpec) or name not in node.parts:
                raise KeyError(f"no part {path!r}")
            node = node.parts[name]
        return node


@dataclass
class RobotSpec:
    """A whole robot config: the root node and the root-only sections."""

    root: NodeSpec
    safety: Any = None  # dict, or False for no gate
    layers: List[Dict[str, Any]] = field(default_factory=list)
    arbiter: Dict[str, Any] = field(default_factory=dict)
    sim: Optional[Dict[str, Any]] = None
    source: Optional[Path] = None


def joint_safety(spec: "RobotSpec", path: str) -> Dict[str, Any]:
    """The ``safety:`` values that hold for the joints of the device at ``path``.

    Two kinds of section, merged key by key:

    * A robot config's own top-level ``safety:`` is a default for its tree.
      The innermost default that sets a key wins (the root config's is the
      outermost).
    * A part entry's ``safety:`` (``left: {from: ..., safety: {...}}``) holds
      for every joint below that part, over any default. The outermost entry
      that sets a key wins: whoever assembles the tree has the last word.
    """
    defaults: List[Dict[str, Any]] = [dict(spec.safety or {})] if spec.safety else []
    entries: List[Dict[str, Any]] = []
    node: Union[DeviceSpec, NodeSpec] = spec.root
    for name in [p for p in path.split(SEP) if p]:
        if not isinstance(node, NodeSpec) or name not in node.parts:
            raise KeyError(f"no part {path!r}")
        node = node.parts[name]
        if isinstance(node, NodeSpec) and node.defaults:
            defaults.append(node.defaults)
        if node.safety:
            entries.append(node.safety)
    out: Dict[str, Any] = {}
    for section in defaults:          # outer -> inner: the inner default wins
        out.update(section)
    for section in reversed(entries):  # inner -> outer: the outer entry wins
        out.update(section)
    out.pop("sim_check", None)
    return out


# ----------------------------------------------------------------- loading


def is_tree_config(config: Mapping[str, Any]) -> bool:
    """Whether a loaded config is a robot config (``assembly:``) or a device file (``device:``)."""
    return "assembly" in config or "device" in config


def load_robot(config: Union[ConfigRef, Mapping[str, Any]],
               overrides: Optional[Mapping[str, Any]] = None) -> RobotSpec:
    """Load a robot config into a :class:`RobotSpec`.

    Args:
        config: A built-in name, a path, or a loaded dict. A device file
            alone (top level ``device:``) is a robot of that one device, named
            ``body`` (or after the device kind).
        overrides: Part path -> overrides for that part
            (``{"left/body": {"driver": {"ip": ...}}}``), merged over what the
            config files say. ``""`` or a path to a node takes the node's form
            (child name -> overrides).

    Raises:
        ValueError: An unknown key, a wrong value, a section only the root
            may have below the root, a reference that is neither a device nor
            a node.
        FileNotFoundError: A ``from:`` that names nothing.
    """
    if isinstance(config, Mapping):
        data = copy.deepcopy(dict(config))
        source = None
        base = Path.cwd()
        defaults = data.pop("defaults", None)
        if defaults:
            # A dict may compose built-in files too; expand them like a file would.
            data = _expand_dict_defaults(defaults, data, base)
    else:
        source = resolve(config).resolve()
        data = _load(source, [])
        base = source.parent
    unknown = sorted(set(data) - set(ROOT_KEYS)) if "device" not in data else []
    if unknown:
        raise ValueError(f"{_where(source)}: unknown top-level key(s) {unknown}; a robot config "
                         f"takes {list(ROOT_KEYS)} (and defaults:)")
    above = _path_overrides(overrides)
    if "device" in data:
        if "assembly" in data:
            raise ValueError(f"{_where(source)}: a file is either a device (device:) or a robot "
                             "(assembly:), not both")
        device = {k: v for k, v in data.items() if k not in ROOT_KEYS or k == "device"}
        kind = device.get("device")
        name = "body" if kind == "body" else str(kind)
        root_data = {"parts": {name: device}}
        root = _node(root_data, base, source, above, where=_where(source))
        if source is not None:
            root.name = source.stem
    else:
        if "assembly" not in data:
            raise ValueError(f"{_where(source)}: a robot config needs an assembly: section")
        root = _node(data["assembly"], base, source, above, where=_where(source))
        if root.name is None:
            root.name = source.stem if source is not None else "robot"
    safety = data.get("safety", {})
    if safety is not False:
        safety = _safety(safety, f"{_where(source)}: safety", root=True)
    layers = data.get("layers") or []
    if not isinstance(layers, list) or not all(isinstance(s, Mapping) for s in layers):
        raise ValueError("layers: must be a list of mappings, each with a 'type'")
    arbiter = data.get("arbiter") or {}
    if not isinstance(arbiter, Mapping):
        raise ValueError(f"arbiter: must be a mapping, got {arbiter!r}")
    sim = data.get("sim")
    if sim is not None and not isinstance(sim, Mapping):
        raise ValueError(f"sim: must be a mapping, got {sim!r}")
    return RobotSpec(root=root, safety=safety, layers=[dict(s) for s in layers],
                     arbiter=dict(arbiter), sim=None if sim is None else dict(sim), source=source)


def _expand_dict_defaults(defaults: Any, data: Dict[str, Any], base: Path) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for entry in [defaults] if isinstance(defaults, str) else list(defaults):
        out = merge(out, _load(resolve(entry, base=base), []))
    return merge(out, data)


def _where(source: Optional[Path]) -> str:
    return str(source) if source is not None else "config"


def _path_overrides(overrides: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    """``{"left/body": {...}}`` as the nested form ``{"left": {"body": {...}}}``."""
    nested: Dict[str, Any] = {}
    for path, value in dict(overrides or {}).items():
        if not isinstance(value, Mapping):
            raise ValueError(f"overrides[{path!r}] must be a mapping, got {value!r}")
        names = [p for p in str(path).split(SEP) if p]
        if not names:
            nested = merge(nested, dict(value))
            continue
        wrapped: Dict[str, Any] = dict(value)
        for name in reversed(names):
            wrapped = {name: wrapped}
        nested = merge(nested, wrapped)
    return nested


def _node(data: Any, base: Path, source: Optional[Path], above: Mapping[str, Any],
          where: str) -> NodeSpec:
    """An ``assembly:`` mapping as a :class:`NodeSpec`; ``above`` is child name -> overrides."""
    if not isinstance(data, Mapping):
        raise ValueError(f"{where}: assembly: must be a mapping, got {data!r}")
    unknown = sorted(set(data) - set(NODE_KEYS))
    if unknown:
        raise ValueError(f"{where}: assembly: unknown key(s) {unknown}; it takes {list(NODE_KEYS)}")
    parts_cfg = data.get("parts")
    if not isinstance(parts_cfg, Mapping) or not parts_cfg:
        raise ValueError(f"{where}: assembly.parts must be a non-empty mapping of name -> part")
    extra = sorted(set(above) - set(parts_cfg))
    if extra:
        raise ValueError(f"{where}: overrides for part(s) {extra}, which this node does not have; "
                         f"its parts: {list(parts_cfg)}")
    node = NodeSpec(name=data.get("name"), source=source)
    hz = data.get("control_hz")
    if hz is not None:
        node.control_hz = _positive_int(hz, f"{where}: assembly.control_hz")
    for name, part in parts_cfg.items():
        name = str(name)
        if not name or SEP in name or "." in name:
            raise ValueError(f"{where}: part name {name!r} must be non-empty, without '/' or '.'")
        node.parts[name] = _part(part, base, above.get(name) or {}, f"{where}: parts.{name}")
    placement = data.get("placement") or {}
    if not isinstance(placement, Mapping):
        raise ValueError(f"{where}: assembly.placement must be a mapping, got {placement!r}")
    for name, entry in placement.items():
        if name not in node.parts:
            raise ValueError(f"{where}: placement for {name!r}, which is not a part; parts: "
                             f"{list(node.parts)}")
        node.placement[str(name)] = _placement(entry, f"{where}: placement.{name}")
    collision = data.get("collision")
    if collision is not None:
        if not isinstance(collision, Mapping):
            raise ValueError(f"{where}: assembly.collision must be a mapping")
        node.collision = _with_paths(dict(collision), base)
    return node


def _part(part: Any, base: Path, above: Mapping[str, Any],
          where: str) -> Union[DeviceSpec, NodeSpec]:
    if not isinstance(part, Mapping):
        raise ValueError(f"{where}: a part is a mapping ({{from: ...}} or an inline device / "
                         f"assembly), got {part!r}")
    part = dict(part)
    safety = part.pop("safety", None)
    entry = _safety(safety, f"{where}.safety")
    if "from" in part:
        unknown = sorted(set(part) - set(REF_KEYS))
        if unknown:
            raise ValueError(f"{where}: a part with from: takes {list(REF_KEYS)}; unknown "
                             f"key(s) {unknown} (write inline parts without from:)")
        ref = part["from"]
        if not isinstance(ref, (str, os.PathLike)):
            raise ValueError(f"{where}: from: must be a config name or a path, got {ref!r}")
        path = resolve(ref, base=base).resolve()
        data = _load(path, [])
        written = part.get("overrides") or {}
        if not isinstance(written, Mapping):
            raise ValueError(f"{where}: overrides must be a mapping, got {written!r}")
        if "device" in data:
            if "assembly" in data:
                raise ValueError(f"{path}: a file is either a device or a robot, not both")
            spec = _device(merge(data, merge(dict(written), dict(above))), path, path.parent,
                           f"{path}")
            spec.safety = entry
            return spec
        if "assembly" in data:
            _check_referenced(data, path)
            ups = merge(dict(written), dict(above))
            node = _node(data["assembly"], path.parent, path, ups, str(path))
            node.defaults = _safety(data.get("safety"), f"{path}: safety")
            node.safety = entry
            return node
        raise ValueError(f"{where}: from: {ref!r} ({path}) is neither a device file (device:) "
                         "nor a robot config (assembly:)"
                         + ("; it is in the old format (embodiment: / body:)"
                            if "embodiment" in data or "body" in data else ""))
    if "overrides" in part:
        raise ValueError(f"{where}: overrides is for a part with from:; an inline part has "
                         "its content written out")
    if "device" in part:
        if "assembly" in part:
            raise ValueError(f"{where}: a part is either a device or an assembly, not both")
        spec = _device(merge(part, dict(above)), None, base, where)
        spec.safety = entry
        return spec
    if "assembly" in part:
        unknown = sorted(set(part) - {"assembly"})
        if unknown:
            raise ValueError(f"{where}: an inline assembly part takes assembly: (and safety:), "
                             f"got {unknown}")
        node = _node(part["assembly"], base, None, above, where)
        node.safety = entry
        return node
    raise ValueError(f"{where}: a part needs from:, or device: / assembly: for an inline part")


def _check_referenced(data: Mapping[str, Any], path: Path) -> None:
    """A robot config used as a part: only ``assembly`` and ``safety`` (no root-only sections)."""
    for key in ("layers", "arbiter", "sim"):
        if data.get(key):
            raise ValueError(f"{path}: {key}: is only for the root config; this file is used as "
                             "a part (put it in the config that references this one)")
    unknown = sorted(set(data) - {"assembly", "safety"} - {"layers", "arbiter", "sim"})
    if unknown:
        raise ValueError(f"{path}: unknown top-level key(s) {unknown}")


# ----------------------------------------------------------------- devices


def _device(data: Mapping[str, Any], source: Optional[Path], base: Path, where: str) -> DeviceSpec:
    data = dict(data)
    kind = data.pop("device", None)
    if kind not in DEVICE_KINDS:
        raise ValueError(f"{where}: device: must be one of {list(DEVICE_KINDS)}, got {kind!r}")
    allowed = DEVICE_FIELDS[kind]
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {unknown} for a {kind}; it takes "
                         f"{list(allowed)}")
    out: Dict[str, Any] = {}
    if "driver" in data and data["driver"] is not None:
        out["driver"] = _driver(data["driver"], f"{where}: driver")
    if kind == "body":
        out["control_hz"] = _positive_int(data.get("control_hz", 30), f"{where}: control_hz")
        out["floating_base"] = _bool(data.get("floating_base", False), f"{where}: floating_base")
        out["joints"] = _joints(data.get("joints") or [], where)
        out["frames"] = _frames(data.get("frames") or [], where)
        out["mounts"] = _mounts(data.get("mounts") or {}, where)
        grippers = data.get("grippers") or []
        names = [f["name"] for f in out["frames"]]
        if not isinstance(grippers, list) or any(g not in names for g in grippers):
            raise ValueError(f"{where}: grippers lists frame names; frames: {names}, got "
                             f"{grippers!r}")
        out["grippers"] = list(grippers)
        out.setdefault("driver", {"type": "toy"})
    elif kind == "end_effector":
        interface = data.get("interface")
        if interface not in ("joints", "opening"):
            raise ValueError(f"{where}: interface must be 'joints' (a hand) or 'opening' "
                             f"(a gripper), got {interface!r}")
        out["interface"] = interface
        joints = _joints(data.get("joints") or [], where)
        if interface == "joints" and not joints:
            raise ValueError(f"{where}: an end effector on the joints interface lists its joints")
        if interface == "opening" and joints:
            raise ValueError(f"{where}: an end effector on the opening interface has no joints")
        out["joints"] = joints
        out["tcp"] = None if data.get("tcp") is None else _pose(data["tcp"], f"{where}: tcp")
        out["mounts"] = _mounts(data.get("mounts") or {}, where)
    elif kind == "camera":
        for key in ("width", "height"):
            if key not in data:
                raise ValueError(f"{where}: a camera needs {key}")
            out[key] = _positive_int(data[key], f"{where}: {key}")
        out["fps"] = None if data.get("fps") is None else _positive_float(data["fps"],
                                                                           f"{where}: fps")
        out["depth"] = _bool(data.get("depth", False), f"{where}: depth")
        out["fovy"] = _positive_float(data.get("fovy", 45.0), f"{where}: fovy")
    if "model" in data and data["model"] is not None:
        out["model"] = _model(data["model"], base, f"{where}: model")
    return DeviceSpec(kind=kind, fields=out, source=source)


def _driver(value: Any, where: str) -> Dict[str, Any]:
    if isinstance(value, str):  # driver: toy
        return {"type": value}
    if not isinstance(value, Mapping) or not isinstance(value.get("type"), str):
        raise ValueError(f"{where} must be a mapping with a 'type', got {value!r}")
    return dict(value)


def _joints(value: Any, where: str) -> List[Dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError(f"{where}: joints must be a list, got {value!r}")
    out = []
    for j in value:
        if not isinstance(j, Mapping) or "name" not in j:
            raise ValueError(f"{where}: every joint is {{name, position_limit, torque_limit, "
                             f"axis, parent_link}}, got {j!r}")
        unknown = sorted(set(j) - {"name", "position_limit", "torque_limit", "axis", "parent_link"})
        if unknown:
            raise ValueError(f"{where}: joint {j['name']!r}: unknown key(s) {unknown}")
        out.append(dict(j))
    names = [j["name"] for j in out]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise ValueError(f"{where}: duplicate joint name(s) {dup}")
    return out


def _frames(value: Any, where: str) -> List[Dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError(f"{where}: frames must be a list, got {value!r}")
    out = []
    for f in value:
        if not isinstance(f, Mapping) or "name" not in f:
            raise ValueError(f"{where}: every frame is {{name, link, pose}}, got {f!r}")
        unknown = sorted(set(f) - {"name", "link", "pose"})
        if unknown:
            raise ValueError(f"{where}: frame {f['name']!r}: unknown key(s) {unknown}")
        out.append({"name": str(f["name"]), "link": f.get("link"),
                    "pose": None if f.get("pose") is None else _pose(
                        f["pose"], f"{where}: frame {f['name']!r} pose")})
    names = [f["name"] for f in out]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        raise ValueError(f"{where}: duplicate frame name(s) {dup}")
    return out


def _mounts(value: Any, where: str) -> Dict[str, Dict[str, Any]]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{where}: mounts must be a mapping of name -> {{link, pose}}")
    out = {}
    for name, m in value.items():
        if not isinstance(m, Mapping) or "link" not in m:
            raise ValueError(f"{where}: mount {name!r} is {{link, pose}}, got {m!r}")
        unknown = sorted(set(m) - {"link", "pose"})
        if unknown:
            raise ValueError(f"{where}: mount {name!r}: unknown key(s) {unknown}")
        out[str(name)] = {"link": str(m["link"]),
                          "pose": None if m.get("pose") is None else _pose(
                              m["pose"], f"{where}: mount {name!r} pose")}
    return out


def _model(value: Any, base: Path, where: str) -> Dict[str, Any]:
    if not isinstance(value, Mapping) or not value.get("mjcf"):
        raise ValueError(f"{where} must be a mapping with mjcf (and kp, kd, armature, home)")
    unknown = sorted(set(value) - set(MODEL_KEYS))
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {unknown}; it takes {list(MODEL_KEYS)}")
    out = dict(value)
    out["mjcf"] = _path_from(str(out["mjcf"]), base)
    return out


def _path_from(path: str, base: Path) -> str:
    """A relative file path taken from ``base``; ``~`` and absolute paths unchanged."""
    if path.startswith(("/", "~")):
        return path
    return str(base / path)


def _with_paths(collision: Dict[str, Any], base: Path) -> Dict[str, Any]:
    models = collision.get("models")
    if isinstance(models, Mapping):
        collision["models"] = {
            k: (dict(v, mjcf=_path_from(str(v["mjcf"]), base))
                if isinstance(v, Mapping) and isinstance(v.get("mjcf"), str) else v)
            for k, v in models.items()}
    return collision


# ----------------------------------------------------------------- values


def _pose(value: Any, where: str) -> np.ndarray:
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        raise ValueError(f"{where} must be [x, y, z, qx, qy, qz, qw], got {value!r}") from None
    if arr.shape != (7,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{where} must be 7 finite numbers [x, y, z, qx, qy, qz, qw], "
                         f"got {value!r}")
    norm = np.linalg.norm(arr[3:])
    if norm < 1e-9:
        raise ValueError(f"{where}: the quaternion is zero")
    return np.concatenate([arr[:3], arr[3:] / norm])


def _placement(entry: Any, where: str) -> Placement:
    if entry is None:
        return Placement()
    if not isinstance(entry, Mapping):
        raise ValueError(f"{where} must be {{on, pose}} or {{origin}}, got {entry!r}")
    # YAML 1.1 (PyYAML) reads the bare key ``on`` as the boolean true.
    entry = {("on" if k is True else k): v for k, v in entry.items()}
    unknown = sorted(set(entry) - {"on", "pose", "origin"})
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {unknown}; it takes on, pose or origin")
    if "origin" in entry and ("on" in entry or "pose" in entry):
        raise ValueError(f"{where}: origin is for a part that moves by itself; it does not go "
                         "with on / pose")
    out = Placement()
    if entry.get("on") is not None:
        on = str(entry["on"])
        part, dot, mount = on.partition(".")
        if not dot or not part or not mount or "." in mount:
            raise ValueError(f"{where}: on: is '<part>.<mount>', got {on!r}")
        out.on = (part, mount)
    if entry.get("pose") is not None:
        out.pose = _pose(entry["pose"], f"{where}: pose")
    if "origin" in entry:
        out.origin = _pose(entry["origin"] if entry["origin"] is not None else
                           [0, 0, 0, 0, 0, 0, 1], f"{where}: origin")
    return out


def _safety(value: Any, where: str, root: bool = False) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{where} must be a mapping, got {value!r}")
    unknown = sorted(set(value) - set(SAFETY_KEYS))
    if unknown:
        raise ValueError(f"{where}: unknown key(s) {unknown}; it takes {list(SAFETY_KEYS)}")
    if not root:
        only = sorted(set(value) & set(ROOT_ONLY_SAFETY))
        if only:
            raise ValueError(f"{where}: {only} only go in the root config's safety:")
    return dict(value)


def _positive_int(value: Any, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value \
            or value <= 0:
        raise ValueError(f"{where} must be a positive integer, got {value!r}")
    return int(value)


def _positive_float(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0:
        raise ValueError(f"{where} must be a positive number, got {value!r}")
    return float(value)


def _bool(value: Any, where: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{where} must be true or false, got {value!r}")
    return value


__all__ = ["DEVICE_KINDS", "DeviceSpec", "NodeSpec", "Placement", "RobotSpec", "is_tree_config",
           "joint_safety", "load_robot"]
