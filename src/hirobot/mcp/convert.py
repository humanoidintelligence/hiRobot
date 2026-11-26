"""JSON in and out of the MCP tools: observations, actions, images.

Pillow is imported only when an image is encoded.
"""

from __future__ import annotations

import io
import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from hirobot.core.types import Action, ControlMode

#: Action fields given as numbers (lists, or per-end-effector lists).
_ARRAY_FIELDS = ("joint_target", "kp", "kd", "base_velocity", "base_target", "latent")
#: Action fields the MCP tools take.
ACTION_FIELDS = ("mode", "joint_target", "kp", "kd", "joint_names", "ee_targets", "gripper",
                 "base_velocity", "base_target", "latent", "parts")


def jsonable(value: Any) -> Any:
    """``value`` with numpy arrays and scalars as lists and floats; NaN / inf become ``None``."""
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, (np.floating, float)):
        f = float(value)
        return f if math.isfinite(f) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, Mapping):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if hasattr(value, "model_dump"):
        return jsonable(value.model_dump(mode="python", exclude_none=True))
    if hasattr(value, "value") and isinstance(getattr(value, "value"), str):  # an Enum
        return value.value
    return value


def observation_dict(obs: Any, joint_names: Sequence[str]) -> Dict[str, Any]:
    """The state part of an :class:`~hirobot.core.types.Observation`, as JSON.

    Joint arrays become ``{joint name: value}``; images and depth are left out
    (see :func:`encode_image`).
    """
    out: Dict[str, Any] = {}
    for key in ("joint_pos", "joint_vel", "joint_torque"):
        arr = getattr(obs, key)
        if arr is not None:
            vals = np.asarray(arr).ravel().tolist()
            out[key] = ({n: v for n, v in zip(joint_names, vals)} if len(vals) == len(joint_names)
                        else vals)
    for key in ("base_orientation", "base_angular_velocity", "base_linear_acceleration",
                "base_position", "base_linear_velocity", "instruction"):
        val = getattr(obs, key)
        if val is not None:
            out[key] = val
    for key in ("ee_poses", "gripper", "extra"):
        val = getattr(obs, key)
        if val:
            out[key] = val
    if obs.stamps:
        out["stamps"] = {name: {"t": st.t, "seq": st.seq} for name, st in obs.stamps.items()}
    return jsonable(out)


def action_from_json(data: Any, joint_names: Sequence[str], where: str = "action") -> Action:
    """An :class:`~hirobot.core.types.Action` from a JSON object.

    ``joint_target`` / ``kp`` / ``kd`` may be a list (one value per joint, in
    embodiment order) or ``{joint name: value}``. A mapping that names every
    joint becomes a full list; one that names some sets ``joint_names`` (for
    the non-joint modes). ``parts`` maps part names to actions of the same form.
    """
    if not isinstance(data, Mapping):
        raise ValueError(f"{where}: must be an object, got {type(data).__name__}")
    unknown = sorted(set(data) - set(ACTION_FIELDS))
    if unknown:
        raise ValueError(f"{where}: unknown field(s) {unknown}; the fields are {list(ACTION_FIELDS)}")
    fields: Dict[str, Any] = {}
    names: Optional[List[str]] = None
    for key, val in data.items():
        if key == "mode":
            try:
                fields["mode"] = ControlMode(val)
            except ValueError:
                raise ValueError(f"{where}: mode {val!r} is not one of "
                                 f"{[m.value for m in ControlMode]}") from None
        elif key in ("joint_target", "kp", "kd") and isinstance(val, Mapping):
            given, arr = _by_name(val, joint_names, f"{where}.{key}")
            if names is not None and given != names:
                raise ValueError(f"{where}: {key} names other joints than the other joint fields")
            names = given
            fields[key] = arr
        elif key in _ARRAY_FIELDS:
            fields[key] = _array(val, f"{where}.{key}")
        elif key == "ee_targets":
            if not isinstance(val, Mapping):
                raise ValueError(f"{where}.ee_targets: must be an object of end-effector -> list")
            fields[key] = {k: _array(v, f"{where}.ee_targets.{k}") for k, v in val.items()}
        elif key == "parts":
            if not isinstance(val, Mapping):
                raise ValueError(f"{where}.parts: must be an object of part -> action")
            fields[key] = {k: action_from_json(v, (), f"{where}.parts.{k}") for k, v in val.items()}
        else:
            fields[key] = val
    if names is not None and names != list(joint_names):
        if "joint_names" in fields and list(fields["joint_names"]) != names:
            raise ValueError(f"{where}: joint_names disagrees with the joints named in joint_target")
        fields["joint_names"] = names
    return Action(**fields)


def _by_name(val: Mapping[str, Any], joint_names: Sequence[str], where: str) -> Tuple[List[str], np.ndarray]:
    known = set(joint_names)
    bad = [k for k in val if joint_names and k not in known]
    if bad:
        raise ValueError(f"{where}: unknown joint(s) {bad}")
    order = [n for n in joint_names if n in val] if set(val) == known else list(val)
    return order, _array([val[n] for n in order], where)


def _array(val: Any, where: str) -> np.ndarray:
    try:
        arr = np.asarray(val, dtype=np.float64)
    except (TypeError, ValueError):
        raise ValueError(f"{where}: must be a number or a list of numbers") from None
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{where}: must be finite")
    return np.atleast_1d(arr)


def encode_image(image: np.ndarray, max_side: int, quality: int) -> Tuple[bytes, Tuple[int, int]]:
    """JPEG bytes of an RGB (or grey) ``uint8`` image, shrunk so its longest side is at most
    ``max_side``; returns the bytes and the encoded ``(width, height)``. Needs Pillow."""
    try:
        from PIL import Image
    except ImportError:
        raise ImportError("images in MCP results need Pillow: pip install 'hirobot[mcp]'") from None
    arr = np.asarray(image)
    if arr.dtype != np.uint8:
        raise ValueError(f"image dtype {arr.dtype} (expected uint8)")
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    if not (arr.ndim == 2 or (arr.ndim == 3 and arr.shape[2] in (1, 3))):
        raise ValueError(f"image shape {arr.shape} (expected HxW or HxWx3)")
    img = Image.fromarray(arr.squeeze(-1) if arr.ndim == 3 and arr.shape[2] == 1 else arr)
    scale = max_side / max(img.size)
    if scale < 1:
        img = img.resize((max(1, round(img.width * scale)), max(1, round(img.height * scale))),
                         Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue(), img.size
