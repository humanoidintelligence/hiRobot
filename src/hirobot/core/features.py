"""Flatten an :class:`Observation` / :class:`Action` into named vectors (for recording).

A dataset (LeRobot's, for one) stores flat vectors with a name for every
dimension. These functions give the vectors and the names for one embodiment:
``*_features`` returns ``{feature key: [name per dimension]}``, ``flatten_*``
returns ``{feature key: float32 vector}`` with the same keys and lengths.

Observation keys:

- ``observation.state``: ``joint_pos`` (embodiment order, ``<joint>.pos``), then
  the ``gripper`` of each end-effector with a gripper (``<ee>.gripper``).
- ``observation.ee_pose``: ``ee_poses`` of every end-effector, 7 each
  (``<ee>.x`` ... ``<ee>.qw``).
- ``observation.joint_vel`` / ``observation.joint_torque``: ``<joint>.vel`` /
  ``<joint>.torque``.
- ``observation.imu``: ``base_orientation`` ``[x, y, z, w]``,
  ``base_angular_velocity``, ``base_linear_acceleration`` (``imu.qx`` ...
  ``imu.az``, 10).
- ``observation.base``: ``base_position``, ``base_linear_velocity``
  (``base.x`` ... ``base.vz``, 6).

Action keys (by ``mode``):

- joint modes: ``action`` = ``joint_target`` (``.pos`` for ``POSITION`` / ``PD``,
  ``.vel``, ``.torque``) then each gripper (``<ee>.gripper``).
- ``CARTESIAN``: ``action.ee`` (7 per end-effector); ``CARTESIAN_DELTA``:
  ``action.ee_delta`` (6 per end-effector: ``dx dy dz dax day daz``).
- ``base_velocity`` (``BASE_VELOCITY``, or beside ``CARTESIAN``):
  ``action.base_velocity`` (``vx``, ``vy``, ``wz``).
- a non-joint mode: grippers in ``action.gripper``, and joint targets named by
  ``joint_names`` in ``action`` (every joint, ``.pos``; ``NaN`` for the others).

**Missing values.** A key whose source the observation (or action) does not
fill is left out; pass the observation / action to ``*_features`` to get the
keys it fills, so a recorder can fix its schema from the first frame. A key
that is present always has the full length the embodiment gives it: a part of
it the observation lacks (one end-effector's pose, the IMU's acceleration, a
gripper) is ``NaN``. Images are not flattened: :func:`image_features` lists
their keys.

An :class:`Action` with ``parts`` is flattened in the robot's names
(``left/j0.pos``, ``left/tcp.x``); its leaves must share one mode family (the
joint modes with the same suffix, or one non-joint mode).
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from hirobot.core.dof import DofMap
from hirobot.core.embodiment import Embodiment
from hirobot.core.types import JOINT_MODES, Action, ControlMode, Observation

POSE_NAMES = ("x", "y", "z", "qx", "qy", "qz", "qw")
DELTA_NAMES = ("dx", "dy", "dz", "dax", "day", "daz")
IMU_NAMES = ("qx", "qy", "qz", "qw", "wx", "wy", "wz", "ax", "ay", "az")
BASE_NAMES = ("x", "y", "z", "vx", "vy", "vz")
BASE_VELOCITY_NAMES = ("vx", "vy", "wz")

_JOINT_SUFFIX = {ControlMode.POSITION: "pos", ControlMode.PD: "pos",
                 ControlMode.VELOCITY: "vel", ControlMode.TORQUE: "torque"}

Columns = Dict[str, Tuple[List[str], Optional[np.ndarray]]]


def _grippers(embodiment: Embodiment) -> List[str]:
    return [ee.name for ee in embodiment.end_effectors if ee.has_gripper]


def _vec(value: Any, n: int, what: str) -> np.ndarray:
    v = np.asarray(value, dtype=np.float32).reshape(-1)
    if v.shape[0] != n:
        raise ValueError(f"{what} has {v.shape[0]} values, the embodiment gives it {n}")
    return v


def _optional(value: Any, n: int, what: str) -> np.ndarray:
    return np.full(n, np.nan, dtype=np.float32) if value is None else _vec(value, n, what)


def _by_name(values: Dict[str, Any], names: List[str], width: int, what: str) -> np.ndarray:
    """``values[name]`` (``width`` numbers each) for every name, ``NaN`` where missing."""
    out = np.full(len(names) * width, np.nan, dtype=np.float32)
    for i, name in enumerate(names):
        if name in values:
            out[i * width:(i + 1) * width] = _vec(values[name], width, f"{what}[{name!r}]")
    return out


def _named(prefix: str, names: Tuple[str, ...]) -> List[str]:
    return [f"{prefix}.{n}" for n in names]


# ----------------------------------------------------------------- observation


def _observation_columns(embodiment: Embodiment, obs: Optional[Observation]) -> Columns:
    """Feature key -> (names, values); values ``None`` (and every key) without ``obs``."""
    joints = embodiment.joint_names
    n = len(joints)
    grippers = _grippers(embodiment)
    ees = [ee.name for ee in embodiment.end_effectors]
    cols: Columns = {}
    have = (lambda *fields: True) if obs is None else (
        lambda *fields: any(_filled(getattr(obs, f)) for f in fields))

    if have("joint_pos", "gripper") and (n or grippers):
        names = [f"{j}.pos" for j in joints] + [f"{g}.gripper" for g in grippers]
        values = None if obs is None else np.concatenate([
            _optional(obs.joint_pos, n, "Observation.joint_pos"),
            _by_name(obs.gripper, grippers, 1, "Observation.gripper")])
        cols["observation.state"] = (names, values)
    if ees and have("ee_poses"):
        cols["observation.ee_pose"] = (
            [f"{e}.{p}" for e in ees for p in POSE_NAMES],
            None if obs is None else _by_name(obs.ee_poses, ees, 7, "Observation.ee_poses"))
    for field, suffix in (("joint_vel", "vel"), ("joint_torque", "torque")):
        if n and have(field):
            cols[f"observation.{field}"] = (
                [f"{j}.{suffix}" for j in joints],
                None if obs is None else _vec(getattr(obs, field), n, f"Observation.{field}"))
    imu = ("base_orientation", "base_angular_velocity", "base_linear_acceleration")
    if (embodiment.floating_base if obs is None else have(*imu)):
        cols["observation.imu"] = (_named("imu", IMU_NAMES), None if obs is None else np.concatenate(
            [_optional(getattr(obs, f), k, f"Observation.{f}") for f, k in zip(imu, (4, 3, 3))]))
    base = ("base_position", "base_linear_velocity")
    if (embodiment.floating_base if obs is None else have(*base)):
        cols["observation.base"] = (_named("base", BASE_NAMES), None if obs is None else
                                    np.concatenate([_optional(getattr(obs, f), 3,
                                                              f"Observation.{f}") for f in base]))
    return cols


def _filled(value: Any) -> bool:
    if value is None:
        return False
    if isinstance(value, dict):
        return bool(value)
    return True


def observation_features(embodiment: Embodiment,
                         obs: Optional[Observation] = None) -> Dict[str, List[str]]:
    """Feature key -> the name of each dimension (see the module docstring).

    Without ``obs``: every key the embodiment can have (``observation.imu`` and
    ``observation.base`` only with a floating base). With ``obs``: the keys
    :func:`flatten_observation` returns for it.
    """
    return {k: names for k, (names, _) in _observation_columns(embodiment, obs).items()}


def flatten_observation(obs: Observation, embodiment: Embodiment) -> Dict[str, np.ndarray]:
    """Feature key -> float32 vector, one value per name of :func:`observation_features`.

    Keys whose source ``obs`` does not fill are left out; inside a key, a
    missing piece is ``NaN``. Raises ``ValueError`` when a vector's length does
    not match the embodiment.
    """
    return {k: values for k, (_, values) in _observation_columns(embodiment, obs).items()}


def image_features(embodiment: Embodiment) -> Dict[str, str]:
    """Image feature key -> camera name: ``observation.images.<cam>`` for each camera.

    Not flattened; a recorder stores ``Observation.images[cam]`` under the key.
    """
    return {f"observation.images.{c.name}": c.name for c in embodiment.cameras}


# ---------------------------------------------------------------------- action


def _family(mode: ControlMode) -> ControlMode:
    return ControlMode.POSITION if mode == ControlMode.PD else mode


def _leaves(action: Action, embodiment: Embodiment) -> Tuple[ControlMode, Action]:
    """The action in the robot's names: an ``Action.parts`` tree becomes one flat action.

    Joint targets of the leaves are placed by name (``NaN`` for joints no leaf
    names); ``ee_targets`` / ``gripper`` keys get the leaf's ``<part>/`` prefix.
    """
    if not action.parts:
        return action.mode, action
    from hirobot.assembly.parts import action_leaves, leaf_prefix, part_joint_names

    leaves = action_leaves(action)
    modes = {_family(sub.mode) for _, sub in leaves}
    if len(modes) != 1:
        raise ValueError(f"Action.parts mixes modes {sorted(m.value for m in modes)}; "
                         "flatten one mode at a time")
    mode = leaves[0][1].mode
    index = {n: i for i, n in enumerate(embodiment.joint_names)}
    target = np.full(len(index), np.nan, dtype=np.float32)
    named: List[str] = []
    ee: Dict[str, Any] = {}
    gripper: Dict[str, Any] = {}
    base_velocity = base_target = None
    for path, sub in leaves:
        pre = leaf_prefix(embodiment, path)
        if sub.joint_target is not None:
            names = ([pre + n for n in sub.joint_names] if sub.joint_names is not None
                     else part_joint_names(embodiment, path))
            target[[index[n] for n in names]] = _vec(sub.joint_target, len(names),
                                                     f"Action.parts[{path!r}].joint_target")
            named += names
        ee.update({pre + k: v for k, v in sub.ee_targets.items()})
        gripper.update({pre + k: v for k, v in sub.gripper.items()})
        base_velocity = sub.base_velocity if sub.base_velocity is not None else base_velocity
        base_target = sub.base_target if sub.base_target is not None else base_target
    joint_mode = mode in JOINT_MODES
    flat = Action.model_construct(
        mode=mode, joint_target=target if named else None,
        joint_names=None if joint_mode or not named else list(embodiment.joint_names),
        ee_targets=ee, gripper=gripper, base_velocity=base_velocity, base_target=base_target,
        kp=None, kd=None, latent=None, motion=None, append=False, extra={}, parts={})
    return mode, flat


def _action_columns(embodiment: Embodiment, mode: ControlMode,
                    action: Optional[Action]) -> Columns:
    joints = embodiment.joint_names
    n = len(joints)
    grippers = _grippers(embodiment)
    ees = [ee.name for ee in embodiment.end_effectors]
    cols: Columns = {}
    have = (lambda value: True) if action is None else _filled
    gripper_names = [f"{g}.gripper" for g in grippers]

    def joint_values(a: Action) -> np.ndarray:
        if a.joint_target is None:
            return np.full(n, np.nan, dtype=np.float32)
        if a.joint_names is None or list(a.joint_names) == joints:
            return _vec(a.joint_target, n, "Action.joint_target")
        values = _vec(a.joint_target, len(a.joint_names), "Action.joint_target")
        return DofMap(joints, a.joint_names).scatter(
            values, default=np.full(n, np.nan, dtype=np.float32))

    if mode in JOINT_MODES:
        suffix = _JOINT_SUFFIX[mode]
        if n or grippers:
            names = [f"{j}.{suffix}" for j in joints] + gripper_names
            values = None if action is None else np.concatenate([
                joint_values(action), _by_name(action.gripper, grippers, 1, "Action.gripper")])
            cols["action"] = (names, values)
        return cols

    # A non-joint mode: joint targets beside it (joint_names) and grippers are columns of their own.
    if action is not None and action.joint_target is not None:
        cols["action"] = ([f"{j}.pos" for j in joints], joint_values(action))
    if grippers and have(None if action is None else action.gripper):
        cols["action.gripper"] = (gripper_names, None if action is None else
                                  _by_name(action.gripper, grippers, 1, "Action.gripper"))
    if mode in (ControlMode.CARTESIAN, ControlMode.CARTESIAN_DELTA) and ees:
        key, parts, width = (("action.ee", POSE_NAMES, 7) if mode == ControlMode.CARTESIAN
                             else ("action.ee_delta", DELTA_NAMES, 6))
        cols[key] = ([f"{e}.{p}" for e in ees for p in parts], None if action is None else
                     _by_name(action.ee_targets, ees, width, "Action.ee_targets"))
    walks = mode == ControlMode.BASE_VELOCITY or (
        mode == ControlMode.CARTESIAN and embodiment.floating_base)
    if (walks if action is None else action.base_velocity is not None):
        cols["action.base_velocity"] = (
            _named("base", BASE_VELOCITY_NAMES), None if action is None else
            _vec(action.base_velocity, 3, "Action.base_velocity"))
    if mode == ControlMode.BASE_POSE and (action is None or action.base_target is not None):
        cols["action.base_target"] = (["base.x", "base.y", "base.yaw"], None if action is None
                                      else _vec(action.base_target, 3, "Action.base_target"))
    return cols


def action_features(embodiment: Embodiment, mode: ControlMode = ControlMode.POSITION,
                    action: Optional[Action] = None) -> Dict[str, List[str]]:
    """Feature key -> the name of each dimension, for actions in ``mode``.

    Without ``action``: the keys an action in ``mode`` can have (joint targets
    beside a non-joint mode are left out: they depend on the robot's
    ``direct_joints``). With ``action``: the keys :func:`flatten_action` returns
    for it (``mode`` is then taken from the action).
    """
    if action is not None:
        mode, action = _leaves(action, embodiment)
    return {k: names for k, (names, _) in _action_columns(embodiment, ControlMode(mode),
                                                          action).items()}


def flatten_action(action: Action, embodiment: Embodiment) -> Dict[str, np.ndarray]:
    """Feature key -> float32 vector, one value per name of :func:`action_features`.

    Joints follow the embodiment's order whatever ``joint_names`` says; a joint
    or end-effector the action does not command is ``NaN``. ``kp`` / ``kd``,
    ``latent``, ``motion`` and ``extra`` are not flattened.
    """
    mode, flat = _leaves(action, embodiment)
    return {k: values for k, (_, values) in _action_columns(embodiment, mode, flat).items()}


__all__ = ["action_features", "flatten_action", "flatten_observation", "image_features",
           "observation_features"]
