"""``mink_ik``: end-effector targets to joint targets by differential IK.

A passive :class:`~hirobot.control.layer.Layer`: each ``send_action`` in
``CARTESIAN`` or ``CARTESIAN_DELTA`` runs a few `mink <https://github.com/kevinzakka/mink>`_
QP steps on a MuJoCo model of the robot and sends one ``POSITION`` action with
every joint to the robot below (through ``inner.send_action``, so the gate
there checks it). Every other mode passes through unchanged.

Config (``layers:`` entry)::

    layers:
      - type: mink_ik
        model: ~/models/arm.xml        # MJCF, or a config whose one body has model: mjcf:
        frames:                        # optional: default every end-effector with a frame
          tcp: {name: wrist_3_link, type: body}   # type: body | site | geom

**Frame.** Targets and the ``ee_poses`` this layer reports are in the robot's
base frame: the root body of the tree the joints are on (a humanoid's pelvis,
an arm's base link), the frame the MuJoCo driver reports ``ee_poses`` in. A
floating base's free joint is frozen (a constraint in the QP) and never solved
for; since targets are relative to the base, the base pose does not enter.

**Targets.** ``CARTESIAN``: ``ee_targets[name]`` is a pose ``[x, y, z, qx, qy,
qz, qw]``. ``CARTESIAN_DELTA``: ``[dx, dy, dz, dax, day, daz]`` per tick,
position plus an axis-angle rotation applied in the base frame, added to the
last target of that end-effector (not to the measured pose, so a small delta is
not lost to tracking error). An end-effector the action does not name keeps its
last target, i.e. stays where it is. The targets are seeded from the measured
pose on first use, after :meth:`reset`, after :meth:`hold` and after a
passed-through command. ``joint_names`` / ``joint_target`` may set joints the
IK model does not have (``direct_joints``, e.g. fingers); ``gripper`` is passed on.

**Failures.** A solver error or a non-finite solution sends the previous joint
target again (logged), never raises. An unreachable target is not an error: the
IK moves as close as the limits allow.

Needs ``pip install 'hirobot[ik]'`` (mink >= 1.4, Python >= 3.11); both are
imported when the layer is built, not when ``hirobot`` is.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np

from hirobot.control.layer import Layer
from hirobot.control.safety import SafetyError
from hirobot.core.robot import Robot
from hirobot.core.types import Action, ControlMode, Observation
from hirobot.geometry.transforms import (
    axis_angle_to_quat,
    pose_compose,
    pose_inverse,
    quat_multiply,
    quat_normalize,
)
from hirobot.loading.registry import register_layer

logger = logging.getLogger(__name__)

IK_MODES = (ControlMode.CARTESIAN, ControlMode.CARTESIAN_DELTA)
FRAME_TYPES = ("body", "site", "geom")
_IDENTITY = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
_INSTALL = "pip install 'hirobot[ik]'"


def _import_ik() -> Tuple[Any, Any]:
    try:
        import mink
        import mujoco
    except ImportError as exc:
        raise ImportError(f"the mink_ik layer needs mink and mujoco (Python >= 3.11): "
                          f"{_INSTALL} ({exc})") from exc
    return mink, mujoco


def resolve_model(model: Any) -> str:
    """An MJCF path, from a file path or from a config whose one body has ``model: mjcf:``."""
    if not isinstance(model, str) or not model:
        raise ValueError("mink_ik: model: must be an MJCF path or a robot config name")
    path = os.path.expanduser(model)
    is_config = path.endswith((".yaml", ".yml"))
    if not is_config and (os.path.isfile(path) or path.endswith(".xml")):
        return path
    from hirobot.loading.build import only_body
    from hirobot.loading.tree import load_robot

    _, dev = only_body(load_robot(path if is_config else model), "mink_ik model:")
    body_model = dev.fields.get("model") or {}
    mjcf = body_model.get("mjcf")
    if not mjcf and body_model.get("urdf"):
        raise ValueError(f"mink_ik: model: {model} has only a urdf; mink_ik needs an mjcf")
    if not mjcf:
        raise ValueError(f"mink_ik: model: {model!r} has no body model: mjcf:")
    return os.path.expanduser(str(mjcf))


def _wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[3], q[0], q[1], q[2]])


def _xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]])


@register_layer("mink_ik")
class MinkIkLayer(Layer):
    """End-effector pose targets to joint ``POSITION`` targets with mink.

    Args:
        inner: The robot below.
        model: MJCF of the robot (path, ``~`` expanded), or the name / path of
            a config whose one body has ``model: mjcf:`` (``unitree_g1``). If
            left out, the MuJoCo model of a simulated robot below
            (``inner.model``) is used on :meth:`connect`; a real robot needs it.
        frames: End-effector name -> ``{name: <MJCF body/site/geom>, type:
            body|site|geom}``. Default: every end-effector of the embodiment
            with a ``frame`` (its body plus offset).
        dt: Seconds one ``send_action`` integrates; default one control period
            (``1 / embodiment.control_hz``).
        iterations: QP steps per ``send_action`` (each ``dt / iterations``).
        position_cost / orientation_cost: Weights of the end-effector tasks.
        posture_cost: Weight pulling every joint towards the pose at the last
            re-seed, which keeps the redundant joints (legs, waist) still.
        damping: Damping of the QP (``mink.solve_ik``).
        lm_damping: Levenberg-Marquardt damping of the end-effector tasks.
        max_joint_velocity: rad/s (or m/s) per joint in the velocity limit;
            ``None`` turns the limit off.
        joint_limits: Keep the solution inside the model's joint ranges.
        solver: QP solver name for qpsolvers (``daqp`` comes with mink).
    """

    def __init__(self, inner: Robot, model: Optional[str] = None,
                 frames: Optional[Mapping[str, Mapping[str, Any]]] = None, *,
                 dt: Optional[float] = None, iterations: int = 3,
                 position_cost: float = 1.0, orientation_cost: float = 0.5,
                 posture_cost: float = 1e-2, damping: float = 1e-3, lm_damping: float = 1e-3,
                 max_joint_velocity: Optional[float] = 3.0, joint_limits: bool = True,
                 solver: str = "daqp") -> None:
        super().__init__(inner)
        if ControlMode.POSITION not in tuple(self.inner.action_modes):
            raise ValueError("mink_ik: the robot below must take POSITION actions")
        self._mink, self._mujoco = _import_ik()
        self._frames = self._frame_specs(frames)
        hz = self.inner.embodiment.control_hz
        self._dt = float(dt) if dt is not None else (1.0 / hz if hz else 0.02)
        if self._dt <= 0 or int(iterations) < 1:
            raise ValueError("mink_ik: dt must be > 0 and iterations >= 1")
        self._iterations = int(iterations)
        self._costs = (float(position_cost), float(orientation_cost), float(posture_cost))
        self._damping, self._lm_damping = float(damping), float(lm_damping)
        self._max_vel = None if max_joint_velocity is None else float(max_joint_velocity)
        self._joint_limits = bool(joint_limits)
        self._solver = str(solver)
        self._ik_lock = threading.RLock()
        self._model_ref = model
        self._ik: Optional[Dict[str, Any]] = None  # the model and everything built from it
        self._q: Optional[np.ndarray] = None        # the IK configuration (qpos), commanded
        self._targets: Optional[Dict[str, np.ndarray]] = None  # base frame, per end-effector
        self._last_cmd: Optional[np.ndarray] = None  # last joint target sent (embodiment order)
        self._sent_cmd: Optional[np.ndarray] = None  # what the robot below accepted; None: none
        #: Solve timing and failures since construction (``count``, ``mean_ms``,
        #: ``max_ms``, ``failures``).
        self.stats: Dict[str, float] = {"count": 0, "mean_ms": 0.0, "max_ms": 0.0, "failures": 0}
        if model is not None:
            self._build(self._mujoco.MjModel.from_xml_path(resolve_model(model)))

    # ------------------------------------------------------------ declarations

    @property
    def action_modes(self):  # type: ignore[override]
        modes = tuple(self.inner.action_modes)
        return modes + tuple(m for m in IK_MODES if m not in modes)

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return tuple(self._frames)

    @property
    def direct_joints(self):  # type: ignore[override]
        """The robot's own free joints, plus joints the IK model does not have (fingers)."""
        own = tuple(self.inner.direct_joints)
        if self._ik is None:
            return own
        free = tuple(n for n in self.embodiment.joint_names if n not in self._ik["index"])
        return own + tuple(n for n in free if n not in own)

    # --------------------------------------------------------------- set up

    def _frame_specs(self, frames: Optional[Mapping[str, Any]]) -> Dict[str, Tuple[str, str,
                                                                                 np.ndarray]]:
        """End-effector -> (MJCF name, type, offset pose in that frame)."""
        out: Dict[str, Tuple[str, str, np.ndarray]] = {}
        if frames is None:
            for ee in self.inner.embodiment.end_effectors:
                if ee.frame is not None:
                    out[ee.name] = (ee.frame.body, "body",
                                    np.array([*ee.frame.pos, *ee.frame.quat], dtype=float))
            if not out:
                raise ValueError("mink_ik: the robot has no end-effector with a frame; "
                                 "give frames: {<ee>: {name: <body or site>, type: body}}")
            return out
        if not isinstance(frames, Mapping) or not frames:
            raise ValueError("mink_ik: frames: must map end-effector names to "
                             "{name: <body or site>, type: body|site|geom}")
        for ee, spec in frames.items():
            if not isinstance(spec, Mapping) or not isinstance(spec.get("name"), str):
                raise ValueError(f"mink_ik: frames.{ee} needs name: (an MJCF body / site / geom)")
            unknown = sorted(set(spec) - {"name", "type"})
            kind = spec.get("type", "body")
            if unknown or kind not in FRAME_TYPES:
                raise ValueError(f"mink_ik: frames.{ee} takes name and type ({'|'.join(FRAME_TYPES)})"
                                 f", got {dict(spec)!r}")
            out[str(ee)] = (spec["name"], kind, _IDENTITY.copy())
        return out

    def _build(self, model: Any) -> None:
        """Map joints and frames in ``model``; set up the tasks and limits."""
        mink, mj = self._mink, self._mujoco
        names = self.inner.embodiment.joint_names
        index: Dict[str, int] = {}
        qadr: List[int] = []
        jids: List[int] = []
        vel: Dict[str, float] = {}
        for i, name in enumerate(names):
            jid = -1
            for cand in (name, f"{name}_joint"):  # as the MuJoCo driver matches them
                jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, cand)
                if jid >= 0:
                    break
            if jid < 0 or int(model.jnt_type[jid]) not in (int(mj.mjtJoint.mjJNT_HINGE),
                                                           int(mj.mjtJoint.mjJNT_SLIDE)):
                continue
            index[name] = i
            qadr.append(int(model.jnt_qposadr[jid]))
            jids.append(jid)
            if self._max_vel is not None:
                vel[model.joint(jid).name] = self._max_vel
        if not index:
            raise ValueError(f"mink_ik: none of the robot's joints ({names[:3]}...) is in the "
                             "IK model (as <name> or <name>_joint)")
        missing = []
        for ee, (fname, kind, _) in self._frames.items():
            obj = getattr(mj.mjtObj, f"mjOBJ_{kind.upper()}")
            if mj.mj_name2id(model, obj, fname) < 0:
                missing.append(f"{ee}: {kind} {fname!r}")
        if missing:
            raise ValueError(f"mink_ik: frame(s) not in the IK model: {missing}")
        # Every other degree of freedom (a free joint, joints the robot does not
        # have) is frozen: an equality constraint of the QP, never solved for.
        moving = {int(model.jnt_dofadr[j]) for j in jids}
        frozen = [d for d in range(model.nv) if d not in moving]
        pos_cost, ori_cost, posture_cost = self._costs
        limits: List[Any] = []
        if self._joint_limits:
            limits.append(mink.ConfigurationLimit(model))
        if vel:
            limits.append(mink.VelocityLimit(model, vel))
        lo = np.where(model.jnt_limited[jids], model.jnt_range[jids, 0], -np.inf)
        hi = np.where(model.jnt_limited[jids], model.jnt_range[jids, 1], np.inf)
        self._ik = {
            "model": model, "index": index,
            "emb_idx": np.array(list(index.values()), dtype=int),
            "qadr": np.array(qadr, dtype=int), "range": (lo, hi),
            "root": int(model.body_rootid[model.jnt_bodyid[jids[0]]]),
            "tasks": {ee: mink.FrameTask(fname, kind, position_cost=pos_cost,
                                         orientation_cost=ori_cost, lm_damping=self._lm_damping)
                      for ee, (fname, kind, _) in self._frames.items()},
            "posture": mink.PostureTask(model, cost=posture_cost),
            "limits": limits,
            "constraints": [mink.DofFreezingTask(model, dof_indices=frozen)] if frozen else [],
            "solve": mink.Configuration(model), "fk": mink.Configuration(model),
        }

    def _need_ik(self) -> Dict[str, Any]:
        if self._ik is None:
            raise RuntimeError("mink_ik: no IK model yet; call connect() first")
        return self._ik

    # -------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        self.inner.connect()
        if self._ik is not None:
            return
        import copy

        sim_model = getattr(self.inner, "model", None)
        if not isinstance(sim_model, self._mujoco.MjModel):
            raise ValueError("mink_ik: give model: (an MJCF path, or a config whose body has "
                             "model: mjcf:); the robot below is not a MuJoCo simulation")
        with self._ik_lock:
            self._build(copy.copy(sim_model))  # a copy: the simulation's model is not touched

    def reset(self, seed: Optional[int] = None) -> None:
        self.inner.reset(seed)
        self._reseed()

    def hold(self) -> None:
        self.inner.hold()
        self._reseed()

    def _reseed(self) -> None:
        """Forget the targets: the next IK command starts from the measured pose."""
        with self._ik_lock:
            self._q = None
            self._targets = None
            self._last_cmd = None
            self._sent_cmd = None

    @property
    def commanded_ee_poses(self) -> Dict[str, np.ndarray]:
        """Each frame's pose (base frame) at the last joint target sent; empty before one.

        The difference to ``get_observation().ee_poses`` is the robot's tracking
        error (an arm sagging under gravity on PD), not the IK's.
        """
        with self._ik_lock:
            if self._ik is None or self._q is None:
                return {}
            fk = self._ik["fk"]
            fk.update(self._q)
            return self._poses(fk)

    @property
    def commanded_joint_targets(self) -> Optional[Dict[str, float]]:
        """Joint name -> the ``POSITION`` target last sent down for an IK command.

        Covers every joint of that ``POSITION`` action (embodiment names), including
        ``direct_joints`` the action set. A recorder reads it after each command to
        store the IK's joint targets next to the requested ``CARTESIAN`` /
        ``CARTESIAN_DELTA`` action. A solver failure resends the previous target, so
        this keeps that target; a command the robot below refused changes nothing.
        ``None`` before the first IK command and after :meth:`reset`, :meth:`hold`
        or a passed-through command in another mode. A copy: changing it does not
        change the layer.
        """
        with self._ik_lock:
            if self._sent_cmd is None:
                return None
            return {name: float(v)
                    for name, v in zip(self.embodiment.joint_names, self._sent_cmd)}

    # ---------------------------------------------------------------- sensing

    def _qpos(self, joint_pos: Optional[np.ndarray]) -> np.ndarray:
        """A ``qpos`` of the IK model with the robot's joints from ``joint_pos``."""
        ik = self._need_ik()
        q = ik["model"].qpos0.copy()
        if joint_pos is not None:
            measured = np.asarray(joint_pos, dtype=float)[ik["emb_idx"]]
            ok = np.isfinite(measured)
            q[ik["qadr"][ok]] = measured[ok]
        return q

    def _root_pose(self, cfg: Any) -> np.ndarray:
        data, root = cfg.data, self._ik["root"]
        return np.concatenate([data.xpos[root], _xyzw(data.xquat[root])])

    def _poses(self, cfg: Any) -> Dict[str, np.ndarray]:
        """Each frame's pose in the base frame, for ``cfg`` (already updated)."""
        to_base = pose_inverse(self._root_pose(cfg))
        out = {}
        for ee, (fname, kind, offset) in self._frames.items():
            T = cfg.get_transform_frame_to_world(fname, kind)
            world = np.concatenate([T.translation(), _xyzw(T.rotation().wxyz)])
            out[ee] = pose_compose(to_base, pose_compose(world, offset))
        return out

    def get_observation(self) -> Observation:
        obs = self.inner.get_observation()
        if self._ik is None:
            return obs
        with self._ik_lock:
            fk = self._ik["fk"]
            fk.update(self._qpos(obs.joint_pos))
            obs.ee_poses = {**obs.ee_poses, **self._poses(fk)}
        return obs

    # --------------------------------------------------------------- commands

    def _send_action(self, action: Action) -> None:
        if action.mode not in IK_MODES:
            self.inner.send_action(action)
            self._reseed()  # the robot was moved some other way
            return
        with self._ik_lock:
            self._send_ik(action)

    def _seed(self) -> np.ndarray:
        """Start from the measured pose: the IK configuration and the targets."""
        ik = self._need_ik()
        obs = self.inner.get_observation()
        q = self._qpos(obs.joint_pos)
        lo, hi = ik["range"]
        q[ik["qadr"]] = np.clip(q[ik["qadr"]], lo, hi)  # a reading just past a limit
        self._q = q
        cfg = ik["solve"]
        cfg.update(q)
        self._targets = self._poses(cfg)
        ik["posture"].set_target(q)
        cmd = np.asarray(obs.joint_pos, dtype=float).copy()
        cmd[ik["emb_idx"]] = q[ik["qadr"]]
        return cmd

    def _new_targets(self, action: Action) -> Dict[str, np.ndarray]:
        """The targets after ``action`` (base frame); raises ``SafetyError`` for a bad one."""
        targets = dict(self._targets or {})
        unknown = sorted(set(action.ee_targets) - set(self._frames))
        if unknown:
            raise SafetyError(f"mink_ik: no IK frame for end-effector(s) {unknown}; "
                              f"cartesian_targets: {list(self._frames)}")
        delta = action.mode == ControlMode.CARTESIAN_DELTA
        for ee, value in action.ee_targets.items():
            v = np.asarray(value, dtype=float).reshape(-1)
            if v.shape != ((6,) if delta else (7,)):
                want = "[dx, dy, dz, dax, day, daz]" if delta else "[x, y, z, qx, qy, qz, qw]"
                raise SafetyError(f"mink_ik: ee_targets[{ee!r}] must be {want}, got {v.shape}")
            if delta:
                old = targets[ee]
                quat = quat_multiply(axis_angle_to_quat(v[3:]), old[3:])
                targets[ee] = np.concatenate([old[:3] + v[:3], quat_normalize(quat)])
            else:
                targets[ee] = np.concatenate([v[:3], quat_normalize(v[3:])])
        return targets

    def _send_ik(self, action: Action) -> None:
        ik = self._need_ik()
        if self._q is None:
            self._last_cmd = self._seed()
        targets = self._new_targets(action)
        mink = self._mink
        cfg = ik["solve"]
        cfg.update(self._q)
        root = self._root_pose(cfg)  # fixed: the base is frozen
        tasks = [ik["posture"]]
        for ee, pose in targets.items():
            # The target is the end-effector frame (link pose composed with its
            # offset, as _poses reports it); the task moves the link itself.
            offset = self._frames[ee][2]
            world = pose_compose(pose_compose(root, pose), pose_inverse(offset))
            task = ik["tasks"][ee]
            task.set_target(mink.SE3.from_rotation_and_translation(
                mink.SO3(_wxyz(world[3:])), world[:3]))
            tasks.append(task)
        step = self._dt / self._iterations
        t0 = time.perf_counter()
        try:
            for _ in range(self._iterations):
                v = mink.solve_ik(cfg, tasks, step, self._solver, damping=self._damping,
                                  limits=ik["limits"], constraints=ik["constraints"] or None)
                if not np.all(np.isfinite(v)):
                    raise FloatingPointError("non-finite joint velocity")
                cfg.integrate_inplace(v, step)
            q = cfg.q.copy()
        except Exception as exc:  # noqa: BLE001 - hold the previous target instead
            self.stats["failures"] += 1
            logger.warning("mink_ik: IK step failed (%s: %s); holding the previous target",
                           type(exc).__name__, exc)
            q = None
        self._time(time.perf_counter() - t0)
        cmd = self._last_cmd.copy()
        if q is not None:
            cmd[ik["emb_idx"]] = q[ik["qadr"]]
        if action.joint_names:  # joints outside the IK model (direct_joints)
            names = self.embodiment.joint_names
            for name, value in zip(action.joint_names, np.asarray(action.joint_target, float)):
                cmd[names.index(name)] = value
        # Refused below (the gate raises): nothing changes here, the caller gets the error.
        self.inner.send_action(Action(mode=ControlMode.POSITION, joint_target=cmd,
                                      gripper=dict(action.gripper)))
        self._last_cmd = cmd
        self._sent_cmd = cmd
        if q is not None:  # a failed step keeps the previous targets too
            self._q = q
            self._targets = targets

    def _time(self, seconds: float) -> None:
        s = self.stats
        ms = seconds * 1e3
        s["count"] += 1
        s["mean_ms"] += (ms - s["mean_ms"]) / s["count"]
        s["max_ms"] = max(s["max_ms"], ms)


__all__ = ["FRAME_TYPES", "IK_MODES", "MinkIkLayer", "resolve_model"]
