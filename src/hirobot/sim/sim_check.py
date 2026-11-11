"""The safety gate's optional simulation check: predict a command in MuJoCo first.

For every command that passed the gate's other checks, :class:`SimCheck` copies
the robot's current joint state into a private MuJoCo model, applies the command
for ``horizon_steps`` control periods, and refuses it
(:class:`~hirobot.control.safety.SafetyError`) when the prediction shows

* a joint torque above ``torque_ratio`` times the joint's limit
  (``actuatorfrcrange``), computed from the PD law before clamping;
* a new or deeper self-collision (two bodies of the robot touching);
* the robot sinking deeper into the ground than it was at the start.

Contacts already there at the start do not block a command unless it makes them
deeper, so a robot that is already touching can move out of it. The model only
has the robot and the ground: tables and people are not checked.

The check is extra, not a replacement: it runs last, only on the hardware gate,
and only refuses (``clamp`` never applies to it). It depends on how well the
model matches the robot, so it can miss or refuse wrongly. When the prediction
itself fails (an exception, the model does not load, it took longer than
``budget_s``) the command is refused by default (``on_error: reject``);
``on_error: warn`` lets it through with a warning and is for debugging only.
After ``max_failures`` failures in a row the robot enters the fault state.

MuJoCo is imported, and the model loaded, on the first check.
"""

from __future__ import annotations

import os
import logging
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, Mapping, Optional, Sequence

import numpy as np

logger = logging.getLogger(__name__)

#: Keys the ``safety.sim_check`` config section takes.
KEYS = ("model", "scene", "fixed_base", "base_height", "horizon_steps", "torque_ratio",
        "self_collision", "ground", "budget_s", "on_error", "max_failures",
        "penetration_tolerance_m", "ee_tolerance_m")

from hirobot.control.safety import _HOLDING, holding  # noqa: E402,F401 - re-exported for drivers

#: Modes the check can predict; other commands pass unchecked (counted as skipped).
_JOINT_MODES = ("position", "pd", "velocity", "torque")


class SimCheckFailed(RuntimeError):
    """The prediction itself failed (not the command)."""


class SimCheck:
    """Predict each command in MuJoCo and refuse the unsafe ones.

    Args:
        model: Where the check world comes from. :func:`~hirobot.make_robot`
            passes the robot's own tree (a :class:`~hirobot.loading.tree.RobotSpec`:
            every body with its ``model.mjcf``, placed as in the config); by
            hand, an MJCF ``.xml`` path of a one-body robot.
        scene: An MJCF with fixed obstacles (a table) to check against.
        fixed_base: Part paths of bodies welded in place (as in ``sim:``); by
            default the robot's ``sim.fixed_base``.
        base_height: Part path -> height (m) of a welded body's root; by
            default the robot's ``sim.base_height``.
        horizon_steps: Control periods to predict per command.
        torque_ratio: Refuse when a joint's predicted torque exceeds this
            fraction of its limit.
        self_collision: Check contacts between the robot's own bodies.
        ground: Check contacts with the ground (any geom of the world body).
        budget_s: Longest a check may take; ``None`` is half a control period.
        on_error: ``"reject"`` (default) or ``"warn"`` when the prediction fails.
        max_failures: Failures in a row that put the robot in the fault state.
        penetration_tolerance_m: Contact depth below which a contact is ignored.
        ee_tolerance_m: A measured end-effector position farther than this
            from the model's (same joint angles) is counted and warned about.
    """

    def __init__(self, model: Any = None, scene: Any = None,
                 fixed_base: Optional[Sequence[str]] = None,
                 base_height: Optional[Mapping[str, float]] = None, horizon_steps: int = 5,
                 torque_ratio: float = 1.0, self_collision: bool = True, ground: bool = True,
                 budget_s: Optional[float] = None, on_error: str = "reject",
                 max_failures: int = 3, penetration_tolerance_m: float = 0.005,
                 ee_tolerance_m: float = 0.02) -> None:
        if on_error not in ("reject", "warn"):
            raise ValueError(f"safety.sim_check.on_error must be 'reject' or 'warn', got {on_error!r}")
        if int(horizon_steps) < 1 or int(max_failures) < 1:
            raise ValueError("safety.sim_check: horizon_steps and max_failures must be >= 1")
        if float(torque_ratio) <= 0:
            raise ValueError("safety.sim_check.torque_ratio must be > 0")
        if isinstance(scene, Mapping):
            scene = scene.get("mjcf")
        self.model = model
        self.scene = None if scene is None else os.path.expanduser(str(scene))
        self.fixed_base = None if fixed_base is None else [str(p).strip("/") for p in fixed_base]
        self.base_height = None if base_height is None else dict(base_height)
        self.ee_tolerance_m = float(ee_tolerance_m)
        self.horizon_steps = int(horizon_steps)
        self.torque_ratio = float(torque_ratio)
        self.self_collision = bool(self_collision)
        self.ground = bool(ground)
        self.budget_s = None if budget_s is None else float(budget_s)
        self.on_error = on_error
        self.max_failures = int(max_failures)
        self.tolerance = float(penetration_tolerance_m)
        self._sim: Any = None
        self._load_error: Optional[BaseException] = None
        self._lock = threading.Lock()
        #: Counters reported in ``get_status().safety["sim_check"]``.
        self.checked = 0
        self.rejected = 0
        self.errors = 0
        self.skipped = 0
        #: Failures in a row (reset by a check that ran to the end).
        self.failures_in_row = 0
        self.last_reason: Optional[str] = None
        self._times: Deque[float] = deque(maxlen=200)

    @classmethod
    def from_config(cls, cfg: Mapping[str, Any]) -> "SimCheck":
        unknown = sorted(set(cfg) - set(KEYS))
        if unknown:
            raise ValueError(f"safety.sim_check: unknown key(s) {unknown}; takes {list(KEYS)}")
        return cls(**dict(cfg))

    # ------------------------------------------------------------- status

    def stats(self) -> Dict[str, Any]:
        """Counters for ``get_status()``: how often it refused, failed, skipped."""
        times = sorted(self._times)
        p99 = times[min(len(times) - 1, int(0.99 * len(times)))] * 1e3 if times else None
        sim = self._sim
        return {"checked": self.checked, "rejected": self.rejected, "errors": self.errors,
                "skipped": self.skipped, "failures_in_row": self.failures_in_row,
                "p99_ms": p99, "last_reason": self.last_reason,
                "ee_mismatch": getattr(sim, "ee_mismatch", 0),
                "ee_error_m": getattr(sim, "ee_error_m", None)}

    # ------------------------------------------------------------- model

    def prepare(self, embodiment: Any) -> None:
        """Load the model now (``make_robot`` does, so the first command does not pay for it).

        A failure is logged and kept: every check then refuses with it.
        """
        with self._lock:
            try:
                self._load(embodiment)
            except SimCheckFailed:
                pass  # logged in _load; every check refuses with it

    def _load(self, embodiment: Any) -> Any:
        if self._sim is not None:
            return self._sim
        if self._load_error is not None:
            raise SimCheckFailed(f"model did not load: {self._load_error}") from self._load_error
        try:
            self._sim = _build_sim(self, embodiment)
        except Exception as exc:  # noqa: BLE001 - kept and reported on every check
            logger.error("safety.sim_check: model did not load: %r", exc)
            self._load_error = exc
            raise SimCheckFailed(f"model did not load: {exc}") from exc
        return self._sim

    # ------------------------------------------------------------- check

    def check(self, action: Any, robot: Any) -> None:
        """Predict ``action`` from ``robot``'s current state; raise ``SafetyError`` to refuse.

        Puts ``robot`` in the fault state after ``max_failures`` failed
        predictions in a row.
        """
        from hirobot.control.safety import SafetyError

        mode = getattr(getattr(action, "mode", None), "value", None)
        if (_HOLDING.get() or mode not in _JOINT_MODES or action.joint_target is None
                or action.joint_names is not None):
            self.skipped += 1  # nothing joint-level to predict (or only some joints)
            return
        budget = self.budget_s
        if budget is None:
            hz = getattr(robot.embodiment, "control_hz", None) or 50
            budget = 0.5 / hz
        with self._lock:
            try:
                sim = self._load(robot.embodiment)
                obs = robot.get_observation()
                start = time.perf_counter()
                reason = sim.predict(action, obs, self)
                took = time.perf_counter() - start
                if took > budget:
                    raise SimCheckFailed(f"prediction took {took * 1e3:.1f} ms, budget "
                                         f"{budget * 1e3:.1f} ms")
            except Exception as exc:  # noqa: BLE001 - fail closed (or warn), below
                self._failed(exc, robot)
                if self.on_error == "warn":
                    logger.warning("safety.sim_check failed, command let through "
                                   "(on_error: warn): %s", exc)
                    return
                raise SafetyError(f"simulation check failed (not the command): {exc}") from exc
            self._times.append(took)
            self.checked += 1
            self.failures_in_row = 0
            if reason:
                self.rejected += 1
                self.last_reason = reason
                raise SafetyError(f"simulation check: {reason}")

    def _failed(self, exc: BaseException, robot: Any) -> None:
        self.errors += 1
        self.failures_in_row += 1
        self.last_reason = f"check failed: {exc}"
        if self.failures_in_row >= self.max_failures and robot.fault is None:
            robot._set_fault(f"safety.sim_check failed {self.failures_in_row} times in a row "
                             f"(last: {exc})")


def _build_sim(check: SimCheck, embodiment: Any) -> "_Sim":
    """The check world: the robot's tree (a ``RobotSpec``) or an MJCF ``.xml`` path."""
    model = check.model
    if model is None or model == "":
        raise SimCheckFailed("no model: make_robot builds the check from the robot's tree; "
                             "a SimCheck made by hand needs model= (an MJCF path or a RobotSpec)")
    if isinstance(model, (str, os.PathLike)):
        if not str(model).endswith(".xml"):
            raise SimCheckFailed(
                f"model {model!r}: give an MJCF .xml path; the check builds the rest from the "
                "robot's own tree (each body's model:), not from a *_mujoco config")
        spec = _single_body_spec(str(model), embodiment)
    else:
        spec = model
    sim = dict(getattr(spec, "sim", None) or {})
    fixed = check.fixed_base if check.fixed_base is not None else sim.get("fixed_base") or []
    heights = check.base_height if check.base_height is not None else sim.get("base_height") or {}
    return _Sim(spec, embodiment, scene=check.scene, fixed_base=fixed, base_height=heights)


def _single_body_spec(mjcf: str, embodiment: Any) -> Any:
    """A one-body tree around an MJCF, for a ``SimCheck(model="x.xml")`` made by hand."""
    from hirobot.loading.tree import load_robot

    body = {"device": "body", "control_hz": embodiment.control_hz or 50,
            "floating_base": bool(embodiment.floating_base),
            "joints": [{"name": n} for n in embodiment.joint_names],
            "model": {"mjcf": os.path.expanduser(mjcf)}}
    return load_robot({"assembly": {"name": embodiment.name, "parts": {"body": body}}})


def _check_names(spec: Any, layout: Any, embodiment: Any) -> None:
    """Every joint of the robot is in the check world and vice versa; else ``ValueError``."""
    names = list(layout.embodiment.joint_names)
    if names != list(embodiment.joint_names):
        missing = [n for n in embodiment.joint_names if n not in names]
        extra = [n for n in names if n not in embodiment.joint_names]
        raise ValueError(f"the check world's joints differ from the robot's: missing {missing}, "
                         f"extra {extra}")


class _Sim:
    """The prediction: the robot's tree in one MuJoCo world, reset for every command.

    Every body is a view of the world (:func:`hirobot.loading.build.world_views`);
    its joints are a slice of the robot's ``joint_pos``. The world is reset to
    the measured state before each prediction, so predictions do not drift.
    """

    def __init__(self, spec: Any, embodiment: Any, scene: Optional[str] = None,
                 fixed_base: Sequence[str] = (), base_height: Optional[Mapping[str, float]] = None
                 ) -> None:
        from hirobot.loading.build import world_views
        from hirobot.sim.sim_world import sim_options

        for path, part in spec.root.walk():
            if getattr(part, "kind", None) == "body" and not part.fields.get("model"):
                raise ValueError(f"{path}: every body needs a model: (its MJCF) for the simulation "
                                 "check; checking only some of the parts would miss collisions")
        options = sim_options({"realtime": False, "viewer": False, "fixed_base": list(fixed_base),
                               "base_height": dict(base_height or {}),
                               **({"scene": {"mjcf": scene}} if scene else {})})
        world, views, layout = world_views(spec, options)  # no cameras: nothing renders here
        _check_moving(layout)
        _check_names(spec, layout, embodiment)
        world.connect()
        self.world, self.views = world, views
        self.names = list(layout.embodiment.joint_names)
        self._floating = bool(layout.floating)
        m, mj = world.model, world.mj
        index = {n: i for i, n in enumerate(self.names)}
        # Per view: where its joints are in the robot's joint_pos.
        self._slices = [np.array([index[v._prefix + n] for n in v.embodiment.joint_names],
                                 dtype=int) for v in views]
        n = len(self.names)
        self._kp, self._kd = np.zeros(n), np.zeros(n)
        self._tau_lo, self._tau_hi = np.full(n, -np.inf), np.full(n, np.inf)
        for v, idx in zip(views, self._slices):
            self._kp[idx], self._kd[idx] = v._kp, v._kd
            self._tau_lo[idx], self._tau_hi[idx] = v._tau_lo, v._tau_hi
        self._act = np.concatenate([v._act for v in views]) if views else np.zeros(0, int)
        self._qadr = np.empty(n, dtype=int)
        self._vadr = np.empty(n, dtype=int)
        for v, idx in zip(views, self._slices):
            self._qadr[idx], self._vadr[idx] = v._qadr, v._vadr
        # Which part owns each body: a prefixed body by its longest matching
        # prefix; an unprefixed one by the tree root of the root's body; the
        # rest (scene objects) by none.
        names = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
        prefixed = sorted((v for v in views if v._prefix), key=lambda v: -len(v._prefix))
        plain = [v for v in views if not v._prefix]
        owner: Dict[int, str] = {}
        for b in range(1, m.nbody):
            hit = next((v for v in prefixed if names[b].startswith(v._prefix)), None)
            if hit is None:
                hit = next((v for v in plain if int(m.body_rootid[b]) == v._root), None)
            if hit is not None:
                owner[b] = hit._path
        self._owner = owner
        self._body_name = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b) or str(b)
                           for b in range(m.nbody)]
        self._free = [v for v in views if v._has_free]
        self._warn_ranges(layout.embodiment)
        #: End effectors whose measured pose differs from the model's (``ee_tolerance_m``).
        self.ee_mismatch = 0
        self.ee_error_m: Optional[float] = None
        self._ee_warned: set = set()

    def _warn_ranges(self, emb: Any) -> None:
        """Warn where a joint's ``position_limit`` and the MJCF's range differ by > 0.1 rad."""
        m = self.world.model
        for v, idx in zip(self.views, self._slices):
            for k, jid in enumerate(v._jid):
                joint = emb.joints[idx[k]]
                if joint.position_limit is None or not m.jnt_limited[jid]:
                    continue
                lo, hi = (float(x) for x in m.jnt_range[jid])
                if max(abs(lo - joint.position_limit[0]), abs(hi - joint.position_limit[1])) > 0.1:
                    logger.warning("safety.sim_check: joint %r: position_limit %s, MJCF range "
                                   "[%.3f, %.3f]", joint.name, list(joint.position_limit), lo, hi)

    def _contacts(self) -> Dict[tuple, float]:
        """Penetration depth (m) per pair of bodies in contact now."""
        m, d = self.world.model, self.world.data
        out: Dict[tuple, float] = {}
        for i in range(d.ncon):
            c = d.contact[i]
            pair = tuple(sorted((int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2]))))
            out[pair] = max(out.get(pair, 0.0), -float(c.dist))
        return out

    def _what(self, pair: tuple, check: SimCheck) -> Optional[str]:
        """The contact ``pair`` in words, or ``None`` when the check ignores it."""
        a, b = pair
        name = self._body_name
        oa, ob = self._owner.get(a), self._owner.get(b)
        if 0 in pair:
            return f"{name[b if a == 0 else a]} into the ground" if check.ground else None
        if oa is None and ob is None:
            return None  # two scene objects
        if oa is None or ob is None:
            robot, thing = (b, a) if oa is None else (a, b)
            return f"{name[robot]} hits {name[thing]} in the scene"
        if not check.self_collision:
            return None
        if oa == ob:
            return f"self-collision {name[a]} / {name[b]}"
        return f"collision between parts {oa} and {ob}: {name[a]} / {name[b]}"

    def _sync(self, obs: Any) -> None:
        """Reset the world to the measured state (joints, base orientation)."""
        w, n = self.world, len(self.names)
        m, d, mj = w.model, w.data, w.mj
        q0 = np.asarray(obs.joint_pos, dtype=float)
        if q0.shape != (n,) or not np.all(np.isfinite(q0)):
            raise SimCheckFailed("observation has no complete, finite joint_pos")
        dq0 = obs.joint_vel
        dq0 = (None if dq0 is None or np.shape(dq0) != (n,) or not np.all(np.isfinite(dq0))
               else np.asarray(dq0, dtype=float))
        mj.mj_resetData(m, d)
        d.qpos[self._qadr] = q0
        if dq0 is not None:
            d.qvel[self._vadr] = dq0
        quat = getattr(obs, "base_orientation", None)
        if quat is not None and self._floating and len(self._free) == 1:
            q = np.asarray(quat, dtype=float)
            norm = np.linalg.norm(q)
            if norm > 0:
                fq = self._free[0]._fq
                d.qpos[fq + 3:fq + 7] = q[[3, 0, 1, 2]] / norm  # MuJoCo [w, x, y, z]
        mj.mj_forward(m, d)

    def ee_errors(self, obs: Any) -> Dict[str, float]:
        """End effector -> distance (m) between its measured and model position.

        The world must be synced to ``obs`` first (:meth:`_sync`). Both are in
        the robot's frame; end effectors ``obs`` has no pose for are left out.
        """
        measured = getattr(obs, "ee_poses", None) or {}
        if not measured:
            return {}
        from hirobot.geometry.transforms import pose_compose, pose_inverse

        base = None
        if self._floating and len(self._free) == 1:
            v = self._free[0]
            base = pose_inverse(v._body_pose(v._root))
        out: Dict[str, float] = {}
        for v in self.views:
            for name, bid, offset in v._ee_frames:
                full = v._prefix + name
                if full not in measured:
                    continue
                pose = pose_compose(v._body_pose(bid), offset)
                if base is not None:
                    pose = pose_compose(base, pose)
                out[full] = float(np.linalg.norm(np.asarray(measured[full], float)[:3] - pose[:3]))
        return out

    def _compare_ee(self, obs: Any, check: SimCheck) -> None:
        """Count (and warn once) where the measured ``ee_poses`` differ from the model's."""
        errors = self.ee_errors(obs)
        if not errors:
            return
        for full, err in errors.items():
            if err > check.ee_tolerance_m:
                self.ee_mismatch += 1
                if full not in self._ee_warned:
                    self._ee_warned.add(full)
                    logger.warning("safety.sim_check: end effector %r is %.0f mm from the "
                                   "model's pose at the same joint angles (calibration or "
                                   "model off?)", full, err * 1e3)
            else:
                self._ee_warned.discard(full)
        self.ee_error_m = max(errors.values())

    def predict(self, action: Any, obs: Any, check: SimCheck) -> Optional[str]:
        n = len(self.names)
        self._sync(obs)
        self._compare_ee(obs, check)
        before = self._contacts()

        target = np.asarray(action.joint_target, dtype=float)
        if target.shape != (n,):
            raise SimCheckFailed(f"joint_target has {target.size} values, the robot {n} joints")
        kp = self._kp if action.kp is None else np.broadcast_to(np.asarray(action.kp, float), (n,))
        kd = self._kd if action.kd is None else np.broadcast_to(np.asarray(action.kd, float), (n,))
        z = np.zeros(n)
        mode = action.mode.value
        if mode in ("position", "pd"):
            q_ref, dq_ref, ff = target, z, z
        elif mode == "velocity":
            kp, q_ref, dq_ref, ff = z, z, target, z
        else:  # torque
            kp, kd, q_ref, dq_ref, ff = z, z, z, z, target
        limit = np.minimum(np.abs(self._tau_lo), np.abs(self._tau_hi)) * check.torque_ratio

        w = self.world
        m, d, mj = w.model, w.data, w.mj
        # The PD runs in the actuators as in the world's views (ctrl - kp q - kd dq,
        # SimWorld._as_motors), so an implicit integrator damps it; the torque
        # compared to the limit is the same PD law, before clamping.
        a = self._act
        m.actuator_biasprm[a, 1] = -kp
        m.actuator_biasprm[a, 2] = -kd
        d.ctrl[a] = kp * q_ref + kd * dq_ref + ff
        for step in range(check.horizon_steps * w.substeps):
            q, dq = d.qpos[self._qadr], d.qvel[self._vadr]
            tau = kp * (q_ref - q) + kd * (dq_ref - dq) + ff
            over = np.abs(tau) > limit
            if np.any(over):
                i = int(np.argmax(np.abs(tau) - limit))
                return (f"joint {self.names[i]!r} would need {abs(tau[i]):.1f} Nm, limit "
                        f"{limit[i]:.1f} Nm (torque_ratio {check.torque_ratio:g}) after "
                        f"{(step + 1) * m.opt.timestep * 1e3:.0f} ms")
            mj.mj_step(m, d)
            for pair, depth in self._contacts().items():
                if depth <= check.tolerance or depth <= before.get(pair, 0.0) + check.tolerance:
                    continue
                what = self._what(pair, check)
                if what is None:
                    continue
                return (f"{what} ({depth * 1e3:.0f} mm deep) after "
                        f"{(step + 1) * m.opt.timestep * 1e3:.0f} ms")
        return None


def _check_moving(layout: Any) -> None:
    """Refuse a tree with a part that moves by itself below the root (no base pose to sync)."""
    def walk(lay: Any, path: str) -> None:
        for p, part in lay.parts.items():
            if part.kind == "moving" and path:
                raise NotImplementedError(
                    f"{path}{p}: a part that moves by itself inside a tree is not in the "
                    "simulation check yet (its base pose would have to be synced)")
            if p in lay.children:
                walk(lay.children[p], f"{path}{p}/")
        for p, child in lay.children.items():
            if child.moving is not None:
                raise NotImplementedError(
                    f"{path}{p}/{child.moving}: a part that moves by itself inside a tree is not "
                    "in the simulation check yet (its base pose would have to be synced)")

    walk(layout, "")


__all__ = ["KEYS", "SimCheck", "SimCheckFailed", "holding"]
