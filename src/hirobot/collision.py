"""Distance checks between the members of a robot group.

Each member with a model is approximated by capsules: one segment from every body's
origin to its parent's (forward kinematics from :mod:`hirobot.kinematics`), all of one
radius. The check measures the smallest gap between capsules of *different* members;
collisions within one member are left to that member (its own controller or SDK).

:class:`CollisionCheck` is what :class:`~hirobot.group.RobotGroup` runs before sending.
It refuses an action whose predicted gap is below ``min_distance`` unless the gap is
not getting smaller, so robots that are already too close can still move apart.

Prediction per member:

* joint position targets (``POSITION`` / ``PD``) are used as the next joint angles;
* any other mode keeps the current angles, and for ``CARTESIAN`` the target points
  themselves (as spheres of the member's radius) are checked against the other members.

The approximation is coarse on purpose: capsules along the kinematic chain ignore the
link's real shape, so the radius has to cover it, and the check can refuse poses a real
collision checker would allow. It is a guard against arms running into each other, not
a planner.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from hirobot.kinematics import Kinematics
from hirobot.transforms import quat_to_matrix


def segment_distances(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> np.ndarray:
    """Smallest distance between every segment of ``a`` and every segment of ``b``.

    Args:
        a0, a1: ``(n, 3)`` segment end points.
        b0, b1: ``(m, 3)`` segment end points. A zero-length segment is a point.

    Returns:
        ``(n, m)`` distances.
    """
    d1 = (a1 - a0)[:, None, :]            # (n, 1, 3)
    d2 = (b1 - b0)[None, :, :]            # (1, m, 3)
    r = a0[:, None, :] - b0[None, :, :]   # (n, m, 3)
    a = np.sum(d1 * d1, axis=-1)          # (n, 1)
    e = np.sum(d2 * d2, axis=-1)          # (1, m)
    f = np.sum(d2 * r, axis=-1)           # (n, m)
    c = np.sum(d1 * r, axis=-1)
    b = np.sum(d1 * d2, axis=-1)
    eps = 1e-12
    denom = a * e - b * b
    # s on segment a: closest point of the infinite lines, clamped (parallel -> 0).
    s = np.where(denom > eps, np.clip((b * f - c * e) / np.where(denom > eps, denom, 1.0), 0, 1), 0.0)
    s = np.where(a > eps, s, 0.0)
    t = np.where(e > eps, (b * s + f) / np.where(e > eps, e, 1.0), 0.0)
    # t outside [0, 1]: clamp it and recompute s for that end of b.
    t_lo, t_hi = t < 0, t > 1
    t = np.clip(t, 0, 1)
    s_lo = np.where(a > eps, np.clip(-c / np.where(a > eps, a, 1.0), 0, 1), 0.0)
    s_hi = np.where(a > eps, np.clip((b - c) / np.where(a > eps, a, 1.0), 0, 1), 0.0)
    s = np.where(t_lo, s_lo, np.where(t_hi, s_hi, s))
    p = a0[:, None, :] + d1 * s[..., None]
    q = b0[None, :, :] + d2 * t[..., None]
    return np.linalg.norm(p - q, axis=-1)


@dataclass
class MemberModel:
    """A member's capsule model.

    Attributes:
        kinematics: Forward kinematics of the member's MJCF.
        joint_names: The member's joints that the model has (others do not move a capsule).
        columns: Index of each of ``joint_names`` in the member's joint vector.
        radius: Capsule radius (m).
        skip: Bodies whose segment to their parent is left out (e.g. a mounting plate).
    """

    kinematics: Kinematics
    joint_names: tuple[str, ...]
    columns: np.ndarray
    radius: float
    skip: tuple[str, ...] = ()
    _seg: tuple[np.ndarray, np.ndarray] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        kin = self.kinematics
        unknown = sorted(set(self.skip) - set(kin.body_names))
        if unknown:
            raise ValueError(f"collision skip: no bodies {unknown} in {kin.path}")
        child = [i for i, p in enumerate(kin.parents) if p >= 0 and kin.body_names[i] not in self.skip]
        if not child:
            # A one-body model: its origin as a point.
            child = [0]
            parent = [0]
        else:
            parent = [kin.parents[i] for i in child]
        self._seg = (np.array(parent, dtype=int), np.array(child, dtype=int))

    def segments(self, joint_pos: np.ndarray, world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Capsule end points in the world for one joint vector of the member.

        Args:
            joint_pos: The member's full joint vector.
            world: Pose ``[x, y, z, qx, qy, qz, qw]`` of the member's base in the world.
        """
        q = np.asarray(joint_pos, dtype=float)[self.columns]
        kin = self.kinematics
        if kin.floating_base:
            poses = kin.forward(q, base_pos=np.zeros(3), base_quat=np.array([0.0, 0.0, 0.0, 1.0]),
                                joint_names=self.joint_names)
        else:
            poses = kin.forward(q, joint_names=self.joint_names)
        r = quat_to_matrix(world[3:])
        pts = poses.pos @ r.T + world[:3]
        parent, child = self._seg
        return pts[parent], pts[child]


@dataclass
class Gap:
    """The smallest gap found by :meth:`CollisionCheck.gap`."""

    distance: float
    members: tuple[str, str]
    bodies: tuple[str, str]


class CollisionCheck:
    """Minimum distance between the members of a group.

    Args:
        models: Member name -> its :class:`MemberModel`. Members without one are
            not checked.
        min_distance: Smallest allowed gap between capsule surfaces (m).
    """

    def __init__(self, models: Mapping[str, MemberModel], min_distance: float) -> None:
        if min_distance < 0:
            raise ValueError("min_distance must be >= 0")
        if len(models) < 2:
            raise ValueError("a collision check needs models for at least two members; "
                             f"got {sorted(models)}")
        self.models: dict[str, MemberModel] = dict(models)
        self.min_distance = float(min_distance)

    @classmethod
    def build(cls, members: Mapping[str, Sequence[str]], config: Mapping[str, object]) -> CollisionCheck:
        """From a group config's ``collision:`` section.

        ``members`` is member name -> its joint names. ``config``::

            min_distance: 0.05          # m between capsule surfaces
            radius: 0.06                # default capsule radius (m)
            backend: numpy              # or mujoco (Kinematics.from_mjcf)
            models:
              left:  {mjcf: ~/models/fr3.xml}
              right: {mjcf: ~/models/ur5e.xml, radius: 0.05, skip: [base_mount]}
        """
        cfg = dict(config)
        bad = sorted(set(cfg) - {"min_distance", "radius", "backend", "models"})
        if bad:
            raise ValueError(f"collision: unknown key(s) {bad}")
        default_radius = float(cfg.get("radius", 0.05))
        backend = str(cfg.get("backend", "numpy"))
        models: dict[str, MemberModel] = {}
        for m, spec in dict(cfg.get("models") or {}).items():
            if m not in members:
                raise ValueError(f"collision.models: no member {m!r}; members: {list(members)}")
            spec = dict(spec or {})
            bad = sorted(set(spec) - {"mjcf", "radius", "skip", "backend"})
            if bad:
                raise ValueError(f"collision.models.{m}: unknown key(s) {bad}")
            if "mjcf" not in spec:
                raise ValueError(f"collision.models.{m} needs 'mjcf'")
            kin = Kinematics.from_mjcf(str(spec["mjcf"]), backend=str(spec.get("backend", backend)))
            models[m] = member_model(kin, members[m], radius=float(spec.get("radius", default_radius)),
                                     skip=tuple(spec.get("skip") or ()))
        return cls(models, min_distance=float(cfg.get("min_distance", 0.05)))

    def gap(self, segments: Mapping[str, tuple[np.ndarray, np.ndarray]],
            points: Mapping[str, np.ndarray] | None = None) -> Gap | None:
        """The smallest surface gap between any two members.

        Args:
            segments: Member -> capsule end points (``MemberModel.segments``).
            points: Member -> ``(k, 3)`` extra points (Cartesian targets), as
                spheres of that member's radius, checked against the *other*
                members' capsules.
        """
        best: Gap | None = None
        names = [m for m in self.models if m in segments]
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                best = self._closer(best, a, b, segments[a], segments[b])
        for a, pts in (points or {}).items():
            pts = np.asarray(pts, dtype=float).reshape(-1, 3)
            for b in names:
                if b != a and len(pts):
                    best = self._closer(best, a, b, (pts, pts), segments[b], a_points=True)
        return best

    def _closer(self, best, a, b, seg_a, seg_b, a_points: bool = False):
        d = segment_distances(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
        d = d - self.models[a].radius - self.models[b].radius
        i, j = np.unravel_index(int(np.argmin(d)), d.shape)
        if best is not None and d[i, j] >= best.distance:
            return best
        body_a = "target" if a_points else self._body(a, i)
        return Gap(float(d[i, j]), (a, b), (body_a, self._body(b, j)))

    def _body(self, m: str, k: int) -> str:
        return self.models[m].kinematics.body_names[self.models[m]._seg[1][k]]


def member_model(kin: Kinematics, joint_names: Sequence[str], radius: float,
                 skip: Sequence[str] = ()) -> MemberModel:
    """A :class:`MemberModel` for a member whose joints are ``joint_names``.

    Joints the model does not have (fingers of a hand not in the MJCF) are ignored;
    a model joint the member does not have stays at 0.
    """
    model = {j.removesuffix("_joint") for j in kin.joint_names}
    keep = [(i, n) for i, n in enumerate(joint_names) if n.removesuffix("_joint") in model]
    if not keep:
        raise ValueError(f"none of the member's joints are in {kin.path}")
    return MemberModel(kin, tuple(n for _, n in keep), np.array([i for i, _ in keep], dtype=int),
                       radius=radius, skip=tuple(skip))
