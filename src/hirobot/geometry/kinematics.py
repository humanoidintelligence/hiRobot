"""Forward kinematics of an MJCF model: body poses from joint angles and the base pose.

The default backend is plain numpy: it reads the body tree (body ``pos`` / ``quat``,
hinge joint ``axis`` / ``pos``) from the MJCF and composes the transforms, so it
needs no simulator. ``backend="mujoco"`` runs ``mujoco.mj_kinematics`` on the same
file instead (needs ``pip install 'hirobot[sim]'``); use it to cross-check the
numpy result or for models the numpy parser rejects.

Poses follow hiRobot's conventions: positions in metres, quaternions
``[x, y, z, w]``. Both backends accept a batch: ``joint_pos`` of shape ``(..., n)``
gives poses of shape ``(..., num_bodies, 3)`` / ``(..., num_bodies, 4)``.

The numpy parser covers what robot description files use: nested ``<body>`` with
``pos`` / ``quat``, hinge joints, and one ``free`` joint on a top-level body. It
raises ``ValueError`` for anything that would change the kinematics and that it
does not read (``euler`` / ``axisangle`` / ``xyaxes`` / ``zaxis`` orientations,
slide / ball joints, joint ``ref``, ``<default>`` joint classes, ``<include>``,
``<frame>``), rather than computing a wrong answer.

Example::

    kin = Kinematics.from_mjcf("g1_29dof_rev_1_0.xml")
    poses = kin.forward(q, base_pos=[0, 0, 0.8], base_quat=[0, 0, 0, 1],
                        joint_names=embodiment.joint_names)
    hand_pos, hand_quat = poses["left_wrist_yaw_link"]
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

_ORIENTATION_ATTRS = ("euler", "axisangle", "xyaxes", "zaxis")


@dataclass
class BodyPoses:
    """World poses of every body of a model.

    Attributes:
        names: Body names, in the model's order.
        pos: ``(..., num_bodies, 3)`` positions.
        quat: ``(..., num_bodies, 4)`` orientations ``[x, y, z, w]`` with ``w >= 0``.
    """

    names: tuple[str, ...]
    pos: np.ndarray
    quat: np.ndarray

    def index(self, name: str) -> int:
        try:
            return self.names.index(name)
        except ValueError:
            raise KeyError(f"no body {name!r}; bodies: {list(self.names)}") from None

    def __getitem__(self, name: str) -> tuple[np.ndarray, np.ndarray]:
        """``(pos, quat)`` of one body."""
        i = self.index(name)
        return self.pos[..., i, :], self.quat[..., i, :]


# ----- vectorized rotation helpers ---------------------------------------------------
def _quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = np.moveaxis(q, -1, 0)
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def _matrix_to_quat_xyzw(m: np.ndarray) -> np.ndarray:
    """Quaternions of rotation matrices ``(..., 3, 3)``, with ``w >= 0``."""
    m00, m11, m22 = m[..., 0, 0], m[..., 1, 1], m[..., 2, 2]
    # Each candidate is exact where its pivot is largest; pick that one per element.
    cands = np.stack([
        np.stack([m[..., 2, 1] - m[..., 1, 2], m[..., 0, 2] - m[..., 2, 0],
                  m[..., 1, 0] - m[..., 0, 1], 1 + m00 + m11 + m22], -1),
        np.stack([1 + m00 - m11 - m22, m[..., 0, 1] + m[..., 1, 0],
                  m[..., 0, 2] + m[..., 2, 0], m[..., 2, 1] - m[..., 1, 2]], -1),
        np.stack([m[..., 0, 1] + m[..., 1, 0], 1 + m11 - m00 - m22,
                  m[..., 1, 2] + m[..., 2, 1], m[..., 0, 2] - m[..., 2, 0]], -1),
        np.stack([m[..., 0, 2] + m[..., 2, 0], m[..., 1, 2] + m[..., 2, 1],
                  1 + m22 - m00 - m11, m[..., 1, 0] - m[..., 0, 1]], -1),
    ], 0)
    pivot = np.stack([m00 + m11 + m22, m00, m11, m22], 0)
    pick = np.argmax(pivot, axis=0)
    q = np.take_along_axis(cands, pick[None, ..., None], axis=0)[0]
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return np.where(q[..., 3:4] < 0, -q, q)


def _axis_rotation(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    """Rotation matrices ``(..., 3, 3)`` about a fixed unit ``axis`` by ``angle`` (...)."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    s, c = np.sin(angle)[..., None, None], np.cos(angle)[..., None, None]
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _strip(name: str) -> str:
    return name.removesuffix("_joint")


# ----- model ---------------------------------------------------------------------------
@dataclass
class _Joint:
    name: str
    axis: np.ndarray
    anchor: np.ndarray


@dataclass
class _Body:
    name: str
    parent: int  # -1 = world
    pos: np.ndarray
    rot: np.ndarray  # 3x3
    joints: list[_Joint]


class Kinematics:
    """Forward kinematics of one MJCF model.

    Build it with :meth:`from_mjcf`. Attributes:
        body_names: Bodies in the model's order (the world body excluded).
        parents: Parent index per body (-1 for top-level bodies).
        joint_names: Hinge joints in the model's order, as named in the MJCF.
        floating_base: True when a top-level body has a free joint; ``forward``
            then places it at ``base_pos`` / ``base_quat``.
    """

    backend = "numpy"

    def __init__(self, bodies: list[_Body], free_body: int | None, path: str) -> None:
        self._bodies = bodies
        self._free = free_body
        self.path = path
        self.body_names: tuple[str, ...] = tuple(b.name for b in bodies)
        self.parents: tuple[int, ...] = tuple(b.parent for b in bodies)
        self.joint_names: tuple[str, ...] = tuple(j.name for b in bodies for j in b.joints)
        self.floating_base = free_body is not None

    @classmethod
    def from_mjcf(cls, path: str, backend: str = "numpy") -> Kinematics:
        """Load ``path`` (``~`` expanded). ``backend``: ``"numpy"`` (default) or ``"mujoco"``."""
        path = os.path.expanduser(str(path))
        if not os.path.isfile(path):
            raise FileNotFoundError(f"MJCF model not found: {path}")
        if backend == "mujoco":
            return _MujocoKinematics(path)
        if backend != "numpy":
            raise ValueError(f"unknown kinematics backend {backend!r}; use 'numpy' or 'mujoco'")
        return cls._parse(path)

    # ----- parsing ---------------------------------------------------------------------
    @classmethod
    def _parse(cls, path: str) -> Kinematics:
        root = ET.parse(path).getroot()
        if root.find(".//include") is not None:
            raise ValueError(f"{path}: <include> is not supported by the numpy backend; "
                             "use backend='mujoco'")
        for d in root.iter("default"):
            for j in d.findall("joint"):
                if any(k in j.attrib for k in ("axis", "pos", "type", "ref")):
                    raise ValueError(f"{path}: <default> sets joint attributes; "
                                     "use backend='mujoco'")
        world = root.find("worldbody")
        if world is None:
            raise ValueError(f"{path}: no <worldbody>")
        if world.find(".//frame") is not None:
            raise ValueError(f"{path}: <frame> is not supported by the numpy backend; "
                             "use backend='mujoco'")

        bodies: list[_Body] = []
        free: list[int] = []

        def vec(el, key, default):
            return np.array([float(x) for x in el.attrib.get(key, default).split()])

        def add(el, parent: int) -> None:
            name = el.attrib.get("name", f"body{len(bodies)}")
            for key in _ORIENTATION_ATTRS:
                if key in el.attrib:
                    raise ValueError(f"{path}: body {name!r} uses {key!r}; the numpy backend "
                                     "reads only quat (use backend='mujoco')")
            w, x, y, z = vec(el, "quat", "1 0 0 0")
            body = _Body(name, parent, vec(el, "pos", "0 0 0"),
                         _quat_xyzw_to_matrix(np.array([x, y, z, w])), [])
            index = len(bodies)
            bodies.append(body)
            for j in el:
                if j.tag == "freejoint" or (j.tag == "joint" and j.attrib.get("type") == "free"):
                    if parent != -1:
                        raise ValueError(f"{path}: free joint on non-root body {name!r}")
                    free.append(index)
                elif j.tag == "joint":
                    jname = j.attrib.get("name", f"{name}_joint{len(body.joints)}")
                    jtype = j.attrib.get("type", "hinge")
                    if jtype != "hinge":
                        raise ValueError(f"{path}: joint {jname!r} is {jtype!r}; the numpy "
                                         "backend reads hinge joints only")
                    if float(j.attrib.get("ref", "0")) != 0.0:
                        raise ValueError(f"{path}: joint {jname!r} sets ref; not supported")
                    axis = vec(j, "axis", "0 0 1")
                    body.joints.append(_Joint(jname, axis / np.linalg.norm(axis),
                                              vec(j, "pos", "0 0 0")))
            for child in el.findall("body"):
                add(child, index)

        for top in world.findall("body"):
            add(top, -1)
        if len(free) > 1:
            raise ValueError(f"{path}: more than one free joint")
        return cls(bodies, free[0] if free else None, path)

    # ----- forward ---------------------------------------------------------------------
    def _columns(self, joint_names: Sequence[str] | None, n: int) -> list[int | None]:
        """Per model joint, the column of ``joint_pos`` holding it (None = 0 rad)."""
        if joint_names is None:
            if n != len(self.joint_names):
                raise ValueError(f"joint_pos has {n} columns, the model has "
                                 f"{len(self.joint_names)} joints; pass joint_names")
            return list(range(n))
        names = [_strip(j) for j in joint_names]
        if len(names) != n:
            raise ValueError(f"joint_names has {len(names)} names, joint_pos has {n} columns")
        col = {name: i for i, name in enumerate(names)}
        unknown = set(names) - {_strip(j) for j in self.joint_names}
        if unknown:
            raise ValueError(f"joints not in {self.path}: {sorted(unknown)}")
        return [col.get(_strip(j)) for j in self.joint_names]

    def _base(self, base_pos, base_quat, batch: tuple[int, ...]):
        if not self.floating_base:
            if base_pos is not None or base_quat is not None:
                raise ValueError(f"{self.path} has no free joint; base_pos / base_quat "
                                 "do not apply")
            return None, None
        top = self._bodies[self._free]
        pos = top.pos if base_pos is None else np.asarray(base_pos, dtype=float)
        if base_quat is None:
            rot = top.rot
        else:
            rot = _quat_xyzw_to_matrix(np.asarray(base_quat, dtype=float))
        return (np.broadcast_to(pos, batch + (3,)), np.broadcast_to(rot, batch + (3, 3)))

    def forward(self, joint_pos, base_pos=None, base_quat=None,
                joint_names: Sequence[str] | None = None) -> BodyPoses:
        """World poses of all bodies.

        Args:
            joint_pos: ``(..., n)`` joint angles (rad). Columns follow
                ``joint_names`` if given (``name`` or ``name_joint``; model joints not
                listed are at 0), else the model's :attr:`joint_names`.
            base_pos: ``(..., 3)`` world position of the free-joint body; defaults
                to its MJCF ``pos``. Floating-base models only.
            base_quat: ``(..., 4)`` its orientation ``[x, y, z, w]``; defaults to
                its MJCF ``quat``.
            joint_names: Names of ``joint_pos``'s columns.
        """
        q = np.asarray(joint_pos, dtype=float)
        if q.ndim == 0:
            raise ValueError("joint_pos must have at least one axis")
        batch = q.shape[:-1]
        cols = self._columns(joint_names, q.shape[-1])
        base_p, base_r = self._base(base_pos, base_quat, batch)
        pos = np.empty(batch + (len(self._bodies), 3))
        rot = np.empty(batch + (len(self._bodies), 3, 3))
        k = 0
        for i, b in enumerate(self._bodies):
            if i == self._free:
                p, r = base_p, base_r
            elif b.parent == -1:
                p, r = np.broadcast_to(b.pos, batch + (3,)), np.broadcast_to(b.rot, batch + (3, 3))
            else:
                rp = rot[..., b.parent, :, :]
                p = pos[..., b.parent, :] + rp @ b.pos
                r = rp @ b.rot
            for j in b.joints:
                c = cols[k]
                k += 1
                if c is None:
                    continue
                rj = _axis_rotation(j.axis, q[..., c])
                r_new = r @ rj
                if np.any(j.anchor):
                    # Rotate about the joint anchor, not the body origin.
                    p = p + r @ j.anchor - r_new @ j.anchor
                r = r_new
            pos[..., i, :] = p
            rot[..., i, :, :] = r
        return BodyPoses(self.body_names, pos, _matrix_to_quat_xyzw(rot))


class _MujocoKinematics(Kinematics):
    """``mujoco.mj_kinematics`` on the same MJCF; same interface as :class:`Kinematics`."""

    backend = "mujoco"

    def __init__(self, path: str) -> None:
        try:
            import mujoco
        except ImportError as e:
            raise ImportError("backend='mujoco' needs mujoco: pip install 'hirobot[sim]'") from e
        self._mj = mujoco
        self.path = path
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.body_names = tuple(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
                                for i in range(1, m.nbody))
        self.parents = tuple(int(m.body_parentid[i]) - 1 for i in range(1, m.nbody))
        free = [i for i in range(m.njnt) if int(m.jnt_type[i]) == int(mujoco.mjtJoint.mjJNT_FREE)]
        if len(free) > 1:
            raise ValueError(f"{path}: more than one free joint")
        self._free_adr = int(m.jnt_qposadr[free[0]]) if free else None
        self.floating_base = bool(free)
        one_dof = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        joints = [i for i in range(m.njnt) if int(m.jnt_type[i]) in one_dof]
        self.joint_names = tuple(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
                                 for i in joints)
        self._qadr = [int(m.jnt_qposadr[i]) for i in joints]

    def forward(self, joint_pos, base_pos=None, base_quat=None,
                joint_names: Sequence[str] | None = None) -> BodyPoses:
        mj, m, d = self._mj, self.model, self.data
        q = np.asarray(joint_pos, dtype=float)
        if q.ndim == 0:
            raise ValueError("joint_pos must have at least one axis")
        batch = q.shape[:-1]
        cols = self._columns(joint_names, q.shape[-1])
        if not self.floating_base and (base_pos is not None or base_quat is not None):
            raise ValueError(f"{self.path} has no free joint; base_pos / base_quat do not apply")
        flat_q = q.reshape(-1, q.shape[-1])
        n = len(flat_q)
        bp = None if base_pos is None else np.broadcast_to(
            np.asarray(base_pos, dtype=float), batch + (3,)).reshape(n, 3)
        bq = None if base_quat is None else np.broadcast_to(
            np.asarray(base_quat, dtype=float), batch + (4,)).reshape(n, 4)
        pos = np.empty((n, m.nbody - 1, 3))
        quat = np.empty((n, m.nbody - 1, 4))
        for t in range(n):
            d.qpos[:] = m.qpos0
            for adr, c in zip(self._qadr, cols):
                if c is not None:
                    d.qpos[adr] = flat_q[t, c]
            if self._free_adr is not None:
                a = self._free_adr
                if bp is not None:
                    d.qpos[a:a + 3] = bp[t]
                if bq is not None:
                    x, y, z, w = bq[t] / np.linalg.norm(bq[t])
                    d.qpos[a + 3:a + 7] = [w, x, y, z]
            mj.mj_kinematics(m, d)
            pos[t] = d.xpos[1:]
            wq = d.xquat[1:]
            quat[t] = np.concatenate([wq[:, 1:], wq[:, :1]], axis=1)
        quat = np.where(quat[..., 3:4] < 0, -quat, quat)
        nb = m.nbody - 1
        return BodyPoses(self.body_names, pos.reshape(batch + (nb, 3)),
                         quat.reshape(batch + (nb, 4)))
