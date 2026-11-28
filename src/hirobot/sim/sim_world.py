"""One MuJoCo world for a whole robot tree (the ``sim:`` section).

:class:`SimWorld` puts every simulated body of a robot into one MuJoCo model:
each body's ``model.mjcf`` attached at its resolved placement, its names
prefixed with its node's path (``left/shoulder_pan``), so the names in the
model are the robot's. ``sim.scene`` (a table, objects) and every camera with a
mount are added too. Each body is driven by a
:class:`~hirobot.drivers.sim.mujoco.MujocoRobot` *view* of the world, which
reads and writes only its own joints; a camera by a :class:`WorldCamera`.

**Time.** The world steps one control period (the root's ``control_hz``) when
every view has had its command for the period: the root's ``send_action``
sends to every part (or idles it), so each command steps the world once and
every part shares one clock (``now()`` is ``data.time``). A view's ``reset``
ramp steps the world itself; a hold only sets the command.

**Cameras.** A camera renders a new frame when simulated time reaches its
next frame time, every ``1 / fps`` (:class:`FrameClock`): its ``Camera.fps``,
else ``sim.camera_fps`` (:data:`SIM_CAMERA_FPS`, 30); ``camera_fps: null``
renders on every observation. In between an observation gets the same image
object with the same stamp, as from a real camera. The first observation after
connect, a ``reset`` or a ``set_state`` renders.

**Actuators.** Each joint is driven by a torque (the view's PD). An actuator
the MJCF declares as a position or velocity servo (MuJoCo Menagerie arms) is
turned into a motor with the same force range. A driven joint no actuator
drives (a model without actuators, a URDF) gets such a motor.

An actuator no view drives (a wrist roll the embodiment does not list, a
zero-torque gripper) holds the pose it had at connect: a PD of
:data:`HOLD_KP` / :data:`HOLD_KD`, set once when the world connects.

**Stage.** A world whose models bring no floor, light or sky gets a default one
(:mod:`hirobot.sim.stage`), so cameras see something; the floor is drawn only.

:func:`build_world` reads a loaded tree and its ``sim:`` section;
``make_robot`` calls it (:mod:`hirobot.loading.build`) when ``sim`` is on.
"""

from __future__ import annotations

import atexit
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from hirobot.core.stamps import Stamp
from hirobot.geometry.transforms import IDENTITY_POSE, pose_compose, quat_multiply
from hirobot.sim.stage import add_default_stage

if TYPE_CHECKING:
    from hirobot.core.embodiment import Camera

logger = logging.getLogger(__name__)

#: Why offscreen rendering failed in this process, or ``None`` (see :meth:`SimWorld._open_renderers`).
_GL_ERROR: Optional[BaseException] = None
_VIEWER_HZ = 60.0
#: Half a turn about x, ``[x, y, z, w]``: OpenCV optical frame -> MuJoCo camera frame.
_OPTICAL_TO_MUJOCO = np.array([1.0, 0.0, 0.0, 0.0])
#: Link name of the world frame (MuJoCo's world body).
WORLD = "world"
#: Keys of the ``sim:`` section.
SIM_KEYS = ("backend", "scene", "timestep", "realtime", "viewer", "fixed_base", "ramp_seconds",
            "damping_kd", "start_from", "camera_fps")
#: Frame rate of a simulated camera whose ``Camera.fps`` is not set (``sim.camera_fps``,
#: the ``mujoco`` driver's ``camera_fps``); ``None`` there renders every observation.
SIM_CAMERA_FPS = 30.0


def camera_fps_option(value: Any, where: str = "camera_fps") -> Optional[float]:
    """``value`` checked as the ``camera_fps`` option: a positive number, or ``None``."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0 \
            or not math.isfinite(value):
        raise ValueError(f"{where} must be a positive number or null (every observation), "
                         f"got {value!r}")
    return float(value)


class FrameClock:
    """When a simulated camera renders: every ``1 / fps`` of simulated time.

    :meth:`frame` renders when the first frame is due, when simulated time
    reaches the next frame time (on a grid of ``1 / fps`` from the first
    frame), or when time went back; otherwise it returns the last frame, the
    same image object and stamp. :meth:`restart` makes the next call render
    and starts a new grid (after a reset or a jump of the state).

    Args:
        fps: Frames per second of simulated time; ``None`` renders on every call.
    """

    def __init__(self, fps: Optional[float]) -> None:
        self.period = None if fps is None else 1.0 / float(fps)
        #: Frames rendered (the ``seq`` of the last frame's stamp).
        self.seq = 0
        self._frame: Optional[Tuple[np.ndarray, Stamp]] = None
        self._t0: Optional[float] = None
        self._next = 0.0

    def restart(self) -> None:
        """Render on the next :meth:`frame`; frame numbers go on."""
        self._t0 = None

    def frame(self, t: float, render: Callable[[], Optional[np.ndarray]]
              ) -> Optional[Tuple[np.ndarray, Stamp]]:
        """The frame at simulated time ``t``: ``render()``'s image if one is due, else the last."""
        last = self._frame
        if last is not None and self._t0 is not None and self.period is not None \
                and last[1].t <= t < self._next - 1e-9:
            return last
        image = render()
        if image is None:
            return None
        self.seq += 1
        self._frame = (image, Stamp(float(t), self.seq))
        if self.period is not None:
            if self._t0 is None or t < self._t0:
                self._t0 = float(t)
            k = math.floor((t - self._t0) / self.period + 1e-6) + 1
            self._next = self._t0 + k * self.period
        return self._frame


@dataclass
class WorldBody:
    """One model file attached to the world.

    Attributes:
        path: Part path of the device (``left/body``), for messages.
        prefix: Prefix of its names in the world (``left/``; ``""`` for the root's body).
        mjcf: The model file (``~`` expanded): the body's ``model.mjcf``, else its
            ``model.urdf`` (MuJoCo compiles a URDF too; it has no actuators, the world
            adds a motor per driven joint).
        anchor: World body it is attached to (``world``, ``cell/torso_link``).
        pose: Offset from ``anchor``, ``[x, y, z, qx, qy, qz, qw]``.
        fixed_base: Remove its free joint (welded in place).
        height: Height of its root body (m) with ``fixed_base``; ``None`` keeps the model's.
    """

    path: str
    prefix: str
    mjcf: str
    anchor: str = WORLD
    pose: np.ndarray = field(default_factory=lambda: IDENTITY_POSE.copy())
    fixed_base: bool = False
    height: Optional[float] = None
    joint_names: Dict[str, str] = field(default_factory=dict)


def rename_joints(mj: Any, spec: Any, names: Mapping[str, str], where: str) -> None:
    """Rename joints of ``spec`` (model name -> new name) before it compiles.

    Actuators and equalities name their joint, so the ones that named an old
    name are pointed at the new one. ``ValueError`` names a joint the model
    lacks or a new name it already has.
    """
    if not names:
        return
    have = {j.name for j in spec.joints}
    missing = sorted(set(names) - have)
    if missing:
        raise ValueError(f"model.joint_names renames {missing}, which are not joints of {where}")
    taken = sorted(set(names.values()) & (have - set(names)))
    if taken:
        raise ValueError(f"model.joint_names renames to {taken}, which {where} already has")
    for j in spec.joints:
        if j.name in names:
            j.name = names[j.name]
    for a in spec.actuators:
        if a.trntype == mj.mjtTrn.mjTRN_JOINT and a.target in names:
            a.target = names[a.target]
    for e in spec.equalities:
        if e.type == mj.mjtEq.mjEQ_JOINT:
            if e.name1 in names:
                e.name1 = names[e.name1]
            if e.name2 in names:
                e.name2 = names[e.name2]


def exclude_hand_contacts(mj: Any, spec: Any, hands: Sequence[Sequence[str]]) -> None:
    """No contacts between the links of one mapped hand (before ``spec`` compiles).

    ``hands`` lists, per end-effector with a ``joint_map``, the model joints the
    map drives. Its links are the bodies those joints move plus the body they
    hang from (the palm). The map moves coupled joints along a fixed curve, as
    the hand's linkage does, but the collision meshes of h1_with_hand.xml overlap
    along it: the palm stops the thumb base short of open, the closed thumb stops
    the closed index finger. Contacts with anything else are kept.
    """
    for joints in hands:
        links = []
        for name in joints:
            j = spec.joint(name)
            if j is None:
                continue  # reported when the joints are mapped
            for b in (j.parent, j.parent.parent):
                if b is not None and b.name and b.name not in links:
                    links.append(b.name)
        links = [b for b in links if b != "world"]
        for i, a in enumerate(links):
            for b in links[i + 1:]:
                spec.add_exclude(bodyname1=a, bodyname2=b)


def _wxyz(q: Sequence[float]) -> List[float]:
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def _xyzw(q: Sequence[float]) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]], dtype=float)


#: Gains (Nm/rad, Nm s/rad) that hold a joint no view drives at its pose at connect:
#: stiff enough for H1's wrist roll (left/right_hand_joint, about 0.5 kg beyond it)
#: to stay within 0.05 rad, soft enough for a light gripper finger at a 2 ms step.
HOLD_KP = 20.0
HOLD_KD = 1.0

#: Armature (kg m^2) a joint without one gets when the world adds its motor (a URDF, an
#: MJCF without actuators): a rotor's reflected inertia of this order keeps light links
#: stable under the PD. A body's ``model.armature`` replaces it when the view binds.
MOTOR_ARMATURE = 0.01


def _import_mujoco() -> Any:
    from hirobot.loading.sdk import require_sdk

    require_sdk("hirobot.drivers.sim.mujoco", "SimWorld")
    import mujoco

    return mujoco


def _copy_option(src: Any, dst: Any) -> None:
    """``dst.option = src.option`` field by field (no attach conflict warnings)."""
    for key in dir(src.option):
        if key.startswith("_"):
            continue
        try:
            setattr(dst.option, key, getattr(src.option, key))
        except (AttributeError, TypeError):
            pass  # read-only fields


class SimWorld:
    """One MuJoCo model and its data, shared by the views of the bodies in it.

    Args:
        bodies: The MJCFs to attach, first one's options (timestep, solver)
            used unless ``scene`` is given.
        cameras: Cameras with a ``mount`` (link names in world names, ``world``
            for the world frame) to add and render.
        scene: An MJCF to start the world from (floor, table, objects).
        timestep: Physics timestep (s); ``None`` keeps the model's.
        control_hz: Steps of :meth:`step` per second (the root's loop rate).
        viewer: Open the MuJoCo viewer on :meth:`connect`.
        realtime: The views' ``reset`` ramps sleep to play at wall-clock speed.
        camera_fps: Frame rate of a camera without ``fps`` of its own (simulated
            time); ``None`` renders it on every observation.
    """

    def __init__(self, bodies: Sequence[WorldBody], cameras: Sequence["Camera"] = (),
                 scene: Optional[str] = None, timestep: Optional[float] = None,
                 control_hz: float = 50.0, viewer: bool = False, realtime: bool = True,
                 camera_fps: Optional[float] = SIM_CAMERA_FPS) -> None:
        if not bodies:
            raise ValueError("a simulation world needs at least one body with a model")
        self.bodies = list(bodies)
        self.cameras = [c for c in cameras if c.mount is not None]
        self.scene = None if scene is None else os.path.expanduser(str(scene))
        self.timestep = None if timestep is None else float(timestep)
        self.period = 1.0 / float(control_hz) if control_hz else 0.02
        self.use_viewer = bool(viewer)
        self.realtime = bool(realtime)
        self.mj: Any = None
        self.model: Any = None
        self.data: Any = None
        #: Control periods stepped since connect (the ``seq`` of every stamp).
        self.ticks = 0
        self.substeps = 1
        self.lock: Any = threading.RLock()
        self._views: List[Any] = []
        self._pending: set = set()
        self._users = 0
        self._renderers: Dict[str, Any] = {}
        self.camera_fps = camera_fps_option(camera_fps)
        self._clocks: Dict[str, FrameClock] = {}
        self._viewer: Any = None
        self._last_sync = 0.0

    # ----------------------------------------------------------------- views

    def add_view(self, view: Any) -> None:
        """Register a body's view; it is bound to the model on :meth:`connect`."""
        if self.model is not None:
            raise RuntimeError("add every view before the world connects")
        self._views.append(view)

    @property
    def views(self) -> List[Any]:
        return list(self._views)

    # ----------------------------------------------------------------- build

    def _build(self) -> Any:
        mj = self.mj
        for b in self.bodies:
            if not os.path.isfile(b.mjcf):
                from hirobot.assets import fetch_hint

                raise FileNotFoundError(
                    f"MJCF model of {b.path} not found: {b.mjcf}. {fetch_hint(b.mjcf)}")
        alone = self.scene is None and len(self.bodies) == 1 and not self.bodies[0].prefix \
            and self.bodies[0].anchor == WORLD and np.array_equal(self.bodies[0].pose,
                                                                   IDENTITY_POSE)
        if alone:
            # One body at the origin: its MJCF is the world (exactly the single-robot model).
            spec = mj.MjSpec.from_file(self.bodies[0].mjcf)
            rename_joints(mj, spec, self.bodies[0].joint_names, self.bodies[0].mjcf)
            self._weld(spec, self.bodies[0])
        else:
            spec = mj.MjSpec.from_file(self.scene) if self.scene else mj.MjSpec()
            for i, b in enumerate(self.bodies):
                child = mj.MjSpec.from_file(b.mjcf)
                rename_joints(mj, child, b.joint_names, b.mjcf)
                if i == 0 and self.scene is None:
                    _copy_option(child, spec)
                else:
                    _copy_option(spec, child)
                self._weld(child, b)
                anchor = spec.worldbody if b.anchor == WORLD else spec.body(b.anchor)
                if anchor is None:
                    raise ValueError(f"{b.path}: it is placed on {b.anchor!r}, which is not a "
                                     "body of the simulation world")
                frame = anchor.add_frame(pos=[float(x) for x in b.pose[:3]],
                                         quat=_wxyz(b.pose[3:]))
                spec.attach(child, prefix=b.prefix, frame=frame)
        if self.timestep is not None:
            spec.option.timestep = self.timestep
        add_default_stage(mj, spec)
        exclude_hand_contacts(mj, spec, [h for v in self._views for h in v._mapped_hands()])
        self._as_motors(spec)
        self._add_cameras(spec)
        return spec.compile()

    def _as_motors(self, spec: Any) -> None:
        """Make every joint actuator a PD the views set: ``kp * ctrl - kp * q - kd * dq``.

        A view writes ``ctrl = kp q* + kd dq* + tau_ff`` and the gains into the
        bias terms (:meth:`~hirobot.drivers.sim.mujoco.MujocoView._apply_torque`),
        so the actuator force is the PD torque. Keeping the damping in the
        actuator, instead of computing the torque outside, lets an implicit
        integrator (``implicitfast``, as MuJoCo Menagerie arms use) integrate it
        implicitly: light wrist links stay stable at stiff gains. The force
        range stays; the control range is dropped.
        """
        mj = self.mj
        for a in spec.actuators:
            if a.trntype != mj.mjtTrn.mjTRN_JOINT:
                continue
            force = [float(x) for x in a.forcerange]
            has_force = a.forcelimited != mj.mjtLimited.mjLIMITED_FALSE and force[0] < force[1]
            ctrl = [float(x) for x in a.ctrlrange]
            if not has_force and a.biastype == mj.mjtBias.mjBIAS_NONE \
                    and a.ctrllimited != mj.mjtLimited.mjLIMITED_FALSE and ctrl[0] < ctrl[1]:
                # A motor with a gain: its ctrlrange times the gain bounded the torque.
                lo, hi = (x * float(a.gainprm[0]) for x in ctrl)
                force, has_force = [min(lo, hi), max(lo, hi)], True
            joint = spec.joint(a.target)
            if not has_force and joint is not None \
                    and joint.actfrclimited != mj.mjtLimited.mjLIMITED_FALSE \
                    and joint.actfrcrange[0] < joint.actfrcrange[1]:
                # The joint's actuatorfrcrange (the G1 MJCF) bounds the torque; on the
                # actuator it also bounds the reported actuator_force.
                force, has_force = [float(x) for x in joint.actfrcrange], True
            a.set_to_motor()
            a.biastype = mj.mjtBias.mjBIAS_AFFINE
            a.forcelimited = mj.mjtLimited.mjLIMITED_TRUE if has_force else mj.mjtLimited.mjLIMITED_FALSE
            if has_force:
                a.forcerange = force
            a.ctrllimited = mj.mjtLimited.mjLIMITED_FALSE
        self._add_missing_motors(spec)

    def _add_missing_motors(self, spec: Any) -> None:
        """Give each joint a view drives, but no actuator drives, a PD motor like the others.

        Some models carry no actuators (Fourier's GR2 MJCF, every URDF): the joints are
        there, nothing moves them. The added motor is the PD of :meth:`_as_motors`
        (``biastype`` affine, no control range), bounded by the joint's
        ``actuatorfrcrange`` when it has one (a URDF joint's ``effort`` limit).
        """
        mj = self.mj
        driven = {a.target for a in spec.actuators if a.trntype == mj.mjtTrn.mjTRN_JOINT}
        added = False
        for view in self._views:
            for cands, _, _ in (t for ts in view._joint_targets() for t in ts):
                joint = next((spec.joint(c) for c in cands if spec.joint(c) is not None), None)
                if joint is None or joint.name in driven:
                    continue  # a missing joint is reported when the view binds
                a = spec.add_actuator()
                a.name = f"hirobot/motor/{joint.name}"
                a.trntype = mj.mjtTrn.mjTRN_JOINT
                a.target = joint.name
                a.set_to_motor()
                a.biastype = mj.mjtBias.mjBIAS_AFFINE
                a.ctrllimited = mj.mjtLimited.mjLIMITED_FALSE
                limited = joint.actfrclimited != mj.mjtLimited.mjLIMITED_FALSE \
                    and joint.actfrcrange[0] < joint.actfrcrange[1]
                a.forcelimited = mj.mjtLimited.mjLIMITED_TRUE if limited \
                    else mj.mjtLimited.mjLIMITED_FALSE
                if limited:
                    a.forcerange = [float(x) for x in joint.actfrcrange]
                if float(joint.armature) == 0.0:
                    joint.armature = MOTOR_ARMATURE
                driven.add(joint.name)
                added = True
        if added:
            # A model without actuators sets no integrator for them either: light links
            # (a GR1 wrist yaw of 3e-4 kg m^2) under a stiff PD go unstable with explicit
            # Euler. Integrate the PD's damping implicitly, as MuJoCo Menagerie models do.
            spec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST

    def _weld(self, spec: Any, body: WorldBody) -> None:
        mj = self.mj
        free = [j for j in spec.joints if j.type == mj.mjtJoint.mjJNT_FREE]
        root = free[0].parent if free else None
        if body.fixed_base:
            for j in free:
                spec.delete(j)
            # Welded, the root body is part of the world body, and MuJoCo does not
            # filter parent-child contacts against the world: collision meshes that
            # overlap at a joint (H1's pelvis and torso_link at waist_yaw) then lock
            # it. Exclude them as the free-floating model did.
            if root is not None:
                for child in root.bodies:
                    spec.add_exclude(bodyname1=root.name, bodyname2=child.name)
        if body.height is not None and root is not None:
            root.pos = [root.pos[0], root.pos[1], float(body.height)]

    def _add_cameras(self, spec: Any) -> None:
        if not self.cameras:
            return
        for cam in self.cameras:
            mount = cam.mount
            host = spec.worldbody if mount.body == WORLD else spec.body(mount.body)
            if host is None:
                raise ValueError(f"camera {cam.name!r}: mount.body {mount.body!r} is not a body "
                                 "of the simulation world")
            # mount.quat is the OpenCV optical frame (z forward, y down); a MuJoCo
            # camera looks along its -z with y up: the optical frame turned half a
            # turn about x.
            q = quat_multiply(np.asarray(mount.quat, dtype=float), _OPTICAL_TO_MUJOCO)
            host.add_camera(name=f"hirobot/{cam.name}", pos=list(mount.pos), quat=_wxyz(q),
                            fovy=float(mount.fovy))
        g = spec.visual.global_
        g.offwidth = max(int(g.offwidth), *(c.width for c in self.cameras))
        g.offheight = max(int(g.offheight), *(c.height for c in self.cameras))

    # ------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        """Build the model (on the first view's ``connect``) and bind every view to it."""
        self._users += 1
        if self.model is not None:
            return
        self.mj = _import_mujoco()
        self.model = self._build()
        self.data = self.mj.MjData(self.model)
        self.substeps = max(1, round(self.period / self.model.opt.timestep))
        self.ticks = 0
        for view in self._views:
            view._bind(self)
        self.mj.mj_forward(self.model, self.data)
        self._hold_unclaimed()
        for view in self._views:
            view._hold_here()
        self._clocks = {c.name: FrameClock(c.fps if c.fps else self.camera_fps)
                        for c in self.cameras}
        self._open_renderers()
        if self.use_viewer:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.lock = self._viewer.lock()
            atexit.register(self._close_viewer)  # scripts that never disconnect()
        self.render(force=True)

    def _hold_unclaimed(self) -> None:
        """Hold each joint actuator no view drives at its pose now (HOLD_KP / HOLD_KD).

        Without it such an actuator (H1's wrist roll, which the embodiment does
        not list; a gripper the arm's view leaves out) gives no torque and its
        link swings. Written once: no view touches these actuators later.
        """
        m, d, mj = self.model, self.data, self.mj
        claimed = {int(a) for v in self._views for a in v._mact}
        for a in range(m.nu):
            if a in claimed or int(m.actuator_trntype[a]) != int(mj.mjtTrn.mjTRN_JOINT):
                continue
            jid = int(m.actuator_trnid[a, 0])
            # int(): a numpy int is never `in` a tuple of mujoco enums.
            if int(m.jnt_type[jid]) not in (int(mj.mjtJoint.mjJNT_HINGE),
                                            int(mj.mjtJoint.mjJNT_SLIDE)):
                continue
            # _as_motors made it ctrl - kp q - kd dq (gain 1); ctrl = kp q0 holds q0.
            m.actuator_gainprm[a, 0] = 1.0
            m.actuator_biasprm[a, 0] = 0.0
            m.actuator_biasprm[a, 1] = -HOLD_KP
            m.actuator_biasprm[a, 2] = -HOLD_KD
            d.ctrl[a] = HOLD_KP * float(d.qpos[m.jnt_qposadr[jid]])

    def disconnect(self) -> None:
        """Close renderers and the viewer once the last view disconnects."""
        self._users = max(0, self._users - 1)
        if self._users:
            return
        self._close_renderers()
        self._close_viewer()

    # -------------------------------------------------------------- stepping

    def ready(self, view: Any) -> None:
        """``view`` has its command for this period; step once every view has."""
        self._pending.add(id(view))
        if len(self._pending) >= len(self._views):
            self.step()

    def step(self) -> None:
        """Advance one control period: every view's PD at every physics step."""
        m, d, mj = self.model, self.data, self.mj
        with self.lock:
            for _ in range(self.substeps):
                for view in self._views:
                    view._apply_torque()
                mj.mj_step(m, d)
            self.ticks += 1
        self._pending.clear()
        for view in self._views:
            view._after_step()
        self.render()

    def stamp(self) -> Stamp:
        return Stamp(float(self.data.time), self.ticks + 1)

    def now(self) -> float:
        return 0.0 if self.data is None else float(self.data.time)

    # ---------------------------------------------------------------- render

    def _open_renderers(self) -> None:
        """One offscreen renderer per camera; none if no GL context can be made.

        Physics does not need OpenGL, so a machine without a working
        ``MUJOCO_GL`` backend still runs, only without images. After one
        failure no further context is attempted in this process: with the
        ``glfw`` backend a second attempt aborts the interpreter (mujoco 3.17,
        no display).
        """
        global _GL_ERROR
        if not self.cameras:
            return
        if _GL_ERROR is None:
            try:
                for cam in self.cameras:
                    self._renderers[cam.name] = self.mj.Renderer(self.model, cam.height,
                                                                 cam.width)
                return
            except Exception as exc:  # noqa: BLE001 - mujoco.FatalError, backend import errors
                self._close_renderers()
                _GL_ERROR = exc
        logger.warning(
            "MuJoCo cannot render cameras %s (%s); running without images. Set MUJOCO_GL "
            "to egl or osmesa on a machine without a display.",
            [c.name for c in self.cameras], _GL_ERROR)

    def _close_renderers(self) -> None:
        renderers, self._renderers = self._renderers, {}
        for r in renderers.values():
            r.close()

    def render_camera(self, name: str) -> Optional[np.ndarray]:
        """Camera ``name`` rendered from the current state, or ``None`` without a renderer."""
        r = self._renderers.get(name)
        if r is None:
            return None
        with self.lock:
            r.update_scene(self.data, camera=f"hirobot/{name}")
            return r.render()

    def camera_frame(self, name: str) -> Optional[Tuple[np.ndarray, Stamp]]:
        """Camera ``name``'s frame now: a new one if due at its fps, else the last one.

        ``None`` without a renderer. The stamp is the simulated time it was
        rendered at and the camera's own frame count.
        """
        clock = self._clocks.get(name)
        if clock is None or name not in self._renderers:
            return None
        with self.lock:
            return clock.frame(self.now(), lambda: self.render_camera(name))

    def restart_cameras(self) -> None:
        """Every camera renders on its next frame (the state jumped: a reset, ``set_state``)."""
        for clock in self._clocks.values():
            clock.restart()

    def render(self, force: bool = False) -> None:
        """Push the current state to the viewer, at most ``_VIEWER_HZ``."""
        v = self._viewer
        if v is None or not v.is_running():
            return
        now = time.time()
        if force or now - self._last_sync >= 1.0 / _VIEWER_HZ:
            v.sync()
            self._last_sync = now

    @property
    def viewer_running(self) -> bool:
        return self._viewer is not None and self._viewer.is_running()

    def _close_viewer(self, timeout: float = 3.0) -> None:
        """Close the viewer and wait for its render thread to stop.

        Exiting the process while that thread is still tearing down (or never
        closing it) segfaults inside GLFW, so this blocks until it is gone.
        """
        v, self._viewer = self._viewer, None
        if v is None:
            return
        # The viewer's lock dies with it; later reads need a live one.
        self.lock = threading.RLock()
        v.close()
        deadline = time.time() + timeout
        while v.is_running() and time.time() < deadline:
            time.sleep(0.02)
        time.sleep(0.2)  # let GLFW finish destroying the window


# ------------------------------------------------------------------ cameras


class WorldCamera:
    """A camera rendered by a :class:`SimWorld` from the current state (``simulated``).

    Built in place of the camera's real driver; :meth:`latest` renders when a
    new frame is due at the camera's fps (:meth:`SimWorld.camera_frame`), from
    the state at that moment. Its stamp is the simulated time of the frame and
    the camera's frame count.
    """

    simulated = True

    def __init__(self, name: str, world: SimWorld) -> None:
        self.name = name
        self.world = world

    def start(self) -> None:
        """Nothing: the world opens its renderers on connect."""

    def latest(self) -> Optional[Tuple[np.ndarray, Stamp]]:
        if self.world.model is None:
            return None
        return self.world.camera_frame(self.name)

    def latest_rgbd(self) -> Optional[Tuple[np.ndarray, Optional[np.ndarray], Stamp]]:
        frame = self.latest()
        return None if frame is None else (frame[0], None, frame[1])

    def stop(self) -> None:
        """Nothing: the world closes its renderers."""


def _as_camera_driver() -> None:
    # A WorldCamera is a CameraDriver (registered virtually, so this module does
    # not import the camera module at load time).
    from hirobot.devices.camera import CameraDriver

    CameraDriver.register(WorldCamera)


_as_camera_driver()


# ------------------------------------------------------------- from a tree


def _stand_in(camera: Any) -> bool:
    """Whether a camera device's driver is a stand-in (``simulated``, e.g. ``array``)."""
    from hirobot.devices.camera import get_camera

    driver = camera.driver or {}
    return bool(driver.get("type")) and bool(getattr(get_camera(driver["type"]), "simulated", False))


def sim_options(sim: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    """The ``sim:`` section, checked, with defaults filled in."""
    sim = dict(sim or {})
    unknown = sorted(set(sim) - set(SIM_KEYS) - {"base_height"})
    if unknown:
        raise ValueError(f"sim: unknown key(s) {unknown}; it takes "
                         f"{sorted(SIM_KEYS + ('base_height',))}")
    backend = sim.get("backend", "mujoco")
    if backend != "mujoco":
        raise ValueError(f"sim.backend: only 'mujoco' is supported, got {backend!r}")
    scene = sim.get("scene")
    if scene is not None and not (isinstance(scene, Mapping) and scene.get("mjcf")):
        raise ValueError(f"sim.scene must be a mapping with mjcf, got {scene!r}")
    fixed = sim.get("fixed_base") or []
    if isinstance(fixed, str) or not isinstance(fixed, Sequence):
        raise ValueError(f"sim.fixed_base must be a list of part paths, got {fixed!r}")
    height = sim.get("base_height") or {}
    if not isinstance(height, Mapping):
        raise ValueError(f"sim.base_height must map part paths to heights (m), got {height!r}")
    if sim.get("start_from") not in (None, "model"):
        raise NotImplementedError(
            f"sim.start_from: {sim['start_from']!r} is not supported yet; start from the real "
            "robot's pose with examples/rehearse_in_sim.py (MujocoRobot.set_state)")
    return {"scene": None if scene is None else str(scene["mjcf"]),
            "timestep": sim.get("timestep"), "realtime": bool(sim.get("realtime", True)),
            "viewer": bool(sim.get("viewer", False)),
            "fixed_base": [str(p).strip("/") for p in fixed],
            "base_height": {str(k).strip("/"): float(v) for k, v in height.items()},
            "ramp_seconds": float(sim.get("ramp_seconds", 2.0)),
            "damping_kd": float(sim.get("damping_kd", 8.0)),
            "camera_fps": camera_fps_option(sim.get("camera_fps", SIM_CAMERA_FPS),
                                            "sim.camera_fps")}


@dataclass
class TreeWorld:
    """What :func:`build_world` returns for :mod:`hirobot.loading.build`.

    Attributes:
        world: The world (not connected yet).
        views: Body device path (``left/body``) -> keyword arguments of its
            :class:`~hirobot.drivers.sim.mujoco.MujocoView` (``world``,
            ``prefix``, gains, ...; without the embodiment).
        cameras: Camera part path -> its :class:`WorldCamera`, for the mounted
            cameras in a node without a body (a scene camera). A body's view
            renders the cameras in its own node into its observation, like a
            single simulated robot.
        rendered: Paths of every camera the world renders (mounted, without a
            stand-in driver of its own).
    """

    world: SimWorld
    views: Dict[str, Dict[str, Any]]
    cameras: Dict[str, WorldCamera]
    rendered: set = field(default_factory=set)


def build_world(spec: Any, layout: Any, options: Mapping[str, Any]) -> TreeWorld:
    """The shared world of a loaded tree (``spec``: :class:`~hirobot.loading.tree.RobotSpec`).

    Every body needs a ``model:``. A moving body (``floating_base``) starts at
    its ``origin``; its ``base_position`` is in the world frame.
    """
    from hirobot.loading.tree import SEP, DeviceSpec

    bodies: List[WorldBody] = []
    views: Dict[str, Dict[str, Any]] = {}
    camera_paths: Dict[str, str] = {}
    fixed = set(options["fixed_base"])
    heights = dict(options["base_height"])
    known = set()
    rendered: set = set()

    def walk(node: Any, lay: Any, anchor: Optional[Tuple[str, np.ndarray]], prefix: str) -> None:
        for p, part in node.parts.items():
            path = prefix + p
            pl = lay.parts[p]
            if pl.anchor is not None:
                where: Optional[Tuple[str, np.ndarray]] = (pl.anchor, pl.pose)
            elif pl.kind == "moving":
                base = anchor or (WORLD, IDENTITY_POSE)
                origin = IDENTITY_POSE if pl.origin is None else pl.origin
                where = (base[0], pose_compose(base[1], origin))
            else:
                where = None
            if not isinstance(part, DeviceSpec):
                walk(part, lay.children[p], where, path + SEP)
                continue
            if part.kind == "camera" and not _stand_in(part):
                # Rendered by the world; a camera with a stand-in driver (array) keeps it.
                rendered.add(path)
                if not any(isinstance(d, DeviceSpec) and d.kind == "body"
                           for d in node.parts.values()):
                    camera_paths[path] = path  # no body in its node: a WorldCamera renders it
            if part.kind == "end_effector" and part.fields.get("model"):
                raise NotImplementedError(f"{path}: a hand with a model of its own is not "
                                          "attached to the simulation world yet; use a body "
                                          "MJCF that has the hand's joints")
            if part.kind != "body":
                continue
            model = part.fields.get("model")
            if not model:
                raise ValueError(f"{path}: a body needs a model: (its MJCF) to be in the "
                                 "simulation (sim:); add one to its device file or set "
                                 f"overrides: {{{path}: {{model: {{mjcf: ...}}}}}}")
            if where is None:
                raise ValueError(f"{path}: cannot place it in the simulation world")
            known.add(path)
            if model.get("package_roots") and not model.get("mjcf"):
                raise ValueError(f"{path}: its urdf resolves meshes through package_roots "
                                 "(package://), which the simulation does not read; add an "
                                 "mjcf: to its model:")
            bodies.append(WorldBody(path=path, prefix=prefix, mjcf=os.path.expanduser(
                str(model.get("mjcf") or model["urdf"])), anchor=where[0],
                pose=np.asarray(where[1], dtype=float),
                fixed_base=path in fixed, height=heights.get(path),
                joint_names=dict(model.get("joint_names") or {})))
            kw: Dict[str, Any] = {"prefix": prefix, "path": path,
                                  "ramp_seconds": options["ramp_seconds"],
                                  "damping_kd": options["damping_kd"]}
            for key, arg in (("kp", "default_kp"), ("kd", "default_kd"),
                             ("armature", "armature"), ("home", "initial_joint_pos")):
                if model.get(key) is not None:
                    kw[arg] = model[key]
            views[path] = kw

    walk(spec.root, layout, (WORLD, IDENTITY_POSE), "")
    for name in list(fixed) + list(heights):
        if name not in known:
            raise ValueError(f"sim: {name!r} is not a body of this robot; bodies: {sorted(known)}")
    emb = layout.embodiment
    # Names and links as in the world.
    mounted = [c for c in emb.cameras if c.mount is not None and c.name in rendered]
    world = SimWorld(bodies, cameras=mounted, scene=options["scene"],
                     timestep=options["timestep"], control_hz=emb.control_hz,
                     viewer=options["viewer"], realtime=options["realtime"],
                     camera_fps=options.get("camera_fps", SIM_CAMERA_FPS))
    for kw in views.values():
        kw["world"] = world
    return TreeWorld(world=world, views=views,
                     cameras={c.name: WorldCamera(c.name, world) for c in mounted
                              if c.name in camera_paths},
                     rendered={c.name for c in mounted})


__all__ = ["SimWorld", "TreeWorld", "WorldBody", "WorldCamera", "build_world", "sim_options"]
