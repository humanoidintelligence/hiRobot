"""MuJoCo physics driver.

Simulates a robot from an MJCF model behind the :class:`~hirobot.robot.Robot`
contract, so control code written for real hardware runs unchanged in sim.
Joints are matched to the embodiment by name (``<name>`` or ``<name>_joint``),
so the model's joint order does not have to match the embodiment's.

The driver mirrors how a PD-controlled motor behaves: ``send_action`` stores the
command, then advances physics by one control period, recomputing
``tau = kp * (q* - q) + kd * (dq* - dq) + tau_ff`` at every physics step (the
real motors run their PD loop much faster than the control rate too). Torques
are clamped by the model's ``actuatorfrcrange``.

Embodiment cameras with a ``mount`` are added to the model on :meth:`connect`
(fixed to ``mount.body``) and rendered offscreen in :meth:`get_observation`,
from the same state as the joints, into ``Observation.images``. Each rendered
camera's stamp in ``Observation.stamps`` equals the body's: the simulated time
(``data.time``, also :meth:`now`) and the count of control periods stepped, so
a frame belongs to the same simulated instant as the joints.
Images are the camera's full resolution, unprocessed.

End-effectors with a ``frame`` get ``Observation.ee_poses``: the pose of
``frame.body`` (plus the frame's offset) in the robot's base frame, the root
body of the model (the pelvis of a humanoid, the base link of an arm), as
``[x, y, z, qx, qy, qz, qw]``. Base frame, like the CARTESIAN targets a
whole-body controller takes; with a floating base, compose with
``base_position`` / ``base_orientation`` for the world pose. The rendering backend
follows ``MUJOCO_GL`` (``glfw`` needs a display; ``egl`` / ``osmesa`` run headless);
without a working one the robot runs without images and logs a warning.

It also serves as the renderer for :class:`~hirobot.twin.TwinRobot`,
which mirrors a real robot's state into the viewer (:meth:`set_state`).

MuJoCo is imported lazily: install it with ``pip install mujoco`` (or the
``[sim]`` extra). Models are not shipped with hiRobot; see
``docs/setup/unitree.md`` for fetching the Unitree G1 MJCF.
"""

from __future__ import annotations

import atexit
import logging
import os
import threading
import time
from typing import Optional, Sequence, Union

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot
from hirobot.sdk import require_sdk
from hirobot.stamps import Stamp
from hirobot.transforms import pose_compose, pose_inverse, quat_multiply
from hirobot.types import Action, ControlMode, Observation

logger = logging.getLogger(__name__)

#: Why offscreen rendering failed in this process, or None (see _open_renderers).
_GL_ERROR: Optional[BaseException] = None

_VIEWER_HZ = 60.0
#: Half a turn about x, ``[x, y, z, w]``: OpenCV optical frame -> MuJoCo camera frame.
_OPTICAL_TO_MUJOCO = np.array([1.0, 0.0, 0.0, 0.0])


@register_body("mujoco")
class MujocoRobot(Robot):
    """A MuJoCo-simulated robot.

    Args:
        embodiment: The embodiment; every joint must exist in the model.
        model_path: Path to the MJCF (``.xml``) file; ``~`` is expanded.
        fixed_base: Remove the model's free joint and weld the root body in
            place, e.g. to match a humanoid hung on a gantry.
        base_height: Root body height (m). With ``fixed_base`` it keeps the
            feet off the floor; ``None`` keeps the model's default.
        viewer: Open an interactive MuJoCo viewer window on :meth:`connect`.
        realtime: Sleep in :meth:`reset` so its gain ramp plays at wall-clock
            speed. ``send_action`` never sleeps; the caller sets the pace.
        default_kp / default_kd: Gains used when an action does not carry its
            own; scalar or one value per joint (embodiment order).
        ramp_seconds: Duration of the gain ramp performed by :meth:`reset`.
        damping_kd: Damping used by :meth:`damp` (kp = 0).
        initial_joint_pos: Joint pose (embodiment order) at connect; defaults
            to the model's zero pose.
        armature: Reflected rotor inertia added to each joint (kg m^2), scalar
            or per joint; ``None`` keeps the model's values. Models exported
            from URDF often have none, which makes light links (wrists) unstable
            under PD at a 2 ms timestep.
    """

    simulated = True

    def __init__(
        self,
        embodiment: Embodiment,
        model_path: str,
        fixed_base: bool = False,
        base_height: Optional[float] = None,
        viewer: bool = False,
        realtime: bool = True,
        default_kp: Union[float, Sequence[float]] = 60.0,
        default_kd: Union[float, Sequence[float]] = 1.0,
        ramp_seconds: float = 2.0,
        damping_kd: float = 8.0,
        initial_joint_pos: Optional[Sequence[float]] = None,
        armature: Union[None, float, Sequence[float]] = None,
    ) -> None:
        self._embodiment = embodiment
        self._n = embodiment.num_dofs
        self._model_path = os.path.expanduser(model_path)
        self._fixed_base = fixed_base
        self._base_height = base_height
        self._use_viewer = viewer
        self._realtime = realtime
        self._kp = self._per_joint(default_kp, "default_kp")
        self._kd = self._per_joint(default_kd, "default_kd")
        self._ramp_seconds = float(ramp_seconds)
        self._damping_kd = float(damping_kd)
        self._armature = None if armature is None else self._per_joint(armature, "armature")
        self._initial = (None if initial_joint_pos is None
                         else self._per_joint(initial_joint_pos, "initial_joint_pos"))

        self._mj = None
        self.model = None
        self.data = None
        #: Control periods stepped since connect (the ``seq`` of every stamp).
        self._ticks = 0
        self._viewer = None
        self._last_sync = 0.0
        # A lock shared by everything that touches mjData (physics, viewer sync,
        # a mirroring thread); falls back to a plain lock without a viewer.
        self._lock = threading.RLock()
        # Command held between control ticks.
        self._cmd_q = np.zeros(self._n)
        self._cmd_dq = np.zeros(self._n)
        self._cmd_tau = np.zeros(self._n)
        self._cmd_kp = np.zeros(self._n)
        self._cmd_kd = np.zeros(self._n)
        self._connected = False
        self._cameras: list = []
        self._renderers: dict = {}
        self._ee_frames: list = []   # (name, body id, offset pose) per end-effector with a frame
        self._root = 0

    # ------------------------------------------------------------------ setup

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def _per_joint(self, value, name: str) -> np.ndarray:
        arr = np.asarray(value, dtype=float)
        if arr.ndim == 0:
            return np.full(self._n, float(arr))
        if arr.shape != (self._n,):
            raise ValueError(f"{name} has {arr.size} values, embodiment expects {self._n}")
        return arr.copy()

    def _import_mujoco(self):
        require_sdk(__package__, "MujocoRobot")
        import mujoco

        return mujoco

    def _build_model(self):
        mj = self._mj
        if not os.path.isfile(self._model_path):
            raise FileNotFoundError(
                f"MJCF model not found: {self._model_path}. For the Unitree G1 see "
                "docs/setup/unitree.md (fetch unitree_ros robots/g1_description)."
            )
        spec = mj.MjSpec.from_file(self._model_path)
        free = [j for j in spec.joints if j.type == mj.mjtJoint.mjJNT_FREE]
        root = free[0].parent if free else None
        if self._fixed_base:
            for j in free:
                spec.delete(j)
        if self._base_height is not None and root is not None:
            root.pos = [root.pos[0], root.pos[1], float(self._base_height)]
        self._add_cameras(spec)
        return spec.compile()

    def _add_cameras(self, spec) -> None:
        """Fix each embodiment camera that has a ``mount`` to its link in the model."""
        self._cameras = [c for c in self._embodiment.cameras if c.mount is not None]
        if not self._cameras:
            return
        bodies = {b.name for b in spec.bodies}
        for cam in self._cameras:
            mount = cam.mount
            if mount.body not in bodies:
                raise ValueError(f"camera {cam.name!r}: mount.body {mount.body!r} is not a body "
                                 f"of {self._model_path}")
            # mount.quat is the OpenCV optical frame (z forward, y down); a MuJoCo
            # camera looks along its -z with y up, i.e. the optical frame turned
            # half a turn about x.
            q = quat_multiply(np.asarray(mount.quat, dtype=float), _OPTICAL_TO_MUJOCO)
            spec.body(mount.body).add_camera(
                name=f"hirobot_{cam.name}", pos=list(mount.pos),
                quat=[q[3], q[0], q[1], q[2]],  # MuJoCo is [w, x, y, z]
                fovy=float(mount.fovy))
        g = spec.visual.global_
        g.offwidth = max(int(g.offwidth), *(c.width for c in self._cameras))
        g.offheight = max(int(g.offheight), *(c.height for c in self._cameras))

    def _open_renderers(self) -> None:
        """One offscreen renderer per mounted camera; none if no GL context can be made.

        Physics does not need OpenGL, so a headless machine without a working
        ``MUJOCO_GL`` backend still runs the robot, only without images. After
        one failure no further context is attempted in this process: with the
        ``glfw`` backend a second attempt aborts the interpreter (seen with
        mujoco 3.17, no display).
        """
        global _GL_ERROR
        if not self._cameras:
            return
        if _GL_ERROR is None:
            try:
                for cam in self._cameras:
                    self._renderers[cam.name] = self._mj.Renderer(self.model, cam.height,
                                                                  cam.width)
                return
            except Exception as exc:  # noqa: BLE001 - mujoco.FatalError, backend import errors
                self._close_renderers()
                _GL_ERROR = exc
        logger.warning(
            "MuJoCo cannot render cameras %s (%s); running without images. Set MUJOCO_GL "
            "to egl or osmesa on a machine without a display.",
            [c.name for c in self._cameras], _GL_ERROR)

    def _close_renderers(self) -> None:
        renderers, self._renderers = self._renderers, {}
        for r in renderers.values():
            r.close()

    def _render_images(self, obs: Observation) -> None:
        """Render every mounted camera from the current state (caller holds the lock)."""
        for name, r in self._renderers.items():
            r.update_scene(self.data, camera=f"hirobot_{name}")
            obs.images[name] = r.render()

    def _map_joints(self) -> None:
        m, mj = self.model, self._mj
        qadr, vadr, act, jids = [], [], [], []
        joint_to_act = {int(m.actuator_trnid[a, 0]): a for a in range(m.nu)
                        if m.actuator_trntype[a] == mj.mjtTrn.mjTRN_JOINT}
        for name in self._embodiment.joint_names:
            jid = -1
            for cand in (name, f"{name}_joint"):
                jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, cand)
                if jid >= 0:
                    break
            if jid < 0:
                raise ValueError(
                    f"Joint {name!r} (or {name}_joint) not found in {self._model_path}"
                )
            if jid not in joint_to_act:
                raise ValueError(f"Joint {name!r} has no joint actuator in {self._model_path}")
            jids.append(jid)
            qadr.append(int(m.jnt_qposadr[jid]))
            vadr.append(int(m.jnt_dofadr[jid]))
            act.append(joint_to_act[jid])
        self._qadr = np.array(qadr)
        self._vadr = np.array(vadr)
        self._act = np.array(act)
        self._jid = np.array(jids)
        self._has_free = m.nq > 0 and m.njnt > 0 and m.jnt_type[0] == mj.mjtJoint.mjJNT_FREE
        self._substeps = max(1, round(self.control_period / m.opt.timestep))
        if self._armature is not None:
            m.dof_armature[self._vadr] = self._armature
        # Torque limits: the joint's actuatorfrcrange, else the actuator's
        # forcerange/ctrlrange, else unlimited. Clamped in _advance so the
        # reported joint_torque is the torque actually applied.
        lo, hi = np.full(self._n, -np.inf), np.full(self._n, np.inf)
        for i, (a, jid) in enumerate(zip(self._act, self._jid)):
            if m.jnt_actfrclimited[jid]:
                lo[i], hi[i] = m.jnt_actfrcrange[jid]
            elif m.actuator_forcelimited[a]:
                lo[i], hi[i] = m.actuator_forcerange[a]
            elif m.actuator_ctrllimited[a]:
                lo[i], hi[i] = m.actuator_ctrlrange[a]
        self._tau_lo, self._tau_hi = lo, hi
        # Base frame for ee_poses: the root body of the tree the joints are on.
        self._root = int(m.body_rootid[m.jnt_bodyid[self._jid[0]]]) if self._n else 0
        self._ee_frames = []
        for ee in self._embodiment.end_effectors:
            if ee.frame is None:
                continue
            bid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, ee.frame.body)
            if bid < 0:
                raise ValueError(f"end-effector {ee.name!r}: frame.body {ee.frame.body!r} is not a "
                                 f"body of {self._model_path}")
            offset = np.array([*ee.frame.pos, *ee.frame.quat], dtype=float)
            self._ee_frames.append((ee.name, bid, offset))

    @property
    def control_period(self) -> float:
        hz = self._embodiment.control_hz
        return 1.0 / hz if hz else 0.02

    def connect(self) -> None:
        self._mj = self._import_mujoco()
        self.model = self._build_model()
        self.data = self._mj.MjData(self.model)
        self._map_joints()
        if self._initial is not None:
            self.data.qpos[self._qadr] = self._initial
        self._mj.mj_forward(self.model, self.data)
        self._ticks = 0
        self._cmd_q = self.data.qpos[self._qadr].copy()
        self._open_renderers()
        if self._use_viewer:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._lock = self._viewer.lock()
            atexit.register(self._close_viewer)  # scripts that never disconnect()
        self._connected = True
        self.render(force=True)

    # --------------------------------------------------------------- control

    def reset(self, seed: Optional[int] = None) -> None:
        """Ramp kp from 0 while holding the current pose (mirrors the Unitree driver).

        An :meth:`estop` from another thread ends the ramp at the next step
        (:class:`~hirobot.robot.RobotStoppedError`), damped.
        """
        token = self._stop_token()
        start = self.data.qpos[self._qadr].copy()
        steps = max(1, int(self._ramp_seconds / self.control_period))
        for s in range(steps + 1):
            ratio = min(1.0, s / steps)
            with self._commanding(token):
                self._set_command(start, self._kp * ratio, self._kd, None, None)
                self._advance()
            if self._realtime:
                self._wait(self.control_period, token)

    def _set_command(self, q, kp, kd, dq, tau) -> None:
        z = np.zeros(self._n)
        self._cmd_q = z if q is None else np.asarray(q, dtype=float).copy()
        self._cmd_dq = z if dq is None else np.asarray(dq, dtype=float).copy()
        self._cmd_tau = z if tau is None else np.asarray(tau, dtype=float).copy()
        self._cmd_kp = np.asarray(kp, dtype=float).copy()
        self._cmd_kd = np.asarray(kd, dtype=float).copy()

    def _advance(self) -> None:
        """Step physics for one control period under the held PD command."""
        m, d, mj = self.model, self.data, self._mj
        with self._lock:
            for _ in range(self._substeps):
                q = d.qpos[self._qadr]
                dq = d.qvel[self._vadr]
                tau = (self._cmd_kp * (self._cmd_q - q)
                       + self._cmd_kd * (self._cmd_dq - dq) + self._cmd_tau)
                d.ctrl[self._act] = np.clip(tau, self._tau_lo, self._tau_hi)
                mj.mj_step(m, d)
            self._ticks += 1
        self.render()

    def _send_action(self, action: Action) -> None:
        if action.joint_target is None:
            return
        target = np.asarray(action.joint_target, dtype=float)
        kp = self._kp if action.kp is None else np.asarray(action.kp, dtype=float)
        kd = self._kd if action.kd is None else np.asarray(action.kd, dtype=float)
        if action.mode in (ControlMode.POSITION, ControlMode.PD):
            self._set_command(target, kp, kd, None, None)
        elif action.mode == ControlMode.VELOCITY:
            self._set_command(None, np.zeros(self._n), kd, target, None)
        elif action.mode == ControlMode.TORQUE:
            self._set_command(None, np.zeros(self._n), np.zeros(self._n), None, target)
        else:
            raise ValueError(f"MujocoRobot does not support mode {action.mode!r}")
        self._advance()

    def idle_step(self) -> None:
        """Step physics one control period under the held command (no new target)."""
        if self.data is not None:
            self._advance()

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if kp is not None:
            self._kp = self._per_joint(kp, "kp")
        if kd is not None:
            self._kd = self._per_joint(kd, "kd")

    def damp(self) -> None:
        """Switch the held command to damping (kp = 0, kd = ``damping_kd``)."""
        z = np.zeros(self._n)
        self._set_command(z, z, np.full(self._n, self._damping_kd), None, None)

    # ----------------------------------------------------------- observation

    def get_observation(self) -> Observation:
        if self.data is None:
            raise RuntimeError("Not connected; call connect() first.")
        d = self.data
        with self._lock:
            obs = Observation(
                joint_pos=d.qpos[self._qadr].copy(),
                joint_vel=d.qvel[self._vadr].copy(),
                joint_torque=d.actuator_force[self._act].copy(),
            )
            if self._embodiment.floating_base:
                if self._has_free:
                    # MuJoCo stores [w, x, y, z]; Observation is [x, y, z, w].
                    obs.base_orientation = d.qpos[[4, 5, 6, 3]].copy()
                    obs.base_angular_velocity = d.qvel[3:6].copy()
                    obs.base_position = d.qpos[0:3].copy()
                    obs.base_linear_velocity = d.qvel[0:3].copy()
                else:
                    obs.base_orientation = np.array([0.0, 0.0, 0.0, 1.0])
                    obs.base_angular_velocity = np.zeros(3)
            if self._ee_frames:
                # mj_step leaves body poses one physics step behind qpos; bring them level
                # so ee_poses match joint_pos (mj_kinematics only touches derived positions).
                self._mj.mj_kinematics(self.model, d)
                obs.ee_poses = self._ee_poses()
            self._render_images(obs)
            stamp = Stamp(float(d.time), self._ticks + 1)
            obs.stamps["body"] = stamp
            for name in obs.images:
                obs.stamps[name] = stamp
        return obs

    def now(self) -> float:
        """Simulated time (``data.time``); 0 before :meth:`connect`."""
        return 0.0 if self.data is None else float(self.data.time)

    def _body_pose(self, bid: int) -> np.ndarray:
        d = self.data
        return np.concatenate([d.xpos[bid], d.xquat[bid][[1, 2, 3, 0]]])  # MuJoCo quat is [w,x,y,z]

    def _ee_poses(self) -> dict:
        """End-effector poses in the base frame (caller holds the lock)."""
        to_base = pose_inverse(self._body_pose(self._root))
        return {name: pose_compose(to_base, pose_compose(self._body_pose(bid), offset))
                for name, bid, offset in self._ee_frames}

    # ------------------------------------------------------ state sync / view

    def set_state(self, joint_pos: np.ndarray, joint_vel: Optional[np.ndarray] = None,
                  base_orientation: Optional[np.ndarray] = None) -> None:
        """Overwrite the simulated state (kinematic: no physics step).

        Used to mirror a real robot into the viewer, or to start a simulation
        from a measured pose. ``base_orientation`` is ``[x, y, z, w]`` and is
        ignored when the base is fixed. The held command is reset to hold this
        pose with zero gains, so nothing jumps when physics resumes.
        """
        d, mj = self.data, self._mj
        with self._lock:
            d.qpos[self._qadr] = np.asarray(joint_pos, dtype=float)
            d.qvel[:] = 0.0
            if joint_vel is not None:
                d.qvel[self._vadr] = np.asarray(joint_vel, dtype=float)
            if self._has_free and base_orientation is not None:
                q = np.asarray(base_orientation, dtype=float)
                norm = np.linalg.norm(q)
                if norm > 0:
                    d.qpos[3:7] = q[[3, 0, 1, 2]] / norm  # -> MuJoCo's [w, x, y, z]
            mj.mj_forward(self.model, d)
        z = np.zeros(self._n)
        self._set_command(np.asarray(joint_pos, dtype=float), z, z, None, None)
        self.render()

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

    def disconnect(self) -> None:
        if self._connected:
            self.damp()
        self._connected = False
        self._close_renderers()
        self._close_viewer()

    def _close_viewer(self, timeout: float = 3.0) -> None:
        """Close the viewer and wait for its render thread to stop.

        Exiting the process while that thread is still tearing down (or never
        closing it) segfaults inside GLFW, so this blocks until it is gone.
        """
        v, self._viewer = self._viewer, None
        if v is None:
            return
        # The viewer's lock dies with it; later reads (e.g. after disconnect) need a live one.
        self._lock = threading.RLock()
        v.close()
        deadline = time.time() + timeout
        while v.is_running() and time.time() < deadline:
            time.sleep(0.02)
        time.sleep(0.2)  # let GLFW finish destroying the window
