"""Assemble a robot from a body driver and its attached components.

:class:`RobotAssembly` wraps a body driver (or a :class:`~hirobot.twin.TwinRobot`)
and the components mounted on it:

* **Camera drivers** (:class:`~hirobot.camera.CameraDriver`). Each
  camera's newest frame goes into ``Observation.images`` with its stamp in
  ``Observation.stamps`` under the camera's name. On a simulated body the
  stand-in cameras' host-clock times are replaced by the simulated time of the
  read (the ``seq`` stays), so every stamp is on :meth:`Robot.now`. For a camera declared ``depth: true``
  the frame comes from ``latest_rgbd()`` and its depth goes into
  ``Observation.depth`` under the same name.
* **End-effector drivers** (:class:`~hirobot.end_effector.EndEffectorDriver`):
  dexterous hands and grippers with a driver of their own. One on the
  ``opening`` interface is read into ``Observation.gripper[<name>]`` and takes
  ``Action.gripper[<name>]`` in any mode; the body never sees that key. The rest
  of this item is about the ``joints`` interface. The assembly's
  embodiment lists the body's joints and the end-effectors' joints together; the
  body driver is built with the body joints only. ``joint_pos`` / ``joint_vel``
  / ``joint_torque`` are gathered from the body and every end-effector into one
  vector, and ``joint_target`` / ``kp`` / ``kd`` are split back out by joint
  name. An end-effector's other signals (tactile arrays) go into
  ``Observation.extra`` as ``<end-effector>_<key>``, and its reading's stamp into
  ``Observation.stamps`` under its name. When the body takes a non-joint mode (a
  body that balances itself and takes ``BASE_VELOCITY``), the action goes to the
  body and any finger targets it carries by ``joint_names`` go to the
  end-effectors, so the fingers are commanded directly while the body follows
  its own controller.
* **Microphones and speakers** (:class:`~hirobot.drivers.audio.MicrophoneDriver`,
  :class:`~hirobot.drivers.audio.SpeakerDriver`). Audio is a stream, not a
  per-tick signal, so nothing goes into the observation: the assembly opens and
  closes the devices, and callers use ``robot.microphones`` / ``robot.speakers``.

Everything else (driver extras such as Unitree's ``raw_state``) goes to the body
unchanged.

``make_robot`` builds the assembly from the embodiment. A simulated body models
the fingers as its own joints and gets only stand-in cameras and audio devices
(video file, WAV, the host's microphone), so those configs keep working on a
host without the robot.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.end_effector import check_opening
from hirobot.robot import Robot
from hirobot.robot_skills import RobotSkillResult, RobotSkillSpec
from hirobot.stamps import Stamp
from hirobot.types import JOINT_MODES, Action, ControlMode, Observation, RobotStatus

if TYPE_CHECKING:  # importing hirobot.drivers here would load every driver with hirobot
    from hirobot.audio import MicrophoneDriver, SpeakerDriver
    from hirobot.camera import CameraDriver
    from hirobot.end_effector import EndEffectorDriver

logger = logging.getLogger(__name__)

_EE_MODES = (ControlMode.POSITION, ControlMode.PD)


class RobotAssembly(Robot):
    """A robot body plus the components attached to it.

    Args:
        body: The driver that owns the body joints.
        cameras: Camera name -> camera driver. Names become ``images`` keys.
        end_effectors: End-effector name -> end-effector driver.
        embodiment: The whole robot's embodiment (body plus finger joints).
            Defaults to the body's, which is only valid without end-effectors.
        first_frame_timeout_s: How long :meth:`connect` waits for each camera's
            first frame. A camera that misses it only logs a warning: the body
            stays usable and the camera's key is absent from ``images`` until
            frames arrive.
        microphones: Microphone name -> driver.
        speakers: Speaker name -> driver.
        first_audio_timeout_s: How long :meth:`connect` waits for each
            microphone's first chunk. A miss only logs a warning, as for cameras.
    """

    def __init__(
        self,
        body: Robot,
        cameras: Optional[Dict[str, "CameraDriver"]] = None,
        end_effectors: Optional[Dict[str, "EndEffectorDriver"]] = None,
        embodiment: Optional[Embodiment] = None,
        first_frame_timeout_s: float = 3.0,
        microphones: Optional[Dict[str, "MicrophoneDriver"]] = None,
        speakers: Optional[Dict[str, "SpeakerDriver"]] = None,
        first_audio_timeout_s: float = 2.0,
    ) -> None:
        self.body = body
        self.cameras = dict(cameras or {})
        self.end_effectors = dict(end_effectors or {})
        self.microphones = dict(microphones or {})
        self.speakers = dict(speakers or {})
        self._first_audio_timeout_s = first_audio_timeout_s
        self._embodiment = embodiment if embodiment is not None else body.embodiment
        self._first_frame_timeout_s = first_frame_timeout_s
        self._depth_cameras = {c.name for c in self._embodiment.cameras if c.depth}
        names = self._embodiment.joint_names
        index = {n: i for i, n in enumerate(names)}
        body_names = body.embodiment.joint_names
        # Joint routing below only concerns the ``joints`` end-effectors; the
        # ``opening`` ones carry no joints and go through ``gripper``.
        self._joint_ees = {k: e for k, e in self.end_effectors.items() if e.interface == "joints"}
        self._opening_ees = {k: e for k, e in self.end_effectors.items() if e.interface == "opening"}
        ee_names = [n for h in self._joint_ees.values() for n in h.joint_names]
        if sorted(body_names + ee_names) != sorted(names):
            raise ValueError(
                "body joints plus end-effector joints must be exactly the embodiment's joints "
                f"(body {len(body_names)} + end-effectors {len(ee_names)} vs {len(names)})"
            )
        self._body_idx = np.array([index[n] for n in body_names], dtype=int)
        self._ee_idx = {k: np.array([index[n] for n in h.joint_names], dtype=int)
                        for k, h in self._joint_ees.items()}

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    @property
    def action_modes(self):  # type: ignore[override]
        # With end-effectors attached, joint targets are split across the body
        # and the end-effectors, which only works for position-style modes.
        # Non-joint modes go to the body, with finger targets (by joint_names)
        # split off to the end-effectors.
        modes = self.body.action_modes
        if not self._joint_ees:
            return modes
        return tuple(m for m in modes if m in _EE_MODES or m not in JOINT_MODES)

    @property
    def motion_spaces(self):  # type: ignore[override]
        return self.body.motion_spaces

    @property
    def direct_joints(self):  # type: ignore[override]
        if not self._joint_ees:
            return self.body.direct_joints
        ee = [n for h in self._joint_ees.values() for n in h.joint_names]
        return tuple(self.body.direct_joints) + tuple(ee)

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return self.body.cartesian_targets

    @property
    def fault(self):  # type: ignore[override]
        # A layer under the assembly (a whole-body controller) that damped itself.
        return self._fault or self.body.fault

    def idle_step(self) -> None:
        self.body.idle_step()

    def now(self) -> float:
        return self.body.now()

    # The sim flag and outcome come from the body. The base class declares them,
    # so ``__getattr__`` would never be reached for them.
    @property
    def simulated(self) -> bool:  # type: ignore[override]
        return bool(getattr(self.body, "simulated", False))

    @property
    def reward(self):  # type: ignore[override]
        return getattr(self.body, "reward", None)

    @property
    def success(self):  # type: ignore[override]
        return getattr(self.body, "success", None)

    @property
    def done(self):  # type: ignore[override]
        return getattr(self.body, "done", None)

    def __getattr__(self, name: str) -> Any:
        # Driver extras (raw_state, motion_mode, mode, real, ...) come from the
        # body. Only reached for attributes not found on the assembly itself.
        if name in ("body", "cameras", "end_effectors", "microphones", "speakers"):
            raise AttributeError(name)
        return getattr(self.body, name)

    # -------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        # The body first: on Unitree it initialises DDS on the chosen NIC, which
        # the videohub camera and the Dex3 hands then reuse.
        token = self._stop_token()
        self.body.connect()
        started: List[Any] = []
        try:
            for ee in self.end_effectors.values():
                self._check_stop(token)
                ee.connect()
                started.append(ee.disconnect)
            for cam in self.cameras.values():
                cam.start()
                started.append(cam.stop)
            for dev in (*self.microphones.values(), *self.speakers.values()):
                dev.open()
                started.append(dev.close)
        except BaseException:
            for undo in reversed(started):
                undo()
            self.body.disconnect()
            raise
        for cam in self.cameras.values():
            wait = getattr(cam, "wait_for_frame", None)
            if wait is None:
                continue
            try:
                wait(self._first_frame_timeout_s)
            except TimeoutError as exc:
                logger.warning("%s; continuing without it until frames arrive", exc)
        for mic in self.microphones.values():
            wait = getattr(mic, "wait_for_audio", None)
            if wait is None:
                continue
            try:
                wait(self._first_audio_timeout_s)
            except TimeoutError as exc:
                logger.warning("%s; continuing without it", exc)

    def reset(self, seed: Optional[int] = None) -> None:
        token = self._stop_token()
        self.body.reset(seed)
        for ee in self.end_effectors.values():
            self._check_stop(token)
            ee.reset()

    def hold(self) -> None:
        if not self._joint_ees:
            self.body.hold()
        elif all(m in JOINT_MODES for m in self.body.action_modes):
            # Body and fingers hold together: the base hold sends every joint.
            super().hold()
        else:
            # The body runs its own controller (a self-balancing humanoid): it
            # holds itself, and each end-effector keeps its measured pose.
            self.body.hold()
            for ee in self._joint_ees.values():
                ee.command(ee.get_state().q)

    def get_observation(self) -> Observation:
        obs = self.body.get_observation()
        sim_now = self.body.now() if self.simulated else None
        if self._joint_ees:
            n = self._embodiment.num_dofs
            for field in ("joint_pos", "joint_vel", "joint_torque"):
                part = getattr(obs, field)
                if part is None:
                    continue
                full = np.full(n, np.nan)
                full[self._body_idx] = part
                setattr(obs, field, full)
            for key, ee in self._joint_ees.items():
                state = ee.get_state()
                idx = self._ee_idx[key]
                obs.joint_pos[idx] = state.q
                if obs.joint_vel is not None:
                    obs.joint_vel[idx] = state.dq
                if obs.joint_torque is not None:
                    obs.joint_torque[idx] = state.tau
                for k, v in state.extra.items():
                    obs.extra[f"{key}_{k}"] = v
                obs.stamps[key] = state.stamp
        for key, ee in self._opening_ees.items():
            state = ee.get_state()
            obs.gripper[key] = state.opening
            for k, v in state.extra.items():
                obs.extra[f"{key}_{k}"] = v
            obs.stamps[key] = state.stamp
        for name, cam in self.cameras.items():
            if name in self._depth_cameras:
                rgbd = cam.latest_rgbd()
                if rgbd is None:
                    continue
                image, depth, stamp = rgbd
                if depth is not None:
                    obs.depth[name] = depth
            else:
                frame = cam.latest()
                if frame is None:
                    continue
                image, stamp = frame
            obs.images[name] = image
            obs.stamps[name] = stamp if sim_now is None else Stamp(sim_now, stamp.seq)
        return obs

    def _split(self, values: Optional[np.ndarray], idx: np.ndarray) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values, dtype=float)
        return arr[idx] if arr.ndim == 1 else arr  # a scalar gain applies to every part

    def _send_action(self, action: Action) -> None:
        # The safety gate already ran in send_action on this wrapper.
        openings = {k: check_opening(v) for k, v in (action.gripper or {}).items()
                    if k in self._opening_ees}
        if openings:
            # The body does not drive these grippers; it never sees their keys.
            action = action.model_copy(update={"gripper": {
                k: v for k, v in action.gripper.items() if k not in openings}})
        self._send_joints(action)
        for key, value in openings.items():
            self._opening_ees[key].command_opening(value)

    def _send_joints(self, action: Action) -> None:
        if not self._joint_ees or action.joint_target is None:
            self.body._send_action(action)
            return
        if action.joint_names is not None:
            self._send_named(action)
            return
        if action.mode not in _EE_MODES:
            raise ValueError(
                f"end-effectors take position targets; mode {action.mode.value!r} is not supported")
        body_action = action.model_copy(update={
            "joint_target": self._split(action.joint_target, self._body_idx),
            "kp": self._split(action.kp, self._body_idx),
            "kd": self._split(action.kd, self._body_idx),
        })
        self.body._send_action(body_action)
        for key, ee in self._joint_ees.items():
            idx = self._ee_idx[key]
            ee.command(np.asarray(action.joint_target, dtype=float)[idx],
                         self._split(action.kp, idx), self._split(action.kd, idx))

    def _send_named(self, action: Action) -> None:
        """A non-joint mode with targets for some joints.

        Finger targets go to the end-effectors, the rest to the body.
        """
        names = list(action.joint_names or [])
        pos = {n: i for i, n in enumerate(names)}
        target = np.asarray(action.joint_target, dtype=float)
        ee_cmds = []
        for key, ee in self._joint_ees.items():
            mine = [n for n in ee.joint_names if n in pos]
            if not mine:
                continue
            if len(mine) != ee.num_joints:
                missing = [n for n in ee.joint_names if n not in pos]
                raise ValueError(
                    f"{key}: an end-effector takes all its joints at once; missing {missing}")
            idx = np.array([pos[n] for n in ee.joint_names], dtype=int)
            ee_cmds.append((ee, idx))
        ee_names = {n for ee, _ in ee_cmds for n in ee.joint_names}
        rest = [i for i, n in enumerate(names) if n not in ee_names]
        body_action = action.model_copy(update={
            "joint_names": [names[i] for i in rest] if rest else None,
            "joint_target": target[rest] if rest else None,
            "kp": self._split(action.kp, np.array(rest, dtype=int)) if rest else None,
            "kd": self._split(action.kd, np.array(rest, dtype=int)) if rest else None,
        })
        self.body._send_action(body_action)
        for ee, idx in ee_cmds:
            ee.command(target[idx], self._split(action.kp, idx), self._split(action.kd, idx))

    @property
    def has_robot_skills(self) -> bool:
        return self.body.has_robot_skills

    def list_robot_skills(self) -> List[RobotSkillSpec]:
        return self.body.list_robot_skills()

    def run_robot_skill(
        self, name: str, args: Optional[Mapping[str, Any]] = None
    ) -> RobotSkillResult:
        return self.body.run_robot_skill(name, args)

    def stop_robot_skill(self) -> None:
        self.body.stop_robot_skill()

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if not self._joint_ees:
            self.body.set_gains(kp, kd)
            return
        self.body.set_gains(self._split(kp, self._body_idx), self._split(kd, self._body_idx))
        for key, ee in self._joint_ees.items():
            idx = self._ee_idx[key]
            ee.set_gains(self._split(kp, idx), self._split(kd, idx))

    def get_status(self) -> RobotStatus:
        status = self.body.get_status()
        status.fault = self.fault
        temps = status.joint_temperature
        if self._joint_ees and temps is not None:
            # End-effector joints are not covered by the body's status; mark them unknown.
            full = np.full(self._embodiment.num_dofs, np.nan)
            full[self._body_idx] = temps
            status.joint_temperature = full
        return status

    def damp(self) -> None:
        """Damp the body (if its driver can) and every end-effector."""
        body_damp = getattr(self.body, "damp", None)
        if body_damp is not None:
            body_damp()
        for ee in self.end_effectors.values():
            ee.damp()

    def _estop(self) -> None:
        """E-stop the body and every end-effector, raising the first error.

        Each part cuts its own ramp short.
        """
        first: Optional[BaseException] = None
        for part in (self.body, *self.end_effectors.values()):
            try:
                part.estop()
            except Exception as exc:  # noqa: BLE001 - stop every part
                logger.exception("estop on %s failed", getattr(part, "name", type(part).__name__))
                first = first or exc
        if first is not None:
            raise first

    def disconnect(self) -> None:
        try:
            self.body.disconnect()  # stop the robot first
        finally:
            for ee in self.end_effectors.values():
                try:
                    ee.disconnect()
                except Exception:  # noqa: BLE001 - damp the others regardless
                    logger.exception("disconnecting end-effector %s failed", ee.name)
            for cam in self.cameras.values():
                cam.stop()
            for dev in (*self.speakers.values(), *self.microphones.values()):
                try:
                    dev.close()
                except Exception:  # noqa: BLE001 - close the other devices regardless
                    logger.exception("closing audio device %s failed", dev.name)
