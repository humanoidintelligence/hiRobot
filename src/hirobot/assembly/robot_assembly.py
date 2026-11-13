"""Assemble a robot from a body driver and its attached components.

:class:`RobotAssembly` wraps a body driver
and the components mounted on it:

* **Camera drivers** (:class:`~hirobot.devices.camera.CameraDriver`). Each
  camera's newest frame goes into ``Observation.images`` with its stamp in
  ``Observation.stamps`` under the camera's name. On a simulated body the
  stand-in cameras' host-clock times are replaced by the simulated time of the
  read (the ``seq`` stays), so every stamp is on :meth:`Robot.now`. For a camera declared ``depth: true``
  the frame comes from ``latest_rgbd()`` and its depth goes into
  ``Observation.depth`` under the same name.
* **End-effector drivers** (:class:`~hirobot.devices.end_effector.EndEffectorDriver`):
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

**Child parts.** With ``parts=`` (part name -> :class:`~hirobot.core.robot.Robot`)
the assembly is a node of an ``assembly:`` config: several robots (two arms,
a humanoid and an arm) plus the cameras, microphones and speakers of the
node, driven as one robot. ``body`` is ``None`` then; what the node's own body
would be is a part like the others. Names, observations, frames, splitting
and the stop-everything-on-error rule are in :mod:`hirobot.assembly.node`. The parts
are usually built without a gate of their own: the one gate around the root
checks the whole action (``Action.parts`` included) in the robot's names.

``make_robot`` builds the assembly from the embodiment. A simulated body models
the fingers as its own joints and gets only stand-in cameras and audio devices
(video file, WAV, the host's microphone), so those configs keep working on a
host without the robot.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.devices.end_effector import check_opening
from hirobot.assembly.node import OWN, NodeParts
from hirobot.assembly.parts import PartView
from hirobot.core.robot import Robot
from hirobot.core.robot_skills import RobotSkillResult, RobotSkillSpec
from hirobot.core.stamps import Stamp
from hirobot.core.types import JOINT_MODES, Action, ControlMode, Observation, RobotStatus

if TYPE_CHECKING:  # importing hirobot.drivers here would load every driver with hirobot
    from hirobot.devices.audio import MicrophoneDriver, SpeakerDriver
    from hirobot.devices.camera import CameraDriver
    from hirobot.devices.end_effector import EndEffectorDriver

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
        parts: Child part name -> robot. Needs ``body=None`` and an
            ``embodiment`` that lists the parts' joints with the ``<part>/``
            prefix (see :func:`hirobot.assembly.parts.merge_embodiments`); ``cameras``
            and the audio devices are then the node's own.
        placement: Child part name -> ``{"pose": [x, y, z, qx, qy, qz, qw]}``
            (where it sits in the node) or ``{"origin": [...]}`` (a part that
            moves by itself: where its odometry origin sits). See
            :mod:`hirobot.assembly.node`.
        collision: A :class:`~hirobot.geometry.collision.CollisionCheck` between child
            parts, run before anything is sent.
        own: With ``parts=``: the node's own body (with its hands and cameras),
            whose names stay unprefixed (a mobile base with an arm part). Its
            joints must be in ``embodiment`` (which is then required).
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
        parts: Optional[Mapping[str, Robot]] = None,
        placement: Optional[Mapping[str, Mapping[str, Any]]] = None,
        collision: Any = None,
        own: Optional[Robot] = None,
    ) -> None:
        self._node: Optional[NodeParts] = None
        if parts is not None:
            self._init_node(parts, cameras, embodiment, first_frame_timeout_s, microphones,
                            speakers, first_audio_timeout_s, placement, collision, own)
            return
        if body is None:
            raise ValueError("RobotAssembly needs a body, or parts=")
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

    def _init_node(self, parts, cameras, embodiment, first_frame_timeout_s, microphones,
                   speakers, first_audio_timeout_s, placement, collision, own=None) -> None:
        from hirobot.assembly.parts import check_part_name, merge_embodiments

        if not parts:
            raise ValueError("parts= needs at least one part")
        for m in parts:
            check_part_name(m)
        self.body = None
        self.end_effectors = {}
        self.cameras = dict(cameras or {})
        self.microphones = dict(microphones or {})
        self.speakers = dict(speakers or {})
        self._first_frame_timeout_s = first_frame_timeout_s
        self._first_audio_timeout_s = first_audio_timeout_s
        self._joint_ees, self._opening_ees = {}, {}
        if embodiment is None:
            from hirobot.core.embodiment import AudioDevice, Camera

            embodiment = merge_embodiments(
                "assembly", None, {m: r.embodiment for m, r in parts.items()},
                cameras=[Camera(name=n, width=0, height=0) for n in self.cameras],
                microphones=[AudioDevice(name=n) for n in self.microphones],
                speakers=[AudioDevice(name=n) for n in self.speakers])
        self._embodiment = embodiment
        self._depth_cameras = {c.name for c in embodiment.cameras if c.depth}
        self.accepts_parts = True
        self._node = NodeParts(self, parts, own, placement, collision)

    @property
    def parts(self) -> Mapping[str, Robot]:
        """Child part name -> robot (empty without child parts)."""
        return dict(self._node.children) if self._node is not None else {}

    def part(self, path: str) -> PartView:
        """A read-only view of part ``path`` (``"left"``, ``"cell/left"``).

        Raises:
            KeyError: No such part.
        """
        robot: Robot = self
        names = [p for p in path.split("/") if p]
        if not names:
            raise KeyError("part(): give a part path, e.g. 'left'")
        for i, name in enumerate(names):
            node = getattr(robot, "_node", None)
            if node is None or name not in node.children:
                have = list(node.children) if node is not None else []
                raise KeyError(f"no part {'/'.join(names[:i + 1])!r}; parts here: {have}")
            robot = node.children[name]
        return PartView("/".join(names), robot)

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    @property
    def action_modes(self):  # type: ignore[override]
        if self._node is not None:
            modes = None
            for r in self._node.robots.values():
                mine = tuple(r.action_modes)
                modes = mine if modes is None else tuple(m for m in modes if m in mine)
            return modes or ()
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
        if self._node is not None:
            floating = self._node.floating
            return tuple(self._node.robots[floating[0]].motion_spaces) if len(floating) == 1 else ()
        return self.body.motion_spaces

    @property
    def direct_joints(self):  # type: ignore[override]
        if self._node is not None:
            return tuple(self._node.out_name(m, j) for m, r in self._node.robots.items()
                         for j in r.direct_joints)
        if not self._joint_ees:
            return self.body.direct_joints
        ee = [n for h in self._joint_ees.values() for n in h.joint_names]
        return tuple(self.body.direct_joints) + tuple(ee)

    @property
    def cartesian_targets(self):  # type: ignore[override]
        if self._node is not None:
            # A moving part whose position is unknown now takes no node-frame targets.
            return tuple(self._node.out_name(m, e) for m, r in self._node.robots.items()
                         if not self._node.lost(m) for e in r.cartesian_targets)
        return self.body.cartesian_targets

    @property
    def fault(self):  # type: ignore[override]
        if self._node is not None:
            return self._fault  # a part's fault stops the node at its next call
        # A layer under the assembly (a whole-body controller) that damped itself.
        return self._fault or self.body.fault

    def _idle_step(self) -> None:
        if self._node is not None:
            for r in self._node.robots.values():
                r._idle_step()
            return
        self.body._idle_step()

    def now(self) -> float:
        if self._node is not None:
            robots = list(self._node.robots.values())
            if all(r.simulated for r in robots):
                return robots[0].now()  # every part simulated: the first one's time
            return time.monotonic()
        return self.body.now()

    # The sim flag and outcome come from the body. The base class declares them,
    # so ``__getattr__`` would never be reached for them.
    @property
    def simulated(self) -> bool:  # type: ignore[override]
        if self._node is not None:
            return all(r.simulated for r in self._node.robots.values())
        return bool(getattr(self.body, "simulated", False))

    # A node with several parts has no task of its own: no reward or outcome.
    @property
    def reward(self):  # type: ignore[override]
        return None if self._node is not None else getattr(self.body, "reward", None)

    @property
    def success(self):  # type: ignore[override]
        return None if self._node is not None else getattr(self.body, "success", None)

    @property
    def done(self):  # type: ignore[override]
        return None if self._node is not None else getattr(self.body, "done", None)

    def __getattr__(self, name: str) -> Any:
        # Driver extras (raw_state, motion_mode, ...) come from the
        # body. Only reached for attributes not found on the assembly itself.
        if name in ("body", "end_effectors", "_node") or name.startswith("__"):
            raise AttributeError(name)
        body = self.__dict__.get("body")
        if body is None:
            raise AttributeError(f"{type(self).__name__} with parts has no attribute {name!r}")
        return getattr(body, name)

    # -------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        # The body first: on Unitree it initialises DDS on the chosen NIC, which
        # the videohub camera and the Dex3 hands then reuse.
        token = self._stop_token()
        started: List[Any] = []
        if self._node is not None:
            try:
                for robot in self._node.robots.values():
                    self._check_stop(token)
                    robot.connect()
                    started.append(robot.disconnect)
            except BaseException:
                for undo in reversed(started):
                    undo()
                raise
        else:
            self.body.connect()
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
            if self.body is not None:
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
        if self._node is not None:
            self._each_part("reset", lambda r: (self._check_stop(token), r.reset(seed)))
            return
        self.body.reset(seed)
        for ee in self.end_effectors.values():
            self._check_stop(token)
            ee.reset()

    def hold_parts(self, parts: Any) -> None:
        """Hold only ``parts`` (part paths, as in ``claim(parts=)``); the others keep going."""
        self._for_parts(parts, "hold", lambda r: r.hold())
        self._forget_joints(parts)

    def reset_parts(self, parts: Any, seed: Optional[int] = None) -> None:
        """Reset only ``parts``. The node's own fault state stays (a whole reset clears it)."""
        token = self._stop_token()
        self._for_parts(parts, "reset", lambda r: (self._check_stop(token), r.reset(seed)))
        self._forget_joints(parts)

    def _for_parts(self, parts: Any, what: str, call: Any) -> None:
        """``call`` on the robots ``parts`` cover, going down the tree (first error raised).

        A path naming a child node covers that child; a deeper path is passed
        on to the child in its own names; a path naming one of this node's own
        devices covers the node's own robot.
        """
        from hirobot.assembly.parts import resolve_part

        if self._node is None:
            raise ValueError(f"{what}_parts: this robot has no parts")
        todo: Dict[str, Optional[List[str]]] = {}  # part -> None (all of it) or paths inside
        for raw in ([parts] if isinstance(parts, str) else parts):
            path = str(raw).strip("/")
            if not path:
                todo = {m: None for m in self._node.robots}
                break
            head, _, rest = path.partition("/")
            if head in self._node.children:
                if not rest:
                    todo[head] = None
                elif todo.get(head, []) is not None:
                    todo.setdefault(head, []).append(rest)
                continue
            resolve_part(self.embodiment, path)  # one of this node's own devices, else raises
            todo[OWN] = None
        first: Optional[BaseException] = None
        for m, inner in todo.items():
            robot = self._node.robots[m]
            try:
                if inner is None or getattr(robot, "_node", None) is None:
                    call(robot)  # the whole part (a single-body part is all its own devices)
                else:
                    robot._for_parts(inner, what, call)
            except Exception as exc:  # noqa: BLE001 - every part gets the call
                logger.exception("%s on part %r failed", what, m)
                first = first or exc
        if first is not None:
            raise first

    def _forget_joints(self, parts: Any) -> None:
        """Drop the gate's step reference on ``parts``' joints only."""
        from hirobot.assembly.parts import scope_joints

        forget = getattr(self.safety, "forget", None)
        joints = scope_joints(self.embodiment, [parts] if isinstance(parts, str) else parts)
        if forget is not None and joints:
            forget(joints)

    def hold(self) -> None:
        if self._node is not None:
            self._each_part("hold", lambda r: r.hold())
            return
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
        if self._node is not None:
            obs, _ = self._node.observe()
            self._read_devices(obs, self.now() if self.simulated else None)
            return obs
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
        self._read_devices(obs, sim_now)
        return obs

    def _read_devices(self, obs: Observation, sim_now: Optional[float]) -> None:
        """This node's own cameras into ``images`` / ``depth`` / ``stamps``."""
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

    def _split(self, values: Optional[np.ndarray], idx: np.ndarray) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values, dtype=float)
        return arr[idx] if arr.ndim == 1 else arr  # a scalar gain applies to every part

    # ------------------------------------------------------------ child parts

    def _send_checked(self, action: Action) -> None:
        if self._node is None:
            super()._send_checked(action)
            return
        # Split and check every part first: if one refuses, nothing is sent.
        pieces = self._node.check(self._node.split(action))
        self._send_node_checked(pieces, action)

    def _send_node_checked(self, pieces: Mapping[str, Any],
                           action: Optional[Action] = None) -> None:
        """Send pieces that :meth:`hirobot.assembly.node.NodeParts.check` passed (here or in a parent)."""
        from hirobot.core.robot import RobotFaultError

        with self._commanding():
            if self.fault is not None:
                raise RobotFaultError(f"{type(self).__name__} is in a fault state "
                                      f"({self.fault}); call reset() first")
            self._node.send(pieces)  # type: ignore[union-attr]
            commit = getattr(self.safety, "commit", None)
            if commit is not None and action is not None:
                commit(action)

    def _fail_parts(self, culprit: str, reason: str) -> None:
        """A part failed: hold the others, enter the fault state."""
        logger.error("assembly: %s; holding the other parts", reason)
        for m, robot in self._node.robots.items():  # type: ignore[union-attr]
            if m == culprit:
                continue
            try:
                robot.hold()
            except Exception:  # noqa: BLE001 - hold every part
                logger.exception("hold on part %r failed", m)
        self._set_fault(reason)

    def _each_part(self, what: str, call: Any) -> None:
        """``call(part)`` for every part, all of them even if one raises; the first error is raised."""
        first: Optional[BaseException] = None
        for m, robot in self._node.robots.items():  # type: ignore[union-attr]
            try:
                call(robot)
            except Exception as exc:  # noqa: BLE001 - every part gets the call
                logger.exception("%s on part %r failed", what, m)
                first = first or exc
        if first is not None:
            raise first

    def _send_action(self, action: Action) -> None:
        # The safety gate already ran in send_action on this wrapper.
        if self._node is not None:  # reached only through Robot.hold()'s fallback
            self._node.send(self._node.check(self._node.split(action)))
            return
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
    def has_onboard_skills(self) -> bool:
        if self._node is not None:
            return any(r.has_onboard_skills for r in self._node.robots.values())
        return self.body.has_onboard_skills

    def list_onboard_skills(self) -> List[RobotSkillSpec]:
        if self._node is not None:
            return [s.model_copy(update={"name": self._node.out_name(m, s.name)})
                    for m, r in self._node.robots.items() for s in r.list_onboard_skills()]
        return self.body.list_onboard_skills()

    def run_onboard_skill(
        self, name: str, args: Optional[Mapping[str, Any]] = None
    ) -> RobotSkillResult:
        if self._node is not None:
            return Robot.run_onboard_skill(self, name, args)
        return self.body.run_onboard_skill(name, args)

    def _run_onboard_skill(self, spec: RobotSkillSpec, args: Dict[str, Any]) -> None:
        from hirobot.core.robot_skills import RobotSkillError

        if self.fault is not None:
            raise RobotSkillError(f"assembly is in a fault state ({self.fault}); reset() first")
        m, local = self._node.owner_of(spec.name, "onboard skill")  # type: ignore[union-attr]
        self._node.robots[m].run_onboard_skill(local, args)  # type: ignore[union-attr]

    def stop_onboard_skill(self) -> None:
        if self._node is not None:
            self._each_part("stop_onboard_skill", lambda r: r.stop_onboard_skill())
            return
        self.body.stop_onboard_skill()

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if self._node is not None:
            for m, r in self._node.robots.items():
                r.set_gains(self._node._slice(kp, m), self._node._slice(kd, m))
            return
        if not self._joint_ees:
            self.body.set_gains(kp, kd)
            return
        self.body.set_gains(self._split(kp, self._body_idx), self._split(kd, self._body_idx))
        for key, ee in self._joint_ees.items():
            idx = self._ee_idx[key]
            ee.set_gains(self._split(kp, idx), self._split(kd, idx))

    def get_status(self) -> RobotStatus:
        if self._node is not None:
            return self._node_status()
        status = self.body.get_status()
        status.fault = self.fault
        temps = status.joint_temperature
        if self._joint_ees and temps is not None:
            # End-effector joints are not covered by the body's status; mark them unknown.
            full = np.full(self._embodiment.num_dofs, np.nan)
            full[self._body_idx] = temps
            status.joint_temperature = full
        return status

    def _node_status(self) -> RobotStatus:
        """Parts' status merged: joint temperatures concatenated, keys prefixed.

        ``battery`` and ``command`` stay top-level when exactly one part reports
        one, else they go to ``extra["<part>/battery"]`` / ``extra["<part>/command"]``.
        ``fault`` is the assembly's own, or
        the first part fault.
        """
        node = self._node
        statuses = {m: r.get_status() for m, r in node.robots.items()}
        out = RobotStatus(timestamp=time.time())
        if any(s.joint_temperature is not None for s in statuses.values()):
            temps = np.full(self._embodiment.num_dofs, np.nan)
            for m, s in statuses.items():
                if s.joint_temperature is not None:
                    temps[node.ranges[m]] = s.joint_temperature
            out.joint_temperature = temps
        with_battery = [m for m, s in statuses.items() if s.battery is not None]
        with_command = [m for m, s in statuses.items() if s.command is not None]
        for m, s in statuses.items():
            name = lambda k, m=m: node.out_name(m, k)  # noqa: E731
            out.joint_faults.update({name(k): v for k, v in s.joint_faults.items()})
            out.board_temperature.update({name(k): v for k, v in s.board_temperature.items()})
            out.extra.update({name(k): v for k, v in s.extra.items()})
            # A part's own audit and sim-check counters stay under its name.
            out.extra.update({name(f"safety/{k}"): v for k, v in s.safety.items()})
            if s.battery is not None:
                if len(with_battery) == 1:
                    out.battery = s.battery
                else:
                    out.extra[name("battery")] = s.battery
            if s.command is not None:
                if len(with_command) == 1:
                    out.command = s.command
                else:
                    out.extra[name("command")] = s.command
        out.fault = self._fault or next(
            (f"part {m!r} fault: {s.fault}" for m, s in statuses.items() if s.fault), None)
        return out

    def damp(self) -> None:
        """Damp the body (if its driver can) and every end-effector."""
        if self._node is not None:
            self._each_part("damp", lambda r: getattr(r, "damp", lambda: None)())
            return
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
        parts = (list(self._node.robots.values()) if self._node is not None
                 else [self.body, *self.end_effectors.values()])
        for part in parts:
            try:
                part.estop()
            except Exception as exc:  # noqa: BLE001 - stop every part
                logger.exception("estop on %s failed", getattr(part, "name", type(part).__name__))
                first = first or exc
        if first is not None:
            raise first

    def disconnect(self) -> None:
        if self._node is not None:
            try:
                self._each_part("disconnect", lambda r: r.disconnect())
            finally:
                self._close_devices()
            return
        try:
            self.body.disconnect()  # stop the robot first
        finally:
            for ee in self.end_effectors.values():
                try:
                    ee.disconnect()
                except Exception:  # noqa: BLE001 - damp the others regardless
                    logger.exception("disconnecting end-effector %s failed", ee.name)
            self._close_devices()

    def _close_devices(self) -> None:
        for cam in self.cameras.values():
            cam.stop()
        for dev in (*self.speakers.values(), *self.microphones.values()):
            try:
                dev.close()
            except Exception:  # noqa: BLE001 - close the other devices regardless
                logger.exception("closing audio device %s failed", dev.name)
