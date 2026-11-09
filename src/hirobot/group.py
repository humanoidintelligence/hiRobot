"""Several robots driven as one: :class:`RobotGroup`.

A group holds complete robots (each built by ``make_robot``, with its own hands,
cameras, sim twin, layers and safety gate) plus scene cameras that belong to
none of them, and is itself a :class:`~hirobot.robot.Robot`. A VLA or hiAgent's
control loop drives it like a single robot.

**Names.** Every joint, end-effector, camera, microphone and speaker of a member
appears in the group's embodiment as ``<member>/<name>`` (``left/joint1``,
``right/tcp``). Scene cameras keep their own names.

**Observations** concatenate the members' joint vectors in member order and
prefix every keyed field (``ee_poses``, ``gripper``, ``images``, ``depth``,
``stamps``, ``extra``): ``stamps["left/body"]`` is when member ``left``'s body
state arrived. Scene cameras' stamps keep the camera's own name. With exactly one
floating-base member its base signals stay in the top-level fields; with more
they go to ``extra["<member>/base_*"]``.

**Frames.** With ``base_poses`` the group's ``ee_poses`` and ``ee_targets``
are in the world frame; members keep their own. See :class:`RobotGroup`.

**Actions** are flat (one mode for every member) or per member
(``Action.members``: member name -> its own action, own names, own mode). In
a flat action ``joint_target`` / ``kp`` /
``kd`` are split by the members' joint counts (or by name with
``joint_names``), ``ee_targets`` / ``gripper`` by name prefix. ``base_velocity``
/ ``base_target`` / ``latent`` / ``motion`` go to the one floating-base member.
A member the action does not mention gets nothing and keeps its last target;
its simulation (if it has one) still advances one period (``idle_step``).

**Checks before sending.** Every member's checks (fault state, its gate, its
``direct_joints``) run on its part first; if any refuses, nothing is sent to
any member. Then the parts go out one member after another.

**Errors.** When a member fails to read or send, or reports a fault (it damped
itself), the group holds every other member, enters its own fault state and
re-raises. It refuses commands until :meth:`RobotGroup.reset`.

**Ownership.** A robot in a group refuses ``send_action`` / ``run_robot_skill``
calls that do not come through the group, so a caller holding the member's
reference cannot skip the group's checks. The mark is on the object the group
holds, not the robots inside it: a whole-body-control layer's own thread keeps
commanding its inner robot.
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from hirobot.embodiment import Camera, Embodiment, EmbodimentKind
from hirobot.robot import Robot, RobotFaultError
from hirobot.robot_skills import RobotSkillError, RobotSkillSpec
from hirobot.safety import SafetyError
from hirobot.stamps import Stamp
from hirobot.transforms import (
    IDENTITY_POSE,
    pose_compose,
    pose_inverse,
    quat_conjugate,
    rotate,
)
from hirobot.types import JOINT_MODES, Action, ControlMode, Observation, RobotStatus

if TYPE_CHECKING:
    from hirobot.camera import CameraDriver
    from hirobot.collision import CollisionCheck

logger = logging.getLogger(__name__)

SEP = "/"

#: Observation fields of a floating base.
_BASE_FIELDS = ("base_orientation", "base_angular_velocity", "base_linear_acceleration",
                "base_position", "base_linear_velocity")
#: Action fields that only a floating-base member can take.
_BASE_COMMANDS = ("base_velocity", "base_target", "latent", "motion")


def _split_name(name: str) -> Tuple[str, str]:
    member, sep, local = name.partition(SEP)
    if not sep or not member or not local:
        raise SafetyError(f"{name!r} is not a member name: expected '<member>{SEP}<name>'")
    return member, local


def group_embodiment(name: str, members: Mapping[str, Embodiment],
                     cameras: Sequence[Camera] = (),
                     control_hz: Optional[int] = None,
                     expose: Optional[Mapping[str, Mapping[str, str]]] = None) -> Embodiment:
    """The embodiment of a group: the members' parts prefixed, plus scene cameras.

    Hardware specs (``driver:``) are dropped: the members own
    their devices. ``floating_base`` is true when exactly one member has a
    floating base (its base signals are the group's top-level base fields).
    ``control_hz`` defaults to the slowest member's. ``expose`` renames
    end-effectors (``{"ee": {"left/ee": "left_arm"}}``) and cameras
    (``{"cameras": {"scene": "agentview"}}``); see :class:`RobotGroup`.
    """
    expose = _check_expose(expose, members, cameras)
    ee_names, cam_names = expose.get("ee", {}), expose.get("cameras", {})
    if not members:
        raise ValueError("a robot group needs at least one member")
    for m in members:
        if not m or SEP in m:
            raise ValueError(f"member name {m!r} must be non-empty and contain no {SEP!r}")

    def pre(m: str, n: str) -> str:
        return f"{m}{SEP}{n}"

    joints, ees, cams, mics, spks = [], [], [], [], []
    for m, emb in members.items():
        joints += [j.model_copy(update={"name": pre(m, j.name)}) for j in emb.joints]
        ees += [ee.model_copy(update={"name": ee_names.get(pre(m, ee.name), pre(m, ee.name)),
                                      "driver": None,
                                      "joints": [pre(m, j) for j in ee.joints]})
                for ee in emb.end_effectors]
        cams += [c.model_copy(update={"name": cam_names.get(pre(m, c.name), pre(m, c.name)),
                                      "driver": None})
                 for c in emb.cameras]
        mics += [d.model_copy(update={"name": pre(m, d.name), "driver": None})
                 for d in emb.microphones]
        spks += [d.model_copy(update={"name": pre(m, d.name), "driver": None})
                 for d in emb.speakers]
    taken = {c.name for c in cams}
    for c in cameras:
        exposed = cam_names.get(c.name, c.name)
        if exposed in taken:
            raise ValueError(f"scene camera {exposed!r} clashes with a member camera")
        taken.add(exposed)
        cams.append(c.model_copy(update={"name": exposed}))
    floating = [m for m, e in members.items() if e.floating_base]
    humanoid = any(e.kind == EmbodimentKind.HUMANOID for e in members.values())
    return Embodiment(
        name=name,
        kind=EmbodimentKind.HUMANOID if humanoid else EmbodimentKind.ARM,
        control_hz=control_hz or min(e.control_hz for e in members.values()),
        joints=joints,
        floating_base=len(floating) == 1,
        end_effectors=ees,
        cameras=cams,
        microphones=mics,
        speakers=spks,
    )


def _check_expose(expose: Optional[Mapping[str, Mapping[str, str]]],
                  members: Mapping[str, Embodiment],
                  cameras: Sequence[Camera]) -> Dict[str, Dict[str, str]]:
    """Validate an ``expose:`` mapping: known names only, no clashes after renaming."""
    out = {k: dict(v or {}) for k, v in dict(expose or {}).items()}
    unknown_kinds = sorted(set(out) - {"ee", "cameras"})
    if unknown_kinds:
        raise ValueError(f"expose: unknown section(s) {unknown_kinds}; known: ['cameras', 'ee']")
    ees = [f"{m}{SEP}{ee.name}" for m, e in members.items() for ee in e.end_effectors]
    cams = [f"{m}{SEP}{c.name}" for m, e in members.items() for c in e.cameras]
    cams += [c.name for c in cameras]
    for kind, names in (("ee", ees), ("cameras", cams)):
        renames = out.get(kind, {})
        unknown = sorted(set(renames) - set(names))
        if unknown:
            raise ValueError(f"expose.{kind}: no such name(s) {unknown}; have {names}")
        final = [renames.get(n, n) for n in names]
        clash = sorted({n for n in final if final.count(n) > 1})
        if clash:
            raise ValueError(f"expose.{kind}: name(s) {clash} used more than once after renaming")
    return out


class MemberView:
    """Read-only access to one member of a group: it cannot be commanded through this."""

    def __init__(self, name: str, robot: Robot) -> None:
        self.name = name
        self._robot = robot

    @property
    def embodiment(self) -> Embodiment:
        return self._robot.embodiment

    @property
    def simulated(self) -> bool:
        return bool(self._robot.simulated)

    @property
    def fault(self) -> Optional[str]:
        return self._robot.fault

    def get_observation(self) -> Observation:
        return self._robot.get_observation()

    def get_status(self) -> RobotStatus:
        return self._robot.get_status()


class RobotGroup(Robot):
    """Several complete robots plus scene cameras, driven as one robot.

    Args:
        members: Member name -> robot, in the order their joints are
            concatenated. Each is a whole single-robot stack (``make_robot``).
        cameras: Scene camera name -> camera driver, for cameras no member owns.
        name: The group's embodiment name.
        control_hz: The group's control rate; defaults to the slowest member's.
            A member slower than this logs a warning (it cannot keep up).
        scene_cameras: Embodiment entries (size) of ``cameras``; a camera
            without one is listed as 0 x 0.
        first_frame_timeout_s: How long :meth:`connect` waits for each scene
            camera's first frame; a miss only logs a warning.
        skew_warn_s: Log a warning when the members' read times in one
            :meth:`get_observation` differ by more than this; default half a
            control period.
        base_poses: Member name -> its base in the world frame,
            ``[x, y, z, qx, qy, qz, qw]``. A fixed-base member without one sits
            at the world origin. A floating-base member without one keeps its
            own (body) frame; with one, it is the origin of its odometry, and
            its poses are converted with its current ``base_position`` /
            ``base_orientation`` (no world frame while it reports no
            ``base_position``).
        expose: Renames: ``{"ee": {"left/ee": "left_arm"}, "cameras":
            {"scene": "agentview"}}``. Exposed names are used everywhere
            (embodiment, observation, actions); joints keep ``<member>/<name>``.
        collision: Distance check between members
            (:class:`~hirobot.collision.CollisionCheck`), run with the other
            checks before anything is sent. Every member it models must have a
            world frame: a floating-base one needs a ``base_pose``.
    """

    accepts_members = True

    def __init__(
        self,
        members: Mapping[str, Robot],
        cameras: Optional[Mapping[str, "CameraDriver"]] = None,
        name: str = "robot_group",
        control_hz: Optional[int] = None,
        scene_cameras: Sequence[Camera] = (),
        first_frame_timeout_s: float = 3.0,
        skew_warn_s: Optional[float] = None,
        base_poses: Optional[Mapping[str, Sequence[float]]] = None,
        expose: Optional[Mapping[str, Mapping[str, str]]] = None,
        collision: Optional["CollisionCheck"] = None,
    ) -> None:
        self.members: Dict[str, Robot] = dict(members)
        self.cameras: Dict[str, "CameraDriver"] = dict(cameras or {})
        declared = {c.name: c for c in scene_cameras}
        unknown = sorted(set(declared) - set(self.cameras))
        if unknown:
            raise ValueError(f"scene_cameras {unknown} have no camera driver")
        scene = [declared.get(n) or Camera(name=n, width=0, height=0) for n in self.cameras]
        self._embodiment = group_embodiment(
            name, {m: r.embodiment for m, r in self.members.items()}, scene, control_hz, expose)
        self.base_poses: Dict[str, np.ndarray] = {}
        for m, pose in dict(base_poses or {}).items():
            if m not in self.members:
                raise ValueError(f"base_poses: no member {m!r}; members: {list(self.members)}")
            pose = np.asarray(pose, dtype=float)
            if pose.shape != (7,) or not np.all(np.isfinite(pose)):
                raise ValueError(f"base_poses[{m!r}] must be a finite [x, y, z, qx, qy, qz, qw]")
            self.base_poses[m] = np.concatenate([pose[:3], pose[3:] / np.linalg.norm(pose[3:])])
        floating_own = [m for m, r in self.members.items()
                        if r.embodiment.floating_base and m not in self.base_poses]
        if self.base_poses and floating_own:
            logger.warning("members %s have no base_pose: their ee_poses stay in their own "
                           "frame while the others are in the world frame", floating_own)
        # Exposed name <-> (member, local name), for end-effectors and cameras.
        ee_rename = dict((expose or {}).get("ee") or {})
        cam_rename = dict((expose or {}).get("cameras") or {})
        self._ee_out: Dict[Tuple[str, str], str] = {}
        self._ee_in: Dict[str, Tuple[str, str]] = {}
        self._cam_out: Dict[Tuple[str, str], str] = {}
        for m, r in self.members.items():
            for ee in r.embodiment.end_effectors:
                full = f"{m}{SEP}{ee.name}"
                self._ee_out[(m, ee.name)] = ee_rename.get(full, full)
                self._ee_in[ee_rename.get(full, full)] = (m, ee.name)
            for c in r.embodiment.cameras:
                full = f"{m}{SEP}{c.name}"
                self._cam_out[(m, c.name)] = cam_rename.get(full, full)
        self._scene_out = {n: cam_rename.get(n, n) for n in self.cameras}
        # Last base pose (odometry frame) seen per floating member, for frames.
        self._odom: Dict[str, Optional[np.ndarray]] = {}
        if collision is not None:
            unknown = sorted(set(collision.models) - set(self.members))
            if unknown:
                raise ValueError(f"collision: no member(s) {unknown}; members: {list(self.members)}")
            no_frame = [m for m in collision.models
                        if self.members[m].embodiment.floating_base and m not in self.base_poses]
            if no_frame:
                raise ValueError(f"collision: floating-base member(s) {no_frame} need a base_pose "
                                 "to be placed in the world")
        self.collision = collision
        for m, robot in self.members.items():
            if robot._group is not None and robot._group is not self:
                raise ValueError(f"member {m!r} already belongs to another RobotGroup")
            hz = robot.embodiment.control_hz
            if hz and hz < self._embodiment.control_hz:
                logger.warning("member %r runs at %s Hz, slower than the group's %s Hz",
                               m, hz, self._embodiment.control_hz)
        for robot in self.members.values():
            robot._group = self
        self._first_frame_timeout_s = first_frame_timeout_s
        period = 1.0 / self._embodiment.control_hz
        self._skew_warn_s = period / 2 if skew_warn_s is None else skew_warn_s
        self._last_skew_warning = -1e9
        self._local = threading.local()
        # Where each member's joints sit in the group's joint vector.
        self._ranges: Dict[str, slice] = {}
        start = 0
        for m, robot in self.members.items():
            n = robot.embodiment.num_dofs
            self._ranges[m] = slice(start, start + n)
            start += n
        self._floating = [m for m, r in self.members.items() if r.embodiment.floating_base]

    # ------------------------------------------------------------ structure

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def member(self, name: str) -> MemberView:
        """A read-only view of member ``name`` (embodiment, observation, status)."""
        if name not in self.members:
            raise KeyError(f"no member {name!r}; members: {list(self.members)}")
        return MemberView(name, self.members[name])

    @property
    def simulated(self) -> bool:  # type: ignore[override]
        # Any real member means the caller paces to real time.
        return all(bool(r.simulated) for r in self.members.values())

    def now(self) -> float:
        """The host clock with any real member; else the first member's simulated time.

        Simulated members advance together (``idle_step``), so their clocks agree.
        """
        if not self.simulated:
            return time.monotonic()
        return next(iter(self.members.values())).now()

    @property
    def action_modes(self):  # type: ignore[override]
        robots = list(self.members.values())
        common = [m for m in robots[0].action_modes
                  if all(m in r.action_modes for r in robots[1:])]
        return tuple(common)

    @property
    def motion_spaces(self):  # type: ignore[override]
        # A flat MOTION_CLIP goes to the one floating-base member.
        if len(self._floating) != 1:
            return ()
        return tuple(self.members[self._floating[0]].motion_spaces)

    @property
    def direct_joints(self):  # type: ignore[override]
        return tuple(f"{m}{SEP}{n}" for m, r in self.members.items() for n in r.direct_joints)

    @property
    def cartesian_targets(self):  # type: ignore[override]
        # A floating member with a base_pose drops out while it reports no
        # base_position (no world frame for it).
        return tuple(self._ee_out[(m, n)] for m, r in self.members.items()
                     for n in r.cartesian_targets
                     if not (m in self._odom and self._odom[m] is None))

    # -------------------------------------------------------- calling members

    def _in_call(self) -> bool:
        """Whether this thread is inside a group call (members accept commands then)."""
        return getattr(self._local, "depth", 0) > 0

    @contextlib.contextmanager
    def _calling(self) -> Iterator[None]:
        self._local.depth = getattr(self._local, "depth", 0) + 1
        try:
            yield
        finally:
            self._local.depth -= 1

    def _fail(self, culprit: Optional[str], reason: str) -> None:
        """Hold every member but ``culprit`` and enter the fault state (``on_error: hold_all``)."""
        with self._calling():
            for m, robot in self.members.items():
                if m == culprit:
                    continue
                try:
                    robot.hold()
                except Exception:  # noqa: BLE001 - hold the others regardless
                    logger.exception("holding member %r after a failure of %r failed", m, culprit)
        self.set_fault(reason)
        logger.error("robot group %r stopped: %s", self._embodiment.name, reason)

    def _check_members(self) -> None:
        for m, robot in self.members.items():
            fault = robot.fault
            if fault is not None:
                reason = f"member {m!r} fault: {fault}"
                self._fail(m, reason)
                raise RobotFaultError(reason)

    # -------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        done: List[Any] = []
        try:
            for robot in self.members.values():
                robot.connect()
                done.append(robot.disconnect)
            for cam in self.cameras.values():
                cam.start()
                done.append(cam.stop)
        except BaseException:
            for undo in reversed(done):
                try:
                    undo()
                except Exception:  # noqa: BLE001 - undo the rest regardless
                    logger.exception("undoing a partial group connect failed")
            raise
        for cam in self.cameras.values():
            wait = getattr(cam, "wait_for_frame", None)
            if wait is None:
                continue
            try:
                wait(self._first_frame_timeout_s)
            except TimeoutError as exc:
                logger.warning("%s; continuing without it until frames arrive", exc)

    def reset(self, seed: Optional[int] = None) -> None:
        """Reset every member, then leave the fault state (unless an e-stop came meanwhile)."""
        token = self._stop_token()
        with self._calling():
            for robot in self.members.values():
                self._check_stop(token)
                robot.reset(seed)

    def hold(self) -> None:
        self._each("hold", lambda r: r.hold())

    def damp(self) -> None:
        """Damp every member that can (an e-stop); applies whatever the fault state."""
        def damp(robot: Robot) -> None:
            fn = getattr(robot, "damp", None)
            if fn is not None:
                fn()
        self._each("damp", damp)

    def _estop(self) -> None:
        """E-stop every member; one failing does not keep the rest from stopping."""
        self._each("estop", lambda r: r.estop())

    def disconnect(self) -> None:
        try:
            self._each("disconnect", lambda r: r.disconnect())
        finally:
            for cam in self.cameras.values():
                cam.stop()

    def _each(self, what: str, fn) -> None:
        """Call ``fn`` on every member; carry on past failures, then raise the first."""
        first: Optional[BaseException] = None
        with self._calling():
            for m, robot in self.members.items():
                try:
                    fn(robot)
                except Exception as exc:  # noqa: BLE001 - reach every member
                    logger.exception("%s on member %r failed", what, m)
                    first = first or exc
        if first is not None:
            raise first

    def idle_step(self) -> None:
        for robot in self.members.values():
            robot.idle_step()

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        for m, robot in self.members.items():
            robot.set_gains(self._slice(kp, m), self._slice(kd, m))

    # ----------------------------------------------------------------- frames

    def _world_from_member(self, m: str) -> Optional[np.ndarray]:
        """Pose of member ``m``'s command frame in the world, or ``None`` if unknown.

        Identity for a member without a ``base_pose`` (its own frame is used).
        """
        base = self.base_poses.get(m)
        if base is None:
            return IDENTITY_POSE
        if not self.members[m].embodiment.floating_base:
            return base
        if m not in self._odom:
            self._remember_odom(m, self.members[m].get_observation())
        odom = self._odom[m]
        return None if odom is None else pose_compose(base, odom)

    def _remember_odom(self, m: str, obs: Observation) -> None:
        # Only a floating member placed in the world needs its odometry.
        if m not in self.base_poses or not self.members[m].embodiment.floating_base:
            return
        if obs.base_position is None or obs.base_orientation is None:
            self._odom[m] = None
        else:
            self._odom[m] = np.concatenate([np.asarray(obs.base_position, dtype=float)[:3],
                                            np.asarray(obs.base_orientation, dtype=float)])

    def _targets_to_member(self, m: str, mode: ControlMode,
                           targets: Mapping[str, Any]) -> Dict[str, np.ndarray]:
        """World-frame ``ee_targets`` of member ``m`` converted to its own frame."""
        if not targets:
            return {}
        frame = self._world_from_member(m)
        if frame is None:
            raise SafetyError(
                f"member {m!r} reports no base_position, so it has no world frame for "
                "ee_targets; give it targets in its own frame by leaving out its base_pose")
        if np.array_equal(frame, IDENTITY_POSE):
            return {k: np.asarray(v, dtype=float) for k, v in targets.items()}
        inv = pose_inverse(frame)
        out = {}
        for name, value in targets.items():
            v = np.asarray(value, dtype=float)
            if mode == ControlMode.CARTESIAN_DELTA:
                if v.shape != (6,):
                    raise SafetyError(f"CARTESIAN_DELTA target {name!r} must be 6 values, "
                                      f"got shape {v.shape}")
                q = quat_conjugate(frame[3:])
                out[name] = np.concatenate([rotate(q, v[:3]), rotate(q, v[3:])])
            else:
                if v.shape != (7,):
                    raise SafetyError(f"ee target {name!r} must be a 7-vector pose, "
                                      f"got shape {v.shape}")
                out[name] = pose_compose(inv, v)
        return out

    # ------------------------------------------------------------ observation

    def get_observation(self) -> Observation:
        parts: Dict[str, Observation] = {}
        read_at: Dict[str, float] = {}
        for m, robot in self.members.items():
            try:
                parts[m] = robot.get_observation()
            except Exception as exc:
                self._fail(m, f"member {m!r} get_observation failed: {exc!r}")
                raise
            read_at[m] = time.monotonic()
            self._remember_odom(m, parts[m])
        obs = Observation()
        n = self._embodiment.num_dofs
        for field in ("joint_pos", "joint_vel", "joint_torque"):
            if all(getattr(p, field) is None for p in parts.values()):
                continue
            full = np.full(n, np.nan)
            for m, p in parts.items():
                value = getattr(p, field)
                if value is not None:
                    full[self._ranges[m]] = value
            setattr(obs, field, full)
        for m, p in parts.items():
            pre = f"{m}{SEP}"
            if len(self._floating) == 1 and m == self._floating[0]:
                for field in _BASE_FIELDS:
                    setattr(obs, field, getattr(p, field))
            else:
                for field in _BASE_FIELDS:
                    if getattr(p, field) is not None:
                        obs.extra[pre + field] = getattr(p, field)
            frame = self._world_from_member(m)
            if frame is None:
                # No world frame for it right now: own-frame poses under extra.
                if p.ee_poses:
                    obs.extra[pre + "ee_poses"] = dict(p.ee_poses)
            else:
                identity = np.array_equal(frame, IDENTITY_POSE)
                for k, v in p.ee_poses.items():
                    name = self._ee_out.get((m, k), pre + k)
                    obs.ee_poses[name] = (np.asarray(v, dtype=float) if identity
                                          else pose_compose(frame, v))
            for k, v in p.gripper.items():
                obs.gripper[self._ee_out.get((m, k), pre + k)] = v
            for k, v in p.images.items():
                obs.images[self._cam_out.get((m, k), pre + k)] = v
            for k, v in p.depth.items():
                obs.depth[self._cam_out.get((m, k), pre + k)] = v
            for k, v in p.stamps.items():
                obs.stamps[self._cam_out.get((m, k)) or self._ee_out.get((m, k), pre + k)] = v
            obs.extra.update({pre + k: v for k, v in p.extra.items()})
            if obs.instruction is None:
                obs.instruction = p.instruction
        for name, cam in self.cameras.items():
            frame = cam.latest()
            if frame is not None:
                out = self._scene_out[name]
                image, stamp = frame
                obs.images[out] = image
                # A simulated group's clock is simulated time; restamp on it.
                obs.stamps[out] = Stamp(self.now(), stamp.seq) if self.simulated else stamp
        skew = max(read_at.values()) - min(read_at.values())
        if skew > self._skew_warn_s and time.monotonic() - self._last_skew_warning > 5.0:
            self._last_skew_warning = time.monotonic()
            logger.warning("robot group read its members %.1f ms apart (more than %.1f ms)",
                           skew * 1e3, self._skew_warn_s * 1e3)
        return obs

    def get_status(self) -> RobotStatus:
        """Members' status merged: joint temperatures concatenated, keys prefixed.

        ``battery`` stays top-level when exactly one member reports one, else it
        goes to ``extra["<member>/battery"]``. ``fault`` is the group's own, or
        the first member fault.
        """
        statuses = {m: r.get_status() for m, r in self.members.items()}
        out = RobotStatus(timestamp=time.time())
        if any(s.joint_temperature is not None for s in statuses.values()):
            temps = np.full(self._embodiment.num_dofs, np.nan)
            for m, s in statuses.items():
                if s.joint_temperature is not None:
                    temps[self._ranges[m]] = s.joint_temperature
            out.joint_temperature = temps
        with_battery = [m for m, s in statuses.items() if s.battery is not None]
        for m, s in statuses.items():
            pre = f"{m}{SEP}"
            out.joint_faults.update({pre + k: v for k, v in s.joint_faults.items()})
            out.board_temperature.update({pre + k: v for k, v in s.board_temperature.items()})
            out.extra.update({pre + k: v for k, v in s.extra.items()})
            if s.battery is not None:
                if len(with_battery) == 1:
                    out.battery = s.battery
                else:
                    out.extra[pre + "battery"] = s.battery
        out.fault = self.fault or next(
            (f"member {m!r} fault: {s.fault}" for m, s in statuses.items() if s.fault), None)
        return out

    # ---------------------------------------------------------------- actions

    def _slice(self, values: Optional[np.ndarray], m: str) -> Optional[np.ndarray]:
        if values is None:
            return None
        arr = np.asarray(values, dtype=float)
        return arr[self._ranges[m]] if arr.ndim == 1 else arr  # a scalar applies to all

    def split(self, action: Action) -> Dict[str, Action]:
        """Each member's part of ``action``, in the member's own names and frame.

        A flat action is split by joint ranges and names (exposed names for
        end-effectors); ``Action.members`` is taken as given. World-frame
        ``ee_targets`` are converted to each member's frame (see
        :attr:`base_poses`); a member action with ``extra["frame"] == "base"``
        is already in the member's frame and is left alone.

        Raises:
            SafetyError: A name with no member, a vector of the wrong length,
                a base command with no single floating-base member to take it,
                or a world-frame target for a member with no world frame now.
        """
        if action.members:
            unknown = sorted(set(action.members) - set(self.members))
            if unknown:
                raise SafetyError(f"Action.members: no member(s) {unknown}; "
                                  f"members: {list(self.members)}")
            parts = {}
            for m, sub in action.members.items():
                if sub.members:
                    raise SafetyError(f"members[{m!r}] has members of its own")
                if sub.extra.get("frame") == "base":
                    parts[m] = sub
                else:
                    parts[m] = sub.model_copy(update={
                        "ee_targets": self._targets_to_member(m, sub.mode, sub.ee_targets)})
            if not parts:
                raise SafetyError("the action commands no member")
            return parts
        fields: Dict[str, Dict[str, Any]] = {m: {} for m in self.members}
        n = self._embodiment.num_dofs
        if action.joint_target is not None and action.joint_names is None:
            target = np.asarray(action.joint_target, dtype=float)
            if target.shape != (n,):
                raise SafetyError(f"joint_target has shape {target.shape}, the group has {n} joints")
            for m in self.members:
                fields[m]["joint_target"] = target[self._ranges[m]]
                fields[m]["kp"] = self._slice(action.kp, m)
                fields[m]["kd"] = self._slice(action.kd, m)
        elif action.joint_names is not None:
            target = np.asarray(action.joint_target, dtype=float)
            by_member: Dict[str, List[int]] = {}
            for i, name in enumerate(action.joint_names):
                m, _ = self._member_of(name, "joint")
                by_member.setdefault(m, []).append(i)
            for m, idx in by_member.items():
                ix = np.array(idx, dtype=int)
                fields[m]["joint_names"] = [_split_name(action.joint_names[i])[1] for i in idx]
                fields[m]["joint_target"] = target[ix]
                for gain in ("kp", "kd"):
                    g = getattr(action, gain)
                    if g is not None:
                        g = np.asarray(g, dtype=float)
                        fields[m][gain] = g[ix] if g.ndim == 1 else g
        for key in ("ee_targets", "gripper"):
            for name, value in getattr(action, key).items():
                m, local = self._ee_member(name)
                fields[m].setdefault(key, {})[local] = value
        base = [f for f in _BASE_COMMANDS if getattr(action, f) is not None]
        if base:
            if len(self._floating) != 1:
                raise SafetyError(
                    f"{base} need exactly one floating-base member to go to; this group has "
                    f"{self._floating or 'none'}; use Action.members")
            for f in base:
                fields[self._floating[0]][f] = getattr(action, f)
        parts = {}
        for m, f in fields.items():
            if not f:
                continue
            if "ee_targets" in f:
                f["ee_targets"] = self._targets_to_member(m, action.mode, f["ee_targets"])
            f.update(mode=action.mode, append=action.append, extra=dict(action.extra))
            parts[m] = Action(**f)
        if not parts:
            raise SafetyError("the action commands no member")
        return parts

    def _ee_member(self, name: str) -> Tuple[str, str]:
        """(member, local name) of an exposed end-effector name."""
        if name in self._ee_in:
            return self._ee_in[name]
        m, local = self._member_of(name, "end-effector")
        raise SafetyError(f"end-effector {name!r}: member {m!r} has no end-effector "
                          f"{local!r} under that name; group end-effectors: {sorted(self._ee_in)}")

    def _member_of(self, name: str, what: str) -> Tuple[str, str]:
        m, local = _split_name(name)
        if m not in self.members:
            raise SafetyError(f"{what} {name!r}: no member {m!r}; members: {list(self.members)}")
        return m, local

    def _send_action(self, action: Action) -> None:
        # 1. Split; 2. check every part (nothing is sent if any is refused);
        # 3. send them one after another. The group's own gate already ran.
        parts = self.split(action)
        self._check_members()
        checked: Dict[str, Action] = {}
        for m, part in parts.items():
            robot = self.members[m]
            if part.mode not in robot.action_modes:
                raise SafetyError(f"member {m!r} does not take mode {part.mode.value!r}; "
                                  f"it takes {[x.value for x in robot.action_modes]}")
            if part.mode not in JOINT_MODES and part.joint_target is not None \
                    and part.joint_names is None:
                raise SafetyError(f"member {m!r}: mode {part.mode.value!r} needs joint_names "
                                  "for joint targets")
            try:
                checked[m] = robot.check_action(part)
            except SafetyError as exc:
                raise SafetyError(f"member {m!r}: {exc}") from exc
        if self.collision is not None:
            self._check_collision(checked)
        with self._calling():
            for m, part in checked.items():
                try:
                    self.members[m]._send_checked(part)
                except Exception as exc:
                    self._fail(m, f"member {m!r} send failed: {exc!r}")
                    raise
            for m, robot in self.members.items():
                if m in checked:
                    continue
                try:
                    robot.idle_step()  # its simulation advances with the others
                except Exception as exc:
                    self._fail(m, f"member {m!r} idle_step failed: {exc!r}")
                    raise

    def _check_collision(self, checked: Mapping[str, Action]) -> None:
        """Refuse ``checked`` if it brings two members closer than allowed.

        The predicted gap (joint targets as the next angles, Cartesian target
        points as spheres) is compared with the current one; below
        ``min_distance`` the action passes only if the gap does not shrink.
        """
        check = self.collision
        now: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        nxt: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        points: Dict[str, np.ndarray] = {}
        for m, model in check.models.items():
            obs = self.members[m].get_observation()
            self._remember_odom(m, obs)
            frame = self._world_from_member(m)
            if frame is None or obs.joint_pos is None:
                raise SafetyError(f"collision check: member {m!r} has no world frame or joint "
                                  "positions now; nothing sent")
            q = np.asarray(obs.joint_pos, dtype=float)
            now[m] = model.segments(q, frame)
            part = checked.get(m)
            q_next = q
            if part is not None and part.mode in (ControlMode.POSITION, ControlMode.PD) \
                    and part.joint_target is not None:
                q_next = q.copy()
                target = np.asarray(part.joint_target, dtype=float)
                if part.joint_names is None:
                    q_next = target
                else:
                    names = self.members[m].embodiment.joint_names
                    for name, v in zip(part.joint_names, target):
                        q_next[names.index(name)] = v
            nxt[m] = model.segments(q_next, frame)
            if part is not None and part.mode == ControlMode.CARTESIAN and part.ee_targets:
                points[m] = np.array([pose_compose(frame, np.asarray(v, dtype=float))[:3]
                                      for v in part.ee_targets.values()])
        after = check.gap(nxt, points)
        if after is None or after.distance >= check.min_distance:
            return
        before = check.gap(now)
        if before is not None and after.distance >= before.distance - 1e-6:
            return  # already too close, and this does not make it worse
        raise SafetyError(
            f"collision check: {after.members[0]}/{after.bodies[0]} and "
            f"{after.members[1]}/{after.bodies[1]} would be {after.distance * 100:.1f} cm apart "
            f"(minimum {check.min_distance * 100:.1f} cm); nothing sent")

    # ----------------------------------------------------------------- skills

    @property
    def has_robot_skills(self) -> bool:
        return any(r.has_robot_skills for r in self.members.values())

    def list_robot_skills(self) -> List[RobotSkillSpec]:
        return [s.model_copy(update={"name": f"{m}{SEP}{s.name}"})
                for m, r in self.members.items() for s in r.list_robot_skills()]

    def _run_robot_skill(self, spec: RobotSkillSpec, args: Dict[str, Any]) -> None:
        m, local = _split_name(spec.name)
        if self.fault is not None:
            raise RobotSkillError(f"robot group is in a fault state ({self.fault}); reset() first")
        with self._calling():
            self.members[m].run_robot_skill(local, args)

    def stop_robot_skill(self) -> None:
        self._each("stop_robot_skill", lambda r: r.stop_robot_skill())


# ------------------------------------------------------------------ config

_GROUP_KEYS = {"name", "control_hz", "members", "cameras", "on_error", "expose", "collision"}
_MEMBER_KEYS = {"config", "overrides", "base_pose"}


def make_group(config: Mapping[str, Any], base_dir: Optional[str] = None,
               body_overrides: Optional[Mapping[str, Any]] = None,
               sim_overrides: Optional[Mapping[str, Any]] = None) -> RobotGroup:
    """Build a :class:`RobotGroup` from a loaded config with a ``group:`` section.

    ``make_robot`` calls this; see the config reference in ``docs/api.md``.
    Each member is ``make_robot(member.config, member.overrides merged with
    body_overrides[member])``; a relative ``config`` path is taken from
    ``base_dir`` (the group file's directory). ``sim_overrides`` applies to
    every member. The top-level ``safety:`` section configures the group's
    own gate; each member keeps its own.
    """
    from hirobot import make_robot
    from hirobot.camera import build_cameras
    from hirobot.config import resolve
    from hirobot.safety import SafetyGate

    if "robot" in config:
        raise ValueError("the config's robot: section is now called body:; rename it")
    if "body" in config or "embodiment" in config:
        raise ValueError("a config has either group: or body: / embodiment:, not both")
    g = dict(config["group"] or {})
    unknown = sorted(set(g) - _GROUP_KEYS)
    if unknown:
        raise ValueError(f"group: unknown key(s) {unknown}; known: {sorted(_GROUP_KEYS)}")
    on_error = g.get("on_error", "hold_all")
    if on_error != "hold_all":
        raise ValueError(f"group.on_error {on_error!r} is not supported; only 'hold_all' is")
    specs = g.get("members")
    if not isinstance(specs, Mapping) or not specs:
        raise ValueError("group.members must be a non-empty mapping of name -> {config: ...}")
    overrides = dict(body_overrides or {})
    unknown = sorted(set(overrides) - set(specs))
    if unknown:
        raise ValueError(f"body_overrides for a group are keyed by member; no member {unknown}")
    members: Dict[str, Robot] = {}
    built: List[Robot] = []
    try:
        for m, spec in specs.items():
            spec = dict(spec or {})
            bad = sorted(set(spec) - _MEMBER_KEYS)
            if bad:
                raise ValueError(f"group member {m!r}: unknown key(s) {bad}")
            if "config" not in spec:
                raise ValueError(f"group member {m!r} needs 'config'")
            ref = spec["config"]
            if isinstance(ref, str) and base_dir is not None:
                ref = str(resolve(ref, base=Path(base_dir)))
            body_ov = dict(spec.get("overrides") or {})
            body_ov.update(overrides.get(m) or {})
            member = make_robot(ref, body_overrides=body_ov or None,
                                sim_overrides=dict(sim_overrides) if sim_overrides else None)
            members[m] = member
            built.append(member)
    except BaseException:
        for robot in built:
            robot._group = None
        raise
    scene = [Camera(**c) for c in (g.get("cameras") or [])]
    sim_only = all(bool(r.simulated) for r in members.values())
    cameras = build_cameras(scene, sim_only=sim_only)
    base_poses = {m: s["base_pose"] for m, s in specs.items()
                  if (s or {}).get("base_pose") is not None}
    try:
        group = RobotGroup(members, cameras=cameras, name=g.get("name", "robot_group"),
                           control_hz=g.get("control_hz"),
                           scene_cameras=[c for c in scene if c.name in cameras],
                           base_poses=base_poses, expose=g.get("expose"),
                           collision=_build_collision(g.get("collision"), members, base_dir))
    except BaseException:
        for robot in members.values():
            robot._group = None
        raise
    safety_cfg = config.get("safety", {})
    if safety_cfg is not False:
        group.set_safety(SafetyGate.from_embodiment(group.embodiment, **(safety_cfg or {})))
    return group


def _build_collision(cfg: Optional[Mapping[str, Any]], members: Mapping[str, Robot],
                     base_dir: Optional[str]) -> Optional["CollisionCheck"]:
    """The group config's ``collision:`` section as a check (``None`` without one).

    A relative ``mjcf`` path is taken from the group config's directory.
    """
    if not cfg:
        return None
    from hirobot.collision import CollisionCheck

    cfg = dict(cfg)
    models = {}
    for m, spec in dict(cfg.get("models") or {}).items():
        spec = dict(spec or {})
        mjcf = spec.get("mjcf")
        if isinstance(mjcf, str) and base_dir is not None and not mjcf.startswith(("/", "~")):
            spec["mjcf"] = str(Path(base_dir) / mjcf)
        models[m] = spec
    cfg["models"] = models
    return CollisionCheck.build({m: r.embodiment.joint_names for m, r in members.items()}, cfg)
