"""AgiBot G1 (精灵 G01) body driver over the GDK's DDS topics (registered as ``agibot_gdk``).

Runs on the robot's own computer (system Python 3.10 with ``cosine_bus`` and the
``genie_msgs_pb`` message package; the vendor environment sourced). It does not
use ``a2d_sdk.robot``: that pulls in ruckig and scipy and needs a newer message
package than the system ships. It talks to the topics directly:

* **Read** (one callback per topic, each keeps its latest message):
  ``/hal/arm_joint_state`` (both arms, 200 Hz), ``/hal/{left,right}_ee_data``
  (grippers, 200 Hz), ``/hal/waist_state`` and ``/hal/neck_state`` (20 Hz),
  ``/hal/whole_body_status`` (errors, e-stops) and ``/wbc/motion_control_status``
  (mode, ``arm_{left,right}_link7`` poses in ``base_link``).
* **Write** ``/wbc/joint_position_control``: one message carries every commanded
  group (group bits as in a2d_sdk 1.5.0). The motion controller follows it for
  ``lifetime`` and stops when commands stop. Mode changes go to
  ``/wbc/set_control_mode`` and ``/wbc/motion_stop`` (``input_type=54``, GDK).

The motion controller has STOP / SERVO / PLANNING modes. :meth:`connect` only
reads; :meth:`reset` asks for SERVO when it is in STOP (it does not move any
joint); :meth:`send_action` refuses unless the mode is SERVO; :meth:`damp` and
:meth:`disconnect` send ``MotionStop`` (the joints stop where they are).

Only the groups in ``groups`` are commanded (default: both arms). The other
joints are read but not sent: a target for one of them must equal its measured
position (within ``hold_tolerance``) or the action is refused, so a target is
never dropped silently.

Grippers: the GDK's tool joint runs 0 open to 1 closed and reports 0 (open) to
about 120 (closed). ``Observation.gripper`` and ``Action.gripper`` use hiRobot's
scale, 0 closed and 1 open; the driver converts both ways.

The chassis has its own controller box (see :mod:`.chassis`). With
``chassis_host`` set, a thread polls its state at 10 Hz (``base_position`` /
``base_orientation`` from the odometer, battery in :meth:`get_status`) and
resends the latest ``BASE_VELOCITY`` at 10 Hz, switching to zero speed when no
new command came for ``command_timeout`` seconds.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from hirobot.drivers.body.agibot.gdk.chassis import ChassisClient, pose_from_state
from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot, RobotFaultError
from hirobot.sdk import require_sdk
from hirobot.stamps import Stamp, StampCounter
from hirobot.types import Action, BatteryState, ControlMode, Observation, RobotStatus

logger = logging.getLogger(__name__)

#: Motion-controller modes (MotionControlStatus.mode / SetControlMode.control_mode).
MODE_STOP, MODE_SERVO, MODE_PLANNING = 0, 1, 2
_MODE_NAMES = {MODE_STOP: "STOP", MODE_SERVO: "SERVO", MODE_PLANNING: "PLANNING"}
#: MotionStop / SetControlMode ``input_type`` for commands from the GDK.
_INPUT_GDK = 54

#: JointPositionControl.control_group bits (a2d_sdk 1.5.0 JOINT_GROUP_IDS).
GROUP_BITS = {"head_yaw": 1, "head_pitch": 2, "left_arm": 4, "right_arm": 8,
              "waist_lift": 16, "waist_pitch": 32, "left_tool": 128, "right_tool": 256}

#: Joint names hiRobot uses (the motion controller's a2d_v01 model without the
#: ``idxNN_`` prefix), with the group each belongs to and its slot in that group.
ARM_JOINTS = tuple(f"{side}_arm_joint{i}" for side in ("left", "right") for i in range(1, 8))
JOINT_SLOTS: Dict[str, Tuple[str, int]] = {
    "waist_lift_joint": ("waist_lift", 0),
    "waist_pitch_joint": ("waist_pitch", 0),
    "head_yaw_joint": ("head_yaw", 0),
    "head_pitch_joint": ("head_pitch", 0),
    **{n: (n[:len("left_arm")] if n.startswith("left") else n[:len("right_arm")], int(n[-1]) - 1)
       for n in ARM_JOINTS},
}
#: Names the HAL state topics use for the waist and head motors.
_HAL_NAMES = {"waist_lift_joint": "joint_lift_body", "waist_pitch_joint": "joint_body_pitch",
              "head_yaw_joint": "joint_head_yaw", "head_pitch_joint": "joint_head_pitch"}

_TOPICS = {
    "arms": "/hal/arm_joint_state",
    "left_ee": "/hal/left_ee_data",
    "right_ee": "/hal/right_ee_data",
    "waist": "/hal/waist_state",
    "head": "/hal/neck_state",
    "body": "/hal/whole_body_status",
    "motion": "/wbc/motion_control_status",
}
_REQUIRED = ("arms", "left_ee", "right_ee", "waist", "head", "motion")


@register_body("agibot_gdk")
class AgibotG1Robot(Robot):
    """AgiBot G1: two 7-DoF arms, waist lift and pitch, head, two grippers, wheeled chassis.

    Args:
        embodiment: Joints named as in :data:`JOINT_SLOTS`, in any order.
        groups: Joint groups to command (keys of :data:`GROUP_BITS` without the
            tools; the tools follow ``Action.gripper``). Default both arms.
        gripper_names: End-effector names of the ``left`` / ``right`` gripper
            (the ``gripper`` and ``ee_poses`` keys).
        gripper_raw_closed: Raw gripper reading when fully closed.
        lifetime: ``JointPositionControl.lifetime`` sent with every command
            (the vendor examples use 0.2 when sending at 20 Hz).
        hold_tolerance: How far a target for a joint outside ``groups`` may be
            from its measured position (rad or m).
        max_state_age: Refuse to command when the arm state or the motion
            status is older than this (s).
        connect_timeout: Seconds to wait for every state topic on connect.
        read_only: Never create a publisher; commands and :meth:`reset` raise.
        chassis_host: Address of the chassis controller box; ``None`` leaves
            the chassis out (no base state, no ``BASE_VELOCITY``).
        chassis_id: The ``id`` the chassis controller answers to.
        command_timeout: Seconds after the last ``BASE_VELOCITY`` before the
            chassis is sent zero speed.
        max_linear_speed: Largest accepted ``|vx|`` (m/s).
        max_angular_speed: Largest accepted ``|wz|`` (rad/s).
    """

    def __init__(
        self,
        embodiment: Embodiment,
        groups: Sequence[str] = ("left_arm", "right_arm"),
        gripper_names: Optional[Dict[str, str]] = None,
        gripper_raw_closed: float = 120.0,
        lifetime: float = 0.2,
        hold_tolerance: float = 0.02,
        max_state_age: float = 0.2,
        connect_timeout: float = 3.0,
        read_only: bool = False,
        chassis_host: Optional[str] = None,
        chassis_id: int = 780,
        command_timeout: float = 0.3,
        max_linear_speed: float = 0.2,
        max_angular_speed: float = 0.5,
    ) -> None:
        self._embodiment = embodiment
        names = list(embodiment.joint_names)
        unknown = [n for n in names if n not in JOINT_SLOTS]
        if unknown:
            raise ValueError(f"agibot_gdk: unknown joint(s) {unknown}; expected names from "
                             f"{sorted(JOINT_SLOTS)}")
        bad = [g for g in groups if g not in GROUP_BITS or g.endswith("_tool")]
        if bad:
            raise ValueError(f"agibot_gdk: unknown group(s) {bad}; choose from "
                             f"{[g for g in GROUP_BITS if not g.endswith('_tool')]}")
        self._groups = tuple(groups)
        self._names = names
        self._commanded = np.array([JOINT_SLOTS[n][0] in self._groups for n in names])
        self._grippers = dict(gripper_names or {"left": "left_gripper", "right": "right_gripper"})
        self._raw_closed = float(gripper_raw_closed)
        self._lifetime = float(lifetime)
        self._hold_tol = float(hold_tolerance)
        self._max_age = float(max_state_age)
        self._connect_timeout = float(connect_timeout)
        self._read_only = bool(read_only)
        self._chassis_host = chassis_host
        self._chassis_id = int(chassis_id)
        self._cmd_timeout = float(command_timeout)
        self._max_v = float(max_linear_speed)
        self._max_w = float(max_angular_speed)

        self._msgs: Any = None  # message classes, set on connect
        self._node: Any = None
        self._subs: List[Any] = []
        self._pubs: Dict[str, Any] = {}
        self._latest: Dict[str, Tuple[Any, Stamp]] = {}
        self._counters: Dict[str, StampCounter] = {k: StampCounter() for k in _TOPICS}
        #: Commands published on /wbc/joint_position_control (for tests and logs).
        self.commands_sent = 0

        self._chassis: Optional[ChassisClient] = None
        self._chassis_thread: Optional[threading.Thread] = None
        self._chassis_stop = threading.Event()
        self._chassis_lock = threading.Lock()
        self._chassis_state: Optional[Tuple[Dict[str, Any], float]] = None
        self._base_cmd: Optional[Tuple[float, float, float]] = None  # (vx, wz, time)
        #: Last chassis error (state poll or velocity send), for diagnosis.
        self.chassis_error: Optional[BaseException] = None
        #: ManualMove messages sent to the chassis.
        self.chassis_sent = 0

    # ------------------------------------------------------------- contract
    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    @property
    def action_modes(self) -> Tuple[ControlMode, ...]:  # type: ignore[override]
        if self._chassis_host is None:
            return (ControlMode.POSITION,)
        return (ControlMode.POSITION, ControlMode.BASE_VELOCITY)

    @property
    def read_only(self) -> bool:
        return self._read_only

    # ------------------------------------------------------------ lifecycle
    def connect(self) -> None:
        if self._node is not None:
            return
        require_sdk(__package__, "AgibotG1Robot")
        self._msgs = _import_messages()
        dds = self._msgs.agibotdds
        self._node = dds.Node("HiRobotAgibotG1")
        qos = dds.qos(depth=10)
        m = self._msgs
        for key, msg_type in (("arms", m.JointState), ("left_ee", m.EndState),
                              ("right_ee", m.EndState), ("waist", m.WaistState),
                              ("head", m.HeadState), ("body", m.WholeBodyStatus),
                              ("motion", m.MotionControlStatus)):
            self._subs.append(self._node.create_subscriber(
                _TOPICS[key], msg_type, qos, self._make_callback(key)))
        if not self._read_only:
            self._pubs = {
                "joints": self._node.create_publisher("/wbc/joint_position_control",
                                                      m.JointPositionControl, qos),
                "mode": self._node.create_publisher("/wbc/set_control_mode",
                                                    m.SetControlMode, qos),
                "stop": self._node.create_publisher("/wbc/motion_stop", m.MotionStop, qos),
            }
        deadline = time.monotonic() + self._connect_timeout
        while time.monotonic() < deadline:
            if all(k in self._latest for k in _REQUIRED):
                break
            time.sleep(0.02)
        else:
            missing = [_TOPICS[k] for k in _REQUIRED if k not in self._latest]
            self._release()
            raise TimeoutError(
                f"AgibotG1Robot: no message on {missing} within {self._connect_timeout} s. "
                "Run on the robot's computer with the vendor environment sourced "
                "(source <vendor app dir>/env.sh <vendor app dir>/lib, docs/setup/agibot_g1.md) and the motion "
                "controller running.")
        if self._chassis_host is not None:
            self._start_chassis()

    def _make_callback(self, key: str):
        counter = self._counters[key]

        def callback(msg: Any) -> None:
            self._latest[key] = (msg, counter.tick())
        return callback

    def reset(self, seed: Optional[int] = None) -> None:
        """Ask for SERVO when the motion controller is in STOP; moves no joint."""
        if self._read_only:
            raise RuntimeError("AgibotG1Robot is read_only; reset() would change the motion "
                               "controller's mode")
        self._require_connected()
        self._check_body_errors()
        if self.motion_mode() != MODE_SERVO:
            msg = self._msgs.SetControlMode()
            msg.input_type = _INPUT_GDK
            msg.control_mode = MODE_SERVO
            self._pubs["mode"].publish(msg)
            token = self._stop_token()
            deadline = time.monotonic() + 2.0
            while self.motion_mode() != MODE_SERVO:
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"AgibotG1Robot: motion controller stayed in "
                        f"{_MODE_NAMES.get(self.motion_mode(), self.motion_mode())} after a "
                        "SERVO request; check the HMI for another controller or an error")
                self._wait(0.02, token)

    def damp(self) -> None:
        """Send ``MotionStop``: the motion controller goes to STOP, joints stay where they are."""
        self._base_cmd = None
        if self._pubs:
            msg = self._msgs.MotionStop()
            msg.input_type = _INPUT_GDK
            self._pubs["stop"].publish(msg)

    def disconnect(self) -> None:
        if self._node is None:
            return
        try:
            self.damp()
        finally:
            self._stop_chassis()
            self._release()

    def _release(self) -> None:
        self._subs.clear()
        self._pubs = {}
        self._node = None
        self._latest.clear()

    # --------------------------------------------------------------- reading
    def motion_mode(self) -> Optional[int]:
        """The motion controller's mode (``MODE_STOP`` / ``MODE_SERVO`` / ``MODE_PLANNING``)."""
        entry = self._latest.get("motion")
        return None if entry is None else int(entry[0].mode)

    def get_observation(self) -> Observation:
        self._require_connected()
        q = np.full(len(self._names), np.nan)
        dq = np.full(len(self._names), np.nan)
        arms = self._latest["arms"][0]
        arm_pos = dict(zip(arms.name, arms.position))
        arm_vel = dict(zip(arms.name, arms.velocity)) if len(arms.velocity) == len(arms.name) \
            else {}
        motors = {}
        for key in ("waist", "head"):
            msg = self._latest[key][0]
            motors.update({n: s for n, s in zip(msg.name, msg.motor_states)})
        for i, name in enumerate(self._names):
            if name in arm_pos:
                q[i] = arm_pos[name]
                dq[i] = arm_vel.get(name, np.nan)
            else:
                motor = motors.get(_HAL_NAMES[name])
                if motor is not None:
                    q[i] = motor.position
                    dq[i] = motor.velocity
        obs = Observation(joint_pos=q, joint_vel=None if np.isnan(dq).any() else dq)
        # The body's joints come from three topics; its stamp is the oldest of them.
        oldest = min((self._latest[k][1] for k in ("arms", "waist", "head")), key=lambda s: s.t)
        obs.stamps["body"] = Stamp(oldest.t, self._latest["arms"][1].seq)
        for side in ("left", "right"):
            gripper = self._gripper_open(side)
            if gripper is not None:
                obs.gripper[self._grippers[side]] = gripper
                obs.stamps[self._grippers[side]] = self._latest[f"{side}_ee"][1]
        motion = self._latest["motion"][0]
        for frame, pose in zip(motion.frame_names, motion.frame_poses):
            side = {"arm_left_link7": "left", "arm_right_link7": "right"}.get(frame)
            if side is None:
                continue
            quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                             pose.orientation.w])
            norm = np.linalg.norm(quat)
            if norm > 0.5:
                obs.ee_poses[self._grippers[side]] = np.concatenate(
                    [[pose.position.x, pose.position.y, pose.position.z], quat / norm])
        chassis = self._chassis_state
        if chassis is not None:
            pose = pose_from_state(chassis[0])
            if pose is not None:
                x, y, theta = pose
                obs.base_position = np.array([x, y, 0.0])
                obs.base_orientation = np.array([0.0, 0.0, math.sin(theta / 2),
                                                 math.cos(theta / 2)])
        return obs

    def _gripper_open(self, side: str) -> Optional[float]:
        """Gripper opening on hiRobot's scale (1 open, 0 closed) from the raw reading."""
        msg = self._latest[f"{side}_ee"][0]
        if not msg.end_state:
            return None
        raw = float(msg.end_state[0].position)
        return float(np.clip(1.0 - raw / self._raw_closed, 0.0, 1.0))

    # -------------------------------------------------------------- commands
    def _send_action(self, action: Action) -> None:
        if self._read_only:
            raise RuntimeError("AgibotG1Robot is read_only")
        self._require_connected()
        if action.mode == ControlMode.BASE_VELOCITY:
            self._command_base(action.base_velocity)
            return
        if action.mode != ControlMode.POSITION:
            raise ValueError(f"AgibotG1Robot takes {[m.value for m in self.action_modes]}, "
                             f"got {action.mode.value}")
        self._check_commandable()
        msg = self._msgs.JointPositionControl()
        msg.lifetime = self._lifetime
        mask = 0
        if action.joint_target is not None:
            target = self._full_target(action)
            values: Dict[str, List[float]] = {}
            for name, value in zip(self._names, target):
                group, slot = JOINT_SLOTS[name]
                if group in self._groups:
                    values.setdefault(group, [0.0] * (7 if group.endswith("_arm") else 1))
                    values[group][slot] = float(value)
            for group, vals in values.items():
                mask |= GROUP_BITS[group]
                if group.endswith("_arm"):
                    getattr(msg, f"{group}_joint_positions").extend(vals)
                else:
                    setattr(msg, f"{group}_joint_position", vals[0])
        for side in ("left", "right"):
            opening = action.gripper.get(self._grippers[side]) if action.gripper else None
            if opening is None:
                continue
            if not 0.0 <= float(opening) <= 1.0:
                raise ValueError(f"gripper {self._grippers[side]!r}: {opening} is outside [0, 1]")
            mask |= GROUP_BITS[f"{side}_tool"]
            getattr(msg, f"{side}_tool_joint_positions").append(1.0 - float(opening))
        if mask == 0:
            return
        msg.control_group = mask
        self._pubs["joints"].publish(msg)
        self.commands_sent += 1

    def _full_target(self, action: Action) -> np.ndarray:
        """The target for every embodiment joint; joints outside ``groups`` must not move."""
        current = self.get_observation().joint_pos
        target = np.asarray(action.joint_target, dtype=float)
        free = ~self._commanded
        drift = np.abs(target[free] - current[free])
        if np.any(drift > self._hold_tol):
            moved = [n for n, d in zip(np.array(self._names)[free], drift) if d > self._hold_tol]
            raise ValueError(
                f"AgibotG1Robot commands only {list(self._groups)}; joint(s) {moved} have a "
                f"target away from their measured position (> {self._hold_tol}). Add their "
                "group to `groups` to command them.")
        return target

    def _check_commandable(self) -> None:
        now = time.monotonic()
        for key in ("arms", "motion"):
            age = now - self._latest[key][1].t
            if age > self._max_age:
                raise RobotFaultError(f"AgibotG1Robot: {_TOPICS[key]} is {age:.2f} s old")
        self._check_body_errors()
        mode = self.motion_mode()
        if mode != MODE_SERVO:
            raise RobotFaultError(
                f"AgibotG1Robot: motion controller is in {_MODE_NAMES.get(mode, mode)}, not "
                "SERVO; call reset() to request SERVO")

    def _check_body_errors(self) -> None:
        motion = self._latest["motion"][0]
        if motion.error_code != 0:
            raise RobotFaultError(f"AgibotG1Robot: motion controller error {motion.error_code} "
                                  f"{motion.error_msg!r}")
        body = self._latest.get("body")
        if body is None:
            return
        body = body[0]
        if body.left_arm_estop or body.right_arm_estop:
            raise RobotFaultError("AgibotG1Robot: an arm e-stop is pressed")
        errors = {f: getattr(body, f) for f in ("left_arm_error", "right_arm_error")
                  if getattr(body, f)}
        for side in ("left", "right"):
            if f"{side}_arm" in self._groups and errors.get(f"{side}_arm_error"):
                raise RobotFaultError(f"AgibotG1Robot: {side}_arm_error "
                                      f"{errors[f'{side}_arm_error']}")

    def get_status(self) -> RobotStatus:
        status = RobotStatus(timestamp=time.time(), fault=self.fault)
        if self._node is None:
            return status
        for key in ("waist", "head"):
            msg = self._latest[key][0]
            for name, motor in zip(msg.name, motor_states(msg)):
                if motor.err_code:
                    status.joint_faults[name] = int(motor.err_code)
        motion = self._latest["motion"][0]
        status.extra["motion_mode"] = _MODE_NAMES.get(int(motion.mode), int(motion.mode))
        if motion.error_code:
            status.extra["motion_error"] = f"{motion.error_code} {motion.error_msg}"
        body = self._latest.get("body")
        if body is not None:
            for field in body[0].DESCRIPTOR.fields:
                if field.name != "header":
                    status.extra[f"hal.{field.name}"] = getattr(body[0], field.name)
        chassis = self._chassis_state
        if chassis is not None:
            content = chassis[0]
            packs = content.get("batteryStateList") or []
            if packs:
                status.battery = BatteryState(soc=packs[0].get("energy"),
                                              voltage=packs[0].get("voltage"))
            for key in ("isSoftAuto", "isHardAuto", "isScram", "isPause", "exceptions",
                        "confidence"):
                if key in content:
                    status.extra[f"chassis.{key}"] = content[key]
        if self.chassis_error is not None:
            status.extra["chassis.error"] = repr(self.chassis_error)
        return status

    def _require_connected(self) -> None:
        if self._node is None:
            raise RuntimeError("AgibotG1Robot is not connected")

    # --------------------------------------------------------------- chassis
    def _command_base(self, velocity: Any) -> None:
        if self._chassis is None:
            raise ValueError("AgibotG1Robot has no chassis (set chassis_host)")
        vx, vy, wz = (float(v) for v in np.asarray(velocity, dtype=float).reshape(3))
        if abs(vy) > 1e-6:
            raise ValueError("AgibotG1Robot: the chassis takes no sideways speed (vy must be 0)")
        if abs(vx) > self._max_v or abs(wz) > self._max_w:
            raise ValueError(f"AgibotG1Robot: base velocity [{vx}, {wz}] exceeds "
                             f"max_linear_speed {self._max_v} / max_angular_speed {self._max_w}")
        state = self._chassis_state
        # Soft-manual on the chassis web page reads isHardAuto=true (the hardware
        # switch hands control to software) and isSoftAuto=false (software is in
        # manual, not navigating). Checked on the G1 on 2026-11-09.
        if state is not None and (vx or wz) and not (
                state[0].get("isHardAuto") is True and state[0].get("isSoftAuto") is False):
            raise RobotFaultError("AgibotG1Robot: the chassis is not in soft-manual mode; "
                                  "switch it on the chassis web page")
        self._base_cmd = (vx, wz, time.monotonic())

    def _start_chassis(self) -> None:
        self._chassis = ChassisClient(self._chassis_host, self._chassis_id)
        self._chassis_state = (self._chassis.get_state(), time.monotonic())
        self._chassis_stop.clear()
        self._chassis_thread = threading.Thread(target=self._chassis_loop,
                                                name="agibot-chassis", daemon=True)
        self._chassis_thread.start()

    def _chassis_loop(self) -> None:
        moving = False
        while not self._chassis_stop.wait(0.1):
            try:
                self._chassis_state = (self._chassis.get_state(), time.monotonic())
            except Exception as exc:  # noqa: BLE001 - keep polling; the error is reported
                self.chassis_error = exc
            if self._read_only:
                continue
            cmd = self._base_cmd
            if cmd is not None and time.monotonic() - cmd[2] > self._cmd_timeout:
                cmd = self._base_cmd = None
            vx, wz = (cmd[0], cmd[1]) if cmd is not None else (0.0, 0.0)
            if not moving and vx == 0.0 and wz == 0.0:
                continue  # nothing to do; never touch the chassis unasked
            try:
                self._chassis.move(vx * 1000.0, math.degrees(wz))
                self.chassis_sent += 1
            except Exception as exc:  # noqa: BLE001
                self.chassis_error = exc
            moving = vx != 0.0 or wz != 0.0

    def _stop_chassis(self) -> None:
        thread, self._chassis_thread = self._chassis_thread, None
        self._chassis_stop.set()
        if thread is not None:
            thread.join(timeout=1.0)
        chassis, self._chassis = self._chassis, None
        if chassis is None:
            return
        try:
            if not self._read_only and self.chassis_sent:
                for _ in range(3):
                    chassis.move(0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            logger.warning("AgibotG1Robot: zero speed to the chassis failed: %r", exc)
        finally:
            chassis.close()
        self._chassis_state = None


def motor_states(msg: Any) -> Sequence[Any]:
    return list(msg.motor_states)


class _Messages:
    """The cosine_bus DDS module and the GDK message classes, imported on connect."""

    agibotdds: Any
    EndState: Any
    HeadState: Any
    WaistState: Any
    JointPositionControl: Any
    MotionStop: Any
    SetControlMode: Any
    MotionControlStatus: Any
    WholeBodyStatus: Any
    JointState: Any


def _import_messages() -> _Messages:
    try:
        from cosine_bus.agibotdds_py3 import agibotdds
        from genie_msgs_pb.msg.EndState_pb2 import EndState
        from genie_msgs_pb.msg.HeadState_pb2 import HeadState
        from genie_msgs_pb.msg.JointPositionControl_pb2 import JointPositionControl
        from genie_msgs_pb.msg.MotionControlStatus_pb2 import MotionControlStatus
        from genie_msgs_pb.msg.MotionStop_pb2 import MotionStop
        from genie_msgs_pb.msg.SetControlMode_pb2 import SetControlMode
        from genie_msgs_pb.msg.WaistState_pb2 import WaistState
        from genie_msgs_pb.msg.WholeBodyStatus_pb2 import WholeBodyStatus
        from sensor_msgs_pb.msg.JointState_pb2 import JointState
    except ImportError as exc:
        raise ImportError(
            "AgibotG1Robot needs cosine_bus and genie_msgs_pb >= 0.8.0 (the GDK installer on "
            "the robot carries it: a2d_sdk/gdk_installer in the vendor app dir; docs/setup/agibot_g1.md). Run with the robot's "
            f"system Python 3.10 and the vendor environment sourced. ({exc})") from exc
    m = _Messages()
    m.agibotdds = agibotdds
    m.EndState, m.HeadState, m.WaistState = EndState, HeadState, WaistState
    m.JointPositionControl, m.MotionStop, m.SetControlMode = (
        JointPositionControl, MotionStop, SetControlMode)
    m.MotionControlStatus, m.WholeBodyStatus, m.JointState = (
        MotionControlStatus, WholeBodyStatus, JointState)
    return m
