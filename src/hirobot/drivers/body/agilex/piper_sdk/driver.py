"""AgileX PiPER arm over ``piper_sdk`` (registered as ``agilex_piper_sdk``).

The PiPER is a 6-joint arm with a parallel gripper on the same CAN bus. AgileX's
`piper_sdk <https://github.com/agilexrobotics/piper_sdk>`_ (``pip install
piper_sdk``) talks to it over SocketCAN; bring the interface up at 1 Mbit/s
first (``sudo ip link set can0 up type can bitrate 1000000``).

The driver reads the arm's own feedback: joint angles, the flange pose the arm
controller computes, and the gripper stroke. ``connect()`` only opens the port
and starts the SDK's reader thread (``ConnectPort(piper_init=False)``); it sends
nothing, so reading never moves or enables the arm. Motors are enabled on the
first command, not before.

Commands are the arm's own joint move (``MOVE J``): :meth:`send_action` with a
POSITION ``joint_target`` sends ``MotionCtrl_2(CAN, MOVE J, speed)`` and
``JointCtrl``, and the arm controller plans the motion to that target at
``speed_percent`` of its top speed. ``Action.gripper`` (0 closed, 1 open) maps
linearly onto ``[0, gripper_max_m]`` and goes out as ``GripperCtrl``.

``estop()`` commands a joint move to the measured position, so the arm stops
where it is and stays powered. It does not use the controller's quick stop
(``EmergencyStop``), which lets the arm sink under damping and has to be
released before the arm moves again.

Units on the wire: joints and Euler angles in 0.001 deg, positions and gripper
stroke in 0.001 mm. The end pose's angles are ``RX, RY, RZ`` with
``R = Rz(RZ) Ry(RY) Rx(RX)`` (the SDK's own forward kinematics uses this order).
"""

from __future__ import annotations

import math
import time
from typing import Any, Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.core.robot import Robot
from hirobot.core.stamps import StampCounter
from hirobot.core.types import Action, ControlMode, Observation, RobotStatus
from hirobot.geometry.transforms import euler_xyz_to_quat
from hirobot.loading.registry import register_body
from hirobot.loading.sdk import require_sdk

MDEG = 1000.0          # wire units per degree
UM = 1e6               # wire units per metre (0.001 mm)
CTRL_CAN = 0x01        # MotionCtrl_2 ctrl_mode: CAN command control
MOVE_J = 0x01          # MotionCtrl_2 move_mode: joint move
GRIPPER_ENABLE = 0x01


def _rad_to_mdeg(q: float) -> int:
    return int(round(math.degrees(float(q)) * MDEG))


@register_body("agilex_piper_sdk")
class PiperRobot(Robot):
    """An AgileX PiPER arm and its gripper over ``piper_sdk``.

    Args:
        embodiment: The 6-joint arm embodiment; its first end-effector is the
            flange frame the arm reports (``ee_poses`` key), and its first
            end-effector with a gripper is the ``Observation.gripper`` /
            ``Action.gripper`` key.
        can: SocketCAN interface the arm is on (``can0``).
        speed_percent: Speed of the arm's joint moves, 1-100 % of its maximum.
        gripper: Read and command the gripper. ``False`` leaves it alone and
            reports no gripper.
        gripper_max_m: Stroke (m) that ``Action.gripper = 1`` opens to; the
            PiPER gripper opens to 0.07 m.
        gripper_effort_n: Gripping force for gripper commands (N, 0-5).
        feedback_timeout_s: How long ``connect()`` waits for the first joint
            feedback before it raises.
        network_interface: Ignored (filled in by ``make_robot`` for hardware).
    """

    action_modes = (ControlMode.POSITION,)

    def __init__(
        self,
        embodiment: Embodiment,
        can: str = "can0",
        speed_percent: int = 30,
        gripper: bool = True,
        gripper_max_m: float = 0.07,
        gripper_effort_n: float = 1.0,
        feedback_timeout_s: float = 2.0,
        network_interface: Optional[str] = None,
    ) -> None:
        self._embodiment = embodiment
        if embodiment.num_dofs != 6:
            raise ValueError(f"PiperRobot drives 6 joints; the embodiment has {embodiment.num_dofs}")
        self.can = str(can)
        if not 1 <= int(speed_percent) <= 100:
            raise ValueError(f"speed_percent must be in 1..100, got {speed_percent}")
        self._speed = int(speed_percent)
        if not 0.0 <= float(gripper_effort_n) <= 5.0:
            raise ValueError(f"gripper_effort_n must be in 0..5 N, got {gripper_effort_n}")
        self._gripper_max = float(gripper_max_m)
        self._gripper_effort = int(round(float(gripper_effort_n) * 1000))
        self._feedback_timeout_s = float(feedback_timeout_s)
        ees = list(embodiment.end_effectors)
        self._ee = ees[0].name if ees else "ee"
        with_gripper = [ee.name for ee in ees if ee.has_gripper]
        self._gripper = (with_gripper[0] if with_gripper else self._ee) if gripper else None
        self._piper: Any = None
        self._connected = False
        self._stamps = StampCounter()

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        require_sdk(__package__, "PiperRobot")
        try:
            from piper_sdk import C_PiperInterface_V2
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError("PiperRobot needs piper_sdk; install it with 'pip install piper_sdk'.") from exc
        piper = C_PiperInterface_V2(self.can)
        piper.ConnectPort(piper_init=False)  # open + reader thread; sends nothing
        self._piper = piper
        self._connected = True
        deadline = time.monotonic() + self._feedback_timeout_s
        while self._joint_msgs().time_stamp <= 0:
            if time.monotonic() > deadline:
                self.disconnect()
                raise ConnectionError(
                    f"no PiPER joint feedback on {self.can} within {self._feedback_timeout_s} s; "
                    "is the arm powered and the CAN interface up at 1 Mbit/s?")
            time.sleep(0.01)
        # Motor status (enable, voltage, temperature) comes at 20 Hz, one motor per
        # frame; give every motor a chance to report before get_status() reads it.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not all(m.vol > 0 for m in self._motors()):
            time.sleep(0.01)

    def disconnect(self) -> None:
        if self._piper is not None:
            try:
                self._piper.DisconnectPort()
            except Exception:  # noqa: BLE001 - closing anyway
                pass
        self._piper = None
        self._connected = False

    def _require(self) -> Any:
        if not self._connected or self._piper is None:
            raise RuntimeError("Not connected; call connect() first.")
        return self._piper

    # ------------------------------------------------------------ reading

    def _joint_msgs(self) -> Any:
        return self._require().GetArmJointMsgs()

    def _motors(self) -> list:
        low = self._require().GetArmLowSpdInfoMsgs()
        return [getattr(low, f"motor_{i}") for i in range(1, 7)]

    def get_observation(self) -> Observation:
        piper = self._require()
        js = piper.GetArmJointMsgs().joint_state
        mdeg = [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6]
        stamp = self._stamps.tick()
        obs = Observation(joint_pos=np.radians(np.asarray(mdeg, dtype=float) / MDEG))
        ep = piper.GetArmEndPoseMsgs()
        if ep.time_stamp > 0:
            p = ep.end_pose
            xyz = np.array([p.X_axis, p.Y_axis, p.Z_axis], dtype=float) / UM
            rpy = np.radians(np.array([p.RX_axis, p.RY_axis, p.RZ_axis], dtype=float) / MDEG)
            obs.ee_poses[self._ee] = np.concatenate([xyz, euler_xyz_to_quat(rpy)])
        if self._gripper is not None:
            g = piper.GetArmGripperMsgs()
            if g.time_stamp > 0:
                stroke = g.gripper_state.grippers_angle / UM
                obs.gripper[self._gripper] = float(np.clip(stroke / self._gripper_max, 0.0, 1.0))
                obs.stamps[self._gripper] = stamp
        obs.stamps["body"] = stamp
        return obs

    def get_status(self) -> RobotStatus:
        status = RobotStatus(timestamp=time.time(), fault=self.fault)
        if not self._connected:
            return status
        piper = self._require()
        arm = piper.GetArmStatus().arm_status
        motors = self._motors()
        names = self._embodiment.joint_names
        status.joint_temperature = np.array([float(m.motor_temp) for m in motors])
        status.board_temperature = {f"{n}_driver": float(m.foc_temp) for n, m in zip(names, motors)}
        err = int(getattr(arm, "err_code", 0) or 0)
        if err:
            status.extra["err_code"] = err
        status.extra.update({
            "ctrl_mode": str(arm.ctrl_mode), "arm_status": str(arm.arm_status),
            "mode_feed": str(arm.mode_feed), "motion_status": str(arm.motion_status),
            "enabled": [bool(m.foc_status.driver_enable_status) for m in motors],
            "voltage_v": float(motors[0].vol) / 10.0,
        })
        return status

    # ------------------------------------------------------------ commands

    def reset(self, seed: Optional[int] = None) -> None:
        """Nothing to do: the arm stays where it is (there is no home pose to go to)."""

    def _estop(self) -> None:
        """Hold the measured joint positions (a joint move to where the arm is)."""
        if not self._connected:
            return
        piper = self._require()
        js = piper.GetArmJointMsgs().joint_state
        piper.MotionCtrl_2(CTRL_CAN, MOVE_J, self._speed, 0x00)
        piper.JointCtrl(js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6)

    def _enable(self, piper: Any) -> None:
        if not all(piper.GetArmEnableStatus()):
            piper.EnableArm(7)

    def _send_action(self, action: Action) -> None:
        piper = self._require()
        if action.mode != ControlMode.POSITION:
            raise ValueError(f"PiperRobot does not support mode {action.mode!r}")
        opening = action.gripper.get(self._gripper) if (action.gripper and self._gripper) else None
        if opening is not None:
            opening = float(opening)
            if not 0.0 <= opening <= 1.0:
                raise ValueError(f"gripper {self._gripper!r}: {opening} is outside [0, 1]")
        if action.joint_target is None and opening is None:
            return
        self._enable(piper)
        if action.joint_target is not None:
            q = np.asarray(action.joint_target, dtype=float)
            piper.MotionCtrl_2(CTRL_CAN, MOVE_J, self._speed, 0x00)
            piper.JointCtrl(*(_rad_to_mdeg(v) for v in q))
        if opening is not None:
            stroke = int(round(opening * self._gripper_max * UM))
            piper.GripperCtrl(stroke, self._gripper_effort, GRIPPER_ENABLE, 0x00)
