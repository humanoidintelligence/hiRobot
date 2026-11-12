"""Fake ``piper_sdk``: a ``C_PiperInterface_V2`` that answers like a PiPER on CAN.

Models the getters the driver reads (joint, end-pose, gripper, status, low-speed
motor feedback, all in the SDK's wire units) and records every command
(``Arm.calls``). A joint move sets the joints straight to the target.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace as NS

COMMANDS = ("MotionCtrl_2", "JointCtrl", "GripperCtrl", "EnableArm", "DisableArm", "EmergencyStop",
            "MotionCtrl_1", "EndPoseCtrl", "PiperInit")


class Arm:
    """The arm's state in wire units (0.001 deg, 0.001 mm) and the commands it got."""

    def __init__(self):
        self.mdeg = [-116, 120, 0, 0, 0, 0]
        self.end_pose = [56316, -114, 213148, 0, 85120, -116]   # 0.001 mm / 0.001 deg
        self.gripper_um = 60200                                  # 60.2 mm
        self.enabled = [True] * 6
        self.feedback = True
        self.calls = []            # (method, args)
        self.can_names = []
        self.connect_kwargs = []
        self.disconnected = 0

    def commands(self):
        return [c for c in self.calls if c[0] in COMMANDS]


def install(monkeypatch) -> Arm:
    arm = Arm()

    class C_PiperInterface_V2:  # piper_sdk's name
        def __init__(self, can_name="can0", *args, **kwargs):
            arm.can_names.append(can_name)

        def ConnectPort(self, can_init=False, piper_init=True, start_thread=True):
            arm.connect_kwargs.append({"piper_init": piper_init})
            if piper_init:
                arm.calls.append(("PiperInit", ()))

        def DisconnectPort(self, thread_timeout=0.1):
            arm.disconnected += 1

        def GetArmJointMsgs(self):
            j = arm.mdeg
            return NS(time_stamp=1.0 if arm.feedback else 0.0, Hz=200.0, joint_state=NS(
                joint_1=j[0], joint_2=j[1], joint_3=j[2], joint_4=j[3], joint_5=j[4], joint_6=j[5]))

        def GetArmEndPoseMsgs(self):
            p = arm.end_pose
            return NS(time_stamp=1.0, Hz=200.0, end_pose=NS(
                X_axis=p[0], Y_axis=p[1], Z_axis=p[2], RX_axis=p[3], RY_axis=p[4], RZ_axis=p[5]))

        def GetArmGripperMsgs(self):
            return NS(time_stamp=1.0, Hz=200.0, gripper_state=NS(grippers_angle=arm.gripper_um,
                                                                   grippers_effort=0))

        def GetArmStatus(self):
            return NS(time_stamp=1.0, Hz=200.0, arm_status=NS(
                ctrl_mode="CAN_CTRL", arm_status="NORMAL", mode_feed="MOVE_J",
                motion_status="REACH_TARGET_POS_SUCCESSFULLY", err_code=0))

        def GetArmLowSpdInfoMsgs(self):
            motors = {f"motor_{i}": NS(vol=230, foc_temp=39, motor_temp=33,
                                       foc_status=NS(driver_enable_status=arm.enabled[i - 1]))
                      for i in range(1, 7)}
            return NS(time_stamp=1.0, Hz=20.0, **motors)

        def GetArmEnableStatus(self):
            return list(arm.enabled)

        def EnableArm(self, motor_num=7, enable_flag=0x02):
            arm.calls.append(("EnableArm", (motor_num,)))
            arm.enabled = [True] * 6

        def DisableArm(self, motor_num=7, enable_flag=0x01):
            arm.calls.append(("DisableArm", (motor_num,)))
            arm.enabled = [False] * 6

        def MotionCtrl_2(self, ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50, is_mit_mode=0x00,
                         residence_time=0, installation_pos=0x00):
            arm.calls.append(("MotionCtrl_2", (ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode)))

        def JointCtrl(self, *q):
            assert len(q) == 6 and all(isinstance(v, int) for v in q)
            arm.calls.append(("JointCtrl", q))
            arm.mdeg = list(q)

        def GripperCtrl(self, gripper_angle=0, gripper_effort=0, gripper_code=0, set_zero=0):
            assert isinstance(gripper_angle, int) and isinstance(gripper_effort, int)
            arm.calls.append(("GripperCtrl", (gripper_angle, gripper_effort, gripper_code, set_zero)))
            arm.gripper_um = gripper_angle

        def EmergencyStop(self, emergency_stop=0):
            arm.calls.append(("EmergencyStop", (emergency_stop,)))

    mod = types.ModuleType("piper_sdk")
    mod.C_PiperInterface_V2 = C_PiperInterface_V2
    monkeypatch.setitem(sys.modules, "piper_sdk", mod)
    return arm
