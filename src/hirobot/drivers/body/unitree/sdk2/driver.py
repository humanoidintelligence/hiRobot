"""Unitree humanoid driver (G1 / H1-2 / H2, and legacy H1).

Wraps ``unitree_sdk2py`` behind the :class:`~hirobot.robot.Robot` contract. All
communication is DDS: we publish a ``LowCmd`` on ``rt/lowcmd`` and subscribe to
``LowState`` on ``rt/lowstate``. G1, H1-2 and H2 all use the ``unitree_hg``
message family; the original 19-DoF H1 uses the older ``unitree_go`` family —
selected here by the ``msg_family`` config key so one driver covers all four.

The SDK is imported lazily so the core package has no hard dependency on it;
install it (https://github.com/unitreerobotics/unitree_sdk2_python) to talk to real hardware.

Safety notes for real hardware:
* The robot must be hung / supported and in damping mode before you start; the
  first thing a control stack should do is ramp from the *measured* pose, never
  jump to a target. ``reset()`` here ramps to the current pose over a short
  window.
* ``mode_machine`` must be read back from ``LowState`` and echoed in every
  ``LowCmd`` — a wrong value is silently rejected by firmware.
* Every ``LowCmd`` must carry a correct CRC or the firmware drops it.
* The legacy H1 (``go`` family) leaves motor slot 9 unused, so its 19 joints
  are not slots 0-18; ``motor_index`` maps each joint to its slot. A ``go``
  command also needs the frame header (``head`` 0xFE 0xEF, ``level_flag``
  0xFF) and a per-motor mode, 0x01 on the "weak" motors and 0x0A elsewhere
  (``go_weak_motors``), as in the SDK's ``example/h1/low_level``.

Health signals (:meth:`UnitreeRobot.get_status`): motor temperatures and fault
codes come from ``LowState`` on both families. On ``hg`` the battery and main
board are separate topics (``rt/lf/bmsstate``, ``rt/lf/mainboardstate``, both
~20 Hz on a G1); on ``go`` the battery rides inside ``LowState``.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Sequence, Union

import numpy as np

from hirobot.drivers._sdk.unitree_sdk2 import channel_api, init_channel
from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot
from hirobot.robot_skills import RobotSkillError, RobotSkillSpec
from hirobot.safety import SafetyError
from hirobot.sdk import require_sdk
from hirobot.stamps import Stamp, StampCounter
from hirobot.types import Action, BatteryState, ControlMode, Observation, RobotStatus

# Topics are fixed by the SDK.
_LOWCMD_TOPIC = "rt/lowcmd"
_LOWSTATE_TOPIC = "rt/lowstate"
# hg family only: battery and main-board state, published by the robot at ~20 Hz.
_BMS_TOPIC = "rt/lf/bmsstate"
_MAINBOARD_TOPIC = "rt/lf/mainboardstate"
# Legacy H1 motor slots that take mode 0x01 (ankles 10-11, arms 12-19); the
# rest take 0x0A. From unitree_sdk2_python example/h1/low_level.
_H1_WEAK_MOTORS = tuple(range(10, 20))
# Legacy H1 motor slot of each of its 19 joints: slot 9 (kNotUsedJoint) is skipped.
_H1_MOTOR_INDEX = tuple(i for i in range(20) if i != 9)
_BUILTIN_OWNS_LOWCMD = (
    "UnitreeRobot was built with builtin_skills: the built-in motion service owns "
    "rt/lowcmd, so joint commands are refused; use robot skills, or rebuild with "
    "release_motion_mode to drive the joints")


@register_body("unitree_sdk2")
class UnitreeRobot(Robot):
    """Driver for Unitree humanoids over the SDK2 DDS interface.

    Args:
        embodiment: The embodiment; its joint order must match the robot's
            native motor index order (see the per-robot configs).
        motor_index: SDK motor slot of each joint, in embodiment order.
            Default ``0..n-1``, except with ``msg_family: go`` and 19 joints
            (the legacy H1), where it is ``0..8, 10..19``: slot 9 is unused.
        network_interface: NIC name the robot is reachable on (e.g. ``eth0``);
            ``None`` lets the SDK auto-detect.
        msg_family: ``"hg"`` for G1/H1-2/H2, ``"go"`` for the legacy 19-DoF H1.
        domain_id: DDS domain id (SDK default is 0).
        default_kp / default_kd: Fallback gains used when a policy does not
            supply them in the action. A scalar is broadcast to every joint; a
            list gives one value per joint, in embodiment order.
        ramp_seconds: Duration of the pose ramp performed by :meth:`reset`.
        release_motion_mode: If true, :meth:`connect` asks the robot's motion
            switcher to release the built-in motion service (e.g. ``ai`` /
            locomotion) so ``rt/lowcmd`` is not contested. The robot then has no
            built-in balance: only enable this with the robot hung or supported.
        damping_kd: Per-motor damping used by :meth:`damp` (kp = 0), which
            :meth:`disconnect` sends so the robot goes limp-but-damped on exit.
        read_only: Subscribe to state but never create the ``rt/lowcmd``
            publisher, so nothing this driver does can command the robot (used
            by the sim ``shadow`` mode). Any command raises ``RuntimeError``.
        go_weak_motors: ``go`` family only: motor slots sent with mode 0x01
            instead of 0x0A. Defaults to the H1's ankles and arms.
        builtin_skills: Expose the robot's built-in skills (walk, sit, arm
            gestures) as robot skills: ``"g1"`` for the G1's fixed skill list,
            run over its ``sport`` / ``arm`` services (see
            :mod:`~hirobot.drivers.body.unitree.sdk2.g1_skills`), ``None`` for none.
            The built-in service then owns ``rt/lowcmd``: the driver creates no lowcmd publisher, so ``send_action`` / ``reset`` raise and
            the two never fight over the motors. Exclusive with
            ``release_motion_mode`` (which hands ``rt/lowcmd`` to hiRobot) and
            ``read_only``.
    """

    def __init__(
        self,
        embodiment: Embodiment,
        network_interface: Optional[str] = None,
        msg_family: str = "hg",
        domain_id: int = 0,
        default_kp: Union[float, Sequence[float]] = 60.0,
        default_kd: Union[float, Sequence[float]] = 1.0,
        ramp_seconds: float = 2.0,
        release_motion_mode: bool = False,
        damping_kd: float = 8.0,
        read_only: bool = False,
        motor_index: Optional[Sequence[int]] = None,
        go_weak_motors: Sequence[int] = _H1_WEAK_MOTORS,
        builtin_skills: Optional[str] = None,
    ) -> None:
        if read_only and release_motion_mode:
            raise ValueError("read_only and release_motion_mode are mutually exclusive")
        if msg_family not in ("hg", "go"):
            raise ValueError(f"msg_family must be 'hg' or 'go', got {msg_family!r}")
        self._embodiment = embodiment
        self._network_interface = network_interface
        self._msg_family = msg_family
        self._domain_id = domain_id
        self._n = embodiment.num_dofs
        if motor_index is None:
            # The go family is only the legacy H1, whose 19 joints skip slot 9.
            go_h1 = msg_family == "go" and self._n == len(_H1_MOTOR_INDEX)
            motor_index = _H1_MOTOR_INDEX if go_h1 else range(self._n)
        self._motor_index = [int(i) for i in motor_index]
        if len(self._motor_index) != self._n:
            raise ValueError(f"motor_index has {len(self._motor_index)} entries, "
                             f"embodiment has {self._n} joints")
        if len(set(self._motor_index)) != self._n or min(self._motor_index) < 0:
            raise ValueError(f"motor_index must be distinct non-negative slots: {self._motor_index}")
        weak = set(int(i) for i in go_weak_motors)
        self._go_mode = [0x01 if i in weak else 0x0A for i in self._motor_index]
        self._kp = self._per_joint(default_kp, "default_kp")
        self._kd = self._per_joint(default_kd, "default_kd")
        self._ramp_seconds = ramp_seconds
        self._release_motion_mode = release_motion_mode
        self._damping_kd = float(damping_kd)
        self._read_only = bool(read_only)
        if builtin_skills not in (None, "g1"):
            raise ValueError(f"builtin_skills must be 'g1' or None, got {builtin_skills!r}")
        if builtin_skills is not None and (read_only or release_motion_mode):
            raise ValueError(
                "builtin_skills needs the built-in motion service, so it cannot be "
                "combined with read_only or release_motion_mode")
        self._builtin_skills = builtin_skills
        self._skills = None

        self._publisher = None
        self._subscriber = None
        self._low_cmd = None
        self._low_state = None
        self._low_state_stamp: Optional[Stamp] = None
        self._state_stamps = StampCounter()
        self._bms_subscriber = None
        self._board_subscriber = None
        self._bms_state = None
        self._board_state = None
        self._crc = None
        self._mode_machine = 0
        self._connected = False

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def _per_joint(self, value, name: str) -> np.ndarray:
        """Broadcast a scalar gain, or validate a per-joint list, to shape (n,)."""
        arr = np.asarray(value, dtype=float)
        if arr.ndim == 0:
            return np.full(self._n, float(arr))
        if arr.shape != (self._n,):
            raise ValueError(f"{name} has {arr.size} values, embodiment expects {self._n}")
        return arr.copy()

    def _import_sdk(self):
        """Import the SDK lazily and return the handful of symbols we need."""
        require_sdk(__package__, "UnitreeRobot")
        _, ChannelPublisher, ChannelSubscriber = channel_api("UnitreeRobot")
        from unitree_sdk2py.utils.crc import CRC

        if self._msg_family == "hg":
            from unitree_sdk2py.idl.default import (
                unitree_hg_msg_dds__LowCmd_ as LowCmdDefault,
            )
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
                BmsState_,
                LowCmd_,
                LowState_,
                MainBoardState_,
            )
        else:
            from unitree_sdk2py.idl.default import (
                unitree_go_msg_dds__LowCmd_ as LowCmdDefault,
            )
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_

            BmsState_ = MainBoardState_ = None  # the go family has no separate topics

        return {
            "ChannelPublisher": ChannelPublisher,
            "ChannelSubscriber": ChannelSubscriber,
            "CRC": CRC,
            "LowCmdDefault": LowCmdDefault,
            "LowCmd_": LowCmd_,
            "LowState_": LowState_,
            "BmsState_": BmsState_,
            "MainBoardState_": MainBoardState_,
        }

    def connect(self) -> None:
        sdk = self._import_sdk()
        init_channel("UnitreeRobot", self._domain_id, self._network_interface)

        if not self._read_only and self._builtin_skills is None:
            self._publisher = sdk["ChannelPublisher"](_LOWCMD_TOPIC, sdk["LowCmd_"])
            self._publisher.Init()
        self._subscriber = sdk["ChannelSubscriber"](_LOWSTATE_TOPIC, sdk["LowState_"])
        self._subscriber.Init(self._on_state, 10)
        if sdk["BmsState_"] is not None:
            # Health topics are best-effort: get_status reports None until they arrive.
            self._bms_subscriber = sdk["ChannelSubscriber"](_BMS_TOPIC, sdk["BmsState_"])
            self._bms_subscriber.Init(self._on_bms, 10)
            self._board_subscriber = sdk["ChannelSubscriber"](
                _MAINBOARD_TOPIC, sdk["MainBoardState_"])
            self._board_subscriber.Init(self._on_board, 10)

        self._low_cmd = sdk["LowCmdDefault"]()
        self._crc = sdk["CRC"]()

        # Wait for the first state so we know mode_machine before commanding.
        token = self._stop_token()
        for _ in range(500):
            if self._low_state is not None:
                break
            time.sleep(0.01)
            self._check_stop(token)
        if self._low_state is None:
            raise TimeoutError("No LowState received; check the network interface and that the robot is on.")
        if self._release_motion_mode:
            self._release_builtin_motion()
        self._connected = True

    @property
    def raw_state(self):
        """The latest raw SDK ``LowState`` message (read-only diagnostics), or ``None``."""
        return self._low_state

    @property
    def motor_index(self) -> list:
        """SDK motor slot of each joint, in embodiment order (for reading ``raw_state``)."""
        return list(self._motor_index)

    def motion_mode(self) -> Optional[dict]:
        """Query (read-only) which built-in motion service is active, if any.

        Returns the motion switcher's reply, e.g. ``{"form": "0", "name": "ai"}``;
        an empty ``name`` means nothing holds ``rt/lowcmd``. Returns ``None`` if
        the switcher did not answer.
        """
        code, result = self._motion_switcher().CheckMode()
        return result if code == 0 else None

    def _motion_switcher(self):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        client = MotionSwitcherClient()
        client.SetTimeout(5.0)
        client.Init()
        return client

    def _release_builtin_motion(self, attempts: int = 10) -> None:
        """Release the built-in motion service so low-level commands take effect."""
        client = self._motion_switcher()
        for _ in range(attempts):
            code, result = client.CheckMode()
            if code == 0 and not (result or {}).get("name"):
                return
            client.ReleaseMode()
            time.sleep(1.0)
        raise RuntimeError(
            f"Built-in motion service still active after {attempts} release attempts: {result!r}"
        )

    def _on_bms(self, msg) -> None:
        self._bms_state = msg

    def _on_board(self, msg) -> None:
        self._board_state = msg

    def _on_state(self, msg) -> None:
        self._low_state_stamp = self._state_stamps.tick()
        self._low_state = msg
        # mode_machine is reported by the robot; echo it back in every command.
        self._mode_machine = int(getattr(msg, "mode_machine", 0))

    def reset(self, seed: Optional[int] = None) -> None:
        """Ramp from the measured pose to hold it, bringing the robot under PD.

        This never jumps: it reads the current joint pose and commands it with a
        ratio ramping 0->1 so gains engage smoothly. It does not move the robot
        to a named home pose — that is a policy/skill decision. With
        ``builtin_skills`` the built-in service owns the motors, so this does nothing.

        An :meth:`estop` from another thread ends the ramp at the next step
        (:class:`~hirobot.robot.RobotStoppedError`); the robot is left damped.
        """
        if self._read_only:
            raise RuntimeError("UnitreeRobot is read_only; reset() would command the robot")
        if self._builtin_skills is not None:
            return  # the built-in service already holds the robot; nothing to ramp
        if self._low_state is None:
            return
        token = self._stop_token()
        start = np.array([self._low_state.motor_state[i].q for i in self._motor_index])
        steps = max(1, int(self._ramp_seconds * self._embodiment.control_hz))
        period = 1.0 / self._embodiment.control_hz if self._embodiment.control_hz else 0.0
        for s in range(steps + 1):
            ratio = min(1.0, s / steps)
            with self._commanding(token):
                self._write_joint_command(start, self._kp * ratio, self._kd, dq=None, tau=None)
            if period:
                self._wait(period, token)

    def get_observation(self) -> Observation:
        if self._low_state is None:
            raise RuntimeError("No state yet; call connect() first.")
        s, stamp = self._low_state, self._low_state_stamp
        obs = Observation(
            joint_pos=np.array([s.motor_state[i].q for i in self._motor_index]),
            joint_vel=np.array([s.motor_state[i].dq for i in self._motor_index]),
            joint_torque=np.array([s.motor_state[i].tau_est for i in self._motor_index]),
        )
        if self._embodiment.floating_base:
            imu = s.imu_state
            w, x, y, z = imu.quaternion  # the SDK reports [w, x, y, z]
            obs.base_orientation = np.array([x, y, z, w])
            obs.base_angular_velocity = np.array(imu.gyroscope)
            obs.base_linear_acceleration = np.array(imu.accelerometer)
        if stamp is not None:
            obs.stamps["body"] = stamp
        return obs

    def get_status(self) -> RobotStatus:
        """Battery, motor temperatures / faults and main-board temperature.

        Units are converted to SI here (the hg BMS reports mV / mA). The
        battery is ``None`` until its first message (hg) or state (go) arrives.
        """
        status = RobotStatus(timestamp=time.time(), fault=self.fault)
        s = self._low_state
        if s is not None:
            motors = [s.motor_state[i] for i in self._motor_index]
            names = self._embodiment.joint_names
            if self._msg_family == "hg":
                # A motor reporting all-zero temperatures is not reporting (on the
                # G1 the locked waist roll / pitch slots read 0): NaN, not 0 degC.
                status.joint_temperature = np.array(
                    [max(m.temperature) if any(m.temperature) else np.nan for m in motors],
                    dtype=float)
                status.joint_faults = {
                    n: int(m.motorstate) for n, m in zip(names, motors) if m.motorstate}
                status.extra["motor_voltage"] = np.array([m.vol for m in motors], dtype=float)
            else:
                status.joint_temperature = np.array(
                    [m.temperature if m.temperature else np.nan for m in motors], dtype=float)
                status.battery = _go_battery(s)
        if self._bms_state is not None:
            status.battery = _hg_battery(self._bms_state)
            cells = [v for v in self._bms_state.cell_vol if v]
            status.extra["battery_cell_voltage"] = np.array(cells, dtype=float) / 1000.0
        b = self._board_state
        if b is not None:
            status.board_temperature["mainboard"] = float(b.temperature[0])
            status.extra["mainboard_value"] = [float(v) for v in b.value]
            status.extra["mainboard_fan_state"] = [int(v) for v in b.fan_state]
        return status

    # ------------------------------------------------------------ robot skills

    def _skill_backend(self):
        """The built-in skill client, or ``None`` before connect / without skills."""
        if self._builtin_skills is None or not self._connected:
            return None
        if self._skills is None:
            from hirobot.drivers.body.unitree.sdk2.g1_skills import G1Skills

            self._skills = G1Skills()
        return self._skills

    @property
    def has_robot_skills(self) -> bool:
        return self._builtin_skills is not None

    def list_robot_skills(self) -> List[RobotSkillSpec]:
        """The G1 skills hiRobot supports: a fixed list, known without a robot."""
        if self._builtin_skills is None:
            return []
        from hirobot.drivers.body.unitree.sdk2.g1_skills import SKILLS

        return [s.model_copy(deep=True) for s in SKILLS]

    def _run_robot_skill(self, spec: RobotSkillSpec, args: Dict[str, Any]) -> None:
        backend = self._skill_backend()
        if backend is None:
            raise RobotSkillError("connect() the robot before running a robot skill")
        backend.run(spec, args)

    def stop_robot_skill(self) -> None:
        backend = self._skill_backend()
        if backend is not None:
            backend.stop()

    def hold(self) -> None:
        """Hold the measured pose; with ``builtin_skills`` the motion service keeps
        the robot standing itself, so this does nothing (joint commands would be refused).
        """
        if self._builtin_skills is not None:
            return
        super().hold()

    def _send_action(self, action: Action) -> None:
        if self._builtin_skills is not None:
            raise SafetyError(_BUILTIN_OWNS_LOWCMD)
        if action.joint_target is None:
            return
        target = np.asarray(action.joint_target, dtype=float)
        kp = self._kp if action.kp is None else np.asarray(action.kp, dtype=float)
        kd = self._kd if action.kd is None else np.asarray(action.kd, dtype=float)

        if action.mode in (ControlMode.POSITION, ControlMode.PD):
            self._write_joint_command(target, kp, kd, dq=None, tau=None)
        elif action.mode == ControlMode.VELOCITY:
            self._write_joint_command(None, kp, kd, dq=target, tau=None)
        elif action.mode == ControlMode.TORQUE:
            # Pure torque: zero gains so only the feed-forward torque applies.
            self._write_joint_command(None, np.zeros(self._n), np.zeros(self._n), dq=None, tau=target)
        else:
            raise ValueError(f"UnitreeRobot does not support mode {action.mode!r}")

    def _write_joint_command(self, q, kp, kd, dq, tau) -> None:
        """Fill the per-motor command fields, CRC it, and publish one LowCmd."""
        if self._builtin_skills is not None:
            raise RuntimeError(_BUILTIN_OWNS_LOWCMD)
        if self._publisher is None:
            raise RuntimeError("UnitreeRobot has no lowcmd publisher (read_only or not connected)")
        cmd = self._low_cmd
        # hg exposes mode_pr/mode_machine on the command; go needs a frame header.
        if self._msg_family == "hg":
            cmd.mode_machine = self._mode_machine
        else:
            cmd.head[0], cmd.head[1] = 0xFE, 0xEF
            cmd.level_flag = 0xFF
            cmd.gpio = 0
        for i, slot in enumerate(self._motor_index):
            m = cmd.motor_cmd[slot]
            m.mode = 1 if self._msg_family == "hg" else self._go_mode[i]  # enable
            m.q = float(q[i]) if q is not None else 0.0
            m.dq = float(dq[i]) if dq is not None else 0.0
            m.tau = float(tau[i]) if tau is not None else 0.0
            m.kp = float(kp[i])
            m.kd = float(kd[i])
        cmd.crc = self._crc.Crc(cmd)
        self._publisher.Write(cmd)

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if kp is not None:
            self._kp = self._per_joint(kp, "kp")
        if kd is not None:
            self._kd = self._per_joint(kd, "kd")

    def damp(self) -> None:
        """Command damping mode: kp = 0, kd = ``damping_kd``, no torque.

        The robot goes limp but resists fast motion. This is the safe state to
        leave hardware in; it bypasses the safety gate because it carries no
        position target. With ``builtin_skills`` there is no lowcmd publisher
        and this does nothing; :meth:`estop` then stops the skill instead.
        """
        if self._publisher is None:
            return
        zeros = np.zeros(self._n)
        self._write_joint_command(zeros, zeros, np.full(self._n, self._damping_kd), None, None)

    def _estop(self) -> None:
        """Damp; with ``builtin_skills``, stop walking and release the arms instead."""
        if self._builtin_skills is not None:
            self.stop_robot_skill()
        else:
            self.damp()

    def disconnect(self) -> None:
        # Leave the robot damped rather than holding the last PD target.
        if self._connected:
            self.damp()
        self._connected = False
        # The DDS entities are torn down with the process; nothing to close here.


def _hg_battery(bms) -> BatteryState:
    """``unitree_hg`` ``BmsState_``: voltage in mV, current in mA, temperatures in degC."""
    temps = [t for t in bms.temperature if t]
    volts = [v for v in bms.bmsvoltage if v]
    return BatteryState(
        soc=float(bms.soc),
        voltage=volts[0] / 1000.0 if volts else None,
        current=bms.current / 1000.0,
        temperature=float(max(temps)) if temps else None,
        health=float(bms.soh),
        cycles=int(bms.cycle),
    )


def _go_battery(state) -> BatteryState:
    """``unitree_go`` ``LowState_``: the BMS block plus the pack's power_v / power_a."""
    bms = state.bms_state
    temps = [t for t in (*bms.bq_ntc, *bms.mcu_ntc) if t]
    return BatteryState(
        soc=float(bms.soc),
        voltage=float(state.power_v) if state.power_v else None,
        current=float(state.power_a),
        temperature=float(max(temps)) if temps else None,
        cycles=int(bms.cycle),
    )
