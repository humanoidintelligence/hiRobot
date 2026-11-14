"""Trossen WidowX 250S over its Dynamixel servos (registered as ``trossen_dynamixel``).

The WidowX 250S is a 6-joint arm with a parallel gripper, built from Dynamixel
X-series servos on a U2D2 USB-to-TTL adapter (``/dev/ttyDXL``). This driver talks
to the servos directly with Robotis' `dynamixel_sdk <https://pypi.org/project/dynamixel-sdk/>`_
(``pip install dynamixel_sdk``), Protocol 2.0 at 1 Mbit/s. It needs no ROS and no
Interbotix stack: it reads and writes the servo control table over the serial bus.

Nine servos, but two are firmware shadows. Interbotix's motor config gives the
shoulder (ID 2) a shadow at ID 3 and the elbow (ID 4) a shadow at ID 5, each with
``Secondary_ID`` set so the shadow mirrors its primary in firmware. So the driver
commands and reads only the six primaries plus the gripper:

    waist 1, shoulder 2, elbow 4, forearm_roll 6, wrist_angle 7, wrist_rotate 8, gripper 9

Positions are the servos' own ticks (0..4095 over one turn), converted to radians
about tick 2048 (``rad = (ticks - 2048) * 2*pi / 4096``). Interbotix sets each
servo's ``Drive_Mode`` reverse bit in firmware so every joint already counts in its
URDF-positive direction, so the driver uses one conversion for all of them, no
per-joint sign.

``connect()`` opens the port, pings the servos and reads their state; it enables
no torque and sends no goal, so reading never moves the arm. Torque is enabled on
the first :meth:`send_action`, not before (holding the then-current position, so
the arm does not jump). ``estop()`` writes the measured positions as the goal, so
the arm holds where it is; it does not enable torque (a limp arm stays limp).

The gripper (ID 9) is read as an opening in ``[0, 1]`` (0 closed, 1 open) by a
linear map between its closed and open ticks (``gripper_close_tick`` /
``gripper_open_tick``, measured per robot). Commanding it moves the servo to
position control (the Interbotix stack runs it on PWM) and writes the mapped goal.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.core.robot import Robot
from hirobot.core.stamps import StampCounter
from hirobot.core.types import Action, ControlMode, Observation, RobotStatus
from hirobot.loading.registry import register_body
from hirobot.loading.sdk import require_sdk

PROTOCOL = 2.0
BAUD = 1_000_000

# X-series control table (Protocol 2.0): address, length in bytes.
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_PWM = 100
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126   # 2 bytes, signed; Present_Current/Velocity/Position are
ADDR_PRESENT_VELOCITY = 128  # 4 bytes, signed; contiguous 126..135, read in one block
ADDR_PRESENT_POSITION = 132  # 4 bytes, signed
ADDR_PRESENT_VOLTAGE = 144   # 2 bytes, 0.1 V
ADDR_PRESENT_TEMPERATURE = 146  # 1 byte, degC
ADDR_HARDWARE_ERROR = 70     # 1 byte
LEN_GOAL_POSITION = 4
PRESENT_BLOCK = (ADDR_PRESENT_CURRENT, 10)  # current(2) + velocity(4) + position(4)

MODE_POSITION = 3
TORQUE_ON = 1
TORQUE_OFF = 0

TICKS_PER_REV = 4096
CENTER_TICK = 2048
# Present_Velocity unit is 0.229 rev/min per tick on XM430 / XL430.
VEL_RAD_PER_S = 0.229 * 2.0 * math.pi / 60.0


def rad_to_tick(rad: float) -> int:
    return int(round(float(rad) * TICKS_PER_REV / (2.0 * math.pi))) + CENTER_TICK


def tick_to_rad(tick: int) -> float:
    return (int(tick) - CENTER_TICK) * (2.0 * math.pi) / TICKS_PER_REV


@register_body("trossen_dynamixel")
class WidowXRobot(Robot):
    """A Trossen WidowX 250S arm and its gripper over ``dynamixel_sdk``.

    Args:
        embodiment: The 6-joint arm embodiment; its joints are in servo order
            (waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate).
            Its first end-effector with a gripper is the ``Observation.gripper``
            / ``Action.gripper`` key.
        port: Serial device the U2D2 is on (``/dev/ttyDXL``, the follower arm).
        servo_ids: Dynamixel IDs of the six arm joints, in embodiment order.
            The shoulder and elbow shadows (IDs 3, 5) are not listed: they
            follow their primaries in firmware.
        gripper: Read and command the gripper. ``False`` leaves it alone and
            reports no gripper.
        gripper_id: Dynamixel ID of the gripper servo.
        gripper_close_tick: Servo position (ticks) at which the gripper reads /
            commands fully closed (``opening`` 0).
        gripper_open_tick: Servo position (ticks) at which it is fully open
            (``opening`` 1).
        feedback_timeout_s: How long ``connect()`` waits for every servo to
            answer a ping before it raises.
        network_interface: Ignored (filled in by ``make_robot`` for hardware).
    """

    action_modes = (ControlMode.POSITION,)

    def __init__(
        self,
        embodiment: Embodiment,
        port: str = "/dev/ttyDXL",
        servo_ids: tuple = (1, 2, 4, 6, 7, 8),
        gripper: bool = True,
        gripper_id: int = 9,
        gripper_close_tick: int = 1780,
        gripper_open_tick: int = 2950,
        feedback_timeout_s: float = 2.0,
        network_interface: Optional[str] = None,
    ) -> None:
        self._embodiment = embodiment
        self._ids = [int(i) for i in servo_ids]
        if embodiment.num_dofs != len(self._ids):
            raise ValueError(
                f"WidowXRobot drives {len(self._ids)} joints; the embodiment has "
                f"{embodiment.num_dofs}")
        self.port = str(port)
        self._gripper_id = int(gripper_id)
        if gripper_close_tick == gripper_open_tick:
            raise ValueError("gripper_close_tick and gripper_open_tick must differ")
        self._g_close = int(gripper_close_tick)
        self._g_open = int(gripper_open_tick)
        self._feedback_timeout_s = float(feedback_timeout_s)
        ees = list(embodiment.end_effectors)
        self._ee = ees[0].name if ees else "ee"
        with_gripper = [ee.name for ee in ees if ee.has_gripper]
        self._gripper = (with_gripper[0] if with_gripper else self._ee) if gripper else None
        self._sdk: Any = None
        self._port_handler: Any = None
        self._packet_handler: Any = None
        self._reader: Any = None       # GroupSyncRead over the present-state block
        self._torque_on = False
        self._connected = False
        self._stamps = StampCounter()

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        require_sdk(__package__, "WidowXRobot")
        try:
            import dynamixel_sdk as dxl
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "WidowXRobot needs dynamixel_sdk; install it with 'pip install dynamixel_sdk'."
            ) from exc
        self._sdk = dxl
        ph = dxl.PortHandler(self.port)
        if not ph.openPort():
            raise ConnectionError(f"could not open {self.port}")
        if not ph.setBaudRate(BAUD):
            ph.closePort()
            raise ConnectionError(f"could not set {BAUD} baud on {self.port}")
        self._port_handler = ph
        self._packet_handler = dxl.PacketHandler(PROTOCOL)
        self._reader = dxl.GroupSyncRead(ph, self._packet_handler, *PRESENT_BLOCK)
        for sid in self._all_ids():
            self._reader.addParam(sid)
        self._connected = True
        deadline = time.monotonic() + self._feedback_timeout_s
        missing = self._ping_missing()
        while missing and time.monotonic() < deadline:
            time.sleep(0.05)
            missing = self._ping_missing()
        if missing:
            self.disconnect()
            raise ConnectionError(
                f"no answer from servo(s) {missing} on {self.port} within "
                f"{self._feedback_timeout_s} s; is the arm powered and the U2D2 connected?")

    def disconnect(self) -> None:
        if self._port_handler is not None:
            try:
                self._port_handler.closePort()
            except Exception:  # noqa: BLE001 - closing anyway
                pass
        self._port_handler = None
        self._packet_handler = None
        self._reader = None
        self._torque_on = False
        self._connected = False

    def _all_ids(self) -> List[int]:
        return self._ids + ([self._gripper_id] if self._gripper is not None else [])

    def _require(self) -> Any:
        if not self._connected or self._packet_handler is None:
            raise RuntimeError("Not connected; call connect() first.")
        return self._packet_handler

    def _ping_missing(self) -> List[int]:
        pk, ph = self._require(), self._port_handler
        missing = []
        for sid in self._all_ids():
            _, res, _ = pk.ping(ph, sid)
            if res != self._sdk.COMM_SUCCESS:
                missing.append(sid)
        return missing

    # ------------------------------------------------------------ reading

    def _read_present(self) -> Dict[int, Dict[str, int]]:
        """One GroupSyncRead of current / velocity / position for every servo (signed)."""
        reader, dxl = self._reader, self._sdk
        if reader.txRxPacket() != dxl.COMM_SUCCESS:
            raise ConnectionError(f"GroupSyncRead failed on {self.port}")
        out: Dict[int, Dict[str, int]] = {}
        for sid in self._all_ids():
            cur = _signed(reader.getData(sid, ADDR_PRESENT_CURRENT, 2), 16)
            vel = _signed(reader.getData(sid, ADDR_PRESENT_VELOCITY, 4), 32)
            pos = _signed(reader.getData(sid, ADDR_PRESENT_POSITION, 4), 32)
            out[sid] = {"current": cur, "velocity": vel, "position": pos}
        return out

    def get_observation(self) -> Observation:
        present = self._read_present()
        stamp = self._stamps.tick()
        pos = np.array([tick_to_rad(present[i]["position"]) for i in self._ids], dtype=float)
        vel = np.array([present[i]["velocity"] * VEL_RAD_PER_S for i in self._ids], dtype=float)
        obs = Observation(joint_pos=pos, joint_vel=vel)
        if self._gripper is not None:
            opening = (present[self._gripper_id]["position"] - self._g_close) / float(
                self._g_open - self._g_close)
            obs.gripper[self._gripper] = float(np.clip(opening, 0.0, 1.0))
            obs.stamps[self._gripper] = stamp
        obs.stamps["body"] = stamp
        return obs

    def get_status(self) -> RobotStatus:
        status = RobotStatus(timestamp=time.time(), fault=self.fault)
        if not self._connected:
            return status
        pk, ph = self._require(), self._port_handler
        temps, errors, enabled = [], {}, []
        names = self._embodiment.joint_names
        for name, sid in zip(names, self._ids):
            temp, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_PRESENT_TEMPERATURE)
            temps.append(float(temp))
            hw, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_HARDWARE_ERROR)
            if hw:
                errors[name] = int(hw)
            torque, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE)
            enabled.append(bool(torque))
        volt, _, _ = pk.read2ByteTxRx(ph, self._ids[0], ADDR_PRESENT_VOLTAGE)
        status.joint_temperature = np.array(temps, dtype=float)
        status.extra["enabled"] = enabled
        status.extra["voltage_v"] = float(volt) / 10.0
        if errors:
            status.extra["hardware_error"] = errors
        return status

    # ------------------------------------------------------------ commands

    def reset(self, seed: Optional[int] = None) -> None:
        """Nothing to move: the arm stays where it is (there is no home pose to go to)."""

    def _enable(self) -> None:
        """Enable torque on all arm servos, holding the current position first.

        Writes each servo's measured position as its goal before the torque
        comes on, so enabling does not snap the arm to a stale goal.
        """
        if self._torque_on:
            return
        pk, ph = self._require(), self._port_handler
        present = self._read_present()
        for sid in self._ids:
            pk.write1ByteTxRx(ph, sid, ADDR_OPERATING_MODE, MODE_POSITION)
            self._write_goal(sid, present[sid]["position"])
            pk.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, TORQUE_ON)
        if self._gripper is not None:
            pk.write1ByteTxRx(ph, self._gripper_id, ADDR_OPERATING_MODE, MODE_POSITION)
            self._write_goal(self._gripper_id, present[self._gripper_id]["position"])
            pk.write1ByteTxRx(ph, self._gripper_id, ADDR_TORQUE_ENABLE, TORQUE_ON)
        self._torque_on = True

    def _write_goal(self, sid: int, tick: int) -> None:
        pk, ph = self._require(), self._port_handler
        pk.write4ByteTxRx(ph, sid, ADDR_GOAL_POSITION, _unsigned(int(tick), 32))

    def _estop(self) -> None:
        """Hold the measured positions. Does not enable torque: a limp arm stays limp."""
        if not self._connected or not self._torque_on:
            return
        present = self._read_present()
        for sid in self._all_ids():
            self._write_goal(sid, present[sid]["position"])

    def _send_action(self, action: Action) -> None:
        if action.mode != ControlMode.POSITION:
            raise ValueError(f"WidowXRobot does not support mode {action.mode!r}")
        opening = action.gripper.get(self._gripper) if (action.gripper and self._gripper) else None
        if opening is not None:
            opening = float(opening)
            if not 0.0 <= opening <= 1.0:
                raise ValueError(f"gripper {self._gripper!r}: {opening} is outside [0, 1]")
        if action.joint_target is None and opening is None:
            return
        self._enable()
        if action.joint_target is not None:
            q = np.asarray(action.joint_target, dtype=float)
            for sid, rad in zip(self._ids, q):
                self._write_goal(sid, rad_to_tick(rad))
        if opening is not None:
            tick = self._g_close + opening * (self._g_open - self._g_close)
            self._write_goal(self._gripper_id, int(round(tick)))


def _signed(value: int, bits: int) -> int:
    if value >= 1 << (bits - 1):
        value -= 1 << bits
    return value


def _unsigned(value: int, bits: int) -> int:
    return value + (1 << bits) if value < 0 else value
