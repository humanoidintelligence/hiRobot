"""Universal Robots arm driver (UR5 / UR10 / e-Series).

Wraps the ``ur_rtde`` library (``RTDEControlInterface`` + ``RTDEReceiveInterface``)
behind the :class:`~hirobot.robot.Robot` contract. UR arms are 6-DoF, fixed
base, so the embodiment has ``floating_base=False`` and no IMU/base fields.

Why ``ur_rtde`` and not Universal Robots' bare ``RTDE_Python_Client_Library``:
that library is the low-level RTDE *protocol* (register read/write),
which requires shipping a companion URScript program that runs on the teach
pendant and calls ``servoj`` from input registers. ``ur_rtde`` wraps exactly
that pattern and exposes ``servoJ``/``moveJ``/``speedJ`` directly, which is what
a driver wants. It is imported lazily; install with ``pip install ur_rtde``.

Streaming joint targets at the control rate uses ``servoJ`` (a non-blocking
servo command); ``reset()`` moves to home with an asynchronous ``moveJ`` and
waits for it, so :meth:`~hirobot.robot.Robot.estop` can stop the move part way
(``stopJ``). Not yet run on a real arm.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot
from hirobot.stamps import StampCounter
from hirobot.sdk import require_sdk
from hirobot.transforms import pose_from_axis_angle, pose_to_axis_angle
from hirobot.types import Action, ControlMode, Observation


@register_body("ur_rtde")
class URRobot(Robot):
    """Driver for Universal Robots arms over RTDE.

    Args:
        embodiment: The 6-joint arm embodiment.
        ip: Robot controller IP address.
        home_q: Optional home joint configuration (rad) for :meth:`reset`.
        home_speed / home_acceleration: ``moveJ`` speed (rad/s) and
            acceleration (rad/s^2) for the move to ``home_q``.
        stop_deceleration: ``stopJ`` deceleration (rad/s^2) for :meth:`estop`.
        servo_velocity / servo_acceleration: ``servoJ`` kinematic limits.
        servo_lookahead / servo_gain: ``servoJ`` smoothing/tracking parameters.
    """

    action_modes = (
        ControlMode.POSITION, ControlMode.PD, ControlMode.VELOCITY, ControlMode.CARTESIAN
    )

    def __init__(
        self,
        embodiment: Embodiment,
        ip: str = "127.0.0.1",
        home_q: Optional[list] = None,
        servo_velocity: float = 0.5,
        servo_acceleration: float = 0.5,
        servo_lookahead: float = 0.1,
        servo_gain: int = 300,
        home_speed: float = 0.5,
        home_acceleration: float = 0.5,
        stop_deceleration: float = 2.0,
    ) -> None:
        self._embodiment = embodiment
        self._ip = ip
        self._home_q = home_q
        self._servo_velocity = servo_velocity
        self._servo_acceleration = servo_acceleration
        self._servo_lookahead = servo_lookahead
        self._servo_gain = servo_gain
        self._home_speed = home_speed
        self._home_acceleration = home_acceleration
        self._stop_deceleration = stop_deceleration
        self._control = None
        self._receive = None
        self._stamps = StampCounter()

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def connect(self) -> None:
        require_sdk(__package__, "URRobot")
        try:
            from rtde_control import RTDEControlInterface
            from rtde_receive import RTDEReceiveInterface
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "URRobot needs the ur_rtde package; install it with "
                "'pip install ur_rtde'."
            ) from exc
        self._receive = RTDEReceiveInterface(self._ip)
        self._control = RTDEControlInterface(self._ip)

    def reset(self, seed: Optional[int] = None) -> None:
        """Move to the configured home pose, if one is set, and wait until it is there.

        An :meth:`estop` from another thread stops the move (``stopJ``) and this
        raises :class:`~hirobot.robot.RobotStoppedError`.
        """
        if self._control is None or self._home_q is None:
            return
        token = self._stop_token()
        with self._commanding(token):
            self._control.moveJ(list(self._home_q), self._home_speed, self._home_acceleration, True)
        while self._control.getAsyncOperationProgress() >= 0:  # < 0: no move running
            self._wait(0.01, token)
        self._check_stop(token)

    def _estop(self) -> None:
        """Decelerate to a stop: leave servo / speed mode, then ``stopJ`` any ``moveJ``."""
        if self._control is None:
            return
        first: Optional[BaseException] = None
        for stop in (self._control.servoStop, self._control.speedStop,
                     lambda: self._control.stopJ(self._stop_deceleration, False)):
            try:
                stop()
            except Exception as exc:  # noqa: BLE001 - try every stop
                first = first or exc
        if first is not None:
            raise first

    def get_observation(self) -> Observation:
        if self._receive is None:
            raise RuntimeError("Not connected; call connect() first.")
        r = self._receive
        obs = Observation(
            joint_pos=np.asarray(r.getActualQ()),        # rad
            joint_vel=np.asarray(r.getActualQd()),       # rad/s
            joint_torque=np.asarray(r.getActualCurrent()),  # motor current (proxy)
        )
        # RTDE reports the TCP as [x, y, z, Rx, Ry, Rz] (m + axis-angle rad) in
        # the base frame; ee_poses carries the 7-vector [x, y, z, qx, qy, qz, qw].
        obs.ee_poses = {"tcp": pose_from_axis_angle(r.getActualTCPPose())}
        # RTDE keeps the newest state; this is when it was read.
        obs.stamps["body"] = self._stamps.tick()
        return obs

    def _send_action(self, action: Action) -> None:
        if self._control is None:
            raise RuntimeError("Not connected; call connect() first.")
        if action.mode in (ControlMode.POSITION, ControlMode.PD):
            if action.joint_target is None:
                return
            # Non-blocking servo to the joint target at the control rate.
            self._control.servoJ(
                list(np.asarray(action.joint_target, dtype=float)),
                self._servo_velocity,
                self._servo_acceleration,
                1.0 / self._embodiment.control_hz if self._embodiment.control_hz else 0.008,
                self._servo_lookahead,
                self._servo_gain,
            )
        elif action.mode == ControlMode.VELOCITY:
            if action.joint_target is None:
                return
            self._control.speedJ(
                list(np.asarray(action.joint_target, dtype=float)),
                self._servo_acceleration,
            )
        elif action.mode == ControlMode.CARTESIAN:
            tcp = action.ee_targets.get("tcp")
            if tcp is None:
                return
            # Non-blocking Cartesian servo to a TCP pose (7-vector -> RTDE's axis-angle).
            self._control.servoL(
                list(pose_to_axis_angle(tcp)),
                self._servo_velocity,
                self._servo_acceleration,
                1.0 / self._embodiment.control_hz if self._embodiment.control_hz else 0.008,
                self._servo_lookahead,
                self._servo_gain,
            )
        else:
            raise ValueError(f"URRobot does not support mode {action.mode!r}")

    def disconnect(self) -> None:
        if self._control is not None:
            self._control.servoStop()
            self._control.stopScript()
            self._control.disconnect()
            self._control = None
        if self._receive is not None:
            self._receive.disconnect()
            self._receive = None
