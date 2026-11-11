"""A minimal in-process simulator driver.

This is not a physics engine — it is a trivial integrator that holds a joint
state and applies commands, enough to exercise the unified interface end to end
without hardware. Real drivers (Unitree, Fourier, AgiBot, Franka, UR) implement
the same :class:`~hirobot.core.robot.Robot` contract against their SDKs; a proper
sim driver would wrap MuJoCo / Isaac / URSim behind this same interface.

With ``mobile_base=True`` (config ``body: {driver: toy, mobile_base: true}``,
for an embodiment with ``floating_base: true``) the toy is also a mobile base: it
takes ``BASE_VELOCITY`` (``[vx, vy, wz]`` in the body frame) and integrates it on
the ground plane into an odometry pose (``base_position``, yaw-only
``base_orientation``), which it reports in every observation. ``joints: []``
makes it a base with no joints at all (a wheeled platform). Without the flag a
floating-base toy reports only a fixed ``base_orientation`` (no odometry).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.loading.registry import register_body
from hirobot.core.robot import Robot
from hirobot.core.stamps import Stamp
from hirobot.core.types import Action, ControlMode, Observation
from hirobot.control.watchdog import DEFAULT_COMMAND_TIMEOUT_S, CommandWatchdog


@register_body("toy")
class ToyRobot(Robot):
    """A toy kinematic simulator that satisfies the Robot contract.

    Args:
        embodiment: The embodiment to simulate.
        dt: Integration timestep; defaults to the embodiment's control period.
        command_timeout_s: Simulated seconds without a new ``VELOCITY`` /
            ``BASE_VELOCITY`` command before :meth:`_idle_step` stops
            integrating it (velocity set to zero); ``None`` turns it off. Needs
            ``dt > 0``.
        mobile_base: Take ``BASE_VELOCITY`` and report odometry; needs an
            embodiment with ``floating_base: true``.

    A mobile-base toy starts at the origin of its odometry frame, facing +x,
    and :meth:`reset` puts it back there. Each ``BASE_VELOCITY`` command (and
    each :meth:`_idle_step` after one) moves it for one ``dt`` along the exact
    arc of a constant ``[vx, vy, wz]``, so ``N`` equal commands land where the
    closed-form motion does. Any joint-mode command stops the base.
    """

    simulated = True

    def __init__(self, embodiment: Embodiment, dt: Optional[float] = None,
                 command_timeout_s: Optional[float] = DEFAULT_COMMAND_TIMEOUT_S,
                 mobile_base: bool = False) -> None:
        if mobile_base and not embodiment.floating_base:
            raise ValueError(f"toy: mobile_base needs an embodiment with floating_base: true; "
                             f"{embodiment.name!r} has a fixed base")
        self._embodiment = embodiment
        self.mobile_base = bool(mobile_base)
        n = embodiment.num_dofs
        self._dt = dt if dt is not None else (1.0 / embodiment.control_hz if embodiment.control_hz else 0.0)
        self._joint_pos = np.zeros(n)
        self._joint_vel = np.zeros(n)
        self._kp: Optional[np.ndarray] = None
        self._kd: Optional[np.ndarray] = None
        self._connected = False
        # Mobile base (mobile_base only): [x, y, z] and yaw in the odometry
        # frame, and the body-frame [vx, vy, wz] being followed.
        self._base_pos = np.zeros(3)
        self._base_yaw = 0.0
        self._base_cmd = np.zeros(3)
        self._last_mode: Optional[ControlMode] = None
        #: How many times :meth:`_idle_step` ran (for tests).
        self.idle_steps = 0
        #: Steps taken since connect; simulated time is ``ticks * dt``.
        self._ticks = 0
        self.command_timeout_s = command_timeout_s if self._dt else None
        self._watchdog = CommandWatchdog(type(self).__name__, self.command_timeout_s,
                                         self._stop_velocity, clock=self.now)

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    @property
    def action_modes(self):  # type: ignore[override]
        # The joint modes of the base class; a mobile base also drives itself.
        modes = Robot.action_modes
        return (*modes, ControlMode.BASE_VELOCITY) if self.mobile_base else modes

    def connect(self) -> None:
        self._connected = True
        self._ticks = 0

    def now(self) -> float:
        """Simulated time: steps taken times ``dt`` (with ``dt`` 0, the step count)."""
        return self._ticks * self._dt if self._dt else float(self._ticks)

    def reset(self, seed: Optional[int] = None) -> None:
        self._joint_pos = np.zeros(self._embodiment.num_dofs)
        self._joint_vel = np.zeros(self._embodiment.num_dofs)
        self._base_pos = np.zeros(3)
        self._base_yaw = 0.0
        self._base_cmd = np.zeros(3)
        if self._last_mode == ControlMode.BASE_VELOCITY:
            self._last_mode = None
            self._watchdog.disarm()

    def get_observation(self) -> Observation:
        obs = Observation(
            joint_pos=self._joint_pos.copy(),
            joint_vel=self._joint_vel.copy(),
        )
        if self.mobile_base:
            half = self._base_yaw / 2
            c, s = np.cos(self._base_yaw), np.sin(self._base_yaw)
            vx, vy, wz = self._base_cmd
            obs.base_position = self._base_pos.copy()
            obs.base_orientation = np.array([0.0, 0.0, np.sin(half), np.cos(half)])
            # Linear velocity in the odometry frame (as MuJoCo's free joint), the
            # angular one in the body frame (as an IMU); about z only, so equal.
            obs.base_linear_velocity = np.array([c * vx - s * vy, s * vx + c * vy, 0.0])
            obs.base_angular_velocity = np.array([0.0, 0.0, wz])
        elif self._embodiment.floating_base:
            obs.base_orientation = np.array([0.0, 0.0, 0.0, 1.0])
            obs.base_angular_velocity = np.zeros(3)
        obs.stamps["body"] = Stamp(self.now(), self._ticks + 1)
        return obs

    def _send_action(self, action: Action) -> None:
        """Apply a command by stepping the toy integrator forward one ``dt``."""
        if action.mode == ControlMode.BASE_VELOCITY and self.mobile_base:
            cmd = None if action.base_velocity is None else np.asarray(action.base_velocity,
                                                                       dtype=float)
            if cmd is None or cmd.shape != (3,):
                raise ValueError("BASE_VELOCITY needs base_velocity = [vx, vy, wz]")
            self._base_cmd = cmd.copy()
            self._last_mode = action.mode
            self._ticks += 1
            self._watchdog.feed(action.mode)
            self._step_base()
            return
        if action.joint_target is None:
            return
        target = np.asarray(action.joint_target)
        self._last_mode = action.mode
        self._ticks += 1
        self._watchdog.feed(action.mode)
        self._base_cmd = np.zeros(3)  # a joint command stops the base

        if action.mode in (ControlMode.POSITION, ControlMode.PD):
            prev = self._joint_pos
            self._joint_pos = target.copy()
            self._joint_vel = (self._joint_pos - prev) / self._dt if self._dt else np.zeros_like(target)
        elif action.mode == ControlMode.VELOCITY:
            self._joint_vel = target.copy()
            self._joint_pos = self._joint_pos + self._joint_vel * self._dt
        elif action.mode == ControlMode.TORQUE:
            # Treat torque as acceleration with unit inertia — purely illustrative.
            self._joint_vel = self._joint_vel + target * self._dt
            self._joint_pos = self._joint_pos + self._joint_vel * self._dt

    def _idle_step(self) -> None:
        """One ``dt`` under the last command: a velocity keeps integrating, a position stays."""
        self._watchdog.check()
        if self._last_mode == ControlMode.VELOCITY:
            self._joint_pos = self._joint_pos + self._joint_vel * self._dt
        elif self._last_mode == ControlMode.BASE_VELOCITY:
            self._step_base()
        elif self._last_mode in (ControlMode.POSITION, ControlMode.PD):
            self._joint_vel = np.zeros_like(self._joint_vel)
        self.idle_steps += 1
        self._ticks += 1

    def _step_base(self) -> None:
        """Move the base for one ``dt`` along the arc of the current ``[vx, vy, wz]``."""
        vx, vy, wz = self._base_cmd
        dt = self._dt
        y0 = self._base_yaw
        y1 = y0 + wz * dt
        if abs(wz * dt) > 1e-9:
            # Integral of the body-frame velocity rotated by yaw(t) = y0 + wz t.
            si, co = (np.sin(y1) - np.sin(y0)) / wz, (np.cos(y0) - np.cos(y1)) / wz
        else:
            si, co = np.cos(y0) * dt, np.sin(y0) * dt
        self._base_pos = self._base_pos + np.array([vx * si - vy * co, vx * co + vy * si, 0.0])
        self._base_yaw = float(np.arctan2(np.sin(y1), np.cos(y1)))

    def _stop_velocity(self, mode: ControlMode) -> None:
        self._joint_vel = np.zeros_like(self._joint_vel)
        self._base_cmd = np.zeros(3)

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        self._kp = None if kp is None else np.asarray(kp)
        self._kd = None if kd is None else np.asarray(kd)

    def disconnect(self) -> None:
        self._connected = False
