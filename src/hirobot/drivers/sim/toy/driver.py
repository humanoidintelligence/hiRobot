"""A minimal in-process simulator driver.

This is not a physics engine — it is a trivial integrator that holds a joint
state and applies commands, enough to exercise the unified interface end to end
without hardware. Real drivers (Unitree, Fourier, AgiBot, Franka, UR) implement
the same :class:`~hirobot.robot.Robot` contract against their SDKs; a proper
sim driver would wrap MuJoCo / Isaac / URSim behind this same interface.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot
from hirobot.stamps import Stamp
from hirobot.types import Action, ControlMode, Observation


@register_body("toy")
class ToyRobot(Robot):
    """A toy kinematic simulator that satisfies the Robot contract.

    Args:
        embodiment: The embodiment to simulate.
        dt: Integration timestep; defaults to the embodiment's control period.
    """

    simulated = True

    def __init__(self, embodiment: Embodiment, dt: Optional[float] = None) -> None:
        self._embodiment = embodiment
        n = embodiment.num_dofs
        self._dt = dt if dt is not None else (1.0 / embodiment.control_hz if embodiment.control_hz else 0.0)
        self._joint_pos = np.zeros(n)
        self._joint_vel = np.zeros(n)
        self._kp: Optional[np.ndarray] = None
        self._kd: Optional[np.ndarray] = None
        self._connected = False
        self._last_mode: Optional[ControlMode] = None
        #: How many times :meth:`idle_step` ran (for tests).
        self.idle_steps = 0
        #: Steps taken since connect; simulated time is ``ticks * dt``.
        self._ticks = 0

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def connect(self) -> None:
        self._connected = True
        self._ticks = 0

    def now(self) -> float:
        """Simulated time: steps taken times ``dt`` (with ``dt`` 0, the step count)."""
        return self._ticks * self._dt if self._dt else float(self._ticks)

    def reset(self, seed: Optional[int] = None) -> None:
        self._joint_pos = np.zeros(self._embodiment.num_dofs)
        self._joint_vel = np.zeros(self._embodiment.num_dofs)

    def get_observation(self) -> Observation:
        obs = Observation(
            joint_pos=self._joint_pos.copy(),
            joint_vel=self._joint_vel.copy(),
        )
        if self._embodiment.floating_base:
            obs.base_orientation = np.array([0.0, 0.0, 0.0, 1.0])
            obs.base_angular_velocity = np.zeros(3)
        obs.stamps["body"] = Stamp(self.now(), self._ticks + 1)
        return obs

    def _send_action(self, action: Action) -> None:
        """Apply a command by stepping the toy integrator forward one ``dt``."""
        if action.joint_target is None:
            return
        target = np.asarray(action.joint_target)
        self._last_mode = action.mode
        self._ticks += 1

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

    def idle_step(self) -> None:
        """One ``dt`` under the last command: a velocity keeps integrating, a position stays."""
        if self._last_mode == ControlMode.VELOCITY:
            self._joint_pos = self._joint_pos + self._joint_vel * self._dt
        elif self._last_mode in (ControlMode.POSITION, ControlMode.PD):
            self._joint_vel = np.zeros_like(self._joint_vel)
        self.idle_steps += 1
        self._ticks += 1

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        self._kp = None if kp is None else np.asarray(kp)
        self._kd = None if kd is None else np.asarray(kd)

    def disconnect(self) -> None:
        self._connected = False
