"""Franka arm driver (FR3 / Panda).

Wraps ``pylibfranka`` (the official libfranka pybind bindings, see
https://github.com/frankarobotics/libfranka) behind the :class:`~hirobot.core.robot.Robot`
contract. Franka arms are 7-DoF, fixed base, so ``floating_base=False``.

libfranka requires a 1 kHz real-time control loop on a PREEMPT_RT kernel for its
streaming controllers. To keep hiRobot's soft-real-time loop usable, this
driver drives the arm through an asynchronous position-control handler
(``AsyncPositionControlHandler``): each :meth:`send_action` sets a new joint
target that libfranka's own RT thread tracks, so a VLA policy can issue targets
at its own (lower) rate. The SDK is imported lazily.

Safety notes:
* Collision thresholds are set on connect; tune them for your payload.
* ``reset()`` runs ``automatic_error_recovery`` to clear reflex/faults.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.loading.registry import register_body
from hirobot.core.robot import Robot
from hirobot.core.stamps import StampCounter
from hirobot.loading.sdk import require_sdk
from hirobot.geometry.transforms import pose_from_matrix
from hirobot.core.types import Action, ControlMode, Observation


@register_body("franka_libfranka")
class FrankaRobot(Robot):
    """Driver for Franka arms via pylibfranka.

    Args:
        embodiment: The 7-joint arm embodiment.
        ip: Robot IP / hostname (the FCI address).
        max_joint_velocity: Per-joint velocity cap for the async handler (rad/s).
        goal_tolerance: Position tolerance for considering a target reached (rad).
        realtime: Pass ``False`` to relax ``RealtimeConfig`` when not on an RT
            kernel (useful for setup; not for production control).
    """

    action_modes = (ControlMode.POSITION, ControlMode.PD)

    def __init__(
        self,
        embodiment: Embodiment,
        ip: str = "172.16.0.2",
        max_joint_velocity: float = 1.0,
        goal_tolerance: float = 0.01,
        realtime: bool = True,
    ) -> None:
        self._embodiment = embodiment
        self._ip = ip
        self._max_joint_velocity = max_joint_velocity
        self._goal_tolerance = goal_tolerance
        self._realtime = realtime
        self._robot = None
        self._handler = None
        self._stamps = StampCounter()

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def connect(self) -> None:
        require_sdk(__package__, "FrankaRobot")
        try:
            import pylibfranka
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "FrankaRobot needs the pylibfranka package; install it with "
                "`pip install pylibfranka` or build it from libfranka."
            ) from exc

        rt = (
            pylibfranka.RealtimeConfig.kEnforce
            if self._realtime
            else pylibfranka.RealtimeConfig.kIgnore
        )
        self._robot = pylibfranka.Robot(self._ip, rt)
        # Conservative collision behavior; tune per payload.
        self._robot.set_collision_behavior(
            [20.0] * 7, [20.0] * 7, [10.0] * 6, [10.0] * 6
        )
        self._pylibfranka = pylibfranka

    def reset(self, seed: Optional[int] = None) -> None:
        """Clear any reflex/fault state so the arm is commandable."""
        if self._robot is not None:
            self._robot.automatic_error_recovery()
            # (Re)start the async position controller fresh.
            self._handler = self._robot.start_async_position_control(
                maximum_joint_velocities=[self._max_joint_velocity] * 7,
                goal_tolerance=self._goal_tolerance,
            )

    def get_observation(self) -> Observation:
        if self._robot is None:
            raise RuntimeError("Not connected; call connect() first.")
        st = self._robot.read_once()
        obs = Observation(
            joint_pos=np.asarray(st.q),       # rad
            joint_vel=np.asarray(st.dq),      # rad/s
            joint_torque=np.asarray(st.tau_J),  # Nm
        )
        # O_T_EE is a 4x4 column-major homogeneous transform in the base frame;
        # ee_poses carries the 7-vector [x, y, z, qx, qy, qz, qw].
        o_t_ee = np.asarray(st.O_T_EE, dtype=float).reshape(4, 4, order="F")
        obs.ee_poses = {"ee": pose_from_matrix(o_t_ee)}
        obs.stamps["body"] = self._stamps.tick()  # read_once() just returned
        return obs

    def _send_action(self, action: Action) -> None:
        if self._robot is None:
            raise RuntimeError("Not connected; call connect() first.")
        if self._handler is None:
            raise RuntimeError("Call reset() to start the position controller first.")
        if action.mode in (ControlMode.POSITION, ControlMode.PD):
            if action.joint_target is None:
                return
            # Hand the new target to libfranka's RT thread; it tracks it.
            self._handler.set_joint_position_target(
                list(np.asarray(action.joint_target, dtype=float))
            )
        else:
            raise ValueError(
                f"FrankaRobot (async position control) does not support mode "
                f"{action.mode!r}; use POSITION/PD."
            )

    def _estop(self) -> None:
        """Stop the async position controller, so the arm tracks no more targets.

        :meth:`reset` starts a new one. What the arm does once the controller
        stops has not been checked on a real FR3.
        """
        handler, self._handler = self._handler, None
        if handler is not None:
            handler.stop()

    def disconnect(self) -> None:
        if self._handler is not None:
            self._handler.stop()
            self._handler = None
        # The Robot object releases the FCI connection on destruction.
        self._robot = None
