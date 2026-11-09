"""Fourier GR-series humanoid driver (GR1T1 / GR1T2).

Wraps ``fourier_grx_client.RobotClient`` behind the
:class:`~hirobot.robot.Robot` contract. The client talks to a GRX server over
Zenoh (default ``server_ip:7447``); GR1T1 and GR1T2 share the same 32-joint API,
so one driver covers both — only the embodiment config differs.

Units: since client v0.2.0 all user-facing values are **radians**. We therefore
read/command in radians and never pass ``degrees=True``.

The SDK is imported lazily; install ``fourier-grx-client`` (``pip install
fourier-grx-client``, https://github.com/FFTAI/fourier-grx-client) to talk to
real hardware.

Safety notes:
* Motors must be enabled (``set_enable(True)``) before commands take effect;
  the client does *not* power down on ``close()``, so ``disconnect()`` here
  disables explicitly.
* For a high-rate control loop use position control (a single lightweight
  publish); do not use the blocking/trajectory ``move_joints`` in the hot loop.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.registry import register_body
from hirobot.robot import Robot
from hirobot.stamps import StampCounter
from hirobot.sdk import require_sdk
from hirobot.types import Action, ControlMode, Observation


@register_body("fourier_grx")
class FourierRobot(Robot):
    """Driver for Fourier GR1 humanoids via the GRX client.

    Args:
        embodiment: The embodiment; joint order must match the GRX joint order
            (see the GR1T1/GR1T2 configs, derived from ``config_GR1_T*.yaml``).
        server_ip: IP of the GRX server (default ``localhost``).
        namespace: Robot namespace; ``None`` lets the client resolve it from the
            ``GRX_ROBOT_ID`` env / ``~/.fourier`` config.
        freq: State refresh frequency the client runs at.
        enable_on_connect: Power the motors on during :meth:`connect`.
    """

    def __init__(
        self,
        embodiment: Embodiment,
        server_ip: str = "localhost",
        namespace: Optional[str] = None,
        freq: int = 400,
        enable_on_connect: bool = True,
    ) -> None:
        self._embodiment = embodiment
        self._server_ip = server_ip
        self._namespace = namespace
        self._freq = freq
        self._enable_on_connect = enable_on_connect
        self._client = None
        self._stamps = StampCounter()

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    def connect(self) -> None:
        require_sdk(__package__, "FourierRobot")
        try:
            from fourier_grx_client import RobotClient
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "FourierRobot needs the fourier-grx-client package; install it "
                "with `pip install fourier-grx-client`."
            ) from exc

        # RobotClient connects to the Zenoh server in its constructor and blocks
        # until the first joint state arrives.
        kwargs = {"freq": self._freq, "server_ip": self._server_ip}
        if self._namespace is not None:
            kwargs["namespace"] = self._namespace
        self._client = RobotClient(**kwargs)
        if self._enable_on_connect:
            self._client.set_enable(True)

    def reset(self, seed: Optional[int] = None) -> None:
        """Enable motors; homing/calibration is left to the operator/skill."""
        if self._client is not None and self._enable_on_connect:
            self._client.set_enable(True)

    def get_observation(self) -> Observation:
        if self._client is None:
            raise RuntimeError("Not connected; call connect() first.")
        c = self._client
        obs = Observation(
            joint_pos=np.asarray(c.joint_positions).copy(),   # rad
            joint_vel=np.asarray(c.joint_velocities).copy(),  # rad/s
            joint_torque=np.asarray(c.joint_efforts).copy(),
        )
        if self._embodiment.floating_base:
            # Passed through as reported: the client (0.2.0a8) documents no component
            # order, and it has not been checked on a GR1 against [x, y, z, w].
            obs.base_orientation = np.asarray(c.imu_quaternion).copy()
            # imu_angular_velocity is reported in deg/s by the client.
            obs.base_angular_velocity = np.deg2rad(np.asarray(c.imu_angular_velocity))
            obs.base_linear_acceleration = np.asarray(c.imu_linear_acceleration).copy()
        # The client keeps the newest state; this is when it was read.
        obs.stamps["body"] = self._stamps.tick()
        return obs

    def _send_action(self, action: Action) -> None:
        if self._client is None:
            raise RuntimeError("Not connected; call connect() first.")
        if action.kp is not None or action.kd is not None:
            self.set_gains(action.kp, action.kd)

        if action.mode in (ControlMode.POSITION, ControlMode.PD):
            if action.joint_target is None:
                return
            self._client.control_joints("position", np.asarray(action.joint_target), degrees=False)
        elif action.mode == ControlMode.VELOCITY:
            if action.joint_target is None:
                return
            self._client.control_joints("velocity", np.asarray(action.joint_target), degrees=False)
        elif action.mode == ControlMode.TORQUE:
            if action.joint_target is None:
                return
            self._client.control_joints("effort", np.asarray(action.joint_target))
        else:
            raise ValueError(f"FourierRobot does not support mode {action.mode!r}")

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if self._client is None:
            raise RuntimeError("Not connected; call connect() first.")
        # Gains must be full 32-length arrays or the client silently ignores them.
        self._client.set_gains(
            pd_control_kp=None if kp is None else np.asarray(kp, dtype=float),
            pd_control_kd=None if kd is None else np.asarray(kd, dtype=float),
        )

    def disconnect(self) -> None:
        if self._client is not None:
            # close() does not power down, so disable motors explicitly first.
            try:
                self._client.set_enable(False)
            finally:
                self._client.close()
            self._client = None
