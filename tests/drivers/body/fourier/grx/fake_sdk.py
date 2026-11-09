"""Fake ``fourier_grx_client``: a ``RobotClient`` that records calls and serves set state.

Mirrors the attributes and methods ``FourierRobot`` uses from fourier-grx-client
0.2.0a8: ``joint_positions`` / ``joint_velocities`` / ``joint_efforts``, the IMU
fields (angular velocity in deg/s), ``set_enable``, ``control_joints``,
``set_gains`` and ``close``.
"""

from __future__ import annotations

import sys
import types

import numpy as np


class FakeRobotClient:
    def __init__(self, freq=400, server_ip="localhost", namespace=None, num_joints=32):
        self.kwargs = {"freq": freq, "server_ip": server_ip, "namespace": namespace}
        self.calls = []
        n = num_joints
        self.joint_positions = np.linspace(-0.1, 0.1, n)
        self.joint_velocities = np.zeros(n)
        self.joint_efforts = np.zeros(n)
        self.imu_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.imu_angular_velocity = np.array([90.0, 0.0, 0.0])  # deg/s
        self.imu_linear_acceleration = np.array([0.0, 0.0, 9.81])

    def set_enable(self, on):
        self.calls.append(("set_enable", bool(on)))

    def control_joints(self, kind, target, degrees=False):
        self.calls.append(("control_joints", kind, np.asarray(target).copy(), degrees))

    def set_gains(self, pd_control_kp=None, pd_control_kd=None):
        self.calls.append(("set_gains", pd_control_kp, pd_control_kd))

    def close(self):
        self.calls.append(("close",))


def install(monkeypatch, num_joints=32):
    """Install the fake; returns the class (its ``instances`` list holds the clients made)."""
    mod = types.ModuleType("fourier_grx_client")

    class Client(FakeRobotClient):
        instances = []

        def __init__(self, **kwargs):
            super().__init__(num_joints=num_joints, **kwargs)
            Client.instances.append(self)

    mod.RobotClient = Client
    monkeypatch.setitem(sys.modules, "fourier_grx_client", mod)
    return Client
