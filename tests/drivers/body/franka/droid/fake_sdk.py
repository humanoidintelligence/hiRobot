"""Fake ``zerorpc``: a ``Client`` that answers like an upstream DROID robot server.

Only ``Client(timeout, heartbeat)``, ``connect``, ``close`` and calling the
client with a method name are modelled; the server side is :class:`Server`,
which keeps the arm state and records every command.
"""

from __future__ import annotations

import sys
import threading
import types


class Server:
    """What the DROID server reports, and the commands it got (``calls``)."""

    def __init__(self):
        self.q = [0.0, -0.6, 0.0, -2.5, 0.0, 1.9, 0.0]
        # [x, y, z, roll, pitch, yaw]: pointing down (180 deg about x).
        self.cartesian = [0.36, 0.0, 0.37, 3.141592653589793, 0.0, 0.0]
        self.gripper = 0.25          # DROID: 0 open, 1 closed
        self.calls = []              # (method, args, thread name)
        self.connected = []
        self.closed = 0
        self.fail = None             # method name -> exception to raise
        self.lock = threading.Lock()

    def commands(self):
        return [c for c in self.calls if c[0].startswith("update_")]

    def handle(self, method, args):
        with self.lock:
            self.calls.append((method, args, threading.current_thread().name))
        if self.fail and method in self.fail:
            raise self.fail[method]
        if method == "get_robot_state":
            state = {"joint_positions": list(self.q), "joint_velocities": [0.0] * 7,
                     "cartesian_position": list(self.cartesian),
                     "gripper_position": self.gripper}
            return [state, {"robot_timestamp_seconds": 1, "robot_timestamp_nanos": 0}]
        if method == "update_joints":
            q, velocity, blocking = args
            assert velocity is False
            self.q = list(q)
            return None
        if method == "update_pose":
            return None
        if method == "update_gripper":
            command, velocity, blocking = args
            assert velocity is False
            self.gripper = float(command)
            return None
        raise AssertionError(f"unexpected DROID call {method}")


def install(monkeypatch) -> Server:
    server = Server()

    class Client:  # zerorpc's name
        def __init__(self, timeout=30, heartbeat=5):
            self.timeout = timeout

        def connect(self, address):
            server.connected.append(address)

        def close(self):
            server.closed += 1

        def __call__(self, method, *args):
            return server.handle(method, args)

    mod = types.ModuleType("zerorpc")
    mod.Client = Client
    monkeypatch.setitem(sys.modules, "zerorpc", mod)
    return server
