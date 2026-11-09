"""Fake ``pylibfranka``: a ``Robot`` with ``read_once`` and an async position-control handler."""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import numpy as np


class Arm:
    def __init__(self):
        self.q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        # O_T_EE as libfranka sends it: 4x4 column-major. Gripper pointing
        # down (180 deg about x), 0.3 m forward and 0.5 m up.
        t = np.diag([1.0, -1.0, -1.0, 1.0])
        t[:3, 3] = [0.3, 0.0, 0.5]
        self.o_t_ee = t
        self.calls = []
        self.targets = []


def install(monkeypatch):
    arm = Arm()

    class Handler:
        def set_joint_position_target(self, q):
            arm.targets.append(list(q))

        def stop(self):
            arm.calls.append(("handler.stop",))

    class Robot:
        def __init__(self, ip, rt):
            arm.calls.append(("Robot", ip, rt))

        def set_collision_behavior(self, *limits):
            arm.calls.append(("set_collision_behavior",))

        def automatic_error_recovery(self):
            arm.calls.append(("automatic_error_recovery",))

        def start_async_position_control(self, maximum_joint_velocities, goal_tolerance):
            arm.calls.append(("start_async_position_control", list(maximum_joint_velocities),
                              goal_tolerance))
            return Handler()

        def read_once(self):
            return SimpleNamespace(q=list(arm.q), dq=[0.0] * 7, tau_J=[0.5] * 7,
                                   O_T_EE=list(arm.o_t_ee.flatten(order="F")))

    mod = types.ModuleType("pylibfranka")
    mod.Robot = Robot
    mod.RealtimeConfig = SimpleNamespace(kEnforce="enforce", kIgnore="ignore")
    monkeypatch.setitem(sys.modules, "pylibfranka", mod)
    return arm
