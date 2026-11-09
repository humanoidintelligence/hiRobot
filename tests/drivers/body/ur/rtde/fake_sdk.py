"""Fake ``ur_rtde``: ``RTDEReceiveInterface`` serves set state, ``RTDEControlInterface``
records every call."""

from __future__ import annotations

import sys
import types

import numpy as np


class Arm:
    """Shared state of one fake controller: what the receive side reports, what control got."""

    def __init__(self):
        self.q = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.tcp = [0.4, 0.1, 0.3, 0.0, 3.14, 0.0]
        self.calls = []
        #: getAsyncOperationProgress values still to report for the running moveJ;
        #: empty means it has finished (-1).
        self.progress = []


def install(monkeypatch):
    arm = Arm()

    class RTDEReceiveInterface:
        def __init__(self, ip):
            arm.calls.append(("receive", ip))

        def getActualQ(self):
            return list(arm.q)

        def getActualQd(self):
            return [0.0] * 6

        def getActualCurrent(self):
            return [0.1] * 6

        def getActualTCPPose(self):
            return list(arm.tcp)

        def disconnect(self):
            arm.calls.append(("receive.disconnect",))

    class RTDEControlInterface:
        def __init__(self, ip):
            arm.calls.append(("control", ip))

        def getAsyncOperationProgress(self):
            return arm.progress.pop(0) if arm.progress else -1

        def stopJ(self, a, asynchronous):
            arm.calls.append(("stopJ", a, asynchronous))
            arm.progress.clear()
            return True

        def __getattr__(self, name):
            def call(*args):
                arm.calls.append((name, *[np.asarray(a).tolist() if isinstance(a, list) else a
                                          for a in args]))
                return True

            return call

    recv = types.ModuleType("rtde_receive")
    recv.RTDEReceiveInterface = RTDEReceiveInterface
    ctrl = types.ModuleType("rtde_control")
    ctrl.RTDEControlInterface = RTDEControlInterface
    monkeypatch.setitem(sys.modules, "rtde_receive", recv)
    monkeypatch.setitem(sys.modules, "rtde_control", ctrl)
    return arm
