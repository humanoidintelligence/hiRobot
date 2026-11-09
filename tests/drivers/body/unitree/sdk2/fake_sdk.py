"""Fake ``unitree_sdk2py`` for the ``unitree_sdk2`` body driver.

A DDS stand-in: ``ChannelSubscriber.Init`` hands the subscriber the bus's
current ``LowState`` at once, ``ChannelPublisher.Write`` records a copy of every
``LowCmd`` per motor slot, and ``MotionSwitcherClient`` reports no active
service. Message types are plain Python objects with the fields the driver
reads and writes; hg and go families are both covered.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

NUM_SLOTS = 35  # hg LowCmd_/LowState_ carry 35 motors; go carries 20


class _MotorCmd:
    def __init__(self):
        self.mode, self.q, self.dq, self.tau, self.kp, self.kd = 0, 0.0, 0.0, 0.0, 0.0, 0.0


class _LowCmd:
    def __init__(self, slots):
        self.motor_cmd = [_MotorCmd() for _ in range(slots)]
        self.mode_machine = 0
        self.head = [0, 0]
        self.level_flag = 0
        self.gpio = 0
        self.crc = 0


def low_state(qs, slots=NUM_SLOTS, quat_wxyz=(1.0, 0.0, 0.0, 0.0), mode_machine=5):
    """A LowState with motor ``i`` at ``qs[i]`` (slots past ``qs`` read 0).

    ``slots=20`` gives the go family's message: one temperature per motor and
    the battery inside LowState (``bms_state``, ``power_v``, ``power_a``).
    """
    qs = list(qs) + [0.0] * (slots - len(qs))
    go = slots == 20
    motors = [SimpleNamespace(q=float(q), dq=0.0, tau_est=0.0, vol=48.0, motorstate=0,
                              temperature=30 if go else [30, 29]) for q in qs]
    imu = SimpleNamespace(quaternion=list(quat_wxyz), gyroscope=[0.0, 0.0, 0.0],
                          accelerometer=[0.0, 0.0, 9.81])
    state = SimpleNamespace(motor_state=motors, imu_state=imu, mode_machine=mode_machine)
    if go:
        state.bms_state = SimpleNamespace(soc=80, cycle=3, bq_ntc=[30, 31], mcu_ntc=[33, 0])
        state.power_v, state.power_a = 50.0, -2.0
    return state


class Bus:
    def __init__(self, state):
        self.state = state
        self.subscribers = {}
        self.publishers = []
        self.sent = []  # [(topic, [(slot, mode, q, dq, tau, kp, kd), ...])]
        self.dds = []

    def commands(self, topic="rt/lowcmd"):
        return [m for t, m in self.sent if t == topic]


def install(monkeypatch, state=None):
    """Install the fake; returns the :class:`Bus`. ``state=None`` means no LowState ever arrives."""
    bus = Bus(state)

    class Publisher:
        def __init__(self, topic, _type):
            self.topic = topic
            bus.publishers.append(topic)

        def Init(self):
            pass

        def Write(self, cmd):
            bus.sent.append((self.topic, [(i, m.mode, m.q, m.dq, m.tau, m.kp, m.kd)
                                          for i, m in enumerate(cmd.motor_cmd)]))

    class Subscriber:
        def __init__(self, topic, _type):
            self.topic = topic

        def Init(self, cb, _depth):
            bus.subscribers[self.topic] = cb
            if self.topic == "rt/lowstate" and bus.state is not None:
                cb(bus.state)

    class CRC:
        def Crc(self, cmd):
            return 0xC0FFEE

    class MotionSwitcherClient:
        def SetTimeout(self, t):
            pass

        def Init(self):
            pass

        def CheckMode(self):
            return 0, {"form": "0", "name": ""}

        def ReleaseMode(self):
            return 0

    mods = {}
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a: bus.dds.append(a)
    channel.ChannelPublisher, channel.ChannelSubscriber = Publisher, Subscriber
    mods["unitree_sdk2py.core.channel"] = channel
    crc = types.ModuleType("unitree_sdk2py.utils.crc")
    crc.CRC = CRC
    mods["unitree_sdk2py.utils.crc"] = crc
    default = types.ModuleType("unitree_sdk2py.idl.default")
    default.unitree_hg_msg_dds__LowCmd_ = lambda: _LowCmd(NUM_SLOTS)
    default.unitree_go_msg_dds__LowCmd_ = lambda: _LowCmd(20)
    mods["unitree_sdk2py.idl.default"] = default
    for fam in ("hg", "go"):
        dds = types.ModuleType(f"unitree_sdk2py.idl.unitree_{fam}.msg.dds_")
        dds.LowCmd_ = dds.LowState_ = dds.BmsState_ = dds.MainBoardState_ = object
        mods[f"unitree_sdk2py.idl.unitree_{fam}.msg.dds_"] = dds
    switcher = types.ModuleType("unitree_sdk2py.comm.motion_switcher.motion_switcher_client")
    switcher.MotionSwitcherClient = MotionSwitcherClient
    mods["unitree_sdk2py.comm.motion_switcher.motion_switcher_client"] = switcher
    parents = {".".join(name.split(".")[:i])
               for name in mods for i in range(1, name.count(".") + 1)}
    for name in sorted(parents - set(mods)):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    for name, mod in mods.items():
        monkeypatch.setitem(sys.modules, name, mod)
    return bus
