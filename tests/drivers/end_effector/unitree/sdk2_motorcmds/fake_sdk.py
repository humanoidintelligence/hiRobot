"""Fake unitree_sdk2py for ``unitree_motorcmds``: one MotorCmds_ / MotorStates_ bus per
topic pair."""

from __future__ import annotations

import sys
import types

from hirobot.drivers.end_effector.unitree.sdk2_motorcmds import driver as motorcmds


class _MotorCmd:
    def __init__(self):
        self.mode, self.q, self.dq, self.tau, self.kp, self.kd = 0, 0.0, 0.0, 0.0, 0.0, 0.0


class _MotorCmds:
    def __init__(self, cmds=None):
        self.cmds = cmds if cmds is not None else []


class _MotorState:
    def __init__(self, q, lost=0):
        self.q = q
        self.lost = lost


class _MotorStates:
    def __init__(self, qs, lost=0):
        self.states = [_MotorState(q, lost) for q in qs]


class _Bus:
    """Fake unitree_sdk2py: one state publisher per topic, a log of sent commands."""

    def __init__(self):
        self.sent = []  # (topic, [12 q])
        self.callbacks = {}

    def publish_state(self, topic, msg):
        for cb in self.callbacks.get(topic, []):
            cb(msg)


def _install(monkeypatch, state=None, state_topic="rt/inspire/state"):
    """Install a fake unitree_sdk2py. ``state`` is the MotorStates_ every subscriber gets."""
    bus = _Bus()

    class Publisher:
        def __init__(self, topic, _type):
            self.topic = topic

        def Init(self):
            pass

        def Write(self, msg):
            bus.sent.append((self.topic, [m.q for m in msg.cmds]))

    class Subscriber:
        def __init__(self, topic, _type):
            self.topic = topic

        def Init(self, cb, _depth):
            bus.callbacks.setdefault(self.topic, []).append(cb)
            if state is not None and self.topic == state_topic:
                cb(state)

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a, **k: None
    channel.ChannelPublisher = Publisher
    channel.ChannelSubscriber = Subscriber
    default = types.ModuleType("unitree_sdk2py.idl.default")
    default.unitree_go_msg_dds__MotorCmd_ = _MotorCmd
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    dds.MotorCmds_ = _MotorCmds
    dds.MotorStates_ = _MotorStates
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_go", "unitree_sdk2py.idl.unitree_go.msg"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    # The manifest also requires cyclonedds (unitree_sdk2py's DDS layer); stand in
    # for it so the tests do not depend on whether it happens to be installed.
    monkeypatch.setitem(sys.modules, "cyclonedds", types.ModuleType("cyclonedds"))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.default", default)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_go.msg.dds_", dds)
    # Buses are cached per topic pair across drivers; start each test clean.
    monkeypatch.setattr(motorcmds, "_BUSES", {})
    return bus
