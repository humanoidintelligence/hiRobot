"""Fake unitree_sdk2py for the Dex3 hands (``unitree_handcmd``): HandCmd_ / HandState_ by topic."""

from __future__ import annotations

import sys
import types


class _Motor:
    def __init__(self):
        self.mode = 0
        self.q = self.dq = self.tau = self.kp = self.kd = 0.0


class _HandCmd:
    def __init__(self):
        self.motor_cmd = [_Motor() for _ in range(7)]


class _MotorState:
    def __init__(self, q):
        self.q, self.dq, self.tau_est = q, 0.0, 1674.594


class _Press:
    def __init__(self, v):
        self.pressure = [v] * 12


class _HandState:
    def __init__(self, q):
        self.motor_state = [_MotorState(float(x)) for x in q]
        self.press_sensor_state = [_Press(30000.0 + i) for i in range(9)]


class _Bus:
    """Holds every publisher's messages and every subscriber's callback by topic."""

    def __init__(self):
        self.sent = {}
        self.subs = {}
        self.publishers = []

    def publish_state(self, topic, q):
        self.subs[topic](_HandState(q))


def _install_fake_sdk(monkeypatch, bus, initial=None):
    initial = initial or {}

    class Publisher:
        def __init__(self, topic, _type):
            self.topic = topic
            bus.publishers.append(topic)

        def Init(self):
            bus.sent.setdefault(self.topic, [])

        def Write(self, msg):
            bus.sent[self.topic].append([(m.mode, m.q, m.kp, m.kd) for m in msg.motor_cmd])

    class Subscriber:
        def __init__(self, topic, _type):
            self.topic = topic

        def Init(self, cb, _depth):
            bus.subs[self.topic] = cb
            if self.topic in initial:
                cb(_HandState(initial[self.topic]))

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a, **k: None
    channel.ChannelPublisher = Publisher
    channel.ChannelSubscriber = Subscriber
    default = types.ModuleType("unitree_sdk2py.idl.default")
    default.unitree_hg_msg_dds__HandCmd_ = _HandCmd
    dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    dds.HandCmd_, dds.HandState_ = _HandCmd, _HandState
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.idl",
                 "unitree_sdk2py.idl.unitree_hg", "unitree_sdk2py.idl.unitree_hg.msg"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.default", default)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.idl.unitree_hg.msg.dds_", dds)
