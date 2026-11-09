"""Fake channel API and inspire_sdkpy message types for ``inspire_sdkpy``."""

from __future__ import annotations

import sys
import types

from hirobot.drivers.end_effector.inspire.sdkpy import driver as sdkpy


class _Ctrl:
    def __init__(self, pos_set, angle_set, force_set, speed_set, mode):
        self.pos_set, self.angle_set, self.force_set = pos_set, angle_set, force_set
        self.speed_set, self.mode = speed_set, mode


class _HandState:
    def __init__(self, angle_act, pos_act=None, err=None):
        self.angle_act = angle_act
        self.pos_act = pos_act if pos_act is not None else [1500] * 6
        self.err = err if err is not None else [0] * 6


def _install_sdkpy(monkeypatch, states):
    """Fake channel API plus fake inspire_sdkpy types. ``states`` maps state topic -> message."""
    sent = []
    callbacks = {}

    class Publisher:
        def __init__(self, topic, _type):
            self.topic = topic

        def Init(self):
            pass

        def Write(self, msg):
            sent.append((self.topic, msg))

    class Subscriber:
        def __init__(self, topic, _type):
            self.topic = topic

        def Init(self, cb, _depth):
            callbacks[self.topic] = cb
            if self.topic in states:
                cb(states[self.topic])

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a, **k: None
    channel.ChannelPublisher = Publisher
    channel.ChannelSubscriber = Subscriber
    for name in ("unitree_sdk2py", "unitree_sdk2py.core"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    # The manifest also requires cyclonedds (unitree_sdk2py's DDS layer); stand in
    # for it so the tests do not depend on whether it happens to be installed.
    monkeypatch.setitem(sys.modules, "cyclonedds", types.ModuleType("cyclonedds"))
    monkeypatch.setattr(sdkpy, "_TYPES", (_Ctrl, _HandState))
    return sent, callbacks
