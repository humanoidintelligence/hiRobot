"""The shared Unitree DDS channel initialisation."""

from __future__ import annotations

import sys
import types

import pytest


def test_unitree_channel_is_initialised_once_and_warns_on_another_nic(monkeypatch, caplog):
    from hirobot.drivers._sdk import unitree_sdk2

    calls = []
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a: calls.append(a)
    for name in ("unitree_sdk2py", "unitree_sdk2py.core"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    unitree_sdk2.init_channel("body", 0, "eth0")
    unitree_sdk2.init_channel("hand", 0, "eth0")
    with caplog.at_level("WARNING"):
        unitree_sdk2.init_channel("camera", 0, "eth1")
    assert "already runs on eth0" in caplog.text

    def broken(*a):
        raise Exception("channel factory init error.")

    unitree_sdk2._reset_for_tests()
    channel.ChannelFactoryInitialize = broken
    with pytest.raises(RuntimeError, match="network_interface='nope'"):
        unitree_sdk2.init_channel("body", 0, "nope")
