"""Fake G1 voice service (``AudioClient``) for ``unitree_voice_speaker``."""

from __future__ import annotations

import sys
import time
import types

from hirobot.devices.audio import (
    SAMPLE_WIDTH,
)

RATE = 16000


class _FakeVoice:
    """Stands in for the robot's voice service; records every call."""

    def __init__(self, volume=100, code=0):
        self.volume = volume
        self.code = code
        self.calls = []
        self.streams = []

    def install(self, monkeypatch):
        voice = self

        class AudioClient:
            def SetTimeout(self, t):
                voice.calls.append(("SetTimeout", t))

            def Init(self):
                voice.calls.append(("Init",))

            def GetVolume(self):
                return voice.code, {"volume": voice.volume}

            def SetVolume(self, v):
                voice.calls.append(("SetVolume", v))
                voice.volume = v
                return voice.code

            def PlayStream(self, app, stream_id, pcm):
                voice.streams.append((time.monotonic(), app, stream_id, len(pcm)))
                return voice.code, None

            def PlayStop(self, app):
                voice.calls.append(("PlayStop", app))
                return 0

        channel = types.ModuleType("unitree_sdk2py.core.channel")
        channel.ChannelFactoryInitialize = lambda *a, **k: voice.calls.append(("DDS", a))
        client = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
        client.AudioClient = AudioClient
        for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.g1",
                     "unitree_sdk2py.g1.audio"):
            monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
        monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
        monkeypatch.setitem(sys.modules, "unitree_sdk2py.g1.audio.g1_audio_client", client)
        return self

    def sent_seconds(self):
        return sum(n for *_, n in self.streams) / (SAMPLE_WIDTH * RATE)
