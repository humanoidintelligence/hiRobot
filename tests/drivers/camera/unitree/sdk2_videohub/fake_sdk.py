"""Fake ``unitree_sdk2py.go2.video.video_client.VideoClient`` serving one JPEG per call."""

from __future__ import annotations

import sys
import types

import numpy as np


class Hub:
    def __init__(self, code=0):
        self.code = code
        self.calls = 0
        self.dds = []
        cv2 = __import__("cv2")
        img = np.zeros((48, 64, 3), dtype=np.uint8)
        img[..., 2] = 200  # red in BGR, so the RGB frame has R = 200
        self.jpeg = cv2.imencode(".jpg", img)[1].tobytes()


def install(monkeypatch, code=0):
    hub = Hub(code)

    class VideoClient:
        def SetTimeout(self, t):
            pass

        def Init(self):
            pass

        def GetImageSample(self):
            hub.calls += 1
            return (hub.code, list(hub.jpeg)) if hub.code == 0 else (hub.code, None)

    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelFactoryInitialize = lambda *a: hub.dds.append(a)
    video = types.ModuleType("unitree_sdk2py.go2.video.video_client")
    video.VideoClient = VideoClient
    for name in ("unitree_sdk2py", "unitree_sdk2py.core", "unitree_sdk2py.go2",
                 "unitree_sdk2py.go2.video"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.go2.video.video_client", video)
    return hub
