"""Fake ``cosine_bus``: image subscribers that hand out packets a test pushes."""

from __future__ import annotations

import sys
import types
from enum import Enum
from typing import Dict, List, Optional

import numpy as np


class PacketType(Enum):
    IMAGE = 1


class EncodingFormat(Enum):
    UNCOMPRESSED = 0
    JPEG = 1
    PNG = 2


class ColorFormat(Enum):
    RGB = 0
    BGR = 1
    RS2_FORMAT_Z16 = 15


class QosReliability(Enum):
    BEST_EFFORT = 0
    RELIABLE = 1


class Packet:
    def __init__(self, data: np.ndarray, width: int, height: int, seq: int, source_ns: int,
                 encoding=EncodingFormat.JPEG, color=ColorFormat.RGB):
        self.type = PacketType.IMAGE
        self.encoding_format = encoding
        self.color_format = color
        self.image_width = width
        self.image_height = height
        self.sequence_number = seq
        self.source_timestamp = source_ns
        self.send_timestamp = source_ns + 1_000_000
        self.recv_timestamp = 0
        self._data = data

    def get_image_data(self) -> np.ndarray:
        return self._data


class Bus:
    """What the fake subscribers read: packets per topic, newest last."""

    def __init__(self):
        self.packets: Dict[str, List[Packet]] = {}
        self.subscribed: List[str] = []

    def push(self, topic: str, packet: Packet) -> None:
        self.packets.setdefault(topic, []).append(packet)

    def color(self, topic: str, seq: int, source_ns: int, rgb=(200, 10, 10),
              size=(64, 48)) -> None:
        """A JPEG colour frame whose pixels are ``rgb`` (encoded as the camera does)."""
        cv2 = __import__("cv2")
        w, h = size
        bgr = np.zeros((h, w, 3), np.uint8)
        bgr[:] = rgb[::-1]
        data = np.frombuffer(cv2.imencode(".jpg", bgr)[1].tobytes(), np.uint8)
        self.push(topic, Packet(data, w, h, seq, source_ns))

    def depth(self, topic: str, seq: int, source_ns: int, raw: int = 1500,
              size=(64, 48)) -> None:
        w, h = size
        z = np.full((h, w), raw, np.uint16)
        z[0, 0] = 0  # one pixel without a reading
        self.push(topic, Packet(np.frombuffer(z.tobytes(), np.uint8), w, h, seq, source_ns,
                                encoding=EncodingFormat.UNCOMPRESSED,
                                color=ColorFormat.RS2_FORMAT_Z16))


def install(monkeypatch, env: bool = True) -> Bus:
    bus = Bus()

    class image_subscriber:  # the SDK's name
        def __init__(self, topic, qos, depth):
            self.topic = topic
            bus.subscribed.append(topic)

        def read_latest_frame(self) -> Optional[Packet]:
            frames = bus.packets.get(self.topic)
            return frames[-1] if frames else None

        def read_frame_nearest(self, stamp_ns: int) -> Optional[Packet]:
            frames = bus.packets.get(self.topic)
            if not frames:
                return None
            return min(frames, key=lambda p: abs(p.source_timestamp - stamp_ns))

    mod = types.ModuleType("cosine_bus")
    mod.PacketType = PacketType
    mod.EncodingFormat = EncodingFormat
    mod.ColorFormat = ColorFormat
    mod.QosReliability = QosReliability
    mod.image_subscriber = image_subscriber
    monkeypatch.setitem(sys.modules, "cosine_bus", mod)
    if env:
        monkeypatch.setenv("LOCATOR_IP", "192.0.2.1")
        monkeypatch.setenv("AORTA_DISCOVERY_URI", "http://192.0.2.1:2379")
    else:
        monkeypatch.delenv("LOCATOR_IP", raising=False)
        monkeypatch.delenv("AORTA_DISCOVERY_URI", raising=False)
    return bus
