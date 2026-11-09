"""A stand-in for Unitree's image_server: a local ZeroMQ PUB socket sending one JPEG per message."""

from __future__ import annotations

import struct
import threading
import time

import numpy as np


class FakeImageServer:
    """Publishes ``image`` (HxWx3 BGR) as JPEG on ``tcp://127.0.0.1:<port>`` at ``hz``.

    Args:
        image: The BGR frame to send. Default: 48x64 with R = 200 on the left
            half and B = 200 on the right half, so a crop can be told apart.
        header: Prefix ``struct.pack('dI', time, frame_id)`` like ``Unit_Test``.
        payload: Raw bytes to send instead of a JPEG (to test bad messages).
    """

    def __init__(self, image=None, header=False, payload=None, hz=100.0):
        import cv2
        import zmq

        if image is None:
            image = np.zeros((48, 64, 3), dtype=np.uint8)
            image[:, :32, 2] = 200
            image[:, 32:, 0] = 200
        self._jpeg = payload if payload is not None else cv2.imencode(".jpg", image)[1].tobytes()
        self._header = header
        self._period = 1.0 / hz
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self.port = self._sock.bind_to_random_port("tcp://127.0.0.1")
        self.address = f"tcp://127.0.0.1:{self.port}"
        self.sent = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            msg = self._jpeg
            if self._header:
                msg = struct.pack("dI", time.time(), self.sent) + msg
            self._sock.send(msg)
            self.sent += 1
            self._stop.wait(self._period)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._sock.close(0)
        self._ctx.term()
