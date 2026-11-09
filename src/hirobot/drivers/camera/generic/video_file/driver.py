"""A video file played back as a camera.

Lets a simulated robot (or a robot whose camera is not wired up) see recorded
footage: frames come out on the video's own timeline, as a live camera would
deliver them. Decoded with PyAV (``video`` extra, imported on ``start()``)::

    cameras:
      - name: head
        driver:
          type: video_file
          path: assets/kitchen.mp4
          fps: 5          # frames sampled per second of video
          loop: true
"""

from __future__ import annotations

import time
from typing import Any, Iterator, Optional, Tuple

import numpy as np

from hirobot.camera import PollingCameraDriver, register_camera
from hirobot.sdk import require_sdk


@register_camera("video_file")
class VideoFileCamera(PollingCameraDriver):
    """Frames from a video file, sampled at ``fps`` and paced to the video's clock.

    Args:
        name: Camera name.
        path: Video file (anything PyAV decodes).
        fps: Frames taken per second of video.
        loop: Start over at the end. Without it the last frame stays as the
            latest one once the file ends.
    """

    #: Not real hardware: ``make_robot`` attaches it to simulated drivers too.
    simulated = True

    def __init__(self, name: str, path: str, fps: float = 5.0, loop: bool = True) -> None:
        if fps <= 0:
            raise ValueError(f"fps must be positive, got {fps}")
        # Poll a bit faster than the frame rate; _grab sleeps until each frame is due.
        super().__init__(name, poll_hz=max(2.0 * fps, 1.0))
        self.path = path
        self.fps = float(fps)
        self.loop = loop
        self.ended = False
        self._container: Any = None
        self._frames: Optional[Iterator[Tuple[float, np.ndarray]]] = None
        self._start = 0.0  # host time of video t = 0 in the current pass
        self._last: Optional[np.ndarray] = None

    def _open(self) -> None:
        require_sdk(__package__, "VideoFileCamera")
        self._restart()

    def _restart(self) -> None:
        import av

        self._close_container()
        self._container = av.open(self.path)
        self._frames = self._decode(self._container)
        self._start = time.monotonic()

    def _decode(self, container: Any) -> Iterator[Tuple[float, np.ndarray]]:
        stream = container.streams.video[0]
        step = 1.0 / self.fps
        next_t = 0.0
        for frame in container.decode(stream):
            if frame.time is None or frame.time + 1e-6 < next_t:
                continue
            yield next_t, frame.to_ndarray(format="rgb24")
            next_t += step

    def _grab(self) -> np.ndarray:
        item = next(self._frames, None) if self._frames is not None else None
        if item is None:
            if self.loop and self._last is not None:
                self._restart()
                item = next(self._frames, None)
            if item is None:
                # The file is over (or empty): keep showing the last frame.
                self.ended = True
                self._close_container()
                self._frames = None
                if self._last is None:
                    raise RuntimeError(f"video {self.path!r} has no frames")
                time.sleep(1.0 / self.fps)
                return self._last
        t, image = item
        delay = self._start + t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._last = image
        return image

    def _close_container(self) -> None:
        if self._container is not None:
            self._container.close()
            self._container = None

    def _close(self) -> None:
        self._close_container()
        self._frames = None
