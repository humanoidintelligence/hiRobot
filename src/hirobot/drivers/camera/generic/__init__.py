"""Stand-in cameras with no hardware: ``array`` and ``video_file``."""

from hirobot.drivers.camera.generic.array import ArrayCamera
from hirobot.drivers.camera.generic.video_file import VideoFileCamera

__all__ = ["ArrayCamera", "VideoFileCamera"]
