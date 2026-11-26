"""Record a robot's episodes as LeRobot v3.0 datasets (``pip install 'hirobot[record]'``).

:class:`Recorder` listens to the outermost robot (``robot.add_listener``),
keeps the raw samples of an episode between :meth:`~Recorder.start` and
:meth:`~Recorder.stop`, and saves each stopped episode on a background thread:
aligned to a frame grid, H.264 videos, one dataset directory per episode, and
the quality flags in ``meta/qc.jsonl``. :func:`append_verdict` adds a
reviewer's verdict to that file.

This package imports without pyarrow and PyAV; :class:`Recorder` needs both
and raises ``ImportError`` naming the extra when one is missing.
"""

from hirobot.record.align import QcFlag
from hirobot.record.qc import VERDICTS, append_verdict, read_qc
from hirobot.record.recorder import MAX_QUEUE, Recorder, RecorderError, SaveEvent, Subscription

__all__ = ["MAX_QUEUE", "QcFlag", "Recorder", "RecorderError", "SaveEvent", "Subscription",
           "VERDICTS", "append_verdict", "read_qc"]
