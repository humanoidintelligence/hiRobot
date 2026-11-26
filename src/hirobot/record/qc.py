"""The quality-control ledger of a recorded dataset: ``meta/qc.jsonl``.

One JSON object per line, appended, never rewritten; the data itself is not
changed. The recorder adds one ``"source": "auto"`` line per episode when it
saves it (the flags it found, ``verdict`` ``"FAIL"`` with flags, else
``"PASS"``); a reviewer (hiApp, a script) adds ``"source": "human"`` lines with
:func:`append_verdict`. Readers merge them: a later human line overrides the
earlier ones and the automatic one.

Needs neither pyarrow nor av.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional, Union

import numpy as np

QC_PATH = "meta/qc.jsonl"

#: Verdicts a reviewer may give.
VERDICTS = ("PASS", "FAIL", "UNREVIEWED")

try:  # POSIX; elsewhere the append is not locked
    import fcntl
except ImportError:  # pragma: no cover - Windows
    fcntl = None  # type: ignore[assignment]


def append_line(dataset_dir: Union[str, Path], line: Dict[str, Any]) -> Dict[str, Any]:
    """Append ``line`` to ``<dataset_dir>/meta/qc.jsonl`` under an exclusive file lock."""
    path = Path(dataset_dir) / QC_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(line, ensure_ascii=False) + "\n"
    with open(path, "a", encoding="utf-8") as f:
        if fcntl is not None:
            fcntl.flock(f.fileno(), fcntl.LOCK_EX)
        try:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        finally:
            if fcntl is not None:
                fcntl.flock(f.fileno(), fcntl.LOCK_UN)
    return line


def append_verdict(dataset_dir: Union[str, Path], episode_index: int, verdict: str,
                   note: str = "", reviewer: Optional[str] = None) -> Dict[str, Any]:
    """Record a reviewer's verdict on one episode of a dataset.

    Appends ``{"episode_index", "source": "human", "verdict", "note",
    "reviewer", "t"}`` to ``<dataset_dir>/meta/qc.jsonl`` (``t``: ``time.time()``)
    and returns it. Safe from several processes at once (the append is
    file-locked on POSIX). Needs no :class:`~hirobot.record.Recorder`.

    Raises:
        ValueError: ``verdict`` is not ``PASS``, ``FAIL`` or ``UNREVIEWED``.
        FileNotFoundError: ``dataset_dir`` has no ``meta/info.json``.
    """
    if verdict not in VERDICTS:
        raise ValueError(f"verdict must be one of {VERDICTS}, got {verdict!r}")
    if not (Path(dataset_dir) / "meta" / "info.json").is_file():
        raise FileNotFoundError(f"{dataset_dir} is not a dataset (no meta/info.json)")
    return append_line(dataset_dir, {
        "episode_index": int(episode_index), "source": "human", "verdict": verdict,
        "note": str(note), "reviewer": reviewer, "t": time.time()})


def read_qc(dataset_dir: Union[str, Path]) -> list:
    """Every line of ``meta/qc.jsonl``, oldest first (empty when there is none)."""
    path = Path(dataset_dir) / QC_PATH
    if not path.is_file():
        return []
    with open(path, encoding="utf-8") as f:
        return [json.loads(s) for s in f if s.strip()]


# ------------------------------------------------------------------ checks


def thumbnail(image: np.ndarray) -> bytes:
    """The 32x24 downsample (every n-th pixel, quantized to 6 bits) of ``image``, as bytes.

    Two frames with the same thumbnail count as the same picture for the
    frozen-frame check. Sensor noise usually changes a few pixels' top bits;
    a stream that repeats one buffer does not.
    """
    h, w = image.shape[:2]
    ys = np.linspace(0, h - 1, 24).astype(int)
    xs = np.linspace(0, w - 1, 32).astype(int)
    small = image[ys][:, xs]
    return (np.asarray(small, dtype=np.uint8) >> 2).tobytes()


def frozen_spans(times: np.ndarray, thumbs: list, min_s: float = 1.0) -> list:
    """``(start, end)`` source times of runs of new readings with an unchanged picture.

    ``times`` and ``thumbs`` are one entry per new reading (its ``seq``
    advanced); a run counts when it lasts at least ``min_s`` seconds.
    """
    spans = []
    i = 0
    n = len(thumbs)
    while i < n:
        j = i
        while j + 1 < n and thumbs[j + 1] == thumbs[i]:
            j += 1
        if j > i and times[j] - times[i] >= min_s - 1e-9:
            spans.append((float(times[i]), float(times[j])))
        i = j + 1
    return spans


__all__ = ["QC_PATH", "VERDICTS", "append_line", "append_verdict", "frozen_spans", "read_qc",
           "thumbnail"]
