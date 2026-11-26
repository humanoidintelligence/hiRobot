"""Write one aligned episode as a LeRobot v3.0 dataset directory (pyarrow + PyAV).

The directory holds one episode (``episode_index`` 0) in the v3.0 layout,
each file holding just that episode; merging many into the usual large files
is left to the data tools downstream. Nothing here imports ``lerobot``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from hirobot.record.align import Aligned

CODEBASE_VERSION = "v3.0"
CHUNKS_SIZE = 1000
DATA_FILES_SIZE_IN_MB = 100
VIDEO_FILES_SIZE_IN_MB = 200
DATA_PATH = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
VIDEO_PATH = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
EPISODES_PATH = "meta/episodes/chunk-000/file-000.parquet"
TASKS_PATH = "meta/tasks.parquet"
ALIGNMENT_PATH = "meta/hi/alignment.json"
EPISODE_META_PATH = "meta/hi/episode.json"
#: The recorder's own dataset-level keys (``hi.action_mode``, ``hi.robot_config``, ...):
#: kept out of ``meta/info.json``, whose unknown keys LeRobot drops with a warning.
HI_INFO_PATH = "meta/hi/info.json"

#: Frame columns every LeRobot dataset has (``lerobot.utils.constants.DEFAULT_FEATURES``).
DEFAULT_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": [1], "names": None},
    "frame_index": {"dtype": "int64", "shape": [1], "names": None},
    "episode_index": {"dtype": "int64", "shape": [1], "names": None},
    "index": {"dtype": "int64", "shape": [1], "names": None},
    "task_index": {"dtype": "int64", "shape": [1], "names": None},
}

#: H.264 settings: fast enough for a robot's CPU, playable in a browser.
VIDEO_CODEC = "libx264"
VIDEO_PIX_FMT = "yuv420p"
VIDEO_GOP = 30
VIDEO_PRESET = "ultrafast"
VIDEO_CRF = 23

QUANTILES = (0.01, 0.10, 0.50, 0.90, 0.99)
IMAGE_STAT_FRAMES = 100

INSTALL_HINT = "pip install 'hirobot[record]'"


def require(module: str):
    """Import ``pyarrow`` / ``av`` or raise ``ImportError`` naming the extra."""
    import importlib

    try:
        return importlib.import_module(module)
    except ImportError as exc:
        raise ImportError(f"hirobot.record needs {module.split('.')[0]} to save datasets: "
                          f"{INSTALL_HINT}") from exc


# ------------------------------------------------------------------ stats


def _stats(values: np.ndarray) -> Dict[str, Any]:
    """LeRobot's per-feature stats over frames (axis 0), ``NaN``-aware."""
    v = np.asarray(values, dtype=np.float64)
    if v.ndim == 1:
        v = v[:, None]
    with np.errstate(all="ignore"), _quiet():
        out = {"min": np.nanmin(v, axis=0), "max": np.nanmax(v, axis=0),
               "mean": np.nanmean(v, axis=0), "std": np.nanstd(v, axis=0)}
        for q in QUANTILES:
            out[f"q{int(round(q * 100)):02d}"] = np.nanquantile(v, q, axis=0)
    out = {k: [float(x) for x in a] for k, a in out.items()}
    out["count"] = [int(v.shape[0])]
    return out


def _image_stats(frames: List[np.ndarray]) -> Dict[str, Any]:
    """Per-channel stats in ``[0, 1]`` over up to 100 frames, shape ``(3, 1, 1)``."""
    n = len(frames)
    pick = np.unique(np.linspace(0, n - 1, min(n, IMAGE_STAT_FRAMES)).astype(int))
    h, w = frames[0].shape[:2]
    step = max(1, int(math.ceil(max(h, w) / 150)))
    px = np.stack([frames[i][::step, ::step, :3] for i in pick]).reshape(-1, 3) / 255.0
    out = {"min": px.min(0), "max": px.max(0), "mean": px.mean(0), "std": px.std(0)}
    for q in QUANTILES:
        out[f"q{int(round(q * 100)):02d}"] = np.quantile(px, q, axis=0)
    out = {k: [[[float(c)]] for c in a] for k, a in out.items()}
    out["count"] = [n]
    return out


class _quiet:
    def __enter__(self):
        import warnings

        self._w = warnings.catch_warnings()
        self._w.__enter__()
        warnings.simplefilter("ignore", RuntimeWarning)

    def __exit__(self, *exc):
        self._w.__exit__(*exc)


# ------------------------------------------------------------------ video


def _even(image: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    return image[:h - h % 2, :w - w % 2]


def encode_video(path: Path, frames: List[np.ndarray], fps: int) -> Dict[str, Any]:
    """Write ``frames`` (HxWx3 uint8 RGB) as H.264 mp4; return the feature's ``info``.

    Odd heights / widths lose their last row / column (``yuv420p`` needs even
    sizes). Frame ``k`` has presentation time ``k / fps``.
    """
    av = require("av")
    path.parent.mkdir(parents=True, exist_ok=True)
    first = _even(np.asarray(frames[0]))
    h, w = first.shape[:2]
    with av.open(str(path), "w", options={"movflags": "+faststart"}) as out:
        stream = out.add_stream(VIDEO_CODEC, rate=fps, options={
            "preset": VIDEO_PRESET, "g": str(VIDEO_GOP), "crf": str(VIDEO_CRF)})
        stream.width, stream.height, stream.pix_fmt = w, h, VIDEO_PIX_FMT
        for k, image in enumerate(frames):
            img = _even(np.ascontiguousarray(image[..., :3], dtype=np.uint8))
            if img.shape[:2] != (h, w):
                raise ValueError(f"{path.parent.parent.name}: frame {k} is "
                                 f"{img.shape[1]}x{img.shape[0]}, the first was {w}x{h}")
            frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            frame.pts = k
            for packet in stream.encode(frame):
                out.mux(packet)
        for packet in stream.encode():
            out.mux(packet)
    return {"video.height": h, "video.width": w, "video.codec": "h264",
            "video.pix_fmt": VIDEO_PIX_FMT, "video.fps": fps,
            "video.channels": 3, "video.g": VIDEO_GOP, "video.crf": VIDEO_CRF,
            "video.preset": VIDEO_PRESET, "has_audio": False, "is_depth_map": False}


# ------------------------------------------------------------------ tables


def _arrow_column(pa: Any, values: np.ndarray, dtype: str):
    if dtype == "string":
        return pa.array([str(v) for v in values], type=pa.string())
    arrow = {"float32": pa.float32(), "float64": pa.float64(), "int64": pa.int64(),
             "bool": pa.bool_()}[dtype]
    v = np.asarray(values)
    if v.ndim == 1:
        return pa.array(v, type=arrow)
    flat = pa.array(v.reshape(-1), type=arrow)
    return pa.FixedSizeListArray.from_arrays(flat, int(v.shape[1]))


def _tasks_table(pa: Any, task: str):
    """``meta/tasks.parquet`` as pandas writes it: index ``task``, column ``task_index``."""
    table = pa.table({"task_index": pa.array([0], pa.int64()), "task": pa.array([task])})
    pandas_meta = {
        "index_columns": ["task"],
        "column_indexes": [{"name": None, "field_name": None, "pandas_type": "unicode",
                            "numpy_type": "object", "metadata": {"encoding": "UTF-8"}}],
        "columns": [
            {"name": "task_index", "field_name": "task_index", "pandas_type": "int64",
             "numpy_type": "int64", "metadata": None},
            {"name": "task", "field_name": "task", "pandas_type": "unicode",
             "numpy_type": "object", "metadata": None}],
        "creator": {"library": "pyarrow", "version": pa.__version__},
        "pandas_version": "2.2.0"}
    return table.replace_schema_metadata({b"pandas": json.dumps(pandas_meta).encode()})


def _episodes_table(pa: Any, row: Dict[str, Any]):
    cols = {}
    for key, value in row.items():
        if key == "tasks":
            cols[key] = pa.array([value], pa.list_(pa.string()))
        elif isinstance(value, list):
            cols[key] = pa.array([value])
        elif isinstance(value, float):
            cols[key] = pa.array([value], pa.float64())
        else:
            cols[key] = pa.array([value], pa.int64())
    return pa.table(cols)


def _json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


# ------------------------------------------------------------------ dataset


def write_dataset(root: Path, aligned: Aligned, *, task: str, robot_type: str,
                  hi_info: Dict[str, Any], episode_meta: Dict[str, Any]) -> Dict[str, Any]:
    """Write ``aligned`` as a one-episode v3.0 dataset under ``root``; return ``info.json``.

    ``hi_info`` (``hi.*`` keys) and ``hi.action_mode`` go to ``meta/hi/info.json``,
    one flat object; ``meta/info.json`` holds only LeRobot's own keys.
    """
    pa = require("pyarrow")
    pq = require("pyarrow.parquet")
    n, fps = aligned.length, aligned.fps
    frame = np.arange(n, dtype=np.int64)
    base = {"timestamp": (frame / fps).astype(np.float32), "frame_index": frame,
            "episode_index": np.zeros(n, np.int64), "index": frame,
            "task_index": np.zeros(n, np.int64)}

    features: Dict[str, Dict[str, Any]] = {}
    columns: Dict[str, Any] = {}
    for key, ft in aligned.features.items():
        features[key] = dict(ft)
        columns[key] = _arrow_column(pa, aligned.columns[key], ft["dtype"])
    for key, ft in DEFAULT_FEATURES.items():
        features[key] = dict(ft)
        columns[key] = _arrow_column(pa, base[key], ft["dtype"])
    data_path = root / DATA_PATH.format(chunk_index=0, file_index=0)
    data_path.parent.mkdir(parents=True, exist_ok=True)
    table = pa.table(columns)
    pq.write_table(table, data_path, compression="snappy", row_group_size=max(1, n))

    episode: Dict[str, Any] = {
        "episode_index": 0, "tasks": [task], "length": n,
        "data/chunk_index": 0, "data/file_index": 0,
        "dataset_from_index": 0, "dataset_to_index": n}
    stats: Dict[str, Dict[str, Any]] = {}
    for key, frames in aligned.videos.items():
        info = encode_video(root / VIDEO_PATH.format(video_key=key, chunk_index=0,
                                                     file_index=0), frames, fps)
        features[key] = {"dtype": "video", "shape": [info["video.height"], info["video.width"], 3],
                         "names": ["height", "width", "channels"], "info": info}
        episode.update({f"videos/{key}/chunk_index": 0, f"videos/{key}/file_index": 0,
                        f"videos/{key}/from_timestamp": 0.0,
                        f"videos/{key}/to_timestamp": float(n / fps)})
        stats[key] = _image_stats(frames)
    for key, ft in features.items():
        if key.startswith("hi.") or ft["dtype"] in ("string", "video"):
            continue
        stats[key] = _stats(aligned.columns[key] if key in aligned.columns else base[key])
    for key, st in stats.items():
        for name, value in st.items():
            episode[f"stats/{key}/{name}"] = value
    episode["meta/episodes/chunk_index"] = 0
    episode["meta/episodes/file_index"] = 0
    episodes_path = root / EPISODES_PATH
    episodes_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(_episodes_table(pa, episode), episodes_path)
    pq.write_table(_tasks_table(pa, task), root / TASKS_PATH)

    info = {
        "codebase_version": CODEBASE_VERSION, "fps": fps, "features": features,
        "total_episodes": 1, "total_frames": n, "total_tasks": 1,
        "chunks_size": CHUNKS_SIZE, "data_files_size_in_mb": DATA_FILES_SIZE_IN_MB,
        "video_files_size_in_mb": VIDEO_FILES_SIZE_IN_MB, "data_path": DATA_PATH,
        "video_path": VIDEO_PATH if aligned.videos else None,
        "robot_type": robot_type, "splits": {"train": "0:1"}}
    hi = {"hi.action_mode": aligned.action_mode, **hi_info}
    _json(root / "meta" / "stats.json", stats)
    _json(root / ALIGNMENT_PATH, aligned.report)
    _json(root / EPISODE_META_PATH, episode_meta)
    _json(root / HI_INFO_PATH, hi)
    _json(root / "meta" / "info.json", info)
    return info


__all__ = ["ALIGNMENT_PATH", "CODEBASE_VERSION", "DATA_PATH", "EPISODES_PATH",
           "EPISODE_META_PATH", "HI_INFO_PATH", "INSTALL_HINT", "TASKS_PATH", "VIDEO_PATH",
           "encode_video", "require", "write_dataset"]
