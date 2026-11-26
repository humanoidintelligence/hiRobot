"""Align an episode's raw samples to a fixed frame grid (the save phase, numpy only).

The recorder keeps, per source, every new reading with its source time; this
module turns them into one row per grid frame ``k`` at robot-clock time
``t0 + k / fps``:

- The grid covers the intersection of the required streams (the body state and
  every camera that delivered frames); frames outside it are trimmed.
- Continuous state vectors are interpolated linearly between the readings
  around the grid time; quaternion blocks (``*.qx`` ... ``*.qw``) are
  normalized after, with the sign of the second reading made to match the
  first. Gripper dimensions (``<ee>.gripper``) take the previous reading.
- Images take the nearest reading; an offset over ``1 / (2 fps)`` is a
  violation, and a camera with more than ``max_violation_ratio`` of its frames
  in violation flags the episode.
- The action of frame ``k`` is the first accepted command sent after the state
  reading the frame shows and before the next reading (row k = the state
  before action k); with none there, the command still in force. Never
  interpolated. After a ``hold()`` or ``hold_after_s`` without a command,
  position-type targets keep their last value and velocities, per-tick
  deltas and torques read 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from hirobot.control.arbiter import Priority
from hirobot.core.embodiment import Embodiment
from hirobot.core.features import action_features, flatten_action, flatten_observation, \
    observation_features
from hirobot.core.types import JOINT_MODES, ControlMode
from hirobot.record.qc import frozen_spans, thumbnail

#: ``hi.control_source`` values.
SOURCES = ("policy", "teleop", "skill", "hold")

#: The column of each non-joint mode that becomes ``action``.
_PRIMARY = {ControlMode.CARTESIAN: "action.ee", ControlMode.CARTESIAN_DELTA: "action.ee_delta",
            ControlMode.BASE_VELOCITY: "action.base_velocity",
            ControlMode.BASE_POSE: "action.base_target"}

_QUAT = ("qx", "qy", "qz", "qw")


def feature_key(name: str) -> str:
    """A source or camera name as it may appear in a LeRobot feature key (no ``/``)."""
    return name.replace("/", ".")


def mode_family(mode: ControlMode) -> str:
    """The dataset action mode of a command mode (``PD`` records as ``position``)."""
    return ControlMode.POSITION.value if mode == ControlMode.PD else ControlMode(mode).value


@dataclass
class QcFlag:
    """One problem the recorder found when it saved an episode."""

    name: str
    detail: str = ""

    def as_dict(self) -> Dict[str, str]:
        return {"name": self.name, "detail": self.detail}


@dataclass
class RawEpisode:
    """What the recording phase kept for one episode (see :class:`~hirobot.record.Recorder`)."""

    number: int
    task: str
    meta: Dict[str, Any]
    wall_start: float
    t_start: float
    t_stop: float = math.nan
    wall_stop: float = math.nan
    #: ``(t, stamps or None, Observation)`` for each new state reading.
    state: List[Tuple[float, Optional[dict], Any]] = field(default_factory=list)
    #: camera name -> ``(t, seq, image)`` for each new frame.
    images: Dict[str, List[Tuple[float, int, np.ndarray]]] = field(default_factory=dict)
    #: ``(CommandRecord, commanded_joint_targets or None)`` per top-level command.
    commands: List[Tuple[Any, Optional[Dict[str, float]]]] = field(default_factory=list)
    #: source -> readings not kept (the buffer was full).
    dropped: Dict[str, int] = field(default_factory=dict)
    #: Bytes of the kept images.
    image_bytes: int = 0
    fault: Optional[str] = None
    lease_events: List[Dict[str, Any]] = field(default_factory=list)


@dataclass
class Aligned:
    """An episode on the grid: columns, features, video frames, report, flags."""

    fps: int
    times: np.ndarray                       # robot clock of each frame (float64)
    columns: Dict[str, np.ndarray]          # key -> (N,) or (N, d)
    features: Dict[str, Dict[str, Any]]     # key -> {"dtype", "shape", "names"}
    videos: Dict[str, List[np.ndarray]]     # feature key -> one image per frame
    action_mode: Optional[str]
    report: Dict[str, Any]
    flags: List[QcFlag]

    @property
    def length(self) -> int:
        return int(self.times.shape[0])


@dataclass
class AlignOptions:
    fps: int
    min_duration_s: float = 1.0
    hold_after_s: float = 0.5
    max_violation_ratio: float = 0.01
    frozen_s: float = 1.0
    camera_gap_s: float = 0.5
    ik_joint_target: bool = False


class AlignError(RuntimeError):
    """The episode cannot be put on a grid (no state, streams that do not overlap)."""


# ---------------------------------------------------------------- helpers


def _interp(t: np.ndarray, v: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """Linear interpolation of rows ``v`` (sampled at ``t``) at ``grid``; clamped at the ends."""
    n = t.shape[0]
    if n == 1:
        return np.repeat(v, grid.shape[0], axis=0)
    i = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, n - 2)
    span = t[i + 1] - t[i]
    with np.errstate(invalid="ignore", divide="ignore"):
        a = np.where(span > 0, (grid - t[i]) / np.where(span > 0, span, 1.0), 0.0)
    a = np.clip(a, 0.0, 1.0)[:, None]
    return v[i] * (1.0 - a) + v[i + 1] * a


def _prev(t: np.ndarray, grid: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Index of the last ``t`` at or before each grid time (``-1``: none)."""
    return np.searchsorted(t, grid + eps, side="right") - 1


def _nearest(t: np.ndarray, grid: np.ndarray) -> np.ndarray:
    i = np.clip(np.searchsorted(t, grid), 1, max(1, t.shape[0] - 1))
    if t.shape[0] == 1:
        return np.zeros(grid.shape[0], dtype=int)
    left = t[i - 1]
    right = t[i]
    return np.where(np.abs(grid - left) <= np.abs(right - grid), i - 1, i)


def _quat_blocks(names: Sequence[str]) -> List[int]:
    """Start index of each ``<x>.qx, <x>.qy, <x>.qz, <x>.qw`` run in ``names``."""
    out = []
    for i in range(len(names) - 3):
        stems = [n.rpartition(".") for n in names[i:i + 4]]
        if [s[2] for s in stems] == list(_QUAT) and len({s[0] for s in stems}) == 1:
            out.append(i)
    return out


def _interp_state(t: np.ndarray, v: np.ndarray, grid: np.ndarray,
                  names: Sequence[str]) -> np.ndarray:
    hold = np.array([n.endswith(".gripper") for n in names], dtype=bool)
    quats = _quat_blocks(names)
    if quats and t.shape[0] > 1:
        v = v.copy()
        for q in quats:  # keep consecutive quaternions in one hemisphere
            block = v[:, q:q + 4]
            for k in range(1, block.shape[0]):
                if np.nansum(block[k] * block[k - 1]) < 0:
                    block[k] = -block[k]
    out = _interp(t, v, grid)
    if hold.any():
        i = np.clip(_prev(t, grid), 0, None)
        out[:, hold] = v[i][:, hold]
    for q in quats:
        block = out[:, q:q + 4]
        with np.errstate(invalid="ignore", divide="ignore"):
            norm = np.linalg.norm(block, axis=1, keepdims=True)
            out[:, q:q + 4] = np.where(norm > 0, block / norm, block)
    return out.astype(np.float32)


def _rate(t: np.ndarray) -> float:
    if t.shape[0] < 2 or t[-1] <= t[0]:
        return 0.0
    return float((t.shape[0] - 1) / (t[-1] - t[0]))


def _max_gap(t: np.ndarray) -> float:
    return float(np.max(np.diff(t))) if t.shape[0] > 1 else 0.0


def _source_of(record: Any) -> str:
    if record.call == "hold":
        return "hold"
    if record.priority == Priority.TAKEOVER:
        return "teleop"
    if record.priority == Priority.LAYER_SKILL:
        return "skill"
    return "policy"


# ---------------------------------------------------------------- actions


def _action_schema(emb: Embodiment, mode: ControlMode,
                   actions: Sequence[Any]) -> Dict[str, List[str]]:
    """Final column -> names for actions in ``mode``; ``action`` first.

    A joint mode keeps :func:`action_features`' ``action``. A non-joint mode
    puts its main key (``action.ee`` for ``CARTESIAN``, ...) and then the
    grippers into ``action``; joint targets beside it become ``action.joints``
    and the other keys (``action.base_velocity`` beside ``CARTESIAN``) stay.
    """
    if mode in JOINT_MODES:
        names = action_features(emb, mode).get("action")
        return {"action": list(names)} if names else {}
    primary = _PRIMARY.get(mode)
    base = action_features(emb, mode)
    if primary is None or primary not in base:
        return {}
    schema = {"action": list(base[primary]) + list(base.get("action.gripper", []))}
    seen = set()
    for a in actions:
        seen.update(action_features(emb, mode, a))
    if "action" in seen:
        schema["action.joints"] = [f"{j}.pos" for j in emb.joint_names]
    for key in sorted(seen - {primary, "action.gripper", "action"}):
        schema[key] = list(base.get(key) or action_features(emb, mode, actions[0]).get(key, []))
    return schema


def _action_row(action: Any, emb: Embodiment, mode: ControlMode,
                schema: Dict[str, List[str]]) -> Dict[str, np.ndarray]:
    """One command as values for the columns of :func:`_action_schema` (``NaN`` where absent)."""
    flat = flatten_action(action, emb)
    if mode in JOINT_MODES:
        row = dict(flat)
    else:
        primary = _PRIMARY[mode]
        n_main = len(action_features(emb, mode)[primary])
        n_grip = len(schema["action"]) - n_main
        main = flat.get(primary)
        grip = flat.get("action.gripper")
        row = {k: v for k, v in flat.items() if k not in (primary, "action.gripper", "action")}
        if "action" in flat:
            row["action.joints"] = flat["action"]
        row["action"] = np.concatenate([
            main if main is not None else np.full(n_main, np.nan, np.float32),
            grip if grip is not None else np.full(n_grip, np.nan, np.float32)])
    out = {}
    for key, names in schema.items():
        v = row.get(key)
        out[key] = (np.asarray(v, np.float32) if v is not None and len(v) == len(names)
                    else np.full(len(names), np.nan, np.float32))
    return out


# ------------------------------------------------------------------ align


def align(raw: RawEpisode, emb: Embodiment, opts: AlignOptions) -> Aligned:
    """Put ``raw`` on the grid; see the module docstring. Raises :class:`AlignError`."""
    fps = int(opts.fps)
    flags: List[QcFlag] = []
    report: Dict[str, Any] = {"fps": fps, "t_start": raw.t_start, "t_stop": raw.t_stop}
    state = [s for s in raw.state if s[0] <= raw.t_stop + 1e-9]
    if not state:
        lost = raw.dropped.get("state", 0)
        if lost:
            raise AlignError(f"no state sample reached the save worker: all {lost} were "
                             f"dropped (a limit reached, or a sample that could not be "
                             f"passed to the worker)")
        raise AlignError("no observation was read while recording (call get_observation)")
    st = np.array([s[0] for s in state], dtype=np.float64)
    order = np.argsort(st, kind="stable")
    state = [state[i] for i in order]
    st = st[order]

    cams = {}
    for name in [c.name for c in emb.cameras] + sorted(set(raw.images) - {
            c.name for c in emb.cameras}):
        frames = [f for f in raw.images.get(name, ()) if f[0] <= raw.t_stop + 1e-9]
        if not frames:
            flags.append(QcFlag("missing_camera", f"{name}: no frame while recording"))
            continue
        frames.sort(key=lambda f: f[0])
        cams[name] = frames

    # The grid: the intersection of the state and every camera that delivered frames.
    lo = max([st[0]] + [f[0][0] for f in cams.values()])
    hi = min([st[-1]] + [f[-1][0] for f in cams.values()])
    report["streams"] = {"state": {"count": len(state), "rate_hz": _rate(st),
                                   "first_t": float(st[0]), "last_t": float(st[-1]),
                                   "max_gap_s": _max_gap(st)}}
    for name, frames in cams.items():
        ct = np.array([f[0] for f in frames])
        report["streams"][name] = {"count": len(frames), "rate_hz": _rate(ct),
                                   "first_t": float(ct[0]), "last_t": float(ct[-1]),
                                   "max_gap_s": _max_gap(ct)}
        if _max_gap(ct) > opts.camera_gap_s:
            flags.append(QcFlag("camera_gap", f"{name}: {_max_gap(ct):.2f} s without a frame"))
    if hi < lo:
        raise AlignError(f"the streams do not overlap in time (state and cameras between "
                         f"{lo:.3f} and {hi:.3f} s)")
    n = int(math.floor((hi - lo) * fps + 1e-6)) + 1
    grid = lo + np.arange(n, dtype=np.float64) / fps
    report.update(grid_start=float(lo), grid_end=float(grid[-1]), frames=n,
                  trimmed_start_s=float(lo - min([st[0]] + [f[0][0] for f in cams.values()])),
                  trimmed_end_s=float(max([st[-1]] + [f[-1][0] for f in cams.values()]) - hi))
    duration = n / fps
    if duration < opts.min_duration_s:
        flags.append(QcFlag("too_short", f"{duration:.2f} s < {opts.min_duration_s:g} s"))
    if raw.dropped:
        flags.append(QcFlag("dropped_samples", ", ".join(
            f"{k}: {v}" for k, v in sorted(raw.dropped.items()))))
        report["dropped"] = dict(raw.dropped)
    if raw.fault:
        flags.append(QcFlag("fault", raw.fault))

    columns: Dict[str, np.ndarray] = {}
    features: Dict[str, Dict[str, Any]] = {}

    def add(key: str, values: np.ndarray, dtype: str, names: Optional[List[str]]) -> None:
        columns[key] = values
        shape = [1] if values.ndim == 1 else [int(values.shape[1])]
        features[key] = {"dtype": dtype, "shape": shape, "names": names}

    # ---- state: one flattened row per reading, then interpolated
    obs_list = [s[2] for s in state]
    schema = observation_features(emb, obs_list[0])
    rows = {k: np.full((len(state), len(v)), np.nan, np.float32) for k, v in schema.items()}
    for i, obs in enumerate(obs_list):
        for k, v in flatten_observation(obs, emb).items():
            if k in rows:
                rows[k][i] = v
    for k, names in schema.items():
        add(k, _interp_state(st, rows[k], grid, names), "float32", list(names))
    if "observation.state" in columns and not np.isfinite(columns["observation.state"]).all():
        bad = int((~np.isfinite(columns["observation.state"])).any(axis=1).sum())
        flags.append(QcFlag("invalid_state", f"{bad} of {n} frames have NaN / inf in "
                                             "observation.state"))
    # The state reading at or before each frame (the first one of the interpolation).
    prev = np.clip(_prev(st, grid), 0, None)
    add("hi.capture_time", st[prev].copy(), "float64", None)
    cam_names = set(raw.images) | {c.name for c in emb.cameras}
    sources = sorted({k for s in state if s[1] for k in s[1]} - cam_names)
    for src in sources:
        t_col = np.full(n, np.nan)
        seq_col = np.full(n, -1, dtype=np.int64)
        for k, i in enumerate(prev):
            stamp = (state[i][1] or {}).get(src)
            if stamp is not None:
                t_col[k], seq_col[k] = stamp[0], stamp[1]
        add(f"hi.stamp.{feature_key(src)}.t", t_col, "float64", None)
        add(f"hi.stamp.{feature_key(src)}.seq", seq_col, "int64", None)

    # ---- images: nearest reading
    videos: Dict[str, List[np.ndarray]] = {}
    limit = 1.0 / (2 * fps)
    report["cameras"] = {}
    for name, frames in cams.items():
        ct = np.array([f[0] for f in frames])
        idx = _nearest(ct, grid)
        skew = ct[idx] - grid
        viol = int((np.abs(skew) > limit + 1e-9).sum())
        ratio = viol / n
        key = f"observation.images.{feature_key(name)}"
        videos[key] = [frames[i][2] for i in idx]
        add(f"hi.image_skew.{feature_key(name)}", skew.astype(np.float32), "float32", None)
        add(f"hi.stamp.{feature_key(name)}.t", ct[idx].astype(np.float64), "float64", None)
        add(f"hi.stamp.{feature_key(name)}.seq",
            np.array([frames[i][1] for i in idx], dtype=np.int64), "int64", None)
        frozen = frozen_spans(ct, [thumbnail(f[2]) for f in frames], opts.frozen_s)
        report["cameras"][name] = {"max_offset_s": float(np.max(np.abs(skew))),
                                   "violations": viol, "violation_ratio": ratio,
                                   "limit_s": limit, "frozen_spans": frozen}
        if ratio > opts.max_violation_ratio:
            flags.append(QcFlag("alignment", f"{name}: {viol} of {n} frames off the grid by "
                                             f"more than {limit * 1e3:.1f} ms"))
        if frozen:
            longest = max(b - a for a, b in frozen)
            flags.append(QcFlag("frozen_camera", f"{name}: the picture did not change for "
                                                 f"{longest:.2f} s while frames arrived"))

    # ---- commands
    cmds = sorted((c for c in raw.commands if c[0].t <= raw.t_stop + 1e-9),
                  key=lambda c: c[0].t)
    action_mode = _commands(cmds, grid, st, prev, emb, opts, add, flags, report)
    return Aligned(fps=fps, times=grid, columns=columns, features=features, videos=videos,
                   action_mode=action_mode, report=report, flags=flags)


#: Modes whose main values are a rate or an effort, not a target pose: a hold stops them.
_STOPPED_BY_HOLD = (ControlMode.VELOCITY, ControlMode.TORQUE, ControlMode.CARTESIAN_DELTA,
                    ControlMode.BASE_VELOCITY)


def _zero_columns(mode: ControlMode, schema: Dict[str, List[str]],
                  emb: Embodiment) -> Dict[str, np.ndarray]:
    """Column -> mask of the dimensions that read 0 while the robot holds.

    Velocities, per-tick deltas and torques; position targets (joint targets,
    poses, ``base_target``) and grippers keep their last value.
    """
    out: Dict[str, np.ndarray] = {}
    if "action" in schema and mode in _STOPPED_BY_HOLD:
        names = schema["action"]
        if mode in JOINT_MODES:
            n_main = len(emb.joint_names)
        else:
            n_main = len(action_features(emb, mode)[_PRIMARY[mode]])
        out["action"] = np.arange(len(names)) < n_main
    if "action.base_velocity" in schema:
        out["action.base_velocity"] = np.ones(len(schema["action.base_velocity"]), dtype=bool)
    return out


def _windows(st: np.ndarray, prev: np.ndarray, fps: float) -> Tuple[np.ndarray, np.ndarray]:
    """Per frame, the time span of commands that answer its state reading.

    Frame ``k`` shows reading ``p = prev[k]``; a command sent after that reading
    and before the next one followed it: ``[st[p], st[p + 1])`` (one frame
    period after the last reading). A command stamped at the reading's own time
    (a simulator whose clock stands still between the two calls) belongs to it.
    """
    lo = st[prev]
    nxt = np.minimum(prev + 1, st.shape[0] - 1)
    hi = np.where(prev + 1 < st.shape[0], st[nxt], lo + 1.0 / fps)
    return lo, hi


def _opens(prev: np.ndarray) -> np.ndarray:
    """Whether each frame is the first one that shows its state reading."""
    out = np.ones(prev.shape[0], dtype=bool)
    out[1:] = prev[1:] != prev[:-1]
    return out


def _first_in(t: np.ndarray, lo: np.ndarray, hi: np.ndarray,
              eps: float = 1e-9) -> Tuple[np.ndarray, np.ndarray]:
    """Index of the first and last ``t`` in ``[lo, hi)`` per frame (``-1``: none)."""
    if t.shape[0] == 0:
        none = np.full(lo.shape[0], -1)
        return none, none
    a = np.searchsorted(t, lo - eps, side="left")
    b = np.searchsorted(t, hi - eps, side="left")  # first index at or after hi
    found = b > a
    return np.where(found, a, -1), np.where(found, b - 1, -1)


def _commands(cmds: List[Tuple[Any, Optional[Dict[str, float]]]], grid: np.ndarray,
              st: np.ndarray, prev: np.ndarray, emb: Embodiment, opts: AlignOptions,
              add: Any, flags: List[QcFlag], report: Dict[str, Any]) -> Optional[str]:
    """Add the action columns; return the dataset's action mode (``None``: no command).

    Row ``k`` pairs the state reading the frame shows with the first command
    sent after it (row k = the state before action k), not with the command in
    force at the frame time: on a robot whose clock moves between
    ``get_observation`` and ``send_action``, that one answers the reading before.
    Frames after the first that show the same reading take the first command in
    their own period, else the one in force; ``hold_after_s`` counts from the
    newest command at the frame time.
    """
    n = grid.shape[0]
    fps = 1.0 / (grid[1] - grid[0]) if n > 1 else float(opts.fps)
    lo, hi = _windows(st, prev, fps)
    accepted = [c for c in cmds if c[0].accepted]
    sends = [c for c in accepted if c[0].call == "send_action" and c[0].action is not None]
    counts: Dict[str, int] = {}
    for rec, _ in sends:
        counts[mode_family(rec.action.mode)] = counts.get(mode_family(rec.action.mode), 0) + 1
    report["commands"] = {"accepted": len(accepted), "refused": len(cmds) - len(accepted),
                          "modes": counts}
    mode_name = max(counts, key=lambda m: (counts[m], m)) if counts else None
    if len(counts) > 1:
        others = ", ".join(f"{m}: {k}" for m, k in sorted(counts.items()) if m != mode_name)
        flags.append(QcFlag("mixed_action_modes", f"action holds the {mode_name} commands "
                                                  f"({counts[mode_name]}); left out {others}"))
    mode = ControlMode(mode_name) if mode_name else None
    in_mode = [c for c in sends if mode_family(c[0].action.mode) == mode_name]
    schema = _action_schema(emb, mode, [c[0].action for c in in_mode]) if mode else {}
    if mode is not None and not schema:
        flags.append(QcFlag("unrecorded_action", f"{mode_name} commands have no action column"))

    # The accepted command of each frame. The first frame that shows a reading
    # takes the first command sent after it (else, none before the next
    # reading, the one still in force). Later frames showing the same reading
    # (readings slower than the grid, e.g. one observation per action chunk)
    # take the command in force at the frame time, but never one older than
    # that first command.
    at = np.array([c[0].t for c in accepted], dtype=np.float64)
    newest = _prev(at, grid) if accepted else np.full(n, -1)  # last at or before grid[k]
    first, _ = _first_in(at, lo, hi)
    before = _prev(at, hi - 2e-9) if accepted else np.full(n, -1)
    opens = _opens(prev)
    pick = np.where(first >= 0, first, before)
    later = ~opens
    if later.any() and accepted:
        # The first command sent in the frame's own period (within the reading's
        # window), as for the first frame: on a robot clock it comes just after
        # the grid time. None there: the one in force.
        step = 1.0 / fps
        f_lo = np.maximum(grid, lo)
        f_hi = np.minimum(grid + step, hi)
        own, _ = _first_in(at, f_lo, f_hi)
        pick[later] = np.where(own[later] >= 0, own[later],
                               np.maximum(newest[later], first[later]))
    # hold_after_s counts from the newest command the frame has seen.
    t_last = np.full(n, -np.inf)
    if accepted:
        t_last = np.where(newest >= 0, at[np.maximum(newest, 0)], -np.inf)
        t_last = np.maximum(t_last, np.where(pick >= 0, at[np.maximum(pick, 0)], -np.inf))
    source = np.empty(n, dtype=object)
    values = {k: np.full((n, len(v)), np.nan, np.float32) for k, v in schema.items()}
    ik = np.full((n, len(emb.joint_names)), np.nan, np.float32)
    has_ik = False
    jindex = {j: i for i, j in enumerate(emb.joint_names)}
    # The last send_action at or before each accepted command (itself for a send).
    send_of = np.full(len(accepted), -1)
    s = -1
    for i, (rec, _) in enumerate(accepted):
        if rec.call == "send_action" and rec.action is not None:
            s = i
        send_of[i] = s
    zero = _zero_columns(mode, schema, emb) if schema else {}
    rows: Dict[int, Dict[str, np.ndarray]] = {}
    for k in range(n):
        i = int(pick[k])
        if i < 0:  # no command yet
            source[k] = "hold"
            continue
        rec, targets = accepted[i]
        held = rec.call == "hold" or grid[k] - t_last[k] > opts.hold_after_s
        s = int(send_of[i])
        if s >= 0 and schema and mode_family(accepted[s][0].action.mode) == mode_name:
            if s not in rows:
                rows[s] = _action_row(accepted[s][0].action, emb, mode, schema)
            for key, v in rows[s].items():
                values[key][k] = v
            if held:  # a hold stops a velocity, delta or torque; a position target stays
                for key, mask in zero.items():
                    values[key][k, mask] = 0.0
        if rec.call == "hold":
            source[k] = "hold"
            continue
        source[k] = "hold" if held else _source_of(rec)
        if targets:
            has_ik = True
            for name, q in targets.items():
                j = jindex.get(name)
                if j is not None:
                    ik[k, j] = q
    for key, v in values.items():
        add(key, v, "float32", list(schema[key]))
    add("hi.control_source", source.astype(str), "string", None)
    srcs = set(source.tolist())
    add("intervention", np.array([s == "teleop" for s in source]) & ("policy" in srcs),
        "bool", None)
    report["control_source"] = {s: int((source == s).sum()) for s in SOURCES if s in srcs}

    # Refused commands: each goes to exactly one frame, the one whose span holds
    # it. A frame's span starts at its reading when it is the first frame to
    # show that reading, else at its grid time; it ends where the next frame's
    # starts. The last refusal in a span wins.
    refused = [c for c in cmds if not c[0].accepted and c[0].call == "send_action"
               and c[0].action is not None]
    if schema and "action" in schema:
        # 0, not NaN, where nothing was refused (or the refused action did not
        # convert): pandas turns NaN inside a list column into null, which LeRobot's
        # aggregate_datasets cannot read back. hi.rejected_action_mask says which rows hold one.
        rej = np.zeros((n, len(schema["action"])), np.float32)
        mask = np.zeros(n, dtype=bool)
        rt = np.array([c[0].t for c in refused], dtype=np.float64)
        if refused:
            starts = np.where(opens, lo, grid)
            end = max(float(hi[-1]), float(grid[-1]) + 1.0 / fps)
            frame = np.searchsorted(starts, rt + 1e-9, side="right") - 1
            j = np.full(n, -1)
            for i, k in enumerate(frame):
                if k >= 0 and rt[i] < end - 1e-9:
                    j[k] = i  # in time order: the last one in the span stays
            for k in range(n):
                i = int(j[k])
                if i < 0:
                    continue
                rec = refused[i][0]
                mask[k] = True
                if mode_family(rec.action.mode) == mode_name:
                    try:
                        rej[k] = _action_row(rec.action, emb, mode, schema)["action"]
                    except ValueError:  # a malformed action is why it was refused
                        pass
        add("hi.rejected_action", rej, "float32", list(schema["action"]))
        add("hi.rejected_action_mask", mask, "bool", None)
    faults = sorted({c[0].reason for c in cmds if not c[0].accepted and c[0].reason
                     and "fault" in c[0].reason.lower()})
    if faults and not any(f.name == "fault" for f in flags):
        flags.append(QcFlag("fault", faults[0]))
    if has_ik or opts.ik_joint_target:
        add("hi.ik_joint_target", ik, "float32", [f"{j}.pos" for j in emb.joint_names])
    if not sends:
        flags.append(QcFlag("no_commands", "no send_action was accepted while recording"))
    return mode_name


__all__ = ["AlignError", "AlignOptions", "Aligned", "QcFlag", "RawEpisode", "SOURCES", "align",
           "feature_key", "mode_family"]
