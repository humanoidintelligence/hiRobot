"""The :class:`Recorder`: listen to a robot, stream its samples to a save worker.

While recording, the robot's listeners (``robot.add_listener``) hand every
top-level observation and command to the recorder on the caller's thread. It
keeps each new reading (de-duplicated by ``Observation.stamps[src].seq``):
state readings and commands go on an outbox by reference, the pixels of each
new image are copied into a preallocated shared-memory ring and the image
itself is let go at once. A sender thread passes the outbox to the save
worker, a child process by default (:mod:`hirobot.record.worker`), which keeps
the episode. After :meth:`Recorder.stop` the worker aligns it to a frame grid,
encodes the videos, writes a LeRobot v3.0 dataset directory and appends the
quality flags to ``meta/qc.jsonl``, one episode after the other; the recorder
turns its answers into :class:`SaveEvent` callbacks.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
import weakref
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

from hirobot.record.align import AlignOptions, QcFlag
from hirobot.record.worker import BATCH, FrameRing, ProcessLink, ThreadLink, can_use_process, \
    slug

logger = logging.getLogger(__name__)

#: Episodes waiting for (or in) the save worker before :meth:`Recorder.start` refuses.
MAX_QUEUE = 15

#: Default frame rate of the grid when the robot has cameras.
CAMERA_FPS = 30

#: Default size of the shared-memory frame ring, in MB (about 40 frames of 1080p RGB).
RING_MB = 256.0

SAVE_STATES = ("saving", "saved", "failed")

WORKERS = ("process", "thread")

#: A child that exits this many times in a row before it is ready makes the
#: recorder fall back to a thread worker.
_MAX_BOOT_FAILURES = 2


class RecorderError(RuntimeError):
    """A recorder call that cannot go ahead (already recording, queue full, ...)."""


@dataclass(frozen=True)
class SaveEvent:
    """What a :meth:`Recorder.subscribe` callback gets about one stopped episode.

    Attributes:
        episode: The episode's number in this recorder (what :meth:`Recorder.stop`
            returned).
        state: ``saving`` (the worker took it), then ``saved`` or ``failed``.
        path: The dataset directory (``saved``); the partly written one
            (``failed``); ``None`` before it is made.
        flags: The quality flags (``saved``): ``QcFlag(name, detail)``, empty
            when none.
        error: Why it failed (``failed``), else ``None``.
        task: The task text given to :meth:`Recorder.start`.
        frames: Frames written (``saved``), else ``None``.
    """

    episode: int
    state: str
    path: Optional[str] = None
    flags: Tuple[QcFlag, ...] = ()
    error: Optional[str] = None
    task: str = ""
    frames: Optional[int] = None


class Subscription:
    """Returned by :meth:`Recorder.subscribe`; :meth:`remove` stops the callbacks."""

    def __init__(self, recorder: "Recorder", callback: Callable[[SaveEvent], None]) -> None:
        self._recorder = recorder
        self.callback = callback
        #: Errors the callback raised (each logged).
        self.errors = 0

    def remove(self) -> None:
        """Stop calling this callback. Safe twice, from any thread."""
        self._recorder._unsubscribe(self)

    @property
    def active(self) -> bool:
        return self in self._recorder._subscribers


@dataclass
class _Episode:
    """What the control process keeps of an episode: counts and times, no samples."""

    number: int
    task: str
    meta: Dict[str, Any]
    wall_start: float
    t_start: float
    t_stop: float = float("nan")
    wall_stop: float = float("nan")
    #: source -> readings not kept (a limit reached, or the frame ring full).
    dropped: Dict[str, int] = field(default_factory=dict)
    #: source -> readings kept (``state``, ``commands``, camera names).
    kept: Dict[str, int] = field(default_factory=dict)
    #: Bytes of the kept images.
    image_bytes: int = 0
    fault: Optional[str] = None
    lease_events: List[Dict[str, Any]] = field(default_factory=list)
    #: Why the episode cannot be saved (its worker exited while it was recording);
    #: it fails when stopped.
    lost: Optional[str] = None
    saving_sent: bool = False
    done: bool = False
    #: The worker link the ``open`` message went to (``None``: not sent yet).
    link: Any = None

    #: Whether the "frame ring full" warning was logged for this episode.
    ring_full_warned: bool = False

    def drop(self, source: str, n: int = 1) -> None:
        self.dropped[source] = self.dropped.get(source, 0) + n

    def keep(self, source: str) -> None:
        self.kept[source] = self.kept.get(source, 0) + 1


class Recorder:
    """Record episodes of ``robot`` as LeRobot v3.0 datasets under ``root``.

    Args:
        robot: The outermost robot (what ``make_robot`` returns); the recorder
            attaches with ``robot.add_listener`` right away.
        root: Directory the datasets go in, one directory per episode
            (``<root>/<YYYYmmdd-HHMMSS>_<task slug>_<number:04d>/``).
        fps: Frame rate of the grid. Default: 30 when the robot has cameras
            (``min(30, control_hz)``), else ``control_hz`` (30 without one).
        max_queue: Stopped episodes waiting to be saved (the one being saved
            included) before :meth:`start` refuses.
        min_duration_s: Shorter episodes are flagged ``too_short``.
        max_buffer_mb: Images kept per episode, in MB of pixels; past it new
            frames are dropped and the episode is flagged ``dropped_samples``.
        max_samples: State readings and commands kept per episode, each.
        copy_images: No effect any more: every kept image is copied into the
            frame ring. Accepted so existing callers keep working.
        robot_config: The config name for ``meta/hi/info.json``
            (``hi.robot_config``) when ``meta`` does not give one.
        hold_after_s: A frame more than this after the last accepted command
            counts as ``hold`` in ``hi.control_source``.
        frozen_s: A camera whose picture (32x24 thumbnail) stays the same this
            long while its frames keep arriving is flagged ``frozen_camera``.
        max_queue_mb: :meth:`start` refuses while the stopped episodes not yet
            saved hold more than this many MB of images. With the episode being
            recorded, the raw images the save worker holds stay under
            ``max_queue_mb + 2 * max_buffer_mb``.
        worker: Where episodes are kept and saved. ``"process"`` (default): a
            child Python process, started in the background on the first
            :meth:`start` and reused, so aligning and encoding never hold
            this process's GIL. Without ``memfd`` (not Linux), or when the child
            exits twice in a row before it is ready, the recorder logs a
            warning and uses ``"thread"``: a thread of this process.
        ring_mb: The shared-memory ring new images are copied into on their
            way to the worker, allocated and touched once here. A frame that
            finds it full is dropped and counted (``dropped_samples``).

    The recorder is a listener: it never commands the robot, and a failure in
    it never reaches the control loop (the robot logs it). Its callbacks run on
    the thread that calls ``get_observation`` / ``send_action``: they append
    references to an outbox and copy each new image into the ring (about
    0.1 to 0.4 ms for a 1080p frame), then let go of the image; one sender
    thread passes the outbox to the worker.
    """

    def __init__(self, robot: Any, root: Union[str, Path], *, fps: Optional[float] = None,
                 max_queue: int = MAX_QUEUE, min_duration_s: float = 1.0,
                 max_buffer_mb: float = 4096.0, max_samples: int = 1_000_000,
                 copy_images: bool = False, robot_config: Optional[str] = None,
                 hold_after_s: float = 0.5, frozen_s: float = 1.0,
                 max_queue_mb: float = 8192.0, worker: str = "process",
                 ring_mb: float = RING_MB) -> None:
        from hirobot.record.writer import require

        require("pyarrow.parquet")  # fail here, not in the save worker
        require("av")
        if int(max_queue) < 1:
            raise ValueError(f"max_queue must be at least 1, got {max_queue}")
        if worker not in WORKERS:
            raise ValueError(f"worker must be one of {WORKERS}, got {worker!r}")
        emb = robot.embodiment
        if fps is None:
            hz = int(emb.control_hz or CAMERA_FPS)
            fps = min(CAMERA_FPS, hz) if emb.cameras else hz
        if fps <= 0 or abs(fps - round(fps)) > 1e-9:
            raise ValueError(f"fps must be a positive whole number (LeRobot stores an integer "
                             f"fps), got {fps}")
        self.robot = robot
        self.root = Path(root)
        self.embodiment = emb
        self.fps = int(round(fps))
        self.max_queue = int(max_queue)
        self.min_duration_s = float(min_duration_s)
        self.max_buffer_bytes = float(max_buffer_mb) * 1e6
        self.max_queue_bytes = float(max_queue_mb) * 1e6
        self.max_samples = int(max_samples)
        self.copy_images = bool(copy_images)
        self.robot_config = robot_config
        self.hold_after_s = float(hold_after_s)
        self.frozen_s = float(frozen_s)
        # mink_ik's solved joint targets: read only when the outermost robot has them.
        self._ik_attr = hasattr(type(robot), "commanded_joint_targets")
        # At least 16 frames of the largest camera the embodiment declares.
        frame = max([int(c.width) * int(c.height) * 3 for c in emb.cameras] or [0])
        self._ring = FrameRing.create(max(int(float(ring_mb) * 1e6), 16 * frame + 4096))
        # Lock order: _ring_lock, then _lock. _ring_lock keeps frames in ring order
        # on the outbox; the pixel copy runs under it but outside _lock.
        self._ring_lock = threading.Lock()
        self._lock = threading.Condition(threading.Lock())
        self._wake = _Signal(self._lock)  # the sender waits on it
        self._episode: Optional[_Episode] = None
        self._count = 0
        self._live: Dict[int, _Episode] = {}  # stopped, final event not delivered yet
        self._by_number: Dict[int, _Episode] = {}  # recording or in _live
        self._pending = 0
        self._queued_bytes = 0
        self._closed = False
        self._shut = False
        self._subscribers: Tuple[Subscription, ...] = ()
        self._in_force: Optional[Tuple[Any, Any]] = None
        self._outbox: List[tuple] = []
        self._sent_end = self._ring.position  # ring end of the last frame handed to a worker
        self._close_deadline: Optional[float] = None
        self._events = _Events()
        self._link: Any = None
        self._boot_failures = 0
        self._mode = worker
        if worker == "process" and not can_use_process():
            logger.warning("Recorder: this platform cannot run the save worker in a "
                           "subprocess (no memfd); saving on a thread of this process")
            self._mode = "thread"
        self._reset_buffers()
        # The threads and the robot's listener hold the recorder only weakly, so a
        # recorder dropped without close() is still collected; its finalizer wakes
        # the threads, which then end the worker, close the ring and exit.
        me = weakref.ref(self)
        self._handle = robot.add_listener(on_observation=_weak(me, "_on_observation"),
                                          on_command=_weak(me, "_on_command"))
        self._sender = threading.Thread(
            target=_send_loop, args=(me, self._wake, self._events, self._handle, self._ring,
                                     self._ring_lock),
            name="hirobot-recorder-send", daemon=True)
        self._dispatcher = threading.Thread(target=_event_loop, args=(me, self._events),
                                            name="hirobot-recorder", daemon=True)
        self._dispatcher.start()
        self._sender.start()
        finalizer = weakref.finalize(self, _poke, self._wake)
        finalizer.atexit = False

    # ------------------------------------------------------------ state

    @property
    def recording(self) -> bool:
        """Whether an episode is being recorded (between :meth:`start` and :meth:`stop`)."""
        return self._episode is not None

    @property
    def queue_length(self) -> int:
        """Stopped episodes not saved yet, the one being saved included."""
        return self._pending

    @property
    def episodes_recorded(self) -> int:
        """Episodes started so far (the next one gets this number)."""
        return self._count

    @property
    def worker(self) -> str:
        """Where episodes are saved now: ``process`` or ``thread`` (after a fallback)."""
        return self._mode

    # ------------------------------------------------------------ control

    def start(self, task: str, meta: Optional[Dict[str, Any]] = None) -> int:
        """Start recording an episode of ``task``; return its number.

        ``meta`` goes into the dataset as is (``meta/hi/episode.json``):
        ``operator``, ``station``, ``device``, ``robot_config``,
        ``hiteleop_version`` or anything else that JSON can hold.

        Raises:
            RecorderError: already recording, the save queue is full, or the
                recorder is closed.
            ValueError: ``task`` is empty.
        """
        if not isinstance(task, str) or not task.strip():
            raise ValueError("start() needs a task: the instruction this episode shows")
        # Ask the robot first, outside the lock the listeners take (none of these raise).
        t_start, wall_start = self._now(), time.time()
        fault = self._fault()
        with self._lock:
            if self._closed:
                raise RecorderError("the recorder is closed")
            if self._episode is not None:
                raise RecorderError(f"already recording episode {self._episode.number}; "
                                    "stop() or cancel() it first")
            if self._pending >= self.max_queue:
                raise RecorderError(f"the save queue is full ({self._pending} of "
                                    f"{self.max_queue} episodes waiting); wait for a save")
            if self._queued_bytes > self.max_queue_bytes:
                raise RecorderError(f"the save queue holds {self._queued_bytes / 1e6:.0f} MB "
                                    f"of images (max_queue_mb={self.max_queue_bytes / 1e6:g}); "
                                    "wait for a save")
            number = self._count
            self._count += 1
            self._reset_buffers()
            ep = _Episode(number=number, task=task.strip(), meta=dict(meta or {}),
                          wall_start=wall_start, t_start=t_start)
            self._outbox.append(("open", number, ep.task, wall_start, t_start))
            if self._in_force is not None:
                self._outbox.append(("command", number, self._in_force))
                ep.keep("commands")
                self._in_force = None
            self._episode = ep
            self._by_number[number] = ep
            self._fault_at_start = fault
            self._wake.notify_all()
            return number

    def stop(self) -> int:
        """Stop recording and queue the episode for saving; return its number.

        Returns at once; the save worker sends ``saving`` then ``saved`` /
        ``failed`` to the :meth:`subscribe` callbacks.

        Raises:
            RecorderError: not recording.
        """
        # Ask the robot first, outside the lock the listeners take (none of these raise).
        t_stop, wall_stop = self._now(), time.time()
        fault = self._fault()
        events = self._command_events()
        with self._ring_lock:  # a frame being copied in goes out before the stop
            with self._lock:
                ep = self._take("stop")
                # The listeners change an episode only under the lock while it is the
                # current one: from here on nothing else touches it.
                fields: Dict[str, Any] = {}
                try:
                    ep.t_stop, ep.wall_stop = t_stop, wall_stop
                    if fault and fault != self._fault_at_start:
                        ep.fault = fault
                    ep.lease_events = _lease_events(events, ep)
                    fields = {"t_stop": t_stop, "wall_stop": wall_stop, "fault": ep.fault,
                              "lease_events": ep.lease_events, "meta": ep.meta,
                              "dropped": dict(ep.dropped)}
                except Exception:  # noqa: BLE001 - a detached episode is never lost
                    logger.exception("Recorder.stop: episode %d metadata incomplete", ep.number)
                    fields.setdefault("t_stop", t_stop)
                finally:
                    if ep.lost:  # its worker is gone: fail it here, the next worker never saw it
                        self._events.put((None, ("failed", ep.number, None, ep.lost, "")))
                    else:
                        self._outbox.append(("stop", ep.number, fields))
                    self._live[ep.number] = ep
                    self._pending += 1
                    self._queued_bytes += ep.image_bytes
                    self._wake.notify_all()
                return ep.number

    def cancel(self) -> Optional[int]:
        """Drop the episode being recorded (it is never saved); return its number.

        ``None`` when not recording.
        """
        with self._lock:
            if self._episode is None:
                return None
            ep = self._take("cancel")
            self._by_number.pop(ep.number, None)
            self._outbox.append(("cancel", ep.number))
            self._wake.notify_all()
            return ep.number

    def wait(self, timeout: Optional[float] = None) -> bool:
        """Block until every stopped episode is saved (or failed); ``False`` on timeout."""
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._lock:
            while self._pending:
                left = None if deadline is None else deadline - time.monotonic()
                if left is not None and left <= 0:
                    return False
                self._lock.wait(left)
        return True

    def close(self, timeout: Optional[float] = None) -> int:
        """Detach from the robot, save what is queued, and end the save worker.

        An episode still being recorded is stopped and saved
        (``meta/hi/episode.json`` says ``"stopped_by": "close"``). Waits up to
        ``timeout`` seconds (``None``: until done) and returns the number of
        episodes still not saved (0 when all are). Then the worker is told to
        exit and the frame ring is freed. A subprocess still saves the stopped
        episodes it already received, even after this process exits; samples
        not yet passed to it when the timeout ran out are lost. A thread worker
        keeps going only while this process lives. Once the worker is done,
        the event thread delivers the last :meth:`subscribe` events and ends.
        Safe to call twice.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        if self._close_deadline is None or deadline is None:
            self._close_deadline = deadline
        self._handle.remove()
        with self._lock:
            if self._episode is not None:
                self._episode.meta.setdefault("stopped_by", "close")
        if self._episode is not None:
            try:
                self.stop()
            except RecorderError:  # stopped meanwhile on another thread
                pass
        with self._lock:
            self._closed = True
        if not self.wait(timeout):
            logger.warning("Recorder.close: %d episode(s) still saving after %.1f s",
                           self._pending, timeout)
        with self._lock:
            self._shut = True
            self._wake.notify_all()
        if threading.current_thread() not in (self._sender, self._dispatcher):
            self._sender.join(self._close_left())
            self._dispatcher.join(self._close_left())
        return self._pending

    def __enter__(self) -> "Recorder":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def subscribe(self, callback: Callable[[SaveEvent], None]) -> Subscription:
        """Call ``callback(SaveEvent)`` for every save state change, on the event thread.

        Events of one episode come in order (``saving``, then ``saved`` or
        ``failed``) and episodes in the order they were stopped. A callback that
        raises is logged and counted (``Subscription.errors``); saving goes on.
        """
        if not callable(callback):
            raise TypeError(f"subscribe() needs a callable, got {callback!r}")
        sub = Subscription(self, callback)
        with self._lock:
            self._subscribers = self._subscribers + (sub,)
        return sub

    def _unsubscribe(self, sub: Subscription) -> None:
        with self._lock:
            self._subscribers = tuple(s for s in self._subscribers if s is not sub)

    # ------------------------------------------------------------ listeners (caller's thread)

    def _reset_buffers(self) -> None:
        self._state_key: Any = None
        self._image_seq: Dict[str, Any] = {}
        # A camera without stamps is de-duplicated by object identity; the last
        # image is held so its id cannot be reused by a new array.
        self._last_image: Dict[str, Any] = {}
        self._calls = 0
        self._fault_at_start: Optional[str] = None

    def _on_observation(self, obs: Any) -> None:
        if self._episode is None or self._episode.lost:
            return
        stamps = obs.stamps
        images = obs.images
        state_stamps = {k: v for k, v in stamps.items() if k not in images} if stamps else None
        now = self._now() if not state_stamps or any(
            stamps.get(name) is None for name in images) else None
        # Nothing downstream reads images, depth or extra off the state sample (an
        # extra that does not pickle would drop every sample on the way to a child).
        lite = obs.model_copy(update={"images": {}, "depth": {}, "extra": {}}) if (
            images or obs.depth or obs.extra) else obs
        new = []
        with self._lock:  # stop() detaches the episode under it
            ep = self._episode
            if ep is None or ep.lost:
                return
            self._calls += 1
            if state_stamps:
                key = tuple(sorted((k, v.seq) for k, v in state_stamps.items()))
                body = state_stamps.get("body")
                t = body.t if body is not None else max(v.t for v in state_stamps.values())
            else:  # no stamps (a minimal driver): every read is a new reading, at now()
                key, t = self._calls, now
            if key != self._state_key:
                self._state_key = key
                if ep.kept.get("state", 0) < self.max_samples:
                    ep.keep("state")
                    self._outbox.append(("state", ep.number, (t, state_stamps, lite)))
                else:
                    ep.drop("state")
            for name, image in images.items():
                stamp = stamps.get(name)
                if stamp is not None:
                    if self._image_seq.get(name) == stamp.seq:
                        continue
                    self._image_seq[name] = stamp.seq
                    seq = stamp.seq
                else:
                    if self._last_image.get(name) is image:
                        continue
                    self._last_image[name] = image
                    seq = self._calls
                if ep.image_bytes + image.nbytes > self.max_buffer_bytes:
                    ep.drop(name)
                    continue
                new.append((name, stamp.t if stamp is not None else now, seq, image))
        for name, t, seq, image in new:
            self._put_frame(ep, name, t, seq, image)

    def _put_frame(self, ep: _Episode, name: str, t: float, seq: int, image: Any) -> None:
        """Copy ``image`` into the ring (outside ``_lock``) and queue its message."""
        nbytes = int(image.nbytes)
        with self._ring_lock:
            with self._lock:
                if self._episode is not ep or ep.lost:
                    return
                slot = self._ring.reserve(nbytes)
                if slot is None:
                    ep.drop(name)  # the worker is behind: the ring is full
                    warn, ep.ring_full_warned = not ep.ring_full_warned, True
                else:
                    warn = False
                    ep.image_bytes += nbytes
            if slot is None:
                if warn:
                    logger.warning(
                        "Recorder: episode %d: the frame ring (%.0f MB) is full, dropping %s "
                        "frames until the save worker catches up. A loop that runs faster "
                        "than real time (an unpaced simulation) or a small ring_mb causes "
                        "this.", ep.number, self._ring.data_size / 1e6, name)
                return
            offset, _, end = slot
            try:
                self._ring.put(offset, image)
                msg = ("frame", ep.number, name, t, seq, offset, end, tuple(image.shape),
                       image.dtype.str)
            except Exception:  # noqa: BLE001 - not an array the ring can hold
                logger.debug("Recorder: frame of %r not kept", name, exc_info=True)
                msg = ("frame", -1, name, t, seq, offset, end, (0,), "u1")
            with self._lock:
                if msg[1] < 0:
                    ep.image_bytes -= nbytes
                    ep.drop(name)
                else:
                    ep.keep(name)
                self._outbox.append(msg)
                self._wake.notify()

    def _on_command(self, record: Any) -> None:
        targets = None
        if self._ik_attr and record.accepted and record.call == "send_action":
            targets = getattr(self.robot, "commanded_joint_targets", None)
        with self._lock:
            ep = self._episode
            if ep is None:
                if record.accepted:  # the command in force when the next episode starts
                    self._in_force = (record, targets)
                return
            if ep.lost:
                return
            if ep.kept.get("commands", 0) < self.max_samples:
                ep.keep("commands")
                self._outbox.append(("command", ep.number, (record, targets)))
            else:
                ep.drop("commands")

    # ------------------------------------------------------------ helpers

    def _take(self, what: str) -> _Episode:
        ep = self._episode
        if ep is None:
            raise RecorderError(f"{what}(): not recording")
        self._episode = None
        return ep

    def _now(self) -> float:
        try:
            return float(self.robot.now())
        except Exception:  # noqa: BLE001 - a clock hiccup must not break recording
            return time.monotonic()

    def _fault(self) -> Optional[str]:
        try:
            return self.robot.fault
        except Exception:  # noqa: BLE001
            return None

    def _command_events(self) -> List[Any]:
        try:
            return list(self.robot.command_events())
        except Exception:  # noqa: BLE001
            return []

    def _options(self) -> AlignOptions:
        return AlignOptions(fps=self.fps, min_duration_s=self.min_duration_s,
                            hold_after_s=self.hold_after_s, frozen_s=self.frozen_s,
                            ik_joint_target=self._ik_attr)

    # ------------------------------------------------------------ sender thread

    def _new_link(self) -> Any:
        """Start a worker (the caller holds no lock); fall back to a thread when it cannot."""
        events = self._events
        if self._mode == "process":
            try:
                return ProcessLink(self._ring, events.reply, events.exit)
            except Exception as exc:  # noqa: BLE001 - e.g. no sys.executable, fork refused
                logger.warning("Recorder: cannot start the save worker process (%s); saving "
                               "on a thread of this process", exc)
                self._mode = "thread"
        return ThreadLink(self._ring, events.reply, events.exit)

    def _send_step(self, st: "_SenderState") -> str:
        """One round of the sender thread: ``wait``, ``again`` or ``done``.

        Never waits itself, so the sender holds the recorder only while a round runs.
        """
        link = st.link
        with self._lock:
            shut = self._shut
            batch: List[tuple] = []
            if link is not None and not link.dead:
                batch, self._outbox = self._outbox[:BATCH], self._outbox[BATCH:]
                self._assign(batch, link)
            waiting = bool(self._outbox)
        if link is not None and link.dead:  # the frames it held are free again
            self._ring.rewind(self._sent_end)
            if not link.ready:
                st.retry = True  # a child died before it was ready: start the next one now
                self._boot_failures += 1
                if self._boot_failures >= _MAX_BOOT_FAILURES and self._mode == "process":
                    logger.warning("Recorder: the save worker process exited %d times "
                                   "before it was ready; saving on a thread of this "
                                   "process", self._boot_failures)
                    self._mode = "thread"
            link = st.link = None
        if link is None:
            if not waiting and not (st.retry and not shut):
                return "done" if shut else "wait"
            # Started on the first message (start()), restarted after a death.
            st.retry = False
            link = st.link = self._new_link()
            st.links += 1
            with self._lock:
                self._link = link
                self._outbox.insert(0, ("init", self._init_config()))
            return "again"
        if batch:
            try:
                link.send(batch)
            except Exception:  # noqa: BLE001 - the worker is gone; the next one takes over
                logger.warning("Recorder: the save worker stopped taking samples",
                               exc_info=True)
                if isinstance(link, ProcessLink):
                    link.exited()
                return "again"
        if shut and not waiting:
            link.close(self._close_left())
            return "done"
        # State and commands do not wake the sender; it looks every 20 ms.
        return "again" if waiting or shut else "wait"

    def _assign(self, batch: List[tuple], link: Any) -> None:
        """Under ``_lock``: note which worker gets each opened episode and the ring's end."""
        for msg in batch:
            kind = msg[0]
            if kind == "open":
                ep = self._by_number.get(msg[1])
                if ep is not None:
                    ep.link = link
            elif kind == "frame":
                self._sent_end = msg[6]

    def _init_config(self) -> Dict[str, Any]:
        return {"embodiment": self.embodiment, "options": self._options(),
                "root": str(self.root), "robot_config": self.robot_config}

    def _close_left(self) -> Optional[float]:
        d = self._close_deadline
        return None if d is None else max(0.0, d - time.monotonic())

    # ------------------------------------------------------------ events

    def _event(self, link: Any, msg: tuple) -> None:
        kind = msg[0]
        if kind == "ready":
            self._boot_failures = 0
            return
        if kind == "exit":
            self._worker_exited(link)
            return
        number = msg[1]
        with self._lock:
            ep = self._live.get(number)
        if ep is None or ep.done:
            return
        if kind == "saving":
            ep.saving_sent = True
            self._emit(SaveEvent(episode=number, state="saving", task=ep.task))
        elif kind == "saved":
            _, _, path, flags, frames = msg
            self._finish(ep, SaveEvent(episode=number, state="saved", path=path,
                                       flags=tuple(QcFlag(n, d) for n, d in flags),
                                       task=ep.task, frames=frames))
        elif kind == "failed":
            _, _, path, error, trace = msg
            logger.warning("Recorder: episode %d (%r) was not saved: %s\n%s", number, ep.task,
                           error, trace)
            self._finish(ep, SaveEvent(episode=number, state="failed", path=path, error=error,
                                       task=ep.task))

    def _finish(self, ep: _Episode, event: SaveEvent) -> None:
        if not ep.saving_sent:
            ep.saving_sent = True
            self._emit(SaveEvent(episode=ep.number, state="saving", task=ep.task))
        ep.done = True
        self._emit(event)
        with self._lock:
            self._live.pop(ep.number, None)
            self._by_number.pop(ep.number, None)
            self._pending -= 1
            self._queued_bytes -= ep.image_bytes
            self._lock.notify_all()

    def _worker_exited(self, link: Any) -> None:
        """A worker process exited: its episodes fail; the next one starts on demand."""
        code = getattr(getattr(link, "proc", None), "returncode", None)
        with self._lock:
            current = self._episode
            if current is not None and current.link is link:
                current.lost = (f"RuntimeError: the save worker exited (code {code}) while "
                                f"this episode was recording")
            stopped = sorted((ep for ep in self._live.values()
                              if ep.link is link and not ep.done), key=lambda e: e.number)
            self._wake.notify_all()
        if current is not None and current.lost:
            logger.warning("Recorder: the save worker exited (code %s) while episode %d was "
                           "recording; it will be reported failed", code, current.number)
        for ep in stopped:
            self._finish(ep, SaveEvent(
                episode=ep.number, state="failed", task=ep.task,
                error=f"RuntimeError: the save worker exited (code {code}) before it saved "
                      f"this episode"))

    def _emit(self, event: SaveEvent) -> None:
        for sub in self._subscribers:
            try:
                sub.callback(event)
            except Exception:  # noqa: BLE001 - a subscriber never stops the saving
                sub.errors += 1
                logger.exception("Recorder: save-event callback %r failed", sub.callback)

    def __repr__(self) -> str:
        return (f"Recorder({str(self.root)!r}, fps={self.fps}, worker={self._mode!r}, "
                f"recording={self.recording}, queue_length={self.queue_length})")


# ------------------------------------------------------------ threads (hold the recorder weakly)


class _Signal(threading.Condition):
    """The sender's wake-up: a condition that remembers a notify nobody waited for."""

    def __init__(self, lock: Any) -> None:
        super().__init__(lock)
        self.pending = False

    def notify(self, n: int = 1) -> None:
        self.pending = True
        super().notify(n)

    def notify_all(self) -> None:
        self.pending = True
        super().notify_all()


class _Events:
    """What the dispatcher thread reads: worker replies and exits, then the sender's end."""

    def __init__(self) -> None:
        self._q: "queue.SimpleQueue[tuple]" = queue.SimpleQueue()

    def put(self, item: tuple) -> None:
        self._q.put(item)

    def get(self) -> tuple:
        return self._q.get()

    def reply(self, link: Any, msg: tuple) -> None:
        self._q.put((link, msg))

    def exit(self, link: Any) -> None:
        self._q.put((link, ("exit",)))

    def final(self, links: int) -> None:
        """The sender is done; it started ``links`` workers, each sends one exit."""
        self._q.put((None, ("final", links)))


class _Weak:
    """A listener callback that calls ``recorder.<name>`` while the recorder lives."""

    __slots__ = ("_ref", "_name")

    def __init__(self, ref: "weakref.ref[Recorder]", name: str) -> None:
        self._ref, self._name = ref, name

    def __call__(self, value: Any) -> None:
        rec = self._ref()
        if rec is not None:
            getattr(rec, self._name)(value)

    def __repr__(self) -> str:
        return f"<Recorder.{self._name} (weak)>"


def _weak(ref: "weakref.ref[Recorder]", name: str) -> _Weak:
    return _Weak(ref, name)


@dataclass
class _SenderState:
    """What the sender thread keeps between rounds (outside the recorder)."""

    link: Any = None
    retry: bool = False
    #: Workers started so far (each one ends with an ``exit`` event).
    links: int = 0


#: How often an idle sender without a worker checks that its recorder still exists.
_IDLE_S = 0.5


def _send_loop(me: "weakref.ref[Recorder]", wake: _Signal, events: _Events, handle: Any,
               ring: Any, ring_lock: Any) -> None:
    """The sender thread: rounds of :meth:`Recorder._send_step`, the recorder held only in one.

    When it ends (``close()``, or the recorder was collected) nothing in this
    process reads the frame ring any more, so it closes the ring and its memfd.
    """
    st = _SenderState()
    try:
        while True:
            rec = me()
            if rec is None:
                _abandon(st, handle)
                return
            try:
                step = rec._send_step(st)
            except Exception:  # noqa: BLE001 - the sender must outlive any bug
                logger.exception("Recorder: the sender thread failed; going on")
                step = "pause"
            del rec
            if step == "done":
                return
            if step == "again":
                continue
            timeout = 0.05 if step == "pause" else 0.02 if st.link is not None else _IDLE_S
            with wake:
                if not wake.pending:
                    wake.wait(timeout)
                wake.pending = False
    finally:
        with ring_lock:  # a listener copying a frame in finishes first
            ring.close()
        events.final(st.links)


def _poke(wake: _Signal) -> None:
    """The recorder's finalizer: wake its sender, which then finds it gone."""
    if wake.acquire(blocking=False):  # never block in a finalizer
        try:
            wake.notify_all()
        finally:
            wake.release()


def _abandon(st: _SenderState, handle: Any) -> None:
    """The recorder was collected without ``close()``: detach and end its worker."""
    logger.debug("Recorder: collected without close(); ending its save worker")
    try:
        handle.remove()
    except Exception:  # noqa: BLE001 - the robot may be gone too
        pass
    link = st.link
    if link is not None and not link.dead:
        try:
            link.close(None)  # a child still saves the episodes it was sent
        except Exception:  # noqa: BLE001
            logger.debug("Recorder: closing an abandoned save worker failed", exc_info=True)


def _event_loop(me: "weakref.ref[Recorder]", events: _Events) -> None:
    """The dispatcher thread: hand worker replies to the recorder (and its subscribers).

    Ends once the sender is done and every worker it started has exited, so the
    last event is delivered before it goes.
    """
    exits, target = 0, None
    while target is None or exits < target:
        link, msg = events.get()
        if msg[0] == "final":
            target = msg[1]
            continue
        if msg[0] == "exit":
            exits += 1
        rec = me()
        if rec is None:
            continue
        try:
            rec._event(link, msg)
        except Exception:  # noqa: BLE001 - the event thread survives anything
            logger.exception("Recorder: handling %r from the save worker failed", msg[:2])
        finally:
            rec = None


def _lease_events(events: List[Any], ep: _Episode) -> List[Dict[str, Any]]:
    """Claims, releases and preemptions while recording, from ``command_events()``."""
    return [{"t": e.t, "kind": e.kind, "lease": e.lease, "priority": e.priority,
             "parts": e.parts, "detail": e.detail} for e in events
            if e.kind in ("claim", "release", "preempted")
            and ep.t_start - 1e-9 <= e.t <= ep.t_stop + 1e-9]


__all__ = ["CAMERA_FPS", "MAX_QUEUE", "RING_MB", "Recorder", "RecorderError", "SaveEvent",
           "Subscription", "WORKERS", "slug"]
