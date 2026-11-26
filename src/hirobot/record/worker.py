"""The save worker: align, encode and write episodes, in a subprocess or on a thread.

The :class:`~hirobot.record.Recorder` streams each episode to its worker while
it records, so that nothing is left to do in the control process after
``stop()``. Messages, in order per episode:

- ``("init", config)`` once per worker (embodiment, options, root), then per
  episode ``("open", number, task, wall_start, t_start)`` from ``start()``;
- ``("state", number, (t, stamps, observation))`` per kept state reading and
  ``("command", number, (record, targets))`` per command (to a subprocess each
  message is pickled on its own, so a sample that does not pickle or unpickle
  is counted as dropped instead of breaking the stream);
- ``("frame", number, camera, t, seq, offset, end, shape, dtype)`` per kept
  image, whose pixels the listener copied into the :class:`FrameRing`;
- ``("dropped", number, source, count)`` for samples lost on the way;
- ``("stop", number, fields)`` (stop times, ``meta``, fault, lease events,
  drop counts) or ``("cancel", number)``; ``("exit",)`` to end the worker.

The worker copies each frame out of the ring at once (which frees the space),
collects each open episode, and after ``stop`` saves it on its own thread:
aligned (:func:`~hirobot.record.align.align`), videos encoded, the dataset
written and ``meta/qc.jsonl`` appended. It answers ``("ready", pid)`` once,
then per stopped episode ``("saving", number)`` and ``("saved", number, path,
flags, frames)`` or ``("failed", number, path, error, trace)``, episodes in
the order they were stopped.

:class:`ProcessLink` runs the worker in a fresh interpreter (``python -I -c``
with this process's ``sys.path``, the socket and the ring's memfd passed as
file descriptors); the child never
imports a robot driver, it gets the embodiment in an ``("init", ...)``
message. :class:`ThreadLink` runs the same code on threads of this process.
"""

from __future__ import annotations

import json
import logging
import mmap
import os
import pickle
import queue
import re
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from hirobot.record.align import RawEpisode, align
from hirobot.record.qc import append_line

logger = logging.getLogger(__name__)

#: Bytes before the ring's data area; the consumer's position (int64) is at 0.
HEADER = 64

#: Frames start on multiples of this many bytes.
ALIGN = 64

#: Messages the sender packs into one write to the subprocess, at most.
BATCH = 256

_PROTOCOL = pickle.HIGHEST_PROTOCOL

#: What the child runs (under ``python -I``: no cwd, no ``PYTHON*`` variables, no
#: user site): the parent's ``sys.path`` in place of its own, then :func:`main`.
_BOOT = ("import json, sys; sys.path[:] = json.loads(sys.argv[1]); "
         "from hirobot.record.worker import main; "
         "main(int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))")


def _child_path() -> List[str]:
    """This process's ``sys.path`` for the child, relative entries made absolute.

    The cwd (``""`` or ``"."``) goes last, so a stray ``pickle.py`` there cannot
    shadow the standard library while a package found only there still imports.
    """
    paths = [p for p in sys.path if isinstance(p, str)]
    here = [os.path.abspath(p) for p in paths if p in ("", ".")]
    rest = [os.path.abspath(p) for p in paths if p not in ("", ".")]
    return [p for p in rest if p not in here] + here[:1]


def slug(task: str, limit: int = 40) -> str:
    """``task`` as a directory-name part: lower-case ASCII letters, digits and ``-``."""
    s = re.sub(r"[^a-z0-9]+", "-", task.lower()).strip("-")
    return (s[:limit].rstrip("-") or "task")


def can_use_process() -> bool:
    """Whether this platform can run the save worker in a subprocess (needs ``memfd``)."""
    return hasattr(os, "memfd_create") and os.name == "posix" and bool(sys.executable)


# ---------------------------------------------------------------- the frame ring


class FrameRing:
    """A preallocated byte ring in shared memory: one producer, one consumer.

    The producer (the recorder's listener) reserves room and copies a frame
    in; the consumer (the worker) copies it out and releases up to the frame's
    end. Positions only grow; a frame sits at ``position % data_size`` and
    never wraps (the tail is skipped instead). The consumer's position is in
    the header, so a child process can move it.

    The buffer is a ``memfd`` when the platform has one (its ``fd`` goes to
    the child), else an anonymous mapping a thread worker can use. Every page
    is written once at creation, so recording never faults a page in.
    """

    def __init__(self, size: int, fd: Optional[int] = None) -> None:
        self.size = int(size)
        self.data_size = self.size - HEADER
        if self.data_size < ALIGN:
            raise ValueError(f"a frame ring needs more than {HEADER + ALIGN} bytes, got {size}")
        self.fd = fd
        self.closed = False
        self._mm = mmap.mmap(-1 if fd is None else fd, self.size)
        self._buf = np.frombuffer(self._mm, dtype=np.uint8)
        self._read = np.frombuffer(self._mm, dtype=np.int64, count=1)
        self._write = int(self._read[0])
        self._close_fd: Optional[Callable[[], Any]] = None

    @classmethod
    def create(cls, size: int) -> "FrameRing":
        """A new ring of ``size`` bytes with every page touched."""
        import weakref

        fd = None
        if hasattr(os, "memfd_create"):
            fd = os.memfd_create("hirobot-record", getattr(os, "MFD_CLOEXEC", 0))
            os.ftruncate(fd, int(size))
        ring = cls(size, fd)
        ring._buf[:] = 0
        if fd is not None:
            ring._close_fd = weakref.finalize(ring, os.close, fd)
        return ring

    def close(self) -> None:
        """Unmap the ring and close its memfd (a child keeps its own copy). Safe twice.

        Afterwards :meth:`reserve` finds no room and :meth:`release` does nothing.
        """
        if self.closed:
            return
        self.closed = True
        self._buf = self._read = None  # type: ignore[assignment]
        try:
            self._mm.close()
        except BufferError:  # a view of it is still alive; it is unmapped when that goes
            logger.debug("Recorder: the frame ring is still in use; unmapped later")
        if self._close_fd is not None:
            self._close_fd()

    # ---- producer side (the recorder serializes these)

    @property
    def position(self) -> int:
        """Where the next reservation starts."""
        return self._write

    def reserve(self, nbytes: int) -> Optional[Tuple[int, int, int]]:
        """Room for ``nbytes``: ``(offset, begin, end)``, or ``None`` when it does not fit now.

        ``offset`` is where the bytes go in the data area; ``begin`` and
        ``end`` are positions (``end`` is what the consumer releases).
        """
        n = -(-int(nbytes) // ALIGN) * ALIGN
        size = self.data_size
        if n <= 0 or n > size or self.closed:
            return None
        pos = self._write % size
        pad = size - pos if pos + n > size else 0
        if self._write + pad + n - int(self._read[0]) > size:
            return None
        begin = self._write + pad
        self._write = begin + n
        return begin % size, begin, self._write

    def put(self, offset: int, image: np.ndarray) -> None:
        """Copy ``image`` into the data area at ``offset``."""
        start = HEADER + offset
        dst = self._buf[start:start + image.nbytes].view(image.dtype).reshape(image.shape)
        np.copyto(dst, image, casting="no")

    def rewind(self, position: int) -> None:
        """Free everything before ``position`` (a consumer died; it held frames up to there)."""
        if not self.closed and position > int(self._read[0]):
            self._read[0] = int(position)

    @property
    def free(self) -> int:
        """Bytes not reserved now."""
        if self.closed:
            return 0
        return self.data_size - (self._write - int(self._read[0]))

    # ---- consumer side

    def get(self, offset: int, shape: Sequence[int], dtype: str) -> np.ndarray:
        """A copy of the frame at ``offset``."""
        dt = np.dtype(dtype)
        count = int(np.prod(shape, dtype=np.int64)) * dt.itemsize
        start = HEADER + offset
        return self._buf[start:start + count].view(dt).reshape(tuple(shape)).copy()

    def release(self, end: int) -> None:
        """Free the room of every frame up to position ``end``."""
        if not self.closed and end > int(self._read[0]):
            self._read[0] = int(end)


# ---------------------------------------------------------------- the worker

class Worker:
    """Collects the streamed episodes and saves the stopped ones on its own thread.

    ``handle(msg)`` takes every message in stream order (one thread);
    ``reply(msg)`` is how answers go back. ``ring`` is the :class:`FrameRing`
    the frames are read from. ``on_done()``, when given, runs on the save
    thread after its last reply (after :meth:`finish`).
    """

    def __init__(self, reply: Callable[[tuple], None], ring: FrameRing,
                 on_done: Optional[Callable[[], None]] = None) -> None:
        self._reply = reply
        self._ring = ring
        self._on_done = on_done
        self._open: Dict[int, RawEpisode] = {}
        self._errors: Dict[int, str] = {}
        self._saves: "queue.SimpleQueue[Optional[RawEpisode]]" = queue.SimpleQueue()
        self._cfg: Optional[Dict[str, Any]] = None
        self._saver = threading.Thread(target=self._save_loop, name="hirobot-recorder-save",
                                       daemon=True)
        self._saver.start()

    # ---- stream (the receiving thread)

    def handle(self, msg: tuple) -> None:
        kind = msg[0]
        if kind == "frame":
            _, number, name, t, seq, offset, end, shape, dtype = msg
            ep = self._open.get(number)
            try:
                if ep is not None:
                    ep.images.setdefault(name, []).append(
                        (t, seq, self._ring.get(offset, shape, dtype)))
            finally:
                self._ring.release(end)
        elif kind == "state":
            ep = self._open.get(msg[1])
            if ep is not None:
                ep.state.append(msg[2])
        elif kind == "command":
            ep = self._open.get(msg[1])
            if ep is not None:
                ep.commands.append(msg[2])
        elif kind == "dropped":
            ep = self._open.get(msg[1])
            if ep is not None:
                ep.dropped[msg[2]] = ep.dropped.get(msg[2], 0) + int(msg[3])
        elif kind == "open":
            _, number, task, wall_start, t_start = msg
            self._open[number] = RawEpisode(number=number, task=task, meta={},
                                            wall_start=wall_start, t_start=t_start)
        elif kind == "stop":
            ep = self._open.pop(msg[1], None)
            if ep is None:  # never opened here (the episode began on a worker that is gone)
                self._send(("saving", msg[1]))
                self._send(("failed", msg[1], None, "RuntimeError: the save worker restarted "
                            "while this episode was recording", ""))
                return
            fields = dict(msg[2])
            for key, count in fields.pop("dropped", {}).items():
                ep.dropped[key] = ep.dropped.get(key, 0) + int(count)
            meta = fields.pop("meta", {})
            if isinstance(meta, _Unusable):
                self._errors[ep.number] = meta.reason
            else:
                ep.meta = dict(meta)
            for key, value in fields.items():
                setattr(ep, key, value)
            self._saves.put(ep)
        elif kind == "cancel":
            self._open.pop(msg[1], None)
        elif kind == "init":
            self._cfg = dict(msg[1])

    def finish(self, timeout: Optional[float] = None) -> None:
        """Drop the open episodes, save the stopped ones, then end the save thread."""
        self._open.clear()
        self._saves.put(None)
        self._saver.join(timeout)

    # ---- saving (the save thread)

    def _save_loop(self) -> None:
        try:
            while True:
                ep = self._saves.get()
                if ep is None:
                    return
                try:
                    self._save(ep)
                except BaseException as exc:  # noqa: BLE001 - the worker survives anything
                    self._send(("failed", ep.number, None, f"{type(exc).__name__}: {exc}",
                                traceback.format_exc()))
                finally:
                    ep.state.clear()
                    ep.images.clear()
                    ep.commands.clear()
        finally:
            if self._on_done is not None:
                self._on_done()

    def _send(self, msg: tuple) -> None:
        try:
            self._reply(msg)
        except Exception:  # noqa: BLE001 - the parent is gone; keep saving
            logger.debug("Recorder worker: reply %r not delivered", msg[:2], exc_info=True)

    def _save(self, ep: RawEpisode) -> None:
        from hirobot import __version__
        from hirobot.record.writer import write_dataset

        self._send(("saving", ep.number))
        error = self._errors.pop(ep.number, None)
        if error is not None:
            raise ValueError(error)
        cfg = self._cfg
        if cfg is None:
            raise RuntimeError("the save worker got no configuration")
        path: Optional[Path] = None
        try:
            aligned = align(ep, cfg["embodiment"], cfg["options"])
            flags = list(aligned.flags)
            aligned.report["lease_events"] = ep.lease_events
            path = _dataset_dir(Path(cfg["root"]), ep)
            path.mkdir(parents=True)
            emb = cfg["embodiment"]
            config = ep.meta.get("robot_config", cfg["robot_config"])
            hi_info = {"hi.robot_config": config, "hi.hirobot_version": __version__,
                       "hi.control_hz": emb.control_hz, "hi.recorder_fps": cfg["options"].fps}
            meta = dict(ep.meta, task=ep.task, episode=ep.number,
                        wall_start=ep.wall_start, wall_stop=ep.wall_stop,
                        t_start=ep.t_start, t_stop=ep.t_stop)
            write_dataset(path, aligned, task=ep.task, robot_type=emb.name, hi_info=hi_info,
                          episode_meta=meta)
            append_line(path, {"episode_index": 0, "source": "auto",
                               "verdict": "FAIL" if flags else "PASS",
                               "flags": [f.as_dict() for f in flags], "t": time.time()})
        except BaseException as exc:  # noqa: BLE001 - reported as "failed"
            self._send(("failed", ep.number, None if path is None else str(path),
                        f"{type(exc).__name__}: {exc}", traceback.format_exc()))
            return
        self._send(("saved", ep.number, str(path), [(f.name, f.detail) for f in flags],
                    aligned.length))


class _Unusable:
    """Stands in for an episode's ``meta`` that could not be passed to the worker."""

    def __init__(self, reason: str) -> None:
        self.reason = reason


def _dataset_dir(root: Path, ep: RawEpisode) -> Path:
    stamp = datetime.fromtimestamp(ep.wall_start).strftime("%Y%m%d-%H%M%S")
    base = root / f"{stamp}_{slug(ep.task)}_{ep.number:04d}"
    path, k = base, 1
    while path.exists():
        path, k = Path(f"{base}-{k}"), k + 1
    return path


# ---------------------------------------------------------------- links (the recorder's side)


class ThreadLink:
    """The worker on threads of this process (``Recorder(worker="thread")``)."""

    kind = "thread"

    def __init__(self, ring: FrameRing, on_reply: Callable[["ThreadLink", tuple], None],
                 on_exit: Callable[["ThreadLink"], None]) -> None:
        self.dead = False
        self.ready = True
        # on_exit comes after the save thread's last reply, like a child's exit.
        self._worker = Worker(lambda msg: on_reply(self, msg), ring,
                              on_done=lambda: on_exit(self))

    def send(self, batch: List[tuple]) -> None:
        for msg in batch:
            self._worker.handle(msg)

    def close(self, timeout: Optional[float] = None) -> None:
        """Save the stopped episodes, then end the save thread (waits up to ``timeout``)."""
        self.dead = True
        self._worker.finish(timeout)


class ProcessLink:
    """The worker in a child interpreter, talking over a Unix socket.

    The child is a fresh ``python -I -c`` (not a fork: the control process has
    threads, a simulator and GL contexts); isolated, so neither its cwd nor
    ``PYTHONPATH`` can put a module ahead of the standard library before it
    takes this process's ``sys.path``. It also gets one end of a socket pair
    and the ring's memfd. A reader
    thread turns its replies into ``on_reply(link, msg)`` calls and its exit
    (or a broken socket) into ``on_exit(link)``.
    """

    kind = "process"

    def __init__(self, ring: FrameRing, on_reply: Callable[["ProcessLink", tuple], None],
                 on_exit: Callable[["ProcessLink"], None]) -> None:
        import socket

        if ring.fd is None:
            raise RuntimeError("the frame ring has no memfd to share")
        self.dead = False
        self.ready = False
        self.pid: Optional[int] = None
        self._on_reply = on_reply
        self._on_exit = on_exit
        self._exit_lock = threading.Lock()
        parent, child = socket.socketpair()
        try:
            self.proc = subprocess.Popen(
                [sys.executable, "-I", "-c", _BOOT, json.dumps(_child_path()),
                 str(child.fileno()), str(ring.fd), str(ring.size)],
                pass_fds=(child.fileno(), ring.fd), stdin=subprocess.DEVNULL, close_fds=True)
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        self._sock = parent
        self._reader = threading.Thread(target=self._read_loop, name="hirobot-recorder-events",
                                        daemon=True)
        self._reader.start()

    def send(self, batch: List[tuple]) -> None:
        """Write ``batch`` in one go, each message pickled on its own (see :func:`_pack`)."""
        _write(self._sock, pickle.dumps([_pack(m) for m in batch], _PROTOCOL))

    def close(self, timeout: Optional[float] = None) -> None:
        """Ask the child to finish and exit; wait up to ``timeout`` for it."""
        try:
            _write(self._sock, pickle.dumps([_pack(("exit",))], _PROTOCOL))
        except OSError:
            pass
        try:
            self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            logger.warning("Recorder: the save worker (pid %d) is still running", self.proc.pid)
            return
        self._reader.join(1.0)

    def _read_loop(self) -> None:
        try:
            while True:
                payload = _read(self._sock)
                if payload is None:
                    break
                msg = pickle.loads(payload)
                if msg[0] == "ready":
                    self.ready, self.pid = True, msg[1]
                self._on_reply(self, msg)
        except Exception:  # noqa: BLE001 - a broken stream counts as the child's exit
            logger.debug("Recorder: the save worker's stream broke", exc_info=True)
        self.exited()

    def exited(self) -> None:
        """Mark the child gone (once): close the socket, reap it, tell the recorder."""
        with self._exit_lock:
            if self.dead:
                return
            self.dead = True
        try:
            self._sock.close()
        except OSError:
            pass
        try:
            self.proc.wait(5.0)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self._on_exit(self)


def _pack(msg: tuple) -> Tuple[str, Any, bytes]:
    """``(kind, number, pickled msg[1:])``; a sample that cannot be pickled becomes a drop."""
    kind = msg[0]
    number = msg[1] if kind not in ("init", "exit") else None
    try:
        return kind, number, pickle.dumps(msg[1:], _PROTOCOL)
    except Exception as exc:  # noqa: BLE001 - e.g. a lock in Observation.extra
        if kind in ("state", "command"):
            return _pack(("dropped", number, "state" if kind == "state" else "commands", 1))
        if kind == "stop":
            fields = {k: v for k, v in msg[2].items() if k in ("t_stop", "wall_stop", "dropped")}
            fields["meta"] = _Unusable(f"the episode's meta could not be passed to the save "
                                       f"worker: {type(exc).__name__}: {exc}")
            return _pack(("stop", number, fields))
        raise


def _unpack(item: Tuple[str, Any, bytes]) -> tuple:
    """The message :func:`_pack` packed; a sample that cannot be unpickled becomes a drop."""
    kind, number, payload = item
    try:
        return (kind,) + tuple(pickle.loads(payload))
    except Exception as exc:  # noqa: BLE001 - a class the child cannot import
        if kind in ("state", "command"):
            return ("dropped", number, "state" if kind == "state" else "commands", 1)
        if kind == "stop":
            return ("stop", number, {"meta": _Unusable(
                f"the episode's meta could not be passed to the save worker: {exc}")})
        raise


def _write(sock: Any, payload: bytes) -> None:
    sock.sendall(len(payload).to_bytes(8, "little") + payload)


def _read(sock: Any) -> Optional[bytes]:
    head = _recv(sock, 8)
    if head is None:
        return None
    body = _recv(sock, int.from_bytes(head, "little"))
    if body is None:
        raise EOFError("the stream ended inside a message")
    return body


def _recv(sock: Any, n: int) -> Optional[bytes]:
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            if got == 0:
                return None
            raise EOFError("the stream ended inside a message")
        got += k
    return bytes(buf)


# ---------------------------------------------------------------- the child


def main(sock_fd: int, ring_fd: int, ring_size: int) -> None:
    """The child process: serve one recorder until its socket closes or it says ``exit``.

    Episodes stopped before that are still saved; open ones are dropped.
    Ctrl-C in the terminal does not stop it, so a save in progress finishes.
    """
    import signal
    import socket

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    sock = socket.socket(fileno=sock_fd)
    ring = FrameRing(ring_size, ring_fd)
    lock = threading.Lock()

    def reply(msg: tuple) -> None:
        payload = pickle.dumps(msg, _PROTOCOL)
        with lock:
            _write(sock, payload)

    worker = Worker(reply, ring)
    worker._send(("ready", os.getpid()))  # a parent already gone is logged at debug only
    try:
        done = False
        while not done:
            try:
                payload = _read(sock)
            except (OSError, EOFError):
                break
            if payload is None:
                break
            for item in pickle.loads(payload):
                msg = _unpack(item)
                if msg[0] == "exit":
                    done = True
                    break
                worker.handle(msg)
    finally:
        worker.finish()
        try:
            sock.close()
        except OSError:
            pass


__all__ = ["FrameRing", "ProcessLink", "ThreadLink", "Worker", "can_use_process", "main", "slug"]
