"""The microphone and speaker contracts and their registry.

Audio devices are separate from the body driver, like cameras and hands:
``make_robot`` builds one per microphone / speaker declared with a ``driver:``
in the embodiment config and attaches them to the :class:`~hirobot.assembly.RobotAssembly`,
which opens them on ``connect()`` and closes them on ``disconnect()``.

Audio is a stream, not a per-tick signal, so it does not go into
:class:`~hirobot.core.types.Observation`. Callers (an interaction layer such as
hiLive) take the devices from ``robot.microphones`` / ``robot.speakers`` and
read or play on their own threads.

All PCM is signed 16-bit little-endian. A microphone yields mono chunks at its
``sample_rate``; a speaker accepts mono PCM at any rate and resamples to its own.
"""

from __future__ import annotations

import collections
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Deque, Dict, Optional, Sequence, Tuple

import numpy as np

from hirobot.loading.registry import Registry

#: Bytes per sample of the PCM every audio device exchanges (int16).
SAMPLE_WIDTH = 2

@dataclass
class AudioChunk:
    """A block of microphone audio.

    Attributes:
        pcm: Mono int16 little-endian samples.
        sample_rate: Samples per second.
        stamp: Host time (``time.time()``) at which the block was received.
    """

    pcm: bytes
    sample_rate: int
    stamp: float

    @property
    def duration(self) -> float:
        """Length in seconds."""
        return len(self.pcm) / (SAMPLE_WIDTH * self.sample_rate)


def resample(pcm: bytes, src_rate: int, dst_rate: int) -> bytes:
    """Resample mono int16 PCM by linear interpolation.

    Good enough for speech (TTS at 22.05 / 24 kHz into a 16 kHz speaker); it
    applies no anti-aliasing filter, so it is not meant for music.
    """
    if src_rate == dst_rate or not pcm:
        return pcm
    x = np.frombuffer(pcm[: len(pcm) // SAMPLE_WIDTH * SAMPLE_WIDTH], dtype="<i2")
    n_out = round(len(x) * dst_rate / src_rate)
    if n_out == 0:
        return b""
    t = np.arange(n_out) * (src_rate / dst_rate)
    y = np.interp(t, np.arange(len(x)), x.astype(np.float64))
    return np.clip(np.round(y), -32768, 32767).astype("<i2").tobytes()


class AudioDriver(ABC):
    """Shared base of microphones and speakers.

    Args:
        name: The device's name; its key in ``robot.microphones`` / ``robot.speakers``.
    """

    #: Samples per second the device works at.
    sample_rate: int = 16000

    #: True for devices usable without the robot (WAV files, the host's own
    #: microphone). ``make_robot`` attaches only these to a simulated driver.
    simulated: bool = False

    def __init__(self, name: str) -> None:
        self.name = name

    @abstractmethod
    def open(self) -> None:
        """Connect to the device. Called once by the robot's ``connect()``."""

    @abstractmethod
    def close(self) -> None:
        """Release the device. Safe to call more than once."""


class MicrophoneDriver(AudioDriver):
    """A microphone that delivers mono int16 PCM in chunks.

    Implementations receive on a background thread and queue chunks; one
    consumer reads them in order with :meth:`read`.
    """

    @abstractmethod
    def read(self, timeout: Optional[float] = None) -> Optional[AudioChunk]:
        """The next chunk, oldest first.

        Blocks up to ``timeout`` seconds (forever if ``None``). Returns ``None``
        on timeout or once the microphone is closed; :attr:`ended` tells which.
        """

    @property
    def ended(self) -> bool:
        """True once no more audio will come: closed, or a file ran out and was read."""
        return False


class SpeakerDriver(AudioDriver):
    """A speaker that plays mono int16 PCM.

    :meth:`play` blocks until the audio has played or :meth:`stop` cuts it
    short; :meth:`stop` is called from another thread.
    """

    @abstractmethod
    def play(self, pcm: bytes, sample_rate: Optional[int] = None) -> float:
        """Play ``pcm`` (mono int16 at ``sample_rate``, default the speaker's).

        Returns the seconds actually played: less than the audio's length when
        :meth:`stop` interrupted it.
        """

    @abstractmethod
    def stop(self) -> None:
        """Cut the current playback short. A no-op when nothing is playing."""


class QueuedMicrophone(MicrophoneDriver):
    """A microphone that runs a blocking ``_receive()`` on a background thread.

    Subclasses implement :meth:`_open`, :meth:`_receive` and optionally
    :meth:`_close`. Received chunks queue up to ``queue_s`` seconds of audio; a
    full queue drops its oldest chunk and counts it in :attr:`dropped`. A
    receive that raises is retried after ``retry_s``, with the last error kept
    on :attr:`error`.

    Args:
        name: Microphone name.
        queue_s: Seconds of audio kept for a slow reader.
        retry_s: Pause after a failed receive.
    """

    def __init__(self, name: str, queue_s: float = 10.0, retry_s: float = 0.2) -> None:
        super().__init__(name)
        self._queue_bytes = int(queue_s * self.sample_rate) * SAMPLE_WIDTH
        self._retry_s = retry_s
        self._chunks: Deque[AudioChunk] = collections.deque()
        self._queued = 0
        self._cond = threading.Condition()
        self._closed = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[BaseException] = None
        self.chunks = 0
        self.dropped = 0

    def _open(self) -> None:
        """Connect to the device. Runs in the caller's thread, so errors raise."""

    @abstractmethod
    def _receive(self) -> Optional[bytes]:
        """Block briefly for the next PCM block; ``None`` if none arrived."""

    def _close(self) -> None:
        """Disconnect from the device."""

    def open(self) -> None:
        if self._thread is not None:
            return
        self._open()
        self._closed.clear()
        with self._cond:
            self._chunks.clear()
            self._queued = 0
        self._thread = threading.Thread(target=self._loop, name=f"mic-{self.name}", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._closed.is_set():
            try:
                pcm = self._receive()
            except Exception as exc:  # noqa: BLE001 - a network hiccup must not end the stream
                if self._closed.is_set():
                    return
                self.error = exc
                self._closed.wait(self._retry_s)
                continue
            if not pcm:
                continue
            chunk = AudioChunk(pcm=pcm, sample_rate=self.sample_rate, stamp=time.time())
            with self._cond:
                self._chunks.append(chunk)
                self._queued += len(pcm)
                while self._queued > self._queue_bytes and len(self._chunks) > 1:
                    self._queued -= len(self._chunks.popleft().pcm)
                    self.dropped += 1
                self._cond.notify_all()
            self.chunks += 1

    @property
    def ended(self) -> bool:
        with self._cond:
            return self._closed.is_set() and not self._chunks

    def read(self, timeout: Optional[float] = None) -> Optional[AudioChunk]:
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cond:
            while not self._chunks:
                if self._closed.is_set() or self._thread is None:
                    return None
                wait = None if deadline is None else deadline - time.monotonic()
                if wait is not None and wait <= 0:
                    return None
                self._cond.wait(wait if wait is not None else 0.5)
            chunk = self._chunks.popleft()
            self._queued -= len(chunk.pcm)
            return chunk

    def wait_for_audio(self, timeout: float) -> None:
        """Block until the first chunk arrives (useful right after ``open()``)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.chunks:
                return
            time.sleep(0.01)
        detail = f": last error {self.error!r}" if self.error is not None else ""
        raise TimeoutError(
            f"microphone {self.name!r} delivered no audio within {timeout:.1f}s{detail}"
        )

    def close(self) -> None:
        self._closed.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
            self._close()


AUDIO: Registry = Registry("audio driver", (MicrophoneDriver, SpeakerDriver))
#: Class decorator that registers a microphone or speaker driver under a name.
register_audio = AUDIO.register
#: Look up a registered microphone or speaker driver class by name.
get_audio = AUDIO.get
#: Names of all registered audio drivers.
available_audio = AUDIO.available


def build_audio(microphones: Sequence[Any], speakers: Sequence[Any],
                network_interface: Optional[str] = None, sim_only: bool = False,
                ) -> Tuple[Dict[str, MicrophoneDriver], Dict[str, SpeakerDriver]]:
    """Microphone and speaker drivers for the devices with a ``driver:`` spec.

    Same rules as :func:`hirobot.devices.camera.build_cameras`: ``network_interface``
    fills a hardware driver's NIC, ``sim_only`` builds only ``simulated`` ones.
    """
    built: Tuple[Dict[str, Any], Dict[str, Any]] = ({}, {})
    for kind, devices, base, out in (("microphone", microphones, MicrophoneDriver, built[0]),
                                     ("speaker", speakers, SpeakerDriver, built[1])):
        for dev in devices:
            if not dev.driver:
                continue
            spec = dict(dev.driver)
            if "type" not in spec:
                raise ValueError(f"{kind} {dev.name!r}: driver needs a 'type'")
            cls = get_audio(spec.pop("type"))
            if not issubclass(cls, base):
                raise ValueError(f"{kind} {dev.name!r}: {cls.__name__} is not a {base.__name__}")
            if sim_only and not cls.simulated:
                continue
            if network_interface is not None and not cls.simulated:
                spec.setdefault("network_interface", network_interface)
            out[dev.name] = cls(dev.name, **spec)
    return built
