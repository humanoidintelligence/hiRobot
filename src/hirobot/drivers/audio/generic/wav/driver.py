"""WAV files as a microphone and a speaker, plus a speaker that plays nothing.

They let the whole listen / speak pipeline run without audio hardware: a
recorded question goes in through ``wav_mic``, the answer is recorded by
``wav_speaker``. Standard library only (``wave``)::

    microphones:
      - name: head
        driver: {type: wav_mic, path: assets/hello.wav}
    speakers:
      - name: head
        driver: {type: wav_speaker, path: out/reply.wav}
"""

from __future__ import annotations

import io
import threading
import time
import wave
from array import array
from typing import List, Optional, Tuple

from hirobot.audio import (
    SAMPLE_WIDTH,
    QueuedMicrophone,
    SpeakerDriver,
    register_audio,
    resample,
)


def read_wav(path: str) -> Tuple[bytes, int]:
    """Mono int16 PCM and sample rate of a 16-bit WAV file (first channel kept)."""
    with wave.open(path, "rb") as w:
        if w.getsampwidth() != SAMPLE_WIDTH:
            raise ValueError(f"{path}: expected 16-bit WAV, got {8 * w.getsampwidth()}-bit")
        channels = w.getnchannels()
        rate = w.getframerate()
        pcm = w.readframes(w.getnframes())
    if channels > 1:
        samples = array("h")
        samples.frombytes(pcm)
        pcm = samples[::channels].tobytes()
    return pcm, rate


def wav_bytes(pcm: bytes, sample_rate: int) -> bytes:
    """Mono int16 PCM wrapped as a WAV file."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(SAMPLE_WIDTH)
        w.setframerate(sample_rate)
        w.writeframes(pcm)
    return buf.getvalue()


@register_audio("wav_mic")
class WavMicrophone(QueuedMicrophone):
    """Play a WAV file into the microphone queue, then end the stream.

    Args:
        name: Microphone name.
        path: 16-bit WAV file. Its sample rate becomes the microphone's.
        chunk_ms: Length of each queued chunk.
        realtime: Deliver chunks at speaking speed (what barge-in needs).
            Off queues the whole file at once, so tests do not wait.
        tail_silence: Seconds of silence after the file, so voice-activity
            detection sees the last utterance end.

    After the last chunk, :meth:`read` returns ``None`` (the end of the stream).
    """

    #: Not real hardware: ``make_robot`` attaches it to simulated drivers too.
    simulated = True

    def __init__(
        self,
        name: str,
        path: str,
        chunk_ms: int = 20,
        realtime: bool = True,
        tail_silence: float = 1.0,
    ) -> None:
        pcm, rate = read_wav(path)
        self.sample_rate = rate
        pcm += b"\x00" * (SAMPLE_WIDTH * int(tail_silence * rate))
        step = SAMPLE_WIDTH * max(1, int(rate * chunk_ms / 1000))
        self._pending: List[bytes] = [pcm[i:i + step] for i in range(0, len(pcm), step)]
        self.path = path
        self.chunk_ms = chunk_ms
        self.realtime = realtime
        # Without pacing the whole file is queued at once: the queue must hold it.
        queue_s = 10.0 if realtime else len(pcm) / (SAMPLE_WIDTH * rate) + 1.0
        super().__init__(name, queue_s=queue_s)
        self._next = 0

    def _open(self) -> None:
        self._next = 0

    def _receive(self) -> Optional[bytes]:
        if self._next >= len(self._pending):
            # End of file: closing makes read() return None once the queue drains.
            self._closed.set()
            return None
        chunk = self._pending[self._next]
        self._next += 1
        if self.realtime:
            time.sleep(self.chunk_ms / 1000.0)
        return chunk


class _TimedSpeaker(SpeakerDriver):
    """Shared part of the file and null speakers: take as long as the audio lasts."""

    #: Not real hardware: ``make_robot`` attaches it to simulated drivers too.
    simulated = True

    def __init__(self, name: str, realtime: bool = True, sample_rate: int = 16000) -> None:
        super().__init__(name)
        self.realtime = realtime
        self.sample_rate = sample_rate
        self._stop = threading.Event()

    def open(self) -> None:
        pass

    def _played_seconds(self, duration: float) -> float:
        self._stop.clear()
        if not self.realtime:
            return duration
        start = time.monotonic()
        self._stop.wait(duration)
        return min(duration, time.monotonic() - start)

    def stop(self) -> None:
        self._stop.set()

    def close(self) -> None:
        self._stop.set()


@register_audio("wav_speaker")
class WavSpeaker(_TimedSpeaker):
    """Record everything played into one WAV file, written on :meth:`close`.

    Args:
        name: Speaker name.
        path: Output WAV path.
        realtime: Take as long as the audio lasts, so :meth:`stop` cuts a
            sentence partway (what barge-in tests check). Off returns at once.
        sample_rate: Rate of the recording; played audio is resampled to it.
    """

    def __init__(self, name: str, path: str, realtime: bool = True, sample_rate: int = 16000) -> None:
        super().__init__(name, realtime=realtime, sample_rate=sample_rate)
        self.path = path
        self._pcm: List[bytes] = []

    def play(self, pcm: bytes, sample_rate: Optional[int] = None) -> float:
        pcm = resample(pcm, sample_rate or self.sample_rate, self.sample_rate)
        duration = len(pcm) / (SAMPLE_WIDTH * self.sample_rate)
        played = self._played_seconds(duration)
        keep = SAMPLE_WIDTH * round(played * self.sample_rate)
        self._pcm.append(pcm[:keep])
        return played

    def close(self) -> None:
        super().close()
        if not self._pcm:
            return
        with open(self.path, "wb") as f:
            f.write(wav_bytes(b"".join(self._pcm), self.sample_rate))
        self._pcm = []


@register_audio("null_speaker")
class NullSpeaker(_TimedSpeaker):
    """Play nothing, but take as long as the audio would (``realtime``)."""

    def play(self, pcm: bytes, sample_rate: Optional[int] = None) -> float:
        rate = sample_rate or self.sample_rate
        return self._played_seconds(len(pcm) / (SAMPLE_WIDTH * rate))
