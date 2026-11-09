"""The computer's own microphone and speaker, through ``sounddevice``.

For running a robot's listen / speak pipeline on a laptop, e.g. a simulated
robot you talk to. ``sounddevice`` is the ``audio`` extra, imported on
``open()`` / ``play()``::

    microphones:
      - name: laptop
        driver: {type: sounddevice_mic}
    speakers:
      - name: laptop
        driver: {type: sounddevice_speaker}
"""

from __future__ import annotations

import queue
import threading
from typing import Any, Optional, Union

from hirobot.audio import (
    SAMPLE_WIDTH,
    QueuedMicrophone,
    SpeakerDriver,
    register_audio,
)
from hirobot.sdk import require_sdk

Device = Union[int, str, None]


def _sounddevice() -> Any:
    require_sdk(__package__, "sounddevice_mic / sounddevice_speaker")
    import sounddevice

    return sounddevice


@register_audio("sounddevice_mic")
class SoundDeviceMicrophone(QueuedMicrophone):  # pragma: no cover - needs hardware
    """The default (or named) input device.

    Args:
        name: Microphone name.
        sample_rate: Capture rate; 16 kHz is what most ASR models expect.
        chunk_ms: Length of each captured block.
        device: sounddevice device name or index; ``None`` is the default input.
    """

    #: The host's own device, usable next to a simulator.
    simulated = True

    def __init__(
        self, name: str, sample_rate: int = 16000, chunk_ms: int = 20, device: Device = None
    ) -> None:
        self.sample_rate = sample_rate
        super().__init__(name)
        self.chunk_ms = chunk_ms
        self.device = device
        self._q: "queue.Queue[bytes]" = queue.Queue()
        self._stream: Any = None

    def _open(self) -> None:
        sd = _sounddevice()
        self._stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="int16",
            blocksize=int(self.sample_rate * self.chunk_ms / 1000),
            device=self.device,
            callback=lambda indata, n, t, status: self._q.put(bytes(indata)),
        )
        self._stream.start()

    def _receive(self) -> Optional[bytes]:
        try:
            return self._q.get(timeout=0.1)
        except queue.Empty:
            return None

    def _close(self) -> None:
        if self._stream is not None:
            self._stream.stop()
            self._stream.close()
            self._stream = None


@register_audio("sounddevice_speaker")
class SoundDeviceSpeaker(SpeakerDriver):  # pragma: no cover - needs hardware
    """The default (or named) output device, written in short blocks so stop is prompt.

    Args:
        name: Speaker name.
        device: sounddevice device name or index; ``None`` is the default output.
        block_ms: Length of each written block; :meth:`stop` takes effect between blocks.
    """

    #: The host's own device, usable next to a simulator.
    simulated = True

    def __init__(self, name: str, device: Device = None, block_ms: int = 50) -> None:
        super().__init__(name)
        self.device = device
        self.block_ms = block_ms
        self._stop = threading.Event()

    def open(self) -> None:
        _sounddevice()

    def play(self, pcm: bytes, sample_rate: Optional[int] = None) -> float:
        sd = _sounddevice()
        rate = sample_rate or self.sample_rate
        self._stop.clear()
        step = SAMPLE_WIDTH * max(1, int(rate * self.block_ms / 1000))
        played = 0
        with sd.RawOutputStream(samplerate=rate, channels=1, dtype="int16", device=self.device) as out:
            for i in range(0, len(pcm), step):
                if self._stop.is_set():
                    break
                block = pcm[i:i + step]
                out.write(block)
                played += len(block)
        return played / (SAMPLE_WIDTH * rate)

    def stop(self) -> None:
        self._stop.set()

    def close(self) -> None:
        self._stop.set()
