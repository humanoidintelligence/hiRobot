"""WAV files as microphone and speaker, plus a silent speaker
(registered as ``wav_mic``, ``wav_speaker``, ``null_speaker``)."""

from hirobot.drivers.audio.generic.wav.driver import (
    NullSpeaker,
    WavMicrophone,
    WavSpeaker,
    read_wav,
    wav_bytes,
)

__all__ = ["WavMicrophone", "WavSpeaker", "NullSpeaker", "read_wav", "wav_bytes"]
