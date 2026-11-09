"""Host and file audio: ``wav_mic``, ``wav_speaker``, ``null_speaker``, ``sounddevice_*``."""

from hirobot.drivers.audio.generic.sounddevice import SoundDeviceMicrophone, SoundDeviceSpeaker
from hirobot.drivers.audio.generic.wav import (
    NullSpeaker,
    WavMicrophone,
    WavSpeaker,
    read_wav,
    wav_bytes,
)

__all__ = ["SoundDeviceMicrophone", "SoundDeviceSpeaker", "NullSpeaker", "WavMicrophone",
           "WavSpeaker", "read_wav", "wav_bytes"]
