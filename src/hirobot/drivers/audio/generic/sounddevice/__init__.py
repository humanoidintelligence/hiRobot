"""The host's microphone and speaker through sounddevice
(registered as ``sounddevice_mic``, ``sounddevice_speaker``)."""

from hirobot.drivers.audio.generic.sounddevice.driver import (
    SoundDeviceMicrophone,
    SoundDeviceSpeaker,
)

__all__ = ["SoundDeviceMicrophone", "SoundDeviceSpeaker"]
