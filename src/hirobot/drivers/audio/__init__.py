"""Microphones and speakers, attached to a robot independently of the body.

Each is declared in the embodiment config with a ``driver:``; with a real body
driver ``make_robot`` builds it and the :class:`~hirobot.assembly.RobotAssembly`
opens it on ``connect()`` and closes it on ``disconnect()``::

    microphones:
      - name: head
        driver:
          type: unitree_voice_mic     # a registered audio driver
    speakers:
      - name: head
        driver:
          type: unitree_voice_speaker
          volume: 60                  # remaining keys go to the driver

The devices are reached as ``robot.microphones[name]`` / ``robot.speakers[name]``.
Vendor SDKs are imported lazily on ``open()``.
"""

from hirobot.drivers.audio.generic import (
    NullSpeaker,
    SoundDeviceMicrophone,
    SoundDeviceSpeaker,
    WavMicrophone,
    WavSpeaker,
    read_wav,
    wav_bytes,
)
from hirobot.drivers.audio.unitree.sdk2_voice import UnitreeG1Microphone, UnitreeG1Speaker

__all__ = [
    "UnitreeG1Microphone",
    "UnitreeG1Speaker",
    "SoundDeviceMicrophone",
    "SoundDeviceSpeaker",
    "WavMicrophone",
    "WavSpeaker",
    "NullSpeaker",
    "read_wav",
    "wav_bytes",
]
