"""G1 microphone (UDP multicast) and speaker (voice RPC): ``unitree_voice_mic`` / ``_speaker``."""

from hirobot.drivers.audio.unitree.sdk2_voice.driver import UnitreeG1Microphone, UnitreeG1Speaker

__all__ = ["UnitreeG1Microphone", "UnitreeG1Speaker"]
