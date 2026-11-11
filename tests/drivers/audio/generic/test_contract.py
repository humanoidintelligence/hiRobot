"""The stand-in audio devices pass the audio contract."""

from hirobot.drivers.audio import NullSpeaker, WavMicrophone, WavSpeaker
from hirobot.drivers.audio.generic.wav import wav_bytes
from tests.drivers.contract import check_microphone, check_speaker


def test_wav_mic_contract(tmp_path):
    path = tmp_path / "in.wav"
    path.write_bytes(wav_bytes(b"\x01\x00" * 1600, 16000))
    check_microphone("wav_mic", lambda: WavMicrophone("m", str(path), realtime=False))


def test_wav_speaker_contract(tmp_path):
    check_speaker("wav_speaker", lambda: WavSpeaker("s", str(tmp_path / "out.wav"), realtime=False))


def test_null_speaker_contract():
    check_speaker("null_speaker", lambda: NullSpeaker("s", realtime=False))
