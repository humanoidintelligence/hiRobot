"""The G1 voice devices pass the audio contract: the speaker on a fake voice service,
the microphone on a local UDP socket (unicast reaches the same receive path)."""

import socket

import pytest

from hirobot.drivers.audio import UnitreeG1Microphone, UnitreeG1Speaker
from tests.drivers.audio.unitree.sdk2_voice.fake_sdk import _FakeVoice
from tests.drivers.contract import check_microphone, check_speaker


def test_speaker_contract(monkeypatch):
    voice = _FakeVoice().install(monkeypatch)
    check_speaker("unitree_voice_speaker",
                  lambda: UnitreeG1Speaker("head", chunk_s=0.1, lead_s=0.1), voice.sent_seconds)


def test_microphone_contract():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]

    def make():
        mic = UnitreeG1Microphone("head", local_ip="127.0.0.1", group="239.255.77.77", port=0)
        mic._port = port
        return mic

    def feed(mic):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            tx.sendto(b"\x05\x00" * 2560, ("127.0.0.1", port))

    try:
        check_microphone("unitree_voice_mic", make, feed=feed)
    except OSError as exc:  # pragma: no cover - a host without multicast
        pytest.skip(f"cannot join a multicast group here: {exc}")
