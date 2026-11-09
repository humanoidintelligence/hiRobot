"""``unitree_voice_mic`` / ``unitree_voice_speaker``: the G1 microphone over UDP, and the
speaker against a fake voice service."""

from __future__ import annotations

import socket
import threading
import time

import numpy as np
import pytest

from hirobot.drivers.audio import UnitreeG1Microphone, UnitreeG1Speaker
from tests.drivers.audio.unitree.sdk2_voice.fake_sdk import _FakeVoice

RATE = 16000


def _tone(seconds, rate=RATE):
    t = np.arange(int(seconds * rate)) / rate
    return (8000 * np.sin(2 * np.pi * 440 * t)).astype("<i2").tobytes()


def test_g1_mic_receives_udp_on_localhost():
    # Unicast to the bound port exercises the same receive path as the multicast.
    mic = UnitreeG1Microphone("head", local_ip="127.0.0.1", group="239.255.77.77", port=0)
    mic._port = _free_udp_port()
    try:
        mic.open()
    except OSError as exc:  # pragma: no cover - a host without multicast
        pytest.skip(f"cannot join a multicast group here: {exc}")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            tx.sendto(b"\x05\x00" * 2560 + b"\x07", ("127.0.0.1", mic._port))
        mic.wait_for_audio(1.0)
        chunk = mic.read(1.0)
        assert len(chunk.pcm) == 5120  # the odd trailing byte is dropped
        assert chunk.duration == pytest.approx(0.16)
    finally:
        mic.close()


def _free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_speaker_sets_and_restores_volume(monkeypatch):
    voice = _FakeVoice(volume=100).install(monkeypatch)
    spk = UnitreeG1Speaker("head", network_interface="eth9", volume=60)
    spk.open()
    assert voice.volume == 60 and ("DDS", (0, "eth9")) in voice.calls
    spk.close()
    assert voice.volume == 100
    spk.close()  # idempotent


def test_speaker_leaves_volume_alone_by_default(monkeypatch):
    voice = _FakeVoice(volume=42).install(monkeypatch)
    spk = UnitreeG1Speaker("head")
    spk.open()
    spk.close()
    assert voice.volume == 42 and not any(c[0] == "SetVolume" for c in voice.calls)


def test_speaker_rejects_bad_volume():
    with pytest.raises(ValueError, match="0-100"):
        UnitreeG1Speaker("head", volume=150)


def test_play_blocks_for_the_audio_and_paces_the_sends(monkeypatch):
    voice = _FakeVoice().install(monkeypatch)
    spk = UnitreeG1Speaker("head", chunk_s=0.1, lead_s=0.1)
    spk.open()
    t0 = time.monotonic()
    played = spk.play(_tone(0.5))
    took = time.monotonic() - t0
    spk.close()
    assert played == pytest.approx(0.5)
    assert 0.45 <= took < 0.8
    assert voice.sent_seconds() == pytest.approx(0.5)
    assert len({sid for _, _, sid, _ in voice.streams}) == 1  # one stream per play
    # Paced: the last block goes out well after the first, not all at once.
    assert voice.streams[-1][0] - voice.streams[0][0] > 0.2


def test_play_resamples_to_16k(monkeypatch):
    voice = _FakeVoice().install(monkeypatch)
    spk = UnitreeG1Speaker("head", chunk_s=1.0, lead_s=1.0)
    spk.open()
    assert spk.play(_tone(0.2, 24000), sample_rate=24000) == pytest.approx(0.2)
    spk.close()
    assert voice.sent_seconds() == pytest.approx(0.2)


def test_stop_cuts_playback_and_flushes_the_robot(monkeypatch):
    voice = _FakeVoice().install(monkeypatch)
    spk = UnitreeG1Speaker("head", chunk_s=0.2, lead_s=0.2)
    spk.open()
    threading.Timer(0.3, spk.stop).start()
    t0 = time.monotonic()
    played = spk.play(_tone(3.0))
    took = time.monotonic() - t0
    spk.close()
    assert 0.25 <= played <= 0.45 and took < 0.6
    assert ("PlayStop", "hirobot") in voice.calls
    assert voice.sent_seconds() < 1.0  # most of the clip was never sent


def test_play_raises_on_service_error(monkeypatch):
    voice = _FakeVoice().install(monkeypatch)
    spk = UnitreeG1Speaker("head")
    spk.open()
    voice.code = 3102
    with pytest.raises(RuntimeError, match="3102.*voice service"):
        spk.play(_tone(0.1))
    spk.close()


def test_play_requires_open():
    with pytest.raises(RuntimeError, match="not open"):
        UnitreeG1Speaker("head").play(_tone(0.1))
