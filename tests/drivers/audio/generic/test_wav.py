"""WAV helpers and the stand-in audio devices: ``wav_mic``, ``wav_speaker``, ``null_speaker``."""

from __future__ import annotations

import threading
import time
import wave

import numpy as np
import pytest

from hirobot.drivers.audio import (
    NullSpeaker,
    WavMicrophone,
    WavSpeaker,
    read_wav,
    wav_bytes,
)

RATE = 16000


def _tone(seconds: float, rate: int = RATE, amp: int = 8000) -> bytes:
    t = np.arange(int(seconds * rate)) / rate
    return (amp * np.sin(2 * np.pi * 440 * t)).astype("<i2").tobytes()


def _write_wav(path, pcm: bytes, rate: int = RATE) -> str:
    path.write_bytes(wav_bytes(pcm, rate))
    return str(path)


def _drain(mic, timeout: float = 2.0) -> bytes:
    out = []
    while True:
        chunk = mic.read(timeout=timeout)
        if chunk is None:
            return b"".join(out)
        out.append(chunk.pcm)


def test_wav_round_trip(tmp_path):
    pcm = _tone(0.1)
    pcm_back, rate = read_wav(_write_wav(tmp_path / "a.wav", pcm))
    assert pcm_back == pcm and rate == RATE


def test_read_wav_keeps_first_channel(tmp_path):
    left = np.arange(100, dtype="<i2")
    stereo = np.stack([left, -left], axis=1).astype("<i2").tobytes()
    path = tmp_path / "s.wav"
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(8000)
        w.writeframes(stereo)
    pcm, rate = read_wav(str(path))
    assert rate == 8000 and pcm == left.tobytes()


def test_wav_microphone_delivers_file_then_ends(tmp_path):
    pcm = _tone(0.5)
    mic = WavMicrophone("m", _write_wav(tmp_path / "a.wav", pcm), realtime=False, tail_silence=0.25)
    assert mic.sample_rate == RATE and mic.simulated
    mic.open()
    try:
        got = _drain(mic)
    finally:
        mic.close()
    assert got[: len(pcm)] == pcm
    assert got[len(pcm):] == b"\x00" * (2 * int(0.25 * RATE))
    assert mic.read(timeout=0.05) is None
    assert mic.dropped == 0


def test_wav_microphone_reports_the_end_of_the_file(tmp_path):
    mic = WavMicrophone("m", _write_wav(tmp_path / "in.wav", _tone(0.1)),
                        realtime=False, tail_silence=0.0)
    assert not mic.ended
    mic.open()
    try:
        chunks = 0
        while mic.read(timeout=1.0) is not None:
            chunks += 1
        assert chunks == 5 and mic.ended  # 100 ms in 20 ms chunks, then the end
    finally:
        mic.close()


def test_wav_microphone_chunks_are_chunk_ms_long(tmp_path):
    mic = WavMicrophone("m", _write_wav(tmp_path / "a.wav", _tone(0.2)), chunk_ms=20,
                        realtime=False, tail_silence=0.0)
    mic.open()
    try:
        chunk = mic.read(timeout=1.0)
    finally:
        mic.close()
    assert chunk.sample_rate == RATE and chunk.duration == pytest.approx(0.02)


def test_wav_microphone_realtime_takes_as_long_as_the_audio(tmp_path):
    mic = WavMicrophone("m", _write_wav(tmp_path / "a.wav", _tone(0.3)), realtime=True,
                        tail_silence=0.0)
    mic.open()
    start = time.monotonic()
    try:
        _drain(mic)
    finally:
        mic.close()
    assert time.monotonic() - start >= 0.25


def test_wav_speaker_records_what_was_played(tmp_path):
    out = tmp_path / "out.wav"
    spk = WavSpeaker("s", str(out), realtime=False)
    spk.open()
    a, b = _tone(0.1), _tone(0.2, amp=3000)
    assert spk.play(a) == pytest.approx(0.1)
    assert spk.play(b) == pytest.approx(0.2)
    spk.close()
    pcm, rate = read_wav(str(out))
    assert rate == RATE and pcm == a + b


def test_wav_speaker_resamples_to_its_rate(tmp_path):
    out = tmp_path / "out.wav"
    spk = WavSpeaker("s", str(out), realtime=False, sample_rate=RATE)
    played = spk.play(_tone(0.1, rate=24000), sample_rate=24000)
    spk.close()
    pcm, rate = read_wav(str(out))
    assert rate == RATE and played == pytest.approx(0.1, abs=1e-3)
    assert len(pcm) == 2 * int(0.1 * RATE)


def test_wav_speaker_stop_cuts_playback(tmp_path):
    out = tmp_path / "out.wav"
    spk = WavSpeaker("s", str(out), realtime=True)
    threading.Timer(0.15, spk.stop).start()
    played = spk.play(_tone(1.0))
    spk.close()
    assert 0.1 <= played < 0.6
    pcm, _ = read_wav(str(out))
    assert len(pcm) == pytest.approx(2 * played * RATE, abs=4)


def test_wav_speaker_writes_nothing_when_silent(tmp_path):
    out = tmp_path / "out.wav"
    WavSpeaker("s", str(out)).close()
    assert not out.exists()


def test_null_speaker_times_without_output():
    spk = NullSpeaker("n", realtime=False)
    assert spk.play(_tone(0.5)) == pytest.approx(0.5)
    spk = NullSpeaker("n", realtime=True)
    threading.Timer(0.1, spk.stop).start()
    assert spk.play(_tone(2.0)) < 0.5
