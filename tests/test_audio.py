"""Audio core: resampling, QueuedMicrophone, and how make_robot / RobotAssembly wire devices.

The drivers themselves are tested under ``tests/drivers/audio/``.
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from hirobot import RobotAssembly, load_embodiment, make_robot
from hirobot.audio import (
    SAMPLE_WIDTH,
    MicrophoneDriver,
    QueuedMicrophone,
    SpeakerDriver,
    available_audio,
    get_audio,
    resample,
)
from hirobot.drivers.audio import UnitreeG1Microphone, UnitreeG1Speaker
from hirobot.drivers.sim.toy import ToyRobot

G1 = "unitree_g1"
RATE = 16000


def _tone(seconds, rate=RATE):
    t = np.arange(int(seconds * rate)) / rate
    return (8000 * np.sin(2 * np.pi * 440 * t)).astype("<i2").tobytes()


# --------------------------------------------------------------- registry


def test_registry_has_g1_devices():
    assert {"unitree_voice_mic", "unitree_voice_speaker"} <= set(available_audio())
    assert get_audio("unitree_voice_mic") is UnitreeG1Microphone
    assert get_audio("unitree_voice_speaker") is UnitreeG1Speaker
    with pytest.raises(KeyError, match="Unknown audio driver"):
        get_audio("nope")


def test_resample_keeps_duration():
    pcm = _tone(1.0, 24000)
    out = resample(pcm, 24000, RATE)
    assert len(out) == RATE * SAMPLE_WIDTH
    assert resample(pcm, RATE, RATE) is pcm
    assert resample(b"", 24000, RATE) == b""


# ------------------------------------------------------------ microphone


class _FakeMic(QueuedMicrophone):
    def __init__(self, name, blocks, queue_s=10.0):
        super().__init__(name, queue_s=queue_s, retry_s=0.01)
        self.blocks = list(blocks)
        self.closed = False

    def _receive(self):
        if self.blocks:
            item = self.blocks.pop(0)
            if isinstance(item, Exception):
                raise item
            return item
        time.sleep(0.01)
        return None

    def _close(self):
        self.closed = True


def test_queued_mic_reads_in_order_and_survives_errors():
    mic = _FakeMic("m", [b"\x01\x00" * 4, RuntimeError("hiccup"), b"\x02\x00" * 4])
    mic.open()
    try:
        a, b = mic.read(1.0), mic.read(1.0)
        assert a.pcm == b"\x01\x00" * 4 and b.pcm == b"\x02\x00" * 4
        assert a.sample_rate == RATE and a.stamp <= b.stamp
        assert isinstance(mic.error, RuntimeError)
        assert mic.read(0.05) is None  # timeout
    finally:
        mic.close()
    assert mic.closed
    assert mic.read(0.05) is None  # closed


def test_queued_mic_drops_oldest_when_full():
    block = b"\x00\x00" * 1600  # 0.1 s
    mic = _FakeMic("m", [block] * 10, queue_s=0.35)
    mic.open()
    try:
        deadline = time.time() + 1.0
        while mic.chunks < 10 and time.time() < deadline:
            time.sleep(0.01)
        assert mic.dropped == 7  # 0.35 s holds three 0.1 s blocks
    finally:
        mic.close()


def test_read_wakes_up_on_close():
    mic = _FakeMic("m", [])
    mic.open()
    out = []
    t = threading.Thread(target=lambda: out.append(mic.read()))
    t.start()
    time.sleep(0.05)
    mic.close()
    t.join(1.0)
    assert out == [None] and not t.is_alive()


# --------------------------------------------------------------- speaker


# ------------------------------------------------------- config and assembly


class _Mic(MicrophoneDriver):
    def __init__(self, name):
        super().__init__(name)
        self.events = []

    def open(self):
        self.events.append("open")

    def read(self, timeout=None):
        return None

    def close(self):
        self.events.append("close")


class _Spk(SpeakerDriver):
    def __init__(self, name, fail=False):
        super().__init__(name)
        self.fail = fail
        self.events = []

    def open(self):
        if self.fail:
            raise OSError("no speaker")
        self.events.append("open")

    def play(self, pcm, sample_rate=None):
        return 0.0

    def stop(self):
        pass

    def close(self):
        self.events.append("close")


def test_config_declares_g1_audio():
    emb = load_embodiment(G1)
    assert [m.name for m in emb.microphones] == ["head"]
    assert emb.microphones[0].driver == {"type": "unitree_voice_mic"}
    assert emb.speakers[0].driver == {"type": "unitree_voice_speaker", "volume": 60}


def test_embodiment_rejects_duplicate_audio_names():
    from hirobot import Embodiment

    with pytest.raises(ValueError, match="duplicate speaker"):
        Embodiment(name="x", speakers=[{"name": "a"}, {"name": "a"}])


def test_make_robot_skips_audio_for_sim_driver():
    robot = make_robot(G1)
    assert not isinstance(robot, RobotAssembly)


def test_make_robot_attaches_audio_for_real_driver():
    robot = make_robot(G1, body_overrides={"driver": "unitree_sdk2", "network_interface": "eth9"})
    assert isinstance(robot, RobotAssembly)
    mic, spk = robot.microphones["head"], robot.speakers["head"]
    assert isinstance(mic, UnitreeG1Microphone) and isinstance(spk, UnitreeG1Speaker)
    assert mic._network_interface == "eth9" and spk._network_interface == "eth9"
    assert spk._volume == 60


def test_make_robot_rejects_a_speaker_driver_as_microphone(tmp_path):
    cfg = tmp_path / "bad.yaml"
    cfg.write_text(
        "embodiment:\n  name: x\n  joints: [{name: a}]\n"
        "  microphones: [{name: m, driver: {type: unitree_voice_speaker}}]\n"
        "body: {driver: unitree_sdk2}\n"
    )
    with pytest.raises(ValueError, match="not a MicrophoneDriver"):
        make_robot(str(cfg))


def test_assembly_opens_and_closes_audio():
    mic, spk = _Mic("m"), _Spk("s")
    robot = RobotAssembly(ToyRobot(load_embodiment(G1)), microphones={"m": mic},
                          speakers={"s": spk})
    robot.connect()
    assert mic.events == ["open"] and spk.events == ["open"]
    assert robot.get_observation().joint_pos is not None
    robot.disconnect()
    assert mic.events == ["open", "close"] and spk.events == ["open", "close"]


def test_assembly_closes_audio_when_a_device_fails_to_open():
    mic = _Mic("m")
    robot = RobotAssembly(ToyRobot(load_embodiment(G1)), microphones={"m": mic},
                          speakers={"s": _Spk("s", fail=True)})
    with pytest.raises(OSError, match="no speaker"):
        robot.connect()
    assert mic.events == ["open", "close"]
