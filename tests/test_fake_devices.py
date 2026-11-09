"""make_robot with stand-in devices on simulated drivers, and Robot.reset(seed) / hold().

The stand-in drivers themselves are tested under ``tests/drivers/{cameras,audio}/generic/``.
"""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import ControlMode, Observation, Robot, RobotAssembly, make_robot
from hirobot.drivers.audio import (
    UnitreeG1Microphone,
    WavMicrophone,
    WavSpeaker,
    wav_bytes,
)
from hirobot.drivers.camera import ArrayCamera
from hirobot.embodiment import Embodiment, Joint

RATE = 16000


def _tone(seconds: float, rate: int = RATE, amp: int = 8000) -> bytes:
    t = np.arange(int(seconds * rate)) / rate
    return (amp * np.sin(2 * np.pi * 440 * t)).astype("<i2").tobytes()


def _write_wav(path, pcm: bytes, rate: int = RATE) -> str:
    path.write_bytes(wav_bytes(pcm, rate))
    return str(path)


# -- cameras ----------------------------------------------------------------------


def _frames(n: int):
    return [np.full((4, 4, 3), i, dtype=np.uint8) for i in range(n)]


def _make_video(path, n: int = 6, fps: int = 10) -> str:
    av = pytest.importorskip("av")
    container = av.open(str(path), mode="w")
    stream = container.add_stream("mpeg4", rate=fps)
    stream.width, stream.height, stream.pix_fmt = 32, 32, "yuv420p"
    for i in range(n):
        img = np.full((32, 32, 3), min(255, 40 * i), dtype=np.uint8)
        for packet in stream.encode(av.VideoFrame.from_ndarray(img, format="rgb24")):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()
    return str(path)


# -- audio ------------------------------------------------------------------------


def _drain(mic, timeout: float = 2.0) -> bytes:
    out = []
    while True:
        chunk = mic.read(timeout=timeout)
        if chunk is None:
            return b"".join(out)
        out.append(chunk.pcm)


# -- make_robot with stand-in devices --------------------------------------------------


def _sim_config(tmp_path, extra: str) -> str:
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(
        "embodiment:\n  name: toy\n  control_hz: 50\n  joints: [{name: a}, {name: b}]\n"
        + extra
        + "body: {driver: toy}\n"
    )
    return str(cfg)


def test_make_robot_attaches_simulated_devices_to_a_sim_driver(tmp_path):
    wav = _write_wav(tmp_path / "in.wav", _tone(0.1))
    cfg = _sim_config(tmp_path,
        "  cameras: [{name: head, width: 64, height: 64, driver: {type: array, fps: 10}}]\n"
        f"  microphones: [{{name: m, driver: {{type: wav_mic, path: {wav}, realtime: false}}}}]\n"
        f"  speakers: [{{name: s, driver: {{type: wav_speaker, path: {tmp_path / 'out.wav'}}}}}]\n")
    robot = make_robot(cfg)
    assert isinstance(robot, RobotAssembly)
    assert isinstance(robot.cameras["head"], ArrayCamera)
    assert isinstance(robot.microphones["m"], WavMicrophone)
    assert isinstance(robot.speakers["s"], WavSpeaker)
    robot.connect()
    try:
        assert robot.get_observation().images["head"].shape == (64, 64, 3)
        assert robot.microphones["m"].read(timeout=1.0) is not None
    finally:
        robot.disconnect()


def test_make_robot_skips_real_devices_on_a_sim_driver(tmp_path):
    wav = _write_wav(tmp_path / "in.wav", _tone(0.1))
    cfg = _sim_config(tmp_path,
        "  cameras: [{name: real, width: 64, height: 64, driver: {type: unitree_videohub}},\n"
        "            {name: fake, width: 64, height: 64, driver: {type: array}}]\n"
        "  microphones: [{name: real, driver: {type: unitree_voice_mic}},\n"
        f"                {{name: fake, driver: {{type: wav_mic, path: {wav}}}}}]\n")
    robot = make_robot(cfg)
    assert set(robot.cameras) == {"fake"}
    assert set(robot.microphones) == {"fake"}


def test_make_robot_unitree_g1_mujoco_takes_a_wav_mic(tmp_path):
    from hirobot.config import load_config

    base = load_config("unitree_g1_mujoco")
    wav = _write_wav(tmp_path / "in.wav", _tone(0.1))
    emb = base["embodiment"]
    emb["microphones"] = [{"name": "head", "driver": {"type": "unitree_voice_mic"}},
                          {"name": "laptop", "driver": {"type": "wav_mic", "path": wav}}]
    emb["cameras"] = list(emb.get("cameras") or []) + [
        {"name": "video", "width": 64, "height": 64, "driver": {"type": "array"}}]
    pytest.importorskip("mujoco")
    try:
        robot = make_robot(base)
    except FileNotFoundError as exc:  # the MJCF is fetched on demand
        pytest.skip(f"G1 model not present: {exc}")
    assert set(robot.microphones) == {"laptop"}
    assert not any(isinstance(m, UnitreeG1Microphone) for m in robot.microphones.values())
    assert "video" in robot.cameras


def test_sim_device_does_not_get_the_robot_nic(tmp_path):
    wav = _write_wav(tmp_path / "in.wav", _tone(0.1))
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(
        "embodiment:\n  name: toy\n  joints: [{name: a}]\n"
        f"  microphones: [{{name: m, driver: {{type: wav_mic, path: {wav}}}}}]\n"
        "  cameras: [{name: c, width: 8, height: 8, driver: {type: array}}]\n"
        "body: {driver: unitree_sdk2, network_interface: eth9}\n")
    # A real body passes its NIC to Unitree devices; stand-ins take no such
    # argument, so this would raise TypeError if it were passed to them too.
    robot = make_robot(str(cfg))
    assert set(robot.microphones) == {"m"} and set(robot.cameras) == {"c"}


# -- Robot.reset(seed) / hold() / outcome ----------------------------------------------------


class _PosRobot(Robot):
    def __init__(self, modes=(ControlMode.POSITION,)) -> None:
        self._emb = Embodiment(name="r", joints=[Joint(name="a"), Joint(name="b")])
        self.action_modes = modes
        self.sent = []
        self.seeds = []

    @property
    def embodiment(self):
        return self._emb

    def connect(self):
        pass

    def reset(self, seed=None):
        self.seeds.append(seed)

    def get_observation(self):
        return Observation(joint_pos=np.array([0.1, 0.2]), timestamp=0.0)

    def _send_action(self, action):
        self.sent.append(action)


def test_hold_sends_current_pose():
    robot = _PosRobot()
    robot.hold()
    assert len(robot.sent) == 1
    action = robot.sent[0]
    assert action.mode == ControlMode.POSITION
    np.testing.assert_allclose(action.joint_target, [0.1, 0.2])


def test_hold_is_a_noop_without_position_mode():
    robot = _PosRobot(modes=(ControlMode.CARTESIAN_DELTA,))
    robot.hold()
    assert robot.sent == []


def test_hold_goes_through_the_safety_gate():
    from hirobot import SafetyGate

    robot = _PosRobot()
    robot.set_safety(SafetyGate.from_embodiment(robot.embodiment))
    robot.hold()
    assert len(robot.sent) == 1


def test_outcome_defaults_to_none():
    robot = _PosRobot()
    assert robot.success is None and robot.done is None and robot.reward is None


def test_assembly_forwards_reset_seed_hold_and_outcome():
    body = _PosRobot()
    body.success, body.done, body.reward = True, False, 0.5
    robot = RobotAssembly(body, cameras={"c": ArrayCamera("c")})
    robot.reset(7)
    assert body.seeds == [7]
    robot.hold()
    assert len(body.sent) == 1
    assert (robot.success, robot.done, robot.reward) == (True, False, 0.5)


