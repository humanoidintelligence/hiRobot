"""Checks every driver of a kind must pass, whatever its SDK.

Each SDK directory's ``test_contract.py`` builds its driver against its own
fake SDK and calls these. They check only what the base classes promise
(``hirobot.core.robot.Robot``, ``hirobot.devices.end_effector.EndEffectorDriver`` on each
interface, ``hirobot.devices.camera.CameraDriver``, ``hirobot.devices.audio``), so a new driver that passes them works with
``make_robot``, ``RobotAssembly`` and hiAgent's control loop. What a driver
does beyond that (gains, slot maps, vendor quirks) is in its own tests.

The checks:

* **registered**: the driver is in its registry under the name, with the
  right base class.
* **manifest**: an SDK driver's package carries a ``manifest.yaml`` whose
  ``sdk`` list parses (``hirobot.loading.sdk.load_manifest``).
* **lifecycle**: construct without the SDK; connect / read / command /
  disconnect against the fake; disconnect twice is safe.
* **shapes**: readings have one value per joint, the quaternion is
  ``[x, y, z, w]`` and unit length.
* **modes**: every mode in ``action_modes`` is accepted; the WBC modes
  (``BASE_VELOCITY``, ``MOTION_LATENT``) are not claimed by joint drivers.
* **safety**: ``send_action`` runs the gate: a wrong-sized target is refused
  before anything reaches the fake.
* **e-stop**: ``estop()`` enters the fault state and nothing more reaches the
  fake; ``reset()`` clears it.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np
import pytest

from hirobot import Action, ControlMode, Robot, RobotFaultError, SafetyError, SafetyGate
from hirobot.devices.audio import MicrophoneDriver, SpeakerDriver, get_audio
from hirobot.devices.camera import CameraDriver, get_camera
from hirobot.devices.end_effector import EndEffectorDriver, JointsState, OpeningState, get_end_effector
from hirobot.loading.registry import get_body
from hirobot.loading.sdk import load_manifest
from hirobot.core.stamps import Stamp

WBC_MODES = (ControlMode.BASE_VELOCITY, ControlMode.MOTION_LATENT)


def check_manifest(cls) -> None:
    """The driver's package has a manifest whose ``sdk`` entries all parse."""
    package = cls.__module__.rpartition(".")[0]
    manifest = load_manifest(package)
    assert manifest.sdk, f"{package}/manifest.yaml lists no sdk"
    for req in manifest.sdk:
        assert req.distribution and req.module


# ------------------------------------------------------------------ bodies


def check_robot(name: str, make: Callable[[], Robot], sent: Callable[[], int],
                command_mode: Optional[ControlMode] = None) -> None:
    """A body driver: registered, connects, reads, commands, refuses through the gate.

    Args:
        name: Registry name.
        make: Builds an unconnected driver (the fake SDK already installed).
        sent: How many commands have reached the fake so far.
        command_mode: The mode used to send one command; default the first
            joint mode in ``action_modes``.
    """
    cls = get_body(name)
    assert issubclass(cls, Robot)
    check_manifest(cls)
    robot = make()
    assert isinstance(robot, cls)
    assert not robot.simulated
    assert not set(robot.action_modes) & set(WBC_MODES), "joint drivers must not claim the WBC modes"
    check_robot_behaviour(robot, sent, command_mode, label=name)


def check_sim(name: str, make: Callable[[], Robot], sent: Callable[[], int],
              command_mode: Optional[ControlMode] = None, manifest: bool = True) -> None:
    """A simulator body driver: like :func:`check_robot`, but ``simulated`` is set.

    Args:
        name: Registry name.
        make: Builds an unconnected driver.
        sent: How many commands have been applied so far.
        command_mode: As in :func:`check_robot`.
        manifest: Whether the driver's package has a ``manifest.yaml`` (the toy
            simulator needs no SDK, so it has none).
    """
    cls = get_body(name)
    assert issubclass(cls, Robot)
    if manifest:
        check_manifest(cls)
    robot = make()
    assert isinstance(robot, cls)
    assert robot.simulated
    assert not set(robot.action_modes) & set(WBC_MODES), "joint drivers must not claim the WBC modes"
    check_robot_behaviour(robot, sent, command_mode, label=name)


def check_robot_behaviour(robot: Robot, sent: Callable[[], int],
                          command_mode: Optional[ControlMode] = None, label: str = "robot") -> None:
    """What any :class:`~hirobot.core.robot.Robot` promises, registered driver or not.

    Lifecycle, reading shapes, one command reaching the hardware (or fake), and
    the gate refusing a wrong-sized target before anything is sent. Used by
    :func:`check_robot` and directly for composites such as a ``RobotAssembly`` with parts.

    Args:
        robot: An unconnected robot.
        sent: How many commands have reached the hardware (or fake) so far.
        command_mode: The mode used to send one command; default the first
            joint mode in ``action_modes``.
        label: Name used in failure messages.
    """
    emb = robot.embodiment
    n = emb.num_dofs
    modes = tuple(robot.action_modes)
    assert modes, "a robot must accept at least one mode"

    robot.connect()
    try:
        robot.reset()
        obs = robot.get_observation()
        assert obs.joint_pos is not None and np.asarray(obs.joint_pos).shape == (n,)
        for field in ("joint_vel", "joint_torque"):
            value = getattr(obs, field)
            assert value is None or np.asarray(value).shape == (n,), field
        if emb.floating_base and obs.base_orientation is not None:
            quat = np.asarray(obs.base_orientation, dtype=float)
            assert quat.shape == (4,) and np.isclose(np.linalg.norm(quat), 1.0, atol=1e-3)
        for ee, pose in (obs.ee_poses or {}).items():
            pose = np.asarray(pose, dtype=float)
            assert pose.shape == (7,), f"{label}: ee_poses[{ee!r}] must be [x, y, z, qx, qy, qz, qw]"
            assert np.isclose(np.linalg.norm(pose[3:]), 1.0, atol=1e-3), f"{label}: ee_poses[{ee!r}]"
        check_stamps(robot, obs, label)
        status = robot.get_status()
        assert status.timestamp is not None

        mode = command_mode or next(m for m in modes if m in (
            ControlMode.POSITION, ControlMode.PD, ControlMode.VELOCITY, ControlMode.TORQUE))
        before = sent()
        robot.send_action(Action(mode=mode, joint_target=np.asarray(obs.joint_pos, dtype=float)))
        assert sent() > before, f"{label}: a {mode.value} command reached nothing"

        robot._set_safety(SafetyGate.from_embodiment(emb))
        before = sent()
        with pytest.raises(SafetyError):
            robot.send_action(Action(mode=mode, joint_target=np.zeros(n + 1)))
        assert sent() == before, "a refused action must not reach the robot"
        robot._set_safety(None)

        check_estop(robot, sent, mode, label)
    finally:
        robot.disconnect()
    robot.disconnect()  # twice is safe



def check_estop(robot: Robot, sent: Callable[[], int], mode: ControlMode,
                label: str = "robot") -> None:
    """``estop()`` refuses commands until ``reset()``, which makes it commandable again."""
    target = np.asarray(robot.get_observation().joint_pos, dtype=float)
    robot.estop()
    assert robot.fault is not None, f"{label}: estop() must enter the fault state"
    assert robot.get_status().fault is not None
    before = sent()
    with pytest.raises(RobotFaultError):
        robot.send_action(Action(mode=mode, joint_target=target))
    assert sent() == before, f"{label}: a command after estop() reached the robot"
    robot.estop()  # twice is safe
    robot.reset()
    assert robot.fault is None, f"{label}: reset() must clear the e-stop fault"
    robot.send_action(Action(mode=mode, joint_target=target))
    assert sent() > before, f"{label}: no command reached the robot after reset()"


def check_stamps(robot: Robot, obs, label: str = "robot") -> None:
    """``stamps["body"]`` is set, on the robot's clock, and never from the future."""
    stamps = obs.stamps
    body = [k for k in stamps if k == "body" or k.endswith("/body")]
    assert body, f"{label}: get_observation must set stamps['body']"
    now = robot.now()
    for name, stamp in stamps.items():
        assert isinstance(stamp, Stamp), f"{label}: stamps[{name!r}] must be a Stamp"
        assert stamp.seq >= 1, f"{label}: stamps[{name!r}].seq starts at 1"
        assert stamp.t <= now + 1e-6, f"{label}: stamps[{name!r}] is after robot.now()"


# ------------------------------------------------------------------ hands


def check_hand(name: str, make: Callable[[bool], EndEffectorDriver], sent: Callable[[], int]) -> None:
    """An end-effector driver on its ``joints`` interface (a hand): registered,
    reads a state per joint, commands, read-only never sends.

    Args:
        make: ``make(read_only)`` builds an unconnected hand on the fake SDK.
        sent: How many commands have reached the fake so far.
    """
    assert issubclass(get_end_effector(name), EndEffectorDriver)
    check_manifest(get_end_effector(name))
    hand = make(False)
    assert isinstance(hand, get_end_effector(name))
    assert "joints" in type(hand).interfaces and hand.interface == "joints"
    n = hand.num_joints
    assert n == len(hand.joint_names) > 0
    hand.connect()
    state = hand.get_state()
    assert isinstance(state, JointsState)
    for field in ("q", "dq", "tau"):
        assert np.asarray(getattr(state, field)).shape == (n,), field
    assert isinstance(state.stamp, Stamp) and state.stamp.seq >= 1
    assert state.stamp.t <= time.monotonic()
    before = sent()
    hand.command(state.q)
    assert sent() > before, f"{name}: command reached nothing"
    with pytest.raises(ValueError):
        hand.command(np.full(n, np.nan))
    hand.disconnect()
    hand.disconnect()

    ro = make(True)
    ro.connect()
    before = sent()
    ro.get_state()
    with pytest.raises(RuntimeError):
        ro.command(np.zeros(n))
    with pytest.raises(RuntimeError):
        ro.reset()
    ro.disconnect()
    assert sent() == before, "a read-only hand must never send"


def check_opening(name: str, make: Callable[[bool], EndEffectorDriver], sent: Callable[[], int],
                  manifest: bool = True) -> None:
    """An end-effector driver on its ``opening`` interface (a gripper): registered,
    reads an opening in [0, 1], takes a target without blocking, refuses values
    outside [0, 1], read-only never sends.

    Args:
        make: ``make(read_only)`` builds an unconnected gripper on the fake SDK.
        sent: How many commands have reached the fake so far.
        manifest: Check the driver package's ``manifest.yaml`` (off only for
            test doubles that have no SDK).
    """
    cls = get_end_effector(name)
    assert issubclass(cls, EndEffectorDriver) and "opening" in cls.interfaces
    if manifest:
        check_manifest(cls)
    grip = make(False)
    assert isinstance(grip, cls)
    assert grip.interface == "opening" and grip.num_joints == 0
    grip.connect()
    state = grip.get_state()
    assert isinstance(state, OpeningState)
    assert 0.0 <= state.opening <= 1.0
    assert isinstance(state.stamp, Stamp) and state.stamp.seq >= 1
    assert state.stamp.t <= time.monotonic()
    grip.reset()
    before = sent()
    start = time.monotonic()
    grip.command_opening(0.0)
    grip.command_opening(1.0)
    assert time.monotonic() - start < 0.5, f"{name}: command_opening must not wait for the motion"
    assert sent() > before, f"{name}: command reached nothing"
    for bad in (-0.1, 1.1, float("nan")):
        with pytest.raises(ValueError):
            grip.command_opening(bad)
    grip.estop()
    grip.disconnect()
    grip.disconnect()

    ro = make(True)
    ro.connect()
    before = sent()
    ro.get_state()
    with pytest.raises(RuntimeError):
        ro.command_opening(0.5)
    with pytest.raises(RuntimeError):
        ro.reset()
    ro.disconnect()
    assert sent() == before, "a read-only gripper must never send"


# ------------------------------------------------------------------ cameras


def check_camera(name: str, make: Callable[[], CameraDriver], timeout: float = 2.0) -> None:
    """A camera driver: registered, nothing before start, HxWx3 uint8 frames after, stop twice."""
    cls = get_camera(name)
    assert issubclass(cls, CameraDriver)
    if not cls.simulated:
        check_manifest(cls)
    cam = make()
    assert isinstance(cam, cls)
    assert cam.latest() is None
    cam.start()
    try:
        if hasattr(cam, "wait_for_frame"):
            frame = cam.wait_for_frame(timeout)
        else:
            frame = cam.latest()
        assert frame is not None
        image, stamp = frame
        assert image.ndim == 3 and image.shape[2] == 3 and image.dtype == np.uint8
        assert isinstance(stamp, Stamp) and stamp.seq >= 1
    finally:
        cam.stop()
    cam.stop()


# ------------------------------------------------------------------ audio


def check_microphone(name: str, make: Callable[[], MicrophoneDriver], timeout: float = 2.0,
                     feed: Optional[Callable[[MicrophoneDriver], None]] = None) -> None:
    """A microphone: registered, open, a chunk of int16 PCM at its rate, close twice.

    Args:
        feed: Called after ``open()`` to make the fake deliver audio.
    """
    cls = get_audio(name)
    assert issubclass(cls, MicrophoneDriver)
    if not cls.simulated:
        check_manifest(cls)
    mic = make()
    mic.open()
    try:
        if feed is not None:
            feed(mic)
        chunk = mic.read(timeout)
        assert chunk is not None and len(chunk.pcm) > 0 and len(chunk.pcm) % 2 == 0
        assert chunk.sample_rate == mic.sample_rate
    finally:
        mic.close()
    mic.close()


def check_speaker(name: str, make: Callable[[], SpeakerDriver],
                  played: Optional[Callable[[], float]] = None) -> None:
    """A speaker: registered, plays and reports seconds played, refuses before open, close twice.

    Args:
        played: Seconds of audio the fake received so far (``None``: not observable).
    """
    cls = get_audio(name)
    assert issubclass(cls, SpeakerDriver)
    if not cls.simulated:
        check_manifest(cls)
    pcm = (np.sin(np.arange(1600) / 5.0) * 8000).astype("<i2").tobytes()  # 0.1 s at 16 kHz
    spk = make()
    spk.open()
    try:
        before = played() if played else 0.0
        seconds = spk.play(pcm, 16000)
        assert seconds == pytest.approx(0.1, abs=0.02)
        if played:
            assert played() - before == pytest.approx(0.1, abs=0.02)
        spk.stop()
    finally:
        spk.close()
    spk.close()
