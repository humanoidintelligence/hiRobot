"""PollingCameraDriver and how RobotAssembly / make_robot attach cameras (fake cameras).

The drivers themselves are tested under ``tests/drivers/camera/``.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from hirobot import (
    Action,
    Camera,
    ControlMode,
    Embodiment,
    EndEffector,
    RobotAssembly,
    load_embodiment,
    make_robot,
)
from hirobot.camera import PollingCameraDriver, available_cameras, get_camera
from hirobot.drivers.camera import UnitreeVideohubCamera
from hirobot.drivers.sim.toy import ToyRobot

G1 = "unitree_g1"


class _CountingCamera(PollingCameraDriver):
    """Returns a 4x6 frame whose pixels hold the grab count; fails on request."""

    def __init__(self, name, fail_first=0, never=False):
        super().__init__(name, poll_hz=200.0, retry_s=0.01)
        self.n = 0
        self.fail_first = fail_first
        self.never = never
        self.opened = self.closed = False

    def _open(self):
        self.opened = True

    def _grab(self):
        if self.never:
            raise RuntimeError("no signal")
        self.n += 1
        if self.n <= self.fail_first:
            raise RuntimeError("hiccup")
        return np.full((4, 6, 3), self.n % 256, dtype=np.uint8)

    def _close(self):
        self.closed = True


def _assembly(**cams):
    emb = load_embodiment(G1)
    return RobotAssembly(ToyRobot(emb), cams, first_frame_timeout_s=1.0)


def test_registry_has_videohub():
    assert "unitree_videohub" in available_cameras()
    assert get_camera("unitree_videohub") is UnitreeVideohubCamera
    with pytest.raises(KeyError, match="Unknown camera driver"):
        get_camera("nope")


def test_polling_source_keeps_newest_frame_and_recovers_from_errors():
    cam = _CountingCamera("head", fail_first=3)
    assert cam.latest() is None
    t0 = time.monotonic()
    cam.start()
    image, stamp = cam.wait_for_frame(1.0)
    assert image.shape == (4, 6, 3) and image.dtype == np.uint8
    assert t0 <= stamp.t <= time.monotonic() and stamp.seq >= 1
    assert cam.errors == 3 and isinstance(cam.error, RuntimeError)
    time.sleep(0.05)
    newer, newer_stamp = cam.latest()
    assert newer[0, 0, 0] > image[0, 0, 0]
    assert newer_stamp.t >= stamp.t and newer_stamp.seq > stamp.seq
    cam.stop()
    assert cam.closed
    cam.stop()  # idempotent


def test_wait_for_frame_times_out_with_last_error():
    cam = _CountingCamera("head", never=True)
    cam.start()
    with pytest.raises(TimeoutError, match="no signal"):
        cam.wait_for_frame(0.1)
    cam.stop()


def test_assembly_adds_images_and_timestamps():
    robot = _assembly(head=_CountingCamera("head"))
    robot.connect()
    try:
        obs = robot.get_observation()
        assert obs.joint_pos is not None  # body observation intact
        assert set(obs.images) == {"head"} and set(obs.stamps) == {"body", "head"}
        assert obs.images["head"].shape == (4, 6, 3)
        # The toy body is simulated, so the frame is restamped on its clock.
        assert obs.stamps["head"].t == robot.now() and obs.stamps["head"].seq >= 1
    finally:
        robot.disconnect()
    assert robot.cameras["head"].closed


class _RgbdCamera(_CountingCamera):
    """Adds a depth image whose value is the grab count, sharing the frame's stamp."""

    def latest_rgbd(self):
        frame = self.latest()
        if frame is None:
            return None
        image, stamp = frame
        return image, np.full(image.shape[:2], float(image[0, 0, 0]), np.float32), stamp


def test_base_latest_rgbd_has_no_depth():
    cam = _CountingCamera("head")
    assert cam.latest_rgbd() is None
    cam.start()
    try:
        image, _ = cam.wait_for_frame(1.0)
        rgb, depth, _ = cam.latest_rgbd()
        assert depth is None and rgb.shape == image.shape
    finally:
        cam.stop()


def test_assembly_adds_depth_for_rgbd_cameras_only():
    emb = load_embodiment(G1)
    cams = [c.model_copy(update={"depth": c.name == "head"}) for c in emb.cameras]
    cams.append(cams[0].model_copy(update={"name": "wrist", "depth": False}))
    emb = emb.model_copy(update={"cameras": cams})
    robot = RobotAssembly(ToyRobot(emb), {"head": _RgbdCamera("head"),
                                          "wrist": _RgbdCamera("wrist")},
                          first_frame_timeout_s=1.0)
    robot.connect()
    try:
        obs = robot.get_observation()
        assert set(obs.images) == {"head", "wrist"} and set(obs.depth) == {"head"}
        # Depth and colour come from the same frame (same grab count).
        assert obs.depth["head"].shape == (4, 6)
        assert obs.depth["head"][0, 0] == obs.images["head"][0, 0, 0]
    finally:
        robot.disconnect()


def test_assembly_tolerates_camera_without_frames():
    robot = _assembly(head=_CountingCamera("head", never=True))
    robot.connect()  # warns, does not raise
    try:
        obs = robot.get_observation()
        assert "head" not in obs.images and "head" not in obs.stamps
    finally:
        robot.disconnect()


def test_assembly_forwards_actions_through_its_safety_gate():
    robot = make_robot(G1)  # the sim default skips sources, so assemble by hand
    robot = RobotAssembly(robot, {})
    from hirobot import SafetyGate

    robot.set_safety(SafetyGate.from_embodiment(robot.embodiment))
    robot.connect()
    n = robot.embodiment.num_dofs
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(n)))
    with pytest.raises(Exception, match="joint"):
        robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(n + 1)))
    robot.disconnect()


def test_assembly_forwards_driver_extras():
    emb = load_embodiment(G1)
    body = ToyRobot(emb)
    body.custom_extra = "from body"
    robot = RobotAssembly(body, {})
    assert robot.custom_extra == "from body"
    with pytest.raises(AttributeError):
        robot.not_there  # noqa: B018


def test_assembly_stops_cameras_when_one_fails_to_start():
    class _Broken(_CountingCamera):
        def _open(self):
            raise OSError("cannot open")

    good = _CountingCamera("a")
    robot = _assembly(a=good, b=_Broken("b"))
    with pytest.raises(OSError):
        robot.connect()
    assert good.closed


def test_make_robot_skips_sources_for_sim_driver():
    robot = make_robot(G1)
    assert not isinstance(robot, RobotAssembly)
    assert [c.name for c in robot.embodiment.cameras] == ["head"]
    assert robot.embodiment.cameras[0].driver == {"type": "unitree_videohub"}




def test_stamps_count_frames_and_stale_sources_reads_them():
    from hirobot.stamps import Stamp, StampCounter, stale_sources
    from hirobot.types import Observation

    counter = StampCounter(clock=lambda: 5.0)
    assert counter.last is None
    assert counter.tick() == Stamp(5.0, 1) and counter.tick(7.0) == Stamp(7.0, 2)
    emb = load_embodiment(G1).model_copy(update={"control_hz": 50})
    obs = Observation(stamps={"body": Stamp(10.0, 1), "head": Stamp(9.0, 1)})
    # 3 control periods = 0.06 s for the body; head declares no fps: never stale.
    assert stale_sources(obs, 10.05, embodiment=emb) == []
    assert stale_sources(obs, 10.1, embodiment=emb) == ["body"]
    assert stale_sources(obs, 10.1, max_age={"head": 0.5}, embodiment=emb) == ["body", "head"]
    assert stale_sources(obs, 9.5, max_age=0.2) == ["head"]


def test_end_effector_and_camera_names_must_differ():
    with pytest.raises(ValueError, match="names must differ"):
        Embodiment(name="x", end_effectors=[EndEffector(name="head")],
                   cameras=[Camera(name="head", width=1, height=1)])
    with pytest.raises(ValueError, match="names must differ"):
        Embodiment(name="x", cameras=[Camera(name="body", width=1, height=1)])
