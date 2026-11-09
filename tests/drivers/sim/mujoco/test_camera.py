"""``mujoco`` renders embodiment cameras that have a ``mount`` (needs the ``sim`` extra).

Rendering needs an OpenGL context (``MUJOCO_GL``); the tests that render skip without one.
"""

import numpy as np
import pytest

from hirobot import Embodiment, get_body

mujoco = pytest.importorskip("mujoco")

# One hinge about z carries the camera link. In front of it (world +x): a red
# wall above the view axis, a blue wall below; a green wall at world +y.
SCENE_XML = """
<mujoco>
  <worldbody>
    <light pos="0 0 3" directional="true"/>
    <geom type="box" size="0.1 3 0.5" pos="3 0 1.6" rgba="1 0 0 1" contype="0" conaffinity="0"/>
    <geom type="box" size="0.1 3 0.5" pos="3 0 0.4" rgba="0 0 1 1" contype="0" conaffinity="0"/>
    <geom type="box" size="3 0.1 1.5" pos="0 3 1" rgba="0 1 0 1" contype="0" conaffinity="0"/>
    <body name="head" pos="0 0 1">
      <joint name="yaw" axis="0 0 1" armature="0.1"/>
      <geom size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="yaw"/>
  </actuator>
</mujoco>
"""

# Optical frame (z forward, x right, y down) looking along the link's +x, z up:
# columns are the optical axes in the link frame -> quat [x, y, z, w].
FORWARD = [-0.5, 0.5, -0.5, 0.5]


def _robot(tmp_path, cameras, **kw):
    path = tmp_path / "scene.xml"
    path.write_text(SCENE_XML)
    emb = Embodiment(name="cam", kind="arm", floating_base=False, control_hz=50,
                     joints=[{"name": "yaw"}], cameras=cameras)
    return get_body("mujoco")(emb, model_path=str(path), realtime=False, ramp_seconds=0.0,
                              default_kp=50.0, default_kd=5.0, **kw)


def _connect(robot):
    robot.connect()
    if not robot._renderers:  # no GL context on this machine (MUJOCO_GL)
        robot.disconnect()
        pytest.skip("offscreen rendering unavailable")
    return robot.get_observation()


def _dominant(pixel):
    """The colour channel that clearly dominates, or "-" (background, mixed)."""
    pixel = np.asarray(pixel, dtype=int)
    top = int(np.argmax(pixel))
    return "rgb"[top] if pixel[top] > 60 and pixel[top] > 2 * np.delete(pixel, top).max() else "-"


def test_mounted_camera_is_upright_and_full_size(tmp_path):
    robot = _robot(tmp_path, [{"name": "head", "width": 160, "height": 120,
                               "mount": {"body": "head", "quat": FORWARD, "fovy": 60}}])
    obs = _connect(robot)
    img = obs.images["head"]
    assert img.shape == (120, 160, 3) and img.dtype == np.uint8
    # Row 0 is the top of the picture: red above, blue below, no flip needed.
    # Rows 40 / 80 look 0.58 m above / below the axis at the walls' 3 m.
    assert _dominant(img[40, 80]) == "r"
    assert _dominant(img[80, 80]) == "b"
    # A rendered frame shares the body's stamp: the same simulated instant.
    assert obs.stamps["head"] == obs.stamps["body"] and obs.stamps["body"].t == robot.now()
    robot.disconnect()


def test_camera_follows_its_link(tmp_path):
    robot = _robot(tmp_path, [{"name": "head", "width": 64, "height": 48,
                               "mount": {"body": "head", "quat": FORWARD, "fovy": 60}}])
    obs = _connect(robot)
    assert _dominant(obs.images["head"][16, 32]) == "r"
    from hirobot import Action

    for _ in range(100):  # turn the head a quarter turn to world +y
        robot.send_action(Action(joint_target=np.array([np.pi / 2])))
    obs = robot.get_observation()
    assert abs(obs.joint_pos[0] - np.pi / 2) < 0.05
    assert _dominant(obs.images["head"][24, 32]) == "g"
    robot.disconnect()


def test_camera_without_mount_is_not_rendered(tmp_path):
    robot = _robot(tmp_path, [{"name": "real_only", "width": 64, "height": 48}])
    robot.connect()
    assert robot.model.ncam == 0
    assert robot.get_observation().images == {}
    robot.disconnect()


def test_mount_on_missing_body_reports_it(tmp_path):
    robot = _robot(tmp_path, [{"name": "head", "width": 64, "height": 48,
                               "mount": {"body": "neck"}}])
    with pytest.raises(ValueError, match="mount.body 'neck'"):
        robot.connect()


def test_offscreen_buffer_grows_to_the_largest_camera(tmp_path):
    robot = _robot(tmp_path, [{"name": "big", "width": 1280, "height": 720,
                               "mount": {"body": "head", "quat": FORWARD}}])
    obs = _connect(robot)
    assert obs.images["big"].shape == (720, 1280, 3)
    assert robot.model.vis.global_.offwidth >= 1280
    robot.disconnect()


def test_no_gl_context_runs_without_images(tmp_path, monkeypatch, caplog):
    from hirobot.drivers.sim.mujoco import driver

    calls = []

    def no_context(*args, **kwargs):
        calls.append(1)
        raise mujoco.FatalError("an OpenGL platform library has not been loaded")

    monkeypatch.setattr(mujoco, "Renderer", no_context)
    monkeypatch.setattr(driver, "_GL_ERROR", None)
    robot = _robot(tmp_path, [{"name": "head", "width": 64, "height": 48,
                               "mount": {"body": "head", "quat": FORWARD}}])
    robot.connect()
    obs = robot.get_observation()
    assert obs.images == {} and obs.joint_pos.shape == (1,)
    assert "cannot render cameras ['head']" in caplog.text
    robot.disconnect()
    # A second robot does not try again (a retry can abort the process).
    robot.connect()
    assert calls == [1] and robot.get_observation().images == {}
    robot.disconnect()
