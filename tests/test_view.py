"""``hirobot view``: a real robot's pose drawn in MuJoCo, opened read-only.

A fake real driver stands in for the robot, so nothing touches hardware. The
MuJoCo tests need ``mujoco`` and the G1 MJCF (see unitree_g1_mujoco).
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from hirobot import Observation, Robot
from hirobot.loading import registry
from hirobot.sim import view
from hirobot.cli import main
from tests.conftest import mjcf_of


class FakeReal(Robot):
    """A real-robot driver with a read_only mode; records any command."""

    simulated = False
    instances: list = []

    def __init__(self, embodiment, read_only=False, network_interface=None, pose=None):
        self._emb = embodiment
        self.read_only = read_only
        self.pose = np.zeros(embodiment.num_dofs) if pose is None else np.asarray(pose, float)
        self.commands = []
        self.reads = 0
        FakeReal.instances.append(self)

    @property
    def embodiment(self):
        return self._emb

    def connect(self):
        pass

    def disconnect(self):
        pass

    def reset(self, seed=None):
        raise AssertionError("the viewer must not reset the real robot")

    def get_observation(self):
        self.reads += 1
        return Observation(joint_pos=self.pose.copy(), joint_vel=np.zeros_like(self.pose),
                           base_orientation=np.array([0.0, 0, 0, 1.0]))

    def _send_action(self, action):
        self.commands.append(action)


class NoReadOnly(FakeReal):
    def __init__(self, embodiment):
        super().__init__(embodiment)


@pytest.fixture
def fake_real(monkeypatch):
    FakeReal.instances = []
    monkeypatch.setitem(registry.BODIES._entries, "fake_real", FakeReal)
    monkeypatch.setitem(registry.BODIES._entries, "no_read_only", NoReadOnly)
    return FakeReal


def _real_cfg(driver="fake_real", n=3):
    return {"assembly": {"name": "fake", "parts": {"body": {
        "device": "body", "control_hz": 50, "joints": [{"name": f"j{i}"} for i in range(n)],
        "driver": {"type": driver}}}}}


def test_opens_read_only_and_refuses_what_it_cannot(fake_real):
    robot = view.open_read_only(_real_cfg())
    assert FakeReal.instances[-1].read_only is True
    assert robot.embodiment.name == "fake"
    with pytest.raises(ValueError, match="no read_only mode"):
        view.open_read_only(_real_cfg("no_read_only"))
    with pytest.raises(ValueError, match="simulated robot"):
        view.open_read_only(_real_cfg("toy"))
    with pytest.raises(ValueError, match="no model"):
        view.open_model(_real_cfg("toy"))


def test_default_model_is_the_builtin_mujoco_config():
    assert view.default_model("unitree_g1") == "unitree_g1_mujoco"
    assert view.default_model("robots/unitree_g1") == "unitree_g1_mujoco"
    assert view.default_model("ur_ur5e") is None
    assert view.default_model({"assembly": {"parts": {}}}) is None


def test_mirror_matches_joints_by_name_and_keeps_unread_joints():
    class Fake(FakeReal):
        pass

    from hirobot import Embodiment
    real = Fake(Embodiment(name="r", joints=[{"name": n} for n in ("b", "x", "a")], control_hz=50),
                pose=[0.2, 9.0, np.nan])
    shown = []

    class Model(FakeReal):
        def set_state(self, q, qd=None, base_orientation=None):
            shown.append(np.asarray(q).copy())

    model = Model(Embodiment(name="m", joints=[{"name": n} for n in ("a", "b", "c")],
                             control_hz=50), pose=[0.5, 0.0, 0.7])
    mirror = view.PoseMirror(real, model)
    assert mirror.unmatched_real == ["x"] and mirror.unmatched_model == ["c"]
    q = mirror.step()
    np.testing.assert_allclose(q, [0.5, 0.2, 0.7])  # a: NaN kept, b copied, c untouched
    assert real.commands == [] and model.commands == []
    frames = view.run(real, model, hz=1000, seconds=None,
                      running=iter([True, True, False]).__next__, sleep=lambda s: None)
    assert frames == 2 and len(shown) == 3


MODEL = mjcf_of("unitree_g1_mujoco")
needs_model = pytest.mark.skipif(not os.path.isfile(MODEL), reason=f"G1 MJCF not found at {MODEL}")


@needs_model
def test_g1_pose_shows_in_mujoco_and_the_real_robot_gets_nothing(fake_real):
    pytest.importorskip("mujoco")
    cfg = {"assembly": {"name": "unitree-g1", "parts": {"body": {
        "from": "body/unitree/g1", "overrides": {"driver": {"type": "fake_real"}}}}}}
    real = view.open_read_only(cfg)
    j = real.embodiment.joint_names.index("right_elbow")
    FakeReal.instances[-1].pose[j] = 1.2
    model = view.open_model("unitree_g1_mujoco", window=False)
    real.connect()
    model.connect()
    try:
        frames = view.run(real, model, hz=200, seconds=0.05)
        assert frames >= 1
        assert model.get_observation().joint_pos[j] == pytest.approx(1.2, abs=1e-6)
    finally:
        model.disconnect()
    assert FakeReal.instances[-1].commands == []


def test_cli_view_errors_are_messages(fake_real, tmp_path, capsys):
    import yaml
    path = tmp_path / "toy.yaml"
    path.write_text(yaml.safe_dump(_real_cfg("toy")))
    assert main(["view", str(path), "--model", "unitree_g1_mujoco", "--no-window"]) == 2
    assert "simulated robot" in capsys.readouterr().err
    path.write_text(yaml.safe_dump(_real_cfg()))
    assert main(["view", str(path)]) == 2
    assert "pass --model" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        main(["view", str(path), "--model", "x", "--body", "novalue"])
