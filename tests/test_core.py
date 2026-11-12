"""Core pieces that are not a driver: registries, SDK checks, the action types."""

from __future__ import annotations

import sys
import types
import warnings

import numpy as np
import pytest

from hirobot import Action, ActionChunk, ControlMode, Observation, Robot
from hirobot.loading import registry, sdk
from hirobot.devices.audio import available_audio
from hirobot.devices.camera import available_cameras
from hirobot.devices.end_effector import available_end_effectors

# ------------------------------------------------------------------ registries


def test_every_registry_loads_the_builtin_drivers():
    assert "toy" in registry.available_bodies()
    assert "video_file" in available_cameras()
    assert "wav_mic" in available_audio()
    assert available_end_effectors()


def test_registry_refuses_duplicates_and_wrong_base():
    reg = registry.Registry("widget", Robot)

    @reg.register("a")
    class A(Robot):  # noqa: D401 - minimal stand-in, never instantiated
        pass

    with pytest.raises(ValueError, match="already registered"):
        reg.register("a")(A)
    with pytest.raises(ValueError, match="must subclass Robot"):
        reg.register("b")(int)
    with pytest.raises(KeyError, match="Unknown widget 'c'"):
        reg.get("c")


def test_entry_point_plugins_are_loaded(monkeypatch):
    loaded = []

    class EP:
        def __init__(self, name, fail=False):
            self.name, self.value, self.fail = name, f"pkg_{name}.drivers", fail

        def load(self):
            if self.fail:
                raise ImportError("broken plugin")
            loaded.append(self.name)

    monkeypatch.setattr(registry, "_entry_points", lambda: [EP("good"), EP("bad", fail=True)])
    monkeypatch.setattr(registry, "_loaded", False)
    registry.load_drivers()  # the broken plugin is logged, not raised
    assert loaded == ["good"]
    registry.load_drivers()  # once per process
    assert loaded == ["good"]


# ------------------------------------------------------------------- SDK checks


def _manifest(tmp_path, monkeypatch, text, name="fake_driver_pkg"):
    pkg = tmp_path / name
    pkg.mkdir()
    (pkg / "__init__.py").write_text("")
    (pkg / "manifest.yaml").write_text(text)
    monkeypatch.syspath_prepend(str(tmp_path))
    sdk.load_manifest.cache_clear()
    sdk._warned.clear()
    return name


def _installed(monkeypatch, versions):
    import importlib.metadata as md

    def version(dist):
        if dist not in versions:
            raise md.PackageNotFoundError(dist)
        return versions[dist]

    monkeypatch.setattr(sdk.importlib.metadata, "version", version)


MANIFEST = """
sdk:
  - distribution: acme_sdk
    minimum: "1.2"
    install: pip install acme_sdk
verified:
  - model: Acme One
    sdk: {acme_sdk: "1.4.0 (abc123)"}
"""


def test_require_sdk_accepts_a_tested_version(tmp_path, monkeypatch):
    pkg = _manifest(tmp_path, monkeypatch, MANIFEST)
    _installed(monkeypatch, {"acme_sdk": "1.4.0"})
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert sdk.require_sdk(pkg) == {"acme_sdk": "1.4.0"}


def test_require_sdk_refuses_missing_and_old(tmp_path, monkeypatch):
    pkg = _manifest(tmp_path, monkeypatch, MANIFEST)
    _installed(monkeypatch, {})
    with pytest.raises(ImportError, match="AcmeRobot: acme_sdk is not installed. Install it: pip install"):
        sdk.require_sdk(pkg, who="AcmeRobot")
    _installed(monkeypatch, {"acme_sdk": "1.1.9"})
    with pytest.raises(ImportError, match=r"needs acme_sdk >= 1.2, found 1.1.9"):
        sdk.require_sdk(pkg)


def test_require_sdk_warns_once_on_an_untested_newer_version(tmp_path, monkeypatch):
    pkg = _manifest(tmp_path, monkeypatch, MANIFEST)
    _installed(monkeypatch, {"acme_sdk": "2.0"})
    with pytest.warns(UserWarning, match="newer than any version run on hardware"):
        sdk.require_sdk(pkg)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        sdk.require_sdk(pkg)


def test_require_sdk_accepts_an_importable_module_without_metadata(tmp_path, monkeypatch):
    pkg = _manifest(tmp_path, monkeypatch, MANIFEST)
    _installed(monkeypatch, {})
    monkeypatch.setitem(sys.modules, "acme_sdk", types.ModuleType("acme_sdk"))
    assert sdk.require_sdk(pkg) == {"acme_sdk": None}


def test_version_ordering():
    assert sdk.parse_version("1.0.10") > sdk.parse_version("1.0.9")
    assert sdk.parse_version("0.2.0a8") < sdk.parse_version("0.2.0")


# ------------------------------------------------------------------ action types


def test_chunk_carries_base_velocity_and_latent():
    chunk = ActionChunk(mode=ControlMode.BASE_VELOCITY,
                        base_velocity=np.arange(12.0).reshape(4, 3))
    assert chunk.horizon == 4
    a = chunk.step(2)
    assert a.mode == ControlMode.BASE_VELOCITY and a.joint_target is None
    assert np.array_equal(a.base_velocity, [6.0, 7.0, 8.0])

    chunk = ActionChunk(mode=ControlMode.MOTION_LATENT, latent=np.ones((3, 64)))
    assert chunk.horizon == 3 and chunk.step(0).latent.shape == (64,)


def test_joint_drivers_do_not_claim_the_wbc_modes():
    assert ControlMode.BASE_VELOCITY not in Robot.action_modes
    assert ControlMode.MOTION_LATENT not in Robot.action_modes
    assert ControlMode("base_velocity") is ControlMode.BASE_VELOCITY
    assert Action(mode=ControlMode.MOTION_LATENT, latent=np.zeros(4)).latent.shape == (4,)


def test_observation_quaternion_is_scalar_last():
    from hirobot import get_body, load_embodiment

    robot = get_body("toy")(load_embodiment("unitree_g1"))
    robot.connect()
    obs: Observation = robot.get_observation()
    assert np.array_equal(obs.base_orientation, [0.0, 0.0, 0.0, 1.0])


# ------------------------------------------------------------- driver layout


#: Driver directories that need nothing beyond the core install, so have no manifest.
NO_SDK = {"sim/toy", "camera/generic/array", "audio/generic/wav",
          "end_effector/robotiq/urcap"}


def _driver_dirs():
    import pathlib

    from hirobot import drivers

    root = pathlib.Path(drivers.__file__).parent
    return sorted(f.parent.relative_to(root).as_posix() for f in root.rglob("driver.py"))


def test_only_sdk_free_drivers_lack_a_manifest():
    import pathlib

    from hirobot import drivers

    root = pathlib.Path(drivers.__file__).parent
    without = {d for d in _driver_dirs() if not (root / d / "manifest.yaml").is_file()}
    assert without == NO_SDK


@pytest.mark.parametrize("package", ["hirobot.drivers." + d.replace("/", ".")
                                     for d in _driver_dirs() if d not in NO_SDK])
def test_every_sdk_driver_has_a_valid_manifest(package):
    m = sdk.load_manifest(package)
    assert m.sdk, f"{package}/manifest.yaml names no SDK"
    for entry in m.verified:
        assert {"model", "sdk", "date"} <= set(entry), entry


