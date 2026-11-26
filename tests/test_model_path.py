"""``Embodiment.model``, ``hirobot.model_path`` and ``hirobot.available_configs``."""

import os
import textwrap
from pathlib import Path

import pydantic
import pytest

import hirobot
from hirobot import Embodiment, ModelRef, load_embodiment, make_robot, model_path
from hirobot.loading import config as config_module

G1_MJCF = Path("~/.cache/hirobot/assets/unitree_ros/robots/g1_description/"
               "g1_29dof_rev_1_0.xml").expanduser()


def _write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text))
    return path


def _body(model: str = "") -> str:
    return "device: body\ncontrol_hz: 50\nfloating_base: false\njoints: [{name: j1}]\n" + model


def test_unitree_g1_model_is_an_absolute_mjcf_path():
    model = load_embodiment("unitree_g1").model
    assert model is not None and model.kind == "mjcf"
    assert os.path.isabs(model.path) and "~" not in model.path
    assert model.path == str(G1_MJCF)
    assert model.package_roots == {}
    assert model_path("unitree_g1") == model
    assert make_robot("unitree_g1").embodiment.model == model  # the built robot has it too
    if not G1_MJCF.is_file():
        pytest.skip(f"G1 MJCF not downloaded to {G1_MJCF}")
    assert os.path.isfile(model.path)


def test_one_body_without_model_is_none():
    # AgiBot publishes no MJCF or URDF with a license that allows redistribution.
    assert load_embodiment("agibot_x1").model is None
    assert model_path("agibot_x1") is None
    assert make_robot("agibot_x1").embodiment.model is None


def test_more_than_one_body_or_none():
    assert load_embodiment("ur5e_dual").model is None
    with pytest.raises(ValueError, match=r"\['left/body', 'right/body'\]"):
        model_path("ur5e_dual")
    with pytest.raises(ValueError, match="none"):
        model_path({"device": "camera", "width": 4, "height": 4})


def test_a_config_that_does_not_load_raises_something_else():
    with pytest.raises(FileNotFoundError):
        model_path("no_such_robot_config")


def test_relative_and_home_paths_are_expanded(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    _write(tmp_path / "body/arm.yaml", _body("model: {mjcf: ~/models/arm.xml}\n"))
    _write(tmp_path / "body/rel.yaml", _body("model: {mjcf: ../models/arm.xml}\n"))
    home = model_path(tmp_path / "body/arm.yaml")
    assert home.path == str(tmp_path / "home/models/arm.xml")
    rel = model_path(tmp_path / "body/rel.yaml")
    assert os.path.isabs(rel.path)
    assert os.path.normpath(rel.path) == str((tmp_path / "models/arm.xml").resolve())


def test_urdf_and_package_roots(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    _write(tmp_path / "body/arm.yaml", _body(
        "model:\n  urdf: ../urdf/arm.urdf\n"
        "  package_roots: {arm_description: ../urdf, meshes: ~/meshes}\n"))
    model = model_path(tmp_path / "body/arm.yaml")
    assert model.kind == "urdf"
    assert os.path.normpath(model.path) == str((tmp_path / "urdf/arm.urdf").resolve())
    roots = {k: os.path.normpath(v) for k, v in model.package_roots.items()}
    assert roots == {"arm_description": str((tmp_path / "urdf").resolve()),
                     "meshes": str(tmp_path / "home/meshes")}


def test_mjcf_wins_over_urdf(tmp_path):
    _write(tmp_path / "body/arm.yaml", _body(
        "model: {mjcf: /m/arm.xml, urdf: /m/arm.urdf, package_roots: {arm: /m}}\n"))
    assert model_path(tmp_path / "body/arm.yaml") == ModelRef(kind="mjcf", path="/m/arm.xml")


@pytest.mark.parametrize("model, match", [
    ("model: {kp: 10}\n", "mjcf or urdf"),
    ("model: {mjcf: /m/a.xml, package_roots: {a: /m}}\n", "package_roots is for a urdf"),
    ("model: {urdf: /m/a.urdf, package_roots: [/m]}\n", "package_roots must map"),
    ("model: {mjcf: /m/a.xml, scale: 2}\n", "unknown key"),
])
def test_bad_model_sections(tmp_path, model, match):
    _write(tmp_path / "body/arm.yaml", _body(model))
    with pytest.raises(ValueError, match=match):
        load_embodiment(tmp_path / "body/arm.yaml")


def test_urdf_only_body_is_refused_where_an_mjcf_is_needed(tmp_path):
    _write(tmp_path / "body/arm.yaml", _body("model: {urdf: /m/arm.urdf}\n"))
    cfg = {"assembly": {"parts": {"body": {"from": str(tmp_path / "body/arm.yaml"),
                                           "overrides": {"driver": {"type": "mujoco"}}}}}}
    with pytest.raises(ValueError, match="only a urdf"):
        make_robot(cfg)
    with pytest.raises(ValueError, match="only a urdf"):
        make_robot({"assembly": {"parts": {"body": {"from": str(tmp_path / "body/arm.yaml")}}}},
                   sim=True)


def test_model_ref_is_frozen_and_checked():
    ref = ModelRef(kind="mjcf", path="/m/arm.xml")
    with pytest.raises(pydantic.ValidationError):
        ref.path = "/other.xml"
    with pytest.raises(pydantic.ValidationError, match="absolute"):
        ModelRef(kind="mjcf", path="models/arm.xml")
    with pytest.raises(pydantic.ValidationError, match="URDF model only"):
        ModelRef(kind="mjcf", path="/m/arm.xml", package_roots={"arm": "/m"})
    with pytest.raises(pydantic.ValidationError):
        ModelRef(kind="sdf", path="/m/arm.sdf")


def test_model_ref_is_hashable():
    a = ModelRef(kind="urdf", path="/m/arm.urdf", package_roots={"b": "/b", "a": "/a"})
    b = ModelRef(kind="urdf", path="/m/arm.urdf", package_roots={"a": "/a", "b": "/b"})
    assert a == b and hash(a) == hash(b)
    assert hash(ModelRef(kind="mjcf", path="/m/arm.xml")) == hash(
        ModelRef(kind="mjcf", path="/m/arm.xml"))
    assert {a: 1}[b] == 1
    assert len({a, b, ModelRef(kind="mjcf", path="/m/arm.xml")}) == 2
    assert a.package_roots["a"] == "/a" and dict(a.package_roots) == {"a": "/a", "b": "/b"}
    assert sorted(a.package_roots) == ["a", "b"]


def test_sim_check_refuses_a_urdf_only_body_at_build_time(tmp_path):
    _write(tmp_path / "body/arm.yaml", _body("model: {urdf: /m/arm.urdf}\n"))
    cfg = {"assembly": {"parts": {"body": {"from": str(tmp_path / "body/arm.yaml")}}},
           "safety": {"sim_check": True}}
    with pytest.raises(ValueError, match=r"needs an mjcf.*\['body'\] have only a urdf"):
        make_robot(cfg)


def test_embodiment_without_model_still_works():
    emb = Embodiment(name="x", joints=[{"name": "j"}])
    assert emb.model is None
    assert Embodiment(**emb.model_dump()) == emb
    with_model = emb.model_copy(update={"model": ModelRef(kind="mjcf", path="/m/a.xml")})
    assert Embodiment.model_validate(with_model.model_dump(mode="json")) == with_model


def test_legacy_config_takes_the_mujoco_model_path():
    old = {"embodiment": {"name": "old", "joints": [{"name": "j"}]},
           "body": {"driver": "mujoco", "model_path": "~/models/old.xml"}}
    with pytest.warns(DeprecationWarning):
        assert model_path(old) == ModelRef(kind="mjcf",
                                           path=os.path.expanduser("~/models/old.xml"))
    with pytest.warns(DeprecationWarning):
        assert load_embodiment({**old, "body": {"driver": "toy"}}).model is None


def test_available_configs_is_the_loader_one():
    assert hirobot.available_configs is config_module.available_configs
    names = hirobot.available_configs("robots")
    assert "robots/unitree_g1" in names and names == sorted(names)
    assert {"ModelRef", "model_path", "available_configs"} <= set(hirobot.__all__)


def test_info_prints_the_model(capsys):
    from hirobot.cli import main

    assert main(["info", "--config", "unitree_g1"]) == 0
    assert f"model:         mjcf {G1_MJCF}" in capsys.readouterr().out
