"""hirobot.assets and `hirobot assets`: a fake huggingface_hub, never the network."""

from __future__ import annotations

import fnmatch
import sys
import types
from pathlib import Path

import pytest

from hirobot import assets
from hirobot.cli import main


@pytest.fixture
def home(tmp_path, monkeypatch):
    """A fresh home directory, so ~/.cache/hirobot/assets is empty and private to the test."""
    monkeypatch.setenv("HOME", str(tmp_path))
    return tmp_path


@pytest.fixture
def adir(home):
    return home / ".cache/hirobot/assets"


@pytest.fixture
def fake_hub(monkeypatch):
    calls = []

    def snapshot_download(**kwargs):
        calls.append(kwargs)
        return kwargs["local_dir"]

    mod = types.ModuleType("huggingface_hub")
    mod.snapshot_download = snapshot_download
    monkeypatch.setitem(sys.modules, "huggingface_hub", mod)
    return calls


# Paths in the dataset: one or two model files per name, plus every license, provenance
# and readme file the dataset has.
DATASET = [
    "README.md",
    "unitree_ros/LICENSE", "unitree_ros/SOURCE.md",
    "unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml",
    "unitree_ros/robots/g1_description/meshes/pelvis.STL",
    "unitree_ros/robots/h1_description/mjcf/h1.xml",
    "unitree_ros/robots/h1_2_description/h1_2.xml",
    "unitree_ros/robots/h2_description/urdf/h2.urdf",
    "unitree_mujoco/LICENSE", "unitree_mujoco/SOURCE.md",
    "unitree_mujoco/unitree_robots/h2/h2_mujoco.xml",
    "unitree_mujoco/unitree_robots/h2/meshes/pelvis.stl",
    "unitree_ros/robots/dexterous_hand_description/urdf/dex3_1_l.urdf",
    "menagerie/SOURCE.md",
    *(f"menagerie/{d}/{f}" for d in assets._MENAGERIE for f in ("LICENSE", "scene.xml")),
    "fourier/README.md", "fourier/SOURCE.md", "fourier/Wiki-GRx-Models/LICENSE",
    "fourier/Wiki-GRx-Models/GR1_legacy/GR1/GR1T1/urdf/GR1T1.urdf",
    "fourier/Wiki-GRx-Models/GRX/GR1/gr1t2/urdf/gr1t2.urdf",
    "fourier/Wiki-GRx-Models/GRX/GR2/mjcf/gr2.xml",
    "inspire/LICENSE", "inspire/SOURCE.md",
    "inspire/rh56dfx_description/LICENSE", "inspire/rh56dfx_description/urdf/rh56dfx_left.urdf",
    "cameras/zed/LICENSE", "cameras/zed/SOURCE.md",
    "cameras/zed/zed_description/urdf/zed_macro.urdf.xacro",
    "cameras/realsense/LICENSE", "cameras/realsense/RealSense Copyright",
    "cameras/realsense/SOURCE.md", "cameras/realsense/realsense2_description/urdf/_d435.urdf.xacro",
    "cameras/orbbec/LICENSE", "cameras/orbbec/NOTICE", "cameras/orbbec/SOURCE.md",
    "cameras/orbbec/orbbec_description/urdf/dabai_dc1.urdf.xacro",
]
_LEGAL = ("LICENSE", "SOURCE.md", "README.md", "NOTICE", "RealSense Copyright")


def _pulled(name):
    return [p for p in DATASET if any(fnmatch.fnmatch(p, pat) for pat in assets.ASSETS[name])]


def test_assets_dir():
    assert assets.REPO_ID == "humanoidintelligence/hirobot-assets"
    assert assets.assets_dir() == Path("~/.cache/hirobot/assets").expanduser()
    assert set(assets.DESCRIPTIONS) == set(assets.ASSETS)


@pytest.mark.parametrize("name", list(assets.ASSETS))
def test_each_name_pulls_a_model_its_license_and_source(name):
    pulled = _pulled(name)
    assert any(Path(p).name not in _LEGAL for p in pulled), name
    assert any(Path(p).name == "LICENSE" for p in pulled), name
    assert any(Path(p).name == "SOURCE.md" for p in pulled), name
    assert "README.md" in pulled


def test_each_pattern_matches_a_dataset_path():
    for name, patterns in assets.ASSETS.items():
        for pat in patterns:
            assert any(fnmatch.fnmatch(p, pat) for p in DATASET), (name, pat)


def test_fourier_models_stay_apart():
    assert not [p for p in _pulled("fourier_gr2") if "GR1" in p]
    assert not [p for p in _pulled("fourier_gr1") if "GR2" in p]
    assert any("GR1_legacy" in p for p in _pulled("fourier_gr1"))
    assert any("GRX/GR1" in p for p in _pulled("fourier_gr1"))


def test_fetch_calls_snapshot_download(adir, fake_hub):
    root = assets.fetch(["unitree_g1", "menagerie_universal_robots_ur5e"], token="t")
    assert root == adir and adir.is_dir()
    (call,) = fake_hub
    assert call["repo_id"] == assets.REPO_ID
    assert call["repo_type"] == "dataset"
    assert call["revision"] == assets.REVISION
    assert call["local_dir"] == str(adir)
    assert call["token"] == "t"
    pats = call["allow_patterns"]
    assert "unitree_ros/robots/g1_description/*" in pats
    assert "menagerie/universal_robots_ur5e/*" in pats
    assert len(pats) == len(set(pats))  # the shared README.md once


def test_fetch_one_name_as_a_string(adir, fake_hub):
    assets.fetch("unitree_h1")
    assert fake_hub[0]["allow_patterns"][0] == "unitree_ros/robots/h1_description/*"


def test_unknown_name_raises(adir, fake_hub):
    with pytest.raises(ValueError, match="unitree_g1"):
        assets.fetch(["nope"])
    with pytest.raises(ValueError):
        assets.is_fetched("nope")
    assert not fake_hub


def test_missing_huggingface_hub(adir, monkeypatch):
    monkeypatch.setitem(sys.modules, "huggingface_hub", None)  # import raises ImportError
    with pytest.raises(RuntimeError, match=r"pip install 'hirobot\[assets\]'"):
        assets.fetch("unitree_g1")


def test_is_fetched_and_asset_of(adir):
    assert not assets.is_fetched("unitree_g1")
    xml = adir / "unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml"
    xml.parent.mkdir(parents=True)
    xml.write_text("<mujoco/>")
    assert assets.is_fetched("unitree_g1")
    assert not assets.is_fetched("unitree_h1")
    assert assets.asset_of(xml) == "unitree_g1"
    assert assets.asset_of("~/.cache/hirobot/assets/menagerie/universal_robots_ur5e/ur5e.xml") \
        == "menagerie_universal_robots_ur5e"
    assert assets.asset_of(adir / "unitree_ros/LICENSE") is None
    assert assets.asset_of("/elsewhere/model.xml") is None
    assert "hirobot assets fetch unitree_g1" in assets.fetch_hint(xml)


def test_builtin_configs_point_into_the_assets(home):
    # The built-in MuJoCo configs name files that `hirobot assets fetch` provides.
    from hirobot import model_path

    path = model_path("unitree_g1_mujoco").path
    assert assets.asset_of(path) == "unitree_g1"


def test_cli_list(adir, capsys):
    assert main(["assets", "list"]) == 0
    out = capsys.readouterr().out
    for name in assets.ASSETS:
        assert name in out
    assert str(adir) in out


def test_cli_fetch(adir, fake_hub, capsys):
    assert main(["assets", "fetch", "unitree_g1"]) == 0
    assert fake_hub[0]["allow_patterns"] == assets.ASSETS["unitree_g1"]
    assert main(["assets", "fetch", "--all"]) == 0
    assert all(p in fake_hub[1]["allow_patterns"] for ps in assets.ASSETS.values() for p in ps)
    assert main(["assets", "fetch", "nope"]) == 2
    with pytest.raises(SystemExit):
        main(["assets", "fetch"])
    with pytest.raises(SystemExit):
        main(["assets", "fetch", "unitree_g1", "--all"])


def test_cli_fetch_download_error(adir, monkeypatch, capsys):
    class RepositoryNotFoundError(OSError):  # what huggingface_hub raises, an OSError
        pass

    def snapshot_download(**kwargs):
        raise RepositoryNotFoundError("404 Client Error.\nRepository Not Found")

    mod = types.ModuleType("huggingface_hub")
    mod.snapshot_download = snapshot_download
    monkeypatch.setitem(sys.modules, "huggingface_hub", mod)
    assert main(["assets", "fetch", "unitree_g1"]) == 2
    err = capsys.readouterr().err
    assert err.startswith("hirobot assets: ") and err.count("\n") == 1
    assert "hf auth login" in err and "HF_ENDPOINT" in err


def test_cli_fetch_without_huggingface_hub(adir, monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "huggingface_hub", None)
    assert main(["assets", "fetch", "unitree_g1"]) == 2
    assert "hirobot assets: " in capsys.readouterr().err


def test_repo_env_points_fetch_at_another_dataset(adir, fake_hub, monkeypatch):
    monkeypatch.setenv(assets.REPO_ENV, "someone/hirobot-assets")
    assert assets.repo_id() == "someone/hirobot-assets"
    assets.fetch("unitree_g1")
    assert fake_hub[0]["repo_id"] == "someone/hirobot-assets"


def test_repo_env_unset_uses_repo_id(monkeypatch):
    monkeypatch.delenv(assets.REPO_ENV, raising=False)
    assert assets.repo_id() == assets.REPO_ID
