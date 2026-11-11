"""``hirobot sim``: runs simulated robots, refuses real ones."""

import os
from pathlib import Path

import pytest

from hirobot.cli import main
from tests.conftest import mjcf_of

EXAMPLES = Path(__file__).parents[1] / "examples"


def test_sim_runs_the_toy_humanoid(capsys):
    assert main(["sim", str(EXAMPLES / "humanoid.yaml"), "--seconds", "0.2"]) == 0
    out = capsys.readouterr().out
    assert "generic-humanoid (10 dofs, 50 Hz, floating base)" in out
    assert "images:        none" in out
    assert "max joint error: 0.0000 rad" in out


def test_sim_refuses_a_real_robot(tmp_path, capsys):
    cfg = tmp_path / "real.yaml"
    cfg.write_text(f"defaults: [{EXAMPLES / 'humanoid.yaml'}]\n"
                   "assembly:\n  parts:\n    body:\n      driver: {type: unitree_sdk2, "
                   "network_interface: eth9}\n")
    pytest.importorskip("unitree_sdk2py")
    assert main(["sim", str(cfg)]) == 2
    assert "only runs simulated ones" in capsys.readouterr().err


def test_sim_viewer_needs_mujoco(capsys):
    assert main(["sim", str(EXAMPLES / "arm.yaml"), "--viewer"]) == 2
    assert "--viewer needs the MuJoCo simulation" in capsys.readouterr().err


def _g1_model_or_skip(name):
    pytest.importorskip("mujoco")
    path = mjcf_of(name)
    if not os.path.isfile(path):
        pytest.skip(f"G1 MJCF not found at {path}")


def test_sim_g1_holds_welded_and_reports_the_head_camera(capsys):
    _g1_model_or_skip("unitree_g1_mujoco")
    assert main(["sim", "unitree_g1_mujoco", "--fast", "--seconds", "1"]) == 0
    out = capsys.readouterr().out
    assert "floating base, welded in sim" in out
    assert "it fell" not in out
    if "'head'" not in out:  # no GL context on this machine
        assert "images:        none" in out


def test_sim_reports_a_free_g1_falling(capsys):
    _g1_model_or_skip("unitree_g1_mujoco_standing")
    assert main(["sim", "unitree_g1_mujoco_standing", "--fast", "--seconds", "2"]) == 0
    assert "it fell" in capsys.readouterr().out
