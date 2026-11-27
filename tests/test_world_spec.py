"""hirobot.sim.world_spec: every body of a robot composed into one MuJoCo spec, no stage.

Uses the MuJoCo Menagerie UR5e and the FR3 MJCFs under ~/.cache/hirobot/assets/menagerie/;
skipped when they or mujoco are missing.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from hirobot import load_embodiment

mujoco = pytest.importorskip("mujoco")

from hirobot.sim.world_spec import world_spec  # noqa: E402

MENAGERIE = Path.home() / ".cache/hirobot/assets/menagerie"
UR5E = MENAGERIE / "universal_robots_ur5e/ur5e.xml"
FR3 = MENAGERIE / "franka_fr3/fr3_hirobot.xml"


def _model_joint(joints, name):
    """The spec joint the simulation driver maps ``name`` to: ``name`` or ``name_joint``."""
    return next((c for c in (name, f"{name}_joint") if c in joints), None)


@pytest.mark.skipif(not UR5E.is_file(), reason=f"UR5e MJCF not found at {UR5E}")
def test_ur5e_dual_has_both_arms_at_their_placements():
    spec, joints = world_spec("ur5e_dual")
    m = spec.compile()
    assert joints == [m.joint(i).name for i in range(m.njnt)]
    emb = load_embodiment("ur5e_dual")
    # Every robot joint is in the world under its prefix, and nothing else is.
    mapped = [_model_joint(joints, n) for n in emb.joint_names]
    assert None not in mapped and sorted(mapped) == sorted(joints)
    assert {j.split("/")[0] for j in joints} == {"left", "right"}
    # No default stage: the models bring no floor, light or sky, and none is added.
    assert not any(int(t) == int(mujoco.mjtGeom.mjGEOM_PLANE) for t in m.geom_type)
    assert m.nlight == 0
    # Each arm's base sits at its placement (y = +-0.4).
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    for side, y in (("left", 0.4), ("right", -0.4)):
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{side}/base")
        assert base >= 0
        np.testing.assert_allclose(d.xpos[base], [0.0, y, 0.0], atol=1e-6)


@pytest.mark.skipif(not (UR5E.is_file() and FR3.is_file()), reason="UR5e or FR3 MJCF missing")
def test_franka_ur5e_dual_has_both_arms():
    spec, joints = world_spec("franka_ur5e_dual")
    spec.compile()
    emb = load_embodiment("franka_ur5e_dual")
    mapped = [_model_joint(joints, n) for n in emb.joint_names]
    assert None not in mapped and sorted(mapped) == sorted(joints)
    assert any(j.startswith("left/joint") for j in joints)
    assert any(j.startswith("right/shoulder_pan") for j in joints)


def test_a_body_without_a_model_is_refused():
    with pytest.raises(ValueError, match="needs a model"):
        world_spec("agibot_g1")
