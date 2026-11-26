"""Every built-in body's ``model:`` points into the asset dataset and matches the body.

The first test needs no download. The second loads each model that is downloaded
(``hirobot assets fetch --all``) and checks that every joint and every frame / mount
link of the body config is in it; missing models are skipped.
"""

from pathlib import Path

import pytest
import yaml

import hirobot
from hirobot import assets, available_configs

CONFIGS = Path(hirobot.__file__).parent / "configs"
BODIES = available_configs("body")
# Bodies with no model: no MJCF or URDF whose license allows redistribution
# (see the dataset README's "Not included").
NO_MODEL = {"body/agibot/g1", "body/agibot/x1"}


def _device(name):
    return yaml.safe_load((CONFIGS / f"{name}.yaml").read_text())


def _model_file(name):
    model = _device(name).get("model") or {}
    path = model.get("mjcf") or model.get("urdf")
    return None if path is None else Path(path).expanduser()


def test_every_body_but_agibot_has_a_model_from_the_dataset():
    for name in BODIES:
        path = _model_file(name)
        if name in NO_MODEL:
            assert path is None, name
            continue
        assert path is not None, f"{name} has no model:"
        assert assets.asset_of(path) is not None, f"{name}: {path} is in no hirobot asset"


@pytest.mark.parametrize("name", sorted(set(BODIES) - NO_MODEL))
def test_model_has_the_body_joints_and_links(name):
    mujoco = pytest.importorskip("mujoco")
    path = _model_file(name)
    if not path.is_file():
        pytest.skip(f"{path} not downloaded; {assets.fetch_hint(path)}")
    m = mujoco.MjModel.from_xml_path(str(path))
    joints = {m.joint(i).name for i in range(m.njnt)}
    links = {m.body(i).name for i in range(m.nbody)}
    dev = _device(name)
    # The simulation and the kinematics match a joint by its name or name + "_joint".
    missing = [j["name"] for j in dev["joints"]
               if j["name"] not in joints and j["name"] + "_joint" not in joints]
    assert not missing, f"{name}: joints not in {path.name}: {missing}"
    named = [f for f in dev.get("frames") or [] if f.get("link")]
    named += [{"name": k, **v} for k, v in (dev.get("mounts") or {}).items()]
    bad = {f["name"]: f["link"] for f in named if f["link"] not in links}
    assert not bad, f"{name}: links not in {path.name}: {bad}"
