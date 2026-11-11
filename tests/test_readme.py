"""The Python blocks in README.md and README.zh-CN.md run as written."""

import os
import re
from pathlib import Path

import pytest

import hirobot
import hirobot.loading.config
from tests.conftest import mjcf_of

ROOT = Path(__file__).resolve().parent.parent
READMES = ["README.md", "README.zh-CN.md"]


def _blocks(name):
    text = (ROOT / name).read_text(encoding="utf-8")
    return re.findall(r"^```python\n(.*?)^```", text, flags=re.S | re.M)


def _code(block):
    """The block without comments, to compare the two languages."""
    return [line.split("#")[0].rstrip() for line in block.splitlines() if line.split("#")[0].strip()]


def _mujoco_model(config):
    return mjcf_of(config)


def test_both_readmes_run_the_same_code():
    en, zh = (_blocks(n) for n in READMES)
    assert len(en) == len(zh) >= 2
    assert [_code(b) for b in en] == [_code(b) for b in zh]


@pytest.mark.parametrize("index", range(len(_blocks("README.md"))))
def test_readme_block_runs(index, monkeypatch):
    block = _blocks("README.md")[index]
    configs = re.findall(r'make_robot\("([^"]+)"\)', block)
    if any("mujoco" in c for c in configs):
        pytest.importorskip("mujoco")
        for c in configs:
            if not os.path.isfile(_mujoco_model(c)):
                pytest.skip(f"MJCF for {c} not found")

    make_robot = hirobot.make_robot

    def fast_make_robot(config, overrides=None, **kw):
        # Same robot, without the wall-clock pacing and the start ramp.
        if "mujoco" in str(config):
            fast = {"body": {"driver": {"realtime": False, "ramp_seconds": 0.0, "viewer": False}}}
            overrides = hirobot.loading.config.merge(fast, dict(overrides or {}))
        return make_robot(config, overrides=overrides, **kw)

    monkeypatch.setattr(hirobot, "make_robot", fast_make_robot)
    namespace = {}
    exec(compile(block, f"README.md block {index + 1}", "exec"), namespace)
    if "elbow" in namespace:  # the G1 example: the elbow got to 1 rad
        obs = namespace["robot"].get_observation()
        assert obs.joint_pos[namespace["elbow"]] == pytest.approx(1.0, abs=0.1)


def _layer_example(name):
    text = (ROOT / "docs" / name).read_text(encoding="utf-8")
    heading = {"api.md": "### Writing a layer", "api.zh-CN.md": "### 编写层"}[name]
    section = text.split(heading, 1)[1].split("\n## ", 1)[0]
    (block,) = re.findall(r"^```python\n(.*?)^```", section, flags=re.S | re.M)
    return block


def test_both_layer_examples_run_the_same_code():
    en, zh = (_layer_example(n) for n in ("api.md", "api.zh-CN.md"))
    assert _code(en) == _code(zh)


@pytest.mark.parametrize("name", ["api.md", "api.zh-CN.md"])
def test_layer_example_runs_and_uses_only_the_public_api(name):
    import numpy as np

    from hirobot.loading.registry import LAYERS
    from hirobot.testing import assert_layer_uses_public_api

    namespace = {}
    try:
        exec(compile(_layer_example(name), f"docs/{name} layer example", "exec"), namespace)
        np.testing.assert_allclose(namespace["robot"].get_observation().joint_pos, 0.05)
        assert_layer_uses_public_api(namespace["RateLimit"], {"max_step": 0.05})
    finally:
        LAYERS._entries.pop("rate_limit", None)
