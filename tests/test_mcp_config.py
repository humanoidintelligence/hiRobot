"""The ``mcp:`` root section: validation, defaults, root-only, and ``Robot.mcp_config``."""

from __future__ import annotations

import pytest

from hirobot import make_robot
from hirobot.mcp import McpSettings, parse_mcp_config

BODY = {"from": "body/ur/ur5e", "overrides": {"driver": {"type": "toy"}}}


def _cfg(**root):
    return {"assembly": {"name": "cell", "parts": {"body": BODY}}, **root}


def test_defaults():
    s = parse_mcp_config(None)
    assert s == McpSettings() == parse_mcp_config({})
    assert s.tools == ("observe",) and s.max_steps == 5
    assert (s.image_max_side, s.image_quality, s.envelope) == (512, 80, {})


def test_tools_keep_the_canonical_order():
    s = parse_mcp_config({"tools": ["episode", "act", "observe", "act"]})
    assert s.tools == ("observe", "act", "episode")


@pytest.mark.parametrize("bad, match", [
    ({"tools": "act"}, "tools: must be a list"),
    ({"tools": ["drive"]}, "unknown categor"),
    ({"max_steps": 0}, "max_steps"),
    ({"max_steps": 51}, "max_steps"),
    ({"max_steps": True}, "max_steps"),
    ({"image_quality": 101}, "image_quality"),
    ({"image_max_side": 8}, "image_max_side"),
    ({"envelope": {"max_speed": 1}}, "envelope: unknown"),
    ({"envelope": [1]}, "envelope: must be a mapping"),
    ({"port": 1}, "unknown key"),
])
def test_rejects(bad, match):
    with pytest.raises(ValueError, match=match):
        parse_mcp_config(bad)


def test_merged_overrides_win_and_none_keeps():
    s = parse_mcp_config({"tools": ["observe"], "max_steps": 3})
    m = s.merged(tools=["act", "observe"], max_steps=None)
    assert m.tools == ("observe", "act") and m.max_steps == 3
    assert s.merged(tools="act").tools == ("act",)
    with pytest.raises(TypeError, match="unknown MCP setting"):
        s.merged(port=1)
    with pytest.raises(ValueError):
        s.merged(max_steps=99)


def test_make_robot_keeps_the_section():
    robot = make_robot(_cfg(mcp={"tools": ["observe", "act"], "max_steps": 8}))
    assert robot.mcp_config == {"tools": ["observe", "act"], "max_steps": 8}
    assert make_robot(_cfg()).mcp_config is None


def test_bad_section_fails_at_load():
    with pytest.raises(ValueError, match="mcp: max_steps"):
        make_robot(_cfg(mcp={"max_steps": 100}))


def test_only_the_root_may_have_it(tmp_path):
    part = tmp_path / "cell.yaml"
    part.write_text("assembly:\n  name: cell\n  parts:\n    body: {from: body/ur/ur5e}\n"
                    "mcp: {tools: [act]}\n")
    with pytest.raises(ValueError, match="mcp: is only for the root config"):
        make_robot({"assembly": {"name": "top", "parts": {"cell": {"from": str(part)}}}})
