"""The MCP server (``hirobot.mcp.server``) and ``hirobot mcp``: a round trip with the SDK's client."""

from __future__ import annotations

import base64
import json
import sys

import pytest

from hirobot import make_robot
from hirobot.cli import main
from hirobot.mcp import McpSettings, RobotTools

CELL = {"assembly": {"name": "cell", "parts": {
    "body": {"from": "body/ur/ur5e", "overrides": {"driver": {"type": "toy"}}},
    "scene": {"device": "camera", "width": 64, "height": 48, "driver": {"type": "array"}}}},
    "safety": {"max_joint_step": 0.2}}


def _pos(v):
    return {"mode": "position", "joint_target": [v] * 6}


def test_round_trip_in_process():
    pytest.importorskip("mcp")
    import anyio
    from mcp import Client

    from hirobot.mcp import build_server

    robot = make_robot(CELL)
    robot.connect()

    async def go():
        with RobotTools(robot, McpSettings(tools=("observe", "act"))) as tools:
            async with Client(build_server(tools)) as client:
                listed = (await client.list_tools()).tools
                assert [t.name for t in listed] == ["describe_robot", "get_observation",
                                                     "get_robot_status", "send_actions",
                                                     "hold", "stop"]
                by_name = {t.name: t for t in listed}
                assert by_name["get_observation"].annotations.read_only_hint is True
                assert by_name["send_actions"].annotations.read_only_hint is False
                assert by_name["send_actions"].input_schema["properties"]["actions"]["maxItems"] == 5

                res = await client.call_tool("get_observation", {})
                assert not res.is_error
                assert [b.type for b in res.content] == ["text", "text", "image"]
                data = json.loads(res.content[0].text)
                assert data == res.structured_content and "joint_pos" in data["observation"]
                assert res.content[2].mime_type == "image/jpeg"
                assert base64.b64decode(res.content[2].data)[:2] == b"\xff\xd8"

                res = await client.call_tool("send_actions", {"actions": [_pos(0.1), _pos(0.2)],
                                                              "images": False})
                data = json.loads(res.content[0].text)
                assert (data["sent"], data["stopped"]) == (2, "done")
                assert data["observation"]["joint_pos"]["elbow"] == pytest.approx(0.2)

                res = await client.call_tool("send_actions", {"actions": [_pos(0.9)],
                                                              "images": False})
                assert json.loads(res.content[0].text)["stopped"] == "refused"

                res = await client.call_tool("send_actions", {"actions": []})
                assert res.is_error and "1 to 5 actions" in res.content[0].text
                res = await client.call_tool("reset", {})
                assert res.is_error and "not enabled" in res.content[0].text

    anyio.run(go)


def test_round_trip_over_stdio(tmp_path):
    pytest.importorskip("mcp")
    import anyio
    from mcp import Client, StdioServerParameters

    cfg = tmp_path / "cell.yaml"
    cfg.write_text("assembly:\n  name: cell\n  parts:\n"
                   "    body: {from: body/ur/ur5e, overrides: {driver: {type: toy}}}\n"
                   "mcp: {tools: [observe, act, episode], max_steps: 3}\n")

    async def go():
        params = StdioServerParameters(command=sys.executable,
                                       args=["-m", "hirobot.cli", "mcp", str(cfg)])
        with anyio.fail_after(60):
            await stdio_session(params)

    async def stdio_session(params):
        async with Client(params) as client:
            names = [t.name for t in (await client.list_tools()).tools]
            assert names[-1] == "reset" and "run_skill" not in names
            res = await client.call_tool("send_actions",
                                         {"actions": [_pos(0.05)] * 3, "images": False})
            data = json.loads(res.content[0].text)
            assert data["sent"] == 3
            assert data["observation"]["joint_pos"]["wrist_3"] == pytest.approx(0.05)
            res = await client.call_tool("send_actions", {"actions": [_pos(0.0)] * 4})
            assert res.is_error and "1 to 3 actions" in res.content[0].text

    anyio.run(go)


def test_cli_without_mcp_installed(monkeypatch, capsys):
    import importlib.util

    real = importlib.util.find_spec
    monkeypatch.setattr(importlib.util, "find_spec",
                        lambda name, *a: None if name == "mcp" else real(name, *a))
    assert main(["mcp", "robots/ur_ur5e"]) == 1
    assert "pip install 'hirobot[mcp]'" in capsys.readouterr().err


def test_build_server_without_mcp_says_how_to_install(monkeypatch):
    import builtins

    real_import = builtins.__import__

    def no_mcp(name, *a, **k):
        if name == "mcp" or name.startswith("mcp.") or name == "mcp_types":
            raise ImportError(name)
        return real_import(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", no_mcp)
    from hirobot.mcp.server import build_server

    robot = make_robot(CELL)
    with RobotTools(robot) as tools, pytest.raises(ImportError, match=r"hirobot\[mcp\]"):
        build_server(tools)


def test_serve_mcp_checks_arguments():
    from hirobot.mcp import serve_mcp

    robot = make_robot(CELL)
    with pytest.raises(ValueError, match="transport"):
        serve_mcp(robot, transport="sse")
    with pytest.raises(ValueError, match="max_steps"):
        serve_mcp(robot, max_steps=0)


def test_import_hirobot_does_not_import_mcp_or_pillow():
    import subprocess

    code = ("import sys, hirobot, hirobot.mcp, hirobot.mcp.tools, hirobot.mcp.convert; "
            "bad = [m for m in ('mcp', 'mcp_types', 'PIL', 'anyio', 'uvicorn') if m in sys.modules]; "
            "print(bad); sys.exit(1 if bad else 0)")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
