"""``hirobot.mcp.tools.RobotTools``: every tool, the action chunk, stop, the robot thread, the lease."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from hirobot import make_robot
from hirobot.mcp import McpSettings
from hirobot.mcp.convert import action_from_json, encode_image, jsonable
from hirobot.mcp.tools import RobotTools, ToolError, tool_specs

from tests.test_skills import _stack

ALL = ("observe", "act", "skill", "episode")


def _robot(**root):
    cfg = {"assembly": {"name": "cell", "parts": {
        "body": {"from": "body/ur/ur5e", "overrides": {"driver": {"type": "toy"}}},
        "scene": {"device": "camera", "width": 64, "height": 48, "driver": {"type": "array"}}}},
        "safety": {"max_joint_step": 0.2}, **root}
    robot = make_robot(cfg)
    robot.connect()
    return robot


class FakeClock:
    def __init__(self):
        self.t, self.sleeps = 0.0, []

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.sleeps.append(round(dt, 6))
        self.t += dt


def _tools(robot=None, **settings):
    clock = FakeClock()
    settings.setdefault("tools", ALL)
    tools = RobotTools(robot or _robot(), McpSettings(**settings), sleep=clock.sleep, clock=clock)
    return tools, clock


def _pos(v):
    return {"mode": "position", "joint_target": [v] * 6}


# ------------------------------------------------------------------ specs


def test_default_settings_serve_only_observe():
    robot = _robot()
    with RobotTools(robot) as tools:
        assert [t.name for t in tools.specs] == ["describe_robot", "get_observation",
                                                  "get_robot_status"]
        with pytest.raises(ToolError, match="not enabled"):
            tools.call("send_actions", {"actions": [_pos(0.0)]})
        with pytest.raises(ToolError, match="unknown tool"):
            tools.call("estop")


def test_config_section_is_used():
    robot = _robot(mcp={"tools": ["observe", "episode"]})
    with RobotTools(robot) as tools:
        assert {t.category for t in tools.specs} == {"observe", "episode"}


def test_no_estop_and_schema_max_items():
    specs = tool_specs(McpSettings(tools=ALL, max_steps=7))
    names = [s.name for s in specs]
    assert "estop" not in names and "finish" not in names
    assert len(names) == len(set(names)) == 10
    chunk = next(s for s in specs if s.name == "send_actions")
    assert chunk.schema["properties"]["actions"]["maxItems"] == 7


def test_bad_arguments():
    tools, _ = _tools()
    with tools:
        with pytest.raises(ToolError, match="unknown argument"):
            tools.call("get_observation", {"camera": "scene"})
        with pytest.raises(ToolError, match="missing argument"):
            tools.call("send_actions", {})
        with pytest.raises(ToolError, match="images must be boolean"):
            tools.call("get_observation", {"images": "yes"})
        with pytest.raises(ToolError, match="seed must be integer"):
            tools.call("reset", {"seed": True})


# ---------------------------------------------------------------- observe


def test_describe_robot():
    tools, _ = _tools()
    with tools:
        d = tools.call("describe_robot").data
    assert d["name"] == "cell" and d["control_hz"] == 125
    assert [j["name"] for j in d["joints"]][:2] == ["shoulder_pan", "shoulder_lift"]
    assert "position" in d["action_modes"] and d["max_steps"] == 5
    (cam,) = d["cameras"]
    assert cam["name"] == "scene" and set(cam) >= {"intrinsics", "intrinsics_source", "mount"}
    assert "send_actions" in d["tools"]


def test_get_observation_with_and_without_images():
    pytest.importorskip("PIL")  # JPEG encoding: the mcp extra
    tools, _ = _tools(image_max_side=32)
    with tools:
        r = tools.call("get_observation")
        assert set(r.data["observation"]["joint_pos"]) >= {"shoulder_pan", "wrist_3"}
        (img,) = r.images
        assert img.camera == "scene" and max(img.width, img.height) == 32
        assert img.jpeg[:2] == b"\xff\xd8"
        assert r.data["images"] == [{"camera": "scene", "width": img.width, "height": img.height}]
        assert tools.call("get_observation", {"images": False}).images == []
        r = tools.call("get_observation", {"cameras": ["wrist"]})
        assert r.images == [] and r.data["missing_images"] == ["wrist"]


def test_get_robot_status():
    tools, _ = _tools()
    with tools:
        s = tools.call("get_robot_status").data
    assert s["skill_running"] is False and s["mcp_lease"] == "none" and "safety" in s


# -------------------------------------------------------------------- act


def test_send_actions_paced_at_control_hz_and_holds():
    robot = _robot()
    held = []
    orig = type(robot).hold
    robot.hold = lambda: (held.append(1), orig(robot))[1]  # noqa: E731 - spy
    tools, clock = _tools(robot)
    with tools:
        r = tools.call("send_actions", {"actions": [_pos(0.05), _pos(0.1), _pos(0.15)],
                                        "images": False})
    assert r.data["sent"] == 3 and r.data["stopped"] == "done"
    assert r.data["observation"]["joint_pos"]["elbow"] == pytest.approx(0.15)
    # first step at once, then one period (1/125 s) apart; no sleep after the last
    assert clock.sleeps == [0.008, 0.008]


def test_send_actions_by_joint_name():
    tools, _ = _tools()
    with tools:
        names = [j["name"] for j in tools.call("describe_robot").data["joints"]]
        r = tools.call("send_actions", {"actions": [{"mode": "position", "joint_target":
                                                     {n: 0.1 for n in names}}], "images": False})
    assert r.data["sent"] == 1


def test_send_actions_limits():
    tools, _ = _tools(max_steps=2)
    with tools:
        with pytest.raises(ToolError, match="1 to 2 actions"):
            tools.call("send_actions", {"actions": [_pos(0.0)] * 3})
        with pytest.raises(ToolError, match=r"actions\[0\]: unknown field"):
            tools.call("send_actions", {"actions": [{"mode": "position", "speed": 1}]})
        with pytest.raises(ToolError, match="mode 'fly'"):
            tools.call("send_actions", {"actions": [{"mode": "fly"}]})


def test_refused_step_stops_the_chunk():
    tools, _ = _tools()
    with tools:
        r = tools.call("send_actions", {"actions": [_pos(0.1), _pos(0.9), _pos(0.1)],
                                        "images": False})
    assert r.data["sent"] == 1 and r.data["stopped"] == "refused"
    assert "max_joint_step" in r.data["error"]


def test_envelope_narrows_the_lease():
    tools, _ = _tools(envelope={"max_joint_step": 0.05})
    with tools:
        r = tools.call("send_actions", {"actions": [_pos(0.1)], "images": False})
    assert r.data["sent"] == 0 and "envelope" in r.data["error"]


def test_stop_interrupts_a_running_chunk():
    started, release = threading.Event(), threading.Event()
    tools, clock = _tools(max_steps=50)

    def slow_sleep(dt):
        started.set()
        release.wait(5)
        clock.t += dt
    tools._sleep = slow_sleep
    with tools:
        out = {}
        t = threading.Thread(target=lambda: out.update(
            r=tools.call("send_actions", {"actions": [_pos(0.01 * i) for i in range(20)],
                                          "images": False})))
        t.start()
        assert started.wait(5)
        stopper = threading.Thread(target=lambda: out.update(s=tools.call("stop")))
        stopper.start()
        release.set()
        t.join(5)
        stopper.join(5)
    assert out["r"].data["stopped"] == "stop" and out["r"].data["sent"] < 20
    assert out["s"].data == {"stopped": True}


def test_hold_and_stop():
    tools, _ = _tools()
    with tools:
        assert tools.call("hold").data == {"held": True}
        assert tools.call("stop").data == {"stopped": True}
        assert tools.call("get_robot_status").data["mcp_lease"] == "held"


def test_preempted_lease_is_reported():
    from hirobot import Priority

    robot = _robot()
    tools, _ = _tools(robot)
    with tools:
        tools.call("hold")
        robot.claim("operator", priority=Priority.TAKEOVER)
        with pytest.raises(ToolError, match="preempted by 'operator'"):
            tools.call("send_actions", {"actions": [_pos(0.0)], "images": False})


def test_all_robot_calls_on_one_thread():
    robot = _robot()
    seen = set()
    for name in ("get_observation", "send_action", "hold", "reset", "get_status"):
        orig = getattr(robot, name)

        def spy(*a, _orig=orig, **k):
            seen.add(threading.get_ident())
            return _orig(*a, **k)
        setattr(robot, name, spy)
    tools, _ = _tools(robot)
    with tools:
        results = []
        threads = [threading.Thread(target=lambda n=n, a=args: results.append(tools.call(n, a)))
                   for n, args in [("get_observation", {"images": False}), ("hold", {}),
                                   ("get_robot_status", {}), ("reset", {"images": False})] * 3]
        for t in threads:
            t.start()
        for t in threads:
            t.join(5)
    assert len(results) == 12
    assert seen == {tools.thread_ident} and tools.thread_ident != threading.get_ident()


def test_close_releases_the_lease():
    robot = _robot()
    tools, _ = _tools(robot)
    tools.call("hold")
    tools.close()
    robot.claim("next")  # free again
    with pytest.raises(ToolError, match="closed"):
        tools.call("hold")


# ------------------------------------------------------- skill, episode


def test_skills_and_reset():
    walker, vendor = _stack()
    tools, _ = _tools(walker)
    with tools:
        names = [s["name"] for s in tools.call("list_skills").data["skills"]]
        assert names == ["walk", "wave", "sit"]
        tools.call("run_skill", {"name": "sit"})
        assert vendor.ran == [("sit", {})]
        tools.call("run_skill", {"name": "wave"})
        assert tools.call("get_robot_status").data["skill_running"] is True
        tools.call("stop_skill")
        assert tools.call("get_robot_status").data["skill_running"] is False
        with pytest.raises(ToolError, match="unknown skill"):
            tools.call("run_skill", {"name": "fly"})
        r = tools.call("reset", {"seed": 3, "images": False})
        assert r.data["reset"] is True and "observation" in r.data


# ------------------------------------------------------------- convert


def test_action_from_json_partial_joints_and_parts():
    names = ["a", "b", "c"]
    a = action_from_json({"mode": "base_velocity", "base_velocity": [0.1, 0, 0],
                          "joint_target": {"c": 0.5}}, names)
    assert a.joint_names == ["c"] and a.joint_target.tolist() == [0.5]
    full = action_from_json({"mode": "position", "joint_target": {"b": 2, "a": 1, "c": 3}}, names)
    assert full.joint_names is None and full.joint_target.tolist() == [1, 2, 3]
    with pytest.raises(ValueError, match="unknown joint"):
        action_from_json({"joint_target": {"z": 1}}, names)
    with pytest.raises(ValueError, match="finite"):
        action_from_json({"joint_target": [float("nan")] * 3}, names)
    p = action_from_json({"parts": {"left": {"mode": "position", "joint_target": [0.0]}}}, names)
    assert p.parts["left"].joint_target.tolist() == [0.0]


def test_jsonable_and_encode_image():
    pytest.importorskip("PIL")  # JPEG encoding: the mcp extra
    assert jsonable({"x": np.array([1.0, np.nan]), "n": np.int64(2)}) == {"x": [1.0, None], "n": 2}
    jpeg, size = encode_image(np.zeros((100, 200, 3), np.uint8), 50, 80)
    assert size == (50, 25) and jpeg[:2] == b"\xff\xd8"
    jpeg, size = encode_image(np.zeros((10, 20), np.uint8), 512, 80)
    assert size == (20, 10)
    with pytest.raises(ValueError, match="uint8"):
        encode_image(np.zeros((4, 4, 3), np.float32), 512, 80)
