"""Drive a robot over MCP from Python, with no LLM: the calls an agent would make.

Starts ``hirobot mcp <config>`` over stdio, lists the tools, describes the
robot, reads one observation, bends the elbow by 0.04 rad with one
``send_actions`` call, then holds. Needs the ``mcp`` extra (Python 3.10+).
Run it from the repository root:

    python examples/mcp/client.py
    python examples/mcp/client.py path/to/config.yaml --command hirobot

The config must turn the action tools on (``mcp: tools: [observe, act]``), as
``examples/mcp/ur5e_sim.yaml`` does, and the robot must have a joint ``elbow``.
"""

from __future__ import annotations

import argparse
import asyncio
import os

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


async def call(session, name, args=None):
    """Call one tool; raise if the server says the call failed."""
    result = await session.call_tool(name, args or {})
    if result.is_error:
        raise RuntimeError(f"{name}: {result.content[0].text}")
    return result.structured_content


async def run(config, command):
    # Pass this environment on, so the server finds the same hirobot install.
    server = StdioServerParameters(command=command, args=["mcp", config], env=dict(os.environ))
    async with stdio_client(server) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            tools = await session.list_tools()
            print("tools:", ", ".join(t.name for t in tools.tools))

            robot = await call(session, "describe_robot")
            print(f"robot: {robot['name']} ({robot['kind']}), {len(robot['joints'])} joints, "
                  f"{robot['control_hz']} Hz")

            obs = await call(session, "get_observation", {"images": False})
            q = obs["observation"]["joint_pos"]           # {joint name: position}
            print("elbow before:", q["elbow"])

            # In position mode the target names every joint: copy the pose, change one.
            target = dict(q, elbow=q["elbow"] + 0.04)
            out = await call(session, "send_actions", {
                "actions": [{"mode": "position", "joint_target": target}],
                "images": False,
            })
            print(f"sent {out['sent']} of {out['of']}, stopped: {out['stopped']}, "
                  f"error: {out.get('error')}")
            print("elbow after:", out["observation"]["joint_pos"]["elbow"])

            await call(session, "hold")
            print("holding")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("config", nargs="?", default="examples/mcp/ur5e_sim.yaml",
                    help="config to serve (a built-in name or a path)")
    ap.add_argument("--command", default="hirobot", help="the hirobot executable to start")
    args = ap.parse_args(argv)
    asyncio.run(run(args.config, args.command))


if __name__ == "__main__":
    main()
