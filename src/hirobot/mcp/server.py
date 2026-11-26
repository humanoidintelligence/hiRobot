"""The MCP server around :class:`~hirobot.mcp.tools.RobotTools` (needs ``hirobot[mcp]``).

:func:`build_server` returns an ``mcp`` low-level ``Server`` (usable in-process,
e.g. with ``mcp.Client(server)`` in tests); :func:`serve_mcp` runs one over
stdio or streamable HTTP until the client goes away or the process is stopped.
"""

from __future__ import annotations

import base64
import json
import logging
from typing import Any, Optional, Sequence

from hirobot.mcp.config import parse_mcp_config
from hirobot.mcp.tools import RobotTools, ToolError

logger = logging.getLogger(__name__)

#: Tools that only read: hinted read-only to the client.
_READ_ONLY = {"describe_robot", "get_observation", "get_robot_status", "list_skills"}


def _import_mcp() -> Any:
    try:
        import mcp  # noqa: F401
        import mcp_types
        from mcp.server.lowlevel import Server
    except ImportError:
        raise ImportError("the MCP server needs the mcp package: pip install 'hirobot[mcp]'") from None
    return Server, mcp_types


def build_server(tools: RobotTools, name: Optional[str] = None) -> Any:
    """An ``mcp.server.lowlevel.Server`` serving ``tools``.

    A tool call runs on the tools' robot thread (``anyio.to_thread``), so the
    event loop stays free and ``stop`` gets through while ``send_actions`` runs.
    A :class:`~hirobot.mcp.tools.ToolError` comes back as a result with
    ``isError`` set and the message as text.
    """
    Server, types = _import_mcp()
    import anyio

    from hirobot import __version__

    robot_name = tools.robot.embodiment.name
    listed = [types.Tool(
        name=spec.name, description=spec.description, input_schema=spec.schema,
        annotations=types.ToolAnnotations(read_only_hint=spec.name in _READ_ONLY,
                                          destructive_hint=False))
        for spec in tools.specs]

    async def on_list_tools(ctx: Any, params: Any) -> Any:
        return types.ListToolsResult(tools=listed)

    async def on_call_tool(ctx: Any, params: Any) -> Any:
        try:
            # A thread per call (abandon_on_cancel: a cancelled request leaves the
            # robot call to finish); RobotTools queues it on its robot thread.
            result = await anyio.to_thread.run_sync(
                tools.call, params.name, dict(params.arguments or {}), abandon_on_cancel=True)
        except ToolError as exc:
            return types.CallToolResult(content=[types.TextContent(text=str(exc))], is_error=True)
        content: list = [types.TextContent(text=json.dumps(result.data, separators=(",", ":")))]
        for img in result.images:
            content.append(types.TextContent(text=f"camera {img.camera} ({img.width}x{img.height}):"))
            content.append(types.ImageContent(data=base64.b64encode(img.jpeg).decode("ascii"),
                                              mime_type="image/jpeg"))
        return types.CallToolResult(content=content, structured_content=result.data)

    return Server(
        name or f"hirobot-{robot_name}", version=__version__,
        instructions=(f"Tools for the robot {robot_name!r}. Call describe_robot first: it lists the "
                      "joints (the order joint lists use), cameras, action modes and limits. "
                      "Commands go through the robot's safety checks; stop interrupts motion."),
        on_list_tools=on_list_tools, on_call_tool=on_call_tool)


def serve_mcp(robot: Any, *, transport: str = "stdio", host: str = "127.0.0.1", port: int = 8000,
              path: str = "/mcp", name: Optional[str] = None,
              tools: Optional[Sequence[str]] = None, max_steps: Optional[int] = None,
              image_max_side: Optional[int] = None, image_quality: Optional[int] = None,
              envelope: Optional[dict] = None) -> None:
    """Serve ``robot`` as an MCP server until the client disconnects (stdio) or the process stops.

    Args:
        robot: A connected robot (from :func:`hirobot.make_robot`, or a hiBench referee).
        transport: ``"stdio"`` or ``"http"`` (streamable HTTP at ``http://host:port/path``).
        host, port, path: Where the HTTP transport listens. The default host is
            local only; another host exposes robot control to that network.
        name: The server name the client sees; default ``hirobot-<robot name>``.
        tools, max_steps, image_max_side, image_quality, envelope: Override the
            robot's ``mcp:`` config section (see :mod:`hirobot.mcp.config`).

    The robot is not disconnected on return; the MCP lease is released.
    """
    settings = parse_mcp_config(getattr(robot, "mcp_config", None)).merged(
        tools=tools, max_steps=max_steps, image_max_side=image_max_side,
        image_quality=image_quality, envelope=envelope)
    if transport not in ("stdio", "http"):
        raise ValueError(f"transport must be 'stdio' or 'http', got {transport!r}")
    with RobotTools(robot, settings) as robot_tools:
        server = build_server(robot_tools, name)
        import anyio

        anyio.run(_serve_stdio if transport == "stdio" else _serve_http, server,
                  *(() if transport == "stdio" else (host, port, path)))


async def _serve_stdio(server: Any) -> None:
    from mcp.server.stdio import stdio_server

    async with stdio_server() as (read, write):
        await server.run(read, write, server.create_initialization_options())


async def _serve_http(server: Any, host: str, port: int, path: str) -> None:
    import uvicorn

    app = server.streamable_http_app(streamable_http_path=path, host=host)
    if host not in ("127.0.0.1", "localhost", "::1"):
        logger.warning("MCP server on %s:%d: anyone who can reach it can command the robot",
                       host, port)
    await uvicorn.Server(uvicorn.Config(app, host=host, port=port, log_level="info")).serve()

