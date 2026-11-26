"""Expose a :class:`~hirobot.core.robot.Robot` as an MCP server (extra ``hirobot[mcp]``).

See ``docs/api.md`` "MCP server". :mod:`hirobot.mcp.config` reads the
config's ``mcp:`` section. Nothing here imports the ``mcp`` SDK or Pillow at
import time.
"""

from typing import Any

from hirobot.mcp.config import CATEGORIES, MAX_STEPS_LIMIT, McpSettings, parse_mcp_config
from hirobot.mcp.tools import RobotTools, ToolError, ToolResult, ToolSpec, tool_specs

__all__ = ["CATEGORIES", "MAX_STEPS_LIMIT", "McpSettings", "RobotTools", "ToolError", "ToolResult",
           "ToolSpec", "build_server", "parse_mcp_config", "serve_mcp", "tool_specs"]


def serve_mcp(robot: Any, **kwargs: Any) -> None:
    """Serve ``robot`` as an MCP server; see :func:`hirobot.mcp.server.serve_mcp`."""
    from hirobot.mcp.server import serve_mcp as serve

    serve(robot, **kwargs)


def build_server(tools: RobotTools, name: Any = None) -> Any:
    """An ``mcp`` low-level server for ``tools``; see :func:`hirobot.mcp.server.build_server`."""
    from hirobot.mcp.server import build_server as build

    return build(tools, name)
