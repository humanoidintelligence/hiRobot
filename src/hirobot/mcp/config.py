"""The ``mcp:`` section of a robot config: which tools to serve, and their limits.

Only the root config may have it, like ``arbiter:``. Example::

    mcp:
      tools: [observe, act]      # categories; default [observe]
      max_steps: 10              # send_actions: most steps per call (1-50, default 5)
      image_max_side: 512        # longest side of a returned image, px (default 512)
      image_quality: 80          # JPEG quality 1-100 (default 80)
      envelope: {max_joint_step: 0.05}   # the MCP lease's limits (see Robot.claim)

:func:`hirobot.mcp.serve_mcp` takes the same names as keyword arguments, which
win over the config.

This module imports neither the ``mcp`` SDK nor Pillow.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Dict, Mapping, Optional, Tuple

#: Tool categories, in the order they are listed.
CATEGORIES: Tuple[str, ...] = ("observe", "act", "skill", "episode")

#: The most steps one ``send_actions`` call may carry, whatever the config says.
MAX_STEPS_LIMIT = 50

_KEYS = ("tools", "max_steps", "image_max_side", "image_quality", "envelope")


@dataclass(frozen=True)
class McpSettings:
    """What :class:`~hirobot.mcp.tools.RobotTools` serves and how (see the module doc)."""

    tools: Tuple[str, ...] = ("observe",)
    max_steps: int = 5
    image_max_side: int = 512
    image_quality: int = 80
    envelope: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        tools = self.tools
        if isinstance(tools, str) or not isinstance(tools, (list, tuple)):
            raise ValueError(f"mcp: tools: must be a list of categories {list(CATEGORIES)}, "
                             f"got {tools!r}")
        unknown = [t for t in tools if t not in CATEGORIES]
        if unknown:
            raise ValueError(f"mcp: tools: unknown categor(ies) {unknown}; "
                             f"they are {list(CATEGORIES)}")
        # Keep the canonical order and drop repeats.
        object.__setattr__(self, "tools", tuple(c for c in CATEGORIES if c in tools))
        _int_in(self.max_steps, "max_steps", 1, MAX_STEPS_LIMIT)
        _int_in(self.image_max_side, "image_max_side", 16, 8192)
        _int_in(self.image_quality, "image_quality", 1, 100)
        if not isinstance(self.envelope, Mapping):
            raise ValueError(f"mcp: envelope: must be a mapping, got {self.envelope!r}")
        from hirobot.control.safety import ENVELOPE_KEYS

        bad = sorted(set(self.envelope) - set(ENVELOPE_KEYS))
        if bad:
            raise ValueError(f"mcp: envelope: unknown key(s) {bad}; it takes {list(ENVELOPE_KEYS)}")
        object.__setattr__(self, "envelope", dict(self.envelope))

    def merged(self, **overrides: Any) -> "McpSettings":
        """These settings with the given (non-``None``) keys replaced."""
        given = {k: v for k, v in overrides.items() if v is not None}
        unknown = sorted(set(given) - set(_KEYS))
        if unknown:
            raise TypeError(f"unknown MCP setting(s) {unknown}; they are {list(_KEYS)}")
        if "tools" in given and isinstance(given["tools"], str):
            given["tools"] = (given["tools"],)
        return replace(self, **given)


def parse_mcp_config(data: Optional[Mapping[str, Any]]) -> McpSettings:
    """The validated ``mcp:`` section; ``None`` or ``{}`` gives the defaults."""
    if data is None:
        return McpSettings()
    if not isinstance(data, Mapping):
        raise ValueError(f"mcp: must be a mapping, got {data!r}")
    unknown = sorted(set(data) - set(_KEYS))
    if unknown:
        raise ValueError(f"mcp: unknown key(s) {unknown}; it takes {list(_KEYS)}")
    kwargs = dict(data)
    if "tools" in kwargs and isinstance(kwargs["tools"], list):
        kwargs["tools"] = tuple(kwargs["tools"])
    return McpSettings(**kwargs)


def _int_in(value: Any, name: str, lo: int, hi: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not lo <= value <= hi:
        raise ValueError(f"mcp: {name}: must be an integer from {lo} to {hi}, got {value!r}")
