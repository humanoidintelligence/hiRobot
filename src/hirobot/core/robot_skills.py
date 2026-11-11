"""Robot skills: motions a robot's own controller carries out, started by name.

A robot skill is a behaviour such as walking, sitting down or waving, run by the
robot's controller instead of by streaming joint targets. Each driver declares
the skills hiRobot supports on its robot; the layers above (hiAgent, hiLive)
do not hard-code them: they call
:meth:`~hirobot.core.robot.Robot.list_robot_skills`, show the result to the model, and
start one with :meth:`~hirobot.core.robot.Robot.run_robot_skill`.

Arguments are checked here against the skill's JSON Schema before anything
reaches the robot, the same way :class:`~hirobot.control.safety.SafetyGate` checks
actions, so callers need no checks of their own.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Mapping, Optional

from pydantic import BaseModel, Field


class RobotSkillError(RuntimeError):
    """A robot skill was refused: unknown name, bad arguments, or the robot said no."""


class RobotSkillSpec(BaseModel):
    """What a caller needs to offer a robot skill to a model.

    Attributes:
        name: Identifier passed to ``run_robot_skill``.
        description: One or two sentences for the model.
        parameters: JSON Schema of the arguments (an ``object`` schema).
            Empty means the skill takes none.
        requires: Preconditions the caller should know about, in plain words
            (e.g. ``"robot standing on the ground"``). Informational; the driver
            enforces the ones it can check.
        duration_s: Typical run time, if known.
    """

    name: str
    description: str
    parameters: Dict[str, Any] = Field(default_factory=dict)
    requires: List[str] = Field(default_factory=list)
    duration_s: Optional[float] = None


class RobotSkillResult(BaseModel):
    """The robot accepted a skill.

    ``run_robot_skill`` returns once the robot has accepted the command, not when
    the motion ends; ``duration_s`` is how long it is expected to run, if known.
    """

    name: str
    args: Dict[str, Any] = Field(default_factory=dict)
    duration_s: Optional[float] = None


_TYPES = {
    "number": (int, float),
    "integer": (int,),
    "boolean": (bool,),
    "string": (str,),
}


def validate_args(spec: RobotSkillSpec, args: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    """Check ``args`` against ``spec.parameters`` and fill in defaults.

    Supports the subset of JSON Schema the drivers use: property ``type``
    (number / integer / boolean / string), ``minimum``, ``maximum``,
    ``exclusiveMinimum``, ``enum``, ``default``, plus ``required``. Unknown
    arguments are refused.

    Raises:
        RobotSkillError: If an argument is unknown, missing, of the wrong type,
            or out of range.
    """
    args = dict(args or {})
    props: Dict[str, Any] = spec.parameters.get("properties", {})
    unknown = sorted(set(args) - set(props))
    if unknown:
        raise RobotSkillError(f"{spec.name}: unknown argument(s) {unknown}")
    out: Dict[str, Any] = {}
    for key, rule in props.items():
        if key not in args:
            if key in spec.parameters.get("required", []):
                raise RobotSkillError(f"{spec.name}: missing argument {key!r}")
            if "default" in rule:
                out[key] = rule["default"]
            continue
        value = args[key]
        kind = rule.get("type")
        if kind in _TYPES:
            ok = isinstance(value, _TYPES[kind]) and not (
                kind in ("number", "integer") and isinstance(value, bool))
            if not ok:
                raise RobotSkillError(f"{spec.name}: {key} must be {kind}, got {value!r}")
        if kind in ("number", "integer"):
            if not math.isfinite(value):
                raise RobotSkillError(f"{spec.name}: {key} must be finite, got {value!r}")
            if "minimum" in rule and value < rule["minimum"]:
                raise RobotSkillError(f"{spec.name}: {key}={value} is below {rule['minimum']}")
            if "maximum" in rule and value > rule["maximum"]:
                raise RobotSkillError(f"{spec.name}: {key}={value} is above {rule['maximum']}")
            if "exclusiveMinimum" in rule and value <= rule["exclusiveMinimum"]:
                raise RobotSkillError(
                    f"{spec.name}: {key}={value} must be above {rule['exclusiveMinimum']}")
        if "enum" in rule and value not in rule["enum"]:
            raise RobotSkillError(f"{spec.name}: {key} must be one of {rule['enum']}")
        out[key] = value
    return out
