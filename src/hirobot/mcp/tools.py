"""The robot tools behind the MCP server, independent of the ``mcp`` SDK.

:class:`RobotTools` wraps one :class:`~hirobot.core.robot.Robot`. Every call
into the robot runs on one thread the tools own (some simulators only work on
the thread that created them); :meth:`RobotTools.call` is safe from any thread
or event loop. ``stop`` is the exception: it does not queue behind a running
call. It bumps a counter at once; a ``send_actions`` submitted before it (running
or still queued) sees the change before its next step and stops. Then the stop
itself is queued.

A tool returns a :class:`ToolResult`: JSON-able ``data`` and zero or more
JPEG images. :mod:`hirobot.mcp.server` turns that into MCP content.
"""

from __future__ import annotations

import concurrent.futures
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from hirobot.mcp.config import McpSettings, parse_mcp_config
from hirobot.mcp.convert import action_from_json, encode_image, jsonable, observation_dict

logger = logging.getLogger(__name__)

#: The name of the command lease the tools take.
LEASE_NAME = "mcp"


class ToolError(Exception):
    """A tool call the caller got wrong or the robot refused; the message goes back to the caller."""


@dataclass
class Image:
    """One encoded image in a tool result."""

    camera: str
    jpeg: bytes
    width: int
    height: int


@dataclass
class ToolResult:
    data: Dict[str, Any]
    images: List[Image] = field(default_factory=list)


@dataclass(frozen=True)
class ToolSpec:
    """One tool: its name, category, description and JSON Schema for its arguments."""

    name: str
    category: str
    description: str
    schema: Dict[str, Any]


_NO_ARGS: Dict[str, Any] = {"type": "object", "properties": {}, "additionalProperties": False}

_IMAGES_ARGS = {
    "images": {"type": "boolean", "default": True,
               "description": "Include camera images (JPEG). False returns only the state."},
    "cameras": {"type": "array", "items": {"type": "string"},
                "description": "Cameras to include; default all. See describe_robot."},
}

_ACTION_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "description": (
        "One robot command. mode is one of the robot's action_modes (describe_robot). "
        "Joint modes (position, velocity, torque, pd) take joint_target: a list with one value "
        "per joint in describe_robot order, or {joint name: value}. cartesian takes "
        "ee_targets {end effector: [x, y, z, qx, qy, qz, qw]}; cartesian_delta takes "
        "[dx, dy, dz, dax, day, daz]. gripper is {end effector: opening 0 (closed) .. 1 (open)}. "
        "base_velocity is [vx, vy, wz] (m/s, rad/s); base_target [x, y, yaw]. "
        "A robot with parts may take parts {part: action} instead."),
    "properties": {
        "mode": {"type": "string"},
        "joint_target": {"type": ["array", "object"]},
        "kp": {"type": ["array", "object"]},
        "kd": {"type": ["array", "object"]},
        "joint_names": {"type": "array", "items": {"type": "string"}},
        "ee_targets": {"type": "object"},
        "gripper": {"type": "object", "additionalProperties": {"type": "number"}},
        "base_velocity": {"type": "array", "items": {"type": "number"}},
        "base_target": {"type": "array", "items": {"type": "number"}},
        "latent": {"type": "array", "items": {"type": "number"}},
        "parts": {"type": "object"},
    },
    "additionalProperties": False,
}


def tool_specs(settings: McpSettings) -> List[ToolSpec]:
    """The tools ``settings`` enables, in category order."""
    n = settings.max_steps
    every = [
        ToolSpec("describe_robot", "observe",
                 "What the robot is: name, kind, control rate, joints (names and limits, in the "
                 "order joint lists use), end effectors, cameras with intrinsics and mount, "
                 "action modes, and which tools are on. Call this first.", _NO_ARGS),
        ToolSpec("get_observation", "observe",
                 "The robot's current state (joint positions and velocities by name, "
                 "end-effector poses, gripper openings, base state) and camera images.",
                 {"type": "object", "properties": dict(_IMAGES_ARGS), "additionalProperties": False}),
        ToolSpec("get_robot_status", "observe",
                 "Health and progress: fault, battery, temperatures, safety-gate counts, the "
                 "command in progress, whether a skill is running.", _NO_ARGS),
        ToolSpec("send_actions", "act",
                 f"Send 1 to {n} actions, one per control tick at the robot's control rate, then "
                 "hold still. Returns how many were sent, why it stopped, and the new observation. "
                 "Each action goes through the robot's safety checks; a refused one stops the "
                 "chunk. Nothing runs after this returns.",
                 {"type": "object", "properties": {
                     "actions": {"type": "array", "items": _ACTION_SCHEMA, "minItems": 1,
                                 "maxItems": n},
                     **_IMAGES_ARGS},
                  "required": ["actions"], "additionalProperties": False}),
        ToolSpec("hold", "act", "Hold the robot still where it is.", _NO_ARGS),
        ToolSpec("stop", "act",
                 "Stop now: interrupt a send_actions in progress and any running skill, then hold "
                 "still. This is not an emergency stop.", _NO_ARGS),
        ToolSpec("list_skills", "skill",
                 "Skills the robot can run, each with a description and an argument schema.",
                 _NO_ARGS),
        ToolSpec("run_skill", "skill",
                 "Start a skill from list_skills. Returns once it has started; poll "
                 "get_robot_status for skill_running, or call stop_skill.",
                 {"type": "object", "properties": {
                     "name": {"type": "string"},
                     "args": {"type": "object", "description": "The skill's arguments."}},
                  "required": ["name"], "additionalProperties": False}),
        ToolSpec("stop_skill", "skill", "Stop the running skill and hold still.", _NO_ARGS),
        ToolSpec("reset", "episode",
                 "Reset the robot to its initial state (a simulator starts a new episode). "
                 "Returns the new observation.",
                 {"type": "object", "properties": {
                     "seed": {"type": "integer", "description": "Scene seed, for simulators."},
                     **_IMAGES_ARGS},
                  "additionalProperties": False}),
    ]
    return [t for t in every if t.category in settings.tools]


class RobotTools:
    """The MCP tools of one robot (see the module doc).

    Args:
        robot: A connected robot, usually from :func:`hirobot.make_robot`.
        settings: What to serve; default: the robot's ``mcp:`` config section.
        sleep: Waits between ``send_actions`` steps (tests replace it).
        clock: ``time.monotonic`` by default.
    """

    def __init__(self, robot: Any, settings: Optional[McpSettings] = None, *,
                 sleep: Callable[[float], None] = time.sleep,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.robot = robot
        self.settings = settings if settings is not None else parse_mcp_config(
            getattr(robot, "mcp_config", None))
        self.specs = tool_specs(self.settings)
        self._by_name = {t.name: t for t in self.specs}
        self._sleep = sleep
        self._clock = clock
        self._stops = 0  # bumped by every stop(); read without a lock (an int, one writer path)
        self._stops_lock = threading.Lock()
        self._executor = concurrent.futures.ThreadPoolExecutor(1, thread_name_prefix="hirobot-mcp")
        self._lease: Any = None
        self._closed = False
        emb = robot.embodiment
        self._joint_names = [j.name for j in emb.joints]
        self._period = 1.0 / emb.control_hz if emb.control_hz else 0.0

    # ------------------------------------------------------------ calling

    @property
    def thread_ident(self) -> Optional[int]:
        """The robot thread's ident, once it has run a call."""
        return getattr(self, "_ident", None)

    def call(self, name: str, args: Optional[Mapping[str, Any]] = None) -> ToolResult:
        """Run tool ``name`` with ``args`` (a JSON object) and wait for it.

        Raises :class:`ToolError` for an unknown or disabled tool, bad arguments,
        or a command the robot refused.
        """
        if self._closed:
            raise ToolError("the robot tools are closed")
        spec = self._by_name.get(name)
        if spec is None:
            off = [t.name for t in tool_specs(McpSettings(tools=tuple(_all_categories())))
                   if t.name == name]
            raise ToolError(f"tool {name!r} is not enabled (mcp: tools: {list(self.settings.tools)})"
                            if off else f"unknown tool {name!r}")
        args = dict(args or {})
        _check_args(spec, args)
        if name == "stop":
            with self._stops_lock:
                self._stops += 1  # seen by any send_actions submitted before this
        elif name == "send_actions":
            args["_stops_at_submit"] = self._stops
        return self._executor.submit(self._run, name, args).result()

    def close(self) -> None:
        """Release the lease and stop the robot thread. Does not disconnect the robot."""
        if self._closed:
            return
        self._closed = True

        def release() -> None:
            if self._lease is not None:
                self._lease.release()
                self._lease = None
        try:
            self._executor.submit(release).result(timeout=5.0)
        except Exception:  # noqa: BLE001 - best effort on shutdown
            logger.exception("releasing the MCP lease failed")
        self._executor.shutdown(wait=False)

    def __enter__(self) -> "RobotTools":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def _run(self, name: str, args: Dict[str, Any]) -> ToolResult:
        self._ident = threading.get_ident()
        from hirobot.control.arbiter import RobotBusyError
        from hirobot.control.safety import SafetyError
        from hirobot.core.robot import RobotFaultError
        from hirobot.core.robot_skills import RobotSkillError

        try:
            return getattr(self, "_tool_" + name)(**args)
        except ToolError:
            raise
        except (RobotBusyError, RobotFaultError, RobotSkillError, SafetyError, ValueError) as exc:
            raise ToolError(f"{type(exc).__name__}: {exc}") from exc

    # --------------------------------------------------------------- lease

    def _commander(self) -> Any:
        """Who commands: the MCP lease (taken on first use) when action tools are on."""
        if self._lease is not None and self._lease.active:
            return self._lease
        if self._lease is not None and self._lease.preempted:
            raise ToolError(f"the MCP lease was preempted by {self._lease.preempted_by!r}; "
                            "the robot is someone else's now")
        self._lease = self.robot.claim(LEASE_NAME, envelope=self.settings.envelope or None)
        return self._lease

    # --------------------------------------------------------------- observe

    def _tool_describe_robot(self) -> ToolResult:
        robot, emb = self.robot, self.robot.embodiment
        cameras = []
        for cam in emb.cameras:
            entry: Dict[str, Any] = {"name": cam.name, "width": cam.width, "height": cam.height,
                                     "depth": cam.depth}
            try:
                info = robot.camera_info(cam.name)
                entry["intrinsics"] = info.intrinsics
                entry["intrinsics_source"] = info.intrinsics_source
                entry["mount"] = info.mount
            except Exception as exc:  # noqa: BLE001 - describe what we can
                entry["camera_info_error"] = str(exc)
            cameras.append(entry)
        data = {
            "name": emb.name, "kind": emb.kind, "control_hz": emb.control_hz,
            "floating_base": emb.floating_base,
            "joints": [{"name": j.name, "position_limit": j.position_limit,
                        "torque_limit": j.torque_limit} for j in emb.joints],
            "end_effectors": [{"name": e.name, "has_gripper": e.has_gripper} for e in emb.end_effectors],
            "cameras": cameras,
            "action_modes": [m.value for m in robot.action_modes],
            "direct_joints": list(robot.direct_joints),
            "parts": sorted(p for p in (emb.parts or {}) if p),
            "tools": [t.name for t in self.specs],
            "max_steps": self.settings.max_steps,
        }
        return ToolResult(jsonable(data))

    def _tool_get_observation(self, images: bool = True,
                              cameras: Optional[Sequence[str]] = None) -> ToolResult:
        return self._observation({}, images, cameras)

    def _tool_get_robot_status(self) -> ToolResult:
        status = jsonable(self.robot.get_status())
        status["skill_running"] = bool(self.robot.robot_skill_running)
        lease = self._lease
        status["mcp_lease"] = ("held" if lease is not None and lease.active else
                               "preempted" if lease is not None and lease.preempted else "none")
        return ToolResult(status)

    def _observation(self, data: Dict[str, Any], images: bool,
                     cameras: Optional[Sequence[str]]) -> ToolResult:
        obs = self.robot.get_observation()
        out = dict(data)
        out["observation"] = observation_dict(obs, self._joint_names)
        for key in ("success", "done", "reward"):
            val = getattr(self.robot, key, None)
            if val is not None:
                out[key] = jsonable(val)
        result = ToolResult(out)
        if images:
            names = list(obs.images) if cameras is None else list(cameras)
            missing = [c for c in names if c not in obs.images]
            if missing:
                out["missing_images"] = missing
            for cam in names:
                if cam in obs.images:
                    jpeg, (w, h) = encode_image(obs.images[cam], self.settings.image_max_side,
                                                self.settings.image_quality)
                    result.images.append(Image(cam, jpeg, w, h))
            out["images"] = [{"camera": i.camera, "width": i.width, "height": i.height}
                             for i in result.images]
        return result

    # ------------------------------------------------------------------ act

    def _tool_send_actions(self, actions: Sequence[Any], images: bool = True,
                           cameras: Optional[Sequence[str]] = None,
                           _stops_at_submit: int = 0) -> ToolResult:
        if not 1 <= len(actions) <= self.settings.max_steps:
            raise ToolError(f"actions: give 1 to {self.settings.max_steps} actions, got {len(actions)}")
        try:
            parsed = [action_from_json(a, self._joint_names, f"actions[{i}]")
                      for i, a in enumerate(actions)]
        except ValueError as exc:
            raise ToolError(str(exc)) from None
        def stop_asked() -> bool:
            return self._stops != _stops_at_submit

        if stop_asked():
            return self._observation({"sent": 0, "of": len(parsed), "stopped": "stop"},
                                     images, cameras)
        commander = self._commander()
        sent, stopped, error = 0, "done", None
        next_t = self._clock()
        try:
            for action in parsed:
                if stop_asked():
                    stopped = "stop"
                    break
                wait = next_t - self._clock()
                if wait > 0:
                    self._sleep(wait)
                if stop_asked():
                    stopped = "stop"
                    break
                try:
                    commander.send_action(action)
                except Exception as exc:  # noqa: BLE001 - reported, then hold
                    stopped, error = "refused", f"{type(exc).__name__}: {exc}"
                    break
                sent += 1
                next_t += self._period
        finally:
            try:
                commander.hold()
            except Exception as exc:  # noqa: BLE001
                logger.warning("hold after send_actions failed: %s", exc)
                if error is None:
                    stopped, error = "refused", f"hold failed: {type(exc).__name__}: {exc}"
        data: Dict[str, Any] = {"sent": sent, "of": len(parsed), "stopped": stopped}
        if error is not None:
            data["error"] = error
        return self._observation(data, images, cameras)

    def _tool_hold(self) -> ToolResult:
        self._commander().hold()
        return ToolResult({"held": True})

    def _tool_stop(self) -> ToolResult:
        # call() bumped the counter; a send_actions submitted before us has stopped.
        errors = []
        if "skill" in self.settings.tools or self.robot.robot_skill_running:
            try:
                self.robot.stop_robot_skill()
            except Exception as exc:  # noqa: BLE001
                errors.append(f"stop_skill: {exc}")
        try:
            self._commander().hold()
        except Exception as exc:  # noqa: BLE001
            errors.append(f"hold: {exc}")
        if errors:
            raise ToolError("; ".join(errors))
        return ToolResult({"stopped": True})

    # ---------------------------------------------------------------- skill

    def _tool_list_skills(self) -> ToolResult:
        return ToolResult({"skills": [jsonable(s) for s in self.robot.list_robot_skills()]})

    def _tool_run_skill(self, name: str, args: Optional[Mapping[str, Any]] = None) -> ToolResult:
        result = self._commander().run_robot_skill(name, dict(args or {}))
        if self._lease is not None and self._lease.preempted:
            # A layer skill takes the stream from our own lease; it is ours, so the
            # next command claims again (and is told the robot is busy while it runs).
            self._lease = None
        return ToolResult({"started": jsonable(result)})

    def _tool_stop_skill(self) -> ToolResult:
        self.robot.stop_robot_skill()
        return ToolResult({"stopped": True})

    # -------------------------------------------------------------- episode

    def _tool_reset(self, seed: Optional[int] = None, images: bool = True,
                    cameras: Optional[Sequence[str]] = None) -> ToolResult:
        self._commander().reset(seed=seed)
        return self._observation({"reset": True}, images, cameras)


def _all_categories() -> Tuple[str, ...]:
    from hirobot.mcp.config import CATEGORIES

    return CATEGORIES


def _check_args(spec: ToolSpec, args: Mapping[str, Any]) -> None:
    """The argument names and top-level types a tool takes (the robot checks the rest)."""
    props = spec.schema.get("properties", {})
    unknown = sorted(set(args) - set(props))
    if unknown:
        raise ToolError(f"{spec.name}: unknown argument(s) {unknown}; it takes {sorted(props)}")
    missing = [k for k in spec.schema.get("required", []) if k not in args]
    if missing:
        raise ToolError(f"{spec.name}: missing argument(s) {missing}")
    py = {"boolean": bool, "integer": int, "string": str, "array": (list, tuple), "object": Mapping}
    for key, val in args.items():
        want = props[key].get("type")
        if val is None or want is None:
            continue
        kinds = want if isinstance(want, list) else [want]
        ok = any(isinstance(val, py[k]) and not (k == "integer" and isinstance(val, bool))
                 for k in kinds if k in py)
        if not ok:
            raise ToolError(f"{spec.name}: {key} must be {' or '.join(kinds)}, "
                            f"got {type(val).__name__}")
