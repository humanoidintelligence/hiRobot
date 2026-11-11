"""Layers on one part of an assembly (``layers: [{type: ..., part: <path>}]``).

A layer with ``part:`` manages one part of a robot built from an ``assembly:``
config: a whole-body controller on the G1 of a G1 + arm cell, say. It sits
outside the tree like every layer, but sees only its part:

* Its ``inner`` is a :class:`PartPort`: the part's own names (``left_hip_pitch``,
  not ``cell/g1/left_hip_pitch``), embodiment, capabilities and observations.
  ``PartPort.send_action`` does not send to the part directly: it sends the
  action as the part's piece of a root action (``Action.parts``), through the
  safety gate around the root.
* :class:`PartLayers` sits between the whole-tree layers and that gate. What a
  caller sends to a part with a layer (fields with its prefix in a flat action,
  or its entry in ``Action.parts``) goes to the layer, in the part's names and
  frame; the rest goes down unchanged. A passive layer's output in that call is
  joined with the rest into one action, so the gate checks both together; a
  layer with its own thread sends its part alone, and the other parts keep
  their last targets.
* The root's ``action_modes``, ``direct_joints`` and ``cartesian_targets`` take
  the layer's for its part.
* ``hold()`` holds each part with a layer through the layer (a humanoid keeps
  balancing) and the rest directly. An e-stop stops the hardware first, then
  each layer (:meth:`hirobot.control.layer.Layer.estop`). A layer that enters the fault
  state (its thread failed) stops the whole tree: the other parts hold and the
  robot refuses commands until ``reset()``.
* Layer skills are ``<part>/<skill>``; one claims only its part
  (``Priority.LAYER_SKILL``), so the other parts stay commandable.

Two layers may not manage the same part, or a part and one inside it.
"""

from __future__ import annotations

import contextvars
import logging
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence, Tuple

from hirobot.control.layer import Layer
from hirobot.assembly.node import OWN
from hirobot.assembly.parts import SEP, PartView, action_owners, part_covers
from hirobot.core.robot import Robot
from hirobot.core.robot_skills import RobotSkillError, RobotSkillResult, RobotSkillSpec
from hirobot.core.types import Action, Observation, RobotStatus

if TYPE_CHECKING:
    from hirobot.core.embodiment import Embodiment

logger = logging.getLogger(__name__)

#: While :class:`PartLayers` passes a caller's action to the layers: the pieces
#: their ``PartPort.send_action`` calls produced, to send with the rest.
_COLLECTING: contextvars.ContextVar[Optional[Tuple["PartLayers", Dict[str, Action]]]] = \
    contextvars.ContextVar("hirobot_part_layers_collecting", default=None)
#: While :meth:`PartLayers.reset` / ``connect`` / ``disconnect`` run the layers:
#: the root already did (or will do) it for the parts, so the ports skip it.
_ROOT_DOES_IT: contextvars.ContextVar[Optional["PartLayers"]] = \
    contextvars.ContextVar("hirobot_part_layers_root", default=None)


def _join(path: str, name: str) -> str:
    return f"{path}{SEP}{name}" if path else name


def _with_base_frame(piece: Action) -> Action:
    """Mark a leaf already in its part's frame, so the root does not convert it again."""
    if piece.parts or not piece.ee_targets or piece.extra.get("frame") == "base":
        return piece
    return piece.model_copy(update={"extra": {**piece.extra, "frame": "base"}})


def _nest(path: str, action: Action) -> Action:
    """``action`` as the entry at ``path`` of an ``Action.parts`` tree."""
    for name in reversed(path.split(SEP)):
        action = Action(parts={name: action})
    return action


def _merge(a: Optional[Action], b: Optional[Action]) -> Optional[Action]:
    """Two ``Action.parts`` trees with disjoint leaves, as one."""
    if a is None or b is None:
        return a if b is None else b
    parts = dict(a.parts)
    for name, sub in b.parts.items():
        parts[name] = _merge(parts[name], sub) if name in parts and parts[name].parts \
            and sub.parts else sub
    return Action(parts=parts)


class PartPort(PartView, Robot):
    """The ``inner`` of a layer on one part: :class:`~hirobot.assembly.parts.PartView` plus commands.

    Names, embodiment, capabilities and observations are the part's own.
    ``send_action`` sends through the root (its gate checks it); ``hold`` and
    ``reset`` act on this part only. ``connect`` / ``disconnect`` do nothing:
    the root connects every part.
    """

    def __init__(self, path: str, part: Robot, owner: "PartLayers") -> None:
        PartView.__init__(self, path, part)
        self._owner = owner

    @property
    def fault(self) -> Optional[str]:  # type: ignore[override]
        root = self._owner._inner.fault
        return root or self._robot.fault

    @property
    def direct_joints(self):  # type: ignore[override]
        return tuple(self._robot.direct_joints)

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return tuple(self._robot.cartesian_targets)

    @property
    def motion_spaces(self):  # type: ignore[override]
        return tuple(self._robot.motion_spaces)

    @property
    def cameras(self) -> Mapping[str, Any]:  # type: ignore[override]
        return self._robot.cameras

    @property
    def microphones(self) -> Mapping[str, Any]:  # type: ignore[override]
        return self._robot.microphones

    @property
    def speakers(self) -> Mapping[str, Any]:  # type: ignore[override]
        return self._robot.speakers

    def now(self) -> float:
        return self._robot.now()

    def get_observation(self) -> Observation:
        return self._robot.get_observation()

    def get_status(self) -> RobotStatus:
        status = self._robot.get_status()
        status.fault = self.fault
        return status

    def connect(self) -> None:
        """Nothing: the root connects every part."""

    def disconnect(self) -> None:
        """Nothing: the root disconnects every part."""

    def reset(self, seed: Optional[int] = None) -> None:
        if _ROOT_DOES_IT.get() is self._owner:
            return
        self._owner._inner.reset_parts([self.path], seed)

    def hold(self) -> None:
        self._owner._inner.hold_parts([self.path])

    def send_action(self, action: Action) -> None:
        """Send ``action`` (this part's names and frame) as the part's piece of a root action."""
        collecting = _COLLECTING.get()
        if collecting is not None and collecting[0] is self._owner:
            collecting[1][self.path] = action  # sent with the rest of the caller's action
            return
        self._owner._inner.send_action(_nest(self.path, _with_base_frame(action)))

    def _send_action(self, action: Action) -> None:  # pragma: no cover - send_action overrides
        self.send_action(action)

    def set_gains(self, kp: Any, kd: Any) -> None:
        self._robot.set_gains(kp, kd)

    def __repr__(self) -> str:
        return f"PartPort({self.path!r}, {self._robot.embodiment.name!r})"


def part_robot(root: Robot, path: str) -> Robot:
    """The robot of node ``path`` inside ``root`` (an assembly with parts)."""
    robot = root
    for name in path.split(SEP):
        node = getattr(robot, "_node", None)
        if node is None or name not in node.children:
            raise ValueError(f"layers: no part {path!r} to put a layer on")
        robot = node.children[name]
    return robot


def check_layer_parts(embodiment: "Embodiment", paths: Sequence[str]) -> None:
    """Raise ``ValueError`` unless every path is a node and no two overlap."""
    for p in paths:
        if not p:
            raise ValueError("layers: part: '' is the whole robot; leave part: out for that")
        if p not in embodiment.parts:
            nodes = sorted(k for k in embodiment.parts if k)
            raise ValueError(f"layers: part {p!r} is not a part of this robot that is a robot of "
                             f"its own; parts: {nodes}")
    for i, a in enumerate(paths):
        for b in paths[i + 1:]:
            if part_covers((a, False), b) or part_covers((b, False), a):
                what = "the same part" if a == b else "a part and one inside it"
                raise ValueError(f"layers: two layers on {what} ({a!r}, {b!r}); one layer per "
                                 "part, and not on parts inside each other")


class PartLayers(Layer):
    """The layers on parts of the tree, between the whole-tree layers and the gate.

    Args:
        inner: The root (the assembly, with the hardware gate on it).
        specs: ``(part path, layer class, keyword arguments)`` for each layer.
    """

    def __init__(self, inner: Robot, specs: Sequence[Tuple[str, Any, Mapping[str, Any]]]) -> None:
        super().__init__(inner)
        root = self._inner
        check_layer_parts(root.embodiment, [p for p, _, _ in specs])
        self.ports: Dict[str, PartPort] = {}
        self.layers: Dict[str, Layer] = {}
        for path, cls, kwargs in specs:
            port = PartPort(path, part_robot(root, path), self)
            layer = cls(port, **dict(kwargs))
            if not isinstance(layer, Layer):
                raise TypeError(f"layers: {cls.__name__} on part {path!r} is not a Layer")
            layer._skill_parts = [path]
            layer._fault_listener = self._layer_fault
            self.ports[path] = port
            self.layers[path] = layer

    def _part_layers(self) -> List[Layer]:
        return list(self.layers.values())

    # --------------------------------------------------------- capabilities

    def _under(self, prefix: str) -> bool:
        """Whether a layer sits on a part strictly inside ``prefix``."""
        return any(part_covers((prefix, False), p) and p != prefix for p in self.layers)

    def _caps(self, robot: Robot, prefix: str, kind: str) -> List[str]:
        if prefix in self.layers:
            return [_join(prefix, n) for n in getattr(self.layers[prefix], kind)]
        node = getattr(robot, "_node", None)
        if node is None or not self._under(prefix):
            return [_join(prefix, n) for n in getattr(robot, kind)]
        out: List[str] = []
        for m, r in node.robots.items():
            if m == OWN:
                out += [_join(prefix, n) for n in getattr(r, kind)]
            elif not (kind == "cartesian_targets" and node.lost(m)):
                out += self._caps(r, _join(prefix, m), kind)
        return out

    def _modes(self, robot: Robot, prefix: str) -> Tuple[Any, ...]:
        if prefix in self.layers:
            return tuple(self.layers[prefix].action_modes)
        node = getattr(robot, "_node", None)
        if node is None or not self._under(prefix):
            return tuple(robot.action_modes)
        modes: Optional[Tuple[Any, ...]] = None
        for m, r in node.robots.items():
            mine = tuple(r.action_modes) if m == OWN else self._modes(r, _join(prefix, m))
            modes = mine if modes is None else tuple(x for x in modes if x in mine)
        return modes or ()

    @property
    def action_modes(self):  # type: ignore[override]
        return self._modes(self._inner, "")

    @property
    def direct_joints(self):  # type: ignore[override]
        return tuple(self._caps(self._inner, "", "direct_joints"))

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return tuple(self._caps(self._inner, "", "cartesian_targets"))

    @property
    def motion_spaces(self):  # type: ignore[override]
        floating = [p for p, info in self.embodiment.parts.items() if p and info.moving]
        top = [p for p in floating if not any(part_covers((q, False), p) and q != p
                                             for q in floating)]
        if len(top) == 1 and top[0] in self.layers:
            return tuple(self.layers[top[0]].motion_spaces)
        return self.inner.motion_spaces

    @property
    def fault(self):  # type: ignore[override]
        return self._fault or self.inner.fault or next(
            (f"layer on part {p!r}: {layer.fault}" for p, layer in self.layers.items()
             if layer._fault is not None), None)

    # ----------------------------------------------------------------- parts

    accepts_parts = True

    @property
    def parts(self) -> Mapping[str, Robot]:
        return self._inner.parts

    def part(self, path: str) -> PartView:
        return self._inner.part(path)

    # ------------------------------------------------------------- commands

    def _own_key(self, prefix: str) -> Optional[str]:
        """A ``parts:`` key of node ``prefix``'s own devices (``Action.parts`` entry for them)."""
        devices = self.embodiment.parts[prefix].devices
        return devices[0] if devices else None

    def _touches_layers(self, action: Action) -> bool:
        owners = action_owners(self.embodiment, action)
        return any(part_covers((p, False), o) for p in self.layers for o in owners)

    def _route(self, robot: Robot, prefix: str,
               action: Action) -> Tuple[Dict[str, Action], Dict[str, Action]]:
        """Split ``action`` (node ``prefix``'s names) into what goes down and what each layer gets.

        Returns the ``Action.parts`` entries to send to the node (each already
        in its part's frame) and part path -> the piece for the layer on it.
        """
        node = robot._node  # type: ignore[attr-defined]
        rest: Dict[str, Action] = {}
        to_layers: Dict[str, Action] = {}
        for m, piece in node.split(action).items():
            if m == OWN:
                key = self._own_key(prefix)
                if key is None:  # pragma: no cover - a node with an own body has devices
                    raise ValueError(f"node {prefix!r} has no own device to address")
                rest[key] = _with_base_frame(piece)
                continue
            path = _join(prefix, m)
            if path in self.layers:
                to_layers[path] = piece
            elif self._under(path):
                sub_rest, sub_layers = self._route(node.children[m], path, piece)
                if sub_rest:
                    rest[m] = Action(parts=sub_rest)
                to_layers.update(sub_layers)
            else:
                rest[m] = _with_base_frame(piece)
        return rest, to_layers

    def _send_action(self, action: Action) -> None:
        if not self._touches_layers(action):
            self.inner.send_action(action)
            return
        rest, to_layers = self._route(self._inner, "", action)
        produced: Dict[str, Action] = {}
        token = _COLLECTING.set((self, produced))
        try:
            for path, piece in to_layers.items():
                self.layers[path].send_action(piece)
        finally:
            _COLLECTING.reset(token)
        combined = Action(parts=rest) if rest else None
        for path, out in produced.items():
            combined = _merge(combined, _nest(path, _with_base_frame(out)))
        if combined is not None:
            self.inner.send_action(combined)

    # ------------------------------------------------------- hold and reset

    def _expand(self, path: str) -> Tuple[List[str], List[str]]:
        """``(layer parts, other part paths)`` that together make up part ``path``.

        A part with a layer on it or above it is that layer's; a part with
        layers inside is split into its own devices and its children.
        """
        from hirobot.assembly.parts import resolve_part

        node, own_only = resolve_part(self.embodiment, path)
        for p in self.layers:
            if part_covers((p, False), node):
                return [p], []
        if own_only or not self._under(node):
            return [], [path]
        layers: List[str] = []
        direct: List[str] = []
        robot = part_robot(self._inner, node) if node else self._inner
        for m in robot._node.robots:  # type: ignore[attr-defined]
            if m == OWN:
                key = self._own_key(node)
                if key is not None:
                    direct.append(_join(node, key))
                continue
            sub_layers, sub_direct = self._expand(_join(node, m))
            layers += [p for p in sub_layers if p not in layers]
            direct += sub_direct
        return layers, direct

    def hold_parts(self, parts: Any) -> None:
        """Hold ``parts``: those with a layer through the layer (it keeps balancing)."""
        layers: List[str] = []
        direct: List[str] = []
        for raw in ([parts] if isinstance(parts, str) else parts):
            sub_layers, sub_direct = self._expand(str(raw).strip(SEP))
            layers += [p for p in sub_layers if p not in layers]
            direct += sub_direct
        first: Optional[BaseException] = None
        if direct:
            try:
                self._inner.hold_parts(direct)
            except Exception as exc:  # noqa: BLE001 - hold the layers' parts too
                logger.exception("hold on parts %s failed", direct)
                first = exc
        for p in layers:
            try:
                self.layers[p].hold()
            except Exception as exc:  # noqa: BLE001 - hold every part
                logger.exception("hold through the layer on part %r failed", p)
                first = first or exc
        self._forget(parts)
        if first is not None:
            raise first

    def _forget(self, parts: Any) -> None:
        """Drop the top gate's step reference on ``parts``' joints."""
        from hirobot.assembly.parts import scope_joints

        forget = getattr(self.safety, "forget", None)
        joints = scope_joints(self.embodiment, [parts] if isinstance(parts, str) else parts)
        if forget is not None and joints:
            forget(joints)

    def hold(self) -> None:
        self.hold_parts([""])

    def reset(self, seed: Optional[int] = None) -> None:
        self.inner.reset(seed)
        self._reset_layers(self.layers)

    def reset_parts(self, parts: Any, seed: Optional[int] = None) -> None:
        paths = [str(p).strip(SEP) for p in ([parts] if isinstance(parts, str) else parts)]
        self._inner.reset_parts(paths, seed)
        self._forget(paths)
        from hirobot.assembly.parts import resolve_part

        scope = [resolve_part(self.embodiment, p) for p in paths]
        self._reset_layers([p for p in self.layers
                            if any(part_covers(s, p) or part_covers((p, False), s[0])
                                   for s in scope)])

    def _reset_layers(self, paths: Sequence[str]) -> None:
        # The parts are reset already; each layer resets itself (its port skips the parts).
        token = _ROOT_DOES_IT.set(self)
        try:
            for p in paths:
                self.layers[p].reset()
        finally:
            _ROOT_DOES_IT.reset(token)

    # ------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        self.inner.connect()
        for layer in self.layers.values():
            layer.connect()  # its port does nothing: the parts are connected

    def disconnect(self) -> None:
        # The layers first (their threads stop sending), then the parts.
        try:
            for p, layer in self.layers.items():
                try:
                    layer.disconnect()
                except Exception:  # noqa: BLE001 - disconnect the robot regardless
                    logger.exception("disconnecting the layer on part %r failed", p)
        finally:
            self.inner.disconnect()

    def _layer_fault(self, layer: Layer, reason: str) -> None:
        """A layer on a part entered the fault state: stop the whole tree."""
        path = next((p for p, x in self.layers.items() if x is layer), "?")
        if self._fault is not None:
            return  # already stopping (another layer, or this one again)
        logger.error("layer on part %r: %s; holding the other parts", path, reason)
        Robot._set_fault(self, f"layer on part {path!r}: {reason}")
        from hirobot.control.safety import holding

        with holding():
            try:
                _, direct = self._expand("")  # every part without a layer
                if direct:
                    self._inner.hold_parts(direct)
            except Exception:  # noqa: BLE001 - hold the other layers' parts too
                logger.exception("holding the parts without a layer failed")
            for p in self.layers:
                if p == path:
                    continue
                try:
                    self.layers[p].hold()
                except Exception:  # noqa: BLE001 - hold every part
                    logger.exception("hold through the layer on part %r failed", p)
        # The whole tree refuses commands (the other layers' threads too) until reset().
        Robot._set_fault(self._inner, f"layer on part {path!r}: {reason}")

    # ---------------------------------------------------------- layer skills

    def list_layer_skills(self) -> List[RobotSkillSpec]:
        """The part layers' skills, named ``<part>/<skill>``."""
        return [s.model_copy(update={"name": _join(p, s.name)})
                for p, layer in self.layers.items() for s in layer.list_layer_skills()]

    def run_layer_skill(self, name: str,
                        args: Optional[Mapping[str, Any]] = None) -> RobotSkillResult:
        """Run ``<part>/<skill>``; it claims only that part (``Priority.LAYER_SKILL``)."""
        for p, layer in self.layers.items():
            if name.startswith(p + SEP):
                result = layer.run_layer_skill(name[len(p) + 1:], args)
                return result.model_copy(update={"name": name})
        raise RobotSkillError(f"no layer skill {name!r}; available: "
                              f"{[s.name for s in self.list_layer_skills()]}")

    def stop_layer_skill(self) -> None:
        for layer in self.layers.values():
            layer.stop_layer_skill()

    @property
    def layer_skill_running(self) -> bool:
        return any(layer.layer_skill_running for layer in self.layers.values())


__all__ = ["PartLayers", "PartPort", "check_layer_parts", "part_robot"]

