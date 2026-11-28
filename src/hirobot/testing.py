"""Checks for plugin authors.

:func:`assert_layer_uses_public_api` runs a layer class on a toy robot and
fails if the layer reached past the public API of the robot below it (see
:class:`~hirobot.control.layer.Layer`, "Layers use the public API of ``inner``").
:func:`assert_forwards` checks that a wrapper passes the robot's declarations up.
"""

from __future__ import annotations

from typing import Any, Callable, Iterable, Mapping, Optional

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.control.layer import InnerProxy, Layer
from hirobot.core.robot import Robot
from hirobot.core.types import Action, ControlMode


def _toy_embodiment(num_joints: int = 4) -> Embodiment:
    return Embodiment(name="toy", joints=[{"name": f"joint_{i}"} for i in range(num_joints)],
                      control_hz=50)


def _default_actions(layer: Robot) -> Iterable[Action]:
    modes = tuple(layer.action_modes)
    n = layer.embodiment.num_dofs
    if ControlMode.POSITION in modes:
        yield Action(mode=ControlMode.POSITION, joint_target=np.zeros(n))
    if ControlMode.BASE_VELOCITY in modes:
        yield Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.zeros(3))


def assert_layer_uses_public_api(
    layer_cls: Callable[..., Robot],
    config: Optional[Mapping[str, Any]] = None,
    *,
    inner: Optional[Robot] = None,
    actions: Optional[Iterable[Action]] = None,
    steps: int = 5,
) -> set:
    """Run ``layer_cls`` over a robot and check it used only the public API below it.

    Builds ``layer_cls(inner, **config)`` (``inner`` defaults to a 4-joint
    ``toy`` robot), then connects, resets, sends ``actions`` (default: a zero
    command in each of ``POSITION`` / ``BASE_VELOCITY`` the layer accepts)
    ``steps`` times, reads an observation and the status, holds, e-stops and
    disconnects. Fails with ``AssertionError`` if the layer is not a
    :class:`~hirobot.control.layer.Layer` or tried a name the
    :class:`~hirobot.control.layer.InnerProxy` refuses (``_send_action``,
    ``_set_safety``, ``body``, ...), even if the layer caught the error.

    Returns the public names the layer read on ``inner``.
    """
    if inner is None:
        from hirobot.drivers.sim.toy import ToyRobot

        inner = ToyRobot(_toy_embodiment())
    layer = layer_cls(inner, **dict(config or {}))
    assert isinstance(layer, Layer), (
        f"{getattr(layer_cls, '__name__', layer_cls)} does not subclass hirobot.Layer, so its "
        "inner is not checked")
    proxy = layer._inner_proxy
    assert type(proxy) is InnerProxy
    layer.connect()
    try:
        layer.reset()
        todo = list(actions) if actions is not None else list(_default_actions(layer))
        for _ in range(steps):
            for action in todo:
                layer.send_action(action)
        layer.get_observation()
        layer.get_status()
        layer.hold()
        layer.estop()
    finally:
        layer.disconnect()
    denied = sorted(proxy.denied)
    assert not denied, (
        f"{type(layer).__name__} reached past the public API of the robot below it: {denied}. "
        "Command it through inner.send_action and the other public methods, so the safety "
        "gate below checks what the layer sends.")
    return set(proxy.accessed)


#: The public declarations of :class:`~hirobot.core.robot.Robot` a wrapper must
#: pass up unchanged unless it means to change them (:func:`assert_forwards`).
FORWARDED = ("embodiment", "action_modes", "motion_spaces", "direct_joints",
             "cartesian_targets", "accepts_parts", "simulated", "command_timeout_s",
             "cameras", "microphones", "speakers", "reward", "success", "done",
             "has_onboard_skills")


def _same(a: Any, b: Any) -> bool:
    if a is b:
        return True
    if isinstance(a, Mapping) and isinstance(b, Mapping):
        return dict(a).keys() == dict(b).keys() and all(a[k] is b[k] for k in a)
    try:
        return bool(tuple(a) == tuple(b)) if isinstance(a, (tuple, list)) else bool(a == b)
    except Exception:  # noqa: BLE001 - values that do not compare count as different
        return False


def assert_forwards(wrapper: Robot, inner: Robot, *, changes: Iterable[str] = ()) -> None:
    """Check that ``wrapper`` passes ``inner``'s public declarations up unchanged.

    Compares each name in :data:`FORWARDED` (``embodiment``, ``action_modes``,
    ``cameras``, ...), skipping those in ``changes`` (the ones the wrapper
    means to change), and checks that ``wrapper.now()`` is ``inner``'s clock
    (a simulator's time, not the host's). Fails with ``AssertionError`` listing
    every name that differs, so a wrapper that misses a declaration added to
    ``Robot`` later is caught. Works for any wrapper, a
    :class:`~hirobot.control.layer.Layer` or not; it only reads.
    """
    changes = set(changes)
    unknown = changes - set(FORWARDED)
    assert not unknown, f"changes: {sorted(unknown)} are not in hirobot.testing.FORWARDED"
    differ = []
    for name in FORWARDED:
        if name in changes:
            continue
        mine, theirs = getattr(wrapper, name, None), getattr(inner, name, None)
        if not _same(mine, theirs):
            differ.append(f"{name}: {mine!r} (inner: {theirs!r})")
    before = inner.now()
    mine = wrapper.now()
    after = inner.now()
    if not before <= mine <= after:
        differ.append(f"now(): {mine} is not the inner clock ({before}..{after})")
    assert not differ, (f"{type(wrapper).__name__} does not pass these up from "
                        f"{type(inner).__name__}: " + "; ".join(differ))


__all__ = ["FORWARDED", "assert_forwards", "assert_layer_uses_public_api"]
