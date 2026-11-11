"""Checks for plugin authors.

:func:`assert_layer_uses_public_api` runs a layer class on a toy robot and
fails if the layer reached past the public API of the robot below it (see
:class:`~hirobot.control.layer.Layer`, "Layers use the public API of ``inner``").
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


__all__ = ["assert_layer_uses_public_api"]
