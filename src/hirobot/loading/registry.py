"""Name-based registries for drivers.

Each kind of device has its own registry: body drivers, hand drivers,
camera drivers and audio drivers. A driver class registers itself under a
string name; a config or caller then looks one up by that name and constructs
it. This is what lets the same control code select a simulator or a
real-hardware driver by changing one config field.

The built-in drivers live in :mod:`hirobot.drivers` and are imported the first
time any registry is queried. Drivers from other packages are found through
the ``hirobot.drivers`` entry-point group: each entry point names a module
that registers its drivers on import, e.g. in that package's ``pyproject.toml``::

    [project.entry-points."hirobot.drivers"]
    acme = "hirobot_acme.drivers"

Layers (:data:`LAYERS`) are the other kind of plugin: a :class:`Robot` that
wraps another robot and changes how it is commanded (a whole-body controller
that turns base velocities into joint targets, say). A config stacks them with
``layers:``; see :func:`hirobot.make_robot`. hiRobot ships none. A plugin
package registers its layers from the same ``hirobot.drivers`` entry point.
"""

from __future__ import annotations

import importlib
import logging
import sys
from typing import Callable, Dict, Generic, List, Type, TypeVar

from hirobot.core.robot import Robot

logger = logging.getLogger(__name__)

#: Entry-point group that external driver packages register under.
ENTRY_POINT_GROUP = "hirobot.drivers"

T = TypeVar("T")

_loaded = False


def load_drivers() -> None:
    """Import the built-in drivers and every ``hirobot.drivers`` entry point, once.

    A plugin that fails to import is logged and skipped, so one broken package
    does not take the others down.
    """
    global _loaded
    if _loaded:
        return
    _loaded = True
    try:
        importlib.import_module("hirobot.drivers")
    except BaseException:
        _loaded = False
        raise
    for ep in _entry_points():
        try:
            ep.load()
        except Exception as exc:  # noqa: BLE001 - a broken plugin must not break the rest
            logger.warning("hirobot driver plugin %r (%s) failed to load: %s", ep.name, ep.value, exc)


def _entry_points():
    from importlib.metadata import entry_points

    if sys.version_info >= (3, 10):
        return list(entry_points(group=ENTRY_POINT_GROUP))
    return list(entry_points().get(ENTRY_POINT_GROUP, []))  # pragma: no cover - py39


class Registry(Generic[T]):
    """Driver classes of one kind, by name.

    Args:
        kind: What the entries are, for error messages (``"robot driver"``).
        base: The base class (or tuple of classes) every entry must subclass.
    """

    def __init__(self, kind: str, base) -> None:
        self.kind = kind
        self.base = base
        self._entries: Dict[str, Type[T]] = {}

    def register(self, name: str) -> Callable[[Type[T]], Type[T]]:
        """Class decorator that registers a driver under ``name``.

        Raises:
            ValueError: If ``name`` is taken, or the class is not a subclass of ``base``.
        """

        def decorator(cls: Type[T]) -> Type[T]:
            if not issubclass(cls, self.base):
                names = (self.base if isinstance(self.base, tuple) else (self.base,))
                raise ValueError(
                    f"{cls.__name__} must subclass {' or '.join(b.__name__ for b in names)}.")
            if name in self._entries:
                raise ValueError(f"{self.kind} {name!r} already registered as "
                                 f"{self._entries[name].__name__}.")
            self._entries[name] = cls
            return cls

        return decorator

    def get(self, name: str) -> Type[T]:
        """The class registered under ``name``.

        Raises:
            KeyError: If nothing is registered under ``name``.
        """
        if name not in self._entries:
            load_drivers()
        if name not in self._entries:
            raise KeyError(f"Unknown {self.kind} {name!r}. Available: {self.available()}")
        return self._entries[name]

    def available(self) -> List[str]:
        """Sorted names of every registered driver of this kind."""
        load_drivers()
        return sorted(self._entries)


BODIES: Registry = Registry("body driver", Robot)


def register_body(name: str):
    """Class decorator that registers a :class:`~hirobot.core.robot.Robot` driver."""
    return BODIES.register(name)


def get_body(name: str):
    """Look up a registered robot driver class by name."""
    return BODIES.get(name)


def available_bodies() -> List[str]:
    """Names of all registered robot drivers."""
    return BODIES.available()


#: Layer classes: ``cls(inner, **config)`` wraps the robot ``inner``.
LAYERS: Registry = Registry("robot layer", Robot)


def register_layer(name: str):
    """Class decorator that registers a layer, a :class:`Robot` built as ``cls(inner, **config)``."""
    return LAYERS.register(name)


def get_layer(name: str):
    """Look up a registered layer class by name."""
    return LAYERS.get(name)


def available_layers() -> List[str]:
    """Names of all registered layers."""
    return LAYERS.available()
