"""Old single-robot config format (``embodiment:`` + ``body:``) and ``body_overrides``.

A thin compatibility layer for code outside hiRobot (hiWBC, hiAgent) that has
not moved to ``assembly:`` configs yet. It is deprecated and goes away once
they have. New code writes ``assembly:`` configs and
passes ``overrides={'<part path>': {...}}``.

* :func:`make_legacy` builds an old-format config as before the migration:
  one body driver, its end-effector drivers, cameras and audio in one
  :class:`~hirobot.assembly.RobotAssembly`. ``group:`` configs are not kept.
* :func:`body_overrides_as_tree` turns ``body_overrides`` for an
  ``assembly:`` config with one body into ``overrides`` for that body's driver.
"""

from __future__ import annotations

import warnings
from typing import Any, Dict, Mapping, Optional

from hirobot.core.embodiment import Embodiment

OLD_KEYS = ("embodiment", "body")


def is_legacy_config(config: Mapping[str, Any]) -> bool:
    """Whether a loaded config is in the old single-robot format."""
    return "embodiment" in config and "assembly" not in config and "device" not in config


def _warn(what: str) -> None:
    warnings.warn(f"{what} is deprecated and will be removed; write an assembly: config and pass "
                  "overrides={'<part path>': {...}}", DeprecationWarning, stacklevel=4)


def legacy_embodiment(config: Mapping[str, Any]) -> Embodiment:
    """The ``embodiment:`` section of an old-format config."""
    _warn("the embodiment: config format")
    return Embodiment(**config["embodiment"])


def body_overrides_as_tree(config: Any, body_overrides: Mapping[str, Any]) -> Dict[str, Any]:
    """``body_overrides`` (old ``body:`` keys) as ``overrides`` for the config's one body.

    ``driver: <name>`` becomes the driver's ``type``; every other key is a
    driver argument, as in the old ``body:`` section.
    """
    from hirobot.loading.build import only_body
    from hirobot.loading.tree import load_robot

    _warn("body_overrides")
    path, _ = only_body(load_robot(config), "body_overrides")
    driver = dict(body_overrides)
    if "driver" in driver:
        driver["type"] = driver.pop("driver")
    return {path: {"driver": driver}}


def make_legacy(config: Mapping[str, Any], ref: Any,
                body_overrides: Optional[Mapping[str, Any]]) -> Any:
    """Build an old-format config (see the module docstring)."""
    from hirobot import _finish, _warn_slow_cameras
    from hirobot.assembly import RobotAssembly
    from hirobot.devices.audio import build_audio
    from hirobot.devices.camera import build_cameras
    from hirobot.devices.end_effector import (
        body_embodiment,
        build_end_effectors,
        has_end_effector_drivers,
    )
    from hirobot.loading.registry import get_body

    unknown = sorted(set(config) - {"embodiment", "body", "safety", "layers", "arbiter"})
    if unknown:
        raise ValueError(f"{ref!r}: key(s) {unknown} are not in the old embodiment: format; "
                         "write the robot as an assembly: config")
    embodiment = legacy_embodiment(config)
    _warn_slow_cameras(embodiment)
    body_cfg = dict(config.get("body") or {})
    body_cfg.update(body_overrides or {})
    driver_name = body_cfg.pop("driver", "toy")
    driver_cls = get_body(driver_name)
    # A real body drives only its own joints; end-effector drivers take the
    # finger joints. A simulator models the fingers itself and gets every joint.
    end_effectors = {}
    body_emb = embodiment
    if not driver_cls.simulated and has_end_effector_drivers(embodiment):
        end_effectors = build_end_effectors(embodiment, body_cfg.get("network_interface"),
                                            read_only=bool(body_cfg.get("read_only")))
        body_emb = body_embodiment(embodiment)
    robot = driver_cls(body_emb, **body_cfg)
    sim_only = bool(driver_cls.simulated)
    nic = body_cfg.get("network_interface")
    cameras = build_cameras(embodiment.cameras, nic, sim_only=sim_only)
    microphones, speakers = build_audio(embodiment.microphones, embodiment.speakers, nic,
                                        sim_only=sim_only)
    if cameras or end_effectors or microphones or speakers:
        robot = RobotAssembly(robot, cameras=cameras, end_effectors=end_effectors,
                              embodiment=embodiment, microphones=microphones, speakers=speakers)
    safety_cfg = config.get("safety", {})
    sim_cfg = None
    if safety_cfg is not False:
        safety_cfg = dict(safety_cfg or {})
        sim_cfg = safety_cfg.pop("sim_check", None)
    return _finish(robot, embodiment, safety_cfg, sim_cfg, config.get("layers"),
                   config.get("arbiter"), None, driver_name)
