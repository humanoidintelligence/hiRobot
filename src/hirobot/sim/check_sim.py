"""``hirobot check --sim``: build the simulation-check world of a config and report it.

Loading the world checks what :mod:`hirobot.sim.sim_check` needs before it is on a
robot: every body has a ``model:``; every joint is in its MJCF (``<name>`` or
``<name>_joint``) with a joint actuator; every mount and end-effector frame is
a body of the MJCF; joint ``position_limit`` and the MJCF's range agree within
0.1 rad (a warning otherwise). The world is not stepped.

With ``against_real``, every body's real driver is opened ``read_only: true``
(a driver without that mode is refused, as it could command the robot), one
observation is read, the world is set to its joint angles, and each end
effector's measured position is compared with the model's.
"""

from __future__ import annotations

import inspect
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional

logger = logging.getLogger(__name__)


@dataclass
class SimReport:
    """What :func:`check_sim` found.

    Attributes:
        bodies: Body part path -> its MJCF path.
        joints: Number of joints in the world (all bodies).
        end_effectors: End effectors with a frame in the model.
        warnings: Range mismatches and similar, as text.
        ee_errors: End effector -> measured vs model distance (m); with ``against_real``.
    """

    bodies: Dict[str, str]
    joints: int
    end_effectors: List[str]
    warnings: List[str] = field(default_factory=list)
    ee_errors: Optional[Dict[str, float]] = None


def bodies(spec: Any) -> Dict[str, Any]:
    """Body part path -> its device spec."""
    from hirobot.loading.tree import DeviceSpec

    return {p: d for p, d in spec.root.walk() if isinstance(d, DeviceSpec) and d.kind == "body"}


def check_sim(config: Any, against_real: bool = False,
              drivers: Optional[Mapping[str, Mapping[str, Any]]] = None) -> SimReport:
    """Build ``config``'s check world (and compare with the real robot when ``against_real``).

    Args:
        config: A built-in name or path (an ``assembly:`` config).
        against_real: Also read the real robot once, read-only.
        drivers: Body part path -> overrides for its driver (``network_interface``...).

    Raises:
        ValueError: A body without ``model:``, a joint or frame not in the
            MJCF, or (``against_real``) a body driver with no read-only mode.
    """
    from hirobot.loading.layout import node_layout
    from hirobot.sim.sim_check import _Sim
    from hirobot.loading.tree import load_robot

    spec = load_robot(config)
    found = bodies(spec)
    missing = [p for p, d in found.items() if not d.fields.get("model")]
    if missing:
        raise ValueError(f"body part(s) {missing} have no model: (their MJCF); the simulation "
                         "check needs one for every body")
    emb = node_layout(spec.root).embodiment
    sim = spec.sim or {}
    records: List[logging.LogRecord] = []
    handler = _Collect(records)
    check_log = logging.getLogger("hirobot.sim.sim_check")
    check_log.addHandler(handler)
    try:
        world = _Sim(spec, emb, fixed_base=sim.get("fixed_base") or [],
                     base_height=sim.get("base_height") or {})
    finally:
        check_log.removeHandler(handler)
    try:
        report = SimReport(
            bodies={p: str(d.fields["model"]["mjcf"]) for p, d in found.items()},
            joints=len(world.names),
            end_effectors=[v._prefix + name for v in world.views for name, _, _ in v._ee_frames],
            warnings=[r.getMessage() for r in records if r.levelno >= logging.WARNING])
        if against_real:
            report.ee_errors = _against_real(config, spec, found, world, drivers or {})
        return report
    finally:
        world.world.disconnect()


def _against_real(config: Any, spec: Any, found: Mapping[str, Any], world: Any,
                  drivers: Mapping[str, Mapping[str, Any]]) -> Dict[str, float]:
    from hirobot import make_robot
    from hirobot.loading.registry import get_body

    unknown = sorted(set(drivers) - set(found))
    if unknown:
        raise ValueError(f"--driver: {unknown} are not bodies of {config}; bodies: {sorted(found)}")
    overrides: Dict[str, Any] = {}
    for path, dev in found.items():
        name = (dev.driver or {"type": "toy"})["type"]
        cls = get_body(name)
        if cls.simulated:
            raise ValueError(f"{path}: body driver {name!r} is a simulator; --against-real "
                             "compares with the real robot")
        if "read_only" not in inspect.signature(cls.__init__).parameters:
            raise ValueError(f"{path}: body driver {name!r} has no read_only mode, so "
                             "--against-real will not open it (it could command the robot)")
        overrides[path] = {"driver": {**dict(drivers.get(path) or {}), "read_only": True}}
    real = make_robot(config, overrides=overrides, sim=False)
    real.connect()
    try:
        obs = real.get_observation()
    finally:
        real.disconnect()
    world._sync(obs)
    return world.ee_errors(obs)


class _Collect(logging.Handler):
    def __init__(self, records: List[logging.LogRecord]) -> None:
        super().__init__(logging.WARNING)
        self.records = records

    def emit(self, record: logging.LogRecord) -> None:
        self.records.append(record)
