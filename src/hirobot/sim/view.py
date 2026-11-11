"""Show a real robot's pose in MuJoCo, outside the robot's stack (``hirobot view``).

The real robot is opened with ``read_only: true`` (its driver never creates a
command publisher), so the viewer cannot move it. Each frame copies the measured
joint positions and base orientation into a MuJoCo model of the same robot with
:meth:`~hirobot.drivers.sim.mujoco.MujocoRobot.set_state` (no physics step) and
draws it. Joints are matched by name; joints the model lacks are skipped, and
joints with no reading (``NaN``, a hand that is not connected) keep the model's
pose.

It only displays. Nothing here is in ``make_robot``'s stack or checks a command.
"""

from __future__ import annotations

import inspect
import logging
import time
from typing import Any, Callable, Mapping, Optional, Tuple

import numpy as np

from hirobot.loading.config import available_configs
from hirobot.loading.registry import get_body
from hirobot.core.robot import Robot

logger = logging.getLogger(__name__)

#: How often the viewer copies the real pose, by default.
VIEW_HZ = 30.0


def sim_view(robot: Robot) -> Any:
    """The robot under the gate and the assembly that has ``set_state``.

    In the shared simulation that is the body's
    :class:`~hirobot.drivers.sim.mujoco.MujocoView`.
    """
    inner: Any = robot
    while not callable(getattr(inner, "set_state", None)):
        nxt = getattr(inner, "body", None) or getattr(inner, "_inner", None)
        if nxt is None:
            raise ValueError(f"{type(robot).__name__} is not a robot in the simulation world")
        inner = nxt
    return inner


def default_model(config: Any) -> Optional[str]:
    """The built-in ``<name>_mujoco`` config next to a built-in robot config, or ``None``."""
    if not isinstance(config, str):
        return None
    name = config.split("/")[-1].removesuffix(".yaml")
    candidate = f"robots/{name}_mujoco"
    return f"{name}_mujoco" if candidate in available_configs("robots") else None


def _body(config: Any, overrides: Optional[Mapping[str, Any]] = None) -> Tuple[str, Any]:
    """The path and device of the config's one body, with ``overrides`` (part path -> overrides)."""
    from hirobot.loading.build import only_body
    from hirobot.loading.tree import load_robot

    return only_body(load_robot(config, overrides=overrides), "hirobot view")


def open_read_only(config: Any, driver: Optional[Mapping[str, Any]] = None) -> Robot:
    """``make_robot`` with the body's driver ``read_only: true``; refuse a driver without it.

    ``driver`` is merged over the body's ``driver:`` (``{"network_interface": "eth0"}``).

    Raises:
        ValueError: The config has more than one body, drives a simulator, or
            its body driver takes no ``read_only`` argument (so it could not be
            opened without a way to command the robot).
    """
    from hirobot import make_robot

    path, _ = _body(config)
    over = {path: {"driver": dict(driver or {})}} if driver else None
    _, dev = _body(config, over)
    name = (dev.driver or {"type": "toy"})["type"]
    cls = get_body(name)
    if cls.simulated:
        raise ValueError(f"{config} is a simulated robot (body driver {name}); "
                         "use `hirobot sim --viewer` for it")
    if "read_only" not in inspect.signature(cls.__init__).parameters:
        raise ValueError(f"body driver {name!r} has no read_only mode, so hirobot view will "
                         "not open it (it could command the robot)")
    return make_robot(config, overrides={path: {"driver": dict(driver or {}, read_only=True)}})


def open_model(config: Any, window: bool = True) -> Robot:
    """The MuJoCo robot to draw into (not real time; a window when ``window``)."""
    from hirobot import make_robot

    _, dev = _body(config)
    if not dev.fields.get("model"):
        raise ValueError(f"--model {config}: its body has no model: (MJCF) to simulate")
    return make_robot(config, sim={"realtime": False, "viewer": bool(window)})


class PoseMirror:
    """Copies a real robot's measured pose into a MuJoCo robot, by joint name."""

    def __init__(self, real: Robot, model: Robot) -> None:
        self.real = real
        self.model = model
        # set_state is on the MuJoCo view, under the gate and the assembly.
        self.driver = sim_view(model)
        real_names = real.embodiment.joint_names
        model_names = model.embodiment.joint_names
        index = {n: i for i, n in enumerate(real_names)}
        pairs = [(i, index[n]) for i, n in enumerate(model_names) if n in index]
        self.model_idx = np.array([p[0] for p in pairs], dtype=int)
        self.real_idx = np.array([p[1] for p in pairs], dtype=int)
        #: Real joints the model does not have, and model joints the real robot lacks.
        self.unmatched_real = [n for n in real_names if n not in set(model_names)]
        self.unmatched_model = [n for n in model_names if n not in index]
        self._q = None

    def step(self) -> np.ndarray:
        """Read the real robot once and show it; return the model's joint positions."""
        obs = self.real.get_observation()
        if self._q is None:
            self._q = np.nan_to_num(self.model.get_observation().joint_pos.copy())
        measured = np.asarray(obs.joint_pos, dtype=float)[self.real_idx]
        ok = np.isfinite(measured)
        self._q[self.model_idx[ok]] = measured[ok]
        self.driver.set_state(self._q, None, obs.base_orientation)
        return self._q.copy()


def run(real: Robot, model: Robot, hz: float = VIEW_HZ, seconds: Optional[float] = None,
        running: Optional[Callable[[], bool]] = None, sleep: Callable[[float], None] = time.sleep
        ) -> int:
    """Mirror ``real`` into ``model`` until the window closes, ``seconds`` pass or Ctrl-C.

    Both robots must already be connected. Returns the number of frames shown.
    """
    if hz <= 0:
        raise ValueError(f"hz must be > 0, got {hz}")
    mirror = PoseMirror(real, model)
    if running is None:
        driver = mirror.driver
        world = getattr(driver, "world", None)
        window = getattr(world if world is not None else driver, "_viewer", None) is not None

        def running() -> bool:
            return driver.viewer_running if window else True

    period, frames, start = 1.0 / hz, 0, time.monotonic()
    try:
        while running() and (seconds is None or time.monotonic() - start < seconds):
            t = time.monotonic()
            mirror.step()
            frames += 1
            sleep(max(0.0, period - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    return frames


__all__ = ["PoseMirror", "VIEW_HZ", "default_model", "open_model", "open_read_only", "run"]
