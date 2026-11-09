"""Pair a real robot with a MuJoCo simulation of it.

:class:`TwinRobot` wraps a real-hardware driver and a
:class:`~hirobot.drivers.sim.mujoco.MujocoRobot` and runs in one of two modes:

``mirror``
    Control goes to the real robot exactly as without the wrapper. A
    background thread copies the real robot's measured joint state (and base
    orientation) into the simulation and renders it, so the viewer shows what
    the hardware is doing. The copy is kinematic: no physics runs.

``shadow``
    The real driver is opened **read-only** (no command publisher is ever
    created), its measured pose seeds the simulation once, and every action,
    gain change, reset and disconnect goes to the simulation only. Use it to
    rehearse a motion from the robot's current pose before running it for real.

Without a real robot, use the ``mujoco`` driver directly.

``make_robot`` builds this wrapper from a config's ``sim:`` section::

    sim:
      mode: mirror            # or shadow
      model: ~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml
      viewer: true
      fixed_base: true        # robot hung on a gantry
"""

from __future__ import annotations

import threading
import time
from typing import Any, List, Mapping, Optional

import numpy as np

from hirobot.embodiment import Embodiment
from hirobot.robot import Robot
from hirobot.robot_skills import RobotSkillError, RobotSkillResult, RobotSkillSpec
from hirobot.stamps import Stamp
from hirobot.types import Action, Observation, RobotStatus

MODES = ("mirror", "shadow")


class TwinRobot(Robot):
    """A real robot plus its simulated twin.

    Args:
        real: The hardware driver. In ``shadow`` mode it must have been built
            read-only; this is checked on :meth:`connect`.
        sim: The MuJoCo driver for the same embodiment.
        mode: ``"mirror"`` or ``"shadow"`` (see the module docstring).
        mirror_hz: Rate of the mirroring thread.
    """

    def __init__(
        self, real: Robot, sim: Robot, mode: str = "mirror", mirror_hz: float = 50.0
    ) -> None:
        if mode not in MODES:
            raise ValueError(f"sim mode must be one of {MODES}, got {mode!r}")
        if real.embodiment.joint_names != sim.embodiment.joint_names:
            raise ValueError("real and sim drivers must share the embodiment's joint order")
        self.real = real
        self.sim = sim
        self.mode = mode
        self._mirror_period = 1.0 / mirror_hz
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.mirror_error: Optional[BaseException] = None

    @property
    def embodiment(self) -> Embodiment:
        return self.real.embodiment

    @property
    def action_modes(self):  # type: ignore[override]
        return self.active.action_modes

    @property
    def motion_spaces(self):  # type: ignore[override]
        return self.active.motion_spaces

    @property
    def direct_joints(self):  # type: ignore[override]
        return self.active.direct_joints

    @property
    def cartesian_targets(self):  # type: ignore[override]
        return self.active.cartesian_targets

    @property
    def fault(self):  # type: ignore[override]
        return self._fault or self.active.fault

    def idle_step(self) -> None:
        self.active.idle_step()

    def get_status(self) -> RobotStatus:
        status = self.real.get_status()
        status.fault = self.fault
        return status

    @property
    def active(self) -> Robot:
        """The driver that receives commands: the real robot, or the sim in shadow mode."""
        return self.sim if self.mode == "shadow" else self.real

    # The outcome comes from the active driver. The base class declares it,
    # so ``__getattr__`` would never be reached for it. ``simulated`` stays
    # False: a real robot is attached, so callers keep pacing to real time.
    @property
    def reward(self):  # type: ignore[override]
        return getattr(self.active, "reward", None)

    @property
    def success(self):  # type: ignore[override]
        return getattr(self.active, "success", None)

    @property
    def done(self):  # type: ignore[override]
        return getattr(self.active, "done", None)

    def __getattr__(self, name: str) -> Any:
        # Driver-specific extras (e.g. Unitree's raw_state / motion_mode) come
        # from the active driver. Only reached for attributes not found here.
        if name in ("real", "sim", "mode"):
            raise AttributeError(name)
        return getattr(self.active, name)

    # -------------------------------------------------------------- lifecycle

    def connect(self) -> None:
        if self.mode == "shadow" and not getattr(self.real, "_read_only", False):
            raise RuntimeError("shadow mode needs the real driver built with read_only=True")
        self.real.connect()
        self.sim.connect()
        self._copy_real_to_sim()
        if self.mode == "mirror":
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._mirror_loop, name="sim-mirror", daemon=True
            )
            self._thread.start()

    def _copy_real_to_sim(self) -> None:
        obs = self.real.get_observation()
        self.sim.set_state(obs.joint_pos, obs.joint_vel, obs.base_orientation)

    def _mirror_loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.time()
            try:
                self._copy_real_to_sim()
            except Exception as exc:  # noqa: BLE001 - keep the real control path alive
                self.mirror_error = exc
                return
            self._stop.wait(max(0.0, self._mirror_period - (time.time() - t0)))

    def reset(self, seed: Optional[int] = None) -> None:
        self.active.reset(seed)

    def _estop(self) -> None:
        # Both: the real robot in mirror mode, the simulation in shadow mode
        # (the read-only real driver then has nothing to stop).
        try:
            self.real.estop()
        finally:
            self.sim.estop()

    def hold(self) -> None:
        self.active.hold()

    def get_observation(self) -> Observation:
        obs = self.active.get_observation()
        if self.active is self.sim and "body" in obs.stamps:
            # Shadow: the state is the simulator's, but the clock stays the
            # host's (real cameras ride along), so stamp it on that clock.
            obs.stamps["body"] = Stamp(self.real.now(), obs.stamps["body"].seq)
        return obs

    def now(self) -> float:
        # The real robot is attached in both modes, so time is the host's.
        return self.real.now()

    def _send_action(self, action: Action) -> None:
        # The safety gate already ran in send_action on this wrapper.
        self.active._send_action(action)

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        self.active.set_gains(kp, kd)

    # Robot skills run on the real robot's own controller, which the simulation
    # cannot rehearse: shadow mode lists none and refuses them.
    @property
    def has_robot_skills(self) -> bool:
        return self.mode != "shadow" and self.real.has_robot_skills

    def list_robot_skills(self) -> List[RobotSkillSpec]:
        return [] if self.mode == "shadow" else self.real.list_robot_skills()

    def run_robot_skill(
        self, name: str, args: Optional[Mapping[str, Any]] = None
    ) -> RobotSkillResult:
        if self.mode == "shadow":
            raise RobotSkillError("robot skills are not available in sim shadow mode")
        return self.real.run_robot_skill(name, args)

    def stop_robot_skill(self) -> None:
        if self.mode != "shadow":
            self.real.stop_robot_skill()

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self.real.disconnect()  # damps the real robot in mirror; no-op send in shadow
        finally:
            self.sim.disconnect()
