"""Unitree Dex3-1 three-finger hand (G1), over the SDK2 DDS interface.

Each hand has 7 motors and its own topics: ``HandCmd_`` on
``rt/dex3/<side>/cmd`` and ``HandState_`` on ``rt/dex3/<side>/state``
(``unitree_hg`` family; the state topic runs at ~800 Hz). Motor order is
thumb 0-2, middle 0-1, index 0-1 for both hands, the order Unitree's
xr_teleoperate maps to (``left_dex3_api_joint_names``); the joint names match
the ``*_hand_*_joint`` names in unitree_ros ``g1_29dof_with_hand_rev_1_0``.

Every motor command carries a mode byte: motor id in bits 0-3, status (1 =
enabled) in bits 4-6, and a timeout flag in bit 7. :meth:`damp` sends what the
SDK example (``g1_dex3_example.cpp``) sends to stop: zero gains with the timeout
flag set. Default gains (kp 1.5, kd 0.2) are those xr_teleoperate uses.

Torque: the ``tau_est`` this hand reports is not in Nm (we read values that are
multiples of ~1674.6 on a hand whose joints are limited to 1.4-2.45 Nm), so
:meth:`get_state` reports NaN torque. The fingertip and palm pressure arrays
are returned as ``pressure`` (shape ``(sensors, 12)``, raw sensor units).
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Sequence, Union

import numpy as np

from hirobot.drivers._sdk.unitree_sdk2 import channel_api, init_channel
from hirobot.core.embodiment import Joint
from hirobot.core.stamps import Stamp, StampCounter
from hirobot.devices.end_effector import EndEffectorDriver, JointsState, register_end_effector
from hirobot.loading.sdk import require_sdk

NUM_MOTORS = 7
_STATUS_ENABLED = 0x01


def motor_mode(motor_id: int, status: int = _STATUS_ENABLED, timeout: bool = False) -> int:
    """The Dex3 per-motor mode byte (id | status << 4 | timeout << 7)."""
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((1 if timeout else 0) << 7)


@register_end_effector("unitree_handcmd")
class UnitreeDex3Hand(EndEffectorDriver):
    """One Dex3-1 hand.

    Args:
        name: End-effector name.
        joints: The hand's 7 embodiment joints, in motor order.
        side: ``"left"`` or ``"right"``; selects the DDS topics.
        network_interface: NIC on the robot subnet. The channel factory is
            shared with the body driver; whichever connects first sets it.
        domain_id: DDS domain id.
        default_kp / default_kd: Gains used when a command carries none.
        ramp_seconds: Duration of the kp ramp in :meth:`reset`.
        ramp_hz: Command rate during that ramp.
        stale_s: :meth:`get_state` raises if the newest state is older.
        read_only: Never create the command publisher.
    """

    def __init__(
        self,
        name: str,
        joints: Sequence[Joint],
        side: str,
        network_interface: Optional[str] = None,
        domain_id: int = 0,
        default_kp: Union[float, Sequence[float]] = 1.5,
        default_kd: Union[float, Sequence[float]] = 0.2,
        ramp_seconds: float = 1.0,
        ramp_hz: float = 100.0,
        stale_s: float = 0.5,
        read_only: bool = False,
    ) -> None:
        super().__init__(name, joints, read_only=read_only)
        if side not in ("left", "right"):
            raise ValueError(f"{name}: side must be 'left' or 'right', got {side!r}")
        if self.num_joints != NUM_MOTORS:
            raise ValueError(f"{name}: Dex3 has {NUM_MOTORS} motors, got {self.num_joints} joints")
        self.side = side
        self.cmd_topic = f"rt/dex3/{side}/cmd"
        self.state_topic = f"rt/dex3/{side}/state"
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._kp = self._per_joint(default_kp, "default_kp")
        self._kd = self._per_joint(default_kd, "default_kd")
        self._ramp_seconds = float(ramp_seconds)
        self._ramp_period = 1.0 / float(ramp_hz)
        self._stale_s = float(stale_s)

        self._lock = threading.Lock()
        self._msg = None
        self._stamps = StampCounter()
        self._stamp: Optional[Stamp] = None
        self._publisher = None
        self._subscriber = None
        self._cmd = None

    def _per_joint(self, value, what: str) -> np.ndarray:
        arr = np.asarray(value, dtype=float)
        if arr.ndim == 0:
            return np.full(NUM_MOTORS, float(arr))
        if arr.shape != (NUM_MOTORS,):
            raise ValueError(f"{self.name}: {what} has {arr.size} values, the hand has {NUM_MOTORS}")
        return arr.copy()

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        require_sdk(__package__, "UnitreeDex3Hand")
        _, ChannelPublisher, ChannelSubscriber = channel_api("UnitreeDex3Hand")
        try:
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_ as HandCmdDefault
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(
                "UnitreeDex3Hand needs unitree_sdk2py (https://github.com/unitreerobotics/"
                "unitree_sdk2_python)."
            ) from exc
        init_channel("UnitreeDex3Hand", self._domain_id, self._network_interface)
        self._cmd = HandCmdDefault()
        if not self.read_only:
            self._publisher = ChannelPublisher(self.cmd_topic, HandCmd_)
            self._publisher.Init()
        self._subscriber = ChannelSubscriber(self.state_topic, HandState_)
        self._subscriber.Init(self._on_state, 10)
        deadline = time.monotonic() + 2.0
        while self._msg is None and time.monotonic() < deadline:
            time.sleep(0.005)
        if self._msg is None:
            raise TimeoutError(f"{self.name}: no JointsState on {self.state_topic} within 2 s")

    def _on_state(self, msg) -> None:
        with self._lock:
            self._msg = msg
            self._stamp = self._stamps.tick()

    @property
    def raw_state(self):
        """The latest raw SDK ``HandState_`` (diagnostics), or ``None``."""
        return self._msg

    def get_state(self) -> JointsState:
        with self._lock:
            msg, stamp = self._msg, self._stamp
        if msg is None:
            raise RuntimeError(f"{self.name}: no state yet; call connect() first")
        age = time.monotonic() - stamp.t
        if age > self._stale_s:
            raise TimeoutError(f"{self.name}: newest state is {age:.2f}s old (> {self._stale_s}s)")
        motors = msg.motor_state[:NUM_MOTORS]
        extra = {}
        if msg.press_sensor_state:
            extra["pressure"] = np.array([s.pressure for s in msg.press_sensor_state], dtype=np.float32)
        return JointsState(
            q=np.array([m.q for m in motors], dtype=float),
            dq=np.array([m.dq for m in motors], dtype=float),
            tau=np.full(NUM_MOTORS, np.nan),
            stamp=stamp,
            extra=extra,
        )

    # ------------------------------------------------------------- commands

    def _write(self, q, kp, kd, timeout: bool = False) -> None:
        if self._publisher is None:
            raise RuntimeError(f"{self.name}: no command publisher (read_only or not connected)")
        with self._lock:
            for i in range(NUM_MOTORS):
                m = self._cmd.motor_cmd[i]
                m.mode = motor_mode(i, timeout=timeout)
                m.q = float(q[i])
                m.dq = 0.0
                m.tau = 0.0
                m.kp = float(kp[i])
                m.kd = float(kd[i])
            self._publisher.Write(self._cmd)

    def command(self, q: np.ndarray, kp: Optional[np.ndarray] = None,
                kd: Optional[np.ndarray] = None) -> None:
        q = np.asarray(q, dtype=float)
        if q.shape != (NUM_MOTORS,) or not np.all(np.isfinite(q)):
            raise ValueError(f"{self.name}: need {NUM_MOTORS} finite targets, got {q!r}")
        self._write(self.clip(q),
                    self._kp if kp is None else self._per_joint(kp, "kp"),
                    self._kd if kd is None else self._per_joint(kd, "kd"))

    def reset(self) -> None:
        """Hold the measured pose, ramping kp from 0 over ``ramp_seconds``."""
        if self.read_only:
            raise RuntimeError(f"{self.name} is read_only; reset() would command the hand")
        token = self._stop_token()
        start = self.clip(self.get_state().q)
        steps = max(1, int(self._ramp_seconds / self._ramp_period))
        for s in range(steps + 1):
            with self._commanding(token):
                self._write(start, self._kp * min(1.0, s / steps), self._kd)
            self._wait(self._ramp_period, token)

    def set_gains(self, kp: Optional[np.ndarray], kd: Optional[np.ndarray]) -> None:
        if kp is not None:
            self._kp = self._per_joint(kp, "kp")
        if kd is not None:
            self._kd = self._per_joint(kd, "kd")

    def damp(self) -> None:
        """Stop the motors as the SDK example does: zero gains, timeout flag set."""
        if self._publisher is None:
            return
        zeros = np.zeros(NUM_MOTORS)
        for _ in range(3):  # best effort over UDP
            self._write(zeros, zeros, zeros, timeout=True)
            time.sleep(0.01)
