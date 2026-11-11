"""Inspire RH56DFX six-DoF hands over ``inspire_sdkpy``'s DDS topics.

``inspire_sdkpy`` (https://github.com/NaCl-1374/inspire_hand_ws,
``ModbusDataHandlerDouble``) bridges the hands' RS485 bus to DDS; hiRobot
ships a launcher, ``scripts/inspire_serial_to_dds.py``. One topic pair per hand:

* command ``inspire.inspire_hand_ctrl`` on ``rt/inspire_hand/ctrl/{l,r}``
* state ``inspire.inspire_hand_state`` on ``rt/inspire_hand/state/{l,r}``

The driver writes ``angle_set`` with ``mode = 1`` and reads ``angle_act``
(registers ANGLE_SET 1486, ANGLE_ACT 1546, range 0-1000). Per hand the order is
pinky, ring, middle, index, thumb bend, thumb rotation, each normalised to
[0, 1] with 0 = closed and 1 = open (register / 1000), the same units as the
Unitree-service path (``unitree_motorcmds``). The stock
``Headless_driver_485_double.py`` only publishes ``pos_act`` (``angle_act``
stays 0), so run the hiRobot launcher.

The hand has no passive mode and holds its last target when commands stop, so
:meth:`InspireSdkpyHand.damp` sends nothing.

Only ``unitree_sdk2py`` (for its DDS channel) and ``cyclonedds`` are needed
here; the ``inspire_sdkpy`` message types are declared in this module, so
``inspire_sdkpy`` itself is only needed on the bridge computer.
"""

from __future__ import annotations

import threading
import time
import warnings
from typing import Optional, Sequence, Tuple

import numpy as np

from hirobot.drivers._sdk.unitree_sdk2 import channel_api, init_channel
from hirobot.core.embodiment import Joint
from hirobot.core.stamps import Stamp, StampCounter
from hirobot.devices.end_effector import EndEffectorDriver, JointsState, register_end_effector
from hirobot.loading.sdk import require_sdk

NUM_JOINTS = 6
SUFFIX = {"left": "l", "right": "r"}
CMD_TOPIC = "rt/inspire_hand/ctrl/{}"
STATE_TOPIC = "rt/inspire_hand/state/{}"
SCALE = 1000.0  # Inspire ANGLE registers: 0 = closed, 1000 = open
MODE_ANGLE = 0b0001

_TYPES = None


def _channel_api():
    return channel_api("InspireSdkpyHand")


def _types():
    """``inspire_hand_ctrl`` / ``inspire_hand_state`` as declared by inspire_sdkpy.

    Same fields and type names as ``inspire_sdkpy/inspire_dds`` (IDL module
    ``inspire``, inspire_hand_ws commit fc75490), so the driver does not need
    inspire_sdkpy installed. The type names must match for DDS to connect.

    The classes are built with ``new_class`` and real annotation objects:
    this module uses ``from __future__ import annotations``, and cyclonedds
    cannot resolve string annotations that name a function's local imports.
    """
    global _TYPES
    if _TYPES is None:
        from dataclasses import dataclass
        from types import new_class

        import cyclonedds.idl as idl
        import cyclonedds.idl.annotations as annotate
        import cyclonedds.idl.types as t

        def struct(name, fields):
            cls = new_class(name, (idl.IdlStruct,), {"typename": f"inspire.{name}"},
                            lambda ns: ns.update(__annotations__=fields))
            return dataclass(annotate.final(annotate.autoid("sequential")(cls)))

        i16, u8 = t.sequence[t.int16, 6], t.sequence[t.uint8, 6]
        ctrl = struct("inspire_hand_ctrl", {
            "pos_set": i16, "angle_set": i16, "force_set": i16, "speed_set": i16,
            "mode": t.int8})
        state = struct("inspire_hand_state", {
            "pos_act": i16, "angle_act": i16, "force_act": i16, "current": i16,
            "err": u8, "status": u8, "temperature": u8})
        _TYPES = (ctrl, state)
    return _TYPES


@register_end_effector("inspire_sdkpy")
class InspireSdkpyHand(EndEffectorDriver):
    """One Inspire RH56DFX hand behind ``inspire_sdkpy`` (see the module docstring).

    Args:
        name: End-effector name.
        joints: The hand's 6 embodiment joints, in order pinky, ring, middle,
            index, thumb bend, thumb rotation. Give them ``position_limit: [0, 1]``.
        side: ``"right"`` or ``"left"``; picks the ``l`` / ``r`` topics.
        network_interface: NIC on the robot subnet. The DDS channel is shared
            with the body driver; whichever connects first sets it.
        domain_id: DDS domain id.
        cmd_topic / state_topic: Override the default topics.
        connect_timeout_s: How long :meth:`connect` waits for the first state.
        stale_s: :meth:`get_state` raises if the newest state is older.
        read_only: Never create the command publisher.
        default_kp / default_kd: Accepted for config compatibility and
            ignored; the hand runs its own position loop.
    """

    def __init__(
        self,
        name: str,
        joints: Sequence[Joint],
        side: str,
        network_interface: Optional[str] = None,
        domain_id: int = 0,
        cmd_topic: Optional[str] = None,
        state_topic: Optional[str] = None,
        connect_timeout_s: float = 2.0,
        stale_s: float = 0.5,
        read_only: bool = False,
        default_kp=None,
        default_kd=None,
    ) -> None:
        super().__init__(name, joints, read_only=read_only)
        if side not in SUFFIX:
            raise ValueError(f"{name}: side must be 'left' or 'right', got {side!r}")
        if self.num_joints != NUM_JOINTS:
            raise ValueError(f"{name}: Inspire hand has {NUM_JOINTS} joints, got {self.num_joints}")
        self.side = side
        self.cmd_topic = cmd_topic or CMD_TOPIC.format(SUFFIX[side])
        self.state_topic = state_topic or STATE_TOPIC.format(SUFFIX[side])
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._connect_timeout_s = float(connect_timeout_s)
        self._stale_s = float(stale_s)
        self._lock = threading.Lock()
        self._msg = None
        self._stamps = StampCounter()
        self._stamp: Optional[Stamp] = None
        self._subscriber = None
        self._publisher = None
        self._ctrl_type = None

    # ------------------------------------------------------------ lifecycle

    def _on_state(self, msg) -> None:
        with self._lock:
            self._msg = msg
            self._stamp = self._stamps.tick()

    def _latest(self) -> Tuple[object, Optional[Stamp]]:
        with self._lock:
            return self._msg, self._stamp

    def connect(self) -> None:
        require_sdk(__package__, "InspireSdkpyHand")
        _, Publisher, Subscriber = _channel_api()
        ctrl_type, state_type = _types()
        init_channel("InspireSdkpyHand", self._domain_id, self._network_interface)
        if self._subscriber is None:
            self._subscriber = Subscriber(self.state_topic, state_type)
            self._subscriber.Init(self._on_state, 10)
        if not self.read_only and self._publisher is None:
            self._ctrl_type = ctrl_type
            self._publisher = Publisher(self.cmd_topic, ctrl_type)
            self._publisher.Init()
        deadline = time.monotonic() + self._connect_timeout_s
        while self._latest()[0] is None and time.monotonic() < deadline:
            time.sleep(0.005)
        msg, _ = self._latest()
        if msg is None:
            raise TimeoutError(
                f"{self.name}: no state on {self.state_topic} within "
                f"{self._connect_timeout_s:.0f} s; is scripts/inspire_serial_to_dds.py running?")
        if not any(msg.angle_act) and any(msg.pos_act):
            warnings.warn(
                f"{self.name}: angle_act on {self.state_topic} is all zero while pos_act is "
                "not; the bridge is probably the stock Headless_driver_485_double.py, which "
                "does not publish angle_act. Run scripts/inspire_serial_to_dds.py instead.")

    @property
    def raw_state(self):
        """The latest raw ``inspire_hand_state`` (diagnostics), or ``None``."""
        return self._latest()[0]

    def get_state(self) -> JointsState:
        msg, stamp = self._latest()
        if msg is None:
            raise RuntimeError(f"{self.name}: no state yet; call connect() first")
        age = time.monotonic() - stamp.t
        if age > self._stale_s:
            raise TimeoutError(f"{self.name}: newest state is {age:.2f}s old (> {self._stale_s}s)")
        angle = np.asarray(list(msg.angle_act), dtype=float)
        if angle.shape != (NUM_JOINTS,):
            raise RuntimeError(f"{self.name}: angle_act has {angle.size} values, need {NUM_JOINTS}")
        extra = {"err": np.asarray(list(msg.err), dtype=float),
                 "pos_act": np.asarray(list(msg.pos_act), dtype=float)}
        return JointsState(q=angle / SCALE, dq=np.zeros(NUM_JOINTS), tau=np.full(NUM_JOINTS, np.nan),
                         stamp=stamp, extra=extra)

    # ------------------------------------------------------------- commands

    def command(self, q: np.ndarray, kp: Optional[np.ndarray] = None,
                kd: Optional[np.ndarray] = None) -> None:
        if self.read_only or self._publisher is None:
            raise RuntimeError(f"{self.name}: no command publisher (read_only or not connected)")
        q = np.asarray(q, dtype=float)
        if q.shape != (NUM_JOINTS,) or not np.all(np.isfinite(q)):
            raise ValueError(f"{self.name}: need {NUM_JOINTS} finite targets, got {q!r}")
        q = np.clip(self.clip(q), 0.0, 1.0)
        angles = [int(round(v * SCALE)) for v in q]
        zeros = [0] * NUM_JOINTS
        self._publisher.Write(self._ctrl_type(
            pos_set=zeros, angle_set=angles, force_set=zeros, speed_set=zeros, mode=MODE_ANGLE))

    def reset(self) -> None:
        """Command the measured pose, so the hand holds where it is."""
        if self.read_only:
            raise RuntimeError(f"{self.name} is read_only; reset() would command the hand")
        self.command(self.get_state().q)

    def damp(self) -> None:
        """No-op: the hand has no passive mode and holds its last target when commands stop."""
