"""Hands behind a Unitree serial-to-DDS service: ``unitree_go`` ``MotorCmds_`` / ``MotorStates_``.

Unitree bridges several third-party hands onto DDS with the same message pair
(``dfx_inspire_service``, ``brainco_hand_service``, ``linker_hand_service``,
``dex1_1_service``): a service on the computer the hand's serial adapter is
plugged into publishes ``MotorStates_`` and applies ``MotorCmds_``. Only ``q``
is used, in whatever unit the service defines (Inspire: 0 = closed, 1 = open).
So one driver covers them all and each hand is a config: which topics, and
which motor slots in each message belong to this hand.

A message can carry more than one hand. Unitree's ``h1_inspire_service``
**1.0.1** puts both Inspire hands on ``rt/inspire/{cmd,state}``, 12 motors:
0-5 the right hand, 6-11 the left (``num_motors: 12``, ``motor_offset: 0`` /
``6``). The hands then share one :class:`_MotorBus`: commanding one hand
rewrites its slots and republishes the other hand's last target, seeded from
its measured pose, so it never moves the other hand. (Version 1.0.2 writes a
zero-initialised command buffer on start and closes every finger; use 1.0.1.)

The services have no passive mode: a hand holds its last target when commands
stop, so :meth:`UnitreeMotorCmdsHand.damp` sends nothing.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from hirobot.drivers._sdk.unitree_sdk2 import channel_api, init_channel
from hirobot.embodiment import Joint
from hirobot.stamps import Stamp, StampCounter
from hirobot.end_effector import EndEffectorDriver, JointsState, register_end_effector
from hirobot.sdk import require_sdk


class _Latest:
    """The newest message on one subscribed topic and its stamp."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.msg = None
        self._stamps = StampCounter()
        self.stamp: Optional[Stamp] = None

    def __call__(self, msg) -> None:
        with self._lock:
            self.msg = msg
            self.stamp = self._stamps.tick()

    def latest(self) -> Tuple[object, Optional[Stamp]]:
        with self._lock:
            return self.msg, self.stamp


class _MotorBus:
    """One ``MotorCmds_`` / ``MotorStates_`` topic pair, shared by the hands on it.

    Holds the latest state and the targets last sent. Targets start as NaN; a
    hand's slots fill in when it is first commanded, and every other slot is
    seeded from its measured value at that moment.
    """

    def __init__(self, cmd_topic: str, state_topic: str, num_motors: int, domain_id: int,
                 network_interface: Optional[str]) -> None:
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.num_motors = num_motors
        self._domain_id = domain_id
        self._network_interface = network_interface
        self._lock = threading.Lock()  # guards targets
        self._open_lock = threading.Lock()  # serialises channel creation
        self.state = _Latest()
        self.targets = np.full(num_motors, np.nan)
        self._subscriber = None
        self._publisher = None
        self._cmd = None

    def open(self, want_publisher: bool) -> None:
        who = "UnitreeMotorCmdsHand"
        _, Publisher, Subscriber = channel_api(who)
        try:
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError(f"{who} needs unitree_sdk2py's unitree_go IDL.") from exc
        with self._open_lock:
            if self._subscriber is None:
                init_channel(who, self._domain_id, self._network_interface)
                self._subscriber = Subscriber(self.state_topic, MotorStates_)
                self._subscriber.Init(self.state, 10)
            if want_publisher and self._publisher is None:
                self._cmd = MotorCmds_(cmds=[unitree_go_msg_dds__MotorCmd_()
                                             for _ in range(self.num_motors)])
                self._publisher = Publisher(self.cmd_topic, MotorCmds_)
                self._publisher.Init()

    def latest(self) -> Tuple[object, Optional[Stamp]]:
        return self.state.latest()

    @property
    def can_write(self) -> bool:
        return self._publisher is not None

    def measured_q(self) -> np.ndarray:
        """Every slot's measured position, NaN where the state is short or missing."""
        msg, _ = self.latest()
        q = np.full(self.num_motors, np.nan)
        if msg is not None:
            states = list(msg.states)[:self.num_motors]
            q[:len(states)] = [float(s.q) for s in states]
        return q

    def write(self, offset: int, q: np.ndarray) -> None:
        """Set one hand's targets and publish every slot."""
        if self._publisher is None:
            raise RuntimeError(f"{self.cmd_topic}: no command publisher")
        measured = self.measured_q()
        with self._lock:
            self.targets[offset:offset + len(q)] = q
            # Slots no hand has commanded yet hold their measured pose.
            unset = ~np.isfinite(self.targets)
            self.targets[unset] = measured[unset]
            if not np.all(np.isfinite(self.targets)):
                raise RuntimeError(
                    f"{self.cmd_topic}: no measured pose for every motor on "
                    f"{self.state_topic}; refusing to command the other slots blind")
            for i, m in enumerate(self._cmd.cmds):
                m.q = float(self.targets[i])
            self._publisher.Write(self._cmd)


_BUSES: Dict[Tuple[str, str, int, int, Optional[str]], _MotorBus] = {}
_BUSES_LOCK = threading.Lock()


def _bus(cmd_topic: str, state_topic: str, num_motors: int, domain_id: int,
         network_interface: Optional[str]) -> _MotorBus:
    key = (cmd_topic, state_topic, num_motors, domain_id, network_interface)
    with _BUSES_LOCK:
        if key not in _BUSES:
            _BUSES[key] = _MotorBus(*key)
        bus = _BUSES[key]
    return bus


@register_end_effector("unitree_motorcmds")
class UnitreeMotorCmdsHand(EndEffectorDriver):
    """One hand on a ``MotorCmds_`` / ``MotorStates_`` topic pair (see the module docstring).

    Args:
        name: End-effector name.
        joints: The hand's embodiment joints, one per motor slot, in slot order.
            Positions are in the service's unit; give them ``position_limit``
            so commands are clipped to the hand's range.
        cmd_topic / state_topic: The service's topics.
        motor_offset: Slot of this hand's first joint in each message.
        num_motors: Motors per message (default: this hand's joint count). Set
            it when several hands share one topic pair.
        service: The bridge's name, quoted in the error when no state arrives.
        network_interface: NIC on the robot subnet. The DDS channel is shared
            with the body driver; whichever connects first sets it.
        domain_id: DDS domain id.
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
        cmd_topic: str,
        state_topic: str,
        motor_offset: int = 0,
        num_motors: Optional[int] = None,
        service: str = "the hand's serial-to-DDS service",
        network_interface: Optional[str] = None,
        domain_id: int = 0,
        connect_timeout_s: float = 2.0,
        stale_s: float = 0.5,
        read_only: bool = False,
        default_kp=None,
        default_kd=None,
    ) -> None:
        super().__init__(name, joints, read_only=read_only)
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.offset = int(motor_offset)
        self.num_motors = int(num_motors) if num_motors is not None else self.num_joints
        if self.offset < 0 or self.offset + self.num_joints > self.num_motors:
            raise ValueError(
                f"{name}: joints {self.offset}..{self.offset + self.num_joints - 1} do not fit "
                f"in a {self.num_motors}-motor message")
        self.service = service
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._connect_timeout_s = float(connect_timeout_s)
        self._stale_s = float(stale_s)
        self._link: Optional[_MotorBus] = None

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        require_sdk(__package__, "UnitreeMotorCmdsHand")
        link = _bus(self.cmd_topic, self.state_topic, self.num_motors, self._domain_id,
                    self._network_interface)
        link.open(want_publisher=not self.read_only)
        deadline = time.monotonic() + self._connect_timeout_s
        while link.latest()[0] is None and time.monotonic() < deadline:
            time.sleep(0.005)
        if link.latest()[0] is None:
            raise TimeoutError(
                f"{self.name}: no state on {self.state_topic} within "
                f"{self._connect_timeout_s:.0f} s; is {self.service} running?")
        self._link = link

    @property
    def raw_state(self):
        """The latest raw ``MotorStates_`` (diagnostics), or ``None``."""
        return self._link.latest()[0] if self._link else None

    def get_state(self) -> JointsState:
        if self._link is None:
            raise RuntimeError(f"{self.name}: no state yet; call connect() first")
        msg, stamp = self._link.latest()
        age = time.monotonic() - stamp.t
        if age > self._stale_s:
            raise TimeoutError(f"{self.name}: newest state is {age:.2f}s old (> {self._stale_s}s)")
        n = self.num_joints
        states = list(msg.states)[self.offset:self.offset + n]
        if len(states) != n:
            raise RuntimeError(f"{self.name}: MotorStates_ has {len(msg.states)} motors, "
                               f"need {self.offset + n}")
        extra = {}
        if all(hasattr(s, "lost") for s in states):
            # Failed serial reads of this motor since the service started.
            extra["lost"] = np.array([s.lost for s in states], dtype=float)
        return JointsState(q=np.array([s.q for s in states], dtype=float), dq=np.zeros(n),
                         tau=np.full(n, np.nan), stamp=stamp, extra=extra)

    # ------------------------------------------------------------- commands

    def command(self, q: np.ndarray, kp: Optional[np.ndarray] = None,
                kd: Optional[np.ndarray] = None) -> None:
        if self.read_only or self._link is None or not self._link.can_write:
            raise RuntimeError(f"{self.name}: no command publisher (read_only or not connected)")
        q = np.asarray(q, dtype=float)
        if q.shape != (self.num_joints,) or not np.all(np.isfinite(q)):
            raise ValueError(f"{self.name}: need {self.num_joints} finite targets, got {q!r}")
        self._link.write(self.offset, self.clip(q))

    def reset(self) -> None:
        """Command the measured pose, so the hand holds where it is."""
        if self.read_only:
            raise RuntimeError(f"{self.name} is read_only; reset() would command the hand")
        self.command(self.get_state().q)

    def damp(self) -> None:
        """No-op: the service has no passive mode; the hand holds its last target."""
