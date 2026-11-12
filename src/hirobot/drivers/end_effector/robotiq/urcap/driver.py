"""Robotiq 2F grippers over the UR controller's URCap socket (:63352).

When Robotiq's *UR Cap* is installed on a Universal Robots controller it opens a
plain TCP server on the controller, by default port ``63352``, that speaks the
gripper's ASCII register protocol. A line ``SET <var> <val> [...]`` writes one or
more registers and is answered ``ack``; ``GET <var>`` is answered ``<var> <val>``.
So a gripper bolted to a UR arm needs no separate wiring or SDK: hiRobot reaches
it through the same controller that runs the arm, at the arm's IP.

The registers used here (Robotiq 2F-85 / 2F-140, ``rq_`` URScript names):

* ``ACT`` activate (0/1), ``STA`` status (0 reset, 3 activated)
* ``GTO`` go-to (1 moves to the requested position, 0 holds)
* ``POS`` position, 0-255: ``SET POS`` writes the request, ``GET POS`` reads the
  actual position. ``PRE`` echoes the request and is read-only (a ``SET PRE``
  is acknowledged but moves nothing).
* ``SPE`` speed, ``FOR`` force, both 0-255
* ``OBJ`` object detection (0 moving, 1/2 stopped on an object, 3 at target)
* ``FLT`` fault code (0 = none)

Robotiq counts position the other way from hiRobot: register 0 is fully **open**,
255 fully **closed**. The driver maps the gripper's ``opening`` interface
(``Observation.gripper`` / ``Action.gripper``, 0 closed, 1 open) to it as
``opening = (255 - POS) / 255`` on the way in and ``POS = round((1 - opening) * 255)``
on the way out, so a hiRobot opening of 1.0 drives the register to 0 (open).

A background thread polls ``POS`` / ``OBJ`` / ``FLT`` so :meth:`get_state` returns
the latest reading without a round-trip. :meth:`command_opening` writes ``POS`` /
``SPE`` / ``FOR`` / ``GTO`` and returns at once; it does not wait for the motion.
There is no passive/torque-off mode on a Robotiq 2F, so :meth:`damp` sends
``GTO 0`` to halt an in-progress motion and hold.

Only the Python standard library is used (a TCP socket), so this driver needs no
SDK install.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Optional, Sequence

import numpy as np

from hirobot.core.embodiment import Joint
from hirobot.core.stamps import Stamp, StampCounter
from hirobot.devices.end_effector import (
    EndEffectorDriver,
    OpeningState,
    check_opening,
    register_end_effector,
)

DEFAULT_PORT = 63352
SCALE = 255.0           # Robotiq position/speed/force registers are 0-255
STATUS_ACTIVATED = 3    # STA == 3: activation finished

@register_end_effector("robotiq_urcap")
class RobotiqUrcapGripper(EndEffectorDriver):
    """A Robotiq 2F gripper reached through a UR controller's URCap socket.

    Args:
        name: End-effector name (its key in ``Observation.gripper`` / ``Action.gripper``).
        joints: Must be empty; this driver is the ``opening`` interface only.
        host: The UR controller's IP (the same address as the arm's body driver).
        port: The URCap socket port (Robotiq default ``63352``).
        speed: Default move speed register (0-255) used when commanding.
        force: Default grip force register (0-255).
        activate: Send ``ACT``/``GTO`` on connect and wait for ``STA 3`` if the
            gripper is not already activated. An activation move closes then opens
            the gripper fully, so leave it ``True`` only when the workspace is clear.
        connect_timeout_s: How long to wait for the first reading (and for
            activation, when requested).
        stale_s: Reject a reading older than this in :meth:`get_state`.
        poll_hz: Background state-poll rate.
        read_only: Never open a command path; any command raises.
        network_interface: Ignored (accepted so the builder may pass the body NIC).
    """

    interfaces = ("opening",)

    def __init__(
        self,
        name: str,
        joints: Sequence[Joint],
        host: str,
        port: int = DEFAULT_PORT,
        speed: int = 255,
        force: int = 255,
        activate: bool = False,
        connect_timeout_s: float = 3.0,
        stale_s: float = 0.5,
        poll_hz: float = 20.0,
        read_only: bool = False,
        network_interface: Optional[str] = None,
    ) -> None:
        super().__init__(name, joints, read_only=read_only)
        if self.num_joints:
            raise ValueError(f"{name}: a Robotiq gripper is the 'opening' interface; "
                             f"drop joints and set has_gripper: true")
        self.host = host
        self.port = int(port)
        self._speed = int(np.clip(speed, 0, 255))
        self._force = int(np.clip(force, 0, 255))
        self._activate = bool(activate)
        self._connect_timeout_s = float(connect_timeout_s)
        self._stale_s = float(stale_s)
        self._poll_dt = 1.0 / float(poll_hz)
        self._sock: Optional[socket.socket] = None
        self._io_lock = threading.Lock()       # one request/reply on the socket at a time
        self._state_lock = threading.Lock()
        self._stamps = StampCounter()
        self._pos: Optional[int] = None         # raw POS (0 open, 255 closed)
        self._obj = 0
        self._flt = 0
        self._stamp: Optional[Stamp] = None
        self._poller: Optional[threading.Thread] = None
        self._stop_poll = threading.Event()

    # ------------------------------------------------------------ socket I/O

    def _request(self, line: str) -> str:
        """Send one ``SET``/``GET`` line and return the controller's reply."""
        if self._sock is None:
            raise RuntimeError(f"{self.name}: not connected")
        with self._io_lock:
            self._sock.sendall(line.encode("ascii") + b"\n")
            data = self._sock.recv(1024)
        return data.decode("ascii", "replace").strip()

    def _get_int(self, var: str) -> int:
        reply = self._request(f"GET {var}")
        parts = reply.split()
        if len(parts) != 2 or parts[0] != var:
            raise RuntimeError(f"{self.name}: unexpected reply to GET {var}: {reply!r}")
        return int(parts[1])

    def _poll_once(self) -> None:
        pos, obj, flt = self._get_int("POS"), self._get_int("OBJ"), self._get_int("FLT")
        with self._state_lock:
            self._pos, self._obj, self._flt = pos, obj, flt
            self._stamp = self._stamps.tick()

    def _poll_loop(self) -> None:
        while not self._stop_poll.is_set():
            try:
                self._poll_once()
            except OSError:
                pass        # a transient socket error; the staleness check covers a real drop
            self._stop_poll.wait(self._poll_dt)

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        if self._sock is None:
            self._sock = socket.create_connection((self.host, self.port),
                                                  timeout=self._connect_timeout_s)
            self._sock.settimeout(self._connect_timeout_s)
        if self._activate and not self.read_only:
            self._do_activate()
        self._poll_once()
        if self._poller is None:
            self._stop_poll.clear()
            self._poller = threading.Thread(target=self._poll_loop, name=f"{self.name}-poll",
                                            daemon=True)
            self._poller.start()

    def _do_activate(self) -> None:
        if self._get_int("STA") == STATUS_ACTIVATED:
            return
        self._request("SET ACT 1")
        self._request("SET GTO 1")
        deadline = time.monotonic() + self._connect_timeout_s
        while time.monotonic() < deadline:
            if self._get_int("STA") == STATUS_ACTIVATED:
                return
            time.sleep(0.05)
        raise TimeoutError(f"{self.name}: gripper did not activate (STA != {STATUS_ACTIVATED}) "
                           f"within {self._connect_timeout_s:.0f}s")

    def get_state(self) -> OpeningState:
        with self._state_lock:
            pos, obj, flt, stamp = self._pos, self._obj, self._flt, self._stamp
        if pos is None or stamp is None:
            raise RuntimeError(f"{self.name}: no state yet; call connect() first")
        age = time.monotonic() - stamp.t
        if age > self._stale_s:
            raise TimeoutError(f"{self.name}: newest state is {age:.2f}s old (> {self._stale_s}s); "
                               f"is the gripper still reachable at {self.host}:{self.port}?")
        opening = (SCALE - pos) / SCALE         # register 0 = open -> opening 1.0
        return OpeningState(float(np.clip(opening, 0.0, 1.0)), stamp,
                            extra={"object": float(obj), "fault": float(flt)})

    # ------------------------------------------------------------- commands

    def command_opening(self, opening: float) -> None:
        if self.read_only or self._sock is None:
            raise RuntimeError(f"{self.name}: no command path (read_only or not connected)")
        opening = check_opening(opening)
        pos = int(round((1.0 - opening) * SCALE))   # opening 1.0 -> register 0 (open)
        # SET POS writes the request register (rPR); GET POS reads the actual
        # position (gPO). PRE is a read-only echo of the request, not writable.
        self._request(f"SET POS {pos} SPE {self._speed} FOR {self._force} GTO 1")

    def reset(self) -> None:
        """Hold the current opening (no jump): command where the gripper already is."""
        if self.read_only:
            raise RuntimeError(f"{self.name} is read_only; reset() would command the gripper")
        self.command_opening(self.get_state().opening)

    def damp(self) -> None:
        """Halt any motion and hold. A Robotiq 2F has no torque-off; this sends ``GTO 0``."""
        if self.read_only or self._sock is None:
            return
        try:
            self._request("SET GTO 0")
        except OSError:
            pass

    def disconnect(self) -> None:
        self._stop_poll.set()
        if self._poller is not None:
            self._poller.join(timeout=1.0)
            self._poller = None
        self.damp()
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
