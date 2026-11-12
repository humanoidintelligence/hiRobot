"""Franka arm through a DROID robot server (registered as ``franka_droid``).

`DROID <https://github.com/droid-dataset/droid>`_ runs the Franka (and its
Robotiq gripper) from a small server on the control computer, usually a NUC:
``scripts/server/run_server.py`` puts ``droid.franka.robot.FrankaRobot`` behind
`zerorpc <https://github.com/0rpc/zerorpc-python>`_ on port 4242, and the
workstation drives it with ``droid.misc.server_interface.ServerInterface``. This
driver is such a client. It uses only the server methods upstream DROID has:

- ``get_robot_state()`` -> ``(state, timestamps)``; ``state`` has
  ``joint_positions``, ``joint_velocities``, ``cartesian_position``
  (``[x, y, z, roll, pitch, yaw]``, extrinsic ``xyz`` Euler angles, as
  ``scipy``'s ``"xyz"``) and ``gripper_position`` (0 open, 1 closed);
- ``update_joints(q, velocity, blocking)``, ``update_pose(pose, velocity, blocking)``,
  ``update_gripper(command, velocity, blocking)``.

So it needs no DROID code on the hiRobot side, only ``pip install zerorpc``, and
leaves the server as it was. It does not start or stop the server's controller
(``launch_controller`` / ``kill_controller``); start the server the DROID way.

``Observation.gripper`` / ``Action.gripper`` use hiRobot's scale (0 closed,
1 open); DROID's is the other way round, so the driver converts both ways.
Each OS thread gets its own zerorpc client (zerorpc runs on gevent, whose
event loop is per thread), so :meth:`estop` from another thread works.
"""

from __future__ import annotations

import threading
from typing import Any, Optional, Sequence

import numpy as np

from hirobot.core.embodiment import Embodiment
from hirobot.core.robot import Robot
from hirobot.core.stamps import StampCounter
from hirobot.core.types import Action, ControlMode, Observation
from hirobot.geometry.transforms import euler_xyz_to_quat, quat_to_euler_xyz
from hirobot.loading.registry import register_body
from hirobot.loading.sdk import require_sdk


@register_body("franka_droid")
class FrankaDroidRobot(Robot):
    """A Franka arm and its gripper behind a DROID robot server.

    Args:
        embodiment: The 7-joint arm embodiment; its first end-effector is the
            flange / tool frame DROID reports (``ee_poses`` /
            ``Action.ee_targets`` key), and its first end-effector with a
            gripper is the ``Observation.gripper`` / ``Action.gripper`` key.
        ip: Address of the computer running the DROID server.
        port: Server port (DROID uses 4242).
        timeout_s: Seconds one call may take before it raises.
        home_q: Optional home joint configuration (rad) for :meth:`reset`; the
            server moves there with its blocking joint move.
        gripper: Drive the gripper through the server (``update_gripper``).
            ``False`` leaves it alone and reports no gripper.
        network_interface: Ignored (filled in by ``make_robot`` for hardware).
    """

    action_modes = (ControlMode.POSITION, ControlMode.CARTESIAN)

    def __init__(
        self,
        embodiment: Embodiment,
        ip: str = "172.16.0.2",
        port: int = 4242,
        timeout_s: float = 10.0,
        home_q: Optional[Sequence[float]] = None,
        gripper: bool = True,
        network_interface: Optional[str] = None,
    ) -> None:
        self._embodiment = embodiment
        self._address = f"tcp://{ip}:{int(port)}"
        self._timeout_s = float(timeout_s)
        self._home_q = None if home_q is None else [float(v) for v in home_q]
        n = embodiment.num_dofs
        if self._home_q is not None and len(self._home_q) != n:
            raise ValueError(f"home_q has {len(self._home_q)} values; the arm has {n} joints")
        ees = list(embodiment.end_effectors)
        self._ee = ees[0].name if ees else "ee"
        with_gripper = [ee.name for ee in ees if ee.has_gripper]
        self._gripper = (with_gripper[0] if with_gripper else self._ee) if gripper else None
        self._zerorpc: Any = None
        self._local = threading.local()
        self._clients: list = []
        self._clients_lock = threading.Lock()
        self._connected = False
        self._stamps = StampCounter()

    @property
    def embodiment(self) -> Embodiment:
        return self._embodiment

    # ------------------------------------------------------------ transport

    def _client(self) -> Any:
        client = getattr(self._local, "client", None)
        if client is None:
            client = self._zerorpc.Client(timeout=self._timeout_s, heartbeat=None)
            client.connect(self._address)
            self._local.client = client
            with self._clients_lock:
                self._clients.append(client)
        return client

    def _call(self, method: str, *args: Any) -> Any:
        if not self._connected:
            raise RuntimeError("Not connected; call connect() first.")
        return self._client()(method, *args)

    def connect(self) -> None:
        require_sdk(__package__, "FrankaDroidRobot")
        try:
            import zerorpc
        except ImportError as exc:  # pragma: no cover - import guard
            raise ImportError("FrankaDroidRobot needs zerorpc; install it with 'pip install zerorpc'.") from exc
        self._zerorpc = zerorpc
        self._connected = True
        try:
            self._read_state()  # fail here, not on the first control step
        except Exception as exc:
            self.disconnect()
            raise ConnectionError(f"DROID server at {self._address} did not answer "
                                  f"get_robot_state: {exc}") from exc

    def disconnect(self) -> None:
        self._connected = False
        with self._clients_lock:
            clients, self._clients = self._clients, []
        for client in clients:
            try:
                client.close()
            except Exception:  # noqa: BLE001 - closing anyway
                pass
        self._local = threading.local()

    # ------------------------------------------------------------ reading

    def _read_state(self) -> dict:
        reply = self._call("get_robot_state")
        state = reply[0] if isinstance(reply, (list, tuple)) else reply
        n = self._embodiment.num_dofs
        q = np.asarray(state["joint_positions"], dtype=float)
        if q.shape != (n,):
            raise RuntimeError(f"DROID server sent {q.shape[0]} joint positions; expected {n}")
        return state

    def get_observation(self) -> Observation:
        state = self._read_state()
        stamp = self._stamps.tick()
        vel = state.get("joint_velocities")
        obs = Observation(
            joint_pos=np.asarray(state["joint_positions"], dtype=float),
            joint_vel=None if vel is None else np.asarray(vel, dtype=float),
        )
        pose = state.get("cartesian_position")
        if pose is not None and len(pose) == 6:
            obs.ee_poses[self._ee] = np.concatenate(
                [np.asarray(pose[:3], dtype=float), euler_xyz_to_quat(pose[3:6])])
        closed = state.get("gripper_position")
        if self._gripper is not None and closed is not None:
            obs.gripper[self._gripper] = float(np.clip(1.0 - float(closed), 0.0, 1.0))
            obs.stamps[self._gripper] = stamp
        obs.stamps["body"] = stamp
        return obs

    # ------------------------------------------------------------ commands

    def reset(self, seed: Optional[int] = None) -> None:
        """Move to ``home_q`` with the server's blocking joint move, if one is set."""
        if self._home_q is None or not self._connected:
            return
        token = self._stop_token()
        with self._commanding(token):
            self._call("update_joints", self._home_q, False, True)
        self._check_stop(token)

    def _estop(self) -> None:
        """Hold the measured joint positions (a target at where the arm is)."""
        if not self._connected:
            return
        q = self._read_state()["joint_positions"]
        self._call("update_joints", [float(v) for v in q], False, False)

    def _send_action(self, action: Action) -> None:
        if not self._connected:
            raise RuntimeError("Not connected; call connect() first.")
        if action.mode == ControlMode.POSITION:
            if action.joint_target is not None:
                q = np.asarray(action.joint_target, dtype=float)
                self._call("update_joints", q.tolist(), False, False)
        elif action.mode == ControlMode.CARTESIAN:
            target = action.ee_targets.get(self._ee) if action.ee_targets else None
            if target is not None:
                target = np.asarray(target, dtype=float)
                pose = np.concatenate([target[:3], quat_to_euler_xyz(target[3:7])])
                self._call("update_pose", pose.tolist(), False, False)
        else:
            raise ValueError(f"FrankaDroidRobot does not support mode {action.mode!r}")
        opening = action.gripper.get(self._gripper) if (action.gripper and self._gripper) else None
        if opening is not None:
            opening = float(opening)
            if not 0.0 <= opening <= 1.0:
                raise ValueError(f"gripper {self._gripper!r}: {opening} is outside [0, 1]")
            self._call("update_gripper", 1.0 - opening, False, False)
