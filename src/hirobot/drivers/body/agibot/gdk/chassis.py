"""The AgiBot G1 chassis controller box: state over HTTP, velocity over WebSocket.

The chassis has its own controller box on the robot network. Its
web page reads ``POST /api/AMR/GetState`` (JSON ``{"id": <robot id>}``) and,
while a drive key is held, sends ``ManualMove`` on a WebSocket every 100 ms:
``{"id", "linearSpeed" (mm/s), "angularSpeed" (deg/s), "angleOffset"}``. It
takes these only in its soft-manual mode; this client never switches modes.
"""

from __future__ import annotations

import json
import urllib.request
from typing import Any, Dict, Optional


class ChassisClient:
    """Blocking client for one chassis controller box.

    Args:
        host: Controller address (shown on its web page).
        robot_id: The ``id`` the controller answers to (``780`` on the tested G1).
        state_port: HTTP port of ``GetState``.
        move_port: WebSocket port of ``ManualMove``.
        timeout_s: Per-request timeout.
    """

    def __init__(self, host: str, robot_id: int, state_port: int = 6002, move_port: int = 6001,
                 timeout_s: float = 0.5) -> None:
        self.host = host
        self.robot_id = int(robot_id)
        self.state_url = f"http://{host}:{state_port}/api/AMR/GetState"
        self.move_url = f"ws://{host}:{move_port}/api/AMR/ManualMove"
        self.timeout_s = float(timeout_s)
        self._ws: Any = None
        # The robot computer has HTTP proxies set for the internet; the box is local.
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def get_state(self) -> Dict[str, Any]:
        """The ``Content`` of a ``GetState`` reply; raises on a transport or API error."""
        body = json.dumps({"id": self.robot_id}).encode()
        req = urllib.request.Request(self.state_url, data=body,
                                     headers={"Content-Type": "application/json"})
        with self._opener.open(req, timeout=self.timeout_s) as resp:
            reply = json.loads(resp.read())
        if reply.get("ErrorCode", 0) != 0:
            raise RuntimeError(f"chassis GetState: {reply.get('ErrorMessage')!r} "
                               f"(code {reply.get('ErrorCode')})")
        content = reply.get("Content")
        if not isinstance(content, dict):
            raise RuntimeError("chassis GetState: reply has no Content")
        return content

    def move(self, linear_mm_s: float, angular_deg_s: float) -> None:
        """Send one ``ManualMove``; the box stops on its own when they stop coming."""
        if self._ws is None:
            from websockets.sync.client import connect

            self._ws = connect(self.move_url, open_timeout=self.timeout_s,
                               close_timeout=self.timeout_s)
        self._ws.send(json.dumps({"id": self.robot_id, "linearSpeed": int(round(linear_mm_s)),
                                  "angularSpeed": round(float(angular_deg_s), 2),
                                  "angleOffset": 0}))

    def close(self) -> None:
        ws, self._ws = self._ws, None
        if ws is not None:
            ws.close()


def pose_from_state(content: Dict[str, Any], field: str = "odometer") -> Optional[tuple]:
    """``(x, y, theta)`` in metres and radians from a ``GetState`` pose field (mm, degrees)."""
    pose = content.get(field)
    if not isinstance(pose, dict):
        return None
    import math

    return (float(pose["x"]) / 1000.0, float(pose["y"]) / 1000.0,
            math.radians(float(pose["theta"])))
