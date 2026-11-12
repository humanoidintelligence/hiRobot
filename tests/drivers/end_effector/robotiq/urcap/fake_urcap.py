"""A fake Robotiq URCap socket server for the ``robotiq_urcap`` driver tests.

Listens on a loopback port and speaks the controller's ASCII register protocol:
``SET <var> <val> [...]`` -> ``ack`` and ``GET <var>`` -> ``<var> <val>``. It models
what the real URCap does with position (checked on a 2F gripper): ``SET POS``
writes the request register, ``GET POS`` reads the actual position, ``GET PRE``
echoes the request, and ``SET PRE`` is acknowledged but changes nothing. With
``GTO 1`` the fake moves to the request at once. ``sets`` counts the ``SET`` lines
that carried a motion command (``GTO`` / ``POS``), standing in for "commands that
reached the gripper".
"""

from __future__ import annotations

import socket
import threading
from typing import Dict

READ_ONLY = {"PRE", "STA", "OBJ", "FLT"}


class FakeUrcap:
    def __init__(self, pos: int = 0) -> None:
        # pos 0 = fully open; STA 3 = activated, no fault
        self.actual = pos                      # what GET POS returns
        self.reg: Dict[str, int] = {"PRE": pos, "SPE": 255, "FOR": 0,
                                    "STA": 3, "GTO": 0, "OBJ": 3, "FLT": 0, "ACT": 1}
        self.sets = 0
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.host, self.port = self._srv.getsockname()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _handle(self, line: str) -> bytes:
        parts = line.split()
        if not parts:
            return b""
        if parts[0] == "GET":
            var = parts[1]
            val = self.actual if var == "POS" else self.reg.get(var, 0)
            return f"{var} {val}\n".encode()
        if parts[0] == "SET":
            toks = parts[1:]
            moved = False
            for i in range(0, len(toks) - 1, 2):
                var, val = toks[i], int(toks[i + 1])
                if var in READ_ONLY:
                    continue
                if var == "POS":
                    self.reg["PRE"] = val      # the request, echoed by GET PRE
                else:
                    self.reg[var] = val
                if var in ("GTO", "POS"):
                    moved = True
            if self.reg.get("GTO") == 1:
                self.actual = self.reg["PRE"]  # move completes instantly
            if moved:
                self.sets += 1
            return b"ack\n"
        return b""

    def _serve(self) -> None:
        self._srv.settimeout(0.2)
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                conn.settimeout(0.2)
                buf = b""
                while not self._stop.is_set():
                    try:
                        data = conn.recv(1024)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        conn.sendall(self._handle(line.decode("ascii", "replace").strip()))

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._srv.close()
