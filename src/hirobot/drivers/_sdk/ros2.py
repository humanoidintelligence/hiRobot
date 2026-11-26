"""What every ROS 2 driver shares: the rclpy context, one executor thread, message lookup.

``rclpy.init()`` is per process and ``rclpy.shutdown()`` ends it for every node,
so a body and (later) a camera driver in the same process must share it.
:func:`acquire` starts the context and a background ``SingleThreadedExecutor``
on the first call; :func:`release` removes a driver's node and shuts the
context down when the last one leaves. Callbacks run on that thread, so a
driver keeps only the latest message under a lock and reads it from
``get_observation``.

rclpy and the message packages come with a ROS 2 install (``source
/opt/ros/<distro>/setup.bash``), never from pip, and are imported only here,
when a driver connects.
"""

from __future__ import annotations

import importlib
import threading
from typing import Any, Optional

INSTALL = "source a ROS 2 install (rclpy comes with it): source /opt/ros/<distro>/setup.bash"

_lock = threading.Lock()
_users = 0
_owns_context = False
_executor: Any = None
_thread: Optional[threading.Thread] = None
_nodes: list = []  # nodes added to the executor


def import_rclpy(who: str) -> Any:
    """``rclpy``, imported lazily, with an error that says how to get it."""
    try:
        import rclpy
    except ImportError as exc:  # pragma: no cover - import guard
        raise ImportError(f"{who} needs rclpy ({INSTALL}).") from exc
    return rclpy


def load_msg(type_name: str, who: str = "ros2") -> Any:
    """The message class for ``pkg/msg/Name`` (``pkg/Name`` also accepted).

    Raises:
        ValueError: ``type_name`` is not of that form.
        ImportError: The package is not on the path, or has no such message.
    """
    parts = type_name.split("/")
    if len(parts) == 2:
        parts = [parts[0], "msg", parts[1]]
    if len(parts) != 3 or parts[1] != "msg" or not all(parts):
        raise ValueError(f"{who}: message type {type_name!r} must look like 'pkg/msg/Name'")
    pkg, _, name = parts
    try:
        module = importlib.import_module(f"{pkg}.msg")
    except ImportError as exc:
        raise ImportError(
            f"{who}: cannot import {pkg}.msg for {type_name!r}. Build the package that defines it "
            f"and source its workspace (source <ws>/install/setup.bash).") from exc
    try:
        return getattr(module, name)
    except AttributeError:
        raise ImportError(f"{who}: {pkg}.msg has no message {name!r}") from None


def acquire(who: str) -> Any:
    """Join the shared ROS 2 context; returns ``rclpy``. Pair with :func:`release`."""
    global _users, _owns_context, _executor, _thread
    rclpy = import_rclpy(who)
    with _lock:
        if _users == 0:
            # Someone else (the application) may have started rclpy already;
            # then it is theirs to shut down.
            _owns_context = not rclpy.ok()
            if _owns_context:
                rclpy.init()
            from rclpy.executors import SingleThreadedExecutor

            _executor = SingleThreadedExecutor()
            _thread = threading.Thread(target=_spin, args=(rclpy, _executor),
                                       name="hirobot-ros2", daemon=True)
            _thread.start()
        _users += 1
    return rclpy


def _spin(rclpy: Any, executor: Any) -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0.05)
        except Exception:  # pragma: no cover - shutdown races
            if not rclpy.ok():
                break
            raise


def add_node(node: Any) -> None:
    """Have the shared executor run ``node``'s callbacks."""
    with _lock:
        if _executor is None:
            raise RuntimeError("ros2: acquire() first")
        _executor.add_node(node)
        _nodes.append(node)


def release(node: Any = None) -> None:
    """Leave the shared context: remove and destroy ``node``; the last one out shuts it down."""
    global _users, _owns_context, _executor, _thread
    with _lock:
        if _users == 0:
            return
        _users -= 1
        if node is not None:
            # A driver whose connect() failed part-way releases a node it never added.
            if any(n is node for n in _nodes):
                _nodes.remove(node)
                _executor.remove_node(node)
            node.destroy_node()
        if _users:
            return
        executor, thread, owns = _executor, _thread, _owns_context
        _executor = _thread = None
        _owns_context = False
    import rclpy

    if owns and rclpy.ok():
        rclpy.shutdown()
    executor.shutdown()
    if thread is not None:
        thread.join(timeout=1.0)


def users() -> int:
    """How many drivers hold the shared context (for tests)."""
    return _users
