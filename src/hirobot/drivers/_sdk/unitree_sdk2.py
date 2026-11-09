"""What every ``unitree_sdk2py`` driver shares: the lazy channel import and DDS init.

The body, the hands, the head camera and the voice service all talk over the
same DDS participant. ``ChannelFactoryInitialize`` is a process-wide singleton
in the SDK: the first call creates the domain on its network interface and
later calls return at once, ignoring their arguments. So whichever driver
connects first picks the NIC; :func:`init_channel` warns when a later driver
asks for a different one, since its setting would be silently dropped.
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Optional, Tuple

logger = logging.getLogger(__name__)

INSTALL = "pip install -e <clone of https://github.com/unitreerobotics/unitree_sdk2_python>"

_lock = threading.Lock()
_initialised: Optional[Tuple[int, Optional[str]]] = None


def channel_api(who: str) -> Tuple[Any, Any, Any]:
    """``(ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)``, imported lazily."""
    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
    except ImportError as exc:  # pragma: no cover - import guard
        raise ImportError(f"{who} needs unitree_sdk2py ({INSTALL}).") from exc
    return ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber


def init_channel(who: str, domain_id: int = 0, network_interface: Optional[str] = None) -> None:
    """Initialise the SDK's DDS channel factory once per process.

    Raises:
        RuntimeError: The SDK could not create the domain (most often a NIC
            name that does not exist on this host).
    """
    global _initialised
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize as init
    except ImportError as exc:  # pragma: no cover - import guard
        raise ImportError(f"{who} needs unitree_sdk2py ({INSTALL}).") from exc
    with _lock:
        if _initialised is not None:
            first_domain, first_nic = _initialised
            if (domain_id, network_interface) != (first_domain, first_nic) and network_interface:
                logger.warning(
                    "%s asked for DDS on %s (domain %d), but the channel factory already runs "
                    "on %s (domain %d); keeping that one. Give every Unitree device the same NIC.",
                    who, network_interface, domain_id, first_nic, first_domain)
            init(domain_id, network_interface)  # a no-op in the SDK; kept for test fakes
            return
        try:
            init(domain_id, network_interface)
        except Exception as exc:
            # The SDK raises a bare Exception("channel factory init error.").
            raise RuntimeError(
                f"{who}: Unitree DDS init failed on network_interface={network_interface!r} "
                f"(domain {domain_id}). Check the NIC name (`ip -br link`) and that it is on "
                "the robot's subnet.") from exc
        _initialised = (domain_id, network_interface)


def _reset_for_tests() -> None:
    """Forget the recorded initialisation (tests install a fresh fake SDK each time)."""
    global _initialised
    with _lock:
        _initialised = None
