"""What the host running hiRobot has: network cards, vendor SDKs, ROS 2, cameras, audio.

:func:`scan_host` only reads: files under ``/sys``, ``/proc`` and ``/etc``, ``ip -j -4 addr``,
package metadata, and (when installed) ``pyrealsense2`` / ``pyzed`` / ``sounddevice`` device
lists. It never changes a setting; anything that needs root is returned as a suggestion.
Every piece is wrapped: a failure leaves that field empty or null and adds a line to
``notes``, so the scan itself never raises.

The helpers take a ``root`` path (default ``/``) so tests can point them at a fake tree.
"""

from __future__ import annotations

import importlib.util
import ipaddress
import json
import os
import platform
import re
import subprocess
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

UNITREE_SUBNET = ipaddress.ip_network("192.168.123.0/24")
# Host address suggested for a NIC with no address on the Unitree subnet. The robot's own
# boards use .161 - .167; .222 is the one the Unitree guides give a development PC.
SUGGESTED_HOST = "192.168.123.222"
UNITREE_SDK = "unitree_sdk2py"
_DRIVERS_LINE = re.compile(r"^#\s*Drivers registered here:\s*(.+)$", re.MULTILINE)


def _read(path: Path) -> Optional[str]:
    try:
        return path.read_text(errors="replace").strip()
    except OSError:
        return None


def _guard(notes: List[str], what: str, fn: Callable[[], Any], default: Any) -> Any:
    """``fn()``, or ``default`` plus a note when it raises."""
    try:
        return fn()
    except Exception as exc:  # noqa: BLE001 - a scan reports failures, never raises
        notes.append(f"{what}: {type(exc).__name__}: {exc}")
        return default


# -- system -----------------------------------------------------------------------------


def jetson_model(root: str = "/") -> Optional[str]:
    """The Jetson board name, or ``None`` on anything else."""
    model = _read(Path(root, "proc/device-tree/model"))
    if model:
        model = model.replace("\x00", "").strip()
        if "jetson" in model.lower() or "nvidia" in model.lower():
            return model
    release = _read(Path(root, "etc/nv_tegra_release"))
    if release:
        first = release.splitlines()[0].lstrip("# ").strip()
        return f"NVIDIA Jetson ({first})" if first else "NVIDIA Jetson"
    return None


# -- network ----------------------------------------------------------------------------


def ip_addresses() -> Optional[Dict[str, List[str]]]:
    """``{nic: ["a.b.c.d/len", ...]}`` from ``ip -j -4 addr``; ``None`` if ``ip`` failed."""
    try:
        out = subprocess.run(["ip", "-j", "-4", "addr"], capture_output=True, text=True,
                             timeout=3.0, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return parse_ip_json(out)


def parse_ip_json(text: str) -> Dict[str, List[str]]:
    """Parse ``ip -j -4 addr`` output into ``{nic: ["a.b.c.d/len", ...]}``."""
    result: Dict[str, List[str]] = {}
    for link in json.loads(text or "[]"):
        name = link.get("ifname")
        if not name:
            continue
        addrs = result.setdefault(name, [])
        for info in link.get("addr_info") or []:
            if info.get("family", "inet") == "inet" and info.get("local"):
                addrs.append(f"{info['local']}/{info.get('prefixlen', 32)}")
    return result


def on_unitree_subnet(addresses: List[str]) -> bool:
    """True when any ``a.b.c.d/len`` in ``addresses`` lies in 192.168.123.0/24."""
    for text in addresses:
        try:
            if ipaddress.ip_interface(text).ip in UNITREE_SUBNET:
                return True
        except ValueError:
            continue
    return False


def interfaces(root: str = "/", addresses: Optional[Dict[str, List[str]]] = None,
               notes: Optional[List[str]] = None) -> List[Dict[str, Any]]:
    """Network cards from ``<root>/sys/class/net`` (``lo`` skipped), with their IPv4."""
    notes = notes if notes is not None else []
    if addresses is None:
        addresses = ip_addresses()
        if addresses is None:
            notes.append("'ip -j -4 addr' failed or is missing; IPv4 addresses not read")
            addresses = {}
    base = Path(root, "sys/class/net")
    out = []
    for path in sorted(base.iterdir()) if base.is_dir() else []:
        name = path.name
        if name == "lo":
            continue
        operstate = (_read(path / "operstate") or "").lower()
        speed: Optional[int] = None
        text = _read(path / "speed")
        if text and text.lstrip("-").isdigit() and int(text) > 0:
            speed = int(text)
        ipv4 = list(addresses.get(name, []))
        out.append({
            "name": name,
            # Tunnels report "unknown" while working; count them up when they have an address.
            "up": operstate == "up" or (operstate == "unknown" and bool(ipv4)),
            "ipv4": ipv4,
            "mac": _read(path / "address") or None,
            "speed_mbps": speed,
            "unitree_subnet": on_unitree_subnet(ipv4),
        })
    return out


def _is_wired(root: str, name: str) -> bool:
    path = Path(root, "sys/class/net", name)
    return (path / "device").exists() and not (path / "wireless").exists()


def _has_carrier(root: str, name: str) -> bool:
    return _read(Path(root, "sys/class/net", name, "carrier")) == "1"


# -- SDKs -------------------------------------------------------------------------------


def _drivers_dir() -> Optional[Path]:
    spec = importlib.util.find_spec("hirobot.drivers")
    locations = list(spec.submodule_search_locations or []) if spec else []
    return Path(locations[0]) if locations else None


def manifest_files(drivers_dir: Optional[Path] = None) -> List[Path]:
    """Every ``manifest.yaml`` under ``hirobot/drivers``, sorted."""
    base = drivers_dir if drivers_dir is not None else _drivers_dir()
    return sorted(base.rglob("manifest.yaml")) if base and base.is_dir() else []


def _driver_names(path: Path) -> List[str]:
    m = _DRIVERS_LINE.search(_read(path) or "")
    if m:
        return [n.strip() for n in m.group(1).split(",") if n.strip()]
    return [path.parent.name]


def sdks(drivers_dir: Optional[Path] = None,
         notes: Optional[List[str]] = None) -> List[Dict[str, Any]]:
    """One entry per SDK distribution any driver manifest lists, with what is installed.

    ``installed`` is the version string, ``"unknown"`` when the module imports but has no
    package metadata (a source checkout), or ``None`` when it is not installed. ``ok`` is
    True when installed and not older than the highest ``minimum`` any manifest asks for.
    """
    from hirobot.loading import sdk as sdk_mod

    notes = notes if notes is not None else []
    base = drivers_dir if drivers_dir is not None else _drivers_dir()
    groups: Dict[str, Dict[str, Any]] = {}
    for path in manifest_files(base):
        package = ".".join(("hirobot", "drivers") + path.parent.relative_to(base).parts)
        try:
            manifest = sdk_mod.load_manifest(package)
        except Exception as exc:  # noqa: BLE001
            notes.append(f"{package}/manifest.yaml: {type(exc).__name__}: {exc}")
            continue
        for req in manifest.sdk:
            g = groups.setdefault(req.distribution, {
                "distribution": req.distribution, "minimum": None, "install": None,
                "drivers": [], "_req": req})
            if req.minimum and (g["minimum"] is None or sdk_mod.parse_version(req.minimum)
                                > sdk_mod.parse_version(g["minimum"])):
                g["minimum"] = req.minimum
            g["install"] = g["install"] or req.install
            for name in _driver_names(path):
                if name not in g["drivers"]:
                    g["drivers"].append(name)
    out = []
    for dist in sorted(groups):
        g = groups.pop(dist)
        req = g.pop("_req")
        try:
            version = sdk_mod.installed_version(req)
            installed: Optional[str] = version if version is not None else "unknown"
        except ImportError:
            installed = None
        except Exception as exc:  # noqa: BLE001
            notes.append(f"sdk {dist}: {type(exc).__name__}: {exc}")
            installed = None
        ok = installed is not None
        if ok and installed != "unknown" and g["minimum"]:
            ok = sdk_mod.parse_version(installed) >= sdk_mod.parse_version(g["minimum"])
        out.append({"distribution": dist, "minimum": g["minimum"], "installed": installed,
                    "ok": ok, "drivers": sorted(g["drivers"]), "install": g["install"]})
    return out


# -- ROS 2 ------------------------------------------------------------------------------


def ros2(environ: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    """The sourced ROS 2 distro (``ROS_DISTRO``) and whether ``rclpy`` is importable."""
    env = os.environ if environ is None else environ
    try:
        rclpy = importlib.util.find_spec("rclpy") is not None
    except (ImportError, ValueError):
        rclpy = False
    return {"distro": env.get("ROS_DISTRO") or None, "rclpy": rclpy}


# -- cameras ----------------------------------------------------------------------------


def realsense_cameras() -> List[Dict[str, Any]]:
    """RealSense devices with their serials; empty when pyrealsense2 is not installed."""
    if importlib.util.find_spec("pyrealsense2") is None:
        return []
    import pyrealsense2 as rs

    out = []
    for dev in rs.context().query_devices():
        def info(key: Any) -> Optional[str]:
            try:
                return str(dev.get_info(key))
            except Exception:  # noqa: BLE001 - not every device reports every field
                return None
        out.append({"kind": "realsense", "name": info(rs.camera_info.name) or "RealSense",
                    "serial": info(rs.camera_info.serial_number), "device": None})
    return out


def zed_cameras() -> List[Dict[str, Any]]:
    """ZED cameras with their serials; empty when pyzed is not installed."""
    if importlib.util.find_spec("pyzed") is None:
        return []
    import pyzed.sl as sl

    out = []
    for dev in sl.Camera.get_device_list():
        serial = getattr(dev, "serial_number", None)
        model = getattr(dev, "camera_model", None)
        out.append({"kind": "zed", "name": str(model) if model is not None else "ZED",
                    "serial": None if serial is None else str(serial), "device": None})
    return out


def v4l2_cameras(root: str = "/") -> List[Dict[str, Any]]:
    """Video devices from ``<root>/sys/class/video4linux/*/name``."""
    base = Path(root, "sys/class/video4linux")
    out = []
    for path in sorted(base.iterdir()) if base.is_dir() else []:
        out.append({"kind": "v4l2", "name": _read(path / "name") or path.name, "serial": None,
                    "device": f"/dev/{path.name}"})
    return out


# -- audio ------------------------------------------------------------------------------


def sounddevice_devices() -> Optional[List[Dict[str, Any]]]:
    """Audio devices from sounddevice; ``None`` when it is not installed."""
    if importlib.util.find_spec("sounddevice") is None:
        return None
    import sounddevice as sd

    return [{"name": str(d.get("name", "")), "inputs": int(d.get("max_input_channels", 0)),
             "outputs": int(d.get("max_output_channels", 0))} for d in sd.query_devices()]


def alsa_devices(root: str = "/") -> Optional[List[Dict[str, Any]]]:
    """Sound cards from ``<root>/proc/asound``; ``None`` when ALSA is not there.

    ``inputs`` / ``outputs`` count a card's capture / playback PCM devices (from
    ``/proc/asound/pcm``), not channels: ALSA does not list channel counts there.
    """
    cards = _read(Path(root, "proc/asound/cards"))
    if cards is None:
        return None
    pcm = _read(Path(root, "proc/asound/pcm")) or ""
    counts: Dict[int, List[int]] = {}
    for line in pcm.splitlines():
        m = re.match(r"^(\d+)-\d+:", line)
        if m:
            c = counts.setdefault(int(m.group(1)), [0, 0])
            c[0] += "capture" in line
            c[1] += "playback" in line
    out = []
    for line in cards.splitlines():
        m = re.match(r"^\s*(\d+)\s+\[[^\]]*\]:\s*(.+)$", line)
        if m:
            index = int(m.group(1))
            ins, outs = counts.get(index, [0, 0])
            out.append({"name": m.group(2).strip(), "inputs": ins, "outputs": outs})
    return out


def audio(root: str = "/", notes: Optional[List[str]] = None) -> Dict[str, Any]:
    """``{"source", "devices"}``: sounddevice when installed, else ALSA, else nothing."""
    notes = notes if notes is not None else []
    devices = _guard(notes, "audio (sounddevice)", sounddevice_devices, None)
    if devices is not None:
        return {"source": "sounddevice", "devices": devices}
    devices = _guard(notes, "audio (alsa)", lambda: alsa_devices(root), None)
    if devices is not None:
        return {"source": "alsa", "devices": devices}
    return {"source": None, "devices": []}


# -- suggestions ------------------------------------------------------------------------


def suggestions(nics: List[Dict[str, Any]], sdk_list: List[Dict[str, Any]],
                root: str = "/") -> List[str]:
    """Commands and hints for the user to act on; nothing here is run."""
    out = []
    on_subnet = [n for n in nics if n["unitree_subnet"]]
    if not on_subnet:
        # Name a card only when it is wired, up or with carrier, and has no IPv4 yet: a card
        # with an address on another subnet is likely the LAN, and a second address there
        # is not what the user wants.
        candidates = [n for n in nics
                      if not n["ipv4"] and _is_wired(root, n["name"])
                      and (n["up"] or _has_carrier(root, n["name"]))]
        nic = candidates[0]["name"] if candidates else "<nic>"
        out.append(f"sudo ip addr add {SUGGESTED_HOST}/24 dev {nic}  # no NIC on 192.168.123.x; "
                   f"use the card wired to the robot")
    for n in on_subnet:
        if n["speed_mbps"] is not None and n["speed_mbps"] < 1000:
            out.append(f"{n['name']} links at {n['speed_mbps']} Mb/s (expected 1000): "
                       f"check the cable and connector")
    for s in sdk_list:
        if s["distribution"] == UNITREE_SDK and not s["ok"]:
            what = "missing" if s["installed"] is None else (
                f"{s['installed']} is older than {s['minimum']}")
            hint = s.get("install") or f"pip install {UNITREE_SDK}"
            out.append(f"{hint}  # {UNITREE_SDK} {what}")
    return out


# -- all together -----------------------------------------------------------------------


def scan_host(root: str = "/") -> Dict[str, Any]:
    """The ``host`` object of a scan report (see :func:`hirobot.scan.scan`)."""
    notes: List[str] = []
    uname = platform.uname()
    nics = _guard(notes, "interfaces", lambda: interfaces(root, notes=notes), [])
    sdk_list = _guard(notes, "sdks", lambda: sdks(notes=notes), [])
    cameras: List[Dict[str, Any]] = []
    cameras += _guard(notes, "cameras (realsense)", realsense_cameras, [])
    cameras += _guard(notes, "cameras (zed)", zed_cameras, [])
    cameras += _guard(notes, "cameras (v4l2)", lambda: v4l2_cameras(root), [])
    return {
        "hostname": _guard(notes, "hostname", platform.node, None) or None,
        "os": f"{uname.system} {uname.release}".strip() or None,
        "arch": uname.machine or None,
        "python": platform.python_version(),
        "jetson": _guard(notes, "jetson", lambda: jetson_model(root), None),
        "interfaces": nics,
        "sdks": sdk_list,
        "ros2": _guard(notes, "ros2", ros2, {"distro": None, "rclpy": False}),
        "cameras": cameras,
        "audio": _guard(notes, "audio", lambda: audio(root, notes), {"source": None,
                                                                   "devices": []}),
        "suggestions": _guard(notes, "suggestions",
                              lambda: suggestions(nics, sdk_list, root), []),
        "notes": notes,
    }

