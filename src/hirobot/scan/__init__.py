"""``hirobot scan``: read-only detection of the host and of a Unitree robot on its network.

:func:`scan` returns a JSON-ready dict (``version`` 1) with a ``host`` part
(:func:`hirobot.scan.host.scan_host`) and a ``unitree`` part from the Unitree probe, which
only subscribes to DDS topics and calls read-only RPCs. Nothing here sends a command or
changes a setting; what needs root comes back as ``host["suggestions"]``. Something the
scan did not see is reported as not seen (``not_seen`` / ``None``), never as absent.
:func:`summary` turns a report into a short text for people.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from hirobot.scan.host import scan_host

__all__ = ["scan", "summary", "scan_host", "REPORT_VERSION"]

REPORT_VERSION = 1

NO_INTERFACE = ("no network card on 192.168.123.x; connect the robot's Ethernet cable, give "
                "the card an address on that subnet (see host suggestions), or pass an interface")


def _unitree_stub(interface: Optional[str], status: str, error: Optional[str] = None,
                  notes: Optional[List[str]] = None) -> Dict[str, Any]:
    """A ``unitree`` object with every key and nothing seen."""
    return {
        "interface": interface,
        "status": status,
        "error": error,
        "msg_family": None,
        "motor_slots": None,
        "motors_reporting": None,
        "reporting_slots": None,
        "mode_machine": None,
        "lowstate_hz": None,
        "motion_service": {"checked": False, "mode": None, "owns_lowcmd": None},
        "hands": {"dex3_left": None, "dex3_right": None},
        "candidates": [],
        "notes": list(notes or []),
    }


def pick_interface(host: Dict[str, Any]) -> Optional[str]:
    """The first up network card with an IPv4 address on 192.168.123.0/24, or ``None``."""
    for nic in host.get("interfaces") or []:
        if nic.get("up") and nic.get("unitree_subnet"):
            return nic["name"]
    return None


def scan(interface: Optional[str] = None, timeout: float = 2.0, unitree: bool = True) -> dict:
    """Scan the host and, when a network card reaches one, a Unitree robot. Never raises.

    Args:
        interface: Network card the robot is on. ``None`` picks the first up card with an
            address on 192.168.123.x.
        timeout: Seconds the Unitree probe listens for each topic.
        unitree: False skips the Unitree probe (``unitree["status"] == "skipped"``).
    """
    try:
        host = scan_host()
    except Exception as exc:  # noqa: BLE001 - scan_host guards itself; this is a backstop
        host = {"hostname": None, "os": None, "arch": None, "python": None, "jetson": None,
                "interfaces": [], "sdks": [], "ros2": {"distro": None, "rclpy": False},
                "cameras": [], "audio": {"source": None, "devices": []}, "suggestions": [],
                "notes": [f"host: {type(exc).__name__}: {exc}"]}
    nic = interface or pick_interface(host)
    if not unitree:
        report_unitree = _unitree_stub(nic, "skipped", notes=["Unitree probe not requested"])
    elif nic is None:
        report_unitree = _unitree_stub(None, "skipped", notes=[NO_INTERFACE])
    else:
        try:
            from hirobot.drivers.body.unitree.sdk2.probe import probe

            report_unitree = probe(nic, timeout)
        except Exception as exc:  # noqa: BLE001 - a failed probe is reported, not raised
            report_unitree = _unitree_stub(nic, "error", error=f"{type(exc).__name__}: {exc}")
    return {"version": REPORT_VERSION, "host": host, "unitree": report_unitree}


def _yes(value: Any) -> str:
    return "not seen" if value is None else ("yes" if value else "no")


def summary(report: dict) -> str:
    """A short, readable multi-line summary of a :func:`scan` report."""
    host = report.get("host") or {}
    lines = []
    head = f"Host: {host.get('hostname') or '?'} ({host.get('os') or '?'}, " \
           f"{host.get('arch') or '?'}, Python {host.get('python') or '?'})"
    if host.get("jetson"):
        head += f", {host['jetson']}"
    lines.append(head)

    lines.append("Network cards:")
    for n in host.get("interfaces") or []:
        speed = f"{n['speed_mbps']} Mb/s" if n.get("speed_mbps") else "speed ?"
        addrs = ", ".join(n.get("ipv4") or []) or "no IPv4"
        mark = "  <- 192.168.123.x" if n.get("unitree_subnet") else ""
        lines.append(f"  {n.get('name')}: {'up' if n.get('up') else 'down'}, {speed}, "
                     f"{addrs}{mark}")
    if not host.get("interfaces"):
        lines.append("  none found")

    sdk_list = host.get("sdks") or []
    ok = [f"{s['distribution']} {s['installed']}" for s in sdk_list if s.get("ok")]
    bad = []
    for s in sdk_list:
        if s.get("ok"):
            continue
        if s.get("installed") is None:
            bad.append(s["distribution"])
        else:
            bad.append(f"{s['distribution']} {s['installed']} < {s['minimum']}")
    lines.append(f"SDKs ok: {', '.join(ok) or 'none'}")
    lines.append(f"SDKs missing or too old: {', '.join(bad) or 'none'}")

    ros = host.get("ros2") or {}
    lines.append(f"ROS 2: distro {ros.get('distro') or 'not sourced'}, "
                 f"rclpy {'importable' if ros.get('rclpy') else 'not importable'}")

    cams = host.get("cameras") or []
    lines.append("Cameras: " + (", ".join(
        f"{c['kind']} {c.get('name')}"
        + (f" (serial {c['serial']})" if c.get("serial") else "")
        + (f" {c['device']}" if c.get("device") else "") for c in cams) or "none seen"))

    aud = host.get("audio") or {}
    devs = aud.get("devices") or []
    lines.append(f"Audio ({aud.get('source') or 'no source'}): " + (", ".join(
        f"{d['name']} (in {d['inputs']}, out {d['outputs']})" for d in devs) or "none seen"))

    u = report.get("unitree") or {}
    status = u.get("status") or "?"
    line = f"Unitree: {status}"
    if u.get("interface"):
        line += f" on {u['interface']}"
    if u.get("error"):
        line += f" ({u['error']})"
    lines.append(line)
    if status == "found":
        lines.append(f"  lowstate {u.get('msg_family')}, {u.get('motors_reporting')} of "
                     f"{u.get('motor_slots')} motor slots report, mode_machine "
                     f"{u.get('mode_machine')}, {u.get('lowstate_hz')} Hz")
        ms = u.get("motion_service") or {}
        if ms.get("checked"):
            lines.append(f"  motion service mode {ms.get('mode')!r}, "
                         f"owns rt/lowcmd: {_yes(ms.get('owns_lowcmd'))}")
        hands = u.get("hands") or {}
        if hands:
            lines.append("  hands: " + ", ".join(f"{k} {_yes(v)}" for k, v in hands.items()))
    for c in u.get("candidates") or []:
        lines.append(f"  candidate {c.get('config')} ({c.get('confidence')}): {c.get('why')}")
    for note in u.get("notes") or []:
        lines.append(f"  note: {note}")

    if host.get("suggestions"):
        lines.append("Suggestions (not run):")
        lines += [f"  {s}" for s in host["suggestions"]]
    for note in host.get("notes") or []:
        lines.append(f"Note: {note}")
    return "\n".join(lines)
