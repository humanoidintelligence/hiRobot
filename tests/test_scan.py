"""hirobot.scan: host detection on a fake /sys tree, the report shape, the Unitree hand-off."""

from __future__ import annotations

import json
import subprocess
import sys
import types
from pathlib import Path

import pytest

import hirobot.scan as scan_mod
from hirobot.scan import host as host_mod

HOST_KEYS = {"hostname", "os", "arch", "python", "jetson", "interfaces", "sdks", "ros2",
             "cameras", "audio", "suggestions"}
NIC_KEYS = {"name", "up", "ipv4", "mac", "speed_mbps", "unitree_subnet"}
UNITREE_KEYS = {"interface", "status", "error", "msg_family", "motor_slots", "motors_reporting",
                "reporting_slots", "mode_machine", "lowstate_hz", "motion_service", "hands",
                "candidates", "notes"}


def _nic(root: Path, name: str, state: str = "up", speed: str = "1000",
         mac: str = "00:11:22:33:44:55", wired: bool = True, wireless: bool = False) -> None:
    d = root / "sys/class/net" / name
    d.mkdir(parents=True)
    (d / "operstate").write_text(state + "\n")
    (d / "speed").write_text(speed + "\n")
    (d / "address").write_text(mac + "\n")
    if wired:
        (d / "device").mkdir()
    if wireless:
        (d / "wireless").mkdir()


def _write(root: Path, rel: str, text: str) -> None:
    p = root / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text)


IP_JSON = json.dumps([
    {"ifname": "lo", "addr_info": [{"family": "inet", "local": "127.0.0.1", "prefixlen": 8}]},
    {"ifname": "eth0", "addr_info": [{"family": "inet", "local": "192.168.123.164",
                                      "prefixlen": 24}]},
    {"ifname": "eth1", "addr_info": [{"family": "inet", "local": "10.0.0.5", "prefixlen": 16}]},
])


# -- network ----------------------------------------------------------------------------


def test_parse_ip_json_and_subnet():
    addrs = host_mod.parse_ip_json(IP_JSON)
    assert addrs["eth0"] == ["192.168.123.164/24"]
    assert host_mod.on_unitree_subnet(addrs["eth0"])
    assert not host_mod.on_unitree_subnet(addrs["eth1"])
    assert not host_mod.on_unitree_subnet(["192.168.124.5/24", "garbage"])
    assert host_mod.parse_ip_json("") == {}


def test_interfaces_from_sys(tmp_path):
    _nic(tmp_path, "lo", state="unknown")
    _nic(tmp_path, "eth0", speed="100")
    _nic(tmp_path, "eth1", state="down", speed="-1")
    nics = host_mod.interfaces(str(tmp_path), addresses=host_mod.parse_ip_json(IP_JSON))
    assert [n["name"] for n in nics] == ["eth0", "eth1"]  # lo skipped
    for n in nics:
        assert set(n) == NIC_KEYS
    eth0, eth1 = nics
    assert eth0["up"] and eth0["unitree_subnet"] and eth0["speed_mbps"] == 100
    assert eth0["mac"] == "00:11:22:33:44:55"
    assert not eth1["up"] and not eth1["unitree_subnet"] and eth1["speed_mbps"] is None


def test_missing_ip_command_means_no_addresses(tmp_path, monkeypatch):
    _nic(tmp_path, "eth0")

    def boom(*a, **k):
        raise FileNotFoundError("ip")

    monkeypatch.setattr(subprocess, "run", boom)
    notes = []
    nics = host_mod.interfaces(str(tmp_path), notes=notes)
    assert nics[0]["ipv4"] == [] and not nics[0]["unitree_subnet"]
    assert any("ip -j -4 addr" in n for n in notes)


def test_suggestions_no_subnet_names_an_up_wired_card(tmp_path):
    _nic(tmp_path, "wlan0", wired=True, wireless=True)
    _nic(tmp_path, "eth1")
    _nic(tmp_path, "eth2")
    nics = host_mod.interfaces(str(tmp_path), addresses={"eth1": ["10.0.0.5/16"]})
    out = host_mod.suggestions(nics, [], str(tmp_path))
    assert out == [s for s in out if s.startswith("sudo ip addr add 192.168.123.")]
    assert "dev eth2" in out[0]  # wired, up, no address yet


def test_suggestions_never_name_a_card_with_an_address_elsewhere(tmp_path):
    _nic(tmp_path, "eth0")  # the only wired card, already on the LAN
    _nic(tmp_path, "wlan0", wireless=True)
    nics = host_mod.interfaces(str(tmp_path), addresses={"eth0": ["10.0.0.5/16"]})
    out = host_mod.suggestions(nics, [], str(tmp_path))
    assert len(out) == 1 and "dev <nic>" in out[0]
    assert "the card wired to the robot" in out[0]


def test_suggestions_name_a_down_card_with_carrier(tmp_path):
    _nic(tmp_path, "eth0", state="down")
    _nic(tmp_path, "eth1", state="down")
    (tmp_path / "sys/class/net/eth1/carrier").write_text("1\n")
    nics = host_mod.interfaces(str(tmp_path), addresses={})
    assert "dev eth1" in host_mod.suggestions(nics, [], str(tmp_path))[0]


def test_suggestions_generic_nic_and_slow_link_and_sdk(tmp_path):
    out = host_mod.suggestions([], [], str(tmp_path))
    assert out and "dev <nic>" in out[0]

    _nic(tmp_path, "eth0", speed="10")
    nics = host_mod.interfaces(str(tmp_path), addresses={"eth0": ["192.168.123.222/24"]})
    sdk = [{"distribution": "unitree_sdk2py", "minimum": "1.0.1", "installed": None,
            "ok": False, "drivers": ["unitree_sdk2"], "install": "pip install -e <clone>"}]
    out = host_mod.suggestions(nics, sdk, str(tmp_path))
    assert not any(s.startswith("sudo") for s in out)  # subnet present
    assert any("10 Mb/s" in s and "cable" in s for s in out)
    assert any(s.startswith("pip install -e <clone>") and "missing" in s for s in out)


# -- system, cameras, audio -------------------------------------------------------------


def test_jetson(tmp_path):
    assert host_mod.jetson_model(str(tmp_path)) is None
    _write(tmp_path, "etc/nv_tegra_release", "# R35 (release), REVISION: 4.1\n")
    assert "Jetson" in host_mod.jetson_model(str(tmp_path))
    _write(tmp_path, "proc/device-tree/model", "NVIDIA Jetson Orin NX Developer Kit\x00")
    assert host_mod.jetson_model(str(tmp_path)) == "NVIDIA Jetson Orin NX Developer Kit"


def test_v4l2_cameras(tmp_path):
    _write(tmp_path, "sys/class/video4linux/video0/name", "USB Camera\n")
    cams = host_mod.v4l2_cameras(str(tmp_path))
    assert cams == [{"kind": "v4l2", "name": "USB Camera", "serial": None,
                     "device": "/dev/video0"}]
    assert host_mod.v4l2_cameras(str(tmp_path / "none")) == []


def test_audio_falls_back_to_alsa(tmp_path, monkeypatch):
    monkeypatch.setattr(host_mod, "sounddevice_devices", lambda: None)
    _write(tmp_path, "proc/asound/cards",
           " 0 [PCH            ]: HDA-Intel - HDA Intel PCH\n"
           "                      HDA Intel PCH at 0xf0 irq 1\n")
    _write(tmp_path, "proc/asound/pcm",
           "00-00: ALC : ALC : playback 1 : capture 1\n00-03: HDMI 0 : HDMI 0 : playback 1\n")
    a = host_mod.audio(str(tmp_path))
    assert a == {"source": "alsa",
                 "devices": [{"name": "HDA-Intel - HDA Intel PCH", "inputs": 1, "outputs": 2}]}
    assert host_mod.audio(str(tmp_path / "none")) == {"source": None, "devices": []}


def test_audio_prefers_sounddevice(tmp_path, monkeypatch):
    devs = [{"name": "mic", "inputs": 2, "outputs": 0}]
    monkeypatch.setattr(host_mod, "sounddevice_devices", lambda: devs)
    assert host_mod.audio(str(tmp_path)) == {"source": "sounddevice", "devices": devs}


def test_ros2(monkeypatch):
    r = host_mod.ros2({"ROS_DISTRO": "humble"})
    assert r["distro"] == "humble" and isinstance(r["rclpy"], bool)
    assert host_mod.ros2({})["distro"] is None


# -- SDKs -------------------------------------------------------------------------------


def test_sdks_group_by_distribution():
    sdks = host_mod.sdks()
    by = {s["distribution"]: s for s in sdks}
    unitree = by["unitree_sdk2py"]
    assert unitree["minimum"] == "1.0.1"
    assert {"unitree_sdk2", "unitree_handcmd"} <= set(unitree["drivers"])
    for s in sdks:
        assert {"distribution", "minimum", "installed", "ok", "drivers"} <= set(s)
        assert isinstance(s["ok"], bool) and isinstance(s["drivers"], list)
        if s["installed"] is None:
            assert not s["ok"]


def test_sdk_too_old_is_not_ok(monkeypatch):
    from hirobot.loading import sdk as sdk_mod

    def fake_version(req):
        if req.distribution == "unitree_sdk2py":
            return "1.0.0"
        raise ImportError("missing")

    monkeypatch.setattr(sdk_mod, "installed_version", fake_version)
    by = {s["distribution"]: s for s in host_mod.sdks()}
    assert by["unitree_sdk2py"]["installed"] == "1.0.0" and not by["unitree_sdk2py"]["ok"]
    assert by["mujoco"]["installed"] is None and not by["mujoco"]["ok"]


# -- the whole report -------------------------------------------------------------------


def _fake_host(nics):
    return {"hostname": "h", "os": "Linux 6.1", "arch": "aarch64", "python": "3.10.12",
            "jetson": None, "interfaces": nics, "sdks": [],
            "ros2": {"distro": None, "rclpy": False}, "cameras": [],
            "audio": {"source": None, "devices": []}, "suggestions": [], "notes": []}


ETH0 = {"name": "eth0", "up": True, "ipv4": ["192.168.123.164/24"], "mac": None,
        "speed_mbps": 1000, "unitree_subnet": True}
PROBE_MODULE = "hirobot.drivers.body.unitree.sdk2.probe"


def _install_probe(monkeypatch, probe):
    mod = types.ModuleType(PROBE_MODULE)
    mod.probe = probe
    monkeypatch.setitem(sys.modules, PROBE_MODULE, mod)


@pytest.fixture
def fake_probe(monkeypatch):
    calls = []
    found = {"interface": "eth0", "status": "found", "error": None, "msg_family": "hg",
             "motor_slots": 35, "motors_reporting": 29, "mode_machine": 5,
             "lowstate_hz": 500.0,
             "motion_service": {"checked": True, "mode": "", "owns_lowcmd": False},
             "hands": {"dex3_left": True, "dex3_right": True},
             "candidates": [{"config": "robots/unitree_g1_dex3", "confidence": 0.9,
                             "why": "29 motors + Dex3"}],
             "notes": []}

    def probe(interface, timeout=2.0, domain_id=0):
        calls.append((interface, timeout))
        return dict(found, interface=interface)

    _install_probe(monkeypatch, probe)
    return calls


def test_scan_hands_the_picked_interface_to_the_probe(monkeypatch, fake_probe):
    down = dict(ETH0, name="eth9", up=False)
    monkeypatch.setattr(scan_mod, "scan_host", lambda: _fake_host([down, ETH0]))
    r = scan_mod.scan(timeout=0.5)
    assert fake_probe == [("eth0", 0.5)]
    assert r["version"] == 1 and r["unitree"]["status"] == "found"
    assert r["unitree"]["candidates"][0]["config"] == "robots/unitree_g1_dex3"
    text = scan_mod.summary(r)
    assert "Unitree: found on eth0" in text and "robots/unitree_g1_dex3" in text
    scan_mod.scan(interface="eth7")  # an explicit interface wins
    assert fake_probe[-1][0] == "eth7"


def test_scan_without_interface_skips(monkeypatch, fake_probe):
    monkeypatch.setattr(scan_mod, "scan_host", lambda: _fake_host([]))
    r = scan_mod.scan()
    assert fake_probe == []
    assert set(r["unitree"]) == UNITREE_KEYS
    assert r["unitree"]["status"] == "skipped" and r["unitree"]["interface"] is None
    assert "192.168.123" in r["unitree"]["notes"][0]


def test_scan_unitree_false_skips(monkeypatch, fake_probe):
    monkeypatch.setattr(scan_mod, "scan_host", lambda: _fake_host([ETH0]))
    r = scan_mod.scan(unitree=False)
    assert fake_probe == [] and r["unitree"]["status"] == "skipped"


def test_raising_probe_is_an_error_status(monkeypatch):
    def probe(interface, timeout=2.0, domain_id=0):
        raise RuntimeError("dds down")

    _install_probe(monkeypatch, probe)
    monkeypatch.setattr(scan_mod, "scan_host", lambda: _fake_host([ETH0]))
    r = scan_mod.scan()
    assert r["unitree"]["status"] == "error"
    assert r["unitree"]["error"] == "RuntimeError: dds down"
    assert set(r["unitree"]) == UNITREE_KEYS
    assert "dds down" in scan_mod.summary(r)


def test_missing_probe_module_is_an_error_status(monkeypatch):
    monkeypatch.setitem(sys.modules, PROBE_MODULE, None)  # import of it raises ImportError
    monkeypatch.setattr(scan_mod, "scan_host", lambda: _fake_host([ETH0]))
    r = scan_mod.scan()
    assert r["unitree"]["status"] == "error"
    assert r["unitree"]["error"].startswith(("ImportError", "ModuleNotFoundError"))


def test_scan_host_never_raises(monkeypatch, tmp_path):
    def boom(*a, **k):
        raise RuntimeError("broken")

    for name in ("interfaces", "sdks", "realsense_cameras", "zed_cameras", "v4l2_cameras",
                 "sounddevice_devices", "alsa_devices", "jetson_model", "ros2"):
        monkeypatch.setattr(host_mod, name, boom)
    h = host_mod.scan_host(str(tmp_path))
    assert HOST_KEYS <= set(h)
    assert h["interfaces"] == [] and h["sdks"] == [] and h["cameras"] == []
    assert h["jetson"] is None and h["audio"] == {"source": None, "devices": []}
    assert h["ros2"] == {"distro": None, "rclpy": False}
    assert any("broken" in n for n in h["notes"])
    assert h["suggestions"] and "dev <nic>" in h["suggestions"][0]


def test_real_scan_on_this_host():
    """Reads this machine only; the Unitree probe is off, so nothing joins a network."""
    r = scan_mod.scan(unitree=False)
    assert isinstance(r, dict) and r["version"] == 1
    assert HOST_KEYS <= set(r["host"]) and set(r["unitree"]) == UNITREE_KEYS
    for n in r["host"]["interfaces"]:
        assert set(n) == NIC_KEYS and n["name"] != "lo"
    json.dumps(r)
    assert isinstance(scan_mod.summary(r), str)

