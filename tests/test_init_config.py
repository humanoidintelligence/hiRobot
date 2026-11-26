"""``hirobot init``: the local config it writes and the read-only check after it.

No SDK or robot: the scan report is passed in, and the read-only check runs on
fake real drivers (registered for the test) or on the plain built-in (toy body).
"""

from __future__ import annotations

import json
import time

import numpy as np
import pytest
import yaml

from hirobot import Observation, Robot, load_embodiment, make_robot
from hirobot.cli import main
from hirobot.core.stamps import Stamp
from hirobot.devices import audio, camera, end_effector
from hirobot.loading import registry
from hirobot.loading.tree import load_robot
from hirobot.scan import init_config


def _report(*cands, interface="eth0", status="found"):
    return {"version": 1, "host": {},
            "unitree": {"interface": interface, "status": status, "error": None,
                        "msg_family": "hg" if cands else None, "motor_slots": 35,
                        "motors_reporting": 29 if cands else None, "mode_machine": None,
                        "motion_service": {"checked": False, "mode": None, "owns_lowcmd": None},
                        "hands": {"dex3_left": None, "dex3_right": None},
                        "candidates": [{"config": c, "confidence": conf, "why": "test"}
                                       for c, conf in cands],
                        "notes": []}}


def _no_input(prompt):
    raise AssertionError(f"asked {prompt!r} without a terminal")


def _toy_open(monkeypatch, model):
    """verify_config opens the plain built-in (toy body) instead of the real robot."""
    opened = []

    def fake(path, drop=()):
        opened.append((path, list(drop)))
        return make_robot(model)

    monkeypatch.setattr(init_config, "open_read_only", fake)
    return opened


# ------------------------------------------------------------------ render


def test_g1_dex3_sets_the_nic_on_body_hands_and_devices(tmp_path):
    text = init_config.render_config("robots/unitree_g1_dex3", "eth0", _report())
    assert text.startswith("# unitree_g1_dex3 on eth0, written by `hirobot init`")
    assert "docs/setup/unitree.md" in text and "built-in defaults" in text
    data = yaml.safe_load(text)
    assert data["defaults"] == ["robots/unitree_g1_dex3"]
    parts = data["assembly"]["parts"]
    assert parts["body"]["overrides"]["driver"] == {"type": "unitree_sdk2",
                                                    "network_interface": "eth0"}
    for hand in ("left_hand", "right_hand", "head", "head_mic", "head_speaker"):
        assert parts[hand]["overrides"]["driver"] == {"network_interface": "eth0"}
    assert "kp" not in text and "gains" not in yaml.safe_dump(data)  # no gains copied

    path = tmp_path / "g1.yaml"
    path.write_text(text)
    assert load_embodiment(str(path)).num_dofs == 43
    drivers = {p: d.driver for p, d in load_robot(str(path)).root.walk()}
    assert drivers["left_hand"] == {"type": "unitree_handcmd", "side": "left",
                                    "network_interface": "eth0"}
    assert drivers["head_speaker"]["volume"] == 60  # the built-in's options are kept
    make_robot(str(path))  # builds without connecting


def test_h1_keeps_the_go_message_family_and_skips_the_zmq_camera(tmp_path):
    data = yaml.safe_load(init_config.render_config("unitree_h1", "eth1", _report()))
    parts = data["assembly"]["parts"]
    assert parts["body"]["overrides"]["driver"] == {
        "type": "unitree_sdk2", "msg_family": "go", "network_interface": "eth1"}
    assert "head" not in parts  # image_server ignores the NIC
    path = tmp_path / "h1.yaml"
    path.write_text(yaml.safe_dump(data))
    assert load_embodiment(str(path)).num_dofs == 19


def test_header_says_what_was_seen_and_not_seen():
    rep = _report(("robots/unitree_g1", 0.6))
    rep["unitree"].update(motors_reporting=27, hands={"dex3_left": True, "dex3_right": None},
                          notes=["only 27 of 29 G1 motors report"])
    text = init_config.render_config("unitree_g1", "eth0", rep)
    head = [line for line in text.splitlines() if line.startswith("#")]
    assert any("27 of 35 motor slots" in line for line in head)
    assert any("dex3_right not seen, please confirm" in line for line in head)
    assert any("only 27 of 29" in line for line in head)


def test_only_unitree_hardware_models_are_accepted():
    assert init_config.check_model("unitree_g1") == "robots/unitree_g1"
    models = init_config.unitree_models()
    assert {"robots/unitree_g1", "robots/unitree_g1_dex3", "robots/unitree_h1",
            "robots/unitree_h1_2", "robots/unitree_h2"} <= set(models)
    assert not any("mujoco" in m for m in models)
    for bad in ("ur_ur5e", "unitree_g1_mujoco", "nope"):
        with pytest.raises(ValueError):
            init_config.check_model(bad)


# ------------------------------------------------------------------ verify on fake real drivers


class FakeBody(Robot):
    """A real body driver with read_only; fails the test on any command."""

    simulated = False
    instances: list = []

    def __init__(self, embodiment, read_only=False, network_interface=None, nan=()):
        super().__init__()
        self._emb = embodiment
        self.read_only = read_only
        self.nan = list(nan)
        self.connected = False
        FakeBody.instances.append(self)

    @property
    def embodiment(self):
        return self._emb

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def reset(self, seed=None):
        raise AssertionError("verify must not reset the robot")

    def hold(self):
        raise AssertionError("verify must not hold the robot")

    def get_observation(self):
        q = np.zeros(self._emb.num_dofs)
        q[self.nan] = np.nan
        return Observation(joint_pos=q, base_orientation=np.array([0.0, 0, 0, 1.0]))

    def _send_action(self, action):
        raise AssertionError("verify must not send an action")


class FakeHand(end_effector.EndEffectorDriver):
    simulated = False
    instances: list = []

    def __init__(self, name, joints, read_only=False, network_interface=None, silent=False):
        super().__init__(name, joints, read_only=read_only)
        self.silent = silent
        FakeHand.instances.append(self)

    def connect(self):
        pass

    def get_state(self):
        n = self.num_joints
        q = np.full(n, np.nan) if self.silent else np.zeros(n)
        return end_effector.JointsState(q=q, dq=np.zeros(n), tau=np.zeros(n),
                                        stamp=Stamp(time.monotonic(), 1))

    def command(self, q, kp=None, kd=None):
        raise AssertionError("verify must not command a hand")

    def damp(self):
        pass


class FakeCam(camera.CameraDriver):
    simulated = False

    def __init__(self, name, network_interface=None, frames=True):
        super().__init__(name)
        self.frames = frames

    def start(self):
        pass

    def latest(self):
        if not self.frames:
            return None
        return np.zeros((4, 6, 3), dtype=np.uint8), Stamp(time.monotonic(), 1)

    def stop(self):
        pass


class FakeMic(audio.QueuedMicrophone):
    simulated = False
    sample_rate = 16000

    def __init__(self, name, network_interface=None):
        super().__init__(name)

    def _open(self):
        pass

    def _receive(self):
        time.sleep(0.01)
        return b"\0\0" * 160


class FakeSpeaker(audio.SpeakerDriver):
    """Sets something on open, like the Unitree speaker's volume: no read_only mode."""

    simulated = False
    opened: list = []

    def __init__(self, name, network_interface=None):
        super().__init__(name)

    def open(self):
        FakeSpeaker.opened.append(self.name)

    def close(self):
        pass

    def play(self, pcm, sample_rate=None):
        return 0.0

    def stop(self):
        pass


@pytest.fixture
def fakes(monkeypatch):
    FakeBody.instances, FakeHand.instances, FakeSpeaker.opened = [], [], []
    monkeypatch.setitem(registry.BODIES._entries, "fake_body", FakeBody)
    monkeypatch.setitem(end_effector.END_EFFECTORS._entries, "fake_hand", FakeHand)
    monkeypatch.setitem(camera.CAMERAS._entries, "fake_cam", FakeCam)
    monkeypatch.setitem(audio.AUDIO._entries, "fake_mic", FakeMic)
    monkeypatch.setitem(audio.AUDIO._entries, "fake_speaker", FakeSpeaker)


def _fake_robot(tmp_path, body=None, hand=None, cam=None):
    cfg = {"assembly": {"name": "fake", "parts": {
        "body": {"device": "body", "control_hz": 50,
                 "joints": [{"name": f"j{i}"} for i in range(3)],
                 "driver": dict({"type": "fake_body"}, **(body or {}))},
        "hand": {"device": "end_effector", "interface": "joints",
                 "joints": [{"name": "f0"}, {"name": "f1"}],
                 "driver": dict({"type": "fake_hand"}, **(hand or {}))},
        "head": {"device": "camera", "width": 6, "height": 4,
                 "driver": dict({"type": "fake_cam"}, **(cam or {}))},
        "mic": {"device": "microphone", "driver": {"type": "fake_mic"}},
        "spk": {"device": "speaker", "driver": {"type": "fake_speaker"}}}}}
    path = tmp_path / "fake.yaml"
    path.write_text(yaml.safe_dump(cfg))
    return path


def _status(result):
    return {p["path"]: p["status"] for p in result["parts"]}


def test_verify_opens_read_only_and_reports_each_part(fakes, tmp_path):
    result = init_config.verify_config(_fake_robot(tmp_path), timeout=0.5)
    assert result["ok"], result
    assert result["num_dofs"] == result["joint_pos_len"] == 5
    assert _status(result) == {"body": "seen", "hand": "seen", "head": "seen", "mic": "seen",
                               "spk": "skipped"}
    body = FakeBody.instances[-1]
    assert body.read_only is True and body.connected is False  # disconnected
    assert FakeHand.instances[-1].read_only is True
    assert FakeSpeaker.opened == []  # no read_only mode: never opened
    assert "no read_only mode" in [p for p in result["parts"] if p["path"] == "spk"][0]["note"]


def test_verify_reports_a_silent_hand_and_camera_as_not_seen(fakes, tmp_path):
    path = _fake_robot(tmp_path, hand={"silent": True}, cam={"frames": False})
    result = init_config.verify_config(path, timeout=0.2)
    assert result["ok"]  # the body reports: hands and cameras only warn
    st = _status(result)
    assert st["hand"] == "not_seen" and st["head"] == "not_seen"
    note = [p for p in result["parts"] if p["path"] == "hand"][0]["note"]
    assert "please confirm" in note


def test_verify_fails_when_body_joints_do_not_report(fakes, tmp_path):
    result = init_config.verify_config(_fake_robot(tmp_path, body={"nan": [1]}), timeout=0.2)
    assert not result["ok"] and _status(result)["body"] == "partial"
    assert FakeBody.instances[-1].connected is False


def test_verify_refuses_a_body_without_read_only(tmp_path):
    path = tmp_path / "toy.yaml"
    path.write_text("defaults: [robots/unitree_g1]\n")  # toy body: simulated
    result = init_config.verify_config(path, timeout=0.2)
    assert not result["ok"] and "could not open read-only" in result["error"]


# ------------------------------------------------------------------ run_init


def _run(out, lines=None, **kw):
    printed = []
    replies = iter(lines or [])
    kw.setdefault("input_fn", _no_input if lines is None else lambda prompt: next(replies))
    code = init_config.run_init(out, print_fn=lambda *a: printed.append(" ".join(map(str, a))),
                                **kw)
    return code, "\n".join(printed)


def test_no_candidate_writes_nothing(tmp_path):
    out = tmp_path / "robot.yaml"
    code, text = _run(out, report=_report(status="not_seen"), yes=True)
    assert code == 1 and not out.exists()
    assert "no Unitree robot seen on eth0" in text and "--model robots/unitree_g1" in text


def test_model_given_without_a_robot_writes_a_loadable_config(tmp_path):
    out = tmp_path / "robot.yaml"
    code, text = _run(out, report=_report(status="not_seen"), model="robots/unitree_g1",
                      verify=False)
    assert code == 0 and "Wrote" in text
    assert load_embodiment(str(out)).num_dofs == 29
    make_robot(str(out))


def test_no_interface_writes_nothing(tmp_path):
    out = tmp_path / "robot.yaml"
    code, text = _run(out, report=_report(interface=None, status="skipped"),
                      model="unitree_g1", verify=False)
    assert code == 1 and not out.exists() and "--interface" in text


def test_existing_file_is_kept_with_yes_and_replaced_when_answered(tmp_path):
    out = tmp_path / "robot.yaml"
    out.write_text("# mine\n")
    rep = _report(("robots/unitree_g1", 0.95))
    code, text = _run(out, report=rep, yes=True, verify=False)
    assert code == 1 and out.read_text() == "# mine\n" and "was kept" in text
    code, _ = _run(out, report=rep, yes=True, answers={"overwrite": False}, verify=False)
    assert code == 1 and out.read_text() == "# mine\n"
    code, _ = _run(out, report=rep, yes=True, answers={"overwrite": True}, verify=False)
    assert code == 0 and "defaults: [robots/unitree_g1]" in out.read_text()


def test_existing_file_prompt_defaults_to_no(tmp_path):
    out = tmp_path / "robot.yaml"
    out.write_text("# mine\n")
    rep = _report(("robots/unitree_g1", 0.95))
    assert _run(out, [""], report=rep, model="unitree_g1", verify=False)[0] == 1
    assert out.read_text() == "# mine\n"
    assert _run(out, ["y"], report=rep, model="unitree_g1", verify=False)[0] == 0
    assert out.read_text() != "# mine\n"
    out.write_text("# mine\n")
    assert _run(out, report=rep, model="unitree_g1", force=True, verify=False)[0] == 0


def test_interactive_pick_of_the_second_candidate(tmp_path):
    out = tmp_path / "robot.yaml"
    rep = _report(("robots/unitree_h1_2", 0.6), ("robots/unitree_g1", 0.4))
    code, text = _run(out, ["2"], report=rep, verify=False)
    assert code == 0 and "1. robots/unitree_h1_2" in text and "2. robots/unitree_g1" in text
    assert yaml.safe_load(out.read_text())["defaults"] == ["robots/unitree_g1"]


def test_without_a_terminal_an_uncertain_model_is_not_guessed(tmp_path, monkeypatch):
    monkeypatch.setattr("sys.stdin", None)
    out = tmp_path / "robot.yaml"
    rep = _report(("robots/unitree_h1_2", 0.6), ("robots/unitree_g1", 0.4))
    printed = []
    code = init_config.run_init(out, report=rep, verify=False, input_fn=input,
                                print_fn=printed.append)
    assert code == 1 and not out.exists() and "not certain" in printed[-1]
    rep = _report(("robots/unitree_g1_dex3", 0.95))
    assert init_config.run_init(out, report=rep, verify=False, input_fn=input,
                                print_fn=printed.append) == 0


def test_answers_file_sets_model_interface_and_verify(tmp_path):
    answers = tmp_path / "answers.yaml"
    answers.write_text("model: unitree_h1\ninterface: eth3\nverify: false\n")
    out = tmp_path / "robot.yaml"
    code, _ = _run(out, report=_report(interface=None, status="skipped"), answers=str(answers))
    assert code == 0
    body = yaml.safe_load(out.read_text())["assembly"]["parts"]["body"]["overrides"]["driver"]
    assert body["network_interface"] == "eth3" and body["msg_family"] == "go"
    answers.write_text("modle: unitree_h1\n")
    code, text = _run(out, report=_report(), answers=str(answers))
    assert code == 1 and "unknown keys" in text


def test_init_verifies_and_reports_failure_with_exit_2(tmp_path, monkeypatch):
    opened = _toy_open(monkeypatch, "robots/unitree_g1")
    out = tmp_path / "robot.yaml"
    code, text = _run(out, report=_report(("robots/unitree_g1", 0.95)), yes=True)
    assert code == 0 and "Read-only check passed" in text
    assert opened == [(out, ["head_speaker"])]  # the speaker has no read_only mode
    assert "body" in text and "data seen" in text

    def broken(path, drop=()):
        raise TimeoutError("No LowState received")

    monkeypatch.setattr(init_config, "open_read_only", broken)
    code, text = _run(out, report=_report(("robots/unitree_g1", 0.95)), yes=True, force=True)
    assert code == 2 and out.exists() and "is kept" in text and "No LowState" in text


def test_init_scans_when_no_report_is_given(tmp_path, monkeypatch):
    import hirobot.scan as scan_mod

    calls = []
    monkeypatch.setattr(scan_mod, "scan",
                        lambda i=None, t=2.0, unitree=True: calls.append((i, t)) or _report(
                            ("robots/unitree_g1", 0.95), interface=i))
    out = tmp_path / "robot.yaml"
    code, _ = _run(out, interface="eth7", yes=True, verify=False, timeout=0.5)
    assert code == 0 and calls == [("eth7", 0.5)]


# ------------------------------------------------------------------ CLI


# ------------------------------------------------------------------ connect failures


class OffBusHand(FakeHand):
    """Like a Dex3 hand off the bus: connect raises, as UnitreeDex3Hand does after 2 s."""

    def connect(self):
        raise TimeoutError(f"{self.name}: no JointsState on rt/dex3/x/state within 2 s")


class BrokenCam(FakeCam):
    def start(self):
        raise RuntimeError("camera not found")


def test_a_hand_off_the_bus_is_not_seen_and_the_rest_verified(fakes, tmp_path, monkeypatch):
    monkeypatch.setitem(end_effector.END_EFFECTORS._entries, "off_bus_hand", OffBusHand)
    monkeypatch.setitem(camera.CAMERAS._entries, "broken_cam", BrokenCam)
    path = _fake_robot(tmp_path, hand={"type": "off_bus_hand"}, cam={"type": "broken_cam"})
    result = init_config.verify_config(path, timeout=0.2)
    assert result["ok"], result
    assert result["joint_pos_len"] == 5  # a part without a driver keeps its joints
    st = _status(result)
    assert st["body"] == "seen" and st["mic"] == "seen"
    assert st["hand"] == "not_seen" and st["head"] == "not_seen"
    notes = {p["path"]: p["note"] for p in result["parts"]}
    assert "TimeoutError" in notes["hand"] and "no JointsState" in notes["hand"]
    assert "camera not found" in notes["head"]
    assert len(FakeBody.instances) == 3  # opened once, then once per failing part
    assert all(not b.connected for b in FakeBody.instances)

    out = tmp_path / "copy.yaml"
    out.write_text(path.read_text())
    monkeypatch.setattr(init_config, "render_config", lambda *a: path.read_text())
    monkeypatch.setattr(init_config, "check_model", lambda m: m)
    code, text = _run(out, report=_report(), model="fake", force=True)
    assert code == 0 and "not seen" in text


def test_a_body_that_fails_to_connect_is_a_failure(fakes, tmp_path, monkeypatch):
    class DeadBody(FakeBody):
        def connect(self):
            raise TimeoutError("No LowState received")

    monkeypatch.setitem(registry.BODIES._entries, "dead_body", DeadBody)
    result = init_config.verify_config(_fake_robot(tmp_path, body={"type": "dead_body"}),
                                       timeout=0.2)
    assert not result["ok"] and "No LowState" in result["error"]
    assert len(FakeBody.instances) == 1  # not retried


# ------------------------------------------------------------------ real drivers, fake SDK


def _fake_unitree_g1_dex3(monkeypatch):
    """The body fake SDK, plus what the Dex3 hands and the videohub camera import."""
    import sys
    import types
    from types import SimpleNamespace

    from tests.drivers.body.unitree.sdk2 import fake_sdk

    def hand_state():
        return SimpleNamespace(motor_state=[SimpleNamespace(q=0.1, dq=0.0, tau_est=0.0)
                                            for _ in range(7)], press_sensor_state=[])

    bus = fake_sdk.install(monkeypatch, fake_sdk.low_state([0.1] * 29, reporting=range(29)),
                           hands={"rt/dex3/left/state": hand_state(),
                                  "rt/dex3/right/state": hand_state()}, mode="ai")
    monkeypatch.setattr(sys.modules["unitree_sdk2py.idl.default"],
                        "unitree_hg_msg_dds__HandCmd_", lambda: SimpleNamespace(motor_cmd=[]),
                        raising=False)
    monkeypatch.setattr(sys.modules["unitree_sdk2py.idl.unitree_hg.msg.dds_"], "HandCmd_",
                        object, raising=False)
    cv2 = pytest.importorskip("cv2")
    jpeg = cv2.imencode(".jpg", np.zeros((48, 64, 3), dtype=np.uint8))[1].tobytes()
    bus.video_calls = []

    class VideoClient:
        def SetTimeout(self, t):
            pass

        def Init(self):
            pass

        def GetImageSample(self):
            bus.video_calls.append("GetImageSample")
            return 0, list(jpeg)

    for name in ("unitree_sdk2py.go2", "unitree_sdk2py.go2.video"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    video = types.ModuleType("unitree_sdk2py.go2.video.video_client")
    video.VideoClient = VideoClient
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.go2.video.video_client", video)
    return bus


def test_verify_g1_dex3_on_the_fake_sdk_writes_nothing(tmp_path, monkeypatch):
    from hirobot.drivers.audio.unitree.sdk2_voice import driver as voice

    bus = _fake_unitree_g1_dex3(monkeypatch)

    def no_multicast(self):  # never join a group on a NIC of the test host
        raise OSError("network interface 'eth0' has no IPv4 address")

    monkeypatch.setattr(voice.UnitreeG1Microphone, "_open", no_multicast)
    path = tmp_path / "g1.yaml"
    path.write_text(init_config.render_config("robots/unitree_g1_dex3", "eth0", _report()))
    result = init_config.verify_config(path, timeout=1.0)
    assert result["ok"], result
    st = _status(result)
    assert st["body"] == "seen" and st["left_hand"] == "seen" and st["right_hand"] == "seen"
    assert st["head"] == "seen" and st["head_speaker"] == "skipped"
    assert st["head_mic"] == "not_seen"  # its open raised: left out, not a failure
    assert result["joint_pos_len"] == 43
    assert bus.publishers == []
    assert bus.sent == []  # nothing on rt/lowcmd or rt/dex3/*/cmd
    assert "SelectMode" not in bus.switcher_calls and "ReleaseMode" not in bus.switcher_calls
    assert bus.video_calls  # the camera's read-only query ran
    assert ("rt/dex3/left/state" in [t for t, _ in bus.subscribed])


def test_cli_scan_json_and_summary(monkeypatch, capsys):
    import hirobot.scan as scan_mod

    seen = []
    monkeypatch.setattr(scan_mod, "scan", lambda i=None, t=2.0, unitree=True:
                        seen.append((i, t, unitree)) or _report())
    monkeypatch.setattr(scan_mod, "summary", lambda r: "SUMMARY " + r["unitree"]["status"])
    assert main(["scan", "--json", "--interface", "eth0", "--timeout", "0.5"]) == 0
    assert json.loads(capsys.readouterr().out)["unitree"]["status"] == "found"
    assert main(["scan", "--no-unitree"]) == 0
    assert capsys.readouterr().out.strip() == "SUMMARY found"
    assert seen == [("eth0", 0.5, True), (None, 2.0, False)]
    assert main(["scan", "--timeout", "0"]) == 2


def test_cli_init_happy_path(tmp_path, monkeypatch, capsys):
    import hirobot.scan as scan_mod

    monkeypatch.setattr(scan_mod, "scan", lambda i=None, t=2.0, unitree=True:
                        _report(("robots/unitree_g1_dex3", 0.9)))
    _toy_open(monkeypatch, "robots/unitree_g1_dex3")
    out = tmp_path / "g1.yaml"
    assert main(["init", "-o", str(out), "--yes"]) == 0
    assert "Read-only check passed" in capsys.readouterr().out
    assert load_embodiment(str(out)).num_dofs == 43
    assert main(["init", "-o", str(out), "--yes"]) == 1  # kept
    assert main(["init", "-o", str(out), "--yes", "--force", "--no-verify"]) == 0

