"""Device files and ``assembly:`` robot configs: loading, overrides, checks (hirobot.loading.tree)."""

from __future__ import annotations

import textwrap
from pathlib import Path

import numpy as np
import pytest

from hirobot.loading.tree import DeviceSpec, NodeSpec, joint_safety, load_robot


def _write(root: Path, rel: str, text: str) -> Path:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text))
    return path


@pytest.fixture
def lab(tmp_path):
    """A small tree of files: an arm, a camera, a hand, a single arm, a dual arm."""
    _write(tmp_path, "body/arm.yaml", """\
        device: body
        control_hz: 125
        floating_base: false
        joints:
          - {name: j1, position_limit: [-3, 3], torque_limit: 150}
          - {name: j2}
        frames: [{name: tcp, link: link2}]
        mounts: {flange: {link: link2}}
        model: {mjcf: ../models/arm.xml, kp: [100, 100], home: [0, 0]}
        driver: {type: toy, rate: 5}
        """)
    _write(tmp_path, "camera/cam.yaml", """\
        device: camera
        width: 640
        height: 480
        fps: 30
        fovy: 42
        driver: {type: array}
        """)
    _write(tmp_path, "end_effector/hand.yaml", """\
        device: end_effector
        interface: joints
        joints: [{name: f1}, {name: f2}]
        tcp: [0, 0, 0.1, 0, 0, 0, 1]
        mounts: {palm_cam: {link: palm, pose: [0, 0.05, 0, 0, 0, 0, 1]}}
        driver: {type: fake_hand}
        """)
    _write(tmp_path, "robots/single.yaml", """\
        assembly:
          name: single
          parts:
            body:  {from: ../body/arm.yaml}
            wrist: {from: ../camera/cam.yaml}
          placement:
            wrist: {on: body.flange, pose: [0, -0.07, 0.03, 0, 0, 0, 1]}
        safety: {joint_limit: 3.0, max_joint_step: 0.2}
        """)
    _write(tmp_path, "robots/dual.yaml", """\
        assembly:
          name: dual
          control_hz: 30
          parts:
            left:  {from: single.yaml, overrides: {body: {driver: {ip: 10.0.0.1}}}}
            right: {from: single.yaml, safety: {joint_limit: 2.5}}
            scene: {from: ../camera/cam.yaml}
          placement:
            left:  {pose: [0, 0.4, 0, 0, 0, 0, 1]}
            right: {pose: [0, -0.4, 0, 0, 0, 0, 1]}
        safety: {max_joint_step: 0.3}
        """)
    return tmp_path


def test_every_kind_of_device_loads(lab):
    _write(lab, "microphone/mic.yaml", "device: microphone\ndriver: {type: wav_mic, path: a.wav}\n")
    _write(lab, "speaker/spk.yaml", "device: speaker\ndriver: null_speaker\n")
    _write(lab, "robots/all.yaml", """\
        assembly:
          parts:
            body: {from: ../body/arm.yaml}
            hand: {from: ../end_effector/hand.yaml}
            cam:  {from: ../camera/cam.yaml}
            mic:  {from: ../microphone/mic.yaml}
            spk:  {from: ../speaker/spk.yaml}
          placement:
            hand: {on: body.flange}
            cam:  {on: hand.palm_cam}
        """)
    spec = load_robot(lab / "robots/all.yaml")
    assert spec.root.name == "all"  # from the file name
    kinds = {p: s.kind for p, s in spec.root.walk()}
    assert kinds == {"body": "body", "hand": "end_effector", "cam": "camera", "mic": "microphone",
                     "spk": "speaker"}
    body = spec.root.find("body")
    assert body.fields["control_hz"] == 125 and body.fields["floating_base"] is False
    assert [j["name"] for j in body.fields["joints"]] == ["j1", "j2"]
    assert body.fields["mounts"]["flange"] == {"link": "link2", "pose": None}
    assert body.fields["model"]["mjcf"] == str(lab.resolve() / "body" / "../models/arm.xml")
    hand = spec.root.find("hand")
    assert hand.fields["interface"] == "joints" and np.allclose(hand.fields["tcp"][2], 0.1)
    assert spec.root.find("cam").fields["fovy"] == 42.0
    assert spec.root.find("spk").driver == {"type": "null_speaker"}
    assert spec.root.placement["cam"].on == ("hand", "palm_cam")


def test_a_device_file_alone_is_a_robot(lab):
    spec = load_robot(lab / "body/arm.yaml")
    assert spec.root.name == "arm" and list(spec.root.parts) == ["body"]
    assert isinstance(spec.root.parts["body"], DeviceSpec)


def test_nodes_nest_and_overrides_go_down_by_part_name(lab):
    spec = load_robot(lab / "robots/dual.yaml")
    assert [p for p, _ in spec.root.walk()] == [
        "left", "left/body", "left/wrist", "right", "right/body", "right/wrist", "scene"]
    assert isinstance(spec.root.find("left"), NodeSpec)
    assert spec.root.find("left/body").driver == {"type": "toy", "rate": 5, "ip": "10.0.0.1"}
    assert spec.root.find("right/body").driver == {"type": "toy", "rate": 5}
    assert spec.root.control_hz == 30
    assert np.allclose(spec.root.placement["left"].pose, [0, 0.4, 0, 0, 0, 0, 1])
    wrist = spec.root.find("left").placement["wrist"]
    assert wrist.on == ("body", "flange") and np.allclose(wrist.pose[:3], [0, -0.07, 0.03])


def test_overrides_from_further_up_win_and_a_new_driver_type_starts_fresh(lab):
    _write(lab, "robots/lab.yaml", """\
        assembly:
          parts:
            cell: {from: dual.yaml, overrides: {left: {body: {driver: {ip: 10.0.0.9}}}}}
        """)
    spec = load_robot(lab / "robots/lab.yaml",
                      overrides={"cell/right/body": {"driver": {"type": "ur_rtde", "ip": "1.2.3.4"}}})
    assert spec.root.find("cell/left/body").driver == {"type": "toy", "rate": 5, "ip": "10.0.0.9"}
    assert spec.root.find("cell/right/body").driver == {"type": "ur_rtde", "ip": "1.2.3.4"}


def test_overrides_back_to_the_files_driver_type_keep_its_options(lab):
    spec = load_robot(lab / "robots/single.yaml",
                      overrides={"body": {"driver": {"type": "toy", "ip": "1.1.1.1"}}})
    assert spec.root.find("body").driver == {"type": "toy", "rate": 5, "ip": "1.1.1.1"}


def test_inline_parts_name_things_like_references(lab):
    _write(lab, "robots/inline.yaml", """\
        assembly:
          parts:
            left: {from: single.yaml}
            right:
              assembly:
                parts:
                  body: {from: ../body/arm.yaml}
                  wrist: {device: camera, width: 320, height: 240, driver: {type: array}}
                placement:
                  wrist: {on: body.flange}
            scene: {device: camera, width: 1280, height: 720, fovy: 55, driver: {type: array}}
        """)
    spec = load_robot(lab / "robots/inline.yaml")
    assert [p for p, _ in spec.root.walk()] == [
        "left", "left/body", "left/wrist", "right", "right/body", "right/wrist", "scene"]
    assert spec.root.find("right/wrist").fields["width"] == 320
    assert spec.root.find("scene").fields["fovy"] == 55.0


def test_a_dict_config_with_builtin_style_references(lab):
    spec = load_robot({"assembly": {"parts": {
        "body": {"from": str(lab / "body/arm.yaml")},
        "cam": {"device": "camera", "width": 64, "height": 48}}}})
    assert spec.root.name == "robot" and spec.root.find("cam").fields["height"] == 48


def test_defaults_still_compose_files(lab):
    _write(lab, "robots/mine.yaml", """\
        defaults: [dual.yaml]
        assembly:
          parts:
            left: {overrides: {body: {driver: {ip: 10.0.0.42}}}}
          placement:
            right: {pose: [0.02, -0.81, 0, 0, 0, 0, 1]}
        sim: {backend: mujoco}
        """)
    spec = load_robot(lab / "robots/mine.yaml")
    assert spec.root.find("left/body").driver["ip"] == "10.0.0.42"
    assert np.allclose(spec.root.placement["right"].pose[:2], [0.02, -0.81])
    assert spec.sim == {"backend": "mujoco"}


def test_safety_sections_merge_along_the_tree(lab):
    _write(lab, "robots/lab.yaml", """\
        assembly:
          parts:
            a: {from: dual.yaml}
            b: {from: dual.yaml, safety: {max_joint_step: 0.1}}
        safety: {max_joint_step: 0.5, max_joint_error: 0.4, clamp: true}
        """)
    spec = load_robot(lab / "robots/lab.yaml")
    # single.yaml's own safety is the innermost default; dual's part entry on right wins over it.
    assert joint_safety(spec, "a/left/body") == {
        "max_joint_step": 0.2, "max_joint_error": 0.4, "clamp": True, "joint_limit": 3.0}
    assert joint_safety(spec, "a/right/body")["joint_limit"] == 2.5
    # A part entry further up wins over everything below it.
    assert joint_safety(spec, "b/left/body")["max_joint_step"] == 0.1
    assert joint_safety(spec, "b/right/body")["joint_limit"] == 2.5


@pytest.mark.parametrize("text, match", [
    ("assembly: {parts: {body: {from: ../body/arm.yaml}}}\nbody: {driver: toy}\n",
     r"unknown top-level key\(s\) \['body'\]"),
    ("assembly: {parts: {}}\n", r"non-empty mapping"),
    ("assembly: {parts: {body: {from: ../body/arm.yaml}}, colour: red}\n",
     r"assembly: unknown key\(s\) \['colour'\]"),
    ("assembly: {parts: {body: {from: ../body/arm.yaml, ip: 1}}}\n",
     r"a part with from: takes"),
    ("assembly: {parts: {body: {device: body, joints: [], overrides: {}}}}\n",
     r"overrides is for a part with from:"),
    ("assembly: {parts: {body: {device: arm}}}\n", r"device: must be one of"),
    ("assembly: {parts: {body: {device: body, joints: [], wheels: 4}}}\n",
     r"unknown key\(s\) \['wheels'\] for a body"),
    ("assembly: {parts: {cam: {device: camera, width: 64}}}\n", r"a camera needs height"),
    ("assembly: {parts: {h: {device: end_effector, interface: joints}}}\n", r"lists its joints"),
    ("assembly: {parts: {g: {device: end_effector, interface: opening, joints: [{name: a}]}}}\n",
     r"opening interface has no joints"),
    ("assembly: {parts: {g: {device: end_effector, interface: fingers}}}\n",
     r"interface must be 'joints'"),
    ("assembly: {parts: {b: {device: body, joints: [{name: a}, {name: a}]}}}\n",
     r"duplicate joint name"),
    ("assembly: {parts: {b: {device: body, joints: [], frames: [{name: t}], grippers: [x]}}}\n",
     r"grippers lists frame names"),
    ("assembly: {parts: {b: {device: body, joints: [], model: {kp: [1]}}}}\n", r"with mjcf"),
    ("assembly: {parts: {b: {from: ../body/arm.yaml}}, placement: {c: {pose: [0,0,0,0,0,0,1]}}}\n",
     r"placement for 'c', which is not a part"),
    ("assembly: {parts: {b: {from: ../body/arm.yaml}}, placement: {b: {pose: [0, 0, 0]}}}\n",
     r"7 finite numbers"),
    ("assembly: {parts: {b: {from: ../body/arm.yaml}}, placement: {b: {on: body}}}\n",
     r"on: is '<part>.<mount>'"),
    ("assembly: {parts: {b: {from: ../body/arm.yaml}}, placement: {b: {origin: null, pose: "
     "[0,0,0,0,0,0,1]}}}\n", r"origin is for a part that moves"),
    ("assembly: {parts: {a/b: {from: ../body/arm.yaml}}}\n", r"without '/' or '.'"),
    ("assembly: {parts: {b: {from: ../body/missing.yaml}}}\n", r"not found"),
    ("assembly: {parts: {b: {from: ../body/arm.yaml, safety: {sim_check: true}}}}\n",
     r"only go in the root config"),
])
def test_bad_configs_say_what_is_wrong(lab, text, match):
    path = _write(lab, "robots/bad.yaml", text)
    with pytest.raises((ValueError, FileNotFoundError), match=match):
        load_robot(path)


def test_a_referenced_robot_config_may_not_have_root_only_sections(lab):
    _write(lab, "robots/with_layers.yaml", """\
        assembly: {parts: {body: {from: ../body/arm.yaml}}}
        layers: [{type: wbc}]
        """)
    _write(lab, "robots/top.yaml", "assembly: {parts: {a: {from: with_layers.yaml}}}\n")
    with pytest.raises(ValueError, match=r"layers: is only for the root config"):
        load_robot(lab / "robots/top.yaml")
    assert load_robot(lab / "robots/with_layers.yaml").layers == [{"type": "wbc"}]


def test_an_old_format_file_is_named_as_such(lab):
    _write(lab, "robots/old.yaml", "embodiment: {name: x}\nbody: {driver: toy}\n")
    _write(lab, "robots/top.yaml", "assembly: {parts: {a: {from: old.yaml}}}\n")
    with pytest.raises(ValueError, match=r"old format"):
        load_robot(lab / "robots/top.yaml")


def test_overrides_for_a_part_that_is_not_there(lab):
    with pytest.raises(ValueError, match=r"overrides for part\(s\) \['middle'\]"):
        load_robot(lab / "robots/dual.yaml", overrides={"middle/body": {"driver": {"ip": "x"}}})
