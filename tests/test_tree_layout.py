"""Placements and the Embodiment computed from an assembly tree (hirobot.loading.layout)."""

from __future__ import annotations

import textwrap
from pathlib import Path

import numpy as np
import pytest

from hirobot.loading.layout import node_layout
from hirobot.loading.tree import load_robot


def _write(root: Path, rel: str, text: str) -> Path:
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text))
    return path


@pytest.fixture
def lab(tmp_path):
    _write(tmp_path, "body/arm.yaml", """\
        device: body
        control_hz: 125
        joints: [{name: j1}, {name: j2}]
        frames: [{name: tcp, link: link2}]
        mounts: {flange: {link: link2, pose: [0, 0, 0.05, 0, 0, 0, 1]}}
        """)
    _write(tmp_path, "body/humanoid.yaml", """\
        device: body
        control_hz: 50
        floating_base: true
        joints: [{name: hip}, {name: wrist}]
        frames: [{name: left_hand, link: wrist_link}, {name: torso, link: torso_link}]
        mounts:
          wrist: {link: wrist_link}
          head: {link: torso_link, pose: [0.1, 0, 0.4, 0, 0, 0, 1]}
        """)
    _write(tmp_path, "body/base.yaml", """\
        device: body
        control_hz: 20
        floating_base: true
        joints: []
        mounts: {top: {link: base_link, pose: [0.2, 0, 0.45, 0, 0, 0, 1]}}
        """)
    _write(tmp_path, "end_effector/hand.yaml", """\
        device: end_effector
        interface: joints
        joints: [{name: f1}, {name: f2}]
        tcp: [0, 0, 0.1, 0, 0, 0, 1]
        mounts: {palm: {link: palm_link}}
        driver: {type: fake_hand}
        """)
    _write(tmp_path, "end_effector/gripper.yaml", """\
        device: end_effector
        interface: opening
        driver: {type: fake_gripper}
        """)
    _write(tmp_path, "camera/cam.yaml", """\
        device: camera
        width: 64
        height: 48
        fovy: 50
        driver: {type: array}
        """)
    _write(tmp_path, "robots/single.yaml", """\
        assembly:
          parts:
            body:  {from: ../body/arm.yaml}
            wrist: {from: ../camera/cam.yaml}
          placement:
            wrist: {on: body.flange, pose: [0, -0.07, 0, 0, 0, 0, 1]}
        """)
    _write(tmp_path, "robots/dual.yaml", """\
        assembly:
          parts:
            left:  {from: single.yaml}
            right: {from: single.yaml}
            scene: {from: ../camera/cam.yaml}
          placement:
            left:  {pose: [0, 0.4, 0, 0, 0, 0, 1]}
            right: {pose: [0, -0.4, 0, 0, 0, 0, 1]}
            scene: {pose: [1, 0, 0.8, 0, 0, 0, 1]}
        """)
    _write(tmp_path, "robots/humanoid.yaml", """\
        assembly:
          parts:
            body:      {from: ../body/humanoid.yaml}
            left_hand: {from: ../end_effector/hand.yaml}
            head:      {from: ../camera/cam.yaml}
          placement:
            left_hand: {on: body.wrist}
            head:      {on: body.head}
        """)
    return tmp_path


def _layout(lab: Path, text: str):
    return node_layout(load_robot(_write(lab, "robots/t.yaml", text)).root)


def test_a_single_robot_keeps_device_names_and_mounts_its_camera(lab):
    lay = node_layout(load_robot(lab / "robots/single.yaml").root)
    emb = lay.embodiment
    assert emb.joint_names == ["j1", "j2"] and emb.control_hz == 125
    assert not emb.floating_base and emb.kind.value == "arm"
    assert [e.name for e in emb.end_effectors] == ["tcp"]
    assert emb.end_effectors[0].frame.body == "link2"
    cam = emb.cameras[0]
    assert cam.name == "wrist" and cam.mount.body == "link2" and cam.mount.fovy == 50
    assert np.allclose(cam.mount.pos, [0, -0.07, 0.05])  # mount pose, then the placement pose
    assert lay.parts["wrist"].kind == "mounted" and lay.parts["body"].kind == "fixed"
    assert set(emb.parts) == {""} and emb.parts[""].joints == (0, 2)


def test_child_nodes_get_prefixes_and_their_placement(lab):
    lay = node_layout(load_robot(lab / "robots/dual.yaml").root)
    emb = lay.embodiment
    assert emb.joint_names == ["left/j1", "left/j2", "right/j1", "right/j2"]
    assert [e.name for e in emb.end_effectors] == ["left/tcp", "right/tcp"]
    assert emb.end_effectors[1].frame.body == "right/link2"
    cams = {c.name: c.mount for c in emb.cameras}
    assert set(cams) == {"left/wrist", "right/wrist", "scene"}
    assert cams["left/wrist"].body == "left/link2"
    assert cams["scene"].body == "world" and np.allclose(cams["scene"].pos, [1, 0, 0.8])
    assert emb.parts["left"].joints == (0, 2) and emb.parts["right"].joints == (2, 4)
    assert emb.parts["right"].end_effectors == ["right/tcp"]
    assert emb.parts["right"].cameras == ["right/wrist"]
    assert emb.parts[""].children == ["left", "right"]
    assert lay.children["right"].parts["body"].anchor == "world"
    assert np.allclose(lay.children["right"].parts["body"].pose[:3], [0, -0.4, 0])


def test_three_levels(lab):
    _write(lab, "robots/lab.yaml", """\
        assembly:
          parts:
            a: {from: dual.yaml}
            b: {from: dual.yaml}
          placement:
            b: {pose: [3, 0, 0, 0, 0, 1, 0]}
        """)
    lay = node_layout(load_robot(lab / "robots/lab.yaml").root)
    emb = lay.embodiment
    assert emb.joint_names[:2] == ["a/left/j1", "a/left/j2"] and emb.num_dofs == 8
    assert set(emb.parts) == {"", "a", "a/left", "a/right", "b", "b/left", "b/right"}
    assert emb.parts["b/right"].joints == (6, 8)
    assert emb.parts["b"].children == ["b/left", "b/right"]
    scene = {c.name: c.mount for c in emb.cameras}["b/scene"]
    # b is turned half a turn about z at x = 3: its scene camera at x = 1 ends up at x = 2.
    assert scene.body == "world" and np.allclose(scene.pos, [2, 0, 0.8])
    right = lay.children["b"].children["right"].parts["body"]
    assert np.allclose(right.pose[:3], [3, 0.4, 0])


def test_a_humanoid_is_a_floating_node_and_its_hand_replaces_the_body_frame(lab):
    lay = node_layout(load_robot(lab / "robots/humanoid.yaml").root)
    emb = lay.embodiment
    assert lay.floating and lay.moving == "body"
    assert emb.floating_base and emb.kind.value == "humanoid" and emb.control_hz == 50
    assert emb.joint_names == ["hip", "wrist", "f1", "f2"]
    ees = {e.name: e for e in emb.end_effectors}
    assert list(ees) == ["torso", "left_hand"]  # the body's left_hand frame gave way to the hand
    hand = ees["left_hand"]
    assert hand.joints == ["f1", "f2"] and hand.driver == {"type": "fake_hand"}
    assert hand.frame.body == "wrist_link" and np.allclose(hand.frame.pos, [0, 0, 0.1])
    assert {c.name: c.mount.body for c in emb.cameras} == {"head": "torso_link"}
    assert emb.parts[""].moving


def test_a_gripper_on_the_opening_interface_has_a_gripper(lab):
    lay = _layout(lab, """\
        assembly:
          parts:
            body: {from: ../body/arm.yaml}
            grip: {from: ../end_effector/gripper.yaml}
          placement:
            grip: {on: body.flange}
        """)
    ees = {e.name: e for e in lay.embodiment.end_effectors}
    assert ees["grip"].has_gripper and ees["grip"].joints == []
    assert not ees["tcp"].has_gripper


def test_grippers_of_the_body_driver(lab):
    _write(lab, "body/agi.yaml", """\
        device: body
        joints: [{name: a}]
        frames: [{name: left_gripper, link: l7}, {name: head, link: h}]
        grippers: [left_gripper]
        """)
    emb = node_layout(load_robot(lab / "body/agi.yaml").root).embodiment
    assert [(e.name, e.has_gripper) for e in emb.end_effectors] == [
        ("left_gripper", True), ("head", False)]


def test_a_mobile_base_with_an_arm_is_a_floating_node(lab):
    lay = _layout(lab, """\
        assembly:
          parts:
            base: {from: ../body/base.yaml}
            arm:  {from: single.yaml}
          placement:
            arm: {on: base.top}
        """)
    emb = lay.embodiment
    assert lay.floating and lay.moving == "base"
    assert emb.joint_names == ["arm/j1", "arm/j2"] and emb.floating_base
    assert emb.control_hz == 20
    assert lay.parts["arm"].kind == "mounted" and lay.parts["arm"].anchor == "base_link"
    assert lay.children["arm"].parts["body"].anchor == "base_link"
    assert emb.parts[""].moving and not emb.parts["arm"].moving


def test_a_cell_with_a_humanoid_has_a_fixed_frame_and_an_origin(lab):
    lay = _layout(lab, """\
        assembly:
          control_hz: 30
          parts:
            g1:    {from: humanoid.yaml}
            arm:   {from: single.yaml}
            scene: {from: ../camera/cam.yaml}
          placement:
            g1:    {origin: [1, 0, 0, 0, 0, 0, 1]}
            arm:   {pose: [2, 0, 0.75, 0, 0, 1, 0]}
        """)
    emb = lay.embodiment
    assert not lay.floating and emb.control_hz == 30
    assert lay.parts["g1"].kind == "moving" and np.allclose(lay.parts["g1"].origin[:3], [1, 0, 0])
    assert emb.parts["g1"].moving and not emb.parts["arm"].moving
    assert emb.floating_base  # one moving part: its base signals are the robot's
    cams = {c.name: c.mount.body for c in emb.cameras}
    assert cams == {"g1/head": "g1/torso_link", "arm/wrist": "arm/link2", "scene": "world"}


def test_two_humanoids(lab):
    lay = _layout(lab, """\
        assembly:
          parts:
            a: {from: humanoid.yaml}
            b: {from: humanoid.yaml}
          placement:
            b: {origin: [0, 2, 0, 0, 0, 0, 1]}
        """)
    assert not lay.floating
    assert lay.parts["a"].kind == lay.parts["b"].kind == "moving"
    assert np.allclose(lay.parts["a"].origin, [0, 0, 0, 0, 0, 0, 1])
    assert lay.embodiment.parts["a"].moving and lay.embodiment.parts["b"].moving


@pytest.mark.parametrize("placement, match", [
    ("wrist: {on: nobody.flange}", r"no part 'nobody'"),
    ("wrist: {on: body.nomount}", r"declares no mount 'nomount'; its mounts: \['flange'\]"),
    ("wrist: {on: wrist.flange}", r"cannot be mounted on itself"),
    ("wrist: {origin: [0, 0, 0, 0, 0, 0, 1]}", r"origin: is for a part that moves by itself"),
    ("wrist: {on: body}", r"'<part>.<mount>'"),
    ("wrist: {pose: [0, 0, 0]}", r"7 finite numbers"),
])
def test_bad_placements(lab, placement, match):
    with pytest.raises(ValueError, match=match):
        _layout(lab, f"""\
            assembly:
              parts:
                body:  {{from: ../body/arm.yaml}}
                wrist: {{from: ../camera/cam.yaml}}
              placement:
                {placement}
            """)


def test_a_cycle_of_mounts_is_refused(lab):
    with pytest.raises(ValueError, match=r"cycle: (a -> b -> a|b -> a -> b)"):
        _layout(lab, """\
            assembly:
              parts:
                a: {from: ../end_effector/hand.yaml}
                b: {from: ../end_effector/hand.yaml}
              placement:
                a: {on: b.palm}
                b: {on: a.palm}
            """)


def test_a_moving_part_cannot_be_mounted_or_posed(lab):
    with pytest.raises(ValueError, match=r"moves by itself.*cannot be mounted"):
        _layout(lab, """\
            assembly:
              parts:
                arm:  {from: ../body/arm.yaml}
                base: {from: ../body/base.yaml}
              placement:
                base: {on: arm.flange}
            """)
    with pytest.raises(ValueError, match=r"moves by itself; give the origin"):
        _layout(lab, """\
            assembly:
              parts:
                g1: {from: humanoid.yaml}
              placement:
                g1: {pose: [0, 0, 0, 0, 0, 0, 1]}
            """)


def test_mounting_on_a_node_is_refused(lab):
    with pytest.raises(ValueError, match=r"'left' is an assembly"):
        _layout(lab, """\
            assembly:
              parts:
                left: {from: single.yaml}
                cam:  {from: ../camera/cam.yaml}
              placement:
                cam: {on: left.flange}
            """)


def test_clashing_names_are_refused(lab):
    with pytest.raises(ValueError, match=r"name more than one"):
        _layout(lab, """\
            assembly:
              parts:
                body: {from: ../body/arm.yaml}
                tcp:  {from: ../camera/cam.yaml}
            """)
    with pytest.raises(ValueError, match=r"duplicate joint names"):
        _layout(lab, """\
            assembly:
              parts:
                a: {from: ../body/arm.yaml}
                b: {from: ../body/arm.yaml}
            """)
