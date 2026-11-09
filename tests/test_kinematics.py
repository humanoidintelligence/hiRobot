"""Forward kinematics: hand-worked chains, input checks, and parity with mj_kinematics."""

import os

import numpy as np
import pytest

from hirobot.kinematics import Kinematics

# A free-floating base, a 2-link arm with an offset joint anchor and rotated bodies,
# and a fixed child body.
ARM = """
<mujoco>
  <worldbody>
    <body name="base" pos="0 0 1">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="upper" pos="0 0.1 0" quat="0.7071068 0 0 0.7071068">
        <joint name="shoulder_joint" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <body name="lower" pos="0.3 0 0" quat="0.9238795 0.3826834 0 0">
          <joint name="elbow_joint" axis="0 0 1" pos="0.05 0 0"/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
          <body name="tip" pos="0.25 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

FIXED = """
<mujoco>
  <worldbody>
    <body name="link1" pos="0 0 0.5">
      <joint name="j1" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="link2" pos="1 0 0">
        <joint name="j2" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <body name="end" pos="1 0 0"><inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/></body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

G1 = "~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml"


def _write(tmp_path, text, name="m.xml"):
    p = tmp_path / name
    p.write_text(text)
    return str(p)


def _same_quat(a, b, atol=1e-9):
    a, b = np.asarray(a), np.asarray(b)
    return np.allclose(a, b, atol=atol) or np.allclose(a, -b, atol=atol)


def test_planar_two_link(tmp_path):
    kin = Kinematics.from_mjcf(_write(tmp_path, FIXED))
    assert kin.body_names == ("link1", "link2", "end")
    assert kin.parents == (-1, 0, 1)
    assert kin.joint_names == ("j1", "j2")
    assert not kin.floating_base
    poses = kin.forward([np.pi / 2, -np.pi / 2])
    assert np.allclose(poses["link2"][0], [0, 1, 0.5])
    assert np.allclose(poses["end"][0], [1, 1, 0.5])
    assert _same_quat(poses["end"][1], [0, 0, 0, 1])
    yaw90 = [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    assert _same_quat(poses["link1"][1], yaw90)  # j2 sits on link2, so link2 is back at 0


def test_batch_and_joint_names(tmp_path):
    kin = Kinematics.from_mjcf(_write(tmp_path, FIXED))
    q = np.random.default_rng(0).uniform(-2, 2, (4, 3, 2))
    batch = kin.forward(q)
    assert batch.pos.shape == (4, 3, 3, 3) and batch.quat.shape == (4, 3, 3, 4)
    one = kin.forward(q[2, 1])
    assert np.allclose(batch.pos[2, 1], one.pos)
    # Columns by name, reordered; a missing joint stays at 0.
    swapped = kin.forward(q[..., ::-1], joint_names=["j2", "j1"])
    assert np.allclose(swapped.pos, batch.pos)
    only_j2 = kin.forward(q[..., 1:], joint_names=["j2"])
    zeroed = kin.forward(np.stack([np.zeros_like(q[..., 0]), q[..., 1]], -1))
    assert np.allclose(only_j2.pos, zeroed.pos)


def test_input_errors(tmp_path):
    kin = Kinematics.from_mjcf(_write(tmp_path, FIXED))
    with pytest.raises(ValueError, match="pass joint_names"):
        kin.forward([0.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="not in"):
        kin.forward([0.0], joint_names=["nope"])
    with pytest.raises(ValueError, match="no free joint"):
        kin.forward([0.0, 0.0], base_pos=[0, 0, 1])
    with pytest.raises(KeyError, match="no body"):
        kin.forward([0.0, 0.0])["nope"]
    with pytest.raises(FileNotFoundError):
        Kinematics.from_mjcf(str(tmp_path / "missing.xml"))
    with pytest.raises(ValueError, match="unknown kinematics backend"):
        Kinematics.from_mjcf(_write(tmp_path, FIXED), backend="pinocchio")


@pytest.mark.parametrize("snippet, match", [
    ('<body name="b" euler="0 0 1"><joint name="j"/></body>', "euler"),
    ('<body name="b"><joint name="j" type="slide"/></body>', "hinge joints only"),
    ('<body name="b"><joint name="j" ref="0.3"/></body>', "ref"),
    ('<frame><body name="b"/></frame>', "frame"),
])
def test_unsupported_mjcf_is_rejected(tmp_path, snippet, match):
    xml = f"<mujoco><worldbody>{snippet}</worldbody></mujoco>"
    with pytest.raises(ValueError, match=match):
        Kinematics.from_mjcf(_write(tmp_path, xml))


def test_default_joint_class_is_rejected(tmp_path):
    xml = ('<mujoco><default><joint axis="1 0 0"/></default>'
           '<worldbody><body name="b"><joint name="j"/></body></worldbody></mujoco>')
    with pytest.raises(ValueError, match="default"):
        Kinematics.from_mjcf(_write(tmp_path, xml))


def test_floating_base_and_anchor(tmp_path):
    kin = Kinematics.from_mjcf(_write(tmp_path, ARM))
    assert kin.floating_base and kin.joint_names == ("shoulder_joint", "elbow_joint")
    # Default base pose is the MJCF pos. At zero angles: upper is yawed 90 deg, so its x
    # is world y; lower's quat turns about x, so the tip is 0.1 + 0.3 + 0.25 along world y.
    rest = kin.forward([0.0, 0.0])
    assert np.allclose(rest["base"][0], [0, 0, 1])
    assert np.allclose(rest["tip"][0], [0.0, 0.65, 1.0], atol=1e-6)
    # Rotating about the elbow anchor (0.05 m along lower's x) keeps the anchor fixed.
    for elbow in (0.0, 0.7, -1.3):
        p = kin.forward([0.4, elbow])
        anchor = p["lower"][0] + _rot(p["lower"][1]) @ [0.05, 0, 0]
        p0 = kin.forward([0.4, 0.0])
        anchor0 = p0["lower"][0] + _rot(p0["lower"][1]) @ [0.05, 0, 0]
        assert np.allclose(anchor, anchor0)
    # Moving the base moves everything rigidly.
    yaw = [0, 0, np.sin(0.3), np.cos(0.3)]
    moved = kin.forward([0.4, 0.7], base_pos=[1, 2, 3], base_quat=yaw)
    still = kin.forward([0.4, 0.7], base_pos=[0, 0, 0], base_quat=[0, 0, 0, 1])
    assert np.allclose(moved.pos, (_rot(yaw) @ still.pos.T).T + [1, 2, 3])


def _rot(q):
    from hirobot.transforms import quat_to_matrix

    return quat_to_matrix(q)


def _parity(path, n_samples=50, seed=0):
    a = Kinematics.from_mjcf(path)
    b = Kinematics.from_mjcf(path, backend="mujoco")
    assert b.backend == "mujoco"
    assert (a.body_names, a.parents, a.joint_names) == (b.body_names, b.parents, b.joint_names)
    rng = np.random.default_rng(seed)
    q = rng.uniform(-1.5, 1.5, (n_samples, len(a.joint_names)))
    bp = rng.normal(size=(n_samples, 3)) if a.floating_base else None
    bq = rng.normal(size=(n_samples, 4)) if a.floating_base else None
    pa, pb = a.forward(q, bp, bq), b.forward(q, bp, bq)
    assert np.abs(pa.pos - pb.pos).max() < 1e-12
    assert np.abs(pa.quat - pb.quat).max() < 1e-12  # both w >= 0
    # Default base pose too.
    assert np.abs(a.forward(q[0]).pos - b.forward(q[0]).pos).max() < 1e-12


def test_numpy_matches_mujoco(tmp_path):
    pytest.importorskip("mujoco")
    _parity(_write(tmp_path, ARM))
    _parity(_write(tmp_path, FIXED, "fixed.xml"))


def test_numpy_matches_mujoco_on_g1():
    pytest.importorskip("mujoco")
    path = os.path.expanduser(G1)
    if not os.path.isfile(path):
        pytest.skip("G1 model not present (fetched on demand)")
    _parity(path)
