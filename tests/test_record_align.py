"""hirobot.record.align and hirobot.record.qc: the save phase on synthetic raw samples.

No pyarrow, PyAV or robot: a RawEpisode is filled by hand.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from hirobot import Action, CommandRecord, ControlMode, Embodiment, Observation, Priority
from hirobot.core.stamps import Stamp
from hirobot.record.align import AlignError, AlignOptions, RawEpisode, align
from hirobot.record.qc import append_verdict, frozen_spans, read_qc, thumbnail


def _emb(cameras=(), grippers=False, floating=False):
    ees = [{"name": "hand", "has_gripper": True}] if grippers else []
    return Embodiment(name="toy", joints=[{"name": f"j{i}", "position_limit": (-2, 2)}
                                          for i in range(2)],
                      end_effectors=ees, control_hz=50, floating_base=floating,
                      cameras=[{"name": c, "width": 8, "height": 6} for c in cameras])


def _obs(t, q, seq, **kw):
    return (t, {"body": Stamp(t, seq)}, Observation(joint_pos=np.asarray(q, float), **kw))


def _cmd(t, q=None, *, accepted=True, call="send_action", priority=None, lease=None,
         mode=ControlMode.POSITION, reason=None, **kw):
    action = None
    if call == "send_action":
        action = Action(mode=mode, joint_target=None if q is None else np.asarray(q, float), **kw)
    return (CommandRecord(t=t, call=call, action=action, lease=lease, priority=priority,
                          accepted=accepted, reason=reason), None)


def _raw(state, t_stop, images=None, commands=(), **kw):
    raw = RawEpisode(number=0, task="t", meta={}, wall_start=0.0, t_start=0.0, t_stop=t_stop)
    raw.state = list(state)
    raw.images = dict(images or {})
    raw.commands = list(commands)
    for k, v in kw.items():
        setattr(raw, k, v)
    return raw


def _ramp(n=101, dt=0.01):
    """Joint 0 goes 0 -> 1 rad over 1 s, read at 100 Hz."""
    return [_obs(k * dt, [k * dt, 0.0], k + 1) for k in range(n)]


def _names(aligned):
    return {f.name for f in aligned.flags}


def test_grid_interpolates_state_and_holds_the_action():
    cmds = [_cmd(0.0, [0.1, 0.0]), _cmd(0.5, [0.7, 0.0])]
    a = align(_raw(_ramp(), 1.0, commands=cmds), _emb(), AlignOptions(fps=30))
    assert a.length == 31 and a.times[-1] == pytest.approx(1.0)
    # Readings at 10 ms; frame k is at k/30 s, between two of them: linear.
    np.testing.assert_allclose(a.columns["observation.state"][:, 0], a.times, atol=1e-6)
    # The action is the last command at or before the frame, never interpolated.
    act = a.columns["action"][:, 0]
    np.testing.assert_allclose(act, np.where(a.times < 0.5 - 1e-9, 0.1, 0.7), atol=1e-6)
    assert a.action_mode == "position"
    assert a.features["action"]["names"] == ["j0.pos", "j1.pos"]
    assert set(a.columns["hi.control_source"]) == {"policy"}
    assert not a.columns["intervention"].any()


def test_capture_time_and_source_stamps():
    a = align(_raw(_ramp(), 1.0, commands=[_cmd(0.0, [0, 0])]), _emb(), AlignOptions(fps=10))
    # The state reading at or before each frame, in the source's clock.
    np.testing.assert_allclose(a.columns["hi.capture_time"], a.times, atol=1e-9)
    assert a.columns["hi.stamp.body.seq"].tolist() == [1 + 10 * k for k in range(11)]
    assert a.features["hi.stamp.body.t"]["dtype"] == "float64"


def test_gripper_takes_the_previous_reading_not_an_interpolation():
    state = [_obs(k * 0.1, [0, 0], k + 1, gripper={"hand": float(k >= 5)}) for k in range(11)]
    a = align(_raw(state, 1.0, commands=[_cmd(0.0, [0, 0])]), _emb(grippers=True),
              AlignOptions(fps=20))
    names = a.features["observation.state"]["names"]
    g = a.columns["observation.state"][:, names.index("hand.gripper")]
    assert set(g.tolist()) == {0.0, 1.0}  # never 0.5
    assert g[a.times < 0.5 - 1e-9].max() == 0.0 and g[a.times >= 0.5].min() == 1.0


def test_quaternions_are_normalized_and_kept_in_one_hemisphere():
    z90 = np.array([0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    q = [np.array([0, 0, 0, 1.0]), -z90]  # the second written with the other sign
    state = [_obs(k * 1.0, [0, 0], k + 1, base_orientation=q[k],
                  base_angular_velocity=np.zeros(3)) for k in range(2)]
    a = align(_raw(state, 1.0, commands=[_cmd(0.0, [0, 0])]), _emb(floating=True),
              AlignOptions(fps=2))
    imu = a.columns["observation.imu"]
    np.testing.assert_allclose(np.linalg.norm(imu[:, :4], axis=1), 1.0, atol=1e-6)
    # Halfway is 45 deg about z, not a quaternion through the origin.
    np.testing.assert_allclose(imu[1, :4], [0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8)],
                               atol=1e-6)


def _frames(times, start_seq=1, same=False):
    rng = np.random.default_rng(0)
    out = []
    for k, t in enumerate(times):
        img = np.zeros((6, 8, 3), np.uint8) if same else rng.integers(0, 255, (6, 8, 3), np.uint8)
        out.append((t, start_seq + k, img))
    return out


def test_images_take_the_nearest_frame_and_the_grid_is_the_overlap():
    cam = _frames([0.2 + k / 30 for k in range(25)])  # 0.2 .. 1.0 s
    a = align(_raw(_ramp(), 1.0, images={"cam": cam}, commands=[_cmd(0.0, [0, 0])]),
              _emb(cameras=["cam"]), AlignOptions(fps=30))
    assert a.times[0] == pytest.approx(0.2) and a.times[-1] <= 1.0 + 1e-9
    assert a.report["trimmed_start_s"] == pytest.approx(0.2)
    video = a.videos["observation.images.cam"]
    assert len(video) == a.length
    assert video[0] is cam[0][2] and video[-1] is cam[-1][2]
    assert np.abs(a.columns["hi.image_skew.cam"]).max() < 1e-6
    assert "alignment" not in _names(a)


def test_alignment_violations_flag_the_episode():
    cam = _frames([k * 0.1 for k in range(11)])  # 10 fps camera on a 30 fps grid
    a = align(_raw(_ramp(), 1.0, images={"cam": cam}, commands=[_cmd(0.0, [0, 0])]),
              _emb(cameras=["cam"]), AlignOptions(fps=30))
    rep = a.report["cameras"]["cam"]
    assert rep["violations"] > 0 and rep["max_offset_s"] > rep["limit_s"]
    assert "alignment" in _names(a)


def test_missing_frozen_and_gappy_cameras():
    emb = _emb(cameras=["a", "b", "c"])
    frozen = _frames([k / 30 for k in range(31)], same=True)
    gappy = _frames([0.0, 0.05, 0.9, 1.0])
    a = align(_raw(_ramp(), 1.0, images={"b": frozen, "c": gappy},
                   commands=[_cmd(0.0, [0, 0])]), emb, AlignOptions(fps=30))
    flags = {f.name: f.detail for f in a.flags}
    assert flags["missing_camera"].startswith("a:")
    assert flags["frozen_camera"].startswith("b:")
    assert flags["camera_gap"].startswith("c:")
    assert "observation.images.a" not in a.videos


def test_short_episode_dropped_samples_fault_and_no_commands():
    a = align(_raw(_ramp(n=20), 0.19, dropped={"cam": 3}, fault="driver damped"),
              _emb(), AlignOptions(fps=30))
    assert {"too_short", "dropped_samples", "fault", "no_commands"} <= _names(a)
    assert a.action_mode is None and "action" not in a.columns
    assert set(a.columns["hi.control_source"]) == {"hold"}


def test_control_source_intervention_and_hold():
    cmds = [_cmd(0.0, [0.1, 0], lease="policy", priority=Priority.CALLER),
            _cmd(0.3, [0.2, 0], lease="teleop", priority=Priority.TAKEOVER),
            _cmd(0.6, call="hold", lease="teleop", priority=Priority.TAKEOVER),
            _cmd(0.8, [0.3, 0], lease="policy", priority=Priority.CALLER)]
    a = align(_raw(_ramp(), 1.0, commands=cmds), _emb(), AlignOptions(fps=10))
    src = a.columns["hi.control_source"].tolist()
    assert src == ["policy"] * 3 + ["teleop"] * 3 + ["hold"] * 2 + ["policy"] * 3
    assert a.columns["intervention"].tolist() == [s == "teleop" for s in src]
    # A hold keeps the last action in the column (the robot holds that pose).
    assert a.columns["action"][6, 0] == pytest.approx(0.2)


def test_stale_commands_count_as_hold_and_pure_teleop_has_no_intervention():
    cmds = [_cmd(0.0, [0.1, 0], lease="teleop", priority=Priority.TAKEOVER)]
    a = align(_raw(_ramp(), 1.0, commands=cmds), _emb(),
              AlignOptions(fps=10, hold_after_s=0.25))
    assert a.columns["hi.control_source"].tolist() == ["teleop"] * 3 + ["hold"] * 8
    assert not a.columns["intervention"].any()


def _loop(offsets, dt=0.02, n=51):
    """A 50 Hz loop: reading k at k*dt holds q=k, the command answering it (q=k) at +offset."""
    state = [_obs(k * dt, [0.01 * k, 0.0], k + 1) for k in range(n)]
    cmds = [_cmd(k * dt + offsets[k], [0.01 * k, 0.0], lease="policy",
                 priority=Priority.CALLER) for k in range(n)]
    return _raw(state, n * dt, commands=cmds)


@pytest.mark.parametrize("offsets", [
    np.zeros(51),                                              # a sim: same timestamp
    np.full(51, 0.001),                                        # 1 ms after the reading
    np.random.default_rng(3).uniform(0.0, 0.015, 51),          # jittered 0-15 ms
], ids=["equal", "plus_1ms", "jitter"])
def test_row_k_pairs_the_state_with_the_command_that_followed_it(offsets):
    a = align(_loop(offsets), _emb(), AlignOptions(fps=50))
    state = a.columns["observation.state"][:, 0]
    act = a.columns["action"][:, 0]
    # The command sent in answer to reading k carries q=k: state and action agree per row.
    np.testing.assert_allclose(act, state, atol=1e-6)
    assert set(a.columns["hi.control_source"]) == {"policy"}


def test_pairing_with_a_coarser_grid_and_late_commands():
    # Readings at 100 Hz, grid at 25 Hz, each command 4 ms after its reading.
    state = [_obs(k * 0.01, [0.01 * k, 0.0], k + 1) for k in range(101)]
    cmds = [_cmd(k * 0.01 + 0.004, [0.01 * k, 0.0]) for k in range(101)]
    a = align(_raw(state, 1.01, commands=cmds), _emb(), AlignOptions(fps=25))
    np.testing.assert_allclose(a.columns["action"][:, 0], a.columns["observation.state"][:, 0],
                               atol=1e-6)


@pytest.mark.parametrize("offset", [0.0, 0.001])
def test_readings_slower_than_the_grid_take_each_command_of_the_chunk(offset):
    # A chunked loop: one observation per 0.2 s (5 Hz), then 10 commands at 50 Hz.
    state = [_obs(c * 0.2, [0.0, 0.0], c + 1) for c in range(6)]
    cmds = [_cmd(k * 0.02 + offset, [0.01 * k, 0.0], lease="policy", priority=Priority.CALLER)
            for k in range(60)]
    a = align(_raw(state, 1.2, commands=cmds), _emb(), AlignOptions(fps=50))
    assert a.length == 51  # the grid ends at the last reading (1.0 s)
    np.testing.assert_allclose(a.columns["action"][:, 0], 0.01 * np.arange(51), atol=1e-6)
    assert set(a.columns["hi.control_source"]) == {"policy"}


def test_long_velocity_chunks_are_not_zeroed_or_hold():
    # 1 s chunks of a 0.3 rad/s velocity at 50 Hz, one observation per chunk.
    state = [_obs(float(c), [0.0, 0.0], c + 1) for c in range(4)]
    cmds = [_cmd(k * 0.02 + 0.001, [0.3, 0.0], mode=ControlMode.VELOCITY, lease="policy",
                 priority=Priority.CALLER) for k in range(150)]
    a = align(_raw(state, 3.0, commands=cmds), _emb(), AlignOptions(fps=50))
    assert set(a.columns["hi.control_source"]) == {"policy"}
    np.testing.assert_allclose(a.columns["action"][:, 0], 0.3, atol=1e-6)


def test_a_refusal_marks_one_frame_when_readings_are_sparse():
    state = [_obs(k * 0.1, [0.0, 0.0], k + 1) for k in range(11)]  # 10 Hz
    cmds = [_cmd(0.0, [0.1, 0]),
            _cmd(0.55, [0.9, 0], accepted=False, reason="refused by the gate")]
    a = align(_raw(state, 1.0, commands=cmds), _emb(), AlignOptions(fps=50))
    assert np.flatnonzero(a.columns["hi.rejected_action_mask"]).tolist() == [27]
    assert a.columns["hi.rejected_action"][27, 0] == pytest.approx(0.9)


def test_ik_targets_and_refusals_pair_with_the_reading_they_followed():
    state = [_obs(k * 0.02, [0.0, 0.0], k + 1) for k in range(11)]
    cmds = []
    for k in range(11):
        rec = CommandRecord(t=k * 0.02 + 0.001, call="send_action", lease=None, priority=None,
                            accepted=True, action=Action(mode=ControlMode.POSITION,
                                                         joint_target=np.zeros(2)))
        cmds.append((rec, {"j0": float(k), "j1": 0.0}))
    cmds += [_cmd(0.1 + 0.005, [0.9, 0], accepted=False, reason="refused by the gate")]
    a = align(_raw(state, 0.21, commands=cmds), _emb(), AlignOptions(fps=50))
    np.testing.assert_allclose(a.columns["hi.ik_joint_target"][:, 0], np.arange(a.length))
    assert np.flatnonzero(a.columns["hi.rejected_action_mask"]).tolist() == [5]


def test_a_hold_keeps_position_targets_and_zeros_velocities():
    vel = [_cmd(0.0, [0.4, -0.2], mode=ControlMode.VELOCITY),
           _cmd(0.5, call="hold")]
    a = align(_raw(_ramp(), 1.0, commands=vel), _emb(), AlignOptions(fps=10))
    act = a.columns["action"]
    np.testing.assert_allclose(act[:5], [[0.4, -0.2]] * 5, atol=1e-6)
    np.testing.assert_allclose(act[5:], 0.0)  # the robot stopped
    assert a.columns["hi.control_source"].tolist() == ["policy"] * 5 + ["hold"] * 6
    pos = [_cmd(0.0, [0.4, -0.2]), _cmd(0.5, call="hold")]
    b = align(_raw(_ramp(), 1.0, commands=pos), _emb(), AlignOptions(fps=10))
    np.testing.assert_allclose(b.columns["action"], [[0.4, -0.2]] * 11, atol=1e-6)


def test_stale_velocity_commands_read_zero_after_hold_after_s():
    # The command watchdog holds the robot without reporting it: frames past
    # hold_after_s count as hold, and a velocity there is 0.
    emb = _emb(floating=True)
    cmds = [_cmd(0.0, mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.5, 0, 0.2]))]
    a = align(_raw(_ramp(), 1.0, commands=cmds), emb, AlignOptions(fps=10, hold_after_s=0.25))
    assert a.columns["hi.control_source"].tolist() == ["policy"] * 3 + ["hold"] * 8
    np.testing.assert_allclose(a.columns["action"][:3], [[0.5, 0, 0.2]] * 3, atol=1e-6)
    np.testing.assert_allclose(a.columns["action"][3:], 0.0)
    delta = Embodiment(name="toy", joints=[{"name": "j0", "position_limit": (-2, 2)}],
                       end_effectors=[{"name": "hand"}], control_hz=50)
    step = {"hand": np.array([0.01, 0, 0, 0, 0, 0.0])}
    state = [_obs(k * 0.1, [0.0], k + 1) for k in range(11)]
    b = align(_raw(state, 1.0, commands=[
        _cmd(0.0, mode=ControlMode.CARTESIAN_DELTA, ee_targets=step)]), delta,
        AlignOptions(fps=10, hold_after_s=0.25))
    np.testing.assert_allclose(b.columns["action"][:3, 0], 0.01, atol=1e-6)
    np.testing.assert_allclose(b.columns["action"][3:], 0.0)


def test_refused_commands_become_rejected_action():
    cmds = [_cmd(0.0, [0.1, 0]),
            _cmd(0.42, [0.9, 0], accepted=False, reason="preempted by teleop")]
    a = align(_raw(_ramp(), 1.0, commands=cmds), _emb(), AlignOptions(fps=10))
    mask = a.columns["hi.rejected_action_mask"]
    # 0.42 follows frame 4's reading (0.40) and comes before frame 5's (0.50).
    assert mask.tolist() == [k == 4 for k in range(11)]
    assert a.columns["hi.rejected_action"][4, 0] == pytest.approx(0.9)
    # 0, not NaN, where nothing was refused: pandas turns NaN in a list column into null.
    assert not np.isnan(a.columns["hi.rejected_action"]).any()
    np.testing.assert_array_equal(a.columns["hi.rejected_action"][~mask], 0.0)
    np.testing.assert_allclose(a.columns["action"][:, 0], 0.1, atol=1e-6)
    assert a.report["commands"] == {"accepted": 1, "refused": 1, "modes": {"position": 1}}


def test_mixed_modes_keep_the_majority_and_flag():
    emb = Embodiment(name="toy", joints=[{"name": "j0", "position_limit": (-2, 2)}],
                     end_effectors=[{"name": "hand"}], control_hz=50)
    pose = {"hand": np.array([0.1, 0, 0.3, 0, 0, 0, 1.0])}
    cmds = [_cmd(0.0, mode=ControlMode.CARTESIAN, ee_targets=pose),
            _cmd(0.2, mode=ControlMode.CARTESIAN, ee_targets=pose),
            _cmd(0.4, [0.5])]
    state = [_obs(k * 0.1, [0.0], k + 1) for k in range(11)]
    a = align(_raw(state, 1.0, commands=cmds), emb, AlignOptions(fps=10))
    assert a.action_mode == "cartesian"
    assert a.features["action"]["names"][:3] == ["hand.x", "hand.y", "hand.z"]
    np.testing.assert_allclose(a.columns["action"][2], pose["hand"], atol=1e-6)
    # Frames under the POSITION command have no cartesian action: NaN, not a stale pose.
    assert np.isnan(a.columns["action"][5]).all()
    assert "mixed_action_modes" in _names(a)


def test_ik_joint_targets_are_recorded_when_present():
    rec = CommandRecord(t=0.0, call="send_action", lease=None, priority=None, accepted=True,
                        action=Action(mode=ControlMode.POSITION, joint_target=np.zeros(2)))
    raw = _raw(_ramp(), 1.0, commands=[(rec, {"j1": 0.25, "j0": -0.5})])
    a = align(raw, _emb(), AlignOptions(fps=10))
    assert a.features["hi.ik_joint_target"]["names"] == ["j0.pos", "j1.pos"]
    np.testing.assert_allclose(a.columns["hi.ik_joint_target"][3], [-0.5, 0.25])
    b = align(_raw(_ramp(), 1.0, commands=[_cmd(0.0, [0, 0])]), _emb(), AlignOptions(fps=10))
    assert "hi.ik_joint_target" not in b.columns


def test_no_state_or_no_overlap_raises():
    with pytest.raises(AlignError, match="no observation"):
        align(_raw([], 1.0), _emb(), AlignOptions(fps=30))
    with pytest.raises(AlignError, match="all 7 were dropped"):
        align(_raw([], 1.0, dropped={"state": 7}), _emb(), AlignOptions(fps=30))
    late = _frames([2.0, 2.1])
    with pytest.raises(AlignError, match="do not overlap"):
        align(_raw(_ramp(), 3.0, images={"cam": late}), _emb(cameras=["cam"]),
              AlignOptions(fps=30))


# ------------------------------------------------------------------ qc


def test_frozen_spans_and_thumbnail():
    a = np.zeros((48, 64, 3), np.uint8)
    b = a.copy()
    b[0, 0] = 1  # below the 6-bit quantization: the same picture
    c = np.full_like(a, 200)
    assert thumbnail(a) == thumbnail(b) != thumbnail(c)
    t = np.arange(8) * 0.25
    thumbs = [thumbnail(x) for x in (a, b, a, a, a, c, c, a)]
    assert frozen_spans(t, thumbs, 1.0) == [(0.0, 1.0)]
    assert frozen_spans(t, thumbs, 1.5) == []


def test_append_verdict(tmp_path):
    with pytest.raises(FileNotFoundError):
        append_verdict(tmp_path, 0, "PASS")
    (tmp_path / "meta").mkdir()
    (tmp_path / "meta" / "info.json").write_text("{}")
    with pytest.raises(ValueError, match="verdict"):
        append_verdict(tmp_path, 0, "OK")
    line = append_verdict(tmp_path, 0, "FAIL", note="cube fell", reviewer="ann")
    append_verdict(tmp_path, 0, "PASS")
    lines = read_qc(tmp_path)
    assert [x["verdict"] for x in lines] == ["FAIL", "PASS"]
    assert lines[0] == line and line["source"] == "human" and line["reviewer"] == "ann"
    raw = (tmp_path / "meta" / "qc.jsonl").read_text().splitlines()
    assert [json.loads(s)["note"] for s in raw] == ["cube fell", ""]
