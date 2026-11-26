"""hirobot.record.Recorder end to end on the toy robot: files on disk, events, limits."""

from __future__ import annotations

import json
import logging
import threading
import time

import numpy as np
import pytest

pytest.importorskip("pyarrow")
pytest.importorskip("av")

import av  # noqa: E402
import pyarrow.parquet as pq  # noqa: E402

from hirobot import Action, ControlMode, Embodiment, Priority, make_robot  # noqa: E402
from hirobot.record import Recorder, RecorderError, append_verdict, read_qc  # noqa: E402
from tests.conftest import toy_config  # noqa: E402

FPS = 25

#: Most tests save on a thread (no ~0.3 s child start); the ``process`` tests
#: below and the first end-to-end test use the default subprocess worker.
THREAD = {"worker": "thread", "ring_mb": 8}


def _emb():
    return Embodiment(name="toy_arm", joints=[{"name": f"j{i}", "position_limit": (-1, 1)}
                                              for i in range(3)], control_hz=50)


@pytest.fixture
def robot():
    cfg = toy_config(_emb())
    # The array driver's own frames are 64x64 whatever width/height say.
    cfg["assembly"]["parts"]["cam"] = {"device": "camera", "width": 64, "height": 64,
                                       "driver": {"type": "array", "fps": 50,
                                                  "advance_on_read": True}}
    r = make_robot(cfg)
    r.connect()
    r.reset()
    yield r
    r.disconnect()


def _run(robot, steps=60, start=0.0):
    for k in range(steps):
        robot.get_observation()
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.full(3, start + 0.002 * k)))


def _record(robot, rec, task="pick the cube", steps=60, meta=None):
    events = []
    sub = rec.subscribe(events.append)
    n = rec.start(task, meta=meta)
    _run(robot, steps)
    assert rec.stop() == n
    assert rec.wait(60)
    sub.remove()
    return events


def test_one_episode_is_a_lerobot_v3_dataset(robot, tmp_path):
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        events = _record(robot, rec, meta={"operator": "ann", "robot_config": "toy"})
    assert [e.state for e in events] == ["saving", "saved"]
    saved = events[-1]
    assert saved.flags == () and saved.error is None
    root = tmp_path / saved.path.rsplit("/", 1)[-1]
    assert root.name.endswith("_pick-the-cube_0000")

    info = json.loads((root / "meta/info.json").read_text())
    assert info["codebase_version"] == "v3.0" and info["fps"] == FPS
    assert info["robot_type"] == "toy_arm" and info["splits"] == {"train": "0:1"}
    # The recorder's own keys are in meta/hi/info.json; LeRobot would drop them from info.json.
    assert not [k for k in info if k.startswith("hi.")]
    hi_info = json.loads((root / "meta/hi/info.json").read_text())
    assert hi_info["hi.robot_config"] == "toy" and hi_info["hi.action_mode"] == "position"
    assert hi_info["hi.recorder_fps"] == FPS and hi_info["hi.control_hz"] == 50
    assert set(hi_info) >= {"hi.hirobot_version"} and all(k.startswith("hi.") for k in hi_info)
    n = info["total_frames"]
    assert n == saved.frames and n >= FPS  # 60 steps at 50 Hz is 1.2 s
    feats = info["features"]
    for key in ("observation.state", "action", "observation.images.cam", "timestamp",
                "frame_index", "episode_index", "index", "task_index", "intervention",
                "hi.control_source", "hi.rejected_action", "hi.capture_time",
                "hi.stamp.body.t", "hi.stamp.cam.seq"):
        assert key in feats, key
    assert feats["action"]["names"] == ["j0.pos", "j1.pos", "j2.pos"]
    assert all("/" not in name for name in feats)

    data = pq.read_table(root / "data/chunk-000/file-000.parquet")
    assert data.num_rows == n
    assert set(data.column_names) == set(feats) - {"observation.images.cam"}
    assert data.column("frame_index").to_pylist() == list(range(n))
    np.testing.assert_allclose(data.column("timestamp").to_numpy(), np.arange(n) / FPS,
                               atol=1e-6)
    assert set(data.column("hi.control_source").to_pylist()) == {"policy"}
    assert np.asarray(data.column("action").to_pylist()).shape == (n, 3)

    with av.open(str(root / "videos/observation.images.cam/chunk-000/file-000.mp4")) as f:
        stream = f.streams.video[0]
        assert stream.codec_context.name == "h264"
        frames = list(f.decode(video=0))
    h, w = robot.get_observation().images["cam"].shape[:2]
    assert len(frames) == n and (frames[0].width, frames[0].height) == (w, h)
    assert feats["observation.images.cam"]["shape"] == [h, w, 3]
    assert feats["observation.images.cam"]["info"]["video.codec"] == "h264"
    np.testing.assert_allclose([float(fr.time) for fr in frames], np.arange(n) / FPS,
                               atol=1e-4)

    ep = pq.read_table(root / "meta/episodes/chunk-000/file-000.parquet").to_pylist()[0]
    assert ep["length"] == n and ep["tasks"] == ["pick the cube"]
    assert ep["dataset_from_index"] == 0 and ep["dataset_to_index"] == n
    assert ep["videos/observation.images.cam/to_timestamp"] == pytest.approx(n / FPS)
    assert "stats/action/mean" in ep
    tasks = pq.read_table(root / "meta/tasks.parquet")
    assert tasks.to_pylist() == [{"task_index": 0, "task": "pick the cube"}]
    stats = json.loads((root / "meta/stats.json").read_text())
    assert len(stats["action"]["mean"]) == 3 and len(stats["observation.images.cam"]["mean"]) == 3
    report = json.loads((root / "meta/hi/alignment.json").read_text())
    assert report["frames"] == n and "cam" in report["cameras"]
    meta = json.loads((root / "meta/hi/episode.json").read_text())
    assert meta["operator"] == "ann" and meta["task"] == "pick the cube"
    assert read_qc(root) == [dict(read_qc(root)[0], verdict="PASS", flags=[])]


def test_readings_are_kept_once_per_seq(robot, tmp_path):
    with Recorder(robot, tmp_path, fps=FPS, **THREAD) as rec:
        rec.start("dedup")
        for _ in range(5):
            robot.get_observation()
        ep = rec._episode
        # The toy body advances only on send_action; the camera on every read
        # (the same array object each time, a new seq: copied each time).
        assert ep.kept == {"state": 1, "cam": 5}
        sent = [m for m in rec._outbox if m[0] == "state"]
        assert not sent or sent[0][2][2].images == {}  # the image goes through the ring
        rec.cancel()


def test_queue_limit_refuses_start_and_cancel_drops(robot, tmp_path):
    gate = threading.Event()
    with Recorder(robot, tmp_path, fps=FPS, max_queue=2, **THREAD) as rec:
        sub = rec.subscribe(lambda e: gate.wait(30) if e.state == "saving" else None)
        for _ in range(2):
            rec.start("busy")
            _run(robot, 10)
            rec.stop()
        assert rec.queue_length == 2
        with pytest.raises(RecorderError, match="queue is full"):
            rec.start("one too many")
        gate.set()
        assert rec.wait(60) and rec.queue_length == 0
        sub.remove()
        number = rec.start("dropped")
        with pytest.raises(RecorderError, match="already recording"):
            rec.start("again")
        assert rec.cancel() == number and rec.cancel() is None
        with pytest.raises(RecorderError, match="not recording"):
            rec.stop()
    assert len(list(tmp_path.iterdir())) == 2  # the cancelled one was never written


def test_queued_image_memory_limit_refuses_start(robot, tmp_path):
    gate = threading.Event()
    # One 64x64 RGB frame is 12 KB; 10 frames queued exceed a 0.05 MB limit.
    with Recorder(robot, tmp_path, fps=FPS, max_queue_mb=0.05, **THREAD) as rec:
        sub = rec.subscribe(lambda e: gate.wait(30) if e.state == "saving" else None)
        rec.start("big")
        _run(robot, 10)
        rec.stop()
        with pytest.raises(RecorderError, match="max_queue_mb"):
            rec.start("refused")
        gate.set()
        assert rec.wait(60)
        sub.remove()
        rec.start("fine again")
        rec.cancel()


def test_stop_while_callbacks_run_on_another_thread_never_loses_the_episode(robot, tmp_path):
    from hirobot import CommandRecord, Observation
    from hirobot.core.stamps import Stamp

    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, (8, 8, 3), np.uint8)
    events = []
    # max_samples=1 and a tiny image buffer: every callback adds to the drop counts.
    with Recorder(robot, tmp_path, fps=FPS, max_samples=1, max_buffer_mb=1e-5,
                  max_queue=200, **THREAD) as rec:
        rec.subscribe(events.append)
        stop = threading.Event()

        def hammer():
            seq = 0
            while not stop.is_set():
                seq += 1
                # A camera name the episode has never seen: a new key each time.
                rec._on_observation(Observation(
                    joint_pos=np.zeros(3), images={f"c{seq % 97}": img},
                    stamps={"body": Stamp(seq * 1e-3, seq), f"c{seq % 97}": Stamp(seq, seq)}))
                rec._on_command(CommandRecord(t=seq * 1e-3, call="hold", action=None, lease=None,
                                              priority=None, accepted=True))

        threads = [threading.Thread(target=hammer) for _ in range(3)]
        for t in threads:
            t.start()
        try:
            numbers = []
            for k in range(100):
                rec.start(f"race {k}")
                numbers.append(rec.stop())
        finally:
            stop.set()
            for t in threads:
                t.join()
        assert rec.wait(120)
    done = [e.episode for e in events if e.state in ("saved", "failed")]
    assert sorted(done) == numbers  # every stopped episode was saved or reported


def test_save_flags_go_to_the_event_and_qc_and_a_verdict_can_be_added(robot, tmp_path):
    with Recorder(robot, tmp_path, fps=FPS, **THREAD) as rec:
        events = _record(robot, rec, steps=10)  # 0.2 s: too short
    saved = events[-1]
    assert "too_short" in {f.name for f in saved.flags}
    line = read_qc(saved.path)[0]
    assert line["source"] == "auto" and line["verdict"] == "FAIL"
    append_verdict(saved.path, 0, "PASS", note="short on purpose")
    assert [x["verdict"] for x in read_qc(saved.path)] == ["FAIL", "PASS"]


def test_a_failed_save_is_reported_and_the_worker_goes_on(robot, tmp_path):
    with Recorder(robot, tmp_path, fps=FPS, **THREAD) as rec:
        events = []
        rec.subscribe(events.append)
        rec.start("nothing read")  # no get_observation: nothing to align
        rec.stop()
        rec.wait(60)
        assert events[-1].state == "failed" and "no observation" in events[-1].error
        assert _record(robot, rec)[-1].state == "saved"


def test_teleop_takeover_is_recorded_as_intervention(robot, tmp_path):
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0, **THREAD) as rec:
        events = []
        rec.subscribe(events.append)
        rec.start("dagger")
        policy = robot.claim("policy", Priority.CALLER)
        for k in range(30):
            robot.get_observation()
            policy.send_action(Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.1)))
        teleop = robot.claim("teleop", Priority.TAKEOVER)
        for k in range(30):
            robot.get_observation()
            teleop.send_action(Action(mode=ControlMode.POSITION, joint_target=np.full(3, 0.2)))
            with pytest.raises(Exception):  # preempted: refused, recorded as rejected
                policy.send_action(Action(mode=ControlMode.POSITION,
                                          joint_target=np.full(3, 0.9)))
        rec.stop()
        rec.wait(60)
        teleop.release()
    data = pq.read_table(f"{events[-1].path}/data/chunk-000/file-000.parquet")
    src = data.column("hi.control_source").to_pylist()
    assert "policy" in src and "teleop" in src
    assert data.column("intervention").to_pylist() == [s == "teleop" for s in src]
    mask = data.column("hi.rejected_action_mask").to_pylist()
    assert any(mask) and not any(m for m, s in zip(mask, src) if s == "policy")
    report = json.loads(open(f"{events[-1].path}/meta/hi/alignment.json").read())
    assert {"claim", "preempted"} <= {e["kind"] for e in report["lease_events"]}


def test_recorder_errors_never_reach_the_control_loop(robot, tmp_path, caplog):
    rec = Recorder(robot, tmp_path, fps=FPS, **THREAD)
    rec.start("broken")
    rec._episode = object()  # every callback now raises inside the recorder
    _run(robot, 3)  # the robot logs the listener errors and goes on
    rec._episode = None
    assert rec.close(60) == 0
    _run(robot, 3)  # detached: no callback any more


def test_close_saves_an_open_episode_and_bad_arguments(robot, tmp_path):
    rec = Recorder(robot, tmp_path, fps=FPS, **THREAD)
    events = []
    rec.subscribe(events.append)
    rec.start("closing")
    _run(robot, 60)
    assert rec.close(60) == 0 and rec.close() == 0
    assert events[-1].state == "saved"
    meta = json.loads(open(f"{events[-1].path}/meta/hi/episode.json").read())
    assert meta["stopped_by"] == "close"
    with pytest.raises(RecorderError, match="closed"):
        rec.start("after close")
    with pytest.raises(ValueError, match="task"):
        Recorder(robot, tmp_path, **THREAD).start("  ")
    with pytest.raises(ValueError, match="whole number"):
        Recorder(robot, tmp_path, fps=29.97)


def test_default_fps(robot, tmp_path):
    rec = Recorder(robot, tmp_path, **THREAD)
    assert rec.fps == 30  # has a camera: min(30, control_hz)
    rec.close()
    plain = make_robot(toy_config(_emb()))
    plain.connect()
    try:
        rec = Recorder(plain, tmp_path, **THREAD)
        assert rec.fps == 50  # no camera: control_hz
        rec.close()
    finally:
        plain.disconnect()


# ------------------------------------------------------------------ the save worker


def test_the_default_worker_is_a_subprocess(robot, tmp_path):
    import os

    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        assert rec.worker == "process"
        events = _record(robot, rec)
        link = rec._link
        assert link.kind == "process" and link.pid not in (None, os.getpid())
    assert events[-1].state == "saved" and events[-1].frames >= FPS
    assert link.proc.returncode == 0  # close() ended the child


def test_a_repeated_seq_is_not_copied_again(robot, tmp_path):
    from hirobot.core.stamps import Stamp

    img = np.zeros((8, 8, 3), np.uint8)
    with Recorder(robot, tmp_path, fps=FPS, **THREAD) as rec:
        rec.start("same frame")
        obs = robot.get_observation()  # the robot's own frame: kept
        before = rec._ring.position
        # A 30 fps camera under a 50 Hz loop: the same object, the same seq.
        same = obs.model_copy(update={"images": {"cam": img},
                                      "stamps": dict(obs.stamps, cam=Stamp(1.0, 10**6))})
        for _ in range(5):
            rec._on_observation(same)
        assert rec._episode.kept["cam"] == 2
        assert rec._ring.position - before == 192  # one 8x8x3 frame copied, not five
        rec.cancel()


def test_a_full_ring_drops_frames_and_flags_them(robot, tmp_path, caplog):
    caplog.set_level(logging.WARNING, logger="hirobot.record.recorder")
    with Recorder(robot, tmp_path, fps=FPS, worker="thread", ring_mb=0.0) as rec:
        assert rec._ring.data_size >= 16 * 64 * 64 * 3  # 16 of the largest declared camera
        gate = threading.Event()
        release = rec._ring.release

        def stuck(end):  # the worker cannot keep up
            gate.wait(30)
            release(end)

        rec._ring.release = stuck
        events = []
        rec.subscribe(events.append)
        rec.start("slow worker")
        _run(robot, 60)
        assert rec._episode.dropped.get("cam", 0) > 1
        rec.stop()
        gate.set()
        assert rec.wait(60)
    assert "dropped_samples" in {f.name for f in events[-1].flags}
    full = [r for r in caplog.records if "frame ring" in r.getMessage()]
    assert len(full) == 1  # once per episode, not once per dropped frame
    assert "ring_mb" in full[0].getMessage()


def _ready(rec, deadline):
    while rec._link is None or not rec._link.ready:
        assert time.monotonic() < deadline, "the save worker did not start"
        time.sleep(0.01)
    return rec._link


def test_a_killed_worker_fails_its_episodes_and_the_next_one_saves(robot, tmp_path):
    deadline = time.monotonic() + 60
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        events = []
        rec.subscribe(events.append)
        rec.start("killed")
        _run(robot, 30)
        first = _ready(rec, deadline)
        first.proc.kill()
        while not first.dead:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        _run(robot, 10)  # the control loop goes on; nothing raises
        number = rec.stop()
        assert rec.wait(60)
        assert [e.state for e in events if e.episode == number] == ["saving", "failed"]
        assert "exited" in events[-1].error
        assert _record(robot, rec)[-1].state == "saved"
        assert rec._link is not first and rec.worker == "process"


def test_without_memfd_the_recorder_saves_on_a_thread(robot, tmp_path, monkeypatch, caplog):
    import hirobot.record.recorder as recorder_mod

    monkeypatch.setattr(recorder_mod, "can_use_process", lambda: False)
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        assert rec.worker == "thread"
        assert _record(robot, rec)[-1].state == "saved"
    assert "subprocess" in caplog.text


def test_a_child_that_cannot_start_falls_back_to_a_thread(robot, tmp_path, monkeypatch, caplog):
    import hirobot.record.worker as worker_mod

    monkeypatch.setattr(worker_mod, "_BOOT", "raise SystemExit(3)")
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        events = []
        rec.subscribe(events.append)
        rec.start("no child")
        _run(robot, 60)
        deadline = time.monotonic() + 60
        while rec.worker != "thread":
            assert time.monotonic() < deadline
            time.sleep(0.01)
        rec.stop()
        assert rec.wait(60)
        assert events[-1].state == "failed"  # it began on a child that died
        assert _record(robot, rec)[-1].state == "saved"
    assert "before it was ready" in caplog.text


def test_unpicklable_meta_fails_that_episode_only(robot, tmp_path):
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        events = []
        rec.subscribe(events.append)
        rec.start("bad meta", meta={"lock": threading.Lock()})
        _run(robot, 60)
        rec.stop()
        assert rec.wait(60)
        assert events[-1].state == "failed" and "meta" in events[-1].error
        assert _record(robot, rec)[-1].state == "saved"


def test_unpicklable_extra_is_not_recorded_and_the_episode_saves(robot, tmp_path):
    lock = threading.Lock()  # cannot be pickled to the child
    with Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0) as rec:
        events = []
        rec.subscribe(events.append)
        rec.start("odd extra")
        for k in range(60):
            obs = robot.get_observation()
            rec._on_observation(obs.model_copy(update={"extra": {"lock": lock}}))
            robot.send_action(Action(mode=ControlMode.POSITION,
                                     joint_target=np.full(3, 0.002 * k)))
        rec.stop()
        assert rec.wait(60)
    assert events[-1].state == "saved", events[-1].error
    assert "dropped_samples" not in {f.name for f in events[-1].flags}


# ------------------------------------------------------------------ resources


def _memfds():
    import os

    n = 0
    for fd in os.listdir("/proc/self/fd"):
        try:
            n += "memfd:" in os.readlink(f"/proc/self/fd/{fd}")
        except OSError:
            pass
    return n


def _settle(check, timeout=30.0):
    import gc

    deadline = time.monotonic() + timeout
    while True:
        gc.collect()
        if check() or time.monotonic() > deadline:
            return
        time.sleep(0.05)


@pytest.mark.skipif(not __import__("os").path.isdir("/proc/self/fd"), reason="needs /proc")
@pytest.mark.parametrize("closed", [True, False], ids=["close", "no-close"])
def test_a_recorder_frees_its_threads_ring_and_itself(robot, tmp_path, closed):
    import gc
    import weakref

    gc.collect()
    threads, fds = threading.active_count(), _memfds()
    refs = []
    for i in range(3):
        rec = Recorder(robot, tmp_path, fps=FPS, frozen_s=10.0, ring_mb=8)
        events = _record(robot, rec, steps=30)
        assert events[-1].state == "saved"
        if closed:
            assert rec.close(60) == 0
            # close() returns after the last event and with the dispatcher gone
            assert not rec._dispatcher.is_alive() and not rec._sender.is_alive()
            assert rec._ring.closed
        refs.append(weakref.ref(rec))
        del rec, events
    _settle(lambda: threading.active_count() == threads and _memfds() == fds
            and all(r() is None for r in refs))
    assert [r() for r in refs] == [None] * 3  # nothing keeps a dropped recorder alive
    assert threading.active_count() == threads, [t.name for t in threading.enumerate()]
    assert _memfds() == fds
    robot.get_observation()  # the robot has no stale listener left
    assert not robot._arbiter._observation_listeners


def test_the_child_ignores_the_cwd_and_pythonpath(robot, tmp_path, monkeypatch):
    """Started with ``-I``: a module in the cwd cannot shadow the standard library."""
    trap = tmp_path / "trap"
    trap.mkdir()
    for name in ("pickle", "json", "socket"):
        (trap / f"{name}.py").write_text("raise SystemExit('planted module imported')\n")
    monkeypatch.chdir(trap)
    with Recorder(robot, tmp_path / "out", fps=FPS, frozen_s=10.0) as rec:
        events = _record(robot, rec)
    assert events[-1].state == "saved"
