"""``unitree_motorcmds``: hands behind a MotorCmds_ service (H1 Inspire via h1_inspire_service)."""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import load_embodiment, make_robot
from hirobot.drivers.end_effector import UnitreeMotorCmdsHand
from hirobot.devices.end_effector import build_end_effectors, end_effector_joint_names
from tests.drivers.end_effector.unitree.sdk2_motorcmds.fake_sdk import _install, _MotorStates

CFG = "unitree_h1_inspire"


# One MotorStates_ for both hands: 0-5 right, 6-11 left; q in [0, 1], 0 = closed.
RIGHT_Q = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]


LEFT_Q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


STATE = _MotorStates(RIGHT_Q + LEFT_Q)


def test_config_layout():
    emb = load_embodiment(CFG)
    assert emb.num_dofs == 31
    assert end_effector_joint_names(emb) == emb.joint_names[19:]
    assert all(j.position_limit == (0.0, 1.0) for j in emb.joints[19:])


def test_make_robot_builds_assembly_with_inspire_hands():
    r = make_robot(CFG, overrides={"body": {"driver": {"type": "unitree_sdk2", "msg_family": "go"}}})
    assert type(r).__name__ == "RobotAssembly"
    assert set(r.end_effectors) == {"left_hand", "right_hand"}
    assert all(isinstance(h, UnitreeMotorCmdsHand) for h in r.end_effectors.values())


def test_hands_share_one_topic_pair_with_their_own_slots():
    hands = build_end_effectors(load_embodiment(CFG), "eth0")
    for h in hands.values():
        assert (h.cmd_topic, h.state_topic) == ("rt/inspire/cmd", "rt/inspire/state")
    assert hands["right_hand"].offset == 0
    assert hands["left_hand"].offset == 6


def test_hands_read_their_own_slots(monkeypatch):
    _install(monkeypatch, STATE)
    hands = build_end_effectors(load_embodiment(CFG), "eth0", read_only=True)
    for h in hands.values():
        h.connect()
    assert np.allclose(hands["right_hand"].get_state().q, RIGHT_Q)
    assert np.allclose(hands["left_hand"].get_state().q, LEFT_Q)
    st = hands["left_hand"].get_state()
    assert np.all(st.dq == 0) and np.all(np.isnan(st.tau))
    assert np.all(st.extra["lost"] == 0)
    assert hands["left_hand"].raw_state is STATE
    with pytest.raises(RuntimeError):
        hands["left_hand"].command(np.ones(6))


def test_command_writes_all_12_and_keeps_the_other_hand(monkeypatch):
    bus = _install(monkeypatch, STATE)
    hands = build_end_effectors(load_embodiment(CFG), "eth0")
    for h in hands.values():
        h.connect()
    hands["left_hand"].command(np.array([1.5, 0.0, 0.5, 0.5, -0.2, 1.0]))
    topic, q = bus.sent[-1]
    assert topic == "rt/inspire/cmd"
    # Left hand in slots 6-11, clipped to [0, 1]; the right hand holds its measured pose.
    assert np.allclose(q[6:], [1.0, 0.0, 0.5, 0.5, 0.0, 1.0])
    assert np.allclose(q[:6], RIGHT_Q)
    hands["right_hand"].command(np.full(6, 0.25))
    topic, q = bus.sent[-1]
    assert np.allclose(q[:6], 0.25)
    assert np.allclose(q[6:], [1.0, 0.0, 0.5, 0.5, 0.0, 1.0])  # left keeps its last target
    n = len(bus.sent)
    hands["right_hand"].damp()  # no passive mode: sends nothing
    assert len(bus.sent) == n
    with pytest.raises(ValueError):
        hands["right_hand"].command(np.array([np.nan] * 6))


def test_reset_holds_measured_pose(monkeypatch):
    bus = _install(monkeypatch, STATE)
    h = build_end_effectors(load_embodiment(CFG), "eth0")["right_hand"]
    h.connect()
    h.reset()
    assert np.allclose(bus.sent[-1][1], RIGHT_Q + LEFT_Q)


def test_refuses_to_command_with_a_short_state(monkeypatch):
    # A state with only 6 motors gives no pose for the other hand: never guess it.
    _install(monkeypatch, _MotorStates(RIGHT_Q))
    h = build_end_effectors(load_embodiment(CFG), "eth0")["right_hand"]
    h.connect()
    with pytest.raises(RuntimeError, match="blind"):
        h.command(np.full(6, 0.5))


def test_lost_counter_reported(monkeypatch):
    _install(monkeypatch, _MotorStates(RIGHT_Q + LEFT_Q, lost=3))
    h = build_end_effectors(load_embodiment(CFG), "eth0", read_only=True)["right_hand"]
    h.connect()
    assert np.all(h.get_state().extra["lost"] == 3)


def test_connect_times_out_without_service(monkeypatch):
    _install(monkeypatch, state=None)
    h = build_end_effectors(load_embodiment(CFG), "eth0")["left_hand"]
    h._connect_timeout_s = 0.05
    with pytest.raises(TimeoutError, match="h1_inspire_service"):
        h.connect()


def test_motor_slots_must_fit_the_message():
    emb = load_embodiment(CFG)
    emb.end_effectors[0].driver["motor_offset"] = 7  # 7..12 in a 12-motor message
    with pytest.raises(ValueError, match="do not fit"):
        build_end_effectors(emb, "eth0")


def test_one_hand_alone_on_its_own_topics(monkeypatch):
    # A hand alone on its topic pair (Dex1-1, BrainCo, Linker): num_motors defaults
    # to its joint count and there is no other hand to hold.
    bus = _install(monkeypatch, _MotorStates(LEFT_Q), state_topic="rt/brainco/left/state")
    emb = load_embodiment(CFG)
    emb.end_effectors[0].driver = {"type": "unitree_motorcmds", "cmd_topic": "rt/brainco/left/cmd",
                                   "state_topic": "rt/brainco/left/state"}
    h = build_end_effectors(emb, "eth0")["left_hand"]
    assert h.num_motors == 6 and h.offset == 0
    h.connect()
    assert np.allclose(h.get_state().q, LEFT_Q)
    h.command(np.full(6, 0.5))
    assert bus.sent[-1] == ("rt/brainco/left/cmd", [0.5] * 6)
