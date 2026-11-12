"""``robotiq_urcap`` passes the opening contract against a fake URCap socket."""

from hirobot import Embodiment
from hirobot.devices.end_effector import build_end_effectors
from tests.drivers.contract import check_opening
from tests.drivers.end_effector.robotiq.urcap.fake_urcap import FakeUrcap


def test_contract():
    fake = FakeUrcap()
    emb = Embodiment(name="arm", joints=[{"name": "j1"}], end_effectors=[
        {"name": "gripper", "has_gripper": True,
         "driver": {"type": "robotiq_urcap", "host": fake.host, "port": fake.port}}])
    try:
        check_opening("robotiq_urcap",
                      lambda ro: build_end_effectors(emb, read_only=ro)["gripper"],
                      lambda: fake.sets, manifest=False)
    finally:
        fake.close()


def test_command_moves_the_gripper():
    # The real URCap only moves on SET POS (SET PRE is acked and ignored); the
    # fake models that, so a command written to the wrong register fails here.
    import time

    from hirobot.drivers.end_effector.robotiq.urcap import RobotiqUrcapGripper

    fake = FakeUrcap(pos=0)
    g = RobotiqUrcapGripper("gripper", [], host=fake.host, port=fake.port)
    try:
        g.connect()
        g.command_opening(0.0)
        deadline = time.monotonic() + 2.0
        while g.get_state().opening > 0.01 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert fake.actual == 255
        assert g.get_state().opening < 0.01
    finally:
        g.disconnect()
        fake.close()
