"""The ``opening`` contract check runs, on the test gripper (no built-in driver has it yet)."""

from hirobot import Embodiment
from hirobot.devices.end_effector import build_end_effectors
from tests.drivers.contract import check_opening
from tests.drivers.end_effector.fake_gripper import NAME, registered


def test_contract():
    emb = Embodiment(name="arm", joints=[{"name": "j1"}], end_effectors=[
        {"name": "gripper", "has_gripper": True, "driver": {"type": NAME}}])
    made = []

    def make(ro):
        made.append(build_end_effectors(emb, read_only=ro)["gripper"])
        return made[-1]

    with registered():
        check_opening(NAME, make, lambda: sum(len(g.sent) for g in made), manifest=False)
