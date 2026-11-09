"""``toy`` passes the body contract (it has no SDK, so no manifest)."""

from hirobot import get_body, load_embodiment
from tests.drivers.contract import check_sim


def test_contract():
    made = []

    def make():
        robot = get_body("toy")(load_embodiment("unitree_g1"))
        made.append(robot)
        return robot

    # Every applied command advances the toy's clock by one tick.
    check_sim("toy", make, lambda: sum(r._ticks for r in made), manifest=False)
