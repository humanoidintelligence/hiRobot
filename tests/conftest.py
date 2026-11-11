"""Shared fixtures."""

import pytest

from hirobot.drivers._sdk import unitree_sdk2


@pytest.fixture(autouse=True)
def _fresh_unitree_channel():
    """Each test installs its own fake SDK, so forget the previous DDS initialisation."""
    unitree_sdk2._reset_for_tests()
    yield
    unitree_sdk2._reset_for_tests()


def mjcf_of(config) -> str:
    """The MJCF path (``~`` expanded) of the one body of a config (its ``model.mjcf``)."""
    import os

    from hirobot.loading.build import only_body
    from hirobot.loading.tree import load_robot

    _, dev = only_body(load_robot(config), "mjcf_of")
    return os.path.expanduser(dev.fields["model"]["mjcf"])


def toy_config(emb, driver="toy", **extra):
    """An ``assembly:`` config with one body for a simple :class:`Embodiment`.

    The body takes the embodiment's joints, end-effectors (as frames, ``has_gripper``
    ones as grippers) and ``control_hz``; ``driver`` is the body driver's type or
    a driver spec. Cameras become camera parts. ``extra`` adds top-level sections
    (``safety``, ``layers``, ``arbiter``).
    """
    data = emb.model_dump(mode="json")
    body = {"device": "body", "control_hz": data["control_hz"],
            "floating_base": data["floating_base"],
            "joints": data["joints"],
            "frames": [{"name": e["name"]} for e in data["end_effectors"]],
            "grippers": [e["name"] for e in data["end_effectors"] if e["has_gripper"]],
            "driver": {"type": driver} if isinstance(driver, str) else dict(driver)}
    parts = {"body": body}
    for cam in data["cameras"]:
        parts[cam["name"]] = {"device": "camera", "width": cam["width"], "height": cam["height"],
                              "driver": cam["driver"]}
    return {"assembly": {"name": data["name"], "parts": parts}, **extra}
