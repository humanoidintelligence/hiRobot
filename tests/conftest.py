"""Shared fixtures."""

import pytest

from hirobot.drivers._sdk import unitree_sdk2


@pytest.fixture(autouse=True)
def _fresh_unitree_channel():
    """Each test installs its own fake SDK, so forget the previous DDS initialisation."""
    unitree_sdk2._reset_for_tests()
    yield
    unitree_sdk2._reset_for_tests()
