"""Every registered driver has a contract test, so a new one cannot land without it.

A driver counts as covered when some ``tests/drivers/**/test_contract.py``
passes its registry name to a ``check_*`` function. An end-effector driver needs
one check per interface it lists: ``check_hand`` for ``joints``,
``check_opening`` for ``opening``. Only the host's sounddevice devices are
exempt: they need real audio hardware.
"""

import re
from pathlib import Path

from hirobot import available_bodies
from hirobot.devices.audio import available_audio
from hirobot.devices.camera import available_cameras
from hirobot.devices.end_effector import available_end_effectors, get_end_effector

EXEMPT = {"sounddevice_mic", "sounddevice_speaker"}


def test_every_driver_has_a_contract_test():
    here = Path(__file__).parent
    text = "\n".join(p.read_text() for p in here.rglob("test_contract.py"))
    covered = set(re.findall(r'check_\w+\(\s*"(\w+)"', text))
    registered = (set(available_bodies()) | set(available_end_effectors())
                  | set(available_cameras()) | set(available_audio()))
    missing = sorted(registered - covered - EXEMPT)
    assert not missing, f"no contract test for: {missing}"
    assert not (EXEMPT - registered), "EXEMPT lists a driver that no longer exists"


INTERFACE_CHECK = {"joints": "check_hand", "opening": "check_opening"}


def test_every_end_effector_interface_has_its_contract_check():
    here = Path(__file__).parent
    text = "\n".join(p.read_text() for p in here.rglob("test_contract.py"))
    missing = []
    for name in available_end_effectors():
        for interface in get_end_effector(name).interfaces:
            check = INTERFACE_CHECK[interface]
            if not re.search(rf'{check}\(\s*"{name}"', text):
                missing.append(f"{name}: {interface} ({check})")
    assert not missing, f"no contract check for: {missing}"
