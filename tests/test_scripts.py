"""Every script under ``scripts/`` imports and parses ``--help`` without hardware or SDKs.

The scripts are run by hand on real robots, so nothing else imports them; this
catches a moved hiRobot import before someone is standing next to the robot.
"""

import subprocess
import sys
from pathlib import Path

import pytest

SCRIPTS = sorted((Path(__file__).resolve().parents[1] / "scripts").glob("*.py"))


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
def test_help(script):
    proc = subprocess.run([sys.executable, str(script), "--help"], capture_output=True, text=True,
                          timeout=60)
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "usage" in proc.stdout.lower()
