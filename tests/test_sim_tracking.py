"""In the simulation, each robot holds its home pose under its body config's gains.

A robot config built with ``sim=True`` is sent its own measured pose after
``reset`` for 3 s. After the first second every joint must stay within
:data:`MAX_ERROR` of that target and move slower than :data:`MAX_SPEED`. This
catches gains too soft to hold the robot against gravity (Franka FR3 joint2
sagged 1.36 rad under the driver's default kp 60) and too stiff for light links
(Unitree H1-2's wrists oscillated at up to 264 rad/s). Configs whose model is not
downloaded (``hirobot assets fetch --all``) are skipped.
"""

import logging
from pathlib import Path

import numpy as np
import pytest

from hirobot import Action, ControlMode, make_robot
from hirobot.loading.config import load_config

pytest.importorskip("mujoco")

MAX_ERROR = 0.05  # rad
MAX_SPEED = 1.0  # rad/s
SECONDS = 3.0

# Robot configs checked here, by the body they use.
CONFIGS = ["fourier_gr1t1", "fourier_gr1t2", "franka_fr3_droid", "unitree_h1_2"]


def _models(config):
    """The model files of the config's bodies (``model.mjcf`` or ``model.urdf``)."""
    from hirobot.loading.tree import load_robot

    out = []

    def walk(node):
        for dev in node.parts.values():
            if hasattr(dev, "parts"):
                walk(dev)
            elif getattr(dev, "kind", None) == "body":
                model = dev.fields.get("model") or {}
                path = model.get("mjcf") or model.get("urdf")
                if path:
                    out.append(Path(path).expanduser())

    walk(load_robot(load_config(config)).root)
    return out


@pytest.mark.parametrize("config", CONFIGS)
def test_holds_home_pose_in_sim(config):
    missing = [p for p in _models(config) if not p.is_file()]
    if missing:
        pytest.skip(f"{missing[0]} not downloaded (hirobot assets fetch --all)")
    logging.getLogger("hirobot").setLevel(logging.ERROR)
    robot = make_robot(config, sim=True)
    robot.connect()
    try:
        robot.reset()
        emb = robot.embodiment
        target = robot.get_observation().joint_pos.copy()
        error = np.zeros(emb.num_dofs)
        speed = np.zeros(emb.num_dofs)
        for i in range(int(SECONDS * emb.control_hz)):
            robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
            obs = robot.get_observation()
            if i >= emb.control_hz:
                error = np.maximum(error, np.abs(obs.joint_pos - target))
                speed = np.maximum(speed, np.abs(obs.joint_vel))
    finally:
        robot.disconnect()
    names = emb.joint_names
    bad = [f"{names[k]}: error {error[k]:.3f} rad, speed {speed[k]:.1f} rad/s"
           for k in range(emb.num_dofs) if error[k] > MAX_ERROR or speed[k] > MAX_SPEED]
    assert not bad, f"{config} does not hold its home pose: " + "; ".join(bad)


def test_a_welded_humanoid_still_turns_its_waist():
    """``sim.fixed_base`` welds the pelvis to the world; MuJoCo then no longer filters
    its contacts with its children, and H1's overlapping pelvis / torso_link meshes
    locked waist_yaw (it stayed at 0 against a target of 0.5)."""
    config = "unitree_h1_inspire"
    missing = [p for p in _models(config) if not p.is_file()]
    if missing:
        pytest.skip(f"{missing[0]} not downloaded (hirobot assets fetch --all)")
    logging.getLogger("hirobot").setLevel(logging.ERROR)
    robot = make_robot(config, sim={"fixed_base": ["body"]})
    robot.connect()
    try:
        robot.reset()
        emb = robot.embodiment
        target = robot.get_observation().joint_pos.copy()
        waist = emb.joint_names.index("waist_yaw")
        target[waist] = 0.5
        for _ in range(int(2 * emb.control_hz)):
            robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
        assert abs(robot.get_observation().joint_pos[waist] - 0.5) < MAX_ERROR
    finally:
        robot.disconnect()
