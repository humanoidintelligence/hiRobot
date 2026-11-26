"""hirobot.core.features: Observation / Action as named vectors."""

from __future__ import annotations

import numpy as np
import pytest

from hirobot import (
    Action,
    ControlMode,
    Embodiment,
    Observation,
    action_features,
    flatten_action,
    flatten_observation,
    image_features,
    load_embodiment,
    make_robot,
    observation_features,
)


def _arm():
    return Embodiment(name="arm", kind="arm", floating_base=False, control_hz=50,
                      joints=[{"name": n} for n in ("shoulder", "elbow", "wrist")],
                      end_effectors=[{"name": "tcp", "has_gripper": True}],
                      cameras=[{"name": "wrist_cam", "width": 4, "height": 3}])


def _humanoid():
    return Embodiment(name="hum", control_hz=50, floating_base=True,
                      joints=[{"name": n} for n in ("hip", "knee")])


def _check(features, flat):
    assert set(features) == set(flat)
    for key, names in features.items():
        assert flat[key].shape == (len(names),) and flat[key].dtype == np.float32
        assert len(set(names)) == len(names)


def test_observation_names_follow_the_embodiment():
    emb = _arm()
    obs = Observation(joint_pos=np.array([1.0, 2.0, 3.0]), joint_vel=np.zeros(3),
                      gripper={"tcp": 0.5}, ee_poses={"tcp": np.arange(7.0)})
    feats, flat = observation_features(emb, obs), flatten_observation(obs, emb)
    _check(feats, flat)
    assert feats["observation.state"] == ["shoulder.pos", "elbow.pos", "wrist.pos",
                                          "tcp.gripper"]
    assert np.allclose(flat["observation.state"], [1, 2, 3, 0.5])
    assert feats["observation.ee_pose"][0] == "tcp.x" and feats["observation.ee_pose"][-1] == "tcp.qw"
    assert np.allclose(flat["observation.ee_pose"], np.arange(7.0))
    assert feats["observation.joint_vel"] == ["shoulder.vel", "elbow.vel", "wrist.vel"]
    # Not filled: left out. No floating base: no imu / base, even in the schema.
    assert "observation.joint_torque" not in flat
    assert "observation.imu" not in observation_features(emb)
    assert "observation.joint_torque" in observation_features(emb)


def test_missing_gripper_is_nan():
    emb = _arm()
    flat = flatten_observation(Observation(joint_pos=np.zeros(3)), emb)
    assert np.isnan(flat["observation.state"][-1])


def test_floating_base_imu_and_base():
    emb = _humanoid()
    obs = Observation(joint_pos=np.zeros(2), base_orientation=np.array([0, 0, 0, 1.0]),
                      base_angular_velocity=np.array([1.0, 2, 3]))
    feats, flat = observation_features(emb, obs), flatten_observation(obs, emb)
    _check(feats, flat)
    assert feats["observation.imu"] == ["imu.qx", "imu.qy", "imu.qz", "imu.qw", "imu.wx",
                                        "imu.wy", "imu.wz", "imu.ax", "imu.ay", "imu.az"]
    assert np.allclose(flat["observation.imu"][:7], [0, 0, 0, 1, 1, 2, 3])
    assert np.isnan(flat["observation.imu"][7:]).all()  # no acceleration reported
    assert "observation.base" not in flat  # no base_position on a real robot
    assert len(observation_features(emb)["observation.base"]) == 6


def test_wrong_length_raises():
    with pytest.raises(ValueError, match="joint_pos"):
        flatten_observation(Observation(joint_pos=np.zeros(5)), _arm())


def test_position_action_with_gripper():
    emb = _arm()
    action = Action(joint_target=np.array([0.1, 0.2, 0.3]), gripper={"tcp": 1.0})
    feats, flat = action_features(emb, action=action), flatten_action(action, emb)
    _check(feats, flat)
    assert feats == action_features(emb, ControlMode.POSITION)
    assert feats["action"] == ["shoulder.pos", "elbow.pos", "wrist.pos", "tcp.gripper"]
    assert np.allclose(flat["action"], [0.1, 0.2, 0.3, 1.0])


def test_velocity_action_suffix():
    emb = _arm()
    flat = flatten_action(Action(mode=ControlMode.VELOCITY, joint_target=np.ones(3)), emb)
    assert action_features(emb, ControlMode.VELOCITY)["action"][0] == "shoulder.vel"
    assert np.isnan(flat["action"][-1])


def test_cartesian_action():
    emb = _arm()
    action = Action(mode=ControlMode.CARTESIAN, ee_targets={"tcp": np.arange(7.0)},
                    gripper={"tcp": 0.0})
    feats, flat = action_features(emb, action=action), flatten_action(action, emb)
    _check(feats, flat)
    assert set(flat) == {"action.ee", "action.gripper"}
    assert np.allclose(flat["action.ee"], np.arange(7.0))
    assert set(action_features(emb, ControlMode.CARTESIAN)) == {"action.ee", "action.gripper"}


def test_base_velocity_and_named_joints_beside_it():
    emb = _humanoid()
    action = Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.5, 0, 0.1]),
                    joint_names=["knee"], joint_target=np.array([0.7]))
    feats, flat = action_features(emb, action=action), flatten_action(action, emb)
    _check(feats, flat)
    assert feats["action.base_velocity"] == ["base.vx", "base.vy", "base.wz"]
    assert feats["action"] == ["hip.pos", "knee.pos"]
    assert np.isnan(flat["action"][0]) and flat["action"][1] == pytest.approx(0.7)
    assert set(action_features(emb, ControlMode.BASE_VELOCITY)) == {"action.base_velocity"}


_ARM = {"device": "body", "control_hz": 100, "joints": [{"name": f"j{i}"} for i in range(2)],
        "frames": [{"name": "tcp"}], "grippers": ["tcp"], "driver": {"type": "toy"}}


def _pair():
    arm = {"assembly": {"parts": {"body": _ARM}}}
    return {"assembly": {"parts": {"left": arm, "right": arm}}}


def test_assembly_names_are_prefixed():
    emb = load_embodiment(_pair())
    feats = observation_features(emb)
    assert feats["observation.state"] == ["left/j0.pos", "left/j1.pos", "right/j0.pos",
                                          "right/j1.pos", "left/tcp.gripper", "right/tcp.gripper"]
    robot = make_robot(_pair())
    robot.connect()
    robot.reset()
    obs = robot.get_observation()
    _check(observation_features(emb, obs), flatten_observation(obs, emb))


def test_parts_action_is_flattened_in_robot_names():
    emb = load_embodiment(_pair())
    action = Action(parts={"right": Action(joint_target=np.array([0.1, 0.2]),
                                           gripper={"tcp": 0.4})})
    feats, flat = action_features(emb, action=action), flatten_action(action, emb)
    _check(feats, flat)
    assert feats["action"][:4] == ["left/j0.pos", "left/j1.pos", "right/j0.pos", "right/j1.pos"]
    values = dict(zip(feats["action"], flat["action"]))
    assert values["right/j1.pos"] == pytest.approx(0.2)
    assert values["right/tcp.gripper"] == pytest.approx(0.4)
    assert np.isnan(values["left/j0.pos"]) and np.isnan(values["left/tcp.gripper"])


def test_parts_with_mixed_modes_raise():
    emb = load_embodiment(_pair())
    action = Action(parts={"left": Action(joint_target=np.zeros(2)),
                           "right": Action(mode=ControlMode.VELOCITY, joint_target=np.zeros(2))})
    with pytest.raises(ValueError, match="mixes"):
        flatten_action(action, emb)


def test_image_features_list_cameras():
    assert image_features(_arm()) == {"observation.images.wrist_cam": "wrist_cam"}
