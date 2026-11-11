"""make_robot on assembly: configs (hirobot.loading.build): one robot per node, one gate at the root."""

import numpy as np
import pytest
import yaml

from hirobot import Action, ControlMode, SafetyError, make_robot
from hirobot.assembly import RobotAssembly

ARM = {"device": "body", "control_hz": 100,
       "joints": [{"name": f"j{i}", "position_limit": [-3, 3]} for i in range(3)],
       "frames": [{"name": "tcp", "link": "l3"}], "mounts": {"flange": {"link": "l3"}},
       "driver": {"type": "toy"}}
CAM = {"device": "camera", "width": 32, "height": 24, "driver": {"type": "array"}}
BASE = {"device": "body", "control_hz": 50, "floating_base": True, "joints": [],
        "mounts": {"arm_mount": {"link": "base_link", "pose": [0.2, 0, 0.45, 0, 0, 0, 1]}},
        "driver": {"type": "toy", "mobile_base": True}}


def _files(tmp_path):
    (tmp_path / "arm.yaml").write_text(yaml.safe_dump(ARM))
    (tmp_path / "cam.yaml").write_text(yaml.safe_dump(CAM))
    (tmp_path / "single.yaml").write_text(yaml.safe_dump({
        "assembly": {"name": "single", "parts": {"body": {"from": "./arm.yaml"},
                                                 "wrist": {"from": "./cam.yaml"}},
                     "placement": {"wrist": {"on": "body.flange"}}},
        "safety": {"joint_limit": 2.0}}))
    (tmp_path / "dual.yaml").write_text(yaml.safe_dump({
        "assembly": {"name": "dual",
                     "parts": {"left": {"from": "./single.yaml"},
                               "right": {"from": "./single.yaml", "safety": {"joint_limit": 1.0}},
                               "scene": {"from": "./cam.yaml"}},
                     "placement": {"left": {"pose": [0, 0.4, 0, 0, 0, 0, 1]},
                                   "right": {"pose": [0, -0.4, 0, 0, 0, 0, 1]}}},
        "safety": {"max_joint_step": 0.5}}))
    return tmp_path


def test_single_node_is_one_stack(tmp_path):
    robot = make_robot(str(_files(tmp_path) / "single.yaml"))
    assert isinstance(robot, RobotAssembly) and robot._node is None  # body + camera
    assert robot.embodiment.joint_names == ["j0", "j1", "j2"]
    assert robot.safety.joint_limit == 2.0
    robot.connect()
    assert "wrist" in robot.get_observation().images


def test_device_file_alone_is_a_robot(tmp_path):
    robot = make_robot(str(_files(tmp_path) / "arm.yaml"))
    assert robot.embodiment.joint_names == ["j0", "j1", "j2"]
    assert not isinstance(robot, RobotAssembly)


def test_dual_names_devices_and_per_part_safety(tmp_path):
    robot = make_robot(str(_files(tmp_path) / "dual.yaml"))
    emb = robot.embodiment
    assert emb.joint_names == [f"{s}/j{i}" for s in ("left", "right") for i in range(3)]
    assert [c.name for c in emb.cameras] == ["left/wrist", "right/wrist", "scene"]
    assert set(emb.parts) == {"", "left", "right"}
    # The referenced config's own joint_limit (2.0) is a default; the part entry's wins.
    assert np.allclose(robot.safety.joint_limit, [2, 2, 2, 1, 1, 1])
    assert robot.safety.max_joint_step == 0.5
    assert np.allclose(robot._node.fixed["left"], [0, 0.4, 0, 0, 0, 0, 1])
    robot.connect()
    robot.reset()
    assert set(robot.get_observation().images) == {"left/wrist", "right/wrist", "scene"}
    robot.send_action(Action(parts={"left": Action(mode=ControlMode.POSITION,
                                                   joint_target=np.array([0.3, 0, 0]))}))
    with pytest.raises(SafetyError, match="'right/j2'"):
        robot.send_action(Action(parts={"right": Action(mode=ControlMode.POSITION,
                                                        joint_target=np.array([0, 0, 1.5]))}))
    with pytest.raises(SafetyError, match="max_joint_step"):
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.array([0.3, 0, 0, 0, 0, 0.9])))


def test_overrides_by_part_path(tmp_path):
    robot = make_robot(str(_files(tmp_path) / "dual.yaml"),
                       overrides={"left/body": {"driver": {"dt": 0.25}}})
    assert robot._node.children["left"].body._dt == 0.25
    assert robot._node.children["right"].body._dt != 0.25
    with pytest.raises(ValueError, match="one body"), pytest.warns(DeprecationWarning):
        make_robot(str(tmp_path / "dual.yaml"), body_overrides={"dt": 0.1})
    with pytest.raises(ValueError, match="does not have"):
        make_robot("unitree_g1", overrides={"nope": {}})


def test_mobile_base_with_an_arm_on_it(tmp_path):
    cfg = {"assembly": {"name": "mobile",
                        "parts": {"base": BASE, "arm": {"assembly": {"parts": {"body": ARM}}}},
                        "placement": {"arm": {"on": "base.arm_mount"}}}}
    robot = make_robot(cfg)
    assert robot.embodiment.floating_base
    assert robot.embodiment.joint_names == ["arm/j0", "arm/j1", "arm/j2"]
    assert np.allclose(robot._node.fixed["arm"], [0.2, 0, 0.45, 0, 0, 0, 1])
    robot.connect()
    robot.reset()
    robot.send_action(Action(mode=ControlMode.BASE_VELOCITY, base_velocity=np.array([0.5, 0, 0])))
    assert robot.get_observation().base_position[0] > 0  # the base's odometry stays top-level
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2, 0.3])))
    assert np.allclose(robot.get_observation().joint_pos, [0.1, 0.2, 0.3])


def test_child_robot_on_a_moving_link_is_refused():
    cfg = {"assembly": {"parts": {"body": ARM, "tool": {"assembly": {"parts": {"body": ARM}}}},
                        "placement": {"tool": {"on": "body.flange"}}}}
    with pytest.raises(ValueError, match="move with that part's joints"):
        make_robot(cfg)


def test_two_bodies_in_one_node_are_refused():
    with pytest.raises(ValueError, match="more than one body"):
        make_robot({"assembly": {"parts": {"a": ARM, "b": dict(
            ARM, joints=[{"name": "k0"}], frames=[], mounts={})}}})


def test_collision_models_by_child_body(tmp_path):
    cfg = {"assembly": {"parts": {"left": {"assembly": {"parts": {"body": ARM}}},
                                  "right": {"assembly": {"parts": {"body": ARM}}}},
                        "collision": {"min_distance": 0.05,
                                      "models": {"nope/body": {"mjcf": "x.xml"}}}}}
    with pytest.raises(ValueError, match="no child robot 'nope'"):
        make_robot(cfg)


def test_whole_robot_keys_must_agree():
    cfg = {"assembly": {"parts": {"left": {"assembly": {"parts": {"body": ARM}}, "safety": {"clamp": True}},
                                  "right": {"assembly": {"parts": {"body": ARM}},
                                            "safety": {"clamp": False}}}}}
    with pytest.raises(ValueError, match="one value for the whole robot"):
        make_robot(cfg)


def test_safety_false_and_layers_and_arbiter():
    robot = make_robot({"assembly": {"parts": {"body": ARM}}, "safety": False,
                        "arbiter": {"require_claim": True}})
    assert robot.safety is None and robot._arbiter.require_claim


def test_sim_needs_a_model_on_every_body_and_a_known_backend():
    with pytest.raises(ValueError, match="needs a model"):
        make_robot({"assembly": {"parts": {"body": ARM}}, "sim": {"backend": "mujoco"}})
    with pytest.raises(ValueError, match="only 'mujoco'"):
        make_robot({"assembly": {"parts": {"body": ARM}}, "sim": {"backend": "isaac"}})
    with pytest.raises(ValueError, match="unknown key"):
        make_robot({"assembly": {"parts": {"body": ARM}}, "sim": {"fixed": []}})
