"""``ros2`` passes the body contract against a fake ROS 2, and maps topics by its config."""

import numpy as np
import pytest

from hirobot import Action, ControlMode, Embodiment, get_body, load_embodiment, make_robot
from hirobot.drivers._sdk import ros2 as ros_sdk
from tests.drivers.body.generic.ros2 import fake_sdk
from tests.drivers.contract import check_robot

NAMES = ["j0", "j1", "j2"]
# The robot publishes its joints in another order, one of them under another name.
ROS_ORDER = ["joint_2", "j0", "j1"]

STATE = [{"type": "joint_state", "topic": "/joint_states"}, {"type": "imu", "topic": "/imu/data"}]
CUSTOM = {"type": "message", "topic": "/motor_cmd", "msg": "vendor_msgs/msg/MotorCommand",
          "modes": ["position", "pd"],
          "fields": {"joint": "$joint_names", "q": "$joint_target", "kp": "$kp", "kd": "$kd",
                     "mode": 10, "header.stamp": "$stamp"}}


def _emb(floating=True):
    return Embodiment(name="r", joints=[{"name": n} for n in NAMES], floating_base=floating)


def _setup(monkeypatch, *, publish=True):
    ros = fake_sdk.install(monkeypatch)
    if publish:
        ros.on_subscribe("/joint_states", lambda: ros.joint_state(ROS_ORDER, [2.0, 0.0, 1.0],
                                                                    [0.2, 0.0, 0.1], [0, 0, 0]))
        ros.on_subscribe("/imu/data", lambda: ros.imu((0.0, 0.0, 0.3827, 0.9239)))
    return ros


def _robot(command, **kw):
    return get_body("ros2")(_emb(), state=STATE, command=command,
                            ros_names={"j2": "joint_2"}, connect_timeout_s=0.2, **kw)


@pytest.fixture(autouse=True)
def _no_leftover_context():
    yield
    assert ros_sdk.users() == 0, "a test left the shared rclpy context held"


def test_contract(monkeypatch):
    ros = _setup(monkeypatch)
    check_robot("ros2", lambda: _robot([CUSTOM], default_kp=50.0, default_kd=1.0),
                lambda: len(ros.published))
    assert not ros.ok and all(n.destroyed for n in ros.nodes)


def test_contract_joint_trajectory(monkeypatch):
    ros = _setup(monkeypatch)
    check_robot("ros2", lambda: _robot([{"type": "joint_trajectory", "topic": "/jtc/joint_trajectory"}]),
                lambda: len(ros.published))


def test_observation_is_by_name(monkeypatch):
    _setup(monkeypatch)
    robot = _robot([CUSTOM], default_kp=50.0, default_kd=1.0)
    robot.connect()
    obs = robot.get_observation()
    assert np.allclose(obs.joint_pos, [0.0, 1.0, 2.0]) and np.allclose(obs.joint_vel, [0, 0.1, 0.2])
    assert np.allclose(obs.base_orientation, [0.0, 0.0, 0.3827, 0.9239])
    assert np.allclose(obs.base_linear_acceleration, [0, 0, 9.81])
    assert obs.stamps["body"].seq == 1
    robot.disconnect()


def test_empty_velocity_and_effort_stay_none(monkeypatch):
    ros = fake_sdk.install(monkeypatch)
    ros.on_subscribe("/joint_states", lambda: ros.joint_state(NAMES, [0.0, 0.0, 0.0]))
    robot = get_body("ros2")(_emb(False), state=STATE[:1], connect_timeout_s=0.2)
    robot.connect()
    obs = robot.get_observation()
    assert obs.joint_vel is None and obs.joint_torque is None and obs.base_orientation is None
    assert robot.action_modes == ()
    robot.disconnect()


def test_custom_message_fields(monkeypatch):
    ros = _setup(monkeypatch)
    robot = _robot([CUSTOM], default_kp=50.0, default_kd=1.0)
    robot.connect()
    robot.send_action(Action(mode=ControlMode.PD, joint_target=np.array([0.1, 0.2, 0.3]),
                             kp=np.array([10.0, 20.0, 30.0])))
    topic, msg = ros.published[-1]
    assert topic == "/motor_cmd" and msg.joint == ["j0", "j1", "joint_2"]
    assert msg.q == [0.1, 0.2, 0.3] and msg.kp == [10.0, 20.0, 30.0] and msg.kd == [1.0] * 3
    assert msg.mode == 10 and msg.header.stamp.sec == 12
    robot.set_gains(np.array(7.0), None)
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(3)))
    assert ros.published[-1][1].kp == [7.0] * 3
    robot.disconnect()


def test_gains_needed_but_missing(monkeypatch):
    _setup(monkeypatch)
    pd_only = dict(CUSTOM, modes=["pd"])
    robot = _robot([pd_only])
    robot.connect()
    with pytest.raises(ValueError, match="needs kp"):
        robot.send_action(Action(mode=ControlMode.PD, joint_target=np.zeros(3)))
    robot.disconnect()


def test_joint_trajectory_message(monkeypatch):
    ros = _setup(monkeypatch)
    robot = _robot([{"type": "joint_trajectory", "topic": "/jtc", "time_from_start_s": 0.25}])
    robot.connect()
    assert robot.action_modes == (ControlMode.POSITION,)
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=np.array([0.1, 0.2, 0.3])))
    msg = ros.published[-1][1]
    assert msg.joint_names == ["j0", "j1", "joint_2"] and len(msg.points) == 1
    assert msg.points[0].positions == [0.1, 0.2, 0.3]
    assert (msg.points[0].time_from_start.sec, msg.points[0].time_from_start.nanosec) == (0, 250000000)
    with pytest.raises(ValueError, match="no command component"):
        robot.send_action(Action(mode=ControlMode.VELOCITY, joint_target=np.zeros(3)))
    robot.disconnect()


def test_float_array_in_its_own_order(monkeypatch):
    ros = _setup(monkeypatch)
    robot = _robot([{"type": "float_array", "topic": "/fwd/commands", "mode": "velocity",
                     "joints": ["j2", "j0", "j1"]}])
    robot.connect()
    assert robot.action_modes == (ControlMode.VELOCITY,)
    robot.send_action(Action(mode=ControlMode.VELOCITY, joint_target=np.array([0.1, 0.2, 0.3])))
    assert ros.published[-1][1].data == [0.3, 0.1, 0.2]
    robot.disconnect()


def test_two_joint_state_topics_and_odometry(monkeypatch):
    ros = fake_sdk.install(monkeypatch)
    ros.on_subscribe("/arm/joint_states", lambda: ros.joint_state(["j0", "j1"], [1.0, 2.0],
                                                                  topic="/arm/joint_states"))
    ros.on_subscribe("/grip/joint_states", lambda: ros.joint_state(["j2"], [3.0],
                                                                   topic="/grip/joint_states"))

    def odom():
        m = fake_sdk.Odometry()
        m.pose.pose.position.x, m.twist.twist.linear.y = 4.0, 0.5
        ros.publish_state("/odom", m)

    ros.on_subscribe("/odom", odom)
    robot = get_body("ros2")(_emb(), connect_timeout_s=0.2, state=[
        {"type": "joint_state", "topic": "/arm/joint_states", "joints": ["j0", "j1"]},
        {"type": "joint_state", "topic": "/grip/joint_states", "joints": ["j2"]},
        {"type": "odometry", "topic": "/odom"}])
    robot.connect()
    obs = robot.get_observation()
    assert np.allclose(obs.joint_pos, [1, 2, 3])
    assert np.allclose(obs.base_position, [4, 0, 0]) and np.allclose(obs.base_linear_velocity, [0, 0.5, 0])
    robot.disconnect()


def test_qos_defaults_and_override(monkeypatch):
    ros = _setup(monkeypatch)
    state = [STATE[0], dict(STATE[1], qos={"reliability": "reliable", "depth": 5})]
    robot = get_body("ros2")(_emb(), state=state, ros_names={"j2": "joint_2"},
                             connect_timeout_s=0.2)
    robot.connect()
    assert (ros.sub_qos["/joint_states"].reliability, ros.sub_qos["/joint_states"].depth) == (
        "best_effort", 1)
    assert (ros.sub_qos["/imu/data"].reliability, ros.sub_qos["/imu/data"].depth) == ("reliable", 5)
    robot.disconnect()


def test_connect_times_out_and_names_the_topic(monkeypatch):
    _setup(monkeypatch, publish=False)
    with pytest.raises(TimeoutError, match="/joint_states"):
        _robot([CUSTOM], default_kp=1.0, default_kd=1.0).connect()


def test_connect_reports_missing_joints(monkeypatch):
    ros = fake_sdk.install(monkeypatch)
    ros.on_subscribe("/joint_states", lambda: ros.joint_state(["j0", "j1"], [0.0, 0.0]))
    robot = get_body("ros2")(_emb(False), state=STATE[:1], connect_timeout_s=0.1)
    with pytest.raises(RuntimeError, match=r"lacks joints \['j2'\]"):
        robot.connect()


def test_unknown_message_field_fails_on_connect(monkeypatch):
    _setup(monkeypatch)
    bad = dict(CUSTOM, fields=dict(CUSTOM["fields"], stiffness="$kp"))
    with pytest.raises(ValueError, match=r"no fields \['stiffness'\]"):
        _robot([bad], default_kp=1.0, default_kd=1.0).connect()
    bad = dict(CUSTOM, fields=dict(CUSTOM["fields"], **{"header.frame": "x"}))
    with pytest.raises(ValueError, match="header.frame"):
        _robot([bad], default_kp=1.0, default_kd=1.0).connect()


def test_missing_message_package(monkeypatch):
    _setup(monkeypatch)
    bad = dict(CUSTOM, msg="no_such_pkg/msg/Cmd")
    with pytest.raises(ImportError, match="no_such_pkg.*source"):
        _robot([bad], default_kp=1.0, default_kd=1.0).connect()


@pytest.mark.parametrize("config, match", [
    ({"state": []}, "needs a joint_state"),
    ({"state": [{"type": "jointstate", "topic": "/j"}]}, r"state\[0\].type is 'jointstate'"),
    ({"state": STATE, "command": [dict(CUSTOM, fields={"q": "$target"})]}, "unknown source"),
    ({"state": STATE, "command": [dict(CUSTOM, fields={"kp": "$kp"})]}, "must map some field to \\$joint_target"),
    ({"state": STATE, "command": [dict(CUSTOM, modes=["position"])]}, "no 'pd'"),
    ({"state": STATE, "command": [dict(CUSTOM, modes=["base_velocity"])]}, "not supported"),
    ({"state": STATE, "command": [dict(CUSTOM, msg="MotorCommand")]}, "pkg/msg/Name"),
    ({"state": STATE, "command": [CUSTOM, {"type": "joint_trajectory", "topic": "/t"}],
      "default_kp": 1.0, "default_kd": 1.0}, "mode 'position' is in command"),
    ({"state": STATE, "command": [CUSTOM]}, "set default_kp"),
    ({"state": STATE, "topcs": 1}, "topcs"),
    ({"state": STATE, "ros_names": {"j9": "x"}}, "ros_names has joints not in"),
    ({"state": [{"type": "joint_state", "topic": "/a", "joints": ["j0"]}]}, "in no joint_state"),
    ({"state": [STATE[0], dict(STATE[0], topic="/b")]}, "more than one joint_state"),
    ({"state": STATE, "command": [{"type": "float_array", "topic": "/f", "mode": "position",
                                   "joints": ["j0", "j1"]}]}, "every joint"),
])
def test_bad_config_fails_when_built(config, match):
    # No fake ROS: these are caught before anything imports rclpy.
    with pytest.raises(ValueError, match=match):
        get_body("ros2")(_emb(), **config)


def test_read_only(monkeypatch):
    ros = _setup(monkeypatch)
    robot = _robot([CUSTOM], default_kp=1.0, default_kd=1.0, read_only=True)
    assert robot.read_only and robot.action_modes == ()
    robot.connect()
    with pytest.raises(RuntimeError, match="read_only"):
        robot._send_action(Action(mode=ControlMode.POSITION, joint_target=np.zeros(3)))
    assert ros.published == []
    robot.disconnect()


def test_two_drivers_share_one_context(monkeypatch):
    ros = _setup(monkeypatch)
    a, b = _robot([], node_name="a"), _robot([], node_name="b")
    a.connect()
    b.connect()
    assert ros.inits == 1 and ros_sdk.users() == 2 and len(ros.executor_nodes) == 2
    a.disconnect()
    assert ros.ok and ros.nodes[0].destroyed and not ros.nodes[1].destroyed
    b.disconnect()
    assert not ros.ok and ros_sdk.users() == 0


def test_context_started_by_the_application_is_left_running(monkeypatch):
    ros = _setup(monkeypatch)
    ros.ok = True  # the application called rclpy.init() itself
    robot = _robot([])
    robot.connect()
    robot.disconnect()
    assert ros.ok and ros.inits == 0


def test_make_robot_from_a_config(monkeypatch):
    _setup(monkeypatch)
    config = {"device": "body", "joints": [{"name": n} for n in NAMES], "floating_base": True,
              "driver": {"type": "ros2", "state": STATE, "ros_names": {"j2": "joint_2"},
                         "connect_timeout_s": 0.2,
                         "command": [{"type": "joint_trajectory", "topic": "/jtc"}]}}
    robot = make_robot(config)
    robot.connect()
    assert np.allclose(robot.get_observation().joint_pos, [0.0, 1.0, 2.0])
    robot.disconnect()


def test_agibot_x1_config(monkeypatch):
    # The shipped X1 body file: the ros2 driver with its JointCommand field map.
    ros = fake_sdk.install(monkeypatch)
    emb = load_embodiment("agibot_x1")
    names = emb.joint_names
    ros.on_subscribe("/joint_states", lambda: ros.joint_state(names[::-1], range(len(names))[::-1]))
    ros.on_subscribe("/imu/data", lambda: ros.imu())
    robot = make_robot("body/agibot/x1")
    robot.connect()
    try:
        assert set(robot.action_modes) == {ControlMode.POSITION, ControlMode.PD}
        obs = robot.get_observation()
        assert np.allclose(obs.joint_pos, np.arange(len(names)))  # by name, not message order
        robot.send_action(Action(mode=ControlMode.PD, joint_target=np.full(len(names), 0.1)))
        topic, cmd = ros.published[-1]
        assert topic == "/joint_cmd" and cmd.name == names
        assert cmd.stiffness == [100.0] * len(names) and cmd.damping == [2.0] * len(names)
        assert cmd.position == [0.1] * len(names) and cmd.velocity == [] and cmd.effort == []
    finally:
        robot.disconnect()


def test_hirobot_view_opens_it_read_only(monkeypatch):
    from hirobot.sim.view import open_read_only

    ros = _setup(monkeypatch)
    config = {"device": "body", "joints": [{"name": n} for n in NAMES],
              "driver": {"type": "ros2", "state": STATE[:1], "ros_names": {"j2": "joint_2"},
                         "command": [{"type": "joint_trajectory", "topic": "/jtc"}]}}
    robot = open_read_only(config)
    robot.connect()
    assert robot.action_modes == () and robot.get_observation().joint_pos is not None
    robot.disconnect()
    assert ros.published == []
