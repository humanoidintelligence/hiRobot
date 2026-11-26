# ROS 2 robots (generic `ros2` driver)

English | [简体中文](./ros2.zh-CN.md)

Driver: `ros2` (`src/hirobot/drivers/body/generic/ros2/`).

A robot that already talks ROS 2 can be driven from a config alone: list the
topics it publishes state on and the topic it takes commands on, and the `ros2`
driver subscribes and publishes. Standard messages need only a topic. A
vendor's own command message is filled from a field map, so it needs no code
either. [AgiBot X1](agibot_x1.md) is one such config.

A config does not replace checking the robot. Before the first move on each
robot, check the joint names, units, zero positions and directions, what the
command message's fields mean (position, PD or torque; what the gain fields
are), and whether something else on the robot also publishes commands.

## 1. Prerequisites

- ROS 2 (tested with Humble) sourced in the shell that runs hiRobot:
  `source /opt/ros/humble/setup.bash`. rclpy comes with it; it is not on pip.
  [RoboStack](https://robostack.github.io/) installs ROS 2 into a conda env.
- The workspace that builds any custom message, sourced too
  (`source <ws>/install/setup.bash`).
- The robot and this computer on the same `ROS_DOMAIN_ID`.
- `ros2 topic list` / `ros2 topic echo /joint_states` show the robot's topics
  from this computer. If they do not, hiRobot will not see them either.

## 2. Power-on and safety

- The driver commands nothing on `connect()` or `reset()`; the robot side owns
  its modes, homing and enabling.
- Joints are matched by **name**. `connect()` fails and lists the joints the
  robot's `JointState` lacks.
- `hold()` sends the current joint positions when the robot takes POSITION.
- Use `read_only: true` (or `hirobot view`) for the first connection: no
  publisher is created.

## 3. Configuration

The `driver:` section of a body file:

```yaml
driver:
  type: ros2
  node_name: hirobot_my_arm        # default hirobot_ros2
  ros_names: {}                    # embodiment joint name -> ROS name, only where they differ
  connect_timeout_s: 5.0           # wait for the first message on each state topic
  read_only: false
  state:
    - {type: joint_state, topic: /joint_states}
    - {type: imu, topic: /imu/data}                      # optional
  command:
    - {type: joint_trajectory, topic: /joint_trajectory_controller/joint_trajectory}
```

### State components

| `type` | Message | Fills |
|---|---|---|
| `joint_state` | `sensor_msgs/msg/JointState` | `joint_pos`, `joint_vel`, `joint_torque` (by `name`; empty `velocity` / `effort` give `None`). `joints:` limits it to some joints, for robots that publish several JointState topics; each joint needs exactly one |
| `imu` | `sensor_msgs/msg/Imu` | `base_orientation` (`[x, y, z, w]`), `base_angular_velocity`, `base_linear_acceleration` |
| `odometry` | `nav_msgs/msg/Odometry` | `base_position`, `base_linear_velocity` |

### Command components

| `type` | Message | Modes |
|---|---|---|
| `joint_trajectory` | `trajectory_msgs/msg/JointTrajectory`, one point; `time_from_start_s` (default 0) | `position` |
| `float_array` | `std_msgs/msg/Float64MultiArray`; `mode:` one of `position` / `velocity` / `torque`; `joints:` the array order (default the embodiment's) | that one |
| `message` | any, `msg: pkg/msg/Name`, filled from `fields:` | `modes:`, from `position` / `velocity` / `torque` / `pd` |

Each mode can have one component. The driver's `action_modes` are the modes
its command components cover.

ros2_control's `joint_trajectory_controller` takes `joint_trajectory`;
`forward_command_controller` takes `float_array`, in the order of its own
`joints:` parameter, so list the same order here.

### Field maps (`type: message`)

```yaml
command:
  - type: message
    topic: /motor_cmd
    msg: vendor_msgs/msg/MotorCommand
    modes: [position, pd]
    fields:
      joint: $joint_names          # ROS names, after ros_names
      q: $joint_target             # position / velocity / torque, by mode
      kp: $kp                      # per joint
      kd: $kd
      header.stamp: $stamp         # the node clock's time
      mode: 10                     # a constant
default_kp: 50.0                   # $kp / $kd when the action carries none
default_kd: 1.0
```

- Keys are message fields; dots reach nested ones (`header.frame_id`).
- Values are a source (`$joint_names`, `$joint_target`, `$kp`, `$kd`,
  `$stamp`) or a constant. There are no expressions or unit conversions.
- Fields not listed keep the message's defaults.
- `$kp` / `$kd` come from the action, else from `set_gains`, else from
  `default_kp` / `default_kd`. A command that needs a gain but has none fails.

Errors in the config (an unknown source, a mode no component can take, a
missing `$joint_target`) fail when the driver is built, before anything
connects. A field the message does not have fails at `connect()`, with the
list of fields it does have.

### QoS

State topics are subscribed `best_effort`, depth 1; commands are published
`reliable`, depth 10. A best-effort subscriber receives from a reliable
publisher, but not the other way round. Override per component:

```yaml
    - {type: imu, topic: /imu/data, qos: {reliability: reliable, depth: 5}}
```

## 4. Minimal loop

Read once with `read_only: true` first. Then, with `read_only` off, send the
current position back:

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_arm.yaml")
robot.connect()
obs = robot.get_observation()
print(obs.joint_pos)
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

All ROS 2 drivers in one process share one rclpy context and one executor
thread. If the application has called `rclpy.init()` already, the driver uses
it and leaves it running.

## 5. Common pitfalls

- **`connect` raises ImportError for rclpy:** ROS 2 is not sourced in this shell.
- **ImportError for a message package:** its workspace is not built or not sourced.
- **Timeout naming a topic:** nothing arrived on it. Check `ros2 topic echo`,
  the `ROS_DOMAIN_ID`, and the QoS (a `reliable` subscription gets nothing from
  a `best_effort` publisher).
- **"lacks joints":** the robot's `JointState` names differ from the
  embodiment's. Rename with `ros_names` or fix the joint names.
- **Commands are published but nothing moves:** the robot's controller is not
  active, listens on another topic, or another node is also commanding it.
