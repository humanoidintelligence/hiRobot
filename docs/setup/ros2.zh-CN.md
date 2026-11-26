# ROS 2 机器人（通用 `ros2` 驱动）

[English](./ros2.md) | 简体中文

驱动：`ros2`（`src/hirobot/drivers/body/generic/ros2/`）。

已经用 ROS 2 通信的机器人，只写配置就能接：列出它发布状态的话题和接收命令的话题，`ros2` 驱动负责订阅和发布。
标准消息只填话题。厂商自己的命令消息用字段映射填，也不用写代码。[AgiBot X1](agibot_x1.zh-CN.md) 就是这样一份配置。

写好配置不等于不用核对机器人。每台机器人第一次运动前，都要核对关节名、单位、零位和方向、命令消息各字段的含义
（位置、PD 还是力矩；增益字段是什么），以及机器人上有没有别的程序也在发命令。

## 1. 前置条件

- 运行 hiRobot 的 shell 里 source 了 ROS 2（测过 Humble）：`source /opt/ros/humble/setup.bash`。
  rclpy 随 ROS 2 安装，pip 上没有。[RoboStack](https://robostack.github.io/) 可以把 ROS 2 装进 conda 环境。
- 自定义消息所在的 workspace 也编译好并 source（`source <ws>/install/setup.bash`）。
- 机器人和本机的 `ROS_DOMAIN_ID` 相同。
- 在本机上 `ros2 topic list` / `ros2 topic echo /joint_states` 能看到机器人的话题。看不到的话，hiRobot 也看不到。

## 2. 上电与安全

- `connect()` 和 `reset()` 不发任何命令；模式、回零、使能由机器人侧负责。
- 关节按**名字**匹配。机器人的 `JointState` 缺关节时，`connect()` 失败并列出缺哪些。
- 机器人收 POSITION 时，`hold()` 发当前关节位置。
- 第一次连接用 `read_only: true`（或 `hirobot view`）：不建发布者。

## 3. 配置

本体文件的 `driver:` 段：

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

### 状态组件

| `type` | 消息 | 填什么 |
|---|---|---|
| `joint_state` | `sensor_msgs/msg/JointState` | `joint_pos`、`joint_vel`、`joint_torque`（按 `name` 对；`velocity` / `effort` 为空时是 `None`）。`joints:` 让它只管部分关节，用于发好几个 JointState 话题的机器人；每个关节恰好一个来源 |
| `imu` | `sensor_msgs/msg/Imu` | `base_orientation`（`[x, y, z, w]`）、`base_angular_velocity`、`base_linear_acceleration` |
| `odometry` | `nav_msgs/msg/Odometry` | `base_position`、`base_linear_velocity` |

### 命令组件

| `type` | 消息 | 模式 |
|---|---|---|
| `joint_trajectory` | `trajectory_msgs/msg/JointTrajectory`，一个点；`time_from_start_s`（默认 0） | `position` |
| `float_array` | `std_msgs/msg/Float64MultiArray`；`mode:` 是 `position` / `velocity` / `torque` 之一；`joints:` 是数组顺序（默认 embodiment 的顺序） | 就是那一个 |
| `message` | 任意，`msg: pkg/msg/Name`，按 `fields:` 填 | `modes:`，从 `position` / `velocity` / `torque` / `pd` 中选 |

每个模式只能有一个组件。驱动的 `action_modes` 就是各命令组件覆盖的模式。

ros2_control 的 `joint_trajectory_controller` 用 `joint_trajectory`；`forward_command_controller` 用 `float_array`，
顺序按它自己的 `joints:` 参数，这里列一样的顺序。

### 字段映射（`type: message`）

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

- 键是消息字段，点号写嵌套字段（`header.frame_id`）。
- 值是来源（`$joint_names`、`$joint_target`、`$kp`、`$kd`、`$stamp`）或常量。不支持表达式，也不做单位换算。
- 没列出的字段保持消息默认值。
- `$kp` / `$kd` 先取 action 里的，没有就取 `set_gains` 设的，再没有就取 `default_kp` / `default_kd`。命令需要增益却哪里都没有时报错。

配置里的错误（来源名不认识、某个模式没有组件能接、没有字段映射到 `$joint_target`）在构造驱动时就报错，不用等连接。
消息里没有的字段在 `connect()` 时报错，同时列出消息有哪些字段。

### QoS

状态话题按 `best_effort`、depth 1 订阅；命令按 `reliable`、depth 10 发布。best_effort 订阅者收得到 reliable 发布者的消息，
反过来收不到。每个组件可以单独改：

```yaml
    - {type: imu, topic: /imu/data, qos: {reliability: reliable, depth: 5}}
```

## 4. 最小循环

先用 `read_only: true` 读一次。再关掉 `read_only`，把当前位置发回去：

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

同一进程里的所有 ROS 2 驱动共用一个 rclpy context 和一个 executor 线程。应用已经调过 `rclpy.init()` 时，驱动直接用它，
退出时也不关它。

## 5. 常见问题

- **`connect` 报找不到 rclpy**：这个 shell 没有 source ROS 2。
- **报找不到某个消息包**：它的 workspace 没编译或没 source。
- **超时并写出某个话题**：那个话题上没有消息到达。检查 `ros2 topic echo <topic>`、`ROS_DOMAIN_ID`，以及 QoS（`reliable` 订阅收不到
  `best_effort` 发布者的消息）。
- **`JointState` 缺关节**：机器人的关节名和 embodiment 的不一样。用 `ros_names` 改，或者改关节名。
- **命令发出去了但机器人不动**：机器人的控制器没激活、在听别的话题，或者有别的节点也在给它发命令。
