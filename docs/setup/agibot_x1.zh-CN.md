# AgiBot X1 设置指南（ROS2 / AimRT bridge）

[English](./agibot_x1.md) | 简体中文

驱动：`agibot_aimrt`（`src/hirobot/drivers/body/agibot/aimrt/`）。

**与其他厂商不同，AgiBot X1 没有 Python API。**
[agibot_x1_infer](https://github.com/AgibotTech/agibot_x1_infer)
（xyber_x1_infer）是一套纯 C++ / AimRT 的软件栈。从 hiRobot 驱动 X1 的可行办法，是用一个 ROS2 节点
通过 AimRT 的 `ros2` channel 后端与它通信。本驱动正是这样一个 **rclpy bridge**：

- 它订阅 `/joint_states`（`sensor_msgs/JointState`）和 `/imu/data`（`sensor_msgs/Imu`）作为观测。
- 它发布 `/joint_cmd`（`my_ros2_proto/JointCommand`：name / position / velocity / effort /
  **stiffness** / **damping**）作为指令，其中 `stiffness` / `damping` 就是 PD 增益 kp / kd。

## ⚠️ Bridge 前置条件（必须在机器人侧满足，否则驱动在 connect 时失败）

1. **启用 ros2 订阅后端**：在 AimRT 配置中，`sub_topics_options` 默认只有 `local`，所以外部的
   `/joint_cmd` 永远进不来。把它改成 `[local, ros2]`。（发布侧的 `pub_topics_options` 默认已包含
   `ros2`，所以状态能到达外部节点。）
2. **只运行底层驱动模块**：你运行的 AimRT 包必须暴露底层驱动（硬件上是 `DcuDriverModule`，仿真中是
   `SimModule`），并且**去掉自带的 `ControlModule`**。否则 `ControlModule`（内置的 RL 行走控制器）
   也会发布 `/joint_cmd`，和你争夺控制权。
3. **编译自定义消息**：`my_ros2_proto` 必须用 `colcon build` 编译并 source，这样 Python 才能
   `from my_ros2_proto.msg import JointCommand`。

这三条都满足后，驱动就能工作；观测 / 动作映射本身是完整的。

## 1. 前置条件

- 带 `rclpy` 和 `sensor_msgs` 的 ROS2（Humble）环境。
- 已编译并 source 的 `my_ros2_proto`（来自 `agibot_x1_infer`）。
- 机器人侧：用 `agibot_x1_infer` 的 `build.sh` 编译 AimRT 软件栈，按上面的 bridge 前置条件修改，然后用
  `run.sh`（硬件）或 `run_sim.sh`（仿真）启动。

## 2. 上电与安全

- 它是人形机器人：必须吊起或托住，急停随时可用。
- 模式切换、归零等由 AimRT 软件栈负责；驱动的 `reset()` 不做任何事。
- `send_action` 按**名字**（而非索引）匹配关节，所以配置中的关节名必须与 AimRT 软件栈一致。

## 3. 配置

`agibot_x1`：

```yaml
body:
  driver: agibot_aimrt
  # node_name: hirobot_agibot_x1
  # default_kp: 100.0        # fallback stiffness
  # default_kd: 2.0          # fallback damping
  # spin_timeout: 5.0        # seconds to wait for the first /joint_states
```

关节顺序沿用 `rl_x1.yaml` 中 ControlModule 的 29 关节 `joint_list`（腰部 3 + 双臂 14 + 双腿 12）。
如果直接驱动 DCU 层，它有 32 个关节（多出左右夹爪，且顺序不同）；相应调整配置。

## 4. 最小循环

用[快速开始](../../README.zh-CN.md#仿真)中的基本循环检查连接，在你的本地配置（内置配置加上面的
驱动和地址）上运行：读一次观测，把当前关节位置作为 POSITION 目标发回去，然后保持。

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_x1.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

驱动做了什么：`connect()` 启动一个 rclpy 节点，创建 publisher 和 subscribers，并 spin 到第一条
`/joint_states` 到达（超过 `spin_timeout` 仍未到达时，抛出一个指向本指南的错误）。
`get_observation()` 组装缓存的 JointState（position / velocity / effort）和 Imu。`send_action()`
（POSITION / PD）发布一条 JointCommand，其中 `stiffness/damping` 取自 kp/kd。

## 5. 常见问题

- **`connect` 抛出 ImportError**：缺少 rclpy / sensor_msgs，或 `my_ros2_proto` 没有编译或 source。
- **等待 /joint_states 超时**：AimRT 进程没有运行，或底层驱动模块没有发布状态。
- **指令发出了但机器人不动**：最可能是前置条件 1（订阅侧没有 ros2 后端）或前置条件 2（`ControlModule`
  仍在争夺 `/joint_cmd`）。
- **只有遥操作行走能用，关节目标不起作用**：你连接走的是官方 ControlModule 路径（`/cmd_vel` 加模式触发
  topics），它只能用内置 RL policy 行走，不是关节级的 VLA 控制。关节级控制需要绕过 ControlModule，
  直接与 DcuDriverModule / SimModule 通信。
- **关节名不匹配**：`/joint_cmd` 按名字匹配；配置中拼错的关节名会被忽略。
