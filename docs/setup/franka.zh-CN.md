# Franka 设置指南（FR3 / Panda）

[English](./franka.md) | 简体中文

驱动：`franka_libfranka`（`src/hirobot/drivers/body/franka/libfranka/`）。一台 7 自由度固定基座机械臂，
没有 IMU 和基座状态。它封装了 `pylibfranka`，即 [libfranka](https://github.com/frankarobotics/libfranka)
官方的 pybind 绑定。

libfranka 的流式控制器需要**在 PREEMPT_RT 内核上运行 1 kHz 实时循环**。为了让 hiRobot 的软实时循环
仍然可用，驱动使用**异步关节位置控制器**（`AsyncPositionControlHandler`）：每次 `send_action` 设置一个新的
关节目标，由 libfranka 自己的实时线程跟踪，所以 VLA policy 可以按自己较低的频率发出目标。

## 1. 前置条件

安装绑定（预编译 wheel；见 libfranka 仓库中的 `pylibfranka/README.md`）：

```bash
pip install pylibfranka
# unsupported platform: build from source per libfranka's pylibfranka/README.md (CMake + pybind11)
```

你需要与机器人固件匹配的 libfranka 版本，以及 PREEMPT_RT 内核（正式控制必需）。

## 2. 网络与机器人设置

- 把主机接到机器人在 FCI 子网上的 FCI 端口。记下 FCI 地址（通常是 `172.16.0.2`），设为 `body.ip`。
- 在 Desk（Franka 的网页界面）中：解锁关节，激活 **FCI**，并确保机器人**没有被 Desk 占用**。
- 状态灯必须显示机器人已准备好接受外部控制（蓝色）。

## 3. 上电与安全

1. 清空工作空间，急停（以及 Franka 的使能装置）放在手边。
2. `connect()` 会设置保守的碰撞阈值（`set_collision_behavior`）；按你的负载调整。
3. `reset()` 调用 `automatic_error_recovery()` 清除反射 / 故障，并**启动异步位置控制器**。所以**在硬件上，
   必须先调用 `reset()` 再调用 `send_action()`**，否则驱动会抛出错误，提示你先 reset。
4. 没有实时内核时，`realtime=False` 会放宽 `RealtimeConfig`。只用于设置检查，**绝不能用于正式控制**。

## 4. 配置

`franka_fr3`：

```yaml
embodiment:
  control_hz: 100           # rate the VLA sends targets; libfranka's RT thread tracks at 1 kHz
body:
  driver: franka_libfranka
  ip: 172.16.0.2            # FCI address
  # max_joint_velocity: 1.0 # per-joint velocity cap of the async controller (rad/s)
  # goal_tolerance: 0.01    # goal tolerance (rad)
  # realtime: true          # set false without an RT kernel (setup only)
```

关节顺序：`joint1..joint7`，与 libfranka 的 `RobotState.q` 一致。

## 5. 最小循环

用[快速开始](../../README.zh-CN.md#仿真)中的基本循环检查连接，在你的本地配置（内置配置加上面的
驱动和地址）上运行：读一次观测，把当前关节位置作为 POSITION 目标发回去，然后保持。

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_franka.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

驱动做了什么：`get_observation()` 从 `read_once()` 读取 `q/dq/tau_J`，并把 4x4 列主序的 `O_T_EE`
（基座坐标系）转换成 7 维位姿 `[x, y, z, qx, qy, qz, qw]`，存入 `ee_poses["ee"]`。
`send_action()`（POSITION / PD）调用 `handler.set_joint_position_target(...)`。

## 6. 常见问题

- **先 reset**：异步控制器在 `reset()` 中启动；不调用它就调用 `send_action` 会抛出错误。
- **实时内核**：`realtime=True`（默认）且没有实时内核时，libfranka 会抛出异常。设置阶段可以临时设为
  `realtime=False`。
- **只支持 POSITION / PD**：本驱动使用异步关节位置控制。笛卡尔控制和力矩控制没有封装（libfranka
  支持它们；可用 `start_cartesian_*` / 力矩控制器扩展）。
- **碰撞阈值**：默认值偏保守；重负载或快速运动很容易触发反射停止。按需在驱动中调整
  `set_collision_behavior`。
- **Desk 占用机器人**：如果 Desk 程序正在运行，或 FCI 没有激活，外部控制就连不上。
- **还没在真机上跑过**：驱动只对假的 `pylibfranka` 测过。没在硬件上核实的：`pylibfranka` 是否按驱动的调用方式提供
  `start_async_position_control` / `set_joint_position_target`；异步控制器运行时 `read_once()` 能不能用；`estop()` 停掉控制器后
  机械臂怎么做。另外 libfranka 会把构造它的线程（Python 主线程）提成实时优先级，从这个线程启动的线程会继承。
- **`reset()` 不动**：只清错并重启控制器，没有初始姿态。
