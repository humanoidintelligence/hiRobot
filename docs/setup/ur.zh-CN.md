# Universal Robots 设置指南（UR5e / UR 系列）

[English](./ur.md) | 简体中文

驱动：`ur_rtde`（`src/hirobot/drivers/body/ur/rtde/`）。一台 6 自由度固定基座机械臂，没有 IMU 和基座状态。
它封装了 `ur_rtde`（`RTDEControlInterface` + `RTDEReceiveInterface`），用 `servoJ` 高频伺服关节，
用 `servoL` 伺服 TCP 笛卡尔位姿。

> 为什么用 `ur_rtde` 而不用 UR 官方的 `RTDE_Python_Client_Library`：后者是底层的 RTDE *协议*库
> （读写寄存器）。它需要示教器上常驻一个 URScript 程序，从寄存器读取目标并调用 `servoj`。`ur_rtde`
> 正好封装了这种模式，并直接提供 `servoJ/moveJ/speedJ/servoL`，更适合写驱动。

## 真机状态

已在两台真 UR 机械臂上跑过（2026-11-12），单臂以及作为 `ur5e_dual`
双臂组装体：`get_observation`（关节、TCP 位姿）和 CARTESIAN `servoL` 让每个 TCP
在 Z 方向 ±10 cm，无保护性停止，净漂移小于 0.1 mm。那边的机械臂实际是
**UR5（CB3）**，所以跑在 `control_hz: 125`；驱动和配置对 UR5e 完全一样，UR5e 的
e-Series 控制器最高到 500 Hz。尚未在真机上验证：`reset()` / `moveJ` 回 home 和 `estop()`
路径，以及 POSITION / PD / VELOCITY 伺服（这些仍只对假的 `ur_rtde` 测过）。每种都先低速试。

## 1. 前置条件

```bash
pip install ur_rtde
```

没有它也能构造驱动并在仿真中运行；只有硬件上的 `connect()` 需要它。

## 2. 网络与机器人设置

- 把主机放在与 UR 控制箱相同的子网上。记下控制箱 IP，设为 `body.ip`。
- 在示教器上**启用 External Control / RTDE**，并确保机器人处于 Remote Control 模式（否则 RTDE 控制接口
  无法接管）。
- UR5e 等 e-Series 机械臂最高支持 500 Hz；CB3 运行在 125 Hz。配置中的 `control_hz` 必须与之一致
  （驱动把 `1/control_hz` 用作 servoJ 的时间步长 `dt`）。

## 3. 上电与安全

1. 给机器人上电，松开抱闸，清空工作空间，急停放在手边。
2. **先设置初始姿态**：在配置中给出 `home_q`（6 个关节，rad），`reset()` 就会按 `home_speed` / `home_acceleration`
   `moveJ` 到那里并等它到位。不设就不动。别的线程调 `estop()` 会停下这次移动（按 `stop_deceleration` `stopJ`），
   `reset()` 抛出 `RobotStoppedError`。`reset()` / `estop()` 这条路径只对假的
   `ur_rtde` 跑过，尚未在真机上验证（见[真机状态](#真机状态)）。
3. servoJ 是非阻塞伺服；控制循环按 `control_hz` 的频率发送目标。
4. `disconnect()` 依次调用 `servoStop` + `stopScript` + `disconnect`；用完后调用它。

## 4. 配置

`ur_ur5e`：

```yaml
embodiment:
  control_hz: 125           # e-Series can go up to 500
body:
  driver: ur_rtde
  ip: 192.168.1.102         # control box IP
  # home_q: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
  # servo_velocity: 0.5
  # servo_acceleration: 0.5
  # servo_lookahead: 0.1    # servoJ smoothing lookahead (s)
  # servo_gain: 300         # servoJ tracking gain
  # home_speed: 0.5         # moveJ to home_q (rad/s)
  # home_acceleration: 0.5  # (rad/s^2)
  # stop_deceleration: 2.0  # stopJ on estop() (rad/s^2)
  # command_timeout_s: 0.2  # 这么久没有新的 VELOCITY 就 speedStop；null 关掉
```

关节顺序：从基座到手腕（`shoulder_pan, shoulder_lift, elbow, wrist_1/2/3`），与 RTDE 的 `actual_q` 一致。

**两台臂。** `ur5e_dual` 把两台 `ur_ur5e` 并排组装成一个机器人，每台的关节和 TCP
带 `left/` 或 `right/` 前缀（`left/shoulder_pan`、`right/tcp`）。两台都默认用 `toy`
驱动；用按部件的 override 把每台指向各自的控制箱：

```python
robot = make_robot("ur5e_dual", overrides={
    "left/body":  {"driver": {"type": "ur_rtde", "ip": "192.168.1.101"}},
    "right/body": {"driver": {"type": "ur_rtde", "ip": "192.168.1.102"}}})
```

一个带 `parts` 的 `Action` 同时命令两台臂（每个部件读自己的 `tcp`）；组装体的
`control_hz` 是外层循环，每台臂的 RTDE 内层循环不变。

## 5. 最小循环

用[快速开始](../../README.zh-CN.md#仿真)中的基本循环检查连接，在你的本地配置（内置配置加上面的
驱动和地址）上运行：读一次观测，把当前关节位置作为 POSITION 目标发回去，然后保持。

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_ur5e.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

驱动做了什么：`get_observation()` 读取 `getActualQ/getActualQd/getActualCurrent` 和 `getActualTCPPose`。
RTDE 报告的 TCP 是 `[x,y,z,Rx,Ry,Rz]`（米 + 轴角 rad）；驱动把它以 7 维向量 `[x, y, z, qx, qy, qz, qw]`
存入 `ee_poses["tcp"]`。POSITION / PD 用 `servoJ`，VELOCITY 用 `speedJ`，CARTESIAN 用 `servoL`
（读取 7 维向量 `ee_targets["tcp"]` 并转换回轴角）。

## 6. 常见问题

- **Remote Control 模式**：机器人不在远程模式时，RTDE 控制接口无法连接或接管。
- **`dt` 要匹配**：servoJ 的 `dt` 来自 `control_hz`。循环必须跟得上这个频率，否则运动会卡顿，或触发保护性停止。
  e-Series 机械臂通常运行在 500 Hz（dt = 2 ms）。
- **笛卡尔奇异点 / 极限**：用 servoL 去往不可达或奇异的位姿会触发保护性停止。先用小步长验证。
- **从停止中恢复**：保护性停止或急停之后，在示教器上清除它并重新使能。
- **驱动只有一部分经过真机验证**：读取和 CARTESIAN `servoL` 已在真机上跑过；
  `reset()` / `estop()` 路径和 POSITION / PD / VELOCITY 伺服还没有（只对假的
  `ur_rtde` 测过）。见[真机状态](#真机状态)，每种模式先低速试。
- **PD 忽略 `kp` / `kd`**：`PD` 可以发，和 `POSITION` 一样用 `servoJ` 跟踪；刚度是固定的 `servo_gain`。
- **VELOCITY 和 `joint_limit`**：关节速度放在 `joint_target` 里，所以安全闸的 `joint_limit` 限的是速度（rad/s），不是位置。
- **VELOCITY 会一直转**：`speedJ` 一直执行到下一条指令。调用方停发后机械臂保持最后的速度；用 `estop()` 或零速度动作停下。
- **RTDE 库不同**：不要把 `ur_rtde`（本驱动使用的）和 UR 的 RTDE 协议库（`RTDE_Python_Client_Library`）
  搞混；后者的 API 完全不同（寄存器 + URScript）。

## 7. RealSense 相机（可选）

UR 机械臂本身没有相机。插在跑 hiRobot 的计算机上的 Intel RealSense 是一个 `cameras`
条目，用 `intel_realsense` 驱动（`src/hirobot/drivers/camera/intel/realsense/`），
走 `pyrealsense2`（`pip install pyrealsense2`）。它采集彩色传感器，设置
`depth: true` 时还会把深度传感器对齐到彩色，使 `Observation.depth[name]` 与
`Observation.images[name]` 共用同一像素网格（单位米，无读数处为 0）。已随 UR 机械臂
在 D435i 和 L515 上验证（2026-11-12），两台同时出图。D405 用同一驱动，但没跑过。

把它加进你的机器人配置，或用内置的 `intel/d435i_realsense` 配置挂到臂上：

```yaml
cameras:
  - name: camera
    from: camera/intel/d435i_realsense
    overrides:
      driver:
        serial: "<your-serial>"   # 接了多个 RealSense 时设置
```

驱动会打开找到的第一个 RealSense；接了多个时，把 `driver.serial` 设为设备序列号
（`pyrealsense2` 能枚举出来，`rs-enumerate-devices` 也会打印），好让固定的那台相机
保持角色。相机是只读源，所以在 `read_only` 下也能跑；用 toy 驱动离线时会跳过。

### L515

L515 有自己的内置配置 `intel/l515_realsense`。和 D435i 不同的有两点：

- **SDK 版本。** 较新的 `pyrealsense2` 去掉了 L515：2.54.2 能列出，2.59.1 列不出（中间的版本没试过）。
  装 `pip install pyrealsense2==2.54.2.5684`（已验证）。2.54.2 也能跑 D435i，一个环境可以同时
  驱动两台。用更新的 `pyrealsense2` 时，设备列表里直接没有 L515，按序列号打开会失败。
- **深度尺寸。** L515 接在 USB 2 上时，深度只有 320x240（彩色最高 640x480、30 fps），所以配置里
  设了 `depth_width: 320`、`depth_height: 240`。深度仍对齐到彩色，`Observation.depth` 出来是
  640x480。按彩色尺寸要深度会报 `Couldn't resolve requests`。接 USB 3 时有更大的深度尺寸。

L515 启动后约 1.1 s 才出第一帧深度，所以配置设了 `timeout_ms: 2000`。`pyrealsense2` 报的 L515
序列号是 8 位十六进制短串，不是 `lsusb` 里那个 16 位的 USB 序列号。

```yaml
cameras:
  - name: scene
    from: camera/intel/l515_realsense
    overrides:
      driver:
        serial: "<your-serial>"
```

## 8. 夹爪（Robotiq 2F）

装在法兰上的 Robotiq 2F-85（或 2F-140）是一个 `opening` 末端执行器，用 `robotiq_urcap`
驱动（`src/hirobot/drivers/end_effector/robotiq/urcap/`）。它通过 UR 控制器上安装的
Robotiq URCap 访问夹爪：URCap 在控制器 IP 的 TCP 63352 端口上提供夹爪寄存器。不需要
额外接线，也不需要安装 Python 包。夹爪在 `Observation.gripper["gripper"]` 里读出，
用 `Action.gripper["gripper"]` 下发，0 为闭合、1 为张开，任何控制模式下都可用。

`ur_ur5e_robotiq` 是法兰（`flange` 挂载点）上装了夹爪的 UR5e。把臂和夹爪指向同一个控制器：

```python
robot = make_robot("ur_ur5e_robotiq", overrides={
    "body":    {"driver": {"type": "ur_rtde", "ip": "192.168.1.102"}},
    "gripper": {"driver": {"type": "robotiq_urcap", "host": "192.168.1.102"}}})
```

驱动参数（`end_effector/robotiq/2f85_urcap`）：`host`、`port`（63352）、`speed` 和
`force`（0-255，默认 255）、`activate`（默认 `false`）。

- **激活。** 默认情况下驱动假定夹爪已经激活（示教器上的 URCap 或之前的会话激活过）；
  未激活的夹爪收到命令不会动。`activate: true` 会在 `connect()` 时按需激活，这会完整闭合
  再张开一次，只在工作空间清空时用。
- **不等待动作完成。** `command_opening` 立即返回；位置从 `get_state()` 读。
  `extra["object"]` 是 Robotiq 的物体检测（0 运动中，1 或 2 碰到物体停下，3 到达目标），
  `extra["fault"]` 是故障码。
- **没有断力矩模式。** Robotiq 2F 没有被动模式，所以 `damp()` / `estop()` 只是停住动作，
  夹爪保持在当前位置。
- **速度和力。** 默认是最大值。先用小一些的值（比如 `speed: 120`、`force: 80`），确认抓取
  情况后再调。

真机状态：通过本驱动在 UR5（CB3）控制器上的 2F 夹爪上做过一次闭合再张开（2026-11-12），
位置读数从全开到闭合极限再回到全开，无故障。UR5e 配置走同一个控制器接口。
