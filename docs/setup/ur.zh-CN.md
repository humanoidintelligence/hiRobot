# Universal Robots 设置指南（UR5e / UR 系列）

[English](./ur.md) | 简体中文

驱动：`ur_rtde`（`src/hirobot/drivers/body/ur/rtde/`）。一台 6 自由度固定基座机械臂，没有 IMU 和基座状态。
它封装了 `ur_rtde`（`RTDEControlInterface` + `RTDEReceiveInterface`），用 `servoJ` 高频伺服关节，
用 `servoL` 伺服 TCP 笛卡尔位姿。

> 为什么用 `ur_rtde` 而不用 UR 官方的 `RTDE_Python_Client_Library`：后者是底层的 RTDE *协议*库
> （读写寄存器）。它需要示教器上常驻一个 URScript 程序，从寄存器读取目标并调用 `servoj`。`ur_rtde`
> 正好封装了这种模式，并直接提供 `servoJ/moveJ/speedJ/servoL`，更适合写驱动。

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
   `reset()` 抛出 `RobotStoppedError`。这条路径只对假的 `ur_rtde` 跑过。
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
- **还没在真机上跑过**：驱动只对假的 `ur_rtde` 测过。每种模式先低速试。
- **PD 忽略 `kp` / `kd`**：`PD` 可以发，和 `POSITION` 一样用 `servoJ` 跟踪；刚度是固定的 `servo_gain`。
- **VELOCITY 和 `joint_limit`**：关节速度放在 `joint_target` 里，所以安全闸的 `joint_limit` 限的是速度（rad/s），不是位置。
- **VELOCITY 会一直转**：`speedJ` 一直执行到下一条指令。调用方停发后机械臂保持最后的速度；用 `estop()` 或零速度动作停下。
- **RTDE 库不同**：不要把 `ur_rtde`（本驱动使用的）和 UR 的 RTDE 协议库（`RTDE_Python_Client_Library`）
  搞混；后者的 API 完全不同（寄存器 + URScript）。
