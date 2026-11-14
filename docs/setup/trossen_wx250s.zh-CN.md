# Trossen WidowX 250S 接入

[English](./trossen_wx250s.md) | 简体中文

驱动：`trossen_dynamixel`（`src/hirobot/drivers/body/trossen/dynamixel/`）。带并
联夹爪的 6 自由度机械臂，由 Dynamixel X 系列舵机经 U2D2（USB 转 TTL）构成。驱动用
Robotis 的 [`dynamixel_sdk`](https://pypi.org/project/dynamixel-sdk/)（协议 2.0，
1 Mbit/s）直接读写舵机控制表，不需要 ROS，也不需要 Interbotix 那套。随附配置再加
两台 Intel RealSense D435i 相机（一台腕部、一台第三人称场景，与双臂遥操采集台一致），
走 `intel_realsense` 相机驱动。

## 硬件状态

在真机 WidowX 250S follower 臂上验证过（2026-11-15），`dynamixel_sdk` 4.1.0：

- `connect()` 和读取都没下发力矩、没下发目标（每个舵机的 `Torque_Enable` 保持 0，
  臂一直是松的）。关节位置、速度、夹爪开度、舵机温度、总线电压、每个舵机的
  力矩/使能状态都读回来了。经驱动读到的 sleep 位姿为 shoulder −1.93、elbow 1.58、
  wrist_angle 0.92 rad，与 Interbotix `wx250s.yaml` 的 `sleep_positions` 对得上。

夹爪开合 tick 已在这只臂上标定（2026-11-15，力矩关闭，手动）：完全闭合 1772、
完全张开 2957；配置用 1780 / 2950，各留一点余量避开机械硬限位。此值在某一只臂上实测，
别的臂可能不同，每只臂都要重新标定（见[第 3 节](#3-配置)）。

真机已下发过首次运动（2026-11-15，逐个关节、小幅度，有人盯着）：第一次
`send_action` 开力矩无跳变，臂能跟随 POSITION 目标，之后又折回 sleep 位。两点注意。
关节离重力平衡姿态太远时可能顶不动、触发舵机过载保护（`Hardware_Error_Status`
bit 5），过载会断力矩；要 reboot 舵机清错并降低速度。舵机按设定的 `Profile_Velocity` /
`Profile_Acceleration` 运动——首次运动前先设一个温和的 profile，让臂缓入目标而不是
猛冲。`estop()` 在运动中保持尚未验证。下发运动要谨慎，见[第 3 节](#3-配置)。

夹爪已通过 driver 验证（`send_action` 命令开度 0.0 / 1.0 / 0.5，读回一致），一台
RealSense 经 `intel_realsense` driver 出了彩色和对齐深度。

## 1. 前置条件

```bash
pip install dynamixel_sdk       # 已验证 4.1.0
```

U2D2 会枚举成一个 USB 串口设备。Interbotix 工作站用 udev 规则把它绑成固定名字：
follower 臂是 `/dev/ttyDXL`；没有这套规则时按 USB 枚举顺序是 `/dev/ttyUSB0`、
`/dev/ttyUSB1`……。确认哪个设备能应答舵机：

```bash
python -c "import dynamixel_sdk as d; p=d.PortHandler('/dev/ttyDXL'); \
p.openPort(); p.setBaudRate(1000000); \
print([i for i in range(1,10) if d.PacketHandler(2.0).ping(p,i)[1]==d.COMM_SUCCESS])"
# 期望 [1, 2, 3, 4, 5, 6, 7, 8, 9]
```

## 2. 上电与安全

- 给臂上电。力矩关闭时（上电后、以及本驱动 `connect()` 之后都是这个状态）臂是松的、
  可反向拖动；扶住它或让它靠在支架上。
- 串口同一时间只能被一个进程占用。Interbotix 工作站上 `xs_sdk` / teleop 栈驱的就是
  同一个 `/dev/ttyDXL`；跑本驱动前先停掉它，并确认没有别的进程占用
  （`fuser /dev/ttyDXL`）。
- 读取不会动臂：`connect()`、`get_observation()`、`get_status()` 都不开力矩、不发目标。
- **第一次** `send_action` 会开力矩。为避免跳变，驱动先把每个舵机测得的位置写成目标，
  再开力矩，让臂停在原地；但第一条目标仍应取在当前位姿附近。
- `estop()` 把测得的位置写成目标，运动中的臂就停在原地并保持上电。它**不**开力矩：
  原本松的臂仍是松的。手要放在臂和工作站电源开关附近。
- shoulder（ID 2）和 elbow（ID 4）各有一个从动舵机（ID 3、5）在固件里跟随它
  （Interbotix 电机配置给每个从动设了 `Secondary_ID`）。驱动只命令和读取 6 个主舵机
  加夹爪；从动自己跟。

## 3. 配置

`robots/trossen_wx250s` 是臂、夹爪和两台 RealSense D435i 相机（腕部 + 场景），默认跑
toy 驱动。上真机时在本地配置里设好串口和每台相机的序列号：

```yaml
defaults: [robots/trossen_wx250s]
assembly:
  parts:
    body:  {overrides: {driver: {type: trossen_dynamixel, port: /dev/ttyDXL}}}
    wrist: {overrides: {driver: {serial: "<腕部相机序列号>"}}}
    scene: {overrides: {driver: {serial: "<场景相机序列号>"}}}
```

`body/trossen/wx250s` 单独是不带相机的臂。驱动参数：

| 参数 | 默认 | 含义 |
|---|---|---|
| `port` | `/dev/ttyDXL` | U2D2 所在的串口设备 |
| `servo_ids` | `(1, 2, 4, 6, 7, 8)` | 6 个关节舵机的 Dynamixel ID，按 embodiment 顺序（不含从动 3、5） |
| `gripper` | `true` | 读取并命令夹爪；`false` 则不碰它 |
| `gripper_id` | `9` | 夹爪舵机的 Dynamixel ID |
| `gripper_close_tick` | `1780` | 完全闭合（`opening` 0）时的舵机位置（tick） |
| `gripper_open_tick` | `2950` | 完全张开（`opening` 1）时的舵机位置（tick） |
| `feedback_timeout_s` | `2.0` | `connect()` 等每个舵机 ping 应答的时长 |

**夹爪标定。** 默认值（闭合 1780、张开 2950）是在某一只臂上实测的，
别的臂可能不同。要重新标定：力矩关闭时手动把夹爪完全闭合、完全张开，各读一次夹爪舵机的
`Present_Position`（地址 132），把两个 tick 填进配置。开度是两端之间的线性映射，所以
不管舵机往哪个方向转，符号都自洽。

关节名（`waist`、`shoulder`、`elbow`、`forearm_roll`、`wrist_angle`、`wrist_rotate`）
和限位来自 Interbotix 的 `wx250s` 电机配置和 URDF。位置换算为
`rad = (ticks − 2048) · 2π / 4096`，所有舵机用同一个公式：Interbotix 在固件里给每个
舵机设了 `Drive_Mode` 反转位，使每个关节都按 URDF 正方向计数。

## 4. 观测与动作

| 字段 | 来源 | 单位 |
|---|---|---|
| `joint_pos` | 各主舵机的 `Present_Position`（地址 132） | rad |
| `joint_vel` | `Present_Velocity`（地址 128） | rad/s |
| `gripper["ee"]` | 夹爪舵机位置在 close/open tick 之间的映射 | 0 闭合，1 张开 |
| `get_status().joint_temperature` | `Present_Temperature`（地址 146） | °C |
| `get_status().extra` | 力矩、电压、硬件错误寄存器 | `enabled`、`voltage_v`、`hardware_error` |

电流、速度、位置由一次 `GroupSyncRead` 读取连续块（地址 126-135）得到。`joint_torque`
和 `ee_poses` 没有填（驱动不含运动学；可由 `Kinematics` 层从 `joint_pos` 算出法兰位姿）。

动作：`ControlMode.POSITION`，带 `joint_target`（6 个值，rad）和/或
`gripper={"ee": 开度}`。关节目标换成 tick 写入各主舵机的 `Goal_Position`；夹爪值映射
后写入夹爪舵机。第一次下发前驱动会把舵机切到位置模式、先写当前位置再开力矩。

## 5. 最小循环

用[快速开始](../../README.zh-CN.md#仿真)里的基础循环在你的本地配置上验证连接：读一次
观测，把当前关节位置作为 POSITION 目标发回去，并保持。

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_wx250s.yaml")
robot.connect()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos,
                         gripper={"ee": obs.gripper["ee"]}))
robot.disconnect()
```

第一次 `send_action` 会开力矩。只在臂有支撑、能自己保持位姿时再跑它。

## 6. RealSense 相机

驱动：`intel_realsense`（`src/hirobot/drivers/camera/intel/realsense/`），配置
`camera/intel/d435i_realsense`。输出彩色，带 `depth: true` 时输出对齐到彩色的深度
（米）。接两台 RealSense 时，设 `wrist.driver.serial` 和 `scene.driver.serial` 让每台
相机保持角色（`rs-enumerate-devices` 列序列号）。RealSense 的通用说明见
[ur.zh-CN.md](ur.zh-CN.md)。

## 排错

- **`could not open /dev/ttyDXL`**：设备名不对，或有别的进程（Interbotix `xs_sdk`）占着
  串口。停掉那套栈，用 `fuser` 查。
- **`no answer from servo(s) [...]`**：臂没上电、波特率不是 1 Mbit/s，或某个 `servo_id`
  填错了。按[第 1 节](#1-前置条件)ping 一遍 ID。
- **夹爪读数卡在 0 或 1**：这只臂的 close/open tick 没标定；测一下（[第 3 节](#3-配置)）。
