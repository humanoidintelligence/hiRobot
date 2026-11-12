# AgileX PiPER 设置指南

[English](./agilex_piper.md) | 简体中文

驱动：`agilex_piper_sdk`（`src/hirobot/drivers/body/agilex/piper_sdk/`）。6 自由度机械臂，平行夹爪接在同一条
CAN 总线上。驱动封装了松灵（AgileX）的 [`piper_sdk`](https://github.com/agilexrobotics/piper_sdk)
（`C_PiperInterface_V2`），走 SocketCAN。内置配置里的场景相机是 Orbbec Dabai DC1，用 `orbbec_pyorbbecsdk`
相机驱动（见[第 6 节](#6-orbbec-相机)）。

## 真机状态

2026-11-12 在一台真 PiPER（双臂工位的右臂）上跑过，`piper_sdk` 0.6.1：

- `connect()` 和读取不发任何帧（CAN 接口的发送计数没变）。关节、法兰位姿、夹爪行程、电机温度、电压、使能状态都能读到。
- POSITION 运动把法兰抬高 99 mm（joint2 +11°、joint3 −19°，速度 20 %）再放回；夹爪合到 20 mm 再张开。
  结束时各关节离起点不到 0.2°。
- 通过 `make_robot` 同时读机械臂、夹爪和一台 Dabai DC1（彩色 640x480 + 对齐的深度）。

还没在真机上跑过：`estop()`，以及上一个运动还没走完时再发命令（`JointCtrl` 会在运动中替换目标）。

## 1. 前置条件

```bash
pip install piper_sdk            # 已验证 0.6.1
sudo apt install can-utils       # candump，用来看总线
```

PiPER 的 USB 转 CAN 模块会出现为一个 SocketCAN 接口（多臂工位上是 `can0`、`can1`……）。每次开机或重插后按
1 Mbit/s 拉起：

```bash
sudo ip link set can0 up type can bitrate 1000000
candump can0 -n 5      # 0x2A1..0x2A8（状态）和 0x251..0x256（电机）
```

接了多个模块时，`can<N>` 的编号跟 USB 枚举顺序走。`piper_sdk` 自带的 `can_activate.sh` 能按 USB 端口给接口起名，
让每条臂的接口名固定。

## 2. 上电与安全

- 给机械臂上电，等控制器启动；起来后 `candump` 能看到约 200 Hz 的反馈。
- 第一条命令之前驱动什么都不发，所以别人在用这条臂时也可以 `connect()` 和 `get_observation()`。
  要一直保持只读，用 `read_only: true`（见 [api.zh-CN.md](../api.zh-CN.md)）。
- 第一条命令会使能没上使能的电机（`EnableArm`）。发第一条命令前，确认目标离当前位置不远。
- 运动用控制器自己的关节运动（`MOVE J`），速度是最高速度的 `speed_percent`（默认 30 %）。轨迹由控制器规划，
  远处的目标也按这个速度走过去，不会一步跳到。
- `estop()` 发一个到当前实测位置的关节运动，机械臂停在原地并保持上电。它不用控制器的快速急停（`EmergencyStop`），
  因为那会让机械臂在阻尼下慢慢下落。工位的硬件急停要放在手边。

## 3. 配置

`robots/agilex_piper` 是机械臂、夹爪和一台场景相机，默认用 toy 驱动。上真机时，在本地配置里写 CAN 接口和相机：

```yaml
defaults: [robots/agilex_piper]
assembly:
  parts:
    body: {overrides: {driver: {type: agilex_piper_sdk, can: can0}}}
    scene: {overrides: {driver: {serial: CC1N00000000}}}
```

只要机械臂、不要相机时用 `body/agilex/piper`。驱动参数：

| 参数 | 默认 | 含义 |
|---|---|---|
| `can` | `can0` | 机械臂所在的 SocketCAN 接口 |
| `speed_percent` | `30` | 关节运动速度，最高速度的 1-100 % |
| `gripper` | `true` | 读、控夹爪；`false` 时不碰夹爪 |
| `gripper_max_m` | `0.07` | `Action.gripper = 1` 对应的张开行程 |
| `gripper_effort_n` | `1.0` | 夹持力，0-5 N |
| `feedback_timeout_s` | `2.0` | `connect()` 等关节反馈的时间 |

关节名（`joint1` .. `joint6`）和限位取自松灵的 `piper_description` URDF 和 `piper_sdk` 的关节限位。

## 4. 观测与动作

| 字段 | 来源 | 单位 |
|---|---|---|
| `joint_pos` | 关节反馈（0x2A5-0x2A7） | rad |
| `ee_poses["ee"]` | 控制器算出的法兰位姿（0x2A2-0x2A4） | `[x, y, z, qx, qy, qz, qw]`，米，机械臂基座坐标系 |
| `gripper["ee"]` | 夹爪行程（0x2A8）/ `gripper_max_m` | 0 闭合，1 张开 |
| `get_status().joint_temperature` | 电机反馈（0x261-0x266） | °C |
| `get_status().extra` | 机械臂状态（0x2A1）和电机反馈 | `ctrl_mode`、`arm_status`、`motion_status`、`enabled`、`voltage_v` |

控制器用欧拉角 `RX, RY, RZ`（0.001°）报法兰姿态，`R = Rz(RZ) Ry(RY) Rx(RX)`；驱动把它换成四元数。
`joint_vel` 和 `joint_effort` 不填。

动作：`ControlMode.POSITION`，带 `joint_target`（6 个值，rad）和/或 `gripper={"ee": 开度}`。每条命令先发
`MotionCtrl_2`（CAN 控制、`MOVE J`、`speed_percent`）再发 `JointCtrl`；有夹爪值时发 `GripperCtrl`。

## 5. 最小循环

用 [快速开始](../../README.zh-CN.md) 里的基本循环在本地配置上检查连接：读一次观测，把当前关节位置作为
POSITION 目标发回去，保持不动。

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_piper.yaml")
robot.connect()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos,
                         gripper={"ee": obs.gripper["ee"]}))
robot.disconnect()
```

## 6. Orbbec 相机

驱动：`orbbec_pyorbbecsdk`（`src/hirobot/drivers/camera/orbbec/pyorbbecsdk/`），配置 `camera/orbbec/dabai_dc1`。
出彩色图；设置 `depth: true` 时同时出对齐到彩色的深度，单位米。

Dabai DC1 这类老型号只能用 **1.x** 版 Python 绑定（OrbbecSDK 1.10）枚举到，2.x 绑定（`pyorbbecsdk2`）列不出它们。
从源码编译 1.x 绑定（[pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk) 的 `v1` 分支），把编译产物的 `install/lib`
加到 `PYTHONPATH`。装上 Orbbec 的 udev 规则，不用 root 也能打开相机。

- **numpy 版本。** 绑定要在编译时用的 numpy 大版本下运行。1.x 编译的绑定跑在 numpy 2 下，彩色缓冲全是 `0xff`；
  驱动这时会报 MJPEG 解不出来并指出原因。`opencv-python` 4.12 及以后要求 numpy 2，所以配 1.x 编译的绑定时用
  `numpy<2` 和 `opencv-python<4.12`。
- **选相机。** 设 `serial`（相机上印的序列号）或 `uid`（SDK 列出的 USB 端口路径，如 `3-1.1.3.4-16`）。Dabai DC1
  在设备列表里序列号是空的，所以用 `serial` 时驱动会挨个打开没有名字的设备去读序列号，这可能打断别的进程正在用的相机；
  共用的机器上用 `uid`。
- **对齐。** `align: hw`（默认）在相机上把深度对齐到彩色。Dabai DC1 只在彩色 640x480 时这样做；1280x720 时用
  `align: sw`。深度和彩色尺寸对不上时驱动会报错。
- **一台相机一个进程。** 别的程序正在取流的相机打不开，先停掉那个程序。

驱动参数：`serial`、`uid`、`width` / `height`（640x480）、`fps`（30）、`color_format`（`MJPG`、`RGB` 或 `YUYV`）、
`depth`、`align`、`timeout_ms`、`poll_hz`。

## 7. 常见问题

- **`no PiPER joint feedback on can0`**：接口没拉起、波特率不对，或者机械臂没上电。看 `ip -details link show can0`
  （state UP、bitrate 1000000）和 `candump can0`。
- **动错了臂**：接口编号跟 USB 顺序走。发命令前先读一次，确认哪条臂在哪个接口上。
- **运动刚结束时 `motion_status` 是 `REACH_TARGET_POS_FAILED`**：关节还在稳定时控制器会这样报；稳定后变成
  `REACH_TARGET_POS_SUCCESSFULLY`。
