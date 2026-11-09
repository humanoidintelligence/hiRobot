# AgiBot G1 设置指南（GDK topics，底盘控制器）

[English](./agibot_g1.md) | 简体中文

驱动：`agibot_gdk` 本体（`src/hirobot/drivers/body/agibot/gdk/`）和
`agibot_cosine` 摄像头（`src/hirobot/drivers/camera/agibot/cosine/`）。
配置：`agibot_g1`。

AgiBot G1（精灵 G01）是装在轮式底盘上的双臂机器人：两条 7 自由度机械臂，配 Omnipicker 夹爪；
一个腰部（升降和俯仰）；一个头部（偏航和俯仰）；一台头部 RGB-D 摄像头，每个手腕各一台彩色摄像头。
hiRobot **在机器人自带的电脑上**运行，与厂商的运动控制器并存，通过 GDK 的 DDS topics 与它通信。
底盘在机器人网络上有自己的控制盒，驱动通过 HTTP 和 WebSocket 访问它。

驱动不使用厂商的 `a2d_sdk.robot` Python 层：这一层需要 ruckig 和 scipy，还需要比机器人系统 Python
自带版本更新的消息包。驱动直接在相同的 topics 上订阅和发布。

## 1. 前置条件

在机器人的电脑上（Ubuntu 22.04，系统 Python 3.10，`/usr/bin/python3`）：

| 包 | 来源 |
|---|---|
| `cosine_bus` 2.0.0 | 预装 |
| `websockets`、`numpy`、`pydantic`、`PyYAML`、`opencv-python` | 测试所用的机器人上已预装 |
| `genie_msgs_pb` >= 0.8.0 | 在机器人自带的 GDK 安装包中；见下文 |

系统自带的消息包可能低于 0.8.0。机器人上的 GDK 安装包（厂商应用目录下的
`a2d_sdk/gdk_installer/gdk_device_installer.tar.gz`）中有 `genie_msgs_pb-0.8.0-py3-none-any.whl`。
把这个 wheel 解压到你自己的目录并加入 `PYTHONPATH`；不需要安装到系统 Python 中：

```bash
mkdir -p <dir>/vendor && cd <dir>/vendor
tar xzf <path to>/gdk_device_installer.tar.gz gdk_device_installer/genie_msgs_pb-0.8.0-py3-none-any.whl
python3 -m zipfile -e gdk_device_installer/genie_msgs_pb-0.8.0-py3-none-any.whl genie_msgs_pb-0.8.0
```

不带依赖安装 hiRobot，让系统包保持原样：

```bash
/usr/bin/python3 -m pip install --user --no-deps <path to hiRobot>
```

也可以用 `PYTHONPATH=<hiRobot>/src` 从源码副本运行。

每个运行 hiRobot 的 shell 都需要厂商环境，它会设置库路径和 DDS 发现变量（`LOCATOR_IP`、
`AORTA_DISCOVERY_URI`）：

```bash
source <vendor app dir>/env.sh <vendor app dir>/lib
export PYTHONPATH=<dir>/vendor/genie_msgs_pb-0.8.0:$PYTHONPATH
```

没有它，`connect()` 会在等待状态 topics 时超时，摄像头驱动会指出缺少哪些变量。

## 2. 上电与安全

- 机械臂周围以及底盘周围约 0.5 m 的范围内保持无人无物，急停放在手边。
- **运动控制器有三种模式：STOP、SERVO 和 PLANNING。**
  `connect()` 只读。控制器处于 STOP 时，`reset()` 会请求切到 SERVO；它不移动任何关节。
  以下情况 `send_action()` 会拒绝：控制器不在 SERVO、急停被按下、控制器报错，或机械臂状态比
  `max_state_age` 更旧。`damp()` 和 `disconnect()` 发送 `MotionStop`，它让关节停在当前位置，
  并把控制器切到 STOP。
- **只有 `groups` 中的关节组会被控制**（默认：双臂）。腰部或头部关节的目标必须等于其实测位置
  （在 `hold_tolerance` 以内），否则动作会被拒绝；不会静默丢弃任何内容。
- 指令带有 `lifetime`（0.2 s）：指令停止到来后，运动控制器就不再跟随。
- 底盘控制盒只在**软手动**（soft-manual）模式下接受速度指令。在底盘控制器的网页上切换；驱动从不更改它。
  在软手动下，控制盒报告 `isHardAuto=true` 和 `isSoftAuto=false`；其他任何组合下，驱动都会拒绝非零的
  `BASE_VELOCITY`。
- 驱动每 100 ms 重发一次最新的底盘速度；如果 `command_timeout`（0.3 s）内没有新的
  `BASE_VELOCITY`，就切换为零速。
- 第一次查看时用 `read_only=True`：它不创建 publisher，发送指令和调用 `reset()` 都会抛出异常。

## 3. 配置

`agibot_g1`（含 `body/agibot/g1` 和 `camera/agibot/g1`）：

```yaml
body:
  driver: toy              # set to 'agibot_gdk' on the robot's computer
  # groups: [left_arm, right_arm]   # joint groups agibot_gdk commands
  # chassis_host: <chassis box IP>  # enables base state and BASE_VELOCITY
safety:
  gripper_range: [0, 1]
```

驱动参数（`AgibotG1Robot`）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `groups` | `[left_arm, right_arm]` | 要控制的关节组：`left_arm`、`right_arm`、`waist`、`head` |
| `read_only` | `false` | 不创建 publisher；发送指令和 `reset()` 会抛出异常 |
| `lifetime` | `0.2` | 控制器跟随一条指令的时长（s） |
| `hold_tolerance` | `0.02` | 未受控关节的目标与其实测位置之间允许的偏差（rad 或 m） |
| `max_state_age` | `0.2` | 机械臂状态或运动状态比这更旧时拒绝发送指令（s） |
| `connect_timeout` | `3.0` | 等待每个状态 topic 的秒数 |
| `gripper_raw_closed` | `120.0` | 夹爪完全闭合时的原始读数 |
| `chassis_host` | 无 | 底盘控制盒地址；不设则没有底盘 |
| `chassis_id` | `780` | 底盘控制盒响应的 `id`（在其网页上） |
| `command_timeout` | `0.3` | 最后一条 `BASE_VELOCITY` 之后多少秒切为零速 |
| `max_linear_speed` | `0.2` | 接受的最大 `vx`（m/s） |
| `max_angular_speed` | `0.5` | 接受的最大 `wz`（rad/s） |

关节按配置顺序为：`waist_lift_joint`（移动关节，单位米）、
`waist_pitch_joint`、`head_yaw_joint`、`head_pitch_joint`、
`left_arm_joint1` ... `left_arm_joint7`、`right_arm_joint1` ...
`right_arm_joint7`。它们是运动控制器 `a2d_v01` 模型中的名字去掉 `idxNN_` 前缀。`position_limit`
来自运行中控制器自身的配置，对肘部（`joint4`）和腰部升降比 URDF 更严。

摄像头（`agibot_cosine`）：

| 名称 | 摄像头 | Topic | 图像 | 深度 |
|---|---|---|---|---|
| `head` | RealSense D457 | `/camera/head_color`、`/camera/head_depth` | 1280×800 RGB | 单位米，与同一次采集的彩色帧配对 |
| `left_wrist` | RealSense D405 | `/camera/hand_left_color` | 848×480 RGB | 无：厂商摄像头服务关闭了手腕深度（`depthEnable: false`） |
| `right_wrist` | RealSense D405 | `/camera/hand_right_color` | 848×480 RGB | 无 |

## 4. 观测与动作

| 字段 | 内容 |
|---|---|
| `joint_pos` | 来自机械臂、腰部和颈部状态 topics 的 18 个关节 |
| `joint_vel` | 在测试所用固件上为 `None`（机械臂 topic 的速度数量与其关节数对不上） |
| `gripper` | `left_gripper`、`right_gripper`：0 闭合，1 张开 |
| `ee_poses` | `left_gripper`、`right_gripper`：`arm_{left,right}_link7` 在 `base_link` 中的位姿，`[x, y, z, qx, qy, qz, qw]`，与运动控制器报告的一致 |
| `base_position`、`base_orientation` | 底盘里程计（需设置 `chassis_host`） |
| `images`、`depth` | 上面的摄像头 |

`get_status()` 给出底盘电量，并在 `extra` 中给出运动模式、各部件的错误码和急停状态，以及底盘标志位
（`isSoftAuto`、`isHardAuto`、`isScram`、`isPause`、`exceptions`）。

动作：

- `POSITION`：全部 18 个关节的 `joint_target`，加上 `gripper`（0 闭合，1 张开）。驱动会把夹爪值换算成
  GDK 的刻度（0 张开，1 闭合）。
- `BASE_VELOCITY`：`[vx, 0, wz]`，单位 m/s 和 rad/s；`vy` 必须为 0。从上方看，`wz` 为正时逆时针转动。

## 5. 最小循环

先只读：

```python
from hirobot import make_robot

robot = make_robot("agibot_g1", body_overrides={"driver": "agibot_gdk", "read_only": True,
                                                "chassis_host": "<chassis box IP>"})
robot.connect()
obs = robot.get_observation()
print(obs.joint_pos, obs.gripper, obs.base_position, {k: v.shape for k, v in obs.images.items()})
print(robot.get_status())
robot.disconnect()
```

然后做一个小幅动作，从实测姿态斜坡过渡（这里是右肩 `right_arm_joint1`，以 30 Hz 在 3 s 内转 +10°；
正方向让手臂向前抬起）：

```python
import math, time
import numpy as np
from hirobot import Action, ControlMode, make_robot

robot = make_robot("agibot_g1", body_overrides={"driver": "agibot_gdk"})
robot.connect()
robot.reset()                              # STOP -> SERVO; moves no joint
q0 = robot.get_observation().joint_pos.copy()
j = robot.embodiment.joint_names.index("right_arm_joint1")
for k in range(91):
    q = q0.copy()
    q[j] += math.radians(10) * 0.5 * (1 - math.cos(math.pi * k / 90))
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=q))
    time.sleep(1 / 30)
robot.disconnect()                         # MotionStop: the arm stays where it is
```

## 6. 常见问题

- **`connect` 等待状态 topics 时超时**：当前 shell 没有 source 厂商环境，或运动控制器没有运行。
- **`genie_msgs_pb` 报 `ImportError`**：0.8.0 的包不在 `PYTHONPATH` 上（第 1 节）。
- **`send_action` 提示控制器不在 SERVO**：调用 `reset()`。如果模式一直停在 STOP，说明另一个控制方
  （HMI、遥操作）占用着机器人，或控制器报了错；`get_status().extra` 会显示错误码。
- **底盘运动被拒绝，提示 "not in soft-manual mode"**：在底盘的网页上把底盘切到软手动。
- **底盘起步慢，停止后还会继续动**：以 0.2 rad/s 转向时，前 0.8 s 几乎没动，发出零速后又多转了约 7°。
  要停在某个朝向或某个距离，就提前停，并等它停稳后再读里程计。
- **在测试所用的机器人上，头部颈部状态报告了错误码**（`get_status().extra` 中的 `hal.neck_error`）。
  默认不控制头部；把 `head` 加入 `groups` 之前，先在厂商 HMI 上检查头部。
- **手腕深度**：手腕深度 topics 存在，但不发布任何帧：厂商摄像头服务的配置关闭了手腕深度
  （`depthEnable: false`）。hiRobot 尚未尝试打开它。

## 7. 测试结果

在固件 `a2d_G01_2.3.4`、`cosine_bus` 2.0.0、`genie_msgs_pb` 0.8.0、底盘固件 2.0.0.7 的 G1 上：

| 测试 | 结果 |
|---|---|
| 只读 | 18 个关节与原始 topics 一致；夹爪、手腕位姿、里程计、电量 |
| 摄像头 | 头部 1280×800 RGB + 深度，21–25 帧/s；手腕 848×480，28–30 帧/s |
| 夹爪 | 两只都各用 2 s 张开（读数 0.998 / 1.0）和闭合（0 / 0） |
| 右肩 | `right_arm_joint1` 转 +10° 和 +30° 再回来；跟踪误差在 0.02° 以内，其他关节 < 0.001 rad |
| 底盘 | 按里程计，以 0.05 m/s 前进 103 mm、后退 101 mm；以 0.2 rad/s 顺时针转向，转了 25° |
