# Fourier GR1 设置指南（GR1T1 / GR1T2）

[English](./fourier.md) | 简体中文

驱动：`fourier_grx`（`src/hirobot/drivers/body/fourier/grx/`）。它封装了
`fourier_grx_client.RobotClient`，后者通过 Zenoh 连接 GRX 服务器（默认 `server_ip:7447`）。
GR1T1 和 GR1T2 共用同一套 32 关节 API；只有 embodiment 配置不同。

## 1. 前置条件

安装客户端（[fourier-grx-client](https://github.com/FFTAI/fourier-grx-client)）：

```bash
pip install fourier-grx-client
```

**GRX 服务器必须已在机器人上运行**（由厂商提供）；客户端只负责连接它。

> 单位：从客户端 v0.2.0 起，所有面向用户的值都以**弧度**表示。驱动读写都用弧度，从不传
> `degrees=True`。客户端报告的 IMU 角速度单位是 deg/s；驱动会把它换算成 rad/s。

## 2. 网络

- 主机必须能访问 GRX 服务器的 IP（在机器人本机上通常是 `localhost`，或同一子网内的某个 IP）。
- 端口固定为 **7447**（TCP / Zenoh）。
- `namespace` 默认从环境变量 `GRX_ROBOT_ID` 或 `~/.fourier/robot_id.yaml` 解析；通常不需要设置。

## 3. 上电与安全

1. 机器人已吊起或托住，周围无人无物，急停在手边。
2. 启动 GRX 服务器。
3. **使能电机**：只有在 `set_enable(True)` 之后指令才会生效。驱动在 `connect()` 和 `reset()` 中使能
   （`enable_on_connect=True`）。
4. **断电**：客户端的 `close()` **不会**让电机失能。驱动的 `disconnect()` 会在 `close()` 之前调用
   `set_enable(False)`；用完后务必调用它。
5. 要标定零位，在现场调用 `client.calibrate_sensors()`（本驱动没有封装；需要时手动执行）。

## 4. 配置

`fourier_gr1t1`：

```yaml
body:
  driver: fourier_grx
  server_ip: localhost      # GRX server address
  # namespace: gr/my_robot  # usually resolved automatically
  # freq: 400               # state refresh rate
  # enable_on_connect: true
```

关节顺序沿用 `config_GR1_T1.yaml` / `config_GR1_T2.yaml` 中的 32 关节列表。增益数组必须恰好有 32 项，
否则客户端会**静默忽略**它们。

## 5. 最小循环

用[快速开始](../../README.zh-CN.md#仿真)中的基本循环检查连接，在你的本地配置（内置配置加上面的
驱动和地址）上运行：读一次观测，把当前关节位置作为 POSITION 目标发回去，然后保持。

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_gr1t1.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

驱动做了什么：`connect()` 构造一个 `RobotClient`（它在构造函数中连接 Zenoh，并阻塞到第一条状态到达），
然后使能电机。`get_observation()` 读取 `joint_positions/velocities/efforts`（rad）和 IMU。
`send_action()` 使用高频路径 `control_joints("position", ..., degrees=False)`。

## 6. 常见问题

- **先使能**：不使能的话，电机会忽略指令。
- **不要高频调用 `move_joints`**：高频控制循环用 `control_joints`（一次轻量发布，REAL_TIME + DROP）。
  `move_joints(duration>0)` / `movej` / `movel` 会启动线程或阻塞在规划上，不适合高频使用。驱动用的是
  `control_joints`。
- **增益长度**：kp/kd 必须是长度为 32 的数组，否则会被忽略。
- **连不上**：构造 `RobotClient` 时会做一次存活检查（`robot/info`），失败时抛出
  `FourierConnectionError`。检查服务器是否在运行，以及 IP 和 7447 端口是否可达。
- **灵巧手**：GR1 的手使用单独的接口（`fourier_dhx.sdk.DexHand`），本驱动不涵盖；需要时单独集成。
