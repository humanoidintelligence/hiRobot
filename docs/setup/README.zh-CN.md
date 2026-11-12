# 机器人设置

[English](./README.md) | 简体中文

每款支持的机器人从上电到通过 hiRobot 发出第一条指令的步骤。每篇指南都包含前置条件（SDK、网络）、
上电与安全、连接参数、最小 hiRobot 循环和常见问题。

> 接触硬件之前，先用 `driver: toy` 把控制循环跑起来（见主 README 的快速开始），确认 embodiment 和
> policy 接得没问题。然后再把配置里的 `body.driver` 换成硬件驱动。

## 支持的机器人

| 机器人 | 类型 | 驱动 | SDK | 配置 | 指南 |
|---|---|---|---|---|---|
| Unitree G1 | 人形，29 自由度 | `unitree_sdk2` | unitree_sdk2py (hg) | `unitree_g1` | [unitree.zh-CN.md](unitree.zh-CN.md) |
| Unitree H1-2 | 人形，27 自由度 | `unitree_sdk2` | unitree_sdk2py (hg) | `unitree_h1_2` | [unitree.zh-CN.md](unitree.zh-CN.md) |
| Unitree H2 | 人形，31 自由度 | `unitree_sdk2` | unitree_sdk2py (hg) | `unitree_h2` | [unitree.zh-CN.md](unitree.zh-CN.md) |
| Unitree H1 | 人形，19 自由度 | `unitree_sdk2` | unitree_sdk2py (go) | `unitree_h1` | [unitree.zh-CN.md](unitree.zh-CN.md) |
| Fourier GR1T1 | 人形，32 自由度 | `fourier_grx` | fourier-grx-client | `fourier_gr1t1` | [fourier.zh-CN.md](fourier.zh-CN.md) |
| Fourier GR1T2 | 人形，32 自由度 | `fourier_grx` | fourier-grx-client | `fourier_gr1t2` | [fourier.zh-CN.md](fourier.zh-CN.md) |
| AgiBot G1 | 轮式底盘上的双臂，18 自由度 | `agibot_gdk` | GDK topics (cosine_bus) | `agibot_g1` | [agibot_g1.zh-CN.md](agibot_g1.zh-CN.md) |
| AgiBot X1 | 人形，29 自由度 | `agibot_aimrt` | ROS2 + AimRT bridge | `agibot_x1` | [agibot_x1.zh-CN.md](agibot_x1.zh-CN.md) |
| Franka FR3 | 带夹爪的机械臂，7 自由度，DROID 平台 | `franka_droid` | zerorpc 连 DROID 机器人服务 | `franka_fr3_droid` | [franka.zh-CN.md](franka.zh-CN.md) |
| UR5e | 机械臂，6 自由度 | `ur_rtde` | ur_rtde | `ur_ur5e` | [ur.zh-CN.md](ur.zh-CN.md) |
| AgileX PiPER | 带夹爪的机械臂，6 自由度 | `agilex_piper_sdk` | piper_sdk | `agilex_piper` | [agilex_piper.zh-CN.md](agilex_piper.zh-CN.md) |

## 通用流程

1. **安装 SDK**。每个驱动都**延迟**导入厂商 SDK：没有 SDK 也能构造驱动并在仿真中运行，只有
   `connect()` 与硬件通信时才需要它。按各指南列出的 SDK 安装。`connect()` 会先对照驱动的
   `manifest.yaml`（`src/hirobot/drivers/<part>/<vendor>/<sdk>/`）检查 SDK：缺少 SDK，或版本低于
   manifest 中的 `minimum`，会停下并给出安装命令；版本高于 `verified` 下列出的所有版本，会记录一条警告并继续运行。
2. **网络与上电**。见各指南的“上电与安全”。
3. **设置连接参数**。写一份本地配置，以 `defaults: [<robot>]`（内置配置名，见 `hirobot configs`）开头，
   在 `body:` 下设置驱动名和连接参数（网卡 / IP / `server_ip`）。这些键会原样传给驱动的构造函数。
   `hirobot info --config my_robot.yaml --resolved` 会显示合并后的结果。
4. **运行最小循环**。用你的配置打开机器人，读一次观测，再把当前关节位置作为 POSITION 目标发回去
   （循环见[快速开始](../../README.zh-CN.md#仿真)）。每篇指南都给出了对应机器人的写法。

> ⚠️ **安全**：人形机器人首次上电时必须吊起或托住。先从阻尼模式开始，并且控制栈的第一步必须始终是
> 从**实测**姿态出发的斜坡过渡，绝不能直接跳到目标。每个驱动的 `reset()` 已经带斜坡过渡，但仍必须有人守在急停旁。
