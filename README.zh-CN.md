# hiRobot: A Unified Interface Layer for Controlling Diverse Robots from VLA Models and Agents

<div align="center">

**Author:** [The Humanoid Intelligence Team](https://humanoidintelligence.github.io)

[OpenMOSS Group](https://openmoss.ai) / [Shanghai Innovation Institute](https://www.sii.edu.cn/) / [Fudan University](https://www.fudan.edu.cn/en/)

[English](./README.md) | **简体中文** · **文档：** [支持的机器人](docs/supported_robots.zh-CN.md) · [设置指南](docs/setup/README.zh-CN.md) · [API 参考](docs/api.zh-CN.md) · [更新日志](CHANGELOG.zh-CN.md) · [贡献指南](CONTRIBUTING.md)

<!-- hardware-gallery:start -->

<table>
<tr><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/unitree_g1.png" width="100" alt="Unitree G1"></a><br><sub>Unitree G1</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/unitree_h1.png" width="100" alt="Unitree H1"></a><br><sub>Unitree H1</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/unitree_h1_2.png" width="100" alt="Unitree H1-2"></a><br><sub>Unitree H1-2</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/unitree_h2.png" width="100" alt="Unitree H2"></a><br><sub>Unitree H2</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/fourier_gr1.png" width="100" alt="Fourier GR1"></a><br><sub>Fourier GR1</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/fourier_gr2.png" width="100" alt="Fourier GR2"></a><br><sub>Fourier GR2</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agibot_x1.png" width="100" alt="AgiBot X1"></a><br><sub>AgiBot X1</sub></td></tr>
<tr><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agibot_g1.png" width="100" alt="AgiBot G1"></a><br><sub>AgiBot G1</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agibot_g2.png" width="100" alt="AgiBot G2"></a><br><sub>AgiBot G2</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agilex_aloha.png" width="100" alt="AgileX ALOHA"></a><br><sub>AgileX ALOHA</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/franka_fr3.png" width="100" alt="Franka FR3"></a><br><sub>Franka FR3</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/ur_ur5e.png" width="100" alt="UR5e"></a><br><sub>UR5e</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agilex_piper.png" width="100" alt="AgileX PiPER"></a><br><sub>AgileX PiPER</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/widowx_250s.png" width="100" alt="WidowX 250S"></a><br><sub>WidowX 250S</sub></td></tr>
<tr><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/unitree_dex3.png" width="100" alt="Unitree Dex3-1"></a><br><sub>Unitree Dex3-1</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/inspire_rh56dfx.png" width="100" alt="Inspire RH56DFX"></a><br><sub>Inspire RH56DFX</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agibot_omnipicker.png" width="100" alt="AgiBot OmniPicker"></a><br><sub>AgiBot OmniPicker</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/robotiq_2f85.png" width="100" alt="Robotiq 2F-85"></a><br><sub>Robotiq 2F-85</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/agilex_piper_gripper.png" width="100" alt="AgileX PiPER gripper"></a><br><sub>AgileX PiPER gripper</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/realsense_d435i.png" width="100" alt="RealSense D435i"></a><br><sub>RealSense D435i</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/realsense_d457.png" width="100" alt="RealSense D457"></a><br><sub>RealSense D457</sub></td></tr>
<tr><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/realsense_d405.png" width="100" alt="RealSense D405"></a><br><sub>RealSense D405</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/realsense_l515.png" width="100" alt="RealSense L515"></a><br><sub>RealSense L515</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/stereolabs_zed.png" width="100" alt="ZED 2i / ZED Mini"></a><br><sub>ZED 2i / ZED Mini</sub></td><td align="center"><a href="docs/supported_robots.zh-CN.md"><img src="assets/hardware/orbbec_dabai_dc1.png" width="100" alt="Orbbec Dabai DC1"></a><br><sub>Orbbec Dabai DC1</sub></td></tr>
</table>

<!-- hardware-gallery:end -->

</div>

<!-- toc -->

<details>
<summary><b>目录</b></summary>

- [简介](#简介)
- [支持的机器人](#支持的机器人)
- [快速开始](#快速开始)
  - [仿真](#仿真)
  - [真机：Unitree G1](#真机unitree-g1)
  - [多台机器人作为一个](#多台机器人作为一个)
- [架构](#架构)
- [仓库结构](#仓库结构)
- [添加新机器人](#添加新机器人)
- [联系方式](#联系方式)
- [引用](#引用)
- [许可证](#许可证)

</details>

<!-- /toc -->

## 简介

hiRobot 为模型和 agent 提供一种统一、一致的方式来观测和控制机器人（**人形或机械臂，仿真或真机**），
不论底层是哪家厂商的 SDK。它统一了 embodiment（本体描述）的输入输出（关节、可选的浮动基座、
可选的末端执行器、摄像头、控制频率）和机器人驱动的接口约定，所以控制代码写一次，就能在不同的
机器人上运行，无需重写。

机器人种类繁多：带浮动基座、约 30 自由度的人形机器人（Unitree、Fourier GRx、AgiBot X1），
装在轮式底盘上的双臂（AgiBot G1），以及支持笛卡尔控制的固定基座机械臂（Franka、UR）。
把模型或 agent 接到每一台机器人上，通常要为每种机器人、每个 SDK 单独写胶水代码。
hiRobot 把这层胶水统一起来：

- **一个接口，多种机器人**：针对 `Robot` 接口约定编写控制逻辑，就能在任何提供了驱动的机器人上运行。
  每台机器人的关节（按名称）、基座和末端执行器都在配置中描述，所以同一个 policy 可以面向不同的机器人。
  一个 `assembly:` 配置把几台完整的机器人（来自不同厂商，各自带有自己的手、安全门和全身控制）加上场景摄像头
  当作一个 `Robot` 来驱动，每台放在共享坐标系中，所以双臂 VLA 或 agent 无需新代码就能控制它们。
- **仿真与真机对称**：仿真和硬件只是同一接口约定背后的不同驱动，由配置中的一个字段选择。
  MuJoCo 机器人可以单独运行，无需硬件即可调试；也可以从真机实测的姿态起步，在动作发出之前先预演它。
  仿真基准环境以驱动的形式接入同一接口约定，所以在仿真中评测过的代码无需修改就能在机器人上运行。
- **与 VLA 无关**：无论是哪个模型，VLA 读取和产出的都是 hiRobot 的 `Observation` / `ActionChunk` 类型。
  把一个 VLA 换成另一个（GR00T、π0.5、StarVLA），改变的是 hiAgent 指向的 hiVLA 服务，而不是机器人代码。
- **对 agent 友好**：接口便于 agent 作为工具调用：`Robot.action_modes` 列出驱动接受的控制模式，
  `get_status` 报告电量、温度和关节故障，`list_skills()` 列出机器人能按名字做的事（行走、手势），每个都附带说明和参数的 JSON Schema，
  `SafetyGate` 检查每一条命令，所以被拒绝的动作会抛出 `SafetyError`，而不会到达电机。
  [docs/api.zh-CN.md](docs/api.zh-CN.md) 记录了每个调用的参数和失败方式。
- **分步部署文档**：真机部署涉及许多新手猜不到的细节，所以每台支持的机器人都有一份设置指南，
  涵盖 SDK 安装、网络设置、上电与安全、连接参数、第一个控制循环和常见的坑。
- **解耦设计**：每个部件（本体、末端执行器、摄像头、音频）都有自己的接口约定和注册表，每个驱动都放在自己的
  `drivers/<part>/<SDK vendor>/<SDK or protocol>/` 目录中，附带所需 SDK 的 manifest、一个假 SDK
  和一个共享的接口约定测试。机器人配置是一棵 `assembly:` 树，每个设备引用一份设备文件，全身控制等控制器
  则从各自的包以 `layers:` 的形式接入。在已支持的 SDK 上新增型号只需要配置；新增 SDK 只需要
  一个驱动目录及其测试，核心无需改动。

> [!NOTE]
> **配套项目。** [hiWBC](https://github.com/humanoidintelligence/hiWBC)
> 在机器人接口之下负责全身平衡与行走，
> [hiVLA](https://github.com/humanoidintelligence/hiVLA)（运动皮层）通过网络部署 VLA 模型，
> [hiAgent](https://github.com/humanoidintelligence/hiAgent)（agent 操作层）从上方驱动机器人。
> 下文以它们为例；hiRobot 可以单独工作，不 import 其中任何一个。

> [!WARNING]
> **状态：早期开发中。** API 尚不稳定，随时可能变动。
>
> **真机安全。** 受软件控制的机器人可能伤人，也可能损坏自身。在硬件上运行之前，请仔细阅读本仓库，
> 包括你的机器人的[设置指南](docs/setup/README.zh-CN.md)。
> 先在仿真中调试到能用，再一步步转到真机：先只读，再把机器人吊在龙门架上用低增益运行，
> 急停放在手边，机器人够得着的范围内不站人。

## 支持的机器人

真机（本体、移动底盘、末端执行器、摄像头）及其配置、驱动、厂商 SDK、设置指南和真机验证情况，
以及仿真机器人，见 [docs/supported_robots.zh-CN.md](docs/supported_robots.zh-CN.md)。

## 快速开始

核心安装很轻（numpy、pydantic、pyyaml）；extras 补上特定驱动需要的依赖：

```bash
git clone https://github.com/humanoidintelligence/hiRobot.git && cd hiRobot
pip install -e .                 # 核心：接口、toy 驱动、安全门
# extras：sim（MuJoCo 机器人）· camera · video · audio · dev
```

查看一份配置描述的 embodiment（不需要机器人，不需要 GPU）：

```bash
hirobot info --config unitree_g1   # 人形机器人
hirobot info --config ur_ur5e      # 固定基座机械臂
```

### 仿真

先在 MuJoCo G1 上调试控制代码。安装 extra 并获取 G1 模型
（[命令](docs/setup/unitree.zh-CN.md#5-mujocog1可选)），然后在查看器中打开：

```bash
pip install -e '.[sim]'
hirobot sim unitree_g1_mujoco --viewer
```

基本的控制循环：读取观测，发送动作，最后保持。下面这段在 2 秒内把右肘弯到 1 rad：

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("unitree_g1_mujoco")   # 骨盆固定，就像吊在龙门架上的机器人
robot.connect()
robot.reset()
elbow = robot.embodiment.joint_names.index("right_elbow")

for step in range(100):                   # control_hz = 50 时为 2 秒
    obs = robot.get_observation()         # joint_pos、基座 IMU、images["head"]
    target = obs.joint_pos.copy()
    target[elbow] = min(target[elbow] + 0.01, 1.0)
    robot.send_action(Action(mode=ControlMode.POSITION, joint_target=target))
robot.hold()
robot.disconnect()
```

仿真器每次 `send_action` 推进一步，所以循环不需要计时器；在真机上，要按 `robot.embodiment.control_hz`
控制节奏。只要换掉配置，同样的代码就能在真实的 G1 上运行。

### 真机：Unitree G1

从全新克隆到 G1 上的第一个动作：把右前臂抬到水平再放回。完整流程（包括每一步要检查什么，以及一次
真机运行的结果）见 [Unitree 设置指南](docs/setup/unitree.zh-CN.md#6-第一次运动逐步进行g1-肘关节)。

**1. 安装 SDK 并连接。** 安装
[unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)，
用网线连接机器人，把主机网卡设在 `192.168.123.x`
（[网络](docs/setup/unitree.zh-CN.md#2-网络)）。第 4 步的预演还要
`pip install -e '.[sim]'` 并获取 G1 模型
（[MuJoCo 设置](docs/setup/unitree.zh-CN.md#5-mujocog1可选)）。

**2. 写一份本地配置。** 以 `defaults: [unitree_g1]` 开头，这样关节、摄像头和音频都来自内置配置，
然后覆盖 `body` 部件的驱动（`type: unitree_sdk2`、网卡、每个关节的增益）。
[`examples/unitree_g1_real.yaml`](examples/unitree_g1_real.yaml) 里有可用的 G1 增益；
[`examples/`](examples/README.md) 里还有只读版本。

**3. 读取机器人（只读）。** 关节角、IMU、温度，以及是否有内置运动服务占用了命令 topic：

```bash
python scripts/unitree_setup.py status --config my_g1.yaml --iface eth0
python scripts/unitree_estop.py --iface eth0     # 第二个终端：软件急停
```

**4. 在仿真中预演（只读）。** 真机以只读方式打开，读取一次姿态，动作在从这个姿态起步的
`unitree_g1_mujoco` 上运行。不向机器人发送任何内容：

```bash
python examples/rehearse_in_sim.py my_g1.yaml unitree_g1_mujoco \
    --body network_interface=eth0 --joint right_elbow --target 0.0
```

**5. 在机器人上运行。** 把机器人吊起来，双脚离地，急停握在手里。前几次运行用低增益；
`examples/unitree_g1_real.yaml` 里的增益就是低增益：

```bash
python scripts/unitree_setup.py move --config my_g1.yaml --iface eth0 \
    --joint right_elbow --target 0.0 --ramp 3 --release-motion-mode
```

任何关节偏离超过 0.3 rad，这个动作就会中止并进入阻尼，而且结束时总是进入阻尼。`restore`
把控制权交还给内置服务。

其他机器人请参考各自的指南：[总览](docs/setup/README.zh-CN.md) ·
[Fourier GR1](docs/setup/fourier.zh-CN.md) ·
[AgiBot G1](docs/setup/agibot_g1.zh-CN.md) · [AgiBot X1](docs/setup/agibot_x1.zh-CN.md) · [Franka FR3](docs/setup/franka.zh-CN.md) ·
[Universal Robots UR5e](docs/setup/ur.zh-CN.md) · [AgileX PiPER](docs/setup/agilex_piper.zh-CN.md)。

### 多台机器人作为一个

一个有多个部件的 `assembly:` 配置会构建一个把它们全部驱动的 `Robot`：每个部件都是一份完整的单机器人配置，
而 assembly 本身也是一个 `Robot`，所以上面的循环无需改动就能驱动它。内置的 `franka_ur5e_dual` 在 toy 驱动上运行：

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("franka_ur5e_dual")          # Franka FR3 + UR5e + 一个场景摄像头
robot.connect()
robot.reset()
obs = robot.get_observation()                    # 关节拼接在一起：left/..., right/...
left = robot.part("left").get_observation()      # 单个部件，使用它自己的名称
robot.send_action(Action(parts={                 # 每个部件一个子动作，各用各的模式
    "left": Action(mode=ControlMode.POSITION, joint_target=left.joint_pos + 0.1),
}))                                              # 没有提到 "right"，它保持原目标
robot.disconnect()
```

在硬件上，每台臂的驱动和地址写成它本体的覆盖（`left/body`、`right/body`）
（[`examples/franka_ur5e_dual_real.yaml`](examples/franka_ur5e_dual_real.yaml)）。
动作的每个部分都先经安全门检查，全部通过后才发送；任何一个部件失败，整台机器人都会进入保持。
扁平动作、部件摆放、世界坐标系和命名规则见 [docs/api.zh-CN.md](docs/api.zh-CN.md#robotassembly)。

## 架构

![hiRobot 架构。hiRobot 内部：make_robot 把配置读成 Embodiment，并构建一个栈，从外到内依次是：Robot 接口约定（Observation / Action）、命令仲裁器（租约、优先级、包络、审计）、可选的 layer、layer 下面的 SafetyGate（限位、步长、电量、温度、仿真检查），再到 RobotAssembly 树（每个节点一台机器人，把本体和手、摄像头、音频合成一台；带子部件的节点把多台机器人当作一台来驱动，下发前检查每一部分、可选碰撞检查、出错全停）直到驱动；estop() 先到驱动，再到每个 layer（真机、仿真器、手、摄像头、音频），这些驱动与厂商 SDK 和仿真器通信。旁边是工具：DofMap、Kinematics、transforms。外围：上方是 hiAgent 和 hiVLA，通过网络交换观测和 action chunk，动作向下发给 Robot，观测向上返回；hiWBC 以 layer 的形式接入。所有配套项目都是可选的。](assets/architecture.png)

hiRobot 负责机器人的输入输出。`make_robot` 读取一份配置（一棵设备文件组成的树，变成一个 `Embodiment`），
构建一个由 `Robot` 组成的栈。从下往上：驱动和厂商 SDK 或仿真器通信；一个 `RobotAssembly` 把本体和它的手、
摄像头、音频合成一台；硬件 `SafetyGate` 挂在这个 assembly 上；可选的 layer（用 `register_layer` 注册的插件，
例如 hiWBC）包在外面；最外层有一个命令仲裁器，决定谁能给这台机器人发命令。`RobotAssembly` 也可以带子部件，
把多台这样的机器人当作一台来驱动：整棵树是一个 `Robot`，共用最外层的安全门和仲裁器，每个部件保留自己的
安全门、layer 和租约。每条命令都经由 `Robot.send_action` 发出，它在驱动接触硬件之前运行安全门，拒绝时抛出
`SafetyError`。硬件安全门在 layer 下面，所以 layer 算出来的关节目标也要过它，任何调用方都无法跳过。

安全门总会检查形状、DoF 数量和数值是否有限；可选地检查关节限位、离上一条目标的步长、离测量位置的距离、电量和
温度，以及在 MuJoCo 里把命令推演一小段（都在配置的 `safety:` 部分设置）。`estop()` 可以从任何线程调用，先直接
到达最底层的驱动，再让每个 layer 停下自己的工作。程序用租约 `robot.claim(name)` 持有命令流；更高的优先级
（遥操作接管）会抢占它，租约还可以带一套更严的安全包络。速度命令在 `command_timeout_s` 内没有刷新就会停下。
`hirobot view` 在栈外以只读方式打开真机，在 MuJoCo 里显示它的姿态。

配套项目从外部使用同一个 `Robot`。hiWBC 注册一个 layer（在配置中写 `layers: [{type: wbc, ...}]`），
它位于人形机器人的本体之上，每个控制周期把基座速度、motion clip 和 motion latent 转换成保持平衡的
关节目标；这个 layer 本身也是一个 `Robot`，所以它上面的调用方无需改动。hiAgent 运行控制循环，
并在同进程内调用 `Robot`：读取一个 `Observation`，发给 hiVLA 部署的模型，再把返回的 action chunk
逐个 `Action` 发给机器人。

核心概念：

| 概念 | 职责 |
| --- | --- |
| `Embodiment` | 机器人在 policy 眼中的样子：关节（按名称，按 SDK 的顺序）、`floating_base`、末端执行器、摄像头、麦克风 / 扬声器、`control_hz`。由配置构建（一份设备文件或一棵 `assembly:` 树）。 |
| `Observation` / `Action` / `ActionChunk` | 类型化的输入输出。关节空间（通用）+ 可选的基座（人形机器人）+ 可选的任务空间（机械臂）+ 图像。`ControlMode` 选择关节、笛卡尔、基座速度或运动目标；一个 `ActionChunk` 包含多个预测步。 |
| `Robot` | 每个驱动、仿真器和包装器都实现的接口约定：`connect` / `reset` / `get_observation` / `send_action` / `hold` / `set_gains` / `get_status` / `disconnect`，加上 `action_modes`（它接受的模式）和技能 `list_skills` / `run_skill` / `stop_skill`（由机器人控制器或某一层执行、按名字启动的动作）。驱动实现 `_send_action`；`send_action` 先运行安全门。[详情](docs/api.zh-CN.md#robot)。 |
| `SafetyGate` / `SafetyError` | 在 `send_action` 内检查每个动作：形状、DoF 数量和数值是否有限总会检查；配置了时还检查关节限位、步长和跟踪误差上限、电量和温度，以及 MuJoCo 仿真检查。拒绝时抛出 `SafetyError`（`clamp: true` 时改为截断）。由配置的 `safety:` 部分设置。[详情](docs/api.zh-CN.md#safetygate)。 |
| `CommandArbiter` / `CommandLease` | 命令仲裁器和租约。`robot.claim(name, priority)` 让一个程序持有命令流；其他调用方得到 `RobotBusyError`，更高的优先级（`TAKEOVER` > `LAYER_SKILL` > `CALLER`）会抢占。租约可以带一套更严的 `envelope`；claim、抢占和被拒的命令记在审计里（`robot.command_events()`）。没人持有租约时谁都能发命令。[详情](docs/api.zh-CN.md#命令仲裁)。 |
| `Layer` | layer 的基类：包住下面栈的一个 `Robot`（`inner`，只有公开 API）。可以提供层技能（`run_layer_skill`）。[写一个 layer](docs/api.zh-CN.md#编写层)。 |
| `make_robot` / 注册表 | 根据配置名或路径构建栈。驱动和 layer 按名称在注册表中查找（`register_body`、`register_end_effector`、`register_camera`、`register_audio`、`register_layer`）；其他包通过 `hirobot.drivers` entry point 添加自己的驱动。 |
| `RobotAssembly` | `make_robot` 根据配置构建的节点：把设备（摄像头、手、音频）挂接到本体上；带子部件时，把几台完整的机器人当作一个 `Robot` 来驱动，处于共享坐标系中。[详情](docs/api.zh-CN.md#robotassembly)。 |
| `DofMap` / `Kinematics` | 在不同布局之间按名称重排关节向量；MJCF 模型的正向运动学。 |

## 仓库结构

```
hiRobot/
├── src/hirobot/
│   ├── __init__.py          # 公开 API：make_robot、Robot、Observation、Action……
│   ├── cli.py               # `hirobot info`、`configs`、`sim`、`view`
│   ├── testing.py           # assert_layer_uses_public_api，给 layer 包用
│   ├── core/                # embodiment、types、robot + robot_skills、dof、stamps
│   ├── devices/             # end_effector、camera、audio：接口约定和注册表
│   ├── geometry/            # transforms、kinematics、collision
│   ├── loading/             # config、tree、layout、legacy、build、registry、sdk
│   ├── assembly/            # RobotAssembly + parts、node、part_layers
│   ├── control/             # safety（安全门）、arbiter、layer、watchdog
│   ├── sim/                 # sim_world、sim_check、check_sim、view
│   ├── configs/             # 内置配置（随包安装）
│   │   ├── body/ end_effector/ camera/ microphone/ speaker/   # 每个型号一份设备文件，按厂商分目录
│   │   └── robots/          # 整台机器人和多机 assembly（franka_ur5e_dual）；*_mujoco 加 sim: 段
│   └── drivers/             # <part>/<SDK vendor>/<SDK or protocol>/{driver.py, manifest.yaml}
│       ├── body/            # unitree/sdk2, fourier/grx, agibot/aimrt, franka/droid, ur/rtde, agilex/piper_sdk
│       ├── end_effector/    # 手：unitree/sdk2_handcmd (Dex3-1), unitree/sdk2_motorcmds, inspire/sdkpy
│       ├── camera/          # unitree/sdk2_videohub, unitree/image_server; generic/{video_file,array}
│       ├── audio/           # unitree/sdk2_voice; generic/{wav,sounddevice}
│       ├── sim/             # toy/（`toy`）、mujoco/（`mujoco`）
│       └── _sdk/            # 同一 SDK 内部共享的代码（Unitree DDS 初始化，每个进程一次）
├── examples/                # 可复制的配置：真机、read_only、多机；rehearse_in_sim.py
├── scripts/
│   ├── unitree_setup.py          # Unitree 机器人上的 status / hold / move / restore
│   ├── unitree_estop.py          # 软件急停（Ctrl-C 时进入阻尼）
│   ├── dex3_setup.py             # G1 Dex3 手的 status / wiggle（从不控制本体）
│   ├── inspire_serial_to_dds.py  # H1 Inspire 手：RS485 串口 <-> DDS 桥接，运行在 PC2 上
│   ├── inspire_hand_demo.py      # 缓慢张开 Inspire 手，然后复位（不动本体）
│   └── camera_image_server.py    # 通过 ZeroMQ 以 JPEG 发布摄像头画面（image_server 格式）
├── docs/
│   ├── api.md               # 公开 API 参考（中文：api.zh-CN.md）
│   ├── supported_robots.md  # 真机和仿真机器人：图片、配置、驱动、真机验证
│   └── setup/               # 各机器人的硬件设置指南（中文：<robot>.zh-CN.md）
├── tests/                   # pytest 测试集；仿真器和 SDK 都是假的，不需要 GPU
│   ├── drivers/             # 与 src/hirobot/drivers/ 对应：每个 SDK 一个 fake_sdk.py + test_contract.py
│   └── data/                # expanded_configs.json：每个内置机器人配置的快照
├── assets/                  # README 和文档图片；hardware/：机器人照片
└── CHANGELOG.md             # 每个版本的变化（中文：CHANGELOG.zh-CN.md）
```

每个版本改了什么见 [CHANGELOG.zh-CN.md](CHANGELOG.zh-CN.md)。

## 添加新机器人

> [!TIP]
> **欢迎提交 pull request。** 新的机器人、手、摄像头或 SDK 版本，对尚未标 ✅ 的驱动做真机检查，
> 或修正一份设置指南，每一项都让机器人控制离一个共享的接口更近一步。提交 pull request 之前，
> 请阅读 [CONTRIBUTING.md](CONTRIBUTING.md)：每类改动需要什么、真机检查和审阅人。

对一台机器人的支持分两层：**配置**（它的关节布局和部件）和**驱动**（与硬件的连接）。代码按照与硬件
通信的方式组织：`drivers/<part>/<SDK vendor>/<SDK or protocol>/`，以 SDK 命名，而不是以产品命名。
型号放在配置中：`configs/<part>/<vendor>/<model>.yaml` 指明使用哪个驱动。一个 SDK 通常覆盖许多型号，
所以大多数新机器人只需要配置。

| 情况 | 需要添加什么 | 示例 |
|---|---|---|
| 新型号，SDK 和协议相同 | 只需配置：`body/<vendor>/<model>.yaml` 和 `robots/<name>.yaml` | Unitree H2D：`body/unitree/h2d.yaml` |
| 已支持 SDK 的新版本 | 在硬件上跑通后，在驱动的 `manifest.yaml` 中加一条 `verified` 记录；如果行为有变化，加一个假 SDK 测试 | G1 + Dex3 运行在 `unitree_sdk2py` 1.1 上 |
| 新的 SDK 或协议 | 一个新的 SDK 目录，包含 `driver.py`、`manifest.yaml`、一个新的注册名，以及 `tests/drivers/.../{fake_sdk,test_contract}.py` | Fourier Aurora、AgiBot OmniHand |
| SDK 代际断裂（消息布局、控制或增益语义改变） | 同一厂商下的新目录（`sdk3/`）和新名称（`unitree_sdk3`） | |

注册名的格式是 `<vendor>_<SDK or protocol>`（`unitree_sdk2`、`fourier_grx`）。
已发布的名称永远不改变含义，所以更新 hiRobot 不会悄悄替换真机运行的驱动。

**配置。** 在 `src/hirobot/configs/` 下为每个型号写一份设备文件（`body/`、`end_effector/`、`camera/`、
`microphone/`、`speaker/`）：`body` 文件按 SDK 的顺序按名称列出关节，并设置 `floating_base`、`control_hz`
和它的 `driver`。在 `robots/<name>.yaml` 中用 `assembly:` 把这些文件放进 `parts:` 引用、用 `placement:` 摆放，
组合出整台机器人。用 `hirobot info --config <name> --resolved` 检查结果。

**驱动。** 继承 `Robot`（或 `EndEffectorDriver`、`CameraDriver`、`MicrophoneDriver` / `SpeakerDriver`），
把 `Observation` / `Action` 映射到 SDK 的读取和命令上，并注册它（`@register_body("<vendor>_<sdk>")`）。
在 `connect()` 内、`require_sdk(__package__)` 之后再 import SDK，这样没有 SDK 时 `hirobot` 也能 import，
SDK 缺失或版本过旧时会给出明确的错误。`manifest.yaml` 列出 SDK（`distribution`、`minimum`、`install`），
一开始写 `verified: []`。`base_orientation` 按 `[x, y, z, w]` 报告。

**测试。** 在 `tests/drivers/<part>/<vendor>/<sdk>/fake_sdk.py` 中为驱动用到的 SDK 调用写一个假实现，
并在 `test_contract.py` 中用 `tests/drivers/contract.py` 里的共享检查（`check_robot`、`check_hand`、
`check_camera`、`check_microphone`、`check_speaker`）测试它。这些检查覆盖注册表、manifest、
连接 / 读取 / 控制 / 断开、每个关节的形状、四元数和安全门。只要有已注册的驱动没有测试，
`tests/drivers/test_coverage.py` 就会失败。硬件运行无法放进 CI：把它们记录为 `verified` 条目
（型号、SDK 版本、日期）。

**仓库之外。** 一个独立的包可以通过 `hirobot.drivers` entry-point 组注册驱动，并按自己 SDK 的节奏发布；
用户的配置保持不变。

## 联系方式

- [Peng Li](https://artpli.github.io): pengli.ds [at] gmail.com

## 引用

如果你在研究中使用了 hiRobot，请引用：

```bibtex
@misc{hirobot,
  title  = {hiRobot: A Unified Interface Layer for Controlling Diverse Robots from VLA Models and Agents},
  author = {Li, Peng and Zhang, Yifei and Zhuang, Zihan and Dong, Yi and Gui, Tao and Qiu, Xipeng},
  year   = {2026},
  url    = {https://github.com/humanoidintelligence/hiRobot}
}
```

## 许可证

[Apache License 2.0](LICENSE)。完整条款见 [LICENSE](LICENSE)。
