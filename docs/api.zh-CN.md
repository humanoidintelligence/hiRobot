# hiRobot API 参考

[English](./api.md) | 简体中文

本页列出 hiRobot 对用户开放的接口：从 `hirobot` 导入的内容、配置文件可以包含的内容、CLI，以及
添加驱动、相机或末端执行器（灵巧手、夹爪）的扩展点。[顶层包](#顶层包)下列出的所有内容都可以用
`from hirobot import ...` 导入。

全文约定：

- 数组是 `numpy.ndarray`。角度单位为 rad，角速度为 rad/s，力矩为 Nm。
- 关节向量（`joint_pos`、`joint_target`、`kp`……）按 `Embodiment.joints` 的顺序排列。
  需要其他顺序时用 [`DofMap`](#dofmap)。
- 四元数为 `[x, y, z, w]`。位姿是 7 维向量 `[x, y, z, qx, qy, qz, qw]`。

## 目录

- [Robot 一览](#robot-一览)
- [快速示例](#快速示例)
- [顶层包](#顶层包)
- [构建机器人：make_robot 与驱动注册表](#构建机器人make_robot-与驱动注册表)
- [Robot](#robot)
- [Embodiment](#embodiment)
- [Observation, Action, ActionChunk, ControlMode](#observation-action-actionchunk-controlmode)
- [技能](#技能)
- [机器人 skill](#机器人-skill)
- [命令仲裁](#命令仲裁)
- [SafetyGate](#safetygate)
- [RobotAssembly](#robotassembly)
- [控制循环](#控制循环)
- [DofMap](#dofmap)
- [Kinematics](#kinematics)
- [配置文件参考](#配置文件参考)
- [CLI](#cli)
- [编写驱动和层（SPI）](#编写驱动和层spi)
- [扩展点：相机、末端执行器与音频](#扩展点相机末端执行器与音频)

## Robot 一览

`Robot` 的全部成员，按用途分组。驱动、仿真器、assembly 和 layer 都有下面每一个成员；签名和行为见
[Robot](#robot)。

| 用途 | 成员 | 见 |
|---|---|---|
| 机器人是什么 | `embodiment`、`action_modes`、`motion_spaces`、`direct_joints`、`cartesian_targets`、`accepts_parts`、`simulated` | [Embodiment](#embodiment)、[ControlMode](#controlmode) |
| 挂载的设备 | `cameras`、`microphones`、`speakers`（按名字） | [扩展点](#扩展点相机末端执行器与音频) |
| 连接和复位 | `connect()`、`reset(seed=None)`、`disconnect()` | [Robot](#robot) |
| 读 | `get_observation() -> Observation`、`get_status() -> RobotStatus`、`now()`、`fault` | [Observation](#observation)、[RobotStatus](#robotstatus) |
| 发命令 | `send_action(action)`、`check_action(action) -> Action`、`hold()`、`set_gains(kp, kd)`、`command_timeout_s` | [Action](#action) |
| 急停 | `estop()` | [Robot](#robot) |
| 共用命令流 | `claim(name, ...) -> CommandLease`、`command_events()` | [命令仲裁](#命令仲裁) |
| 技能 | `list_skills()`、`run_skill(name, args=None)`、`stop_skill()`、`skill_running` | [技能](#技能) |
| 按类别的技能 | 机器人 skill：`has_robot_skills`、`list_robot_skills()`、`run_robot_skill(name, args=None)`、`stop_robot_skill()`。层技能：`list_layer_skills()`、`run_layer_skill(name, args=None)`、`stop_layer_skill()`、`layer_skill_running` | [机器人 skill](#机器人-skill)、[Layer](#layer) |
| 安全和回合结果 | `safety`（挂上的 `SafetyGate`）、`success`、`done`、`reward` | [SafetyGate](#safetygate)、[控制循环](#控制循环) |

## 快速示例

```python
import numpy as np
from hirobot import Action, ControlMode, make_robot

robot = make_robot("examples/humanoid.yaml")  # from the repository root; body.driver: toy
robot.connect()
try:
    robot.reset()
    for _ in range(100):                             # hold every joint at zero
        obs = robot.get_observation()
        robot.send_action(Action(mode=ControlMode.POSITION,
                                 joint_target=np.zeros(robot.embodiment.num_dofs)))
finally:
    robot.disconnect()
```

## 顶层包

| 名字 | 类别 | 模块 | 用途 |
|---|---|---|---|
| `Embodiment`, `Joint`, `EndEffector`, `EndEffectorFrame`, `Camera`, `CameraMount`, `AudioDevice`, `EmbodimentKind` | pydantic 模型 / 枚举 | `hirobot.core.embodiment` | 声明式的机器人 I/O 契约 |
| `Observation`, `Action`, `ActionChunk`, `ControlMode`, `MotionClip`, `JOINT_MODES`, `MOTION_SPACES` | pydantic 模型 / 枚举 / 元组 | `hirobot.core.types` | 机器人与 policy 之间的类型化 I/O |
| `RobotStatus`, `BatteryState`, `CommandProgress` | pydantic 模型 | `hirobot.core.types` | `get_status()` 的返回值（见 [`RobotStatus`](#robotstatus)） |
| `Robot` | 抽象类 | `hirobot.core.robot` | 驱动契约 |
| `RobotAssembly` | `Robot` | `hirobot.assembly` | 本体驱动加上挂接的相机、末端执行器（灵巧手、夹爪）、麦克风和扬声器；带子部件时，把几台完整的机器人当作一台驱动（见 [RobotAssembly](#robotassembly)） |
| `Layer` | `Robot` | `hirobot.control.layer` | layer 的基类：包装 `inner` 并默认转发给它（见 [Layer](#layer)） |
| `RobotFaultError` | 异常 | `hirobot.core.robot` | 机器人处于故障状态时由 `send_action` 抛出 |
| `RobotStoppedError` | 异常 | `hirobot.core.robot` | `RobotFaultError` 的子类：长调用（`connect`、`reset`）因 `estop()` 而放弃 |
| `register_body`, `get_body`, `available_bodies` | 函数 | `hirobot.loading.registry` | 按名字注册的驱动注册表 |
| `register_layer`, `get_layer`, `available_layers` | 函数 | `hirobot.loading.registry` | 由 `layers:` 叠加的 layer 的注册表（见 [Layer](#layer)） |
| `load_embodiment`, `make_robot` | 函数 | `hirobot` | 从配置名、文件或 dict 构建 embodiment / 机器人 |
| `require_sdk` | 函数 | `hirobot.loading.sdk` | 按驱动的 `manifest.yaml` 检查其厂商 SDK |
| `RobotSkillSpec`, `RobotSkillResult`, `RobotSkillError` | pydantic 模型 / 异常 | `hirobot.core.robot_skills` | 描述、启动和拒绝机器人 skill（行走、手势） |
| `SafetyGate`, `SafetyVerdict`, `SafetyError` | 类 / dataclass / 异常 | `hirobot.control.safety` | 在每个动作到达硬件之前检查它 |
| `DofMap` | 类 | `hirobot.core.dof` | 按名字重排关节向量 |
| `pose_compose`, `pose_inverse`, `pose_from_matrix`, `pose_from_axis_angle`, ... | 函数 | `hirobot.geometry.transforms` | 7 维向量位姿 `[x, y, z, qx, qy, qz, qw]` 的位姿运算（不从 `hirobot` 重新导出） |
| `Kinematics`, `BodyPoses` | 类 | `hirobot.geometry.kinematics` | MJCF 模型的正运动学（见 [Kinematics](#kinematics)；不从 `hirobot` 重新导出） |
| `__version__` | str | `hirobot` | 包版本 |

相机、末端执行器和音频的接口与注册表在 `hirobot.devices.camera`、`hirobot.devices.end_effector` 和 `hirobot.devices.audio` 中；
驱动本身在 `hirobot.drivers` 下（`drivers/<part>/<SDK vendor>/<SDK or protocol>/`）。两者分别在
[构建机器人](#构建机器人make_robot-与驱动注册表)和[扩展点](#扩展点相机末端执行器与音频)中介绍。

## 构建机器人：make_robot 与驱动注册表

### `load_embodiment(config) -> Embodiment`

一份机器人配置（`assembly:`）或设备文件（`device:`）描述的 `Embodiment`：可以是内置名字
（`"unitree_g1"`，等同于 `"robots/unitree_g1"`）、文件路径或已加载的 dict，在展开 `defaults:` 之后
（见[配置文件参考](#配置文件参考)）。对带部件的 assembly，embodiment 是合并后的整棵树，每个部件的名字都在它的
`<part>/` 前缀下。不连接、不构建任何东西。配置两者都不是时抛出 `ValueError`。

### `make_robot(config, overrides=None, sim=None) -> Robot`

构建配置描述的机器人。`config` 是内置名字、文件路径或 dict：一份机器人配置（`assembly:`）或单独一份设备
文件（`device:`）。配置被加载并按树构建，每个节点一台机器人，驱动在注册表中查找（内置驱动和 `hirobot.drivers`
entry-point 插件）。按顺序执行以下步骤：

1. 展开 `defaults:` 并加载树。每个节点自己的设备（`body` 及其手、相机、音频）组成它自己的机器人；
   每个设备的 `driver:` 在注册表中查找（`body` 默认 `toy`）。
2. 本体为真机、且有末端执行器带 `driver:` 时，为每个这样的末端执行器构建一个末端执行器驱动，并只把躯体关节
   （不含这些驱动自己管的夹爪）交给本体驱动。以 `driver_cls(embodiment, **driver_kwargs)` 实例化本体；
   `driver:` 的其他键都作为构造参数传入。
3. 为每个带 `driver:` 的相机构建相机驱动，为每个带 `driver:` 的麦克风 / 扬声器构建音频驱动。本体是仿真时，
   只构建替身（`simulated = True`，例如 `array`、`video_file`、`wav_mic`），所以配置不需要机器人也能跑。
4. 有设备可挂时，把一个节点的设备（本体、末端执行器、相机、音频）拼进一个 `RobotAssembly`。带子部件的节点
   成为一个持有这些部件（以及该节点自己的本体，如果有）的 `RobotAssembly`；节点可以嵌套。
5. 给根挂一个 `SafetyGate`，按关节的阈值从树中每个 `safety:` 合并而来（关节数和非有限值检查始终生效）；
   `safety: false` 会把它摘掉。
6. 如果设置了 `layers:`，按顺序用每个 layer 包装机器人（见 [Layer](#layer)）。带 `part: <path>` 的 layer
   只包装 assembly 的那个部件（`hirobot.assembly.part_layers`；它们位于整机 layer 之下）。layer 下面的安全门检查它们发出的
   关节指令；有 layer 时，最外层再加一个安全门检查调用方发送的内容。

`overrides` 把部件路径映射到合并进配置的 overrides，所以与主机相关的值（网络接口、IP、`read_only`）可以来自命令行：
单台机器人写 `{"body": {"driver": {"network_interface": "eth0"}}}`，assembly 的某个部件写
`{"left/body": {"driver": {"ip": ...}}}`。

`sim=True`（或配置带 `sim:` 段而 `sim` 留 `None`）改为在一个共享的 MuJoCo 世界里构建机器人
（`hirobot.sim.sim_world`）：每个本体来自它设备文件的 `model:`，放在它的 placement 上，用世界的视图作为它的驱动，
带挂载点的相机由世界渲染，不建任何真机驱动。`sim=False` 为带 `sim:` 段的配置构建真机。`sim` 也可以是一个
合并进配置 `sim:` 的映射（`{"realtime": False, "viewer": True}`）。

要在上真机之前先在仿真中预演一个动作，也可以用 `read_only: true` 打开真机，用它的 `*_mujoco` 配置另建一个机器人，
再用 `MujocoRobot.set_state` 让它从真机姿态起步（见 `examples/rehearse_in_sim.py`）。

以下情况抛出 `ValueError`：相机 / 末端执行器 / 音频 / layer 规格没有 `type`、末端执行器驱动不支持配置要求的接口、
`layers:` 不是列表，或扬声器驱动被列为麦克风（或反过来）。驱动名或 layer 名未知时抛出 `KeyError`。

### Layer

layer 是一个包装另一台机器人、改变其指挥方式的 `Robot`：例如一个全身控制器，接受 `BASE_VELOCITY` /
`MOTION_CLIP` / `CARTESIAN`，每个 tick 向下层机器人发送保持平衡的关节目标。hiRobot 本身不提供任何
layer；由插件包（hiWBC）注册。

```python
from hirobot import Layer, Robot, register_layer

@register_layer("my_wbc")
class MyWbc(Layer):
    def __init__(self, inner: Robot, **config):
        super().__init__(inner)
    def _send_action(self, action): ...   # compute, then self.inner.send_action(...)
```

`register_layer`、`Layer` 和它要实现的方法属于扩展接口，见[编写层](#编写层)；使用者只写 `layers:` 条目。

```yaml
layers:                     # first entry innermost
  - type: my_wbc            # registered layer name
    controller: sonic       # every other key -> the layer's constructor
```

`Layer` 默认把一切转给 `inner`：`embodiment`、能力声明（`action_modes`、`motion_spaces`、`cartesian_targets`、
`direct_joints`）、设备（`cameras`、`microphones`、`speakers`）、生命周期、观测、`get_status`、`hold`、增益和 robot
skills；`fault` 报告 layer 自己的故障或下面的故障。子类只覆盖自己改的部分。`estop()` 先到最底下的机器人，再让每一层停下自己的
工作。怎么写一个层见[编写层](#编写层)。

layer 通过 `inner.send_action` 发送关节指令，所以 `make_robot` 挂在下面的安全门会检查它们。layer 的 `inner` 是整机：
有真实灵巧手时，它可以自己驱动手指，也可以把手指留空（`direct_joints`），由调用方通过 `Action.joint_names` 指挥；
相机和音频设备也能像在下面的机器人上一样经它拿到。layer 与驱动一样通过
`hirobot.drivers` entry point 注册。

**层技能。** 层可以跑自己的技能（行为、宏动作），跨很多个控制周期，和厂商的 robot skills 分开：`list_layer_skills()`（本层和
下面各层的）、`run_layer_skill(name, args=None) -> RobotSkillResult`（参数按 schema 检查，启动后返回）、`stop_layer_skill()`、
`layer_skill_running`（本层或下面的层有技能在跑就为真）。这四个在每个 `Robot` 上都有，调用方不用先判断最外层是不是层。技能运行期间层以 `Priority.LAYER_SKILL` 持有栈的[命令租约](#命令仲裁)：调用方的租约被抢占，直接发命令抛
`RobotBusyError`，直到技能结束或调 `stop_layer_skill()`。`TAKEOVER` 会抢占技能（技能被停下）。`estop()` 会结束技能，调用方才能
`reset()`。

### 注册表

```python
from hirobot import Robot, register_body, get_body, available_bodies

@register_body("my_robot")
class MyRobot(Robot):
    ...
```

| 函数 | 说明 |
|---|---|
| `register_body(name)` | 类装饰器。名字已被占用或类不是 `Robot` 时抛出 `ValueError` |
| `get_body(name) -> Type[Robot]` | 抛出的 `KeyError` 会列出可用的名字 |
| `available_bodies() -> List[str]` | 排好序的已注册名字 |

内置驱动在导入 `hirobot.drivers` 时注册（`make_robot` 会导入它）。已安装的包可以通过 `hirobot.drivers`
entry-point 组添加驱动；它们与内置驱动一起加载。厂商 SDK 在 `connect()` 时才惰性导入，`connect()`
会先调用 `require_sdk`（见下文）。

| 名字 | 类 | 模块 | 目标 |
|---|---|---|---|
| `toy` | `ToyRobot` | `drivers/sim/toy/` | 玩具运动学积分器，无依赖 |
| `mujoco` | `MujocoRobot` | `drivers/sim/mujoco/` | MuJoCo 物理仿真，本身就是一台机器人；渲染带 `mount` 的相机；为带 `frame` 的末端执行器提供 `ee_poses`；`set_state` 让它从测得的姿态起步 |
| `unitree_sdk2` | `UnitreeRobot` | `drivers/body/unitree/sdk2/` | Unitree G1 / H1 / H1-2 / H2 |
| `fourier_grx` | `FourierRobot` | `drivers/body/fourier/grx/` | Fourier GR1T1 / GR1T2 |
| `agibot_gdk` | `AgibotG1Robot` | `drivers/body/agibot/gdk/` | AgiBot G1（精灵 G01）：手臂、腰、头和夹爪走 GDK 的 DDS topic；底盘走其控制盒 |
| `agibot_aimrt` | `AgibotX1Robot` | `drivers/body/agibot/aimrt/` | AgiBot X1 |
| `franka_droid` | `FrankaDroidRobot` | `drivers/body/franka/droid/` | 走 DROID 机器人服务（zerorpc）的 Franka FR3 和夹爪 |
| `ur_rtde` | `URRobot` | `drivers/body/ur/rtde/` | Universal Robots UR5e |
| `agilex_piper_sdk` | `PiperRobot` | `drivers/body/agilex/piper_sdk/` | AgileX PiPER 及其夹爪，走 SocketCAN（`piper_sdk`） |

每个硬件驱动的构造参数见其[设置指南](setup/README.zh-CN.md)。

### SDK 清单（`hirobot.loading.sdk`）

每个硬件驱动目录都有一个 `manifest.yaml`，写明它需要的厂商 SDK、可用的最低版本、安装命令，以及
哪个型号在哪个 SDK 版本上做过真机验证：

```yaml
sdk:
  - distribution: unitree_sdk2py    # importlib.metadata name
    module: unitree_sdk2py          # import name (default: distribution)
    minimum: "1.0.1"
    install: pip install -e <clone of https://github.com/unitreerobotics/unitree_sdk2_python>
verified:                           # append-only, one entry per model x SDK version
  - model: Unitree G1 (29 DoF)
    sdk: {unitree_sdk2py: "1.0.1 (a035ade)"}
    date: 2026-11-04
maintainers: []
```

`require_sdk(package, who=None)` 读取它：SDK 缺失时抛出带安装命令的 `ImportError`，版本低于 `minimum`
时抛出异常，版本比 `verified` 中所有版本都新时记录一次警告并照常运行。它返回
`{distribution: installed version}`。`load_manifest(package)` 返回解析后的 `Manifest`。

## Robot

`hirobot.core.robot.Robot`，每个机器人、仿真器和包装器都要实现的抽象驱动契约。

**线程。** `get_observation`、`get_status`、`now`、`estop` 任何时候都能从任何线程调。其余方法（`send_action`、`hold`、`reset`、
`run_skill`、`run_robot_skill`、`set_gains`、`connect`、`disconnect` 等）同一时刻只由一个持有者调：[命令租约](#命令仲裁)的持有者，没人持有时就是
唯一的调用方。租约可以在线程之间传，但调用不能重叠。两个线程同时发命令时，驱动不保证任何顺序。

| 成员 | 类别 | 说明 |
|---|---|---|
| `embodiment` | abstract property | 该驱动暴露的 `Embodiment` |
| `connect()` | abstract | 打开连接。在控制前调用一次 |
| `reset(seed=None)` | abstract | 让机器人进入安全的初始状态。仿真器用 `seed` 选择场景（例如环境 seed 或初始状态索引）；其他驱动忽略它 |
| `get_observation() -> Observation` | abstract | 读取传感器 |
| `get_status() -> RobotStatus` | optional | 健康状态和命令进度，见 [`RobotStatus`](#robotstatus)。默认只有 `timestamp` 和 `fault` |
| `send_action(action)` | concrete | 先 `check_action`，再调驱动的 `_send_action`（[SPI](#编写驱动和层spi)）。被拒绝时抛出 `SafetyError` 且不发送任何内容，`action.joint_names` 列出了 `direct_joints` 之外的关节时也一样。处于故障状态时抛出 `RobotFaultError`；别人持有机器人的[命令租约](#命令仲裁)时（例如它是某个 assembly 的部件而调用不经过 assembly）抛出 `RobotBusyError`（`RuntimeError` 子类） |
| `check_action(action) -> Action` | concrete | 执行 `send_action` 的全部检查但不发送；返回（可能被裁剪过的）动作。assembly 在发送给任何部件之前对每个部件那部分都运行它 |
| `fault` | property | 机器人拒绝指令的原因，或 `None`。驱动或 layer 自己进入阻尼或停止时设置，`estop()` 也会设置。`reset()` 返回时清除，期间来过 `estop()` 则不清除。包装器（`RobotAssembly`）也会报告内层机器人的故障。也见 `get_status().fault` |
| `estop()` | concrete | 立即停下；任何线程、任何时候都能调，不走安全闸，也不受[命令仲裁](#命令仲裁)限制。别的线程上的长调用（`reset` ramp、`connect` 里的等待）在下一步抛出 `RobotStoppedError`；机器人进入故障状态；正在下发的指令先发完（最多等 `ESTOP_LOCK_TIMEOUT_S`，0.2 s），之后不会再有指令发出；然后执行驱动自己的停止。Unitree 和 MuJoCo 进阻尼（Unitree 开着 `builtin_skills` 时改为停掉 skill），AgiBot G1 发 `MotionStop`，UR 减速停下（`servoStop` / `speedStop` / `stopJ`），Franka 停掉位置控制器；没有 `damp()` 的驱动（`toy`、`fourier_grx`、`agibot_aimrt`）只拒绝后续指令。`RobotAssembly` 对它的本体和每个末端执行器、或每个部件都调急停。在 layer 上调时，先直接急停栈最底下的机器人，再从外往里让每一层停下自己的工作（见[层的钩子](#层的钩子)、[Layer](#layer)） |
| `claim(name, priority=Priority.CALLER, on_preempt=None, envelope=None, parts=None) -> CommandLease` | concrete | 拿机器人的命令流。见[命令仲裁](#命令仲裁) |
| `command_events() -> List[CommandEvent]` | concrete | 仲裁器的审计事件，从旧到新；没有仲裁器时为空。见[审计](#审计) |
| `command_timeout_s` | attribute | 多少秒没有新的 `VELOCITY` / `BASE_VELOCITY` 指令就停下运动并打 warning（`hirobot.control.watchdog.CommandWatchdog`）；不进故障态，下一条指令照常执行。`toy`、`mujoco`（仿真时间）、`unitree_sdk2`（零速、kp 0）、`ur_rtde`（`speedStop`）默认 0.2 s；`agibot_gdk` 的底盘有自己的 `command_timeout`（0.3 s）；其他驱动是 `None`。配置里写 `body.command_timeout_s: null` 关掉。位置目标不看 |
| `action_modes` | class attribute | 机器人接受的模式。构建动作的调用方读取它 |
| `motion_spaces` | class attribute | 机器人在 `MOTION_CLIP` 下能跟踪的 `MotionClip.space` 取值。默认为空 |
| `direct_joints` | class attribute | 调用方可以在非关节模式旁按名字指挥的关节。默认为空 |
| `cartesian_targets` | property | 接受 `CARTESIAN` 位姿目标的末端执行器：`CARTESIAN` 在 `action_modes` 中时为全部末端执行器，否则为空。只跟踪其中一部分的机器人会覆盖它 |
| `cameras`、`microphones`、`speakers` | attributes | 挂载的设备，按名字：相机驱动、麦克风和扬声器。裸驱动上为空；`make_robot` 建的 assembly 会填上，每一层 layer 往上传，所以最外层也有。见[扩展点](#扩展点相机末端执行器与音频) |
| `accepts_parts` | class attribute | `send_action` 是否接受 `Action.parts`：带子部件的 assembly 为 `True`，其余为 `False` |
| `now()` | optional | 机器人时钟（秒），`Observation.stamps` 的时间在它上面：默认 `time.monotonic()`，`toy` / `mujoco` 是仿真时间。包装层返回内层机器人的 |
| `hold()` | optional | 让机器人保持在原处，例如在 policy 停止之后。默认情况下，驱动接受 POSITION 时把当前 `joint_pos` 作为 POSITION 动作发送，否则什么都不做。只接受笛卡尔增量的仿真器什么都不做；带 `builtin_skills` 的 Unitree 什么都不做（运动服务会让机器人保持站立）。有自己平衡回路的驱动（全身控制的人形机器人）必须覆盖它，指挥机器人站定（基座速度 `[0, 0, 0]`）：发送 `joint_pos` 会把关节冻住，人形机器人会摔倒。经过安全门 |
| `set_gains(kp, kd)` | optional | 每个关节的 PD 增益。默认什么都不做 |
| `has_robot_skills` | property | 驱动是否提供机器人 skill。默认 `False` |
| `list_robot_skills() -> List[RobotSkillSpec]` | optional | 机器人自身控制器运行的 skill。默认为空。见[机器人 skill](#机器人-skill) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | concrete | 按 skill 的 schema 检查 `args`，再执行。抛出 `RobotSkillError` |
| `stop_robot_skill()` | optional | 停止正在运行的 skill。默认什么都不做 |
| `list_layer_skills() -> List[RobotSkillSpec]` | concrete | 栈里各层跑的技能。每个机器人都有；没有层时为空。见[层技能](#layer) |
| `run_layer_skill(name, args=None) -> RobotSkillResult` | concrete | 启动一个层技能。没有层时抛 `RobotSkillError` |
| `stop_layer_skill()` | concrete | 停止正在运行的层技能。没有层时什么都不做 |
| `layer_skill_running` | property | 是否有层技能（栈里任一层的）持有命令流。没有层时为 `False` |
| `list_skills() -> List[RobotSkillSpec]` | concrete | 机器人能跑的所有技能，层技能和机器人 skill 合在一张表里。见[技能](#技能) |
| `run_skill(name, args=None) -> RobotSkillResult` | concrete | 按名字启动 `list_skills()` 里的技能，不管是哪一种 |
| `stop_skill()` | concrete | 停掉正在跑的技能，两种都停 |
| `skill_running` | property | 是否有层技能在跑。机器人 skill 不跟踪 |
| `disconnect()` | optional | 释放并停止。默认什么都不做 |
| `safety` | attribute | 挂接的安全门，或 `None` |
| `simulated` | class attribute | 仿真器驱动为 `True`。驱动是仿真驱动时，`make_robot` 不构建末端执行器驱动，只构建类同样为 `simulated` 的相机和音频设备（视频文件、WAV、主机的麦克风） |
| `success`, `done`, `reward` | attributes | 回合结果。hiRobot 里的驱动（`toy`、`mujoco`、各真机）都不跑任务，始终为 `None`；任务环境才设置它们，例如 hiBench 的基准环境和裁判。控制循环可以据此停止 |

## Embodiment

`hirobot.core.embodiment`。一组 pydantic 模型，描述机器人在 policy 眼中的样子（即 embodiment，本体描述），
与驱动无关。用 `load_embodiment(config)` 从配置构建（一份设备文件或一棵 `assembly:` 树）；对 assembly，
它是合并后的整棵树，每个部件的名字都在它的 `<part>/` 前缀下，`parts` 把每条路径映射到它的关节区间和子节点。

### `Embodiment`

| 字段 | 类型 | 默认值 | 含义 |
|---|---|---|---|
| `name` | `str` | 必填 | 标识符 |
| `kind` | `EmbodimentKind` | `HUMANOID` | `"humanoid"` 或 `"arm"` |
| `control_hz` | `int` | `30` | 控制循环对真机限速所用的频率 |
| `joints` | `List[Joint]` | `[]` | 驱动关节，按规范的向量顺序排列 |
| `floating_base` | `bool` | `True` | 机器人有可移动的基座（IMU / 基座状态） |
| `end_effectors` | `List[EndEffector]` | `[]` | 任务空间坐标系、手和夹爪 |
| `cameras` | `List[Camera]` | `[]` | 相机流 |
| `microphones` | `List[AudioDevice]` | `[]` | 麦克风 |
| `speakers` | `List[AudioDevice]` | `[]` | 扬声器 |

属性：`joint_names -> List[str]`、`num_dofs -> int`。

以下情况校验会抛出 `ValueError`：关节名重复、末端执行器列出了未知关节、一个关节被两个末端执行器占用，
或末端执行器有 `driver` 却没有 `joints`。

### `Joint`, `EndEffector`, `Camera`, `AudioDevice`, `EmbodimentKind`

| 模型 | 字段 | 类型 | 含义 |
|---|---|---|---|
| `Joint` | `name` | `str` | 唯一名字；DoF 映射的键 |
| | `position_limit` | `(lower, upper)` or `None` | 位置上下限（rad）。末端执行器驱动按它裁剪 |
| | `torque_limit` | `float` or `None` | 最大绝对力矩（Nm） |
| `EndEffector` | `name` | `str` | `ee_poses` / `ee_targets` / `gripper` 中的键 |
| | `has_gripper` | `bool` | 有通过 `Action.gripper` 控制的夹爪。末端执行器驱动不列 `joints`（`opening` 接口）时必须为 true，列了 `joints` 时不能为 true |
| | `joints` | `List[str]` | 该末端执行器驱动的 embodiment 关节（手指关节），按其驱动的电机顺序排列 |
| | `driver` | `dict` or `None` | 末端执行器驱动规格：`type` 指定一个已注册的末端执行器驱动，其他键是它的参数。需要 `joints` 或 `has_gripper: true` |
| | `frame` | `EndEffectorFrame` or `None` | 读取位姿的连杆：`body`（连杆名）、`pos`（m，在该连杆中）、`quat` `[x, y, z, w]`（在该连杆中）。`mujoco` 本体为带 frame 的末端执行器填写 `ee_poses`；SDK 自己报告位姿的驱动（UR、Franka）忽略它；`Kinematics` 由关节角算出同一个位姿 |
| `Camera` | `name` | `str` | `Observation.images` 中的键 |
| | `width`, `height` | `int` | 交付的画面尺寸 |
| | `driver` | `dict` or `None` | 相机驱动规格：`type` 指定一个已注册的相机驱动，其他键是它的参数。由本体驱动自己渲染的相机（仿真器）省略此项。旧键 `source` 会报错 |
| | `depth` | `bool` | RGB-D 相机：其驱动还会交付一张与彩色图像配准的深度图，放在 `Observation.depth` 中同名键下。默认 `False` |
| | `fps` | `float` or `None` | 标称帧率。`stale_sources` 用它判断过期；`control_hz` 超过它两倍时 `make_robot` 警告 |
| | `mount` | `CameraMount` or `None` | 相机的安装位置：`body`（连杆名）、`pos`（m，在该连杆中）、光学坐标系（z 向前，x 向右，y 向下）在该连杆中的 `quat` `[x, y, z, w]`、`fovy`（垂直视场角，度）。`mujoco` 本体把装好的相机渲染进 `images`；真实相机驱动忽略它 |
| `AudioDevice` | `name` | `str` | `robot.microphones` / `robot.speakers` 中的键 |
| | `driver` | `dict` or `None` | 音频驱动规格：`type` 指定一个已注册的麦克风或扬声器驱动，其他键是它的参数 |

`EmbodimentKind` 是字符串枚举，取值 `HUMANOID = "humanoid"` 和 `ARM = "arm"`。

## Observation, Action, ActionChunk, ControlMode

`hirobot.core.types`。接受 numpy 数组的 pydantic 模型。每个可选字段默认为 `None` 或 `{}`，所以驱动只需填写
其机器人提供的内容。

### `ControlMode`

字符串枚举，选择驱动如何解释一个 `Action`。

| 取值 | 含义 | 使用的字段 |
|---|---|---|
| `POSITION` (`"position"`) | 关节位置目标 | `joint_target` |
| `VELOCITY` (`"velocity"`) | 关节速度目标 | `joint_target` |
| `TORQUE` (`"torque"`) | 关节力矩目标 | `joint_target` |
| `PD` (`"pd"`) | 用每条指令自带的增益跟踪的位置目标 | `joint_target`, `kp`, `kd` |
| `CARTESIAN` (`"cartesian"`) | 末端执行器位姿目标（7 维向量） | `ee_targets` |
| `CARTESIAN_DELTA` (`"cartesian_delta"`) | 每个 tick 的末端执行器增量 `[dx, dy, dz, dax, day, daz]`（轴角），与 robosuite OSC_POSE 的输入形式相同 | `ee_targets` |
| `BASE_VELOCITY` (`"base_velocity"`) | 以本体坐标系下的 `[vx, vy, wz]` 行走（m/s, m/s, rad/s） | `base_velocity` |
| `BASE_POSE` (`"base_pose"`) | 走到 `Observation.base_position` 所在坐标系中的 `[x, y, yaw]`，然后站定 | `base_target` |
| `MOTION_LATENT` (`"motion_latent"`) | 不透明的运动 token（例如 SONIC latent） | `latent` |
| `MOTION_CLIP` (`"motion_clip"`) | 跟踪一段参考动作 | `motion`, `append` |

最后四种模式用于指挥能自己移动的机器人（全身控制器，见 [Layer](#layer)）；此时 `CARTESIAN` 也可以携带
`base_velocity`（边走边够取）。`JOINT_MODES` 是前四种关节模式组成的元组。

并非每个驱动都支持每种模式：读取 `Robot.action_modes`，并查看该驱动的设置指南。

### `Observation`

| 字段 | 类型 | 由谁填写 |
|---|---|---|
| `joint_pos` | `(N,)` | 所有驱动 |
| `joint_vel` | `(N,)` | 所有驱动 |
| `joint_torque` | `(N,)` | 测量或估计力矩的驱动 |
| `base_orientation` | `(4,)` 四元数 | 浮动基座机器人 |
| `base_angular_velocity` | `(3,)` | 浮动基座机器人（IMU） |
| `base_linear_acceleration` | `(3,)` | 浮动基座机器人（IMU） |
| `base_position` | `(3,)` | 通常只有仿真器 |
| `base_linear_velocity` | `(3,)` | 通常只有仿真器 |
| `ee_poses` | `Dict[str, (7,)]` | 机械臂 / 任务空间驱动；机器人基座坐标系下的 `[x, y, z, qx, qy, qz, qw]`。驱动负责转换其 SDK 的形式（Franka 的 4x4 `O_T_EE`、UR 的轴角） |
| `gripper` | `Dict[str, float]` | 带夹爪的机器人：每个末端执行器测得的开度，取值 [0, 1]，0 为闭合、1 为张开（与 `Action.gripper` 同一标度）。由本体驱动填，或由夹爪自己的末端执行器驱动（`opening` 接口）填 |
| `images` | `Dict[str, HxWxC uint8]` | 相机，以 `Camera.name` 为键 |
| `depth` | `Dict[str, HxW float32]` | 设置了 `depth: true` 的相机：与 `images[name]` 配准的深度（米，尺寸相同、像素一一对应），无读数处为 0 |
| `stamps` | `Dict[str, Stamp]` | 每个驱动都设 `stamps["body"]`；手、单独话题的夹爪和相机按自己的名字加上（组里是 `left/body` 等）。`Stamp(t, seq)`：`t` 是这次读数到达的时间，在 `robot.now()` 上；`seq` 是这个源的第几次读数，从 1 开始。某一路有多旧：`robot.now() - obs.stamps[name].t`；同一帧再次读到时 `seq` 不变。`hirobot.core.stamps.stale_sources(obs, robot.now(), max_age=None, embodiment=None)` 列出比 `max_age`（秒，或按源给）旧的源，默认是 3 个标称周期（`Camera.fps`、`control_hz`） |
| `instruction` | `str` or `None` | 给 VLA 的任务指令 |
| `extra` | `Dict[str, Any]` | 驱动特有的信号（触觉、原始状态……） |

### `Action`

| 字段 | 类型 | 默认值 | 含义 |
|---|---|---|---|
| `mode` | `ControlMode` | `POSITION` | 如何解释各字段 |
| `joint_target` | `(N,)` or `None` | `None` | 每个关节的目标，单位由 `mode` 决定 |
| `kp`, `kd` | `(N,)` or `None` | `None` | 每个关节的增益（`PD`） |
| `joint_names` | `List[str]` or `None` | `None` | 仅用于非关节模式：`joint_target` / `kp` / `kd` 覆盖的关节，每个名字对应一个值。必须在 `Robot.direct_joints` 中 |
| `ee_targets` | `Dict[str, (7,) or (6,)]` | `{}` | 位姿目标（`CARTESIAN`）或增量（`CARTESIAN_DELTA`） |
| `gripper` | `Dict[str, float]` | `{}` | 每个末端执行器的夹爪开度目标，取值 [0, 1]，0 为闭合、1 为张开，任何模式下都可以发。驱动负责换算到其硬件的范围。有自己末端执行器驱动的夹爪拒绝 [0, 1] 以外的值（`ValueError`） |
| `base_velocity` | `(3,)` or `None` | `None` | `[vx, vy, wz]`（`BASE_VELOCITY`，或与 `CARTESIAN` 一起） |
| `base_target` | `(3,)` or `None` | `None` | `[x, y, yaw]`（`BASE_POSE`） |
| `latent` | `(D,)` or `None` | `None` | 运动 token（`MOTION_LATENT`） |
| `motion` | `MotionClip` or `None` | `None` | 参考运动（`MOTION_CLIP`） |
| `append` | `bool` | `False` | `MOTION_CLIP`：排在正在播放的片段之后，而不是替换它（流式生成） |
| `extra` | `Dict[str, Any]` | `{}` | 自由格式的输出 |
| `parts` | `Dict[str, Action]` | `{}` | 用于带部件的机器人（`assembly:` 配置）：部件名 -> 该部件的动作，使用它自己的名字和模式。某个部件的动作可以带自己的 `parts`，对应嵌套的节点。此时除 `extra` 外的其他字段都必须为空；不带部件的机器人会拒绝它 |

以下情况构造时抛出 `ValueError`：`MOTION_CLIP` 没有 `motion`，`BASE_POSE` 没有 `base_target`，
`joint_names` 与关节模式同用、有重复或没有 `joint_target`，或 `parts` 与其他指令字段同时出现。
（旧名 `members` 会抛出指向 `parts` 的 `ValueError`。）

`joint_names` 让调用方在同一个动作中指挥那些运行机器人的控制器留空的关节，例如全身控制器只驱动躯体时
人形机器人的手指。如果控制器也驱动手，它就不会在 `direct_joints` 中列出手指关节，这样的动作会被拒绝。

### `MotionClip`

供机器人跟踪的参考运动，与具体控制器无关。帧以 `fps` 等间隔排列；所有已设置的数组帧数 `T` 相同。

| 字段 | 类型 | 含义 |
|---|---|---|
| `fps` | `float` | 帧率，为正数 |
| `space` | `str` | `"joint"`（机器人的关节，按名字）或 `"smpl"`（人体运动）；见 `MOTION_SPACES` |
| `joint_names` | `List[str]` | `joint`：`joint_pos` / `joint_vel` 的列名；可以是机器人关节的子集 |
| `joint_pos` | `(T, n)` | `joint`：关节位置（rad） |
| `joint_vel` | `(T, n)` or `None` | `joint`：关节速度（rad/s），可选 |
| `smpl_joints` | `(T, J, 3)` | `smpl`：SMPL 关节位置（m），根坐标系 |
| `root_pos` | `(T, 3)` or `None` | 根位置（m） |
| `root_quat` | `(T, 4)` or `None` | 根姿态 `[x, y, z, w]` |
| `extra` | `Dict[str, Any]` | 自由格式 |

`num_frames` 和 `duration_s`（`num_frames / fps`）是属性；`arrays()` 返回已设置的逐帧数组。
形状错误、帧数不一致或缺少 `space` 所需字段时，构造会抛出 `ValueError`。片段本身已经是一个序列，
所以它作为一个 `Action(mode=MOTION_CLIP)` 传递，而不放进 `ActionChunk`。

### `RobotStatus`

`robot.get_status()` 的返回值：给上层（agent、界面）看的健康和运行状态。不放进 `Observation`，因为远程 policy 每拍都会把
observation 转给模型。机器人不报的字段保持为空。

| 字段 | 类型 | 含义 |
|---|---|---|
| `timestamp` | `float` or `None` | 组装时的 `time.time()` |
| `battery` | `BatteryState` or `None` | `soc`（%）、`voltage`、`current`、`temperature`、`health`（%）、`cycles` |
| `joint_temperature` | `(N,)` or `None` | 每个关节电机的温度（degC） |
| `joint_faults` | `Dict[str, int]` | 关节名 -> 非零的厂商故障码 |
| `board_temperature` | `Dict[str, float]` | 板子名 -> degC |
| `fault` | `str` or `None` | 同 `robot.fault` |
| `command` | `CommandProgress` or `None` | 最近一条高层命令的进度；机器人不报时为 `None` |
| `extra` | `Dict[str, Any]` | 驱动或层自己的读数（例如 `extra["wbc"]`） |
| `safety` | `Dict[str, Any]` | 安全闸计数：`audit`、`sim_check` |

`CommandProgress` 用于控制器要跟很多拍的命令，让调用方不用懂驱动就能知道机器人走到没有、片段放完没有。由控制器填
（全身控制层；hiRobot 自带的驱动都不填，为 `None`）；命令结束后保留最后一条（`done=True`），直到下一条命令替换它。层原样往上传。

| 字段 | 类型 | 含义 |
|---|---|---|
| `mode` | `ControlMode` | 这条命令的模式 |
| `done` | `bool` or `None` | `BASE_POSE` 到达目标、`MOTION_CLIP` 放完时为 `True`；没有终点的模式（`BASE_VELOCITY`）为 `None` |
| `goal` | `[x, y, yaw]` or `None` | `BASE_POSE` 的目标，在 `Observation.base_position` 的坐标系里 |
| `clip_time`, `clip_duration` | `float` or `None` | `MOTION_CLIP`：已播放的秒数和总长 |

### `ActionChunk`

一次推理调用预测出的 `T` 个动作组成的时间窗口（horizon）。

| 字段 | 类型 | 含义 |
|---|---|---|
| `mode` | `ControlMode` | 应用于每一步 |
| `joint_targets` | `(T, N)` or `None` | 每一步的关节目标 |
| `kp`, `kd` | `(N,)` or `None` | 整个 chunk 共用的增益 |
| `joint_names` | `List[str]` or `None` | 非关节模式：`joint_targets` 覆盖的关节（见 `Action`） |
| `ee_targets` | `Dict[str, (T, 7) or (T, 6)]` | 每一步的任务空间目标 |
| `gripper` | `Dict[str, (T,)]` | 每一步的夹爪指令 |
| `base_velocity` | `(T, 3)` or `None` | 每一步的基座速度 |
| `base_target` | `(T, 3)` or `None` | 每一步的基座目标 |
| `latent` | `(T, D)` or `None` | 每一步的运动 token |
| `extra` | `Dict[str, Any]` | 自由格式的输出 |
| `parts` | `Dict[str, ActionChunk]` | 带部件的机器人：部件名 -> 它的 chunk。所有部件的 horizon 相同（否则 `ValueError`），其他字段保持为空，`step(i)` 给出带 `parts` 的 `Action` |

- `horizon -> int`（也可用 `len(chunk)`）：即 `T`，依次取自 `joint_targets`、`base_velocity`、
  `base_target` 或 `latent`，然后是 `ee_targets`，然后是 `gripper`；全部为空时为 `0`。
- `step(i) -> Action`：第 `i` 步作为单个 `Action`。超出 `[0, horizon)` 时抛出 `IndexError`。

## 技能

机器人能按名字做的事，给要把它们展示给模型的调用方（agent、语音前端）用。底下分两种：
[机器人 skill](#机器人-skill)（厂商控制器跑）和[层技能](#layer)（某一层跑）；下面四个调用把两种一起列出、
一起启动，调用方不用区分。

```python
robot = make_robot("my_g1.yaml")
robot.connect()
for s in robot.list_skills():          # RobotSkillSpec：name、description、parameters（JSON Schema）……
    print(s.name, s.description)
robot.run_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_skill()
```

| 调用 | 说明 |
|---|---|
| `list_skills() -> List[RobotSkillSpec]` | 先列层技能，再列机器人 skill。层技能和机器人 skill 同名时，层技能盖住机器人 skill（外层优先）。组装出来的机器人，名字带部件路径（`left/grasp`） |
| `run_skill(name, args=None) -> RobotSkillResult` | 转给 `run_layer_skill` 或 `run_robot_skill`，参数检查和租约规则照旧：层技能在结束前持有命令流。名字未知或参数错误抛 `RobotSkillError`，别人持有机器人时抛 `RobotBusyError` |
| `stop_skill()` | 两种都停：先 `stop_layer_skill()`，再 `stop_robot_skill()` |
| `skill_running` | 就是 `layer_skill_running`。机器人 skill 的 `run_skill` 在机器人接受命令时就返回，跑多久只有估计值（`duration_s`） |

`CommandLease` 也有 `run_skill`。机器人 skill 经由租约执行；层技能会抢占这个租约，技能结束后要重新 claim。

驱动和层不实现这四个。驱动通过 `list_robot_skills` / `_run_robot_skill` 提供机器人 skill，层通过
`_layer_skills` / `_start_layer_skill` 提供层技能（[SPI](#编写驱动和层spi)）。

## 机器人 skill

`hirobot.core.robot_skills`。机器人 skill 是由机器人自身控制器完成的动作（行走、坐下、挥手），按名字启动，
而不是靠流式发送关节目标。每个驱动声明 hiRobot 在其机器人上支持的 skill，所以调用方（agent、语音前端）
可以列出它们并展示给模型，不必写死。

```python
robot = make_robot("my_g1.yaml")      # robot.builtin_skills: g1
robot.connect()
[s.name for s in robot.list_robot_skills()]   # ['start_locomotion', 'walk', ...]
robot.run_robot_skill("start_locomotion")
robot.run_robot_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_robot_skill()
```

### `RobotSkillSpec`

| 字段 | 类型 | 说明 |
|---|---|---|
| `name` | `str` | 传给 `run_robot_skill` |
| `description` | `str` | 写给模型看的一两句话 |
| `parameters` | `dict` | 参数的 JSON Schema（一个 `object` schema）。为空表示没有参数 |
| `requires` | `List[str]` | 用自然语言写的前提条件，例如 `"robot standing on the ground"`。仅供参考 |
| `duration_s` | `float` or `None` | 典型运行时长（如果已知） |

### `run_robot_skill(name, args=None) -> RobotSkillResult`

在任何内容到达机器人之前按 `parameters` 检查 `args`，然后交给驱动执行。支持的 schema
关键字：属性的 `type`（number / integer / boolean / string）、`minimum`、`maximum`、
`exclusiveMinimum`、`enum`、`default` 和 `required`。缺少的参数取其 `default`；未知参数会被拒绝。
它在机器人接受指令后就返回，而不是等动作结束。`RobotSkillResult` 包含 `name`、检查后的 `args`
（已填入默认值）和 `duration_s`。

skill 未知、参数错误或机器人拒绝时抛出 `RobotSkillError`。

### 包装器

`RobotAssembly` 把这四个调用转发给其本体驱动，并把每个子部件的 skill 列在该部件的前缀下（`<part>/<skill>`）。

### 带 skill 的驱动

| 驱动 | 配置 | Skill |
|---|---|---|
| `unitree_sdk2` | `builtin_skills: g1` | G1 行走和手臂手势，见 [Unitree 设置指南](setup/unitree.zh-CN.md#机器人-skillg1) |

其他驱动都不提供 skill。

## 命令仲裁

同一时刻只有一个调用方给机器人发命令。`make_robot` 在栈的最外层挂一个 `CommandArbiter`（`hirobot.control.arbiter`）；调用方用 `claim`
拿到命令流，经返回的 `CommandLease` 发命令：

```python
from hirobot import Priority, PreemptedError

with robot.claim("runner", on_preempt=lambda lease: stop_runner()) as r:
    r.send_action(action)       # 还有 r.hold()、r.reset()、r.run_skill(...)
```

| 项 | 含义 |
|---|---|
| `robot.claim(name, priority=Priority.CALLER, on_preempt=None, envelope=None) -> CommandLease` | 拿命令流。高优先级立即抢占低优先级；同级或更低抛 `RobotBusyError` |
| `Priority` | `TAKEOVER`（人工接管：遥操作、手柄）> `LAYER_SKILL`（层技能运行期间）> `CALLER`（应用） |
| 有人持有时直接调 | 机器人上的 `send_action`、`hold`、`reset`、`run_robot_skill` 抛 `RobotBusyError`（`RuntimeError` 子类）。`estop`、`get_observation`、`get_status`、`now` 不检查 |
| 没人持有 | 谁都能发，单调用方的脚本不用改。配置 `arbiter: {require_claim: true}` 后没有租约就拒绝 |
| 抢占 | 先调旧租约的 `on_preempt(lease)`（出错记日志），再让机器人 `hold()`，不会继续跑旧目标（hold 失败记日志）。旧租约下一次调用抛 `PreemptedError`（`RobotBusyError` 子类） |
| 归还 | `lease.release()` 或退出 `with`。不会还给被抢的租约，它要重新 claim |
| 线程 | 租约可以在任何线程上用。抢占时最多等旧持有者正在进行的调用 `PREEMPT_LOCK_TIMEOUT_S`（0.2 s），然后 hold |
| 层 | 层发给 `inner` 的命令在仲裁器下面，不检查 |

### 安全包络

`claim(..., envelope={...})` 给一个租约比机器人 gate 更严的限制，用于只该做机器人一部分能力的应用：

```python
lease = robot.claim("demo_app", envelope={"joint_limit": 1.0, "max_joint_step": 0.05,
                                          "allowed_modes": ["position"]})
lease.envelope    # 生效的项：只留比机器人 gate 严的
```

| 键 | 含义 |
|---|---|
| `joint_limit`、`max_joint_step`、`max_joint_error`、`gripper_range` | 和 `safety:` 段（SafetyGate）一样，只管这个租约的命令 |
| `allowed_modes` | 这个租约能发的控制模式，例如 `["position", "pd"]` |

- 只能收紧。比机器人 gate 宽的项忽略，每次 claim 记一次 warning；`gripper_range` 和机器人的取交集。未知键、坏的值、
  `allowed_modes` 里机器人一个都不支持，在 claim 之前抛 `ValueError`。
- 在 `lease.send_action` 里、机器人自己的 gate 之前检查。拒绝时抛 `SafetyError`，以 `envelope of lease '<name>':` 开头。
  `clamp` 跟机器人的 gate 一致。
- 租约存续期间不变，要改就 release 再 claim。
- 只管租约发给最外层机器人的命令，不往下传到 layers 下面的 gate：全身控制器发的关节命令仍只受机器人自己的限制
  （在那里收紧可能让控制器保持不了平衡）。

### 审计

仲裁器把事件记在一个环形缓冲里（`arbiter: {audit_size: 256}`），命令只计数：

```python
robot.command_events()               # [CommandEvent, ...]，从旧到新
robot.get_status().safety["audit"]   # 计数、当前持有者、最后一次拒绝
```

| `CommandEvent.kind` | 什么时候 |
|---|---|
| `claim`、`release` | 租约拿到、归还（`TAKEOVER` 的一对标出一次干预） |
| `preempted` | 租约被抢；`detail` 写谁抢的 |
| `refused` | gate 或包络拒了一次调用（`SafetyError`）；`detail` 是原因 |
| `busy` | 因为别的租约持有、已被抢占或已归还、或 `require_claim`，调用或 claim 被拒 |

每个事件有 `t`（机器人时钟 `now()`）、`wall`（`time.time()`）、`lease`（没人持有时的直接调用为 `None`）、`priority`、`call`、
`detail`。放行的命令只计数不记事件：`safety["audit"]` 里有 `accepted`、`refused`、`by_lease`（没人持有时的调用记在
`"(direct)"`；最多留最近 64 个名字）、`holder`、`last_refusal`。租约上也有 `accepted`、`refused`。

## SafetyGate

`hirobot.control.safety`。`Robot.send_action` 在每个动作到达驱动之前运行的检查。

```python
SafetyGate(num_dofs=None, joint_limit=None, clamp=False, gripper_range=None,
           max_joint_step=None, max_joint_error=None,
           min_battery_soc=None, max_joint_temperature=None, sim_check=None)
SafetyGate.from_embodiment(embodiment, **kwargs)   # num_dofs = embodiment.num_dofs
gate.check(action, embodiment=None) -> SafetyVerdict   # 只看动作本身的检查
```

参数就是机器人配置的 `safety:` 段。检查按顺序串起来跑，第一个拒绝就停：先是只看动作本身的检查（下表前四行），
再是要读机器人状态的检查（后四行），最后是仿真检查。

| 参数 | 作用 |
|---|---|
| `num_dofs` | `joint_target` 必须是这个长度（除非设置了 `joint_names`：此时每个名字一个值，且每个名字都必须是 `embodiment` 的关节）。`None` 跳过此检查 |
| `joint_limit` | 对 `joint_target` 施加的对称边界 `[-joint_limit, joint_limit]`（rad）。`None` 跳过 |
| `clamp` | `False`（默认）：越界的目标被拒绝。`True`：裁剪到边界内 |
| `gripper_range` | 夹爪值的 `[low, high]`；值会被裁剪到此范围。`None` 跳过 |
| `max_joint_step` | 位置目标相对机器人上一次接受的目标，每个关节、每条命令最大的变化（rad）。`None` 跳过 |
| `max_joint_error` | 位置目标离实测位置最远多少（rad），每个关节。`None` 跳过 |
| `min_battery_soc` | `get_status().battery.soc` 低于这个值（百分比）时拒绝命令。`None` 跳过 |
| `max_joint_temperature` | `get_status().joint_temperature` 里有关节高于这个值（摄氏度）时拒绝命令。`None` 跳过 |

始终检查：`joint_target` 是一维且有限的，夹爪值有限；`base_velocity`、`base_target`、`latent`、
`ee_targets` 的每一项以及 `motion` 的每个数组都是有限的。

不检查：每关节上下限（`Joint.position_limit`，只有末端执行器驱动按它裁剪）。关节限位不对称的机器人（AgiBot G1 的肘部、
UR、Franka）上，每关节限位只靠机器人自己的控制器。`joint_limit` 在所有关节模式下都作用于 `joint_target`，包括 `VELOCITY`，
那时它装的是速度（rad/s）。

### 限步和偏差上限

`max_joint_step` 管命令流里的跳变：操作员手快、追踪丢帧、策略输出突变。`max_joint_error` 管目标慢慢跑离关节实际位置：
手臂被桌子挡住，调用方每条命令往前推 0.05 rad，每一步都过得了限步，桌子一挪开手臂就冲过去。PD 控制下快速运动时目标
领先实测位置，手臂带负载时会下垂，所以 `max_joint_error` 要比 `max_joint_step` 设得宽。

- 两个都只管位置目标：`POSITION` 和 `PD` 动作，以及非关节模式旁边 `joint_names` 点名的关节（只查这些关节）。
- `max_joint_step` 按每条命令算，不按每秒算：调用方停 10 s 再发一个大跳变会被拒，因为机器人还停在旧目标上。
  发送频率低的调用方，折算成 rad/s 的限制更紧。
- 没有上一次目标时以实测位置为参照：`connect`、`reset()`、`hold()` 之后的第一条命令，速度、力矩、底盘或笛卡尔命令之后，
  以及上一条命令没点名的关节。被拒的命令不会成为参照。
- `hold()` 不过这四项检查，也不过仿真检查：被挡住的手臂 `hold()` 发的是实测位置，可能离上一次目标很远，拒绝停下比命令本身更糟。
- `clamp: true` 时两者都把目标裁到上限，不拒绝，和 `joint_limit` 一样。
- `max_joint_error` 每条位置命令都读一次观测；`max_joint_step` 只在没有参照时读。

`min_battery_soc` 和 `max_joint_temperature` 每秒最多读一次 `get_status()`，读数越限期间除 `hold()` 外的命令都拒绝。
机器人不报电量或温度就不拒。没有默认值，按机器人定。

`SafetyVerdict` 包含 `allowed: bool`、`action`（可能被裁剪过，仅在允许时有效）和 `reason: str`。
`SafetyError`（一个 `RuntimeError`）就是 `send_action` 带着该原因抛出的异常。

### 仿真检查

可选，给每个本体都有 `model:`（MJCF）的真机。检查世界沿机器人自己的树拼，和 `sim:` 世界一样：每个本体按配置的位置放，
名字加部件路径前缀，可以再加 `scene`。每条过完其他检查的命令，根上的硬件 gate（在 layers 下面）把实测的关节状态拷进这个世界，
按这条命令推 `horizon_steps` 个控制周期；推演里关节力矩超过限值的 `torque_ratio` 倍、出现新的或更深的接触（自碰撞、
两个部件相碰、碰到场景里的东西）、或者陷进地面，就抛 `SafetyError` 拒绝。开始时就有的接触可以保持或变浅。模型里没有人。
它只能拒绝：`clamp` 不作用，`hold()` 不推演。

有本体没有 `model:` 时 `make_robot` 报错：只查一部分部件会漏掉碰撞。树里自己会移动的部件（移动底盘）暂不支持。
原来的 `model:` 键（`*_mujoco` 配置）已去掉。

```yaml
safety:
  sim_check:
    scene: {mjcf: ./scenes/my_table.xml}   # 可选：固定障碍物
    fixed_base: [body]         # 默认用机器人的 sim.fixed_base
    base_height: {body: 1.0}   # 默认用机器人的 sim.base_height
    horizon_steps: 5           # 每条命令推几个控制周期
    torque_ratio: 1.0
    self_collision: true
    ground: true
    budget_s: null             # 默认半个控制周期
    on_error: reject           # warn 只给调试用，上真机不能这么配
    max_failures: 3
    ee_tolerance_m: 0.02       # 实测和模型的末端位置差超过它时警告
```

推演本身失败（抛异常、模型没加载上、超过 `budget_s`）时，这条命令被拒，原因写 "simulation check failed (not the command)"。
连续 `max_failures` 次失败后机器人进故障状态，`reset()` 退出。检查拒掉命令不算失败。
每次检查把实测的 `ee_poses` 和模型在同一组关节角下的比，差超过 `ee_tolerance_m` 的警告一次并计数（标定或模型不准），不拒命令。
`get_status().safety["sim_check"]` 里有计数：`checked`、`rejected`、`errors`、`skipped`（没有关节目标可推的命令和 hold）、
`failures_in_row`、`p99_ms`、`last_reason`、`ee_mismatch`、`ee_error_m`（上次检查最大的末端误差，m）。
G1 模型上推 5 个周期，台式机 CPU 约 1.5 ms；机器人 PC2 上还没量。`safety: false` 时它和 gate 其他部分一起关掉。

## RobotAssembly

`make_robot` 根据配置构建的 `Robot` 包装器，很少需要直接构造。它自己没有定义的属性
（例如驱动的 `raw_state`）会转发给被包装的驱动。

### `RobotAssembly(body, cameras=None, end_effectors=None, embodiment=None, first_frame_timeout_s=3.0, microphones=None, speakers=None, first_audio_timeout_s=2.0, parts=None, placement=None, collision=None, own=None)`

`hirobot.assembly`。本体驱动加上相机驱动、末端执行器驱动和音频设备；或者用 `parts=`（`body=None`）表示
一棵组装树的节点，其子节点都是自成一体的机器人（见下文[子部件](#子部件)）。

- `cameras: Dict[str, CameraDriver]`：每个相机的最新帧放进 `images[name]`，时间戳放进
  `stamps[name]`（仿真本体上换成仿真时间，`seq` 不变）。相机在 `first_frame_timeout_s` 内没收到第一帧时只记录一条警告；在帧到达之前
  它的键不存在。
- `end_effectors: Dict[str, EndEffectorDriver]`，按接口分：
  - `joints`（手）：此时 `embodiment` 必须列出躯体关节加上所有末端执行器关节（否则 `ValueError`）。
    关节向量从躯体和末端执行器汇集成一个向量，`joint_target` / `kp` / `kd` 再按名字拆回去。关节模式下，
    这些末端执行器只接受 `POSITION` 和 `PD`。躯体使用非关节模式（它自己保持平衡）时，动作交给躯体，
    `joint_names` 中指定的手指目标交给末端执行器（每个一次接收它的全部关节）；此时 `direct_joints` 包含
    所有末端执行器关节，`hold()` 让躯体自己保持，并让每个末端执行器保持在测得的姿态。
  - `opening`（夹爪）：`Action.gripper[<name>]` 在任何模式下、经过安全门的 `gripper_range` 之后交给驱动，
    并从发给躯体的动作中去掉；[0, 1] 以外的值在任何东西动之前就 `ValueError`。读数放进 `gripper[<name>]`。
    它们不限制任何模式，`hold()` 不动它们。

  无论哪种接口，其他信号（触觉阵列、接触）都以 `<name>_<key>` 出现在 `extra` 中，读数的时间戳在 `stamps[<name>]`。
- `microphones: Dict[str, MicrophoneDriver]`、`speakers: Dict[str, SpeakerDriver]`：在 `connect()` 时打开，
  在 `disconnect()` 时关闭；不向观测中放入任何内容。麦克风在 `first_audio_timeout_s` 内没有音频时只记录
  一条警告。
- `damp()`：让躯体（如果其驱动支持）和每个末端执行器进入阻尼。
- `estop()`：对躯体和每个末端执行器调急停，各自打断正在跑的 ramp。
- `disconnect()`：先断开躯体，再断开末端执行器，然后停止相机并关闭音频设备。

### 子部件

用 `parts=`（部件名 -> `Robot`）且 `body=None` 时，`RobotAssembly` 是一棵组装树的节点：几台完整的机器人
（两条臂、一个人形加一条臂）加上该节点自己的相机、麦克风和扬声器，作为一个 `Robot` 驱动。`make_robot` 从带
`parts:` 段的 `assembly:` 配置构建它；部件可以嵌套，所以一个部件本身也可以是带部件的 assembly。行为在
`hirobot.assembly.node` 中：

```python
robot = make_robot("ur5e_dual")              # 两条 ur_ur5e 臂，left 和 right
robot = make_robot("examples/lab_two_cells.yaml")   # 两个 cell，每个各两条臂
```

| 主题 | 行为 |
|---|---|
| 名字 | 在 `embodiment` 中，每个子部件的关节、末端执行器、相机、麦克风和扬声器都命名为 `<part>/<name>`，嵌套节点用整条路径（`cell_a/left/shoulder_pan`）。节点自己的本体（`own=`），如果有，保留它的名字。关节按顺序拼接：先是自己的本体，再是每个子部件 |
| 坐标系 | `placement={part: {"pose": [...]}}` 固定子部件在节点中的位置；`{"origin": [...]}` 用于自己会移动的子部件（它的里程计原点）。`ee_poses` 和 `ee_targets` 在节点坐标系中：固定子部件用它的 pose 转换，移动子部件用它的 origin 与当前的 `base_position` / `base_orientation` 复合。移动子部件没报告 `base_position` 期间，它的 `ee_poses` 放进 `extra["<part>/ee_poses"]`（自身坐标系），它的末端执行器移出 `cartesian_targets`。没有 placement 的子部件放在节点原点 |
| `embodiment` | 合并后的整棵树（`hirobot.assembly.parts.merge_embodiments`）：关节拼接，恰好一个部件有浮动基座时 `floating_base` 为真，`control_hz` 取该节点的。`embodiment.parts` 把每条路径映射到它的关节区间和子节点 |
| `get_observation()` | 关节向量拼接；`ee_poses`、`gripper`、`images`、`depth`、`stamps`、`extra` 的键加前缀（`stamps["left/body"]`）。恰好一个浮动基座部件时它的 `base_*` 字段留在顶层；有多个时放进 `extra["<part>/base_*"]` |
| `send_action(action)` | 扁平或按部件。按部件（`Action.parts`）：部件名 -> 它自己名字和模式下的动作；部件的动作可以为子节点嵌套 `parts`。扁平：所有部件用同一模式，`joint_target` / `kp` / `kd` 按各部件关节数拆分（或在有 `joint_names` 时按名字），`ee_targets` / `gripper` 按名字前缀拆分，`base_velocity` / `base_target` / `latent` / `motion` 发给唯一的浮动基座部件。动作没提及的部件收不到任何内容，保持其最后的目标；它的仿真仍前进一个周期（`idle_step`） |
| 检查 | 每个部件的模式和 `direct_joints` 在它那部分上检查，然后是碰撞检查（配置了时）；只要有一处被拒绝，就什么都不发送给任何部件。根外面的那一个安全门按机器人的名字、逐关节检查整个动作（含 `Action.parts`） |
| 能力 | `action_modes`：所有部件都接受的模式。`direct_joints`、`cartesian_targets`：各部件的值，加前缀。`motion_spaces`：恰好一个浮动基座部件时取它的值 |
| 错误 | 某个部件读取或发送失败，或报告了 `fault`，assembly 就让其他所有部件保持，并进入自己的故障状态。此后 `send_action` 抛出 `RobotFaultError`，直到 `reset()` |
| 租约 | 部件是带自己[命令租约](#命令仲裁)的 `Robot`，所以 `robot.claim(name, parts=["left"])` 只持有那个部件（嵌套的用 `parts=["cell_a/left"]`）；不相交的部件各自独立 claim，路径重叠抛 `RobotBusyError`。直接对已被持有的部件调用抛 `RobotBusyError`。部件自己的线程（全身控制器）仍能指挥它的内层机器人 |
| `part(path)` | 只读视图：`embodiment`、`simulated`、`fault`、`get_observation()`、`get_status()` |
| `get_status()` | `joint_temperature` 拼接；`joint_faults`、`board_temperature`、`extra` 的键加前缀；`battery` 和 `command` 只有一个部件报告时放在顶层，否则放进 `extra["<part>/battery"]` / `extra["<part>/command"]`；`fault` 取 assembly 自己的或第一个部件的 |
| Skill | 部件的 skill 列为 `<part>/<skill>`；`run_robot_skill("g1/wave")` 在该部件上运行它 |
| 生命周期 | `connect` 先连接各部件，再连接节点自己的相机，失败时撤销。`reset`、`hold`、`damp`、`estop`、`disconnect`、`stop_robot_skill` 即使某个部件失败也会作用到每个部件，然后抛出第一个错误。`reset` 也会清除故障状态 |
| 碰撞 | 可选的 `collision=CollisionCheck(...)`（配置：`collision:`）。每个建了模的部件表示为一组胶囊体，其 MJCF（`hirobot.geometry.kinematics`）中每段 body 到父 body 的连线一个，半径都是 `radius`；只计算不同部件之间的间隙。发送前，assembly 预测每个部件的下一个姿态（`POSITION` / `PD` 的关节目标作为下一时刻的角度，其他模式用当前角度；`CARTESIAN` 的目标点作为球体比较），当最小间隙会低于 `min_distance` 时以 `SafetyError` 拒绝，除非间隙没有在缩小（已经靠得太近的部件可以分开）。每次发送读取每个建模部件的观测一次。浮动基座部件需要 base pose。两台 G1 约 1 ms。沿运动链的胶囊体比较粗糙：选一个能覆盖连杆的 `radius`；`skip:` 排除安装板之类的 body |
| 全身控制部件 | 用 hiWBC 的 `wbc` layer 包装的部件（在其配置中写 `layers: [{type: wbc, profile: ...}]`）只接受 `BASE_VELOCITY` / `MOTION_LATENT` / `MOTION_CLIP`，所以 assembly 的 `action_modes` 通常为空：用 `Action.parts` 指挥，手臂用关节模式，它用行走指令。它的空闲关节（`direct_joints`，例如上半身）可以在同一个子动作中按名字一并发送。未被提及时，`idle_step` 运行一个控制器 tick，让仿真中的人形机器人保持站立。控制器出错会让该部件进入故障并使 assembly 保持。hiWBC 自己的测试用 `echo` profile 覆盖了这种情况 |
| 示例 | `ur5e_dual`：两条 UR5e 臂（`left`、`right`），相距 ±0.4 m；`franka_ur5e_dual` 再加一个 Franka FR3 和一个 `scene` 相机；`examples/lab_two_cells.yaml` 把两个 `ur5e_dual` cell 嵌在一个 lab 下。所有部件默认用 toy 驱动，所以不需要硬件就能运行（按路径覆盖每条臂的本体驱动，`overrides={"left/body": {"driver": ...}}`） |

碰撞检查 `collision:` 段（路径相对于配置文件）：

```yaml
collision:
  min_distance: 0.05        # m between capsule surfaces (default 0.05)
  radius: 0.05              # default capsule radius (m)
  backend: numpy            # or mujoco, for MJCFs the numpy parser rejects
  models:                   # parts without a model are not checked
    left:  {mjcf: ~/models/fr3.xml}
    right: {mjcf: ~/models/ur5e.xml, radius: 0.06, skip: [base_mount]}
```

## 控制循环

hiRobot 没有 policy 类、控制循环或模型服务客户端；它只提供循环所驱动的 `Robot`。循环从驱动用到的内容有：
`reset(seed=None)`、`get_observation()`、`send_action()`、停止后的 `hold()`、用于给真机限速的
`embodiment.control_hz`、`simulated`（仿真器不限速地步进），以及仿真器上的 `success` / `done` /
`reward`。`ActionChunk.step(i)` 把预测出的 chunk 中的一步转换成一个 `Action`。

## DofMap

`hirobot.core.dof.DofMap(source_names, target_names)` 按名字在两种关节顺序之间映射。目标名字在源中不存在时
抛出 `ValueError`。

| 方法 | 说明 |
|---|---|
| `remap(values)` | 按源顺序排列的数组 → 目标顺序（第一个轴）。会检查长度 |
| `scatter(values, default=None)` | 按目标顺序排列的数组 → 源顺序。不在目标中的源关节取 `default`（或 `0`）。policy 只指挥部分关节时使用 |

```python
m = DofMap(robot.embodiment.joint_names, policy_joint_names)
policy_q = m.remap(obs.joint_pos)
robot_target = m.scatter(policy_target, default=obs.joint_pos)
```

## Kinematics

`hirobot.geometry.kinematics.Kinematics.from_mjcf(path, backend="numpy")` 加载一个 MJCF 模型，并由关节角和基座位姿
算出每个 body 的世界位姿。位置单位为米，四元数为 `[x, y, z, w]` 且 `w >= 0`。

| 后端 | 依赖 | 说明 |
|---|---|---|
| `"numpy"`（默认） | 仅 numpy | 从 MJCF 读取 body 树：带 `pos` / `quat` 的嵌套 `<body>`、hinge 关节（`axis`、`pos`）、顶层 body 上的一个 free 关节。遇到它不读取的 MJCF 特性（`euler` / `axisangle` / `xyaxes` / `zaxis`、slide / ball 关节、关节 `ref`、`<default>` 关节属性、`<include>`、`<frame>`）时抛出 `ValueError` |
| `"mujoco"` | `pip install 'hirobot[sim]'` | 对同一文件调用 `mujoco.mj_kinematics`。没有 mujoco 时抛出 `ImportError`。用它交叉检查 numpy 的结果，或处理 numpy 解析器拒绝的模型 |

后端总是显式选择，绝不按已安装的包自动决定。测试会把 numpy 后端与 `mj_kinematics` 对比（G1 29 DoF：
最大差值低于 1e-12）。

| 成员 | 说明 |
|---|---|
| `body_names`, `parents` | 按模型顺序排列的 body（不含 world body）及其父节点索引（`-1` = 顶层） |
| `joint_names` | 按模型顺序排列的 hinge 关节，名字与 MJCF 中一致 |
| `floating_base` | 顶层 body 有 free 关节时为 True |
| `forward(joint_pos, base_pos=None, base_quat=None, joint_names=None) -> BodyPoses` | `joint_pos` 形状为 `(..., n)`。给了 `joint_names`（`name` 或 `name_joint`）时按名字匹配列，未列出的模型关节取 0；没给时列按 `Kinematics.joint_names` 排列。`base_pos` / `base_quat` 放置带 free 关节的 body（默认：其 MJCF 位姿；仅适用于浮动基座模型） |

`BodyPoses` 包含 `names`、`pos` `(..., num_bodies, 3)` 和 `quat` `(..., num_bodies, 4)`；
`poses["left_wrist_yaw_link"]` 返回该 body 的 `(pos, quat)`。

```python
from hirobot.geometry.kinematics import Kinematics

kin = Kinematics.from_mjcf("~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")
poses = kin.forward(obs.joint_pos, base_pos=[0, 0, 0.79], base_quat=[0, 0, 0, 1],
                    joint_names=robot.embodiment.joint_names)
pos, quat = poses["torso_link"]
```

## 配置文件参考

由 `load_embodiment` / `make_robot` / CLI 读取的 YAML 文件，以内置名字（`"unitree_g1"`、
`"body/unitree/h1"`；用 `hirobot configs` 列出）或文件路径给出。内置名字先在包里找 `configs/<name>.yaml`，
再找 `configs/robots/<name>.yaml`，所以 `"unitree_g1"` 和 `"robots/unitree_g1"`（`hirobot configs` 列出的写法）
指同一个文件。以 `.yaml` / `.yml` 结尾或以 `/`、`~`、`.` 开头的字符串是路径。一个配置可以在 `defaults:` 下列出其他配置；
它们按顺序合并，文件本身最后合并。映射逐键合并；`joints`、`frames`、`mounts` 和各设备列表按 `name` 合并
（同名：字段合并；新名字：追加）；其他值整体替换。设备的 `driver:` 在 `type` 不变时逐键合并，`type` 改变时
整体替换。`hirobot info --config <name> --resolved` 打印合并结果。规则和示例见 `hirobot.loading.config`、`hirobot.loading.tree`。

配置是两种形态之一。**设备文件**描述一个设备型号；它的顶层是 `device:` 类型（`body`、`end_effector`、
`camera`、`microphone` 或 `speaker`）：

```yaml
device: body                # 类型；其余键取决于它
control_hz: 125
floating_base: false
joints:                     # body / end_effector
  - name: shoulder_pan
  - {name: wrist_3, position_limit: [-6.28, 6.28], torque_limit: 28}
frames:                     # 任务空间坐标系（末端执行器 site）
  - {name: tcp}
mounts:                     # 组件可以挂接的位置（link -> site）
  wrist_camera: {link: wrist_3_link}
model:                      # 可选的 MJCF，用于仿真 / 碰撞 / 运动学
  mjcf: ~/models/ur5e.xml
driver: {type: ur_rtde}     # 注册的驱动名 + 它的构造参数
```

相机文件带 `width` / `height` / `fps` / `depth` / `fovy`；末端执行器文件带 `interface`（`joints` 或
`opening`）、`joints`、`tcp`；麦克风或扬声器只带 `driver`。

**机器人配置**有一个 `assembly:` 节点列出它的部件。部件是一个设备或另一个节点，用 `from:`（内置名字或路径，
相对于引用它的文件）引用，或内联写出（`device:` 或 `assembly:`）：

```yaml
assembly:
  name: ur5e_dual
  control_hz: 30            # 可选；默认取最慢部件的
  parts:                    # name -> 部件；关节顺序按这个顺序
    left:  {from: robots/ur_ur5e, overrides: {body: {driver: {ip: 192.168.1.101}}}}
    right: {from: robots/ur_ur5e, safety: {max_joint_step: 0.1}}   # 只收紧这个部件
    scene: {device: camera, width: 640, height: 480, driver: {type: array}}   # 内联设备
  placement:               # 每个部件在节点中的位置
    left:  {pose: [0, 0.4, 0, 0, 0, 0, 1]}         # 固定：[x, y, z, qx, qy, qz, qw]
    right: {pose: [0, -0.4, 0, 0, 0, 0, 1]}
    scene: {on: body.wrist_camera, pose: [0, -0.07, 0.03, 0, 0, 0, 1]}  # 挂在某部件的 mount 上
    # 自己会移动的部件用 origin: 代替 pose:
    # rover: {origin: [0, 0, 0, 0, 0, 0, 1]}
  collision: {min_distance: 0.05}                  # 可选，见 RobotAssembly

# 只有根配置能带的段（被引用的部件不能带）：
safety:                    # 可选；`safety: false` 摘掉安全门
  max_joint_step: 0.2      # SafetyGate 构造参数（joint_limit、clamp、gripper_range、...）
layers:                    # 可选；注册的 layer，第一个在最内
  - {type: my_wbc, part: left}    # part: 只包装那个部件（否则包整机）
arbiter:
  require_claim: false     # 可选；没有租约就拒绝命令（见“命令仲裁”）
  audit_size: 256          # 可选；审计环形缓冲保留的事件数
sim:                       # 可选；make_robot(..., sim=True) 构建一个共享的 MuJoCo 世界
  realtime: true
```

单机器人配置就是只有一个 `body` 部件的 `assembly:`（见 `robots/ur_ur5e`）。设备引用上的 `overrides` 合并进
设备文件；节点引用上的 `overrides` 把子部件名映射到该部件的 overrides，逐层下去。
`make_robot(cfg, overrides={"left/body": {"driver": {"ip": ...}}})` 用同样的方式从命令行合并与主机相关的值。

policy 设置（模型服务器、chunk 长度）不属于机器人配置。

## CLI

安装后的命令名为 `hirobot`。

| 命令 | 说明 |
|---|---|
| `hirobot --version` | 打印版本 |
| `hirobot info --config NAME_OR_PATH [--resolved]` | 打印 embodiment：类型、控制频率、浮动基座、关节、末端执行器、相机。`--resolved` 打印合并 `defaults:` 之后的完整配置 |
| `hirobot configs [GROUP]` | 列出内置配置（`robots`、`body`、`end_effector`、`camera`、`microphone`、`speaker`）。`robots/` 下的名字也可以不带前缀（`unitree_g1`） |
| `hirobot sim CONFIG [--seconds N] [--viewer] [--fast]` | 运行一台仿真机器人：连接，保持起始姿态 N 秒（默认 3），打印剩余的关节误差、基座高度、渲染出的图像和 `get_observation` 耗时。自由基座摔倒时会提示。对真机本体以退出码 2 退出；`--viewer` 需要 MuJoCo 本体 |
| `hirobot check --sim CONFIG [--against-real] [--driver PART:KEY=VALUE ...] [--tolerance 0.02]` | 建仿真检查用的世界，不推进：每个本体要有 `model:`，每个关节要在它的 MJCF 里且有执行器，每个末端 frame 是 MJCF 里的 body；关节限位和 MJCF 的 range 差超过 0.1 rad 打印 warning。`--against-real` 再以 `read_only: true` 打开每个本体的真机驱动（没有这个模式就拒绝），读一次观测，打印同一组关节角下每个末端的实测和模型位置差。出错或误差超过 `--tolerance` m 时退出码 1。库：`hirobot.sim.check_sim` |
| `hirobot view CONFIG [--model MUJOCO_CONFIG] [--body KEY=VALUE ...] [--hz 30] [--seconds N]` | 在 MuJoCo viewer 里显示真机姿态。以 `read_only: true` 打开 CONFIG（驱动没有 `read_only` 模式就拒绝），每帧把测得的关节（按名字对）和基座姿态拷进模型，从不发命令。`--model` 默认用内置的 `<config>_mujoco`。仿真机器人或找不到模型时退出码 2。库：`hirobot.sim.view` |

## 编写驱动和层（SPI）

上面几节是应用 API。这一节给扩展 hiRobot 的代码看：驱动和层。这些方法以 `_` 开头；应用从不调用，层的 `inner` 也不暴露它们。

| 成员 | 类型 | 说明 |
|---|---|---|
| `_send_action(action)` | abstract | 驱动原语。**驱动实现这个方法，绝不实现 `send_action`** |
| `_estop()` | optional | `estop()` 背后的驱动原语。默认在驱动有 `damp()` 时调用它 |
| `_idle_step()` | optional，SPI | 在最后一条指令下让仿真时间前进一个控制周期。assembly 对动作未提及的仿真部件调用它。默认什么都不做；toy 仿真和 MuJoCo 会步进 |
| `_set_safety(gate)` | concrete，SPI | 挂接或清除（`None`）`SafetyGate`。给 `make_robot` 和测试用；应用在配置的 `safety:` 段里设限制 |
| `_run_robot_skill(spec, args)` | optional | `run_robot_skill` 背后的驱动原语；`args` 已检查过 |
| `_set_fault(reason)` / `_clear_fault()` | concrete，SPI | 让机器人进故障态（驱动或 layer 自己进入阻尼或停止时），命令抛 `RobotFaultError`，直到 `reset()`；清除故障 |
| `_stop_token()`、`_commanding(token)`、`_wait(seconds, token)` | concrete，SPI | 让驱动里跑一阵的循环在别的线程调 `estop()` 时立刻结束（见[编写驱动](#编写驱动)） |

### 编写驱动

继承 `Robot`，实现 `embodiment`、`connect`、`reset`、`get_observation` 和 `_send_action`，然后注册它。
新驱动需要的目录结构、清单和契约测试见 README 的
[Add a new robot](../README.zh-CN.md#添加新机器人)。构造函数的第一个参数必须是 `Embodiment`；
其余关键字参数就是用户写在 `body:` 下的内容。仿真器要设置 `simulated = True`。要提供机器人 skill，
覆盖 `has_robot_skills`、`list_robot_skills`、`_run_robot_skill` 和 `stop_robot_skill`。机器人有安全状态的驱动覆盖
`_estop()`（或提供 `damp()`）。会跑一阵的循环（ramp、等首帧）先取 `token = self._stop_token()`，每一步在
`with self._commanding(token):` 里下发，用 `self._wait(seconds, token)` 睡眠，这样别的线程调 `estop()` 时它会立即结束。在
`connect()` 内部导入厂商 SDK，这样没有它们也能导入本包；并用一个假 SDK 做测试
（`tests/drivers/contract.py`）。

### 编写层

层是一个单独的包，依赖 `hirobot`。步骤：

1. **继承 `Layer`。** 构造函数是 `cls(inner, **config)`：`inner` 是下面的机器人，其余参数是配置里这一层 `layers:` 条目去掉 `type`。
2. **声明接受什么。** 只有和 `inner` 不同时才覆盖 `action_modes`、`direct_joints` 等声明。对 `inner` 只用 `inner` 声明过的模式。
3. **实现。** 被动层实现 `_send_action`：算完调 `self.inner.send_action(...)`。带自己线程的层（全身控制）在 `_send_action` 里只更新
   目标，覆盖 `hold()` 和 `_estop()`（只停线程），线程出错时调 `self._set_fault(reason)`。层自己跑的技能走[层技能](#层的钩子)。
4. **只用 `inner` 的公开接口。** 不碰 `_` 开头的名字，不经 `inner` 拿驱动对象。
5. **注册。** 用 `@register_layer("name")`，所在模块写进 `pyproject.toml` 的 `hirobot.drivers` entry point：

   ```toml
   [project.entry-points."hirobot.drivers"]
   mypkg = "mypkg.plugin"
   ```

6. **测试。** 先在 `toy` 和 `*_mujoco` 配置上跑，再用 `hirobot.testing.assert_layer_uses_public_api(MyLayer, config)` 查对 `inner`
   的访问。上真机按只读 → 从真机姿态起步的仿真 → 吊架低增益。

一个最小的被动层，在 toy 上的 UR5e 上运行（这段代码在测试里会跑）：

```python
import numpy as np

from hirobot import Action, ControlMode, Layer, Robot, make_robot, register_layer
from hirobot.loading.config import load_config


@register_layer("rate_limit")
class RateLimit(Layer):  # 被动层：限制每条命令里每个关节目标最多移动多少

    def __init__(self, inner: Robot, max_step: float = 0.05):
        super().__init__(inner)
        self.max_step = float(max_step)
        self._last = None

    def reset(self, seed=None):
        self.inner.reset(seed)
        self._last = None

    def _send_action(self, action: Action) -> None:
        if action.mode is ControlMode.POSITION and action.joint_target is not None:
            if self._last is None:
                self._last = self.inner.get_observation().joint_pos
            step = np.clip(action.joint_target - self._last, -self.max_step, self.max_step)
            self._last = self._last + step
            action = action.model_copy(update={"joint_target": self._last})
        self.inner.send_action(action)


config = load_config("ur_ur5e")  # toy 上的 UR5e
config["layers"] = [{"type": "rate_limit", "max_step": 0.05}]
robot = make_robot(config)
robot.connect()
robot.reset()
target = Action(mode=ControlMode.POSITION, joint_target=np.full(6, 0.3))
robot.send_action(target)
print(robot.get_observation().joint_pos)  # 每个关节只走了 0.05 rad，不是 0.3
```

### 层的钩子

在任何一层上调 `estop()`，都先直接急停栈最底下的机器人，再从外往里调每层自己的 `_estop()`。每层有
`ESTOP_LAYER_TIMEOUT_S`（0.2 s），抛异常或卡住的记日志后跳过，卡住的层挡不住急停。覆盖 `_estop` 只停自己这层，不要调 `inner.estop()`。

layer 是受信任代码，按内核模块的标准审。`self.inner` 是只给公开接口的代理：`_` 开头的名字（`_send_action`、`_set_safety` 等）
和拼装层的 `body` / `end_effectors` 会抛 `AttributeError`。`hirobot.testing.assert_layer_uses_public_api(MyLayer, config)`
在 toy 上跑一遍层，碰过这些就失败。

层技能（[使用方](#layer)）：层实现 `_layer_skills() -> List[RobotSkillSpec]`、`_start_layer_skill(spec, args)`（启动后返回；
在里面调层自己的 `hold()` 等能通过）、`_stop_layer_skill()`，技能自己结束时调 `_layer_skill_finished()`。层自己的线程照常给
`inner` 发命令，那在仲裁器下面。

## 扩展点：相机、末端执行器与音频

相机、末端执行器（灵巧手、夹爪）和音频设备有各自的注册表，与本体驱动分开。与本体驱动一样，它们惰性导入厂商 SDK。

### 相机驱动（`hirobot.devices.camera`，驱动在 `hirobot.drivers.camera` 中）

| 名字 | 说明 |
|---|---|
| `register_camera(name)`, `get_camera(name)`, `available_cameras()` | 注册表，行为与驱动注册表相同 |
| `CameraDriver(name)` | 抽象类：`start()`、`latest() -> Optional[Frame]`（`(image, Stamp)`，`Stamp.t` 在 `time.monotonic()` 上，每来一帧新的 `seq` 加一；不能阻塞；第一帧之前返回 `None`）、`stop()`（幂等）。RGB-D 驱动还要覆盖 `latest_rgbd() -> Optional[RgbdFrame]`；默认实现返回不带深度的 `latest()` |
| `PollingCameraDriver(name, poll_hz=30.0, retry_s=0.2)` | 在后台线程上运行阻塞的 `_grab() -> HxWx3 uint8 RGB`，只保留最新一帧。子类实现 `_grab`，可选实现 `_open` / `_close`。提供 `wait_for_frame(timeout)`、`error`、`frames`、`errors` |
| `Frame` | `Tuple[np.ndarray, Stamp]`：图像及其到达时间（`Stamp(t, seq)`，`t` 在 `time.monotonic()` 上） |
| `RgbdFrame` | `Tuple[np.ndarray, Optional[np.ndarray], Stamp]`：彩色图像、与之配准的深度（HxW float32，米）或 `None`，以及两者共同的 `Stamp` |
| `intel_realsense` → `RealSenseCamera(name, serial=None, width=640, height=480, fps=30, depth=False, depth_width=None, depth_height=None, timeout_ms=1000, poll_hz=60.0)` | 直接插在跑 hiRobot 的计算机本地 USB 上的 Intel RealSense（D435i、D405、L515……），走 `pyrealsense2`。接了多个时用 `serial` 指定设备（否则取枚举到的第一个）。设置 `depth: true` 时它是 RGB-D 相机：深度传感器对齐到彩色，使 `Observation.depth` 与彩色共用同一像素网格，并按传感器的 `depth_scale` 把设备单位换算为米。深度流尺寸和彩色不同时用 `depth_width` / `depth_height` 设置（默认和彩色一样）；对齐后的深度图仍是彩色的尺寸。需要 `pyrealsense2`（`pip install pyrealsense2`）；L515 要老版本（2.54.2 已验证；2.59.1 列不出 L515） |
| `orbbec_pyorbbecsdk` → `OrbbecCamera(name, serial=None, uid=None, width=640, height=480, fps=30, color_format="MJPG", depth=False, align="hw", timeout_ms=1000, poll_hz=60.0)` | 直接插在跑 hiRobot 的计算机本地 USB 上的 Orbbec 相机（Dabai DC1……），走 1.x 版 `pyorbbecsdk`（OrbbecSDK 1.10；老型号在 2.x 下枚举不到）。用 `uid`（USB 端口路径）或 `serial` 指定设备。设置 `depth: true` 时它是 RGB-D 相机：深度对齐到彩色（`align`：`hw` 在相机上做，`sw` 在 SDK 里做），设备单位换算为米。MJPEG / YUYV 彩色用 OpenCV 解码。绑定需从源码编译，见 [agilex_piper.zh-CN.md](setup/agilex_piper.zh-CN.md#6-orbbec-相机) |
| `stereolabs_zed` → `ZedCamera(name, serial=None, resolution="HD720", fps=30, depth=False, depth_mode="NEURAL", min_depth_m=0.1, poll_hz=60.0)` | 直接插在跑 hiRobot 的计算机本地 USB 上的 Stereolabs ZED（ZED 2i、ZED Mini……），走 `pyzed`。接了多台时用 `serial` 指定（否则取第一台）。图像是左目彩色（`resolution`：`HD2K`、`HD1080`、`HD1200`、`HD720`、`SVGA`、`VGA`）。设置 `depth: true` 时它是 RGB-D 相机：深度由 SDK 的 `depth_mode`（`NEURAL`、`ULTRA`、`QUALITY`、`PERFORMANCE`）算出，单位米，对齐到左目图像，没有读数处为 0。需要 ZED SDK、它的 `pyzed` wheel 和 NVIDIA GPU |
| `agibot_cosine` → `AgibotCosineCamera(name, topic, depth_topic=None, depth_scale=0.001, max_pair_skew_s=0.005, pair_wait_s=0.02, queue=4, poll_hz=60.0)` | 走机器人 `cosine_bus` topic 的 AgiBot G1 相机（`/camera/head_color` + `/camera/head_depth`、`/camera/hand_left_color`……）。设置了 `depth_topic` 时它是 RGB-D 相机：每个彩色帧与同一次采集（`source_timestamp`）的深度帧配对，Z16 换算为米。在机器人的计算机上运行，需先 source 厂商环境（`LOCATOR_IP`、`AORTA_DISCOVERY_URI`） |
| `unitree_videohub` → `UnitreeVideohubCamera(name, network_interface=None, domain_id=0, timeout_s=1.0, poll_hz=30.0)` | G1 头部相机 |
| `unitree_image_server` → `UnitreeImageServerCamera(name, address, header=False, crop=None, timeout_s=1.0, poll_hz=60.0)` | H1 头部相机：通过 ZeroMQ SUB 接收 Unitree 遥操作 `image_server`（或 `scripts/camera_image_server.py`）发出的 JPEG 帧。`header` 去掉 `Unit_Test` 的时间戳 / 帧号（保存在 `remote_stamp` / `remote_frame_id` 中）；`crop: [x, y, w, h]` 从左右拼接的画面中取出一个相机。需要 `camera` extra |
| `video_file` → `VideoFileCamera(name, path, fps=5.0, loop=True)` | 按自己的时间线播放的视频文件，以 `fps` 采样。不设 `loop` 时，文件结束后停留在最后一帧（`ended` 变为 `True`）。需要 `video` extra（PyAV）。仿真类 |
| `array` → `ArrayCamera(name, frames=None, fps=10.0, advance_on_read=False)` | 按主机时钟循环播放内存中的帧；没有 `frames` 时为一张 64x64 的灰色帧。`advance_on_read=True` 让每次 `latest()` 返回下一帧（用于测试）。仿真类 |
| `build_cameras(cameras, network_interface=None, sim_only=False) -> Dict[str, CameraDriver]` | 为每个带 `driver:` 的相机构建一个相机驱动；`sim_only` 只构建仿真类驱动。`make_robot` 用它构建 embodiment 的相机 |

不需要硬件的相机驱动类会设置 `simulated = True`。仿真本体只会构建这类驱动，而且它们不会拿到本体的
`network_interface`。

### 末端执行器驱动（`hirobot.devices.end_effector`，驱动在 `hirobot.drivers.end_effector` 中）

有自己驱动的灵巧手和夹爪共用一个基类和一个注册表，区别只在**接口**，即末端执行器在 `Observation` /
`Action` 中怎么出现。接口由配置决定：

| 接口 | 配置 | 观测 | 指令 |
|---|---|---|---|
| `joints` | 列出 `joints: [...]`（它们在 `Embodiment.joints` 中） | 这些关节在 `joint_pos` / `joint_vel` / `joint_torque` 中 | `joint_target` / `kp` / `kd` 中它们那一段；非关节模式下按 `joint_names` |
| `opening` | 不列 `joints`，`has_gripper: true` | `gripper[<name>]`，取值 [0, 1]，0 闭合、1 张开 | `Action.gripper[<name>]`，**任何**模式下都发 |

按设备接受什么来选，不按形状：多指手总是 `joints`；接受电机级目标和增益的单电机夹爪也可以用 `joints`，
只接受开度目标（自己闭环）的夹爪用 `opening`。驱动类在 `interfaces` 中列出它支持的接口；配置要求别的
接口时，构建机器人时就会被拒绝。

| 名字 | 说明 |
|---|---|
| `register_end_effector(name)`, `get_end_effector(name)`, `available_end_effectors()` | 注册表 |
| `EndEffectorDriver(name, joints, read_only=False)` | 抽象类：`connect()`、`get_state()`、`damp()`。`joints` 接口：`command(q, kp=None, kd=None)`，可选 `set_gains(kp, kd)`。`opening` 接口：`command_opening(opening)`，不等动作完成就返回，新目标替换旧目标，[0, 1] 以外 `ValueError`。可选：`reset()`、`disconnect()`（默认进入阻尼）。`estop()`：结束正在跑的 `reset` ramp（`RobotStoppedError`），再进入阻尼。属性：`interfaces`（类属性，默认 `("joints",)`）、`interface`（列了关节为 `"joints"`，否则为 `"opening"`）。辅助成员：`joint_names`、`num_joints`、`clip(q)`（裁剪到 `Joint.position_limit`） |
| `JointsState` | `joints` 读数，dataclass：`q`、`dq`、`tau`（不可信处为 NaN）、`stamp`（`Stamp`，`time.monotonic()`；放进 `Observation.stamps[<name>]`）、`extra` |
| `OpeningState` | `opening` 读数，dataclass：`opening`，取值 [0, 1]（放进 `Observation.gripper[<name>]`）、`stamp`、`extra`（接触、电流等；以 `<name>_<key>` 合并进 `Observation.extra`） |
| `check_opening(value)` | 把 `value` 转成 float，非有限值或不在 [0, 1] 内时 `ValueError` |
| `end_effector_joint_names(emb)`, `has_end_effector_drivers(emb)`, `body_embodiment(emb)`, `build_end_effectors(emb, network_interface=None, read_only=False)` | `make_robot` 用来把末端执行器从躯体中拆分出来的辅助函数。`body_embodiment` 还会清掉 `opening` 末端执行器的 `has_gripper`，因为那个夹爪由它自己的驱动控制 |
| `unitree_handcmd` → `UnitreeDex3Hand(name, joints, side, network_interface=None, domain_id=0, default_kp=1.5, default_kd=0.2, ramp_seconds=1.0, ramp_hz=100.0, stale_s=0.5, read_only=False)` | Unitree Dex3-1 灵巧手（`unitree_hg` `HandCmd_`）；`side` 为 `"left"` 或 `"right"` |
| `unitree_motorcmds` → `UnitreeMotorCmdsHand(name, joints, cmd_topic, state_topic, motor_offset=0, num_motors=None, service=..., network_interface=None, domain_id=0, connect_timeout_s=2.0, stale_s=0.5, read_only=False)` | 任何接在 Unitree 串口转 DDS 服务（`unitree_go` `MotorCmds_` / `MotorStates_`）后面的手：H1 上通过 `h1_inspire_service` 接的 Inspire，以及布局相同的 Dex1-1、BrainCo、Linker。共用一对 topic 的手（Inspire 1.0.1：12 个电机，`motor_offset` 为 0 / 6）共用一条总线；还没有手指挥过的槽位保持其测得的姿态 |
| `inspire_sdkpy` → `InspireSdkpyHand(name, joints, side, network_interface=None, domain_id=0, cmd_topic=None, state_topic=None, connect_timeout_s=2.0, stale_s=0.5, read_only=False)` | 通过 inspire_sdkpy DDS 桥接的 Inspire RH56DFX（`rt/inspire_hand/{ctrl,state}/{l,r}`，角度 0-1000 换算为 0-1） |
| `robotiq_urcap` → `RobotiqUrcapGripper(name, joints, host, port=63352, speed=255, force=255, activate=False, connect_timeout_s=3.0, stale_s=0.5, poll_hz=20.0, read_only=False)` | Robotiq 2F 夹爪（`opening` 接口），通过 UR 控制器上的 Robotiq URCap socket（控制器 IP 的 TCP `port`）访问。寄存器 0 为全开、255 为全闭，驱动换算成 `opening` 1 张开 / 0 闭合。`speed` / `force` 为 0-255；`activate` 在 `connect()` 时按需激活（会完整闭合再张开一次）。`OpeningState.extra` 含 `object`（Robotiq 物体检测）和 `fault`。`damp()` 发 `GTO 0`（停住并保持）。只用 Python 标准库 |

末端执行器驱动以 `cls(ee.name, joints, **spec)` 构造，其中 `opening` 接口的 `joints` 为空，`spec` 是末端执行器的 `driver:` dict 去掉 `type`。
spec 没有设置 `network_interface` 时，`make_robot` 会填入本体的值。

### 音频设备（`hirobot.devices.audio`，驱动在 `hirobot.drivers.audio` 中）

麦克风和扬声器在 `embodiment.microphones` / `embodiment.speakers` 下声明。`make_robot` 把每个带
`driver:` 的设备以 `cls(name, **spec)` 构建，并挂接到 `RobotAssembly` 上。硬件设备会被填入本体的
`network_interface`。驱动是仿真驱动时，只构建类设置了 `simulated = True` 的设备，其余跳过。音频不进入
`Observation`。所有 PCM 都是单声道 int16 小端。

| 名字 | 说明 |
|---|---|
| `register_audio(name)`, `get_audio(name)`, `available_audio()` | 麦克风和扬声器共用的注册表 |
| `MicrophoneDriver(name)` | 抽象类：`open()`、`read(timeout=None) -> Optional[AudioChunk]`（最早的先返回；超时或关闭后返回 `None`）、`close()`（幂等）、`sample_rate` |
| `QueuedMicrophone(name, queue_s=10.0, retry_s=0.2)` | 在后台线程上运行阻塞的 `_receive() -> Optional[bytes]`，最多缓存 `queue_s` 秒的音频，超出部分丢弃最早的 chunk。子类实现 `_receive`，可选实现 `_open` / `_close`。提供 `wait_for_audio(timeout)`、`error`、`chunks`、`dropped` |
| `SpeakerDriver(name)` | 抽象类：`open()`、`play(pcm, sample_rate=None) -> float`（阻塞；返回播放的秒数，被停止时会更短）、`stop()`（可从另一个线程调用）、`close()`、`sample_rate` |
| `AudioChunk` | Dataclass：`pcm`、`sample_rate`、`stamp`（主机接收时间）、`duration` |
| `resample(pcm, src_rate, dst_rate)` | 用于语音的线性插值重采样 |
| `unitree_voice_mic` → `UnitreeG1Microphone(name, network_interface=None, local_ip=None, group="239.168.123.161", port=5555, queue_s=10.0)` | G1 麦克风：音频板的 UDP 组播，16 kHz |
| `unitree_voice_speaker` → `UnitreeG1Speaker(name, network_interface=None, domain_id=0, volume=None, app_name="hirobot", chunk_s=1.0, lead_s=1.0, timeout_s=2.0)` | G1 扬声器：SDK 的 `voice` 服务，16 kHz。`volume` 在 `open()` 时设置，在 `close()` 时恢复；`set_volume(v)` 立即修改音量 |
| `wav_mic` → `WavMicrophone(name, path, chunk_ms=20, realtime=True, tail_silence=1.0)` | 把一个 16-bit WAV 文件送入队列（有 `realtime` 时按说话速度，没有时一次送完），追加 `tail_silence` 秒静音，之后 `read()` 返回 `None`。仿真类 |
| `wav_speaker` → `WavSpeaker(name, path, realtime=True, sample_rate=16000)` | 把播放的所有内容录进一个 WAV 文件，在 `close()` 时写出。有 `realtime` 时，`play` 耗时与音频时长相同，`stop()` 会截断它。仿真类 |
| `null_speaker` → `NullSpeaker(name, realtime=True, sample_rate=16000)` | 什么都不播放；有 `realtime` 时耗时与音频时长相同。仿真类 |
| `sounddevice_mic` → `SoundDeviceMicrophone(name, sample_rate=16000, chunk_ms=20, device=None)` | 计算机的输入设备。需要 `audio` extra。仿真类（可与仿真器一起使用） |
| `sounddevice_speaker` → `SoundDeviceSpeaker(name, device=None, block_ms=50)` | 计算机的输出设备，按 `block_ms` 分块写入，让 `stop()` 能及时生效。需要 `audio` extra。仿真类 |
| `build_audio(microphones, speakers, network_interface=None, sim_only=False) -> (mics, speakers)` | 为带 `driver:` 的设备构建驱动，规则与 `build_cameras` 相同 |
| `read_wav(path) -> (pcm, rate)`, `wav_bytes(pcm, rate)` | `hirobot.drivers.audio` 中的 16-bit WAV 辅助函数（单声道；多声道文件取第一个声道） |
