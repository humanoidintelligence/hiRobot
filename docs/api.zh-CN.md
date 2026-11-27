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
- [机载 skill](#机载-skill)
- [命令仲裁](#命令仲裁)
- [SafetyGate](#safetygate)
- [RobotAssembly](#robotassembly)
- [控制循环](#控制循环)
- [DofMap](#dofmap)
- [特征向量](#特征向量)
- [录制](#录制)
- [Kinematics](#kinematics)
- [仿真资产](#仿真资产)
- [配置文件参考](#配置文件参考)
- [CLI](#cli)
- [MCP 服务器](#mcp-服务器)
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
| 旁听（录制、监控） | `add_listener(on_observation=None, on_command=None) -> ListenerHandle` | [旁听回调](#旁听回调)、[特征向量](#特征向量) |
| 技能 | `list_robot_skills()`、`run_robot_skill(name, args=None)`、`stop_robot_skill()`、`robot_skill_running` | [技能](#技能) |
| 按类别的技能 | 机载 skill：`has_onboard_skills`、`list_onboard_skills()`、`run_onboard_skill(name, args=None)`、`stop_onboard_skill()`。层技能：`list_layer_skills()`、`run_layer_skill(name, args=None)`、`stop_layer_skill()`、`layer_skill_running` | [机载 skill](#机载-skill)、[Layer](#layer) |
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
| `Embodiment`, `Joint`, `EndEffector`, `EndEffectorFrame`, `Camera`, `CameraMount`, `CameraIntrinsics`, `CameraInfo`, `AudioDevice`, `ModelRef`, `EmbodimentKind` | pydantic 模型 / 枚举 | `hirobot.core.embodiment` | 声明式的机器人 I/O 契约 |
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
| `model_path`, `available_configs` | 函数 | `hirobot` | 配置里唯一本体的模型文件；内置配置的名字 |
| `require_sdk` | 函数 | `hirobot.loading.sdk` | 按驱动的 `manifest.yaml` 检查其厂商 SDK |
| `RobotSkillSpec`, `RobotSkillResult`, `RobotSkillError` | pydantic 模型 / 异常 | `hirobot.core.robot_skills` | 描述、启动和拒绝机载 skill（行走、手势） |
| `SafetyGate`, `SafetyVerdict`, `SafetyError` | 类 / dataclass / 异常 | `hirobot.control.safety` | 在每个动作到达硬件之前检查它 |
| `DofMap` | 类 | `hirobot.core.dof` | 按名字重排关节向量 |
| `CommandRecord`、`ListenerHandle` | frozen dataclass / 类 | `hirobot.control.arbiter` | `on_command` 回调收到的记录；`add_listener` 的返回值（见[旁听回调](#旁听回调)） |
| `observation_features`、`flatten_observation`、`action_features`、`flatten_action`、`image_features` | 函数 | `hirobot.core.features` | 把 `Observation` / `Action` 拍平成带名字的向量（见[特征向量](#特征向量)） |
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

### `model_path(config) -> Optional[ModelRef]`

配置里唯一那个本体的模型文件，不构建机器人：除了本体个数，和 `load_embodiment(config).model` 一样。
配置没有本体或有多个本体时抛出 `ValueError`（信息里列出各本体的路径）。返回 `None` 表示只有一个本体，
但它的设备文件没写 `model:`。不检查文件是否存在。配置加载不了时抛出 `load_embodiment` 抛的异常
（`ValueError`、`FileNotFoundError`），所以要区分这几种情况的调用方先调 `load_embodiment`：

```python
import os
import hirobot

try:
    hirobot.load_embodiment(config)
except Exception as e:                # the config does not load
    ...
try:
    ref = hirobot.model_path(config)
except ValueError as e:               # no body, or more than one
    ...
if ref is None:                       # one body without model:
    ...
elif ref.kind == "mjcf" and os.path.isfile(ref.path):
    ...
```

### `available_configs(group=None) -> List[str]`

内置配置的名字，已排序：`robots/unitree_g1`、`body/unitree/g1`……`group`（`robots`、`body`、
`end_effector`、`camera`、`microphone`、`speaker`）只列一个目录。和 `hirobot configs` 打印的一样。

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

仿真相机和真机相机一样按自己的帧率出图：仿真时间到了下一帧的时刻才渲染新的一帧，间隔 `1 / fps`（用它的
`Camera.fps`；没有时用 `sim.camera_fps`，默认 30）。两帧之间的观测带着同一个图像对象和同一个时间戳（`t`、`seq`），
所以图像要当只读用。`camera_fps: null` 让没有 `fps` 的相机每次观测都渲染。`connect`、`reset` 或 `set_state`
之后的第一次观测一定渲染。单独使用的 `mujoco` 本体也有同名的驱动选项 `camera_fps`。

要在上真机之前先在仿真中预演一个动作，也可以用 `read_only: true` 打开真机，用它的 `*_mujoco` 配置另建一个机器人，
再用 `MujocoRobot.set_state` 让它从真机姿态起步（见 `examples/rehearse_in_sim.py`）。

以下情况抛出 `ValueError`：相机 / 末端执行器 / 音频 / layer 规格没有 `type`、末端执行器驱动不支持配置要求的接口、
`layers:` 不是列表，或扬声器驱动被列为麦克风（或反过来）。驱动名或 layer 名未知时抛出 `KeyError`。

### Layer

layer 是一个包装另一台机器人、改变其指挥方式的 `Robot`：例如一个全身控制器，接受 `BASE_VELOCITY` /
`MOTION_CLIP` / `CARTESIAN`，每个 tick 向下层机器人发送保持平衡的关节目标。hiRobot 自带一个，即 IK 层
[`mink_ik`](#mink_ik内置-ik-层)；其余由插件包（hiWBC）注册。

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

**层技能。** 层可以跑自己的技能（行为、宏动作），跨很多个控制周期，和厂商的 onboard skills 分开：`list_layer_skills()`（本层和
下面各层的）、`run_layer_skill(name, args=None) -> RobotSkillResult`（参数按 schema 检查，启动后返回）、`stop_layer_skill()`、
`layer_skill_running`（本层或下面的层有技能在跑就为真）。这四个在每个 `Robot` 上都有，调用方不用先判断最外层是不是层。技能运行期间层以 `Priority.LAYER_SKILL` 持有栈的[命令租约](#命令仲裁)：调用方的租约被抢占，直接发命令抛
`RobotBusyError`，直到技能结束或调 `stop_layer_skill()`。`TAKEOVER` 会抢占技能（技能被停下）。`estop()` 会结束技能，调用方才能
`reset()`。

#### `mink_ik`（内置 IK 层）

`mink_ik`（`hirobot.layers.MinkIkLayer`）接收末端目标、发出关节目标：每个 `CARTESIAN` 或 `CARTESIAN_DELTA` 动作在机器人的
MuJoCo 模型上跑几步微分 IK（[mink](https://github.com/kevinzakka/mink)），再经 `inner.send_action` 发一个含全部关节的
`POSITION` 动作，所以下面的安全门会检查它。其他模式原样透传。安装：`pip install 'hirobot[ik]'`（mink 要 Python 3.11+）；
没装时 `hirobot` 照常导入，层也照常注册，构建它时抛 `ImportError`，提示要装的 extra。

```yaml
layers:
  - type: mink_ik
    model: unitree_g1           # MJCF path, or a config whose body has model: mjcf:
    frames:                     # optional; default: every end-effector with a frame
      right_hand: {name: right_wrist_yaw_link, type: body}   # body | site | geom
```

| 键 | 默认值 | 含义 |
|---|---|---|
| `model` | 仿真的模型 | MJCF 路径（展开 `~`），或某个配置的名字 / 路径，该配置唯一的本体带 `model: mjcf:`。不给时，层在 `connect()` 时用下面仿真机器人的 MuJoCo 模型；真机必须给 |
| `frames` | 每个带 `frame` 的末端 | 末端名 -> MJCF body、site 或 geom 的 `{name, type}` |
| `dt` | 一个控制周期 | 一次 `send_action` 积分的秒数 |
| `iterations` | `3` | 每次 `send_action` 的 QP 步数 |
| `position_cost`、`orientation_cost` | `1.0`、`0.5` | 末端任务的权重 |
| `posture_cost` | `0.01` | 把每个关节拉向上次重新取种时的姿态（让腿和腰不动） |
| `damping`、`lm_damping` | `0.001`、`0.001` | QP 阻尼；末端任务的 Levenberg-Marquardt 阻尼 |
| `max_joint_velocity` | `3.0` | 每个关节的 rad/s（或 m/s）；`null` 关掉速度限制 |
| `joint_limits` | `true` | 解保持在模型的关节范围内 |
| `solver` | `daqp` | QP 求解器名（qpsolvers） |

- **坐标系。** 目标和层报告的 `ee_poses` 都在机器人基座系：关节所在那棵树的根 body（人形的骨盆、机械臂的基座连杆），与 MuJoCo
  驱动报告 `ee_poses` 的坐标系相同。浮动基座的自由关节在 QP 里被冻结，不参与求解。
- **目标。** `CARTESIAN`：`ee_targets[name]` 是 `[x, y, z, qx, qy, qz, qw]`。`CARTESIAN_DELTA`：每个 tick 一个
  `[dx, dy, dz, dax, day, daz]`（位置加基座系下的轴角旋转），加在该末端上一个目标上，而不是加在测量位姿上。动作里没写的末端保持上一个目标。
  第一次使用、`reset()`、`hold()` 以及透传过一条命令之后，目标从测量位姿重新取种。未知的末端或长度不对的目标抛 `SafetyError`。
- **其他关节。** 机器人有而 IK 模型没有的关节（手指）在 `direct_joints` 里，用 `Action.joint_names` / `joint_target` 设置。
  `gripper` 原样下传。
- **失败。** 求解器出错或结果非有限时，再发一次上一个关节目标并记一条警告，从不抛异常。够不到的目标不算错误：IK 在限制允许的范围内尽量靠近。
- **查看状态。** `layer.commanded_ee_poses` 是上一个关节目标的正运动学（它和 `ee_poses` 的差是跟踪误差，例如手臂在 PD 下下垂）；
  `layer.stats` 有 `count`、`mean_ms`、`max_ms`、`failures`。
- **关节目标。** `layer.commanded_joint_targets` 是关节名 -> 最近一次 IK 命令下发的 `POSITION` 目标，含该动作的全部关节
  （录制器每条命令后读它，把 IK 解出的关节目标和请求的 `CARTESIAN` 动作存在一起）。求解失败时保留重发的那个上一个目标。
  第一次 IK 命令之前、`reset()`、`hold()` 或透传过一条命令之后为 `None`。返回的是副本。

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
| `toy` | `ToyRobot` | `drivers/sim/toy/` | 玩具运动学积分器，无依赖。浮动基座的 toy 像 IMU 一样报告 `base_linear_acceleration`，静止时为 `[0, 0, 9.81]` |
| `mujoco` | `MujocoRobot` | `drivers/sim/mujoco/` | MuJoCo 物理仿真，本身就是一台机器人；按相机的 `fps` 渲染带 `mount` 的相机（没有 `fps` 的相机用 `camera_fps`，默认 30；`None` 为每次观测都渲染）；为带 `frame` 的末端执行器提供 `ee_poses`；`set_state` 让它从测得的姿态起步。有浮动基座时，`base_linear_acceleration` 是 IMU 加速度计的读数，与真实 Unitree G1 一致：基座坐标系下的比力（加速度减去重力），单位 m/s²，直立静止时约为 `[0, 0, 9.81]`；固定基座读数为 `[0, 0, 9.81]` |
| `unitree_sdk2` | `UnitreeRobot` | `drivers/body/unitree/sdk2/` | Unitree G1 / H1 / H1-2 / H2 |
| `fourier_grx` | `FourierRobot` | `drivers/body/fourier/grx/` | Fourier GR1T1 / GR1T2 |
| `agibot_gdk` | `AgibotG1Robot` | `drivers/body/agibot/gdk/` | AgiBot G1（精灵 G01）：手臂、腰、头和夹爪走 GDK 的 DDS topic；底盘走其控制盒 |
| `ros2` | `Ros2Robot` | `drivers/body/generic/ros2/` | 任何 ROS 2 机器人，写配置接入：读 `JointState` / `Imu` / `Odometry`；发 `JointTrajectory`、`Float64MultiArray` 或自定义消息（字段映射）。AgiBot X1 是它的一份配置（[指南](setup/ros2.zh-CN.md)） |
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
`run_robot_skill`、`run_onboard_skill`、`set_gains`、`connect`、`disconnect` 等）同一时刻只由一个持有者调：[命令租约](#命令仲裁)的持有者，没人持有时就是
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
| `estop()` | concrete | 立即停下；任何线程、任何时候都能调，不走安全闸，也不受[命令仲裁](#命令仲裁)限制。别的线程上的长调用（`reset` ramp、`connect` 里的等待）在下一步抛出 `RobotStoppedError`；机器人进入故障状态；正在下发的指令先发完（最多等 `ESTOP_LOCK_TIMEOUT_S`，0.2 s），之后不会再有指令发出；然后执行驱动自己的停止。Unitree 和 MuJoCo 进阻尼（Unitree 开着 `builtin_skills` 时改为停掉 skill），AgiBot G1 发 `MotionStop`，UR 减速停下（`servoStop` / `speedStop` / `stopJ`），Franka 停掉位置控制器；没有 `damp()` 的驱动（`toy`、`fourier_grx`、`ros2`）只拒绝后续指令。`RobotAssembly` 对它的本体和每个末端执行器、或每个部件都调急停。在 layer 上调时，先直接急停栈最底下的机器人，再从外往里让每一层停下自己的工作（见[层的钩子](#层的钩子)、[Layer](#layer)） |
| `claim(name, priority=Priority.CALLER, on_preempt=None, envelope=None, parts=None) -> CommandLease` | concrete | 拿机器人的命令流。见[命令仲裁](#命令仲裁) |
| `command_events() -> List[CommandEvent]` | concrete | 仲裁器的审计事件，从旧到新；没有仲裁器时为空。见[审计](#审计) |
| `add_listener(on_observation=None, on_command=None) -> ListenerHandle` | concrete | 每次顶层 `get_observation()` 和 `send_action` / `hold` 都回调一次。见[旁听回调](#旁听回调) |
| `command_timeout_s` | attribute | 多少秒没有新的 `VELOCITY` / `BASE_VELOCITY` 指令就停下运动并打 warning（`hirobot.control.watchdog.CommandWatchdog`）；不进故障态，下一条指令照常执行。`toy`、`mujoco`（仿真时间）、`unitree_sdk2`（零速、kp 0）、`ur_rtde`（`speedStop`）默认 0.2 s；`agibot_gdk` 的底盘有自己的 `command_timeout`（0.3 s）；其他驱动是 `None`。配置里写 `body.command_timeout_s: null` 关掉。位置目标不看 |
| `action_modes` | class attribute | 机器人接受的模式。构建动作的调用方读取它 |
| `motion_spaces` | class attribute | 机器人在 `MOTION_CLIP` 下能跟踪的 `MotionClip.space` 取值。默认为空 |
| `direct_joints` | class attribute | 调用方可以在非关节模式旁按名字指挥的关节。默认为空 |
| `cartesian_targets` | property | 接受 `CARTESIAN` 位姿目标的末端执行器：`CARTESIAN` 在 `action_modes` 中时为全部末端执行器，否则为空。只跟踪其中一部分的机器人会覆盖它 |
| `cameras`、`microphones`、`speakers` | attributes | 挂载的设备，按名字：相机驱动、麦克风和扬声器。裸驱动上为空；`make_robot` 建的 assembly 会填上，每一层 layer 往上传，所以最外层也有。见[扩展点](#扩展点相机末端执行器与音频) |
| `camera_info(name)` | method -> `CameraInfo` | 相机 `name` 的内参和安装位姿（外参），用来把 `images[name]` 和 `depth[name]`（米，已配准到彩色图）换成 3D。内参取第一个有的来源：配置的 `intrinsics:`；相机驱动（`connect()` 之后；RealSense、Orbbec、ZED 从设备读）；否则由安装位置的 `fovy` 推出理想针孔。结果会缩放到相机声明的 `width` / `height`。`mount` 是光学坐标系在某个机器人连杆中的位姿；头部、腕部相机还要乘上该连杆的当前位姿，这一步由调用方做。子部件的相机名为 `<部件>/<名字>`。相机名不存在时抛 `KeyError` |
| `accepts_parts` | class attribute | `send_action` 是否接受 `Action.parts`：带子部件的 assembly 为 `True`，其余为 `False` |
| `now()` | optional | 机器人时钟（秒），`Observation.stamps` 的时间在它上面：默认 `time.monotonic()`，`toy` / `mujoco` 是仿真时间。包装层返回内层机器人的 |
| `hold()` | optional | 让机器人保持在原处，例如在 policy 停止之后。默认情况下，驱动接受 POSITION 时把当前 `joint_pos` 作为 POSITION 动作发送，否则什么都不做。只接受笛卡尔增量的仿真器什么都不做；带 `builtin_skills` 的 Unitree 什么都不做（运动服务会让机器人保持站立）。有自己平衡回路的驱动（全身控制的人形机器人）必须覆盖它，指挥机器人站定（基座速度 `[0, 0, 0]`）：发送 `joint_pos` 会把关节冻住，人形机器人会摔倒。经过安全门 |
| `set_gains(kp, kd)` | optional | 每个关节的 PD 增益。默认什么都不做 |
| `has_onboard_skills` | property | 驱动是否提供机载 skill。默认 `False` |
| `list_onboard_skills() -> List[RobotSkillSpec]` | optional | 机器人自身控制器运行的 skill。默认为空。见[机载 skill](#机载-skill) |
| `run_onboard_skill(name, args=None) -> RobotSkillResult` | concrete | 按 skill 的 schema 检查 `args`，再执行。抛出 `RobotSkillError` |
| `stop_onboard_skill()` | optional | 停止正在运行的 skill。默认什么都不做 |
| `list_layer_skills() -> List[RobotSkillSpec]` | concrete | 栈里各层跑的技能。每个机器人都有；没有层时为空。见[层技能](#layer) |
| `run_layer_skill(name, args=None) -> RobotSkillResult` | concrete | 启动一个层技能。没有层时抛 `RobotSkillError` |
| `stop_layer_skill()` | concrete | 停止正在运行的层技能。没有层时什么都不做 |
| `layer_skill_running` | property | 是否有层技能（栈里任一层的）持有命令流。没有层时为 `False` |
| `list_robot_skills() -> List[RobotSkillSpec]` | concrete | 机器人能跑的所有技能，层技能和机载 skill 合在一张表里。见[技能](#技能) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | concrete | 按名字启动 `list_robot_skills()` 里的技能，不管是哪一种 |
| `stop_robot_skill()` | concrete | 停掉正在跑的技能，两种都停 |
| `robot_skill_running` | property | 是否有层技能在跑。机载 skill 不跟踪 |
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
| `battery_soc_limit` | `(warn, shutdown)` 或 `None` | `None` | 电池电量（百分比）：`warn` 是监控该标为电量低的值，`shutdown` 是不能再继续命令的值。两档都在 0-100 内，`warn >= shutdown`。`safety:` 没写 `min_battery_soc` 时安全门用 `shutdown` |
| `model` | `ModelRef` 或 `None` | `None` | 机器人唯一本体的模型文件，来自它设备文件的 `model:`。有多个本体或没写 `model:` 时为 `None`。由 `load_embodiment` 和 `make_robot` 填写，真机也有 |

属性：`joint_names -> List[str]`、`num_dofs -> int`。

以下情况校验会抛出 `ValueError`：关节名重复、末端执行器列出了未知关节、一个关节被两个末端执行器占用，
或末端执行器有 `driver` 却没有 `joints`。

### `Joint`, `EndEffector`, `Camera`, `AudioDevice`, `EmbodimentKind`

| 模型 | 字段 | 类型 | 含义 |
|---|---|---|---|
| `Joint` | `name` | `str` | 唯一名字；DoF 映射的键 |
| | `position_limit` | `(lower, upper)` or `None` | 位置上下限（rad）。末端执行器驱动按它裁剪 |
| | `torque_limit` | `float` or `None` | 最大绝对力矩（Nm） |
| | `axis` | `(x, y, z)` or `None` | 关节转轴的单位向量，在该关节带动的 link 的坐标系里（同 URDF `<axis>` 和 MJCF `axis`），不是世界坐标系 |
| | `parent_link` | `str` or `None` | 关节挂在哪个 link 上（URDF `<parent>`） |
| | `temperature_limit` | `(warn, shutdown)` or `None` | 电机温度（摄氏度）：`warn` 是监控该标为过热的温度，`shutdown` 是不能再继续命令的温度。两档都 > 0，`warn <= shutdown`。`safety:` 没写 `max_joint_temperature` 时安全门用 `shutdown` |
| `EndEffector` | `name` | `str` | `ee_poses` / `ee_targets` / `gripper` 中的键 |
| | `has_gripper` | `bool` | 有通过 `Action.gripper` 控制的夹爪。末端执行器驱动不列 `joints`（`opening` 接口）时必须为 true，列了 `joints` 时不能为 true |
| | `joints` | `List[str]` | 该末端执行器驱动的 embodiment 关节（手指关节），按其驱动的电机顺序排列 |
| | `driver` | `dict` or `None` | 末端执行器驱动规格：`type` 指定一个已注册的末端执行器驱动，其他键是它的参数。需要 `joints` 或 `has_gripper: true` |
| | `joint_map` | `Dict[str, List[JointMapEntry]]` | 只用于仿真：每个手部关节驱动的模型关节，各自到 `offset + scale * value`（`joint`，`scale` 默认 1，`offset` 默认 0）；读数取第一个。没有条目的关节驱动同名的模型关节。真实手的驱动忽略它；`safety.sim_check` 目前拒绝它 |
| | `frame` | `EndEffectorFrame` or `None` | 读取位姿的连杆：`body`（连杆名）、`pos`（m，在该连杆中）、`quat` `[x, y, z, w]`（在该连杆中）。`mujoco` 本体为带 frame 的末端执行器填写 `ee_poses`；SDK 自己报告位姿的驱动（UR、Franka）忽略它；`Kinematics` 由关节角算出同一个位姿 |
| `Camera` | `name` | `str` | `Observation.images` 中的键 |
| | `width`, `height` | `int` | 交付的画面尺寸 |
| | `driver` | `dict` or `None` | 相机驱动规格：`type` 指定一个已注册的相机驱动，其他键是它的参数。由本体驱动自己渲染的相机（仿真器）省略此项。旧键 `source` 会报错 |
| | `depth` | `bool` | RGB-D 相机：其驱动还会交付一张与彩色图像配准的深度图，放在 `Observation.depth` 中同名键下。默认 `False` |
| | `fps` | `float` or `None` | 标称帧率。`stale_sources` 用它判断过期；`control_hz` 超过它两倍时 `make_robot` 警告。仿真相机按这个帧率渲染（没有时按 `sim.camera_fps`） |
| | `mount` | `CameraMount` or `None` | 相机的安装位置：`body`（连杆名）、`pos`（m，在该连杆中）、光学坐标系（z 向前，x 向右，y 向下）在该连杆中的 `quat` `[x, y, z, w]`、`fovy`（垂直视场角，度）。这就是相机的外参：`mujoco` 本体按这个位姿把相机渲染进 `images`；对真实相机，它是标定出的位姿，用 `Robot.camera_info` 读回。配置里由相机的 `placement` 给出 |
| | `intrinsics` | `CameraIntrinsics` or `None` | 标定得到的固定内参（相机文件的 `intrinsics:`）。比驱动从设备读到的优先 |
| `CameraIntrinsics` | `width`, `height` | `int` | 这套内参对应的图像尺寸。配置里默认取相机的 `width` / `height` |
| | `fx`, `fy`, `cx`, `cy` | `float` | 针孔模型的焦距和主点，单位像素。像素 `(u, v)` 原点在左上角，`u` 向右，`v` 向下，像素中心在整数坐标上。光学坐标系（OpenCV：z 向前，x 向右，y 向下；单位 m）中的点 `(x, y, z)` 在畸变前投影到 `u = fx·x/z + cx`、`v = fy·y/z + cy` |
| | `model`, `coeffs` | `str`, `tuple` | 畸变：`none`（默认，无系数）、`brown_conrady` / `plumb_bob`（OpenCV 的 `k1, k2, p1, p2, k3`）、`inverse_brown_conrady`（RealSense，同样 5 个）、`kannala_brandt`（OpenCV 鱼眼 `k1..k4`）。系数全为 0 时算作 `none` |
| | `K`, `scaled(w, h)`, `from_fovy(w, h, fovy)` | | `K` 是 3x3 的 numpy 相机矩阵。`scaled` 给出同一相机在另一分辨率下的内参（整图缩放，不裁剪）。`from_fovy` 是主点在图像中心的理想针孔，即仿真器渲染出的相机 |
| `CameraInfo` | `name`, `intrinsics`, `intrinsics_source`, `mount`, `depth` | | `Robot.camera_info` 的返回值。`intrinsics_source` 为 `config`、`driver`、`fovy` 或 `None` |
| `AudioDevice` | `name` | `str` | `robot.microphones` / `robot.speakers` 中的键 |
| | `driver` | `dict` or `None` | 音频驱动规格：`type` 指定一个已注册的麦克风或扬声器驱动，其他键是它的参数 |
| `ModelRef` | `kind` | `"mjcf"` 或 `"urdf"` | `path` 的格式。`model:` 两个都有时取 `mjcf`。不可修改，可哈希；以后加的字段都是可选的 |
| | `path` | `str` | 绝对路径，`~` 已展开。不检查文件是否存在 |
| | `package_roots` | `Dict[str, str]` | 只用于 URDF：包名 -> 绝对目录，用来解析 `package://` 网格路径。MJCF 时为空 |

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
[机载 skill](#机载-skill)（厂商控制器跑）和[层技能](#layer)（某一层跑）；下面四个调用把两种一起列出、
一起启动，调用方不用区分。

```python
robot = make_robot("my_g1.yaml")
robot.connect()
for s in robot.list_robot_skills():          # RobotSkillSpec：name、description、parameters（JSON Schema）……
    print(s.name, s.description)
robot.run_robot_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_robot_skill()
```

| 调用 | 说明 |
|---|---|
| `list_robot_skills() -> List[RobotSkillSpec]` | 先列层技能，再列机载 skill。层技能和机载 skill 同名时，层技能盖住机载 skill（外层优先）。组装出来的机器人，名字带部件路径（`left/grasp`） |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | 转给 `run_layer_skill` 或 `run_onboard_skill`，参数检查和租约规则照旧：层技能在结束前持有命令流。名字未知或参数错误抛 `RobotSkillError`，别人持有机器人时抛 `RobotBusyError` |
| `stop_robot_skill()` | 两种都停：先 `stop_layer_skill()`，再 `stop_onboard_skill()` |
| `robot_skill_running` | 就是 `layer_skill_running`。机载 skill 的 `run_robot_skill` 在机器人接受命令时就返回，跑多久只有估计值（`duration_s`） |

`CommandLease` 也有 `run_robot_skill`。机载 skill 经由租约执行；层技能会抢占这个租约，技能结束后要重新 claim。

驱动和层不实现这四个。驱动通过 `list_onboard_skills` / `_run_onboard_skill` 提供机载 skill，层通过
`_layer_skills` / `_start_layer_skill` 提供层技能（[SPI](#编写驱动和层spi)）。

## 机载 skill

`hirobot.core.robot_skills`。机载 skill 是由机器人自身控制器完成的动作（行走、坐下、挥手），按名字启动，
而不是靠流式发送关节目标。每个驱动声明 hiRobot 在其机器人上支持的 skill，所以调用方（agent、语音前端）
可以列出它们并展示给模型，不必写死。

```python
robot = make_robot("my_g1.yaml")      # robot.builtin_skills: g1
robot.connect()
[s.name for s in robot.list_onboard_skills()]   # ['start_locomotion', 'walk', ...]
robot.run_onboard_skill("start_locomotion")
robot.run_onboard_skill("walk", {"vx": 0.3, "seconds": 2.0})
robot.stop_onboard_skill()
```

### `RobotSkillSpec`

| 字段 | 类型 | 说明 |
|---|---|---|
| `name` | `str` | 传给 `run_onboard_skill` |
| `description` | `str` | 写给模型看的一两句话 |
| `parameters` | `dict` | 参数的 JSON Schema（一个 `object` schema）。为空表示没有参数 |
| `requires` | `List[str]` | 用自然语言写的前提条件，例如 `"robot standing on the ground"`。仅供参考 |
| `duration_s` | `float` or `None` | 典型运行时长（如果已知） |

### `run_onboard_skill(name, args=None) -> RobotSkillResult`

在任何内容到达机器人之前按 `parameters` 检查 `args`，然后交给驱动执行。支持的 schema
关键字：属性的 `type`（number / integer / boolean / string）、`minimum`、`maximum`、
`exclusiveMinimum`、`enum`、`default` 和 `required`。缺少的参数取其 `default`；未知参数会被拒绝。
它在机器人接受指令后就返回，而不是等动作结束。`RobotSkillResult` 包含 `name`、检查后的 `args`
（已填入默认值）和 `duration_s`：驱动按这次参数给了估计值就用它（G1 `walk` 是它的 `seconds`；`turn` 按角度估的上限），
否则用 spec 的 `duration_s`。

skill 未知、参数错误或机器人拒绝时抛出 `RobotSkillError`。

### 包装器

`RobotAssembly` 把这四个调用转发给其本体驱动，并把每个子部件的 skill 列在该部件的前缀下（`<part>/<skill>`）。

### 带 skill 的驱动

| 驱动 | 配置 | Skill |
|---|---|---|
| `unitree_sdk2` | `builtin_skills: g1` | G1 行走和手臂手势，见 [Unitree 设置指南](setup/unitree.zh-CN.md#机载-skillg1) |

其他驱动都不提供 skill。

## 命令仲裁

同一时刻只有一个调用方给机器人发命令。`make_robot` 在栈的最外层挂一个 `CommandArbiter`（`hirobot.control.arbiter`）；调用方用 `claim`
拿到命令流，经返回的 `CommandLease` 发命令：

```python
from hirobot import Priority, PreemptedError

with robot.claim("runner", on_preempt=lambda lease: stop_runner()) as r:
    r.send_action(action)       # 还有 r.hold()、r.reset()、r.run_robot_skill(...)
```

| 项 | 含义 |
|---|---|
| `robot.claim(name, priority=Priority.CALLER, on_preempt=None, envelope=None) -> CommandLease` | 拿命令流。高优先级立即抢占低优先级；同级或更低抛 `RobotBusyError` |
| `Priority` | `TAKEOVER`（人工接管：遥操作、手柄）> `LAYER_SKILL`（层技能运行期间）> `CALLER`（应用） |
| 有人持有时直接调 | 机器人上的 `send_action`、`hold`、`reset`、`run_onboard_skill` 抛 `RobotBusyError`（`RuntimeError` 子类）。`estop`、`get_observation`、`get_status`、`now` 不检查 |
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

### 旁听回调

`robot.add_listener` 让录制器或监控程序看到机器人读到什么、被要求做什么，自己不在命令通路上。
在最外层机器人（`make_robot` 返回的）上调用；在单个驱动上调用时，会像 `claim` 一样挂一个仲裁器。

```python
import queue

q: "queue.Queue" = queue.Queue()
handle = robot.add_listener(on_observation=q.put, on_command=q.put)
...
handle.remove()
```

| 项 | 含义 |
|---|---|
| `on_observation(obs)` | 这个机器人上每次顶层 `get_observation()`，参数是返回的 `Observation`。命令内部的读取（`hold()` 读当前姿态）不回调 |
| `on_command(record)` | 每次顶层 `send_action` 和 `hold`，直接调用或经租约，放行或被拒都算。`hold()` 内部发的 `send_action` 不回调，`reset`、skill、层往下发的命令也不回调 |
| `CommandRecord` | `t`（调用开始时的 `now()`）、`call`（`send_action` / `hold`）、`action`（调用方传入的原样，包络或 gate 截断之前；`hold` 为 `None`）、`lease` 和 `priority`（直接调用为 `None`）、`accepted`、`reason`（`SafetyError`、`RobotBusyError`、`RobotFaultError` 或驱动错误的消息）、`parts`（租约的部件路径） |
| 线程 | 回调在调用方线程上、调用结束后同步执行，必须很快返回：把数据放进队列。任何线程都可以添加、移除回调 |
| 错误 | 回调抛出的异常只记日志（每个回调每 10 s 最多一条；`handle.errors` 计数），不改变调用结果 |
| `ListenerHandle` | `remove()`（调两次也行）、`active` |

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

没给 `min_battery_soc` 和 `max_joint_temperature` 时，`from_embodiment` 从本体取：`Embodiment.battery_soc_limit[1]`
和所有 `Joint.temperature_limit[1]` 里最小的那个。`safety:` 里写了的值优先。

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
| Skill | 部件的 skill 列为 `<part>/<skill>`；`run_onboard_skill("g1/wave")` 在该部件上运行它 |
| 生命周期 | `connect` 先连接各部件，再连接节点自己的相机，失败时撤销。`reset`、`hold`、`damp`、`estop`、`disconnect`、`stop_onboard_skill` 即使某个部件失败也会作用到每个部件，然后抛出第一个错误。`reset` 也会清除故障状态 |
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

## 特征向量

`hirobot.core.features` 把 `Observation` 或 `Action` 拍平成 float32 向量，每一维带名字，即 LeRobot 这类数据集存的格式。
关节按 `Embodiment.joints` 的顺序；组装体里的名字带 `<part>/` 前缀。

```python
from hirobot import flatten_observation, observation_features

names = observation_features(robot.embodiment)   # {"observation.state": ["j0.pos", ...], ...}
obs = robot.get_observation()
row = flatten_observation(obs, robot.embodiment)  # {"observation.state": array([...]), ...}
```

| 键 | 值 | 名字 |
|---|---|---|
| `observation.state` | `joint_pos`，接着每个带夹爪的末端的 `gripper` | `<joint>.pos`、`<ee>.gripper` |
| `observation.ee_pose` | `ee_poses`，每个末端 7 维 | `<ee>.x` ... `<ee>.qw` |
| `observation.joint_vel`、`observation.joint_torque` | `joint_vel`、`joint_torque` | `<joint>.vel`、`<joint>.torque` |
| `observation.imu` | `base_orientation` `[x, y, z, w]`、`base_angular_velocity`、`base_linear_acceleration` | `imu.qx` ... `imu.az`（10 维） |
| `observation.base` | `base_position`、`base_linear_velocity` | `base.x` ... `base.vz`（6 维） |
| `action` | 关节模式：`joint_target`，接着各夹爪。非关节模式旁带的关节：`joint_names` 给出的目标，按全部关节排，其余为 `NaN` | `<joint>.pos`（对应模式下为 `.vel`、`.torque`）、`<ee>.gripper` |
| `action.gripper` | 非关节模式下的夹爪 | `<ee>.gripper` |
| `action.ee`、`action.ee_delta` | `CARTESIAN`（每个 7 维）/ `CARTESIAN_DELTA`（每个 6 维）下的 `ee_targets` | `<ee>.x` ... `<ee>.qw` / `<ee>.dx` ... `<ee>.daz` |
| `action.base_velocity`、`action.base_target` | `base_velocity`、`base_target`（`BASE_POSE`） | `base.vx`、`base.vy`、`base.wz` / `base.x`、`base.y`、`base.yaw` |

| 函数 | 返回 |
|---|---|
| `observation_features(embodiment, obs=None)` | 键 -> 名字。不给 `obs` 时是这个 embodiment 可能有的全部键（`imu` 和 `base` 只在浮动基座时有）；给了就是它填了的键 |
| `flatten_observation(obs, embodiment)` | 键 -> 向量。长度不对抛 `ValueError` |
| `action_features(embodiment, mode=POSITION, action=None)` | 键 -> 名字，按 `mode` 或按 `action` |
| `flatten_action(action, embodiment)` | 键 -> 向量。`Action.parts` 按机器人里的名字拍平；各叶子必须同一种模式 |
| `image_features(embodiment)` | `observation.images.<cam>` -> 相机名。图像不拍平 |

观测或动作没填的来源，对应的键就不出现。出现的键总是完整长度；其中缺的部分（某个末端的位姿、IMU 的加速度、某个夹爪）为 `NaN`。
`kp` / `kd`、`latent`、`motion`、`extra` 不拍平。

## 录制

`hirobot.record` 把 episode 存成 LeRobot v3.0 数据集，不 import lerobot（`pip install 'hirobot[record]'`：pyarrow 和 PyAV）。
`Recorder` 是最外层机器人上的一个[旁听回调](#旁听回调)：它从不给机器人发命令，它出错也不会影响控制循环。

```python
from hirobot.record import Recorder

rec = Recorder(robot, "data/")           # fps：min(30, control_hz)；没有相机时为 control_hz
rec.subscribe(lambda e: print(e.state, e.path, [f.name for f in e.flags]))
rec.start("pick the cube", meta={"operator": "ann"})
...                                      # 控制循环照常跑
rec.stop()                               # 立刻返回；episode 在后台保存
rec.close()                              # 把队列里的存完
```

| 项 | 含义 |
|---|---|
| `Recorder(robot, root, *, fps=None, max_queue=15, min_duration_s=1.0, max_buffer_mb=4096, max_samples=1_000_000, copy_images=False, robot_config=None, hold_after_s=0.5, frozen_s=1.0, max_queue_mb=8192, worker="process", ring_mb=256)` | 没装 pyarrow 或 PyAV 时抛 `ImportError`。`fps` 必须是整数。图像像素超过 `max_buffer_mb`、或读数超过 `max_samples` 后，这个 episode 不再存新的。`copy_images` 仍可传但不起作用：存下的每张图本来就会复制进帧环 |
| `worker` | episode 在哪里保留和保存。`"process"`（默认）：一个 Python 子进程，第一次 `start()` 时启动并复用，对齐和编码不占控制进程的 GIL。`"thread"`：本进程的一个线程。没有 `memfd`（非 Linux），或子进程连续两次没就绪就退出时，录制器记一条警告并改用 `"thread"` |
| `ring_mb` | 新图像送往 worker 途中复制进的共享内存环的大小（至少能放下声明的最大相机的 16 帧），在构造时分配并逐页写一遍。环满时到达的帧被丢弃，episode 标红 `dropped_samples` |
| `Recorder.worker` | `"process"` 或 `"thread"`：现在在哪里保存（算上回退） |
| `start(task, meta=None) -> int` | 开始一个 episode，返回编号。正在录、或等着保存的停下的 episode 已有 `max_queue` 个或占的图像超过 `max_queue_mb` 时抛 `RecorderError`。`meta` 原样保存 |
| `stop() -> int` / `cancel()` | `stop` 把 episode 放进保存队列；`cancel` 丢弃 |
| `subscribe(callback) -> Subscription` | 在录制器的分发线程上调 `callback(SaveEvent)`（不是控制循环的线程，也不是 worker 的）：先 `saving`，再 `saved`（带 `path`、`flags`、`frames`）或 `failed`（带 `error`）。回调慢只会推迟后面的事件，不影响录制 |
| `wait(timeout=None)`、`close(timeout=None)`、`recording`、`queue_length` | `close` 停下正在录的 episode 并保存（元数据里 `"stopped_by": "close"`），结束 worker，送出最后的事件，然后结束录制器的线程、释放帧环。超过 `timeout` 后，子进程仍会存完已经收到的 episode，即使本进程已退出；还没交给它的样本会丢失。没调 `close()` 就丢掉的录制器会被垃圾回收，它的线程和 worker 也按同样方式结束 |
| `append_verdict(path, episode, verdict, note="", reviewer=None)` | 给 `meta/qc.jsonl` 加一行人工 `PASS` / `FAIL` / `UNREVIEWED`；`read_qc(path)` 读回各行 |

录制时旁听回调每个新的 `stamps[source].seq` 存一个状态样本（不带 `images`、`depth` 和 `extra`，数据集不读它们），
每个新的相机 `seq` 把一张图复制进帧环；一个发送线程把两者流式交给 worker。`stop()` 之后，worker 把所有数据对齐到 `1/fps` 的网格上，网格覆盖所有流都有数据的时段（连续量线性插值，四元数归一化，夹爪取帧时刻及之前的最近读数，
命令和它所回应的那次状态读数配对，图像取最近的一帧），每路相机编码成 H.264，每个 episode 写一个目录：

```
<root>/<YYYYmmdd-HHMMSS>_<task>_<NNNN>/
  meta/info.json, meta/stats.json, meta/tasks.parquet, meta/episodes/chunk-000/file-000.parquet
  data/chunk-000/file-000.parquet
  videos/observation.images.<cam>/chunk-000/file-000.mp4
  meta/hi/info.json           # hi.action_mode, hi.robot_config, hi.hirobot_version, hi.control_hz, hi.recorder_fps
  meta/hi/alignment.json      # 各流、图像偏差、裁掉的时间、命令、租约事件
  meta/hi/episode.json        # 任务、meta、起止时间
  meta/qc.jsonl               # 保存时的标红项，之后是人工结论
```

`meta/info.json` 只有 LeRobot 自己的键。录制器的数据集级键在 `meta/hi/info.json` 里，是一个扁平的 JSON 对象，键保留 `hi.` 前缀
（LeRobot 加载数据集时会丢掉 `info.json` 里不认识的键）。

状态和动作列就是[特征向量](#特征向量)（`observation.state`、`action` 等）。录制器另加：

| 列 | 含义 |
|---|---|
| `action` | 这个 episode 里最多的那种模式的命令（`meta/hi/info.json` 的 `hi.action_mode`）。第 `k` 行是这一帧的状态读数之后、下一次读数之前发出的第一条命令，所以第 `k` 行的状态就是这条命令回应的状态；那段时间没有命令时取仍在生效的那条。状态读数比网格慢时（每个动作块只观测一次的循环），显示同一次读数的后续帧取自己这一帧时段内发出的第一条命令，没有时取帧时刻仍在生效的那条。`hold()` 之后，以及 `hold_after_s` 内没有命令而算作 `hold` 的帧（命令看门狗的 hold 不会上报），位置类目标（`POSITION`、`PD`、`CARTESIAN`、`BASE_POSE`）和夹爪保持最后的值，速度、每拍增量和力矩（`VELOCITY`、`TORQUE`、`CARTESIAN_DELTA`、`BASE_VELOCITY`，以及 `CARTESIAN` 旁的 `base_velocity`）为 0。处在别的模式命令下的帧为 `NaN`，episode 标红 `mixed_action_modes` |
| `hi.control_source` | 那条命令的来源：`policy`（直接调用或 `CALLER` 租约）、`teleop`（`TAKEOVER` 租约）、`skill`（`LAYER_SKILL`），或 `hold`：还没有命令、调了 `hold()`、或 `hold_after_s` 内没有命令 |
| `intervention` | 在同时有 `policy` 帧的 episode 里 `hi.control_source == "teleop"`；纯遥操作的 episode 全是 `false` |
| `hi.rejected_action`、`hi.rejected_action_mask` | 被拒的 `send_action`（被抢占、被安全包络或 gate 拒绝）只记在时段包含它的那一帧上，一段里有多条时取最后一条。一帧的时段从它的状态读数开始（前面已有帧显示同一次读数时从它的网格时刻开始），到下一帧时段开始为止。没有时 `hi.rejected_action` 为 `0.0`、mask 为 `false`；被拒的动作转换不成这些列时也是 `0.0`（mask 仍为 `true`）。判断哪一行有被拒动作要看 mask，不要看值。用 0 而不用 `NaN`，是因为 pandas（以及 LeRobot 的 `aggregate_datasets`）会把列表列里的 `NaN` 变成 null |
| `hi.capture_time` | 帧时刻及之前那次状态读数的机器人时钟时间 |
| `hi.stamp.<source>.t`、`hi.stamp.<source>.seq` | 那次读数的时间戳，每个源都有，相机也有 |
| `hi.image_skew.<cam>` | 图像时间减帧时间，单位秒 |
| `hi.ik_joint_target` | 这一帧的命令对应的 IK 层下发的关节目标（机器人有 `commanded_joint_targets` 时才有） |

保存时的标红项，在 `SaveEvent` 和 `meta/qc.jsonl` 里（有任何一项时结论为 `FAIL`）：`too_short`、`missing_camera`、`camera_gap`、
`frozen_camera`（帧在来但 32x24 缩略图 `frozen_s` 内没变）、`alignment`（超过 1% 的帧取到的图像偏差大于 `1/(2 fps)` 秒）、
`invalid_state`、`dropped_samples`（到了缓冲上限、帧环满了，或有 worker 收不下的样本）、`fault`（`stop()` 时机器人报了 `start()` 时没有的故障）、`no_commands`、
`mixed_action_modes`、`unrecorded_action`。

正在录的 episode 和停下但还没存完的 episode 的原始图像都在 worker 里，1080p 每帧约 6 MB。一个 episode 最多留 `max_buffer_mb`。
停下但还没存完的 episode 占的图像超过 `max_queue_mb` 时 `start()` 拒绝开录，所以原始图像不超过 `max_queue_mb + 2 * max_buffer_mb`
（默认值下 16 GB）。控制进程自己只占帧环（`ring_mb`）。

仿真（`toy`、`mujoco`）只在发命令或装配体空转某个部件时推进时间，所以 `hold()` 之后只调 `get_observation()` 的循环一直读到同一个仿真时刻。
这些读数按 stamp 去重，录不出超过 `hold_after_s` 的帧，episode 在最后一条命令处结束。要在仿真里录 hold，就每拍调一次 `hold()`（或重发保持的目标）。
真机的时钟自己走，不用多调。

这个版本之前录的 MuJoCo episode，`observation.imu` 第 7 到 9 维（线加速度）是 `NaN`，不要和新录的 episode 合并。

**已知限制：大图像下的控制循环延迟。** 在 50 Hz 的 MuJoCo G1 + IK 循环、一路 1920x1080 相机、仿真在空闲 GPU 上渲染时测得：

- 录制期间单拍 p99 为 8.2 到 8.9 ms，最慢 14.3 ms，没有超过 20 ms 的拍。
- 保存期间单拍 p99 为 8.4 到 10.2 ms。
- `stop()` 之后控制进程每个 episode 花约 2 ms CPU（之前在本进程线程上保存时约 600 ms）。

如果 GPU 同时在驱动显示器，不录制时仿真同步读回 1080p 画面本身的 p99 就到了 23.7 ms。用 `MUJOCO_EGL_DEVICE_ID` 给 MuJoCo 的 EGL 渲染选一块空闲 GPU。
在 import numpy 之前设 `NUMPY_MADVISE_HUGEPAGE=0`，可以避免分配大帧数组时透明大页整理造成的卡顿。

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

kin = Kinematics.from_mjcf("~/.cache/hirobot/assets/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")
poses = kin.forward(obs.joint_pos, base_pos=[0, 0, 0.79], base_quat=[0, 0, 0, 1],
                    joint_names=robot.embodiment.joint_names)
pos, quat = poses["torso_link"]
```

## 仿真资产

`hirobot.assets` 从 Hugging Face 数据集 `humanoidintelligence/hirobot-assets` 下载仿真模型
（MJCF / URDF）到 `~/.cache/hirobot/assets`，保留数据集里的目录结构；内置的 `*_mujoco` 配置就指向
那里的文件。需要 `pip install 'hirobot[assets]'`（`huggingface_hub`）。import 和加载配置都不联网，
只有 `fetch` 联网。命令行是 `hirobot assets list` / `hirobot assets fetch`。

| 名字 | 说明 |
|---|---|
| `REPO_ID`, `REVISION` | 数据集，以及 `fetch` 下载它的哪个 tag |
| `repo_id() -> str` | `fetch` 用的数据集：设了 `$HIROBOT_ASSETS_REPO`（目录和 tag 都相同的另一个数据集，比如镜像）就用它，否则用 `REPO_ID` |
| `ASSETS` | `{名字: allow_patterns}`：每个模型的目录，加上来源的 `LICENSE`、`SOURCE.md` 和数据集的 `README.md` |
| `assets_dir() -> Path` | `~/.cache/hirobot/assets`，内置配置就在这里找 |
| `fetch(names, *, token=None) -> Path` | 下载一个名字或一组（`snapshot_download`，已有的文件不重下），返回 `assets_dir()`。未知名字报 `ValueError`，没装 `huggingface_hub` 报 `RuntimeError`。`token` 默认用 `hf auth login` 保存的或 `$HF_TOKEN` |
| `is_fetched(name) -> bool` | 这个模型的文件是否已在 `assets_dir()` 里 |
| `asset_of(path) -> Optional[str]` | 哪个名字的模型文件包含 `path`（在 `assets_dir()` 下），都不是就 `None` |

```python
from hirobot import assets

assets.fetch(["unitree_g1", "menagerie_universal_robots_ur5e"])
assets.is_fetched("unitree_g1")   # True
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
model:                      # 可选：本体的模型文件（Embodiment.model）
  mjcf: ~/models/ur5e.xml   # MJCF，用于仿真 / 碰撞 / 运动学
  # urdf: ../urdf/ur5e.urdf                        # 或者 URDF（只用于绘制）
  # package_roots: {ur_description: ../urdf}       # URDF package:// 的根目录
driver: {type: ur_rtde}     # 注册的驱动名 + 它的构造参数
```

`model:` 要有 `mjcf` 或 `urdf`；两个都有时 `Embodiment.model` 取 MJCF。相对路径（包括
`package_roots` 里的）按设备文件所在目录展开。仿真、`mujoco` 驱动和 `mink_ik` 需要 `mjcf`，只有 `urdf`
的本体在那里会被拒绝。`kp`、`kd`、`armature`、`home` 设置 `mujoco` 本体的增益、armature 和初始姿态。
`joint_names`（模型关节 -> 新名字）在模型编译前给 MJCF 关节改名，引用它们的执行器一起改。

相机文件带 `width` / `height` / `fps` / `depth` / `fovy` / `intrinsics`（`{fx, fy, cx, cy}`，可选 `width`、`height`、`model`、`coeffs`）；末端执行器文件带 `interface`（`joints` 或
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
  camera_fps: 30           # 没有 fps 的相机；null 为每次观测都渲染
mcp:                       # 可选；`hirobot mcp` / serve_mcp 提供什么（见“MCP 服务器”）
  tools: [observe]
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
| `hirobot info --config NAME_OR_PATH [--resolved]` | 打印 embodiment：类型、控制频率、浮动基座、关节、末端执行器、相机、模型文件。`--resolved` 打印合并 `defaults:` 之后的完整配置 |
| `hirobot configs [GROUP]` | 列出内置配置（`robots`、`body`、`end_effector`、`camera`、`microphone`、`speaker`）。`robots/` 下的名字也可以不带前缀（`unitree_g1`） |
| `hirobot assets list` | 列出 `hirobot assets fetch` 能下载的仿真模型、是否已在 `~/.cache/hirobot/assets` 里、是什么（见“[仿真资产](#仿真资产)”） |
| `hirobot assets fetch NAME... \| --all [--token T]` | 把这些模型（或全部）下载到 `~/.cache/hirobot/assets`；需要 `hirobot[assets]`。未知名字、没装 `huggingface_hub` 或下载失败时退出码 2（私有数据集要先 `hf auth login`；`HF_ENDPOINT` 可以指向镜像） |
| `hirobot sim CONFIG [--seconds N] [--viewer] [--fast]` | 运行一台仿真机器人：连接，保持起始姿态 N 秒（默认 3），打印剩余的关节误差、基座高度、渲染出的图像和 `get_observation` 耗时。自由基座摔倒时会提示。对真机本体以退出码 2 退出；`--viewer` 需要 MuJoCo 本体 |
| `hirobot check --sim CONFIG [--against-real] [--driver PART:KEY=VALUE ...] [--tolerance 0.02]` | 建仿真检查用的世界，不推进：每个本体要有 `model:`，每个关节要在它的 MJCF 里且有执行器，每个末端 frame 是 MJCF 里的 body；关节限位和 MJCF 的 range 差超过 0.1 rad 打印 warning。`--against-real` 再以 `read_only: true` 打开每个本体的真机驱动（没有这个模式就拒绝），读一次观测，打印同一组关节角下每个末端的实测和模型位置差。出错或误差超过 `--tolerance` m 时退出码 1。库：`hirobot.sim.check_sim` |
| `hirobot mcp CONFIG [--transport stdio\|http] [--host H] [--port P] [--path /mcp] [--tools CATEGORY ...] [--max-steps N] [--no-reset]` | 把机器人作为 MCP 服务器提供出来（见“[MCP 服务器](#mcp-服务器)”）；需要 `hirobot[mcp]`。没装 `mcp` 包或配置有错时退出码 1 |
| `hirobot view CONFIG [--model MUJOCO_CONFIG] [--body KEY=VALUE ...] [--hz 30] [--seconds N]` | 在 MuJoCo viewer 里显示真机姿态。以 `read_only: true` 打开 CONFIG（驱动没有 `read_only` 模式就拒绝），每帧把测得的关节（按名字对）和基座姿态拷进模型，从不发命令。`--model` 默认用内置的 `<config>_mujoco`。仿真机器人或找不到模型时退出码 2。库：`hirobot.sim.view` |
| `hirobot scan [--interface NIC] [--timeout 2] [--no-unitree] [--json]` | 只读地查看本机和机器人网络，打印摘要（`--json`：只打印报告）。不发命令，不改任何设置。什么都没找到也退出码 0。见“[`hirobot scan` 和 `hirobot init`](#hirobot-scan-和-hirobot-init)”。库：`hirobot.scan` |
| `hirobot init -o FILE [--model NAME] [--interface NIC] [--answers FILE] [--yes] [--force] [--no-verify] [--timeout 2]` | 扫描，选定型号和网卡，把本地配置写到 FILE，再只读打开它，说明哪些部件发来了数据。退出码 0 已写入（并已检查），1 什么都没写，2 已写入但检查失败。库：`hirobot.scan.init_config` |

### `hirobot scan` 和 `hirobot init`

这个版本只认识 Unitree 机器人（G1、带 Dex3 手的 G1、H1、带 Inspire 手的 H1、H1-2、H2）。
`hirobot scan` 读取本机信息（网卡、已装的 SDK、ROS 2、相机、音频设备）；当某块网卡有
`192.168.123.x` 的地址（或用 `--interface` 指定一块）时，探测这块网卡上的 Unitree 机器人。
探测只订阅 `rt/lowstate` 和 `rt/dex3/{left,right}/state`，并向 motion switcher 查询一次
`CheckMode`。它不创建 DDS publisher，不发命令，不改网络设置。需要 `sudo` 的操作只列在
suggestions 里，不会执行。

在带 Dex3 手的 G1 上，`hirobot scan --json` 打印的报告形如（示例，有删节）：

```json
{
  "version": 1,
  "host": {
    "hostname": "robot-pc", "os": "Linux 5.15.0-tegra", "arch": "aarch64",
    "python": "3.10.12", "jetson": "NVIDIA Jetson Orin NX",
    "interfaces": [{"name": "eth0", "up": true, "ipv4": ["192.168.123.164/24"],
                    "mac": "00:11:22:33:44:55", "speed_mbps": 1000, "unitree_subnet": true}],
    "sdks": [{"distribution": "unitree_sdk2py", "minimum": "1.0.1", "installed": "1.0.1",
              "ok": true, "drivers": ["unitree_handcmd", "unitree_sdk2", "..."],
              "install": "pip install -e <clone of https://github.com/unitreerobotics/unitree_sdk2_python>"}],
    "ros2": {"distro": null, "rclpy": false},
    "cameras": [{"kind": "realsense", "name": "Intel RealSense D435", "serial": "123456789",
                 "device": null}],
    "audio": {"source": "alsa", "devices": [{"name": "USB Audio", "inputs": 1, "outputs": 2}]},
    "suggestions": [],
    "notes": []
  },
  "unitree": {
    "interface": "eth0",
    "status": "found",
    "error": null,
    "msg_family": "hg",
    "motor_slots": 35,
    "motors_reporting": 29,
    "reporting_slots": [0, 1, 2, "...", 28],
    "mode_machine": 5,
    "lowstate_hz": 500.0,
    "motion_service": {"checked": true, "mode": "ai", "owns_lowcmd": true},
    "hands": {"dex3_left": true, "dex3_right": true},
    "candidates": [
      {"config": "robots/unitree_g1_dex3", "confidence": 0.9, "why": "unitree_hg with 29 motors reporting: G1 29-DoF; rt/dex3/left/state and rt/dex3/right/state seen"},
      {"config": "robots/unitree_g1", "confidence": 0.85, "why": "unitree_hg with 29 motors reporting: G1 29-DoF"}
    ],
    "notes": ["built-in motion service 'ai' owns rt/lowcmd; joint commands need it released (robot hung or supported)"]
  }
}
```

扫描没看到的值是 `null`（或 `[]`），意思是“没看到，请确认”，不是“不存在”。锁了腰的 G1
只报告 27 个电机；不在总线上的 Dex3 手显示为 `null`。

| 键 | 含义 |
|---|---|
| `host.interfaces[].unitree_subnet` | 网卡有 `192.168.123.0/24` 上的 IPv4 地址；探测第一块处于 up 状态的这种网卡 |
| `host.sdks[]` | 驱动 manifest 里列出的每个 SDK 一项。`installed` 是版本号；模块能导入但没有包元数据（源码检出）时为 `"unknown"`；没装时为 `null`。已安装且不低于 `minimum` 时 `ok` 为 true。`install` 是 manifest 里的安装提示 |
| `host.suggestions` | 给你执行的命令（机器人子网的地址、SDK 安装）；扫描从不执行它们 |
| `host.notes` | 本机上失败的探测项（缺 `ip` 命令、相机库抛了异常）；扫描会继续 |
| `unitree.status` | `found`（收到了 `rt/lowstate`）、`not_seen`（`--timeout` 内什么都没收到）、`skipped`（没有 `192.168.123.x` 上的网卡，或给了 `--no-unitree`）、`error`（缺 SDK，或 DDS 无法在这块网卡上启动；原因在 `error` 里） |
| `unitree.msg_family` | `hg`（G1、H1-2、H2）或 `go`（19 自由度的 H1） |
| `unitree.motor_slots`、`motors_reporting`、`reporting_slots` | `LowState` 消息里的电机槽位数，其中读到温度或电压的个数，以及它们的下标。只有探测真正运行时才有 `reporting_slots` |
| `unitree.motion_service` | `CheckMode` 的结果：`mode` 是正在运行的内置服务（没有时为 `""`）；有服务运行时 `owns_lowcmd` 为 true。查询没有回应时 `checked: false` |
| `unitree.hands` | 对应的 Dex3 状态 topic 发来数据时为 `true`，没看到时为 `null` |
| `unitree.candidates` | 符合的内置机器人配置，按 `confidence` 从高到低，每项带 `why` |

`hirobot init` 先扫描，然后：

1. **型号。** `--model`（或 answers 文件里的 `model:`）照原样使用（`unitree_g1` 和
   `robots/unitree_g1` 都可以）。没有给出时，`init` 列出候选并询问；`--yes` 取第一个；
   没有终端时，只有第一个候选的 confidence 不低于 0.9 才取它。既没有候选也没有给型号：
   什么都不写，退出码 1。
2. **网卡。** `--interface`、answers 文件里的 `interface:`，或扫描到的网卡。都没有：退出码 1。
3. **已有文件。** 替换 FILE 之前 `init` 会询问（默认否）。只有回答“y”、给了 `--force`
   或 `overwrite: true` 时才覆盖。只给 `--yes` 从不覆盖：文件保留，退出码 1。
4. **写入。** FILE 开头是一段注释（型号、网卡、日期、扫描看到的内容），然后是
   `defaults: [robots/<model>]` 和 `assembly: parts:` 覆盖项：把本体驱动设为
   `unitree_sdk2` 并带上 `network_interface`，并给每个驱动接受该参数的部件设
   `network_interface`（Dex3 手、头部相机、麦克风、扬声器）。不写增益，也不写安全参数：
   它们保持内置默认值。写完立即用 `load_embodiment` 加载一次。
5. **只读检查**（用 `--no-verify` 或 `verify: false` 跳过）。`init` 以本体驱动
   `read_only: true` 打开 FILE（和 `hirobot view` 一样；手随本体一起变为只读），在
   `--timeout` 秒内（至少 1 秒）读取观测，把每个部件打印为 data seen、partly seen、
   not seen 或 skipped。相机和麦克风只接收数据，所以会打开。驱动没有 `read_only`
   模式的部件跳过、不打开：扬声器就是这样（它在 connect 时设置音量）。机器人连不上、
   `joint_pos` 不是每个关节一个值、或本体有关节不是有限值时，检查失败。手、相机或
   麦克风没有数据或连不上时，报告为 not seen 并略过，不算失败；只有本体必须有数据。

answers 文件（`--answers FILE`）是 YAML，只能有下面这些键，用于没有终端的运行：

```yaml
model: robots/unitree_g1_dex3   # skip the model question
interface: eth0                 # skip the network card choice
overwrite: true                 # replace FILE if it exists (default: keep it)
verify: false                   # skip the read-only check
```

退出码：0 已写入（检查通过或已跳过）；1 什么都没写（没有型号、没有网卡、文件已存在且被保留、
answers 或型号名有误）；2 已写入但只读检查失败（文件保留，排除原因后重新运行 `init`）。

第一次发命令让机器人动之前，先检查写出的文件：见
[Unitree 设置指南](setup/unitree.zh-CN.md#生成本地配置)。

## MCP 服务器

怎样接入客户端、在仿真里第一次跑通、在真机上安全使用：见 [MCP 指南](./mcp.zh-CN.md)。

`pip install -e '.[mcp]'`（`mcp` SDK，要 Python 3.10+，以及 Pillow）把一台机器人作为
[MCP](https://modelcontextprotocol.io) 服务器提供出来，让 LLM agent 或任何 MCP 客户端调用它的观测和命令工具。这些工具就是
下面的机器人接口，不多做事：每条命令都经过机器人的 `send_action`、它的 `SafetyGate` 和一个命令租约。控制循环留在调用工具的一方。

```bash
hirobot mcp robots/ur_ur5e                       # stdio：由 MCP 客户端启动这个命令
hirobot mcp my_robot.yaml --tools observe act    # 同时允许运动
hirobot mcp my_robot.yaml --transport http --port 8000   # streamable HTTP，地址 http://127.0.0.1:8000/mcp
```

`hirobot mcp` 构建机器人、连接、reset（`--no-reset` 时不 reset），一直服务到客户端断开或 Ctrl-C，然后断开连接。日志写到
stderr。HTTP 默认只听 `127.0.0.1`；用别的 `--host` 时，能访问这个端口的人都能命令机器人，并会打一条警告。

### 工具

| 类别 | 工具 | 作用 |
|---|---|---|
| `observe` | `describe_robot` | 名字、类型、`control_hz`、关节（名字和限位，顺序就是关节列表用的顺序）、末端执行器、相机（内参和安装位置）、`action_modes`、开着的工具、`max_steps` |
| `observe` | `get_observation` | 状态 JSON（关节数组为 `{关节: 值}`、末端位姿、夹爪、基座、时间戳），加每个相机一张 JPEG。`images: false` 只要状态；`cameras: [...]` 选相机 |
| `observe` | `get_robot_status` | `get_status()`（故障、电池、温度、安全门计数、命令进度），加 `skill_running` 和 MCP 租约状态 |
| `act` | `send_actions` | 1 到 `max_steps` 个动作，按 `control_hz` 每拍一个（仿真也一样），最后 `hold`。有一步被拒就结束整个动作块。返回 `sent`、`of`、`stopped`（`done` / `refused` / `stop`）、`error` 和新的观测 |
| `act` | `hold` | `hold()` |
| `act` | `stop` | 让正在执行的 `send_actions` 在下一步之前停下，停掉正在运行的技能，然后 hold。不是急停 |
| `skill` | `list_skills` | `list_robot_skills()`，带每个技能的参数 schema |
| `skill` | `run_skill` | `run_robot_skill(name, args)`；启动后就返回 |
| `skill` | `stop_skill` | `stop_robot_skill()` |
| `episode` | `reset` | `reset(seed)`，然后返回新的观测 |

默认只开 `observe`；运动要在配置里或用 `--tools` 打开。没有 `estop` 工具（急停给人或看门狗用，不给模型调），也没有声明任务完成的工具
（那是 agent 或 benchmark 的事）。深度图不发送。

`send_actions` 里的一个动作就是 JSON 形式的 `Action`：`mode`（`action_modes` 之一）和这个模式用的字段。`joint_target` /
`kp` / `kd` 可以是按 `describe_robot` 顺序的列表，也可以是 `{关节名: 值}`；只写了部分关节的映射会设 `joint_names`（用于非关节
模式）。有部件的机器人可以用 `parts` 写 `{部件: 动作}`。

第一条命令申请一个名为 `mcp` 的租约（见[命令仲裁](#命令仲裁)），配置设了 `envelope` 就带上。如果优先级更高的一方接管了机器人
（`Priority.TAKEOVER`），下一条命令会报告被抢占并被拒绝，不会重新申请。

调用方写错的、或被机器人拒绝的工具调用，返回一个设了 `isError` 的结果，原因写在文本里。结果同时在 `structuredContent` 里带 JSON。

### `mcp:` 配置段

只能写在根配置里，和 `arbiter:` 一样。每个键都可选；命令行选项和 `serve_mcp` 的关键字参数优先于它。

```yaml
mcp:
  tools: [observe, act]          # 类别：observe、act、skill、episode（默认 [observe]）
  max_steps: 10                  # 每次 send_actions 最多几个动作，1-50（默认 5）
  image_max_side: 512            # 返回图像的最长边，像素（默认 512）
  image_quality: 80              # JPEG 质量 1-100（默认 80）
  envelope: {max_joint_step: 0.05}   # mcp 租约的限制；键和 claim(envelope=...) 相同
```

### 在 Python 里用

```python
from hirobot import make_robot
from hirobot.mcp import serve_mcp

robot = make_robot("my_robot.yaml")
robot.connect()
serve_mcp(robot, transport="stdio", tools=["observe", "act"])   # 阻塞
```

`serve_mcp(robot, *, transport="stdio", host="127.0.0.1", port=8000, path="/mcp",
name=None, tools=None, max_steps=None, image_max_side=None, image_quality=None,
envelope=None)` 不连接也不断开机器人；返回时释放租约。`hirobot.mcp.RobotTools(robot, settings=None)` 是不依赖 SDK 的同一套
工具：`tools.call(name, args)` 返回 `ToolResult`（`data`、`images`）。`build_server(tools)` 返回 SDK 的低层 `Server`，
用于进程内（`mcp.Client(server)`）或别的传输方式。所有机器人调用都在 `RobotTools` 自己的一个线程上执行。

## 编写驱动和层（SPI）

上面几节是应用 API。这一节给扩展 hiRobot 的代码看：驱动和层。这些方法以 `_` 开头；应用从不调用，层的 `inner` 也不暴露它们。

| 成员 | 类型 | 说明 |
|---|---|---|
| `_send_action(action)` | abstract | 驱动原语。**驱动实现这个方法，绝不实现 `send_action`** |
| `_estop()` | optional | `estop()` 背后的驱动原语。默认在驱动有 `damp()` 时调用它 |
| `_idle_step()` | optional，SPI | 在最后一条指令下让仿真时间前进一个控制周期。assembly 对动作未提及的仿真部件调用它。默认什么都不做；toy 仿真和 MuJoCo 会步进 |
| `_set_safety(gate)` | concrete，SPI | 挂接或清除（`None`）`SafetyGate`。给 `make_robot` 和测试用；应用在配置的 `safety:` 段里设限制 |
| `_run_onboard_skill(spec, args)` | optional | `run_onboard_skill` 背后的驱动原语；`args` 已检查过。返回这次参数预计跑多少秒，返回 `None` 时用 `spec.duration_s` |
| `_set_fault(reason)` / `_clear_fault()` | concrete，SPI | 让机器人进故障态（驱动或 layer 自己进入阻尼或停止时），命令抛 `RobotFaultError`，直到 `reset()`；清除故障 |
| `_stop_token()`、`_commanding(token)`、`_wait(seconds, token)` | concrete，SPI | 让驱动里跑一阵的循环在别的线程调 `estop()` 时立刻结束（见[编写驱动](#编写驱动)） |

### 编写驱动

继承 `Robot`，实现 `embodiment`、`connect`、`reset`、`get_observation` 和 `_send_action`，然后注册它。
新驱动需要的目录结构、清单和契约测试见 README 的
[Add a new robot](../README.zh-CN.md#添加新机器人)。构造函数的第一个参数必须是 `Embodiment`；
其余关键字参数就是用户写在 `body:` 下的内容。仿真器要设置 `simulated = True`。要提供机载 skill，
覆盖 `has_onboard_skills`、`list_onboard_skills`、`_run_onboard_skill` 和 `stop_onboard_skill`。机器人有安全状态的驱动覆盖
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
| `CameraDriver(name)` | 抽象类：`start()`、`latest() -> Optional[Frame]`（`(image, Stamp)`，`Stamp.t` 在 `time.monotonic()` 上，每来一帧新的 `seq` 加一；不能阻塞；第一帧之前返回 `None`）、`stop()`（幂等）。RGB-D 驱动还要覆盖 `latest_rgbd() -> Optional[RgbdFrame]`；默认实现返回不带深度的 `latest()`。可选的 `intrinsics() -> Optional[CameraIntrinsics]`：交付图像的内参，`start()` 之后从设备读；默认 `None` |
| `PollingCameraDriver(name, poll_hz=30.0, retry_s=0.2)` | 在后台线程上运行阻塞的 `_grab() -> HxWx3 uint8 RGB`，只保留最新一帧。子类实现 `_grab`，可选实现 `_open` / `_close`。提供 `wait_for_frame(timeout)`、`error`、`frames`、`errors` |
| `Frame` | `Tuple[np.ndarray, Stamp]`：图像及其到达时间（`Stamp(t, seq)`，`t` 在 `time.monotonic()` 上） |
| `RgbdFrame` | `Tuple[np.ndarray, Optional[np.ndarray], Stamp]`：彩色图像、与之配准的深度（HxW float32，米）或 `None`，以及两者共同的 `Stamp` |
| `intel_realsense` → `RealSenseCamera(name, serial=None, width=640, height=480, fps=30, depth=False, depth_width=None, depth_height=None, timeout_ms=1000, poll_hz=60.0)` | 直接插在跑 hiRobot 的计算机本地 USB 上的 Intel RealSense（D435i、D405、L515……），走 `pyrealsense2`。接了多个时用 `serial` 指定设备（否则取枚举到的第一个）。设置 `depth: true` 时它是 RGB-D 相机：深度传感器对齐到彩色，使 `Observation.depth` 与彩色共用同一像素网格，并按传感器的 `depth_scale` 把设备单位换算为米。深度流尺寸和彩色不同时用 `depth_width` / `depth_height` 设置（默认和彩色一样）；对齐后的深度图仍是彩色的尺寸。需要 `pyrealsense2`（`pip install pyrealsense2`）；L515 要老版本（2.54.2 已验证；2.59.1 列不出 L515）。`intrinsics()` 是彩色流的 `get_intrinsics()`，带它的畸变模型和系数 |
| `orbbec_pyorbbecsdk` → `OrbbecCamera(name, serial=None, uid=None, width=640, height=480, fps=30, color_format="MJPG", depth=False, align="hw", timeout_ms=1000, poll_hz=60.0)` | 直接插在跑 hiRobot 的计算机本地 USB 上的 Orbbec 相机（Dabai DC1……），走 1.x 版 `pyorbbecsdk`（OrbbecSDK 1.10；老型号在 2.x 下枚举不到）。用 `uid`（USB 端口路径）或 `serial` 指定设备。设置 `depth: true` 时它是 RGB-D 相机：深度对齐到彩色（`align`：`hw` 在相机上做，`sw` 在 SDK 里做），设备单位换算为米。MJPEG / YUYV 彩色用 OpenCV 解码。绑定需从源码编译，见 [agilex_piper.zh-CN.md](setup/agilex_piper.zh-CN.md#6-orbbec-相机)。`intrinsics()` 是 `get_camera_param()` 给的彩色内参，有畸变时为 Brown-Conrady（有理模型 k4-k6 会去掉畸变并警告） |
| `stereolabs_zed` → `ZedCamera(name, serial=None, resolution="HD720", fps=30, depth=False, depth_mode="NEURAL", min_depth_m=0.1, poll_hz=60.0)` | 直接插在跑 hiRobot 的计算机本地 USB 上的 Stereolabs ZED（ZED 2i、ZED Mini……），走 `pyzed`。接了多台时用 `serial` 指定（否则取第一台）。图像是左目彩色（`resolution`：`HD2K`、`HD1080`、`HD1200`、`HD720`、`SVGA`、`VGA`）。设置 `depth: true` 时它是 RGB-D 相机：深度由 SDK 的 `depth_mode`（`NEURAL`、`ULTRA`、`QUALITY`、`PERFORMANCE`）算出，单位米，对齐到左目图像，没有读数处为 0。需要 ZED SDK、它的 `pyzed` wheel 和 NVIDIA GPU。`intrinsics()` 是校正后左目的标定，无畸变 |
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
| `SpeakerDriver(name)` | 抽象类：`open()`、`play(pcm, sample_rate=None) -> float`（阻塞；返回播放的秒数，被停止时会更短）、`stop()`（可从另一个线程调用）、`close()`、`sample_rate`。可选的音量能力：`supports_volume`（类属性，默认 `False`）、`volume -> Optional[int]`（0-100，不知道时为 `None`）、`set_volume(level: int)`（0-100，可在 `play()` 运行时从另一个线程调用；超出 0-100 抛 `ValueError`，`supports_volume` 为 `False` 时抛 `NotImplementedError`）。以后其他可选的设备能力也照这个写法：一个 `supports_<名字>` 标记，调用方先查它 |
| `SoftwareVolume` | 给没有硬件音量的扬声器用的 mixin：`supports_volume = True`，音量存在驱动里，驱动用 `apply_gain(pcm, level)` 缩放 PCM |
| `AudioChunk` | Dataclass：`pcm`、`sample_rate`、`stamp`（主机接收时间）、`duration` |
| `resample(pcm, src_rate, dst_rate)` | 用于语音的线性插值重采样 |
| `unitree_voice_mic` → `UnitreeG1Microphone(name, network_interface=None, local_ip=None, group="239.168.123.161", port=5555, queue_s=10.0)` | G1 麦克风：音频板的 UDP 组播，16 kHz |
| `unitree_voice_speaker` → `UnitreeG1Speaker(name, network_interface=None, domain_id=0, volume=None, app_name="hirobot", chunk_s=1.0, lead_s=1.0, timeout_s=2.0)` | G1 扬声器：SDK 的 `voice` 服务，16 kHz。硬件音量（`supports_volume`）：`volume` 读机器人当前的音量（`GetVolume`），`set_volume(level)` 立即修改（没打开或调用失败时抛 `RuntimeError`）。配置项 `volume` 在 `open()` 时设置；只要两者之一改过音量，`close()` 时就恢复成 `open()` 时机器人原来的音量 |
| `wav_mic` → `WavMicrophone(name, path, chunk_ms=20, realtime=True, tail_silence=1.0)` | 把一个 16-bit WAV 文件送入队列（有 `realtime` 时按说话速度，没有时一次送完），追加 `tail_silence` 秒静音，之后 `read()` 返回 `None`。仿真类 |
| `wav_speaker` → `WavSpeaker(name, path, realtime=True, sample_rate=16000)` | 把播放的所有内容录进一个 WAV 文件，在 `close()` 时写出。有 `realtime` 时，`play` 耗时与音频时长相同，`stop()` 会截断它。音量是软件增益，作用在录下的音频上（初始 100）。仿真类 |
| `null_speaker` → `NullSpeaker(name, realtime=True, sample_rate=16000)` | 什么都不播放；有 `realtime` 时耗时与音频时长相同。有软件音量（只记录音量值）。仿真类 |
| `sounddevice_mic` → `SoundDeviceMicrophone(name, sample_rate=16000, chunk_ms=20, device=None)` | 计算机的输入设备。需要 `audio` extra。仿真类（可与仿真器一起使用） |
| `sounddevice_speaker` → `SoundDeviceSpeaker(name, device=None, block_ms=50)` | 计算机的输出设备，按 `block_ms` 分块写入，让 `stop()` 能及时生效。音量是每块的软件增益（初始 100，不改系统混音器）。需要 `audio` extra。仿真类 |
| `build_audio(microphones, speakers, network_interface=None, sim_only=False) -> (mics, speakers)` | 为带 `driver:` 的设备构建驱动，规则与 `build_cameras` 相同 |
| `read_wav(path) -> (pcm, rate)`, `wav_bytes(pcm, rate)` | `hirobot.drivers.audio` 中的 16-bit WAV 辅助函数（单声道；多声道文件取第一个声道） |
