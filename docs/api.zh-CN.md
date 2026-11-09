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

- [顶层包](#顶层包)
- [快速示例](#快速示例)
- [Embodiment](#embodiment)
- [Observation, Action, ActionChunk, ControlMode](#observation-action-actionchunk-controlmode)
- [Robot](#robot)
- [机器人 skill](#机器人-skill)
- [构建机器人：make_robot 与驱动注册表](#构建机器人make_robot-与驱动注册表)
- [SafetyGate](#safetygate)
- [DofMap](#dofmap)
- [Kinematics](#kinematics)
- [控制循环](#控制循环)
- [RobotAssembly 与 TwinRobot](#robotassembly-与-twinrobot)
- [扩展点：相机、末端执行器与音频](#扩展点相机末端执行器与音频)
- [配置文件参考](#配置文件参考)
- [CLI](#cli)

## 顶层包

| 名字 | 类别 | 模块 | 用途 |
|---|---|---|---|
| `Embodiment`, `Joint`, `EndEffector`, `EndEffectorFrame`, `Camera`, `CameraMount`, `AudioDevice`, `EmbodimentKind` | pydantic 模型 / 枚举 | `hirobot.embodiment` | 声明式的机器人 I/O 契约 |
| `Observation`, `Action`, `ActionChunk`, `ControlMode`, `MotionClip`, `JOINT_MODES`, `MOTION_SPACES` | pydantic 模型 / 枚举 / 元组 | `hirobot.types` | 机器人与 policy 之间的类型化 I/O |
| `Robot` | 抽象类 | `hirobot.robot` | 驱动契约 |
| `RobotAssembly` | `Robot` | `hirobot.assembly` | 本体驱动加上挂接的相机、末端执行器（灵巧手、夹爪）、麦克风和扬声器 |
| `RobotGroup` | `Robot` | `hirobot.group` | 把多台完整的机器人加场景相机作为一个整体驱动（见 [RobotGroup](#robotgroup)） |
| `RobotFaultError` | 异常 | `hirobot.robot` | 机器人处于故障状态时由 `send_action` 抛出 |
| `RobotStoppedError` | 异常 | `hirobot.robot` | `RobotFaultError` 的子类：长调用（`connect`、`reset`）因 `estop()` 而放弃 |
| `register_body`, `get_body`, `available_bodies` | 函数 | `hirobot.registry` | 按名字注册的驱动注册表 |
| `register_layer`, `get_layer`, `available_layers` | 函数 | `hirobot.registry` | 由 `layers:` 叠加的 layer 的注册表（见 [Layer](#layer)） |
| `load_embodiment`, `make_robot` | 函数 | `hirobot` | 从配置名、文件或 dict 构建 embodiment / 机器人 |
| `require_sdk` | 函数 | `hirobot.sdk` | 按驱动的 `manifest.yaml` 检查其厂商 SDK |
| `RobotSkillSpec`, `RobotSkillResult`, `RobotSkillError` | pydantic 模型 / 异常 | `hirobot.robot_skills` | 描述、启动和拒绝机器人 skill（行走、手势） |
| `SafetyGate`, `SafetyVerdict`, `SafetyError` | 类 / dataclass / 异常 | `hirobot.safety` | 在每个动作到达硬件之前检查它 |
| `DofMap` | 类 | `hirobot.dof` | 按名字重排关节向量 |
| `pose_compose`, `pose_inverse`, `pose_from_matrix`, `pose_from_axis_angle`, ... | 函数 | `hirobot.transforms` | 7 维向量位姿 `[x, y, z, qx, qy, qz, qw]` 的位姿运算（不从 `hirobot` 重新导出） |
| `Kinematics`, `BodyPoses` | 类 | `hirobot.kinematics` | MJCF 模型的正运动学（见 [Kinematics](#kinematics)；不从 `hirobot` 重新导出） |
| `__version__` | str | `hirobot` | 包版本 |

相机、末端执行器和音频的接口与注册表在 `hirobot.camera`、`hirobot.end_effector` 和 `hirobot.audio` 中；
驱动本身在 `hirobot.drivers` 下（`drivers/<part>/<SDK vendor>/<SDK or protocol>/`）。两者分别在
[构建机器人](#构建机器人make_robot-与驱动注册表)和[扩展点](#扩展点相机末端执行器与音频)中介绍。

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

## Embodiment

`hirobot.embodiment`。一组 pydantic 模型，描述机器人在 policy 眼中的样子（即 embodiment，本体描述），
与驱动无关。通常用 `load_embodiment(path)` 从配置的 `embodiment:` 段加载。

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

`hirobot.types`。接受 numpy 数组的 pydantic 模型。每个可选字段默认为 `None` 或 `{}`，所以驱动只需填写
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
| `stamps` | `Dict[str, Stamp]` | 每个驱动都设 `stamps["body"]`；手、单独话题的夹爪和相机按自己的名字加上（组里是 `left/body` 等）。`Stamp(t, seq)`：`t` 是这次读数到达的时间，在 `robot.now()` 上；`seq` 是这个源的第几次读数，从 1 开始。某一路有多旧：`robot.now() - obs.stamps[name].t`；同一帧再次读到时 `seq` 不变。`hirobot.stamps.stale_sources(obs, robot.now(), max_age=None, embodiment=None)` 列出比 `max_age`（秒，或按源给）旧的源，默认是 3 个标称周期（`Camera.fps`、`control_hz`） |
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
| `members` | `Dict[str, Action]` | `{}` | 仅用于机器人组：成员名 -> 该成员的动作，使用它自己的名字和模式。此时除 `extra` 外的其他字段都必须为空；单台机器人会拒绝它（`SafetyError`） |

以下情况构造时抛出 `ValueError`：`MOTION_CLIP` 没有 `motion`，`BASE_POSE` 没有 `base_target`，
`joint_names` 与关节模式同用、有重复或没有 `joint_target`，或 `members` 与其他指令字段同时出现。

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
| `members` | `Dict[str, ActionChunk]` | 机器人组：成员名 -> 它的 chunk。所有成员的 horizon 相同（否则 `ValueError`），其他字段保持为空，`step(i)` 给出带 `members` 的 `Action` |

- `horizon -> int`（也可用 `len(chunk)`）：即 `T`，依次取自 `joint_targets`、`base_velocity`、
  `base_target` 或 `latent`，然后是 `ee_targets`，然后是 `gripper`；全部为空时为 `0`。
- `step(i) -> Action`：第 `i` 步作为单个 `Action`。超出 `[0, horizon)` 时抛出 `IndexError`。

## Robot

`hirobot.robot.Robot`，每个机器人、仿真器和包装器都要实现的抽象驱动契约。

| 成员 | 类别 | 说明 |
|---|---|---|
| `embodiment` | abstract property | 该驱动暴露的 `Embodiment` |
| `connect()` | abstract | 打开连接。在控制前调用一次 |
| `reset(seed=None)` | abstract | 让机器人进入安全的初始状态。仿真器用 `seed` 选择场景（例如环境 seed 或初始状态索引）；其他驱动忽略它 |
| `get_observation() -> Observation` | abstract | 读取传感器 |
| `send_action(action)` | concrete | 先 `check_action`，再 `_send_action`。被拒绝时抛出 `SafetyError` 且不发送任何内容，`action.joint_names` 列出了 `direct_joints` 之外的关节时也一样。处于故障状态时抛出 `RobotFaultError`；机器人是 `RobotGroup` 成员而调用不经过组时抛出 `RuntimeError` |
| `check_action(action) -> Action` | concrete | 执行 `send_action` 的全部检查但不发送；返回（可能被裁剪过的）动作。机器人组在发送给任何成员之前对每个成员都运行它 |
| `fault` | property | 机器人拒绝指令的原因，或 `None`。由 `set_fault(reason)`（驱动或 layer 自己进入阻尼或停止时）和 `estop()` 设置。`reset()` 返回时清除（每个子类的 `reset` 都被包了一层，返回时调 `clear_fault()`），期间来过 `estop()` 则不清除。包装器（`RobotAssembly`、`TwinRobot`）也会报告内层机器人的故障。也见 `get_status().fault` |
| `estop()` | concrete | 立即停下；任何线程、任何时候都能调，不走安全闸，也不受组的所有权检查限制。别的线程上的长调用（`reset` ramp、`connect` 里的等待）在下一步抛出 `RobotStoppedError`；机器人进入故障状态；正在下发的指令先发完（最多等 `ESTOP_LOCK_TIMEOUT_S`，0.2 s），之后不会再有指令发出；然后调 `_estop()`。Unitree 和 MuJoCo 进阻尼（Unitree 开着 `builtin_skills` 时改为停掉 skill），AgiBot G1 发 `MotionStop`，UR 减速停下（`servoStop` / `speedStop` / `stopJ`），Franka 停掉位置控制器；没有 `damp()` 的驱动（`toy`、`fourier_grx`、`agibot_aimrt`）只拒绝后续指令。`RobotAssembly`、`RobotGroup`、`TwinRobot` 对每个部分都调急停 |
| `_estop()` | optional | `estop()` 背后的驱动原语。默认在驱动有 `damp()` 时调用它 |
| `idle_step()` | optional | 在最后一条指令下让仿真时间前进一个控制周期。机器人组对动作未提及的仿真成员调用它。默认什么都不做；toy 仿真和 MuJoCo 会步进 |
| `action_modes` | class attribute | `_send_action` 接受的模式。构建动作的调用方读取它 |
| `motion_spaces` | class attribute | 机器人在 `MOTION_CLIP` 下能跟踪的 `MotionClip.space` 取值。默认为空 |
| `direct_joints` | class attribute | 调用方可以在非关节模式旁按名字指挥的关节。默认为空 |
| `cartesian_targets` | property | 接受 `CARTESIAN` 位姿目标的末端执行器：`CARTESIAN` 在 `action_modes` 中时为全部末端执行器，否则为空。只跟踪其中一部分的机器人会覆盖它 |
| `_send_action(action)` | abstract | 驱动原语。**驱动实现这个方法，绝不实现 `send_action`** |
| `now()` | optional | 机器人时钟（秒），`Observation.stamps` 的时间在它上面：默认 `time.monotonic()`，`toy` / `mujoco` 是仿真时间。包装层返回内层机器人的 |
| `hold()` | optional | 让机器人保持在原处，例如在 policy 停止之后。默认情况下，驱动接受 POSITION 时把当前 `joint_pos` 作为 POSITION 动作发送，否则什么都不做。只接受笛卡尔增量的仿真器什么都不做；带 `builtin_skills` 的 Unitree 什么都不做（运动服务会让机器人保持站立）。有自己平衡回路的驱动（全身控制的人形机器人）必须覆盖它，指挥机器人站定（基座速度 `[0, 0, 0]`）：发送 `joint_pos` 会把关节冻住，人形机器人会摔倒。经过安全门 |
| `set_gains(kp, kd)` | optional | 每个关节的 PD 增益。默认什么都不做 |
| `set_safety(gate)` | concrete | 挂接或清除（`None`）`SafetyGate` |
| `has_robot_skills` | property | 驱动是否提供机器人 skill。默认 `False` |
| `list_robot_skills() -> List[RobotSkillSpec]` | optional | 机器人自身控制器运行的 skill。默认为空。见[机器人 skill](#机器人-skill) |
| `run_robot_skill(name, args=None) -> RobotSkillResult` | concrete | 按 skill 的 schema 检查 `args`，再调用 `_run_robot_skill`。抛出 `RobotSkillError` |
| `_run_robot_skill(spec, args)` | optional | `run_robot_skill` 背后的驱动原语；`args` 已检查过 |
| `stop_robot_skill()` | optional | 停止正在运行的 skill。默认什么都不做 |
| `disconnect()` | optional | 释放并停止。默认什么都不做 |
| `safety` | attribute | 挂接的安全门，或 `None` |
| `simulated` | class attribute | 仿真器驱动为 `True`。驱动是仿真驱动时，`make_robot` 不构建末端执行器驱动，只构建类同样为 `simulated` 的相机和音频设备（视频文件、WAV、主机的麦克风） |
| `success`, `done`, `reward` | attributes | 回合结果。hiRobot 里的驱动（`toy`、`mujoco`、各真机）都不跑任务，始终为 `None`；任务环境才设置它们，例如 hiBench 的基准环境和裁判。控制循环可以据此停止 |

## 机器人 skill

`hirobot.robot_skills`。机器人 skill 是由机器人自身控制器完成的动作（行走、坐下、挥手），按名字启动，
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

在任何内容到达机器人之前按 `parameters` 检查 `args`，然后调用驱动的 `_run_robot_skill`。支持的 schema
关键字：属性的 `type`（number / integer / boolean / string）、`minimum`、`maximum`、
`exclusiveMinimum`、`enum`、`default` 和 `required`。缺少的参数取其 `default`；未知参数会被拒绝。
它在机器人接受指令后就返回，而不是等动作结束。`RobotSkillResult` 包含 `name`、检查后的 `args`
（已填入默认值）和 `duration_s`。

skill 未知、参数错误或机器人拒绝时抛出 `RobotSkillError`。

### 包装器

`RobotAssembly` 把这四个成员转发给其本体驱动。`TwinRobot` 在 `mirror` 模式下把它们转发给真机，在
`shadow` 模式下不提供任何 skill，因为数字孪生无法运行厂商的控制器。

### 带 skill 的驱动

| 驱动 | 配置 | Skill |
|---|---|---|
| `unitree_sdk2` | `builtin_skills: g1` | G1 行走和手臂手势，见 [Unitree 设置指南](setup/unitree.zh-CN.md#机器人-skillg1) |

其他驱动都不提供 skill。

## 构建机器人：make_robot 与驱动注册表

### `load_embodiment(config) -> Embodiment`

在展开 `defaults:` 之后读取配置的 `embodiment:` 段（见[配置文件参考](#配置文件参考)）。配置可以是
内置名字（`"unitree_g1"`，等同于 `"robots/unitree_g1"`）、文件路径或已加载的 dict。缺少该段时抛出 `ValueError`。

### `make_robot(config, body_overrides=None, sim_overrides=None) -> Robot`

构建配置描述的机器人。`config` 是内置名字、文件路径或 dict。按顺序执行以下步骤：

1. 展开 `defaults:`，加载 embodiment，并在注册表中查找 `body.driver`（默认 `toy`）。
2. 驱动是真机驱动且有末端执行器带 `driver:` 时，为每个这样的末端执行器构建一个末端执行器驱动，并只把躯体关节（不含这些驱动自己管的夹爪）交给本体驱动。
3. 以 `driver_cls(embodiment, **robot_cfg)` 实例化驱动。`body:` 中的其他所有键都作为构造参数传入。
4. 如果 `sim.mode` 是 `mirror` 或 `shadow`，用一个 `TwinRobot` 把机器人和 MuJoCo 数字孪生包在一起。
   `shadow` 会对真机驱动强制 `read_only=True`。
5. 如果设置了 `layers:`：把躯体和末端执行器合并进一个 `RobotAssembly`（让每个 layer 都能看到所有关节），
   给它挂接一个 `SafetyGate`，再按顺序用每个 layer 包装它（见 [Layer](#layer)）。
6. 驱动是真机驱动时，为每个带 `driver:` 的相机构建相机驱动，为每个带 `driver:` 的麦克风 / 扬声器
   构建音频驱动，并把全部内容包进一个 `RobotAssembly`。
7. 除非 `safety: false`，否则根据 embodiment 和 `safety:` 段给最外层机器人挂接一个 `SafetyGate`。
   有 layer 时，layer 发出的关节指令由第 5 步的安全门检查，调用方发送的内容由这一个检查。

`body_overrides` 合并到 `body:` 之上，`sim_overrides` 合并到 `sim:` 之上，所以与主机相关的值
（网络接口、IP、模式）可以来自命令行。

以下情况抛出 `ValueError`：`sim.mode` 无效、`sim:` 段没有 `model`、`shadow` 与 `release_motion_mode`
同用、相机 / 末端执行器 / 音频 / layer 规格没有 `type`、末端执行器驱动不支持配置要求的接口、`layers:` 不是列表，或扬声器驱动被列为麦克风
（或反过来）。驱动名或 layer 名未知时抛出 `KeyError`。

### Layer

layer 是一个包装另一台机器人、改变其指挥方式的 `Robot`：例如一个全身控制器，接受 `BASE_VELOCITY` /
`MOTION_CLIP` / `CARTESIAN`，每个 tick 向下层机器人发送保持平衡的关节目标。hiRobot 本身不提供任何
layer；由插件包（hiWBC）注册。

```python
from hirobot import Robot, register_layer

@register_layer("my_wbc")
class MyWbc(Robot):
    def __init__(self, inner: Robot, **config): ...
```

```yaml
layers:                     # first entry innermost
  - type: my_wbc            # registered layer name
    controller: sonic       # every other key -> the layer's constructor
```

layer 通过 `inner.send_action` 发送关节指令，所以 `make_robot` 挂在下面的安全门会检查它们。layer 通过
`action_modes`、`motion_spaces`、`cartesian_targets` 和 `direct_joints` 报告自己接受什么。
有真实灵巧手时，layer 的 `inner` 是躯体加末端执行器的 `RobotAssembly`，所以它可以自己驱动手指，也可以把手指
留空（`direct_joints`），由调用方通过 `Action.joint_names` 指挥。layer 与驱动一样通过
`hirobot.drivers` entry point 注册。

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
| `mujoco` | `MujocoRobot` | `drivers/sim/mujoco/` | MuJoCo 物理仿真，本身就是一台机器人；渲染带 `mount` 的相机；为带 `frame` 的末端执行器提供 `ee_poses`；也用作真机的数字孪生 |
| `unitree_sdk2` | `UnitreeRobot` | `drivers/body/unitree/sdk2/` | Unitree G1 / H1 / H1-2 / H2 |
| `fourier_grx` | `FourierRobot` | `drivers/body/fourier/grx/` | Fourier GR1T1 / GR1T2 |
| `agibot_gdk` | `AgibotG1Robot` | `drivers/body/agibot/gdk/` | AgiBot G1（精灵 G01）：手臂、腰、头和夹爪走 GDK 的 DDS topic；底盘走其控制盒 |
| `agibot_aimrt` | `AgibotX1Robot` | `drivers/body/agibot/aimrt/` | AgiBot X1 |
| `franka_libfranka` | `FrankaRobot` | `drivers/body/franka/libfranka/` | Franka FR3 |
| `ur_rtde` | `URRobot` | `drivers/body/ur/rtde/` | Universal Robots UR5e |

每个硬件驱动的构造参数见其[设置指南](setup/README.zh-CN.md)。

### SDK 清单（`hirobot.sdk`）

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

## SafetyGate

`hirobot.safety`。`Robot.send_action` 在每个动作到达驱动之前运行的检查。

```python
SafetyGate(num_dofs=None, joint_limit=None, clamp=False, gripper_range=None)
SafetyGate.from_embodiment(embodiment, **kwargs)   # num_dofs = embodiment.num_dofs
gate.check(action, embodiment=None) -> SafetyVerdict
```

| 参数 | 作用 |
|---|---|
| `num_dofs` | `joint_target` 必须是这个长度（除非设置了 `joint_names`：此时每个名字一个值，且每个名字都必须是 `embodiment` 的关节）。`None` 跳过此检查 |
| `joint_limit` | 对 `joint_target` 施加的对称边界 `[-joint_limit, joint_limit]`（rad）。`None` 跳过 |
| `clamp` | `False`（默认）：越界的目标被拒绝。`True`：裁剪到边界内 |
| `gripper_range` | 夹爪值的 `[low, high]`；值会被裁剪到此范围。`None` 跳过 |

始终检查：`joint_target` 是一维且有限的，夹爪值有限；`base_velocity`、`base_target`、`latent`、
`ee_targets` 的每一项以及 `motion` 的每个数组都是有限的。

不检查：每关节上下限（`Joint.position_limit`，只有末端执行器驱动按它裁剪）、目标离实测关节位置多远、离上一次目标多远。
关节限位不对称的机器人（AgiBot G1 的肘部、UR、Franka）上，每关节限位只靠机器人自己的控制器；伺服模式下第一个目标出错，
机械臂会跳一下。`joint_limit` 在所有关节模式下都作用于 `joint_target`，包括 `VELOCITY`，那时它装的是速度（rad/s）。

`SafetyVerdict` 包含 `allowed: bool`、`action`（可能被裁剪过，仅在允许时有效）和 `reason: str`。
`SafetyError`（一个 `RuntimeError`）就是 `send_action` 带着该原因抛出的异常。

## DofMap

`hirobot.dof.DofMap(source_names, target_names)` 按名字在两种关节顺序之间映射。目标名字在源中不存在时
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

`hirobot.kinematics.Kinematics.from_mjcf(path, backend="numpy")` 加载一个 MJCF 模型，并由关节角和基座位姿
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
from hirobot.kinematics import Kinematics

kin = Kinematics.from_mjcf("~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml")
poses = kin.forward(obs.joint_pos, base_pos=[0, 0, 0.79], base_quat=[0, 0, 0, 1],
                    joint_names=robot.embodiment.joint_names)
pos, quat = poses["torso_link"]
```

## 控制循环

hiRobot 没有 policy 类、控制循环或模型服务客户端；它只提供循环所驱动的 `Robot`。循环从驱动用到的内容有：
`reset(seed=None)`、`get_observation()`、`send_action()`、停止后的 `hold()`、用于给真机限速的
`embodiment.control_hz`、`simulated`（仿真器不限速地步进），以及仿真器上的 `success` / `done` /
`reward`。`ActionChunk.step(i)` 把预测出的 chunk 中的一步转换成一个 `Action`。

## RobotAssembly 与 TwinRobot

两者都是 `make_robot` 根据配置构建的 `Robot` 包装器，很少需要直接构造。它们自己没有定义的属性
（例如驱动的 `raw_state`）会转发给被包装的驱动。

### `RobotAssembly(body, cameras=None, end_effectors=None, embodiment=None, first_frame_timeout_s=3.0, microphones=None, speakers=None, first_audio_timeout_s=2.0)`

`hirobot.assembly`。本体驱动加上相机驱动、末端执行器驱动和音频设备。

- `cameras: Dict[str, CameraDriver]`：每个相机的最新帧放进 `images[name]`，时间戳放进
  `stamps[name]`（仿真本体上换成仿真时间，`seq` 不变）。相机在 `first_frame_timeout_s` 内没收到第一帧时只记录一条警告；在帧到达之前
  它的键不存在。
- `end_effectors: Dict[str, EndEffectorDriver]`，按接口分：
  - `joints`（手）：此时 `embodiment` 必须列出躯体关节加上所有末端执行器关节（否则 `ValueError`）。
    关节向量从躯体和末端执行器汇集成一个向量，`joint_target` / `kp` / `kd` 再按名字拆回去。关节模式下，
    这些末端执行器只接受 `POSITION` 和 `PD`。躯体使用非关节模式（它自己保持平衡）时，动作交给躯体，
    `joint_names` 中指定的手指目标交给末端执行器（每个一次接收它的全部关节）；此时 `direct_joints` 包含
    所有末端执行器关节，`hold()` 让躯体自己保持，并让每个末端执行器保持在测得的姿态。触觉阵列等信号以
    `<name>_<key>` 出现在 `extra` 中。
  - `opening`（夹爪）：`Action.gripper[<name>]` 在任何模式下、经过安全门的 `gripper_range` 之后交给驱动，
    并从发给躯体的动作中去掉；[0, 1] 以外的值在任何东西动之前就 `ValueError`。读数放进 `gripper[<name>]`。
    它们不限制任何模式，`hold()` 不动它们。
- `microphones: Dict[str, MicrophoneDriver]`、`speakers: Dict[str, SpeakerDriver]`：在 `connect()` 时打开，
  在 `disconnect()` 时关闭；不向观测中放入任何内容。麦克风在 `first_audio_timeout_s` 内没有音频时只记录
  一条警告。
- `damp()`：让躯体（如果其驱动支持）和每个末端执行器进入阻尼。
- `estop()`：对躯体和每个末端执行器调急停，各自打断正在跑的 ramp。
- `disconnect()`：先断开躯体，再断开末端执行器，然后停止相机并关闭音频设备。

### `TwinRobot(real, sim, mode="mirror", mirror_hz=50.0)`

`hirobot.twin`。一个真机驱动与一个 `MujocoRobot` 数字孪生配对。需要 `sim` extra。

| 模式 | 指令发往 | 数字孪生 |
|---|---|---|
| `mirror` | 真机 | 后台线程把测得的关节状态复制到仿真中并渲染（纯运动学，无物理） |
| `shadow` | 只发往仿真 | 从真机姿态初始化一次。真机驱动必须是只读的，在 `connect()` 时检查。不提供机器人 skill |

- `active`：接收指令的驱动（shadow 模式下为 `sim`，否则为 `real`）。
- `mirror_error`：镜像线程最后抛出的异常，或 `None`。

## RobotGroup

`hirobot.group.RobotGroup(members, cameras=None, name="robot_group", control_hz=None, scene_cameras=(), first_frame_timeout_s=3.0, skew_warn_s=None, base_poses=None, expose=None)`。
把多台完整的机器人（每台都是一个 `make_robot` 栈，有自己的灵巧手、相机、仿真数字孪生、layer 和安全门）
加上场景相机作为一个 `Robot`。配置中有 `group:` 段时由 `make_robot` 构建，也可以直接构造：

```python
group = RobotGroup({"left": make_robot("franka_fr3"), "right": make_robot("ur_ur5e")},
                   cameras={"scene": scene_camera})
```

| 主题 | 行为 |
|---|---|
| 名字 | 在 `embodiment` 中，每个成员的关节、末端执行器、相机、麦克风和扬声器都命名为 `<member>/<name>`。场景相机保留自己的名字。`expose={"ee": {"left/ee": "left_arm"}, "cameras": {"scene": "agentview"}}` 重命名末端执行器和相机（不能重命名关节）；重命名后只有暴露出的名字有效，在 embodiment、观测和动作中都一样。名字未知或重命名后冲突时抛出 `ValueError` |
| 坐标系 | `base_poses={member: [x, y, z, qx, qy, qz, qw]}` 把成员放到世界坐标系中。此时 `ee_poses` 为世界坐标系，`ee_targets` 按世界坐标系理解，发送前转换到各成员的基座坐标系；`CARTESIAN_DELTA` 的平移和旋转按基座朝向旋转。没有基座位姿的成员保持自己的坐标系。对浮动基座成员，基座位姿是其里程计的原点：组把它与成员当前的 `base_position` / `base_orientation` 复合。这样的成员在没有报告 `base_position` 期间，其 `ee_poses` 放进 `extra["<member>/ee_poses"]`（自身坐标系），其末端执行器移出 `cartesian_targets`，发给它的世界坐标系目标会被拒绝；带 `extra={"frame": "base"}` 的 `members` 动作仍可给它自身坐标系下的目标 |
| `embodiment` | 关节按成员顺序拼接。恰好一个成员有浮动基座时 `floating_base` 为真；`control_hz` 默认取最慢成员的值 |
| `get_observation()` | 关节向量拼接；`ee_poses`、`gripper`、`images`、`depth`、`stamps`、`extra` 的键加前缀（`stamps["left/body"]`）；场景相机的画面用自己的名字加入；读各成员的时间差超过 `skew_warn_s` 时警告（默认半个周期）。`now()`：有真机成员时是主机时钟，否则是第一个成员的仿真时间。只有一个浮动基座成员时，它的 `base_*` 字段留在顶层；有多个时放进 `extra["<member>/base_*"]` |
| `send_action(action)` | 扁平或按成员。按成员（`Action.members`）：每个子动作原样发给对应成员，只做坐标系转换；模式可以各不相同。扁平：所有成员使用同一模式。`joint_target` / `kp` / `kd` 按成员关节数拆分，或在有 `joint_names` 时按名字拆分；`ee_targets` / `gripper` 按（暴露出的）名字拆分；`base_velocity` / `base_target` / `latent` / `motion` 发给唯一的浮动基座成员（否则 `SafetyError`）。发送任何部分之前先检查每一部分（`check_action`：故障、成员安全门、`direct_joints`、成员 `action_modes`）；只要有一处被拒绝，就什么都不发送。未被提及的成员收不到任何内容，保持其最后的目标；仿真成员前进一个周期（`idle_step`） |
| 能力 | `action_modes`：所有成员都接受的模式。`direct_joints`、`cartesian_targets`：各成员的值，加前缀。`motion_spaces`：恰好一个浮动基座成员时取它的值 |
| 错误 | 某个成员读取或发送失败，或报告了 `fault`，组就让其他所有成员保持，并进入自己的故障状态（`on_error: hold_all`）。此后 `send_action` 抛出 `RobotFaultError`，直到 `reset()` |
| 归属 | 成员会拒绝不经过组的 `send_action` / `run_robot_skill`（`RuntimeError`）。标记只在组持有的那个对象上，所以全身控制 layer 的线程仍能指挥它自己的内层机器人 |
| `member(name)` | 只读视图：`embodiment`、`simulated`、`fault`、`get_observation()`、`get_status()` |
| `get_status()` | `joint_temperature` 拼接；`joint_faults`、`board_temperature`、`extra` 的键加前缀；只有一个成员报告 `battery` 时放在顶层，否则放进 `extra["<member>/battery"]`；`fault` 取组自己的或第一个成员的 |
| Skill | 成员的 skill 列为 `<member>/<skill>`；`run_robot_skill("g1/wave")` 在该成员上运行它 |
| 生命周期 | `connect` 先连接成员再连接场景相机，失败时撤销。`reset`、`hold`、`damp`、`estop`、`disconnect`、`stop_robot_skill` 即使某个成员失败也会作用到每个成员，然后抛出第一个错误。`reset` 也会清除故障状态 |
| 安全门 | 组自己的安全门（组配置的 `safety:`）检查 `members` 动作中的每个数都有限；每个成员的安全门检查它自己那部分 |
| 碰撞 | 可选的 `collision=CollisionCheck(...)`（配置：`group.collision:`）。每个建了模的成员表示为一组胶囊体，其 MJCF（`hirobot.kinematics`）中每段 body 到父 body 的连线一个，半径都是 `radius`；只计算不同成员之间的间隙。发送前，组预测每个成员的下一个姿态（`POSITION` / `PD` 的关节目标作为下一时刻的角度，其他模式用当前角度；`CARTESIAN` 的目标点作为球体与其他成员比较），当最小间隙会低于 `min_distance` 时以 `SafetyError` 拒绝，除非间隙没有在缩小（已经靠得太近的成员可以分开）。每次发送读取每个建模成员的观测一次。浮动基座成员需要 `base_pose`。两台 G1 约 1 ms。沿运动链的胶囊体比较粗糙：选一个能覆盖连杆的 `radius`；`skip:` 排除安装板之类的 body |
| 全身控制成员 | 用 hiWBC 的 `wbc` layer 包装的成员（在其配置中写 `layers: [{type: wbc, profile: ...}]`）只接受 `BASE_VELOCITY` / `MOTION_LATENT` / `MOTION_CLIP`，所以组的 `action_modes` 通常为空：用 `Action.members` 指挥，手臂用关节模式，它用行走指令。它的空闲关节（`direct_joints`，例如上半身）可以在同一个子动作中按名字一并发送。未被提及时，`idle_step` 运行一个控制器 tick，让仿真中的人形机器人保持站立。控制器出错会让该成员进入故障并使组保持。`tests/test_group_wbc.py` 用 `echo` profile 覆盖了这种情况（没有 hiWBC 时跳过） |
| 示例 | `groups/franka_ur5e_dual`：Franka FR3（`left`）+ UR5e（`right`），相距 ±0.4 m 放置，一个场景相机暴露为 `agentview`，末端执行器暴露为 `left_arm` / `right_arm`。除非被覆盖（`group.members.<name>.overrides`），两个成员都使用 toy `sim` 驱动，所以不需要硬件就能运行 |

组配置的 `collision:` 段（路径相对于组配置文件）：

```yaml
group:
  collision:
    min_distance: 0.05        # m between capsule surfaces (default 0.05)
    radius: 0.05              # default capsule radius (m)
    backend: numpy            # or mujoco, for MJCFs the numpy parser rejects
    models:                   # members without a model are not checked
      left:  {mjcf: ~/models/fr3.xml}
      right: {mjcf: ~/models/ur5e.xml, radius: 0.06, skip: [base_mount]}
```


## 扩展点：相机、末端执行器与音频

相机、末端执行器（灵巧手、夹爪）和音频设备有各自的注册表，与本体驱动分开。与本体驱动一样，它们惰性导入厂商 SDK。

### 相机驱动（`hirobot.camera`，驱动在 `hirobot.drivers.camera` 中）

| 名字 | 说明 |
|---|---|
| `register_camera(name)`, `get_camera(name)`, `available_cameras()` | 注册表，行为与驱动注册表相同 |
| `CameraDriver(name)` | 抽象类：`start()`、`latest() -> Optional[Frame]`（`(image, Stamp)`，`Stamp.t` 在 `time.monotonic()` 上，每来一帧新的 `seq` 加一；不能阻塞；第一帧之前返回 `None`）、`stop()`（幂等）。RGB-D 驱动还要覆盖 `latest_rgbd() -> Optional[RgbdFrame]`；默认实现返回不带深度的 `latest()` |
| `PollingCameraDriver(name, poll_hz=30.0, retry_s=0.2)` | 在后台线程上运行阻塞的 `_grab() -> HxWx3 uint8 RGB`，只保留最新一帧。子类实现 `_grab`，可选实现 `_open` / `_close`。提供 `wait_for_frame(timeout)`、`error`、`frames`、`errors` |
| `Frame` | `Tuple[np.ndarray, Stamp]`：图像及其到达时间（`Stamp(t, seq)`，`t` 在 `time.monotonic()` 上） |
| `RgbdFrame` | `Tuple[np.ndarray, Optional[np.ndarray], Stamp]`：彩色图像、与之配准的深度（HxW float32，米）或 `None`，以及两者共同的 `Stamp` |
| `agibot_cosine` → `AgibotCosineCamera(name, topic, depth_topic=None, depth_scale=0.001, max_pair_skew_s=0.005, pair_wait_s=0.02, queue=4, poll_hz=60.0)` | 走机器人 `cosine_bus` topic 的 AgiBot G1 相机（`/camera/head_color` + `/camera/head_depth`、`/camera/hand_left_color`……）。设置了 `depth_topic` 时它是 RGB-D 相机：每个彩色帧与同一次采集（`source_timestamp`）的深度帧配对，Z16 换算为米。在机器人的计算机上运行，需先 source 厂商环境（`LOCATOR_IP`、`AORTA_DISCOVERY_URI`） |
| `unitree_videohub` → `UnitreeVideohubCamera(name, network_interface=None, domain_id=0, timeout_s=1.0, poll_hz=30.0)` | G1 头部相机 |
| `unitree_image_server` → `UnitreeImageServerCamera(name, address, header=False, crop=None, timeout_s=1.0, poll_hz=60.0)` | H1 头部相机：通过 ZeroMQ SUB 接收 Unitree 遥操作 `image_server`（或 `scripts/camera_image_server.py`）发出的 JPEG 帧。`header` 去掉 `Unit_Test` 的时间戳 / 帧号（保存在 `remote_stamp` / `remote_frame_id` 中）；`crop: [x, y, w, h]` 从左右拼接的画面中取出一个相机。需要 `camera` extra |
| `video_file` → `VideoFileCamera(name, path, fps=5.0, loop=True)` | 按自己的时间线播放的视频文件，以 `fps` 采样。不设 `loop` 时，文件结束后停留在最后一帧（`ended` 变为 `True`）。需要 `video` extra（PyAV）。仿真类 |
| `array` → `ArrayCamera(name, frames=None, fps=10.0, advance_on_read=False)` | 按主机时钟循环播放内存中的帧；没有 `frames` 时为一张 64x64 的灰色帧。`advance_on_read=True` 让每次 `latest()` 返回下一帧（用于测试）。仿真类 |
| `build_cameras(cameras, network_interface=None, sim_only=False) -> Dict[str, CameraDriver]` | 为每个带 `driver:` 的相机构建一个相机驱动；`sim_only` 只构建仿真类驱动。`make_robot` 用它构建 embodiment 的相机 |

不需要硬件的相机驱动类会设置 `simulated = True`。仿真本体只会构建这类驱动，而且它们不会拿到本体的
`network_interface`。

### 末端执行器驱动（`hirobot.end_effector`，驱动在 `hirobot.drivers.end_effector` 中）

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

末端执行器驱动以 `cls(ee.name, joints, **spec)` 构造，其中 `opening` 接口的 `joints` 为空，`spec` 是末端执行器的 `driver:` dict 去掉 `type`。
spec 没有设置 `network_interface` 时，`make_robot` 会填入本体的值。

### 音频设备（`hirobot.audio`，驱动在 `hirobot.drivers.audio` 中）

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

## 配置文件参考

由 `load_embodiment` / `make_robot` / CLI 读取的 YAML 文件，以内置名字（`"unitree_g1"`、
`"body/unitree/h1"`；用 `hirobot configs` 列出）或文件路径给出。内置名字先在包里找 `configs/<name>.yaml`，
再找 `configs/robots/<name>.yaml`，所以 `"unitree_g1"` 和 `"robots/unitree_g1"`（`hirobot configs` 列出的写法）
指同一个文件。以 `.yaml` / `.yml` 结尾或以 `/`、`~`、`.` 开头的字符串是路径。一个配置可以在 `defaults:` 下列出其他配置；
它们按顺序合并，文件本身最后合并。映射逐键合并；`joints`、`end_effectors`、`cameras`、`microphones` 和
`speakers` 按 `name` 合并（同名：字段合并；新名字：追加）；其他值整体替换。设备的 `driver:` 在 `type`
不变时逐键合并，`type` 改变时整体替换。`hirobot info --config <name> --resolved` 打印合并结果。规则和
示例见 `hirobot.config`。

```yaml
embodiment:                 # -> Embodiment (see above)
  name: unitree-g1
  kind: humanoid
  control_hz: 50
  floating_base: true
  joints:
    - name: left_hip_pitch_joint
    - {name: left_hand_thumb_0, position_limit: [-1.0472, 1.0472], torque_limit: 2.45}
  end_effectors:
    - name: left_hand
      joints: [left_hand_thumb_0, ...]
      driver: {type: unitree_handcmd, side: left}     # 可选的末端执行器驱动
    - name: gripper                                # 有自己驱动的夹爪：
      has_gripper: true                            # 不列 joints，所以用 'opening' 接口
      driver: {type: ...}
  cameras:
    - name: head
      width: 1920
      height: 1080
      driver: {type: unitree_videohub}             # optional camera driver
  microphones:                                     # optional audio devices
    - name: head
      driver: {type: unitree_voice_mic}
  speakers:
    - name: head
      driver: {type: unitree_voice_speaker, volume: 60}

body:
  driver: unitree_sdk2           # registered driver name; default: toy
  network_interface: eth0   # every other key -> driver constructor kwargs

safety:                     # optional; `safety: false` detaches the gate
  joint_limit: 3.0          # SafetyGate kwargs: joint_limit, clamp, gripper_range
  clamp: false

layers:                     # optional; registered layers, first innermost
  - type: my_wbc            # other keys -> the layer's constructor

sim:                        # optional MuJoCo twin for a real driver
  mode: mirror              # off (default) | mirror | shadow
  model: path/to/robot.xml  # required unless mode is off
  mirror_hz: 50
  viewer: true              # other keys -> MujocoRobot kwargs
```

在 `sim:` 下，`default_kp`、`default_kd`、`ramp_seconds` 和 `damping_kd` 未设置时从 `body:` 复制。

机器人组配置用 `group:` 段代替 `embodiment:` / `body:`（`make_robot` 拒绝在同一文件中同时出现两者）：

```yaml
group:
  name: dual_arm
  control_hz: 30              # optional; default: the slowest member's
  members:                    # name -> member; joint order follows this order
    left:  {config: franka_fr3, overrides: {ip: 172.16.0.2},   # overrides -> member's body:
            base_pose: [0, 0.4, 0, 0, 0, 0, 1]}      # optional; world [x, y, z, qx, qy, qz, qw]
    right: {config: ./my_ur5e.yaml, base_pose: [0, -0.4, 0, 0, 0, 0, 1]}  # relative: this file's dir
  cameras:                    # scene cameras, same format as embodiment.cameras
    - {name: scene, width: 640, height: 480, driver: {type: video_file, path: scene.mp4}}
  expose:                     # optional renames, e.g. to match training-data keys
    ee: {left/ee: left_arm, right/tcp: right_arm}
    cameras: {scene: agentview}
  on_error: hold_all          # the only value so far
safety: {}                    # the group's own gate; each member keeps its own
```

`make_robot(cfg, body_overrides={"right": {"ip": ...}})` 把每一项合并进对应成员的 overrides；
`sim_overrides` 发给每个成员。

policy 设置（模型服务器、chunk 长度）不属于机器人配置。

## CLI

安装后的命令名为 `hirobot`。

| 命令 | 说明 |
|---|---|
| `hirobot --version` | 打印版本 |
| `hirobot info --config NAME_OR_PATH [--resolved]` | 打印 embodiment：类型、控制频率、浮动基座、关节、末端执行器、相机。`--resolved` 打印合并 `defaults:` 之后的完整配置 |
| `hirobot configs [GROUP]` | 列出内置配置（`robots`、`body`、`end_effector`、`camera`、`audio`、`sim`、`groups`）。`robots/` 下的名字也可以不带前缀（`unitree_g1`） |
| `hirobot sim CONFIG [--seconds N] [--viewer] [--fast]` | 运行一台仿真机器人：连接，保持起始姿态 N 秒（默认 3），打印剩余的关节误差、基座高度、渲染出的图像和 `get_observation` 耗时。自由基座摔倒时会提示。对真机本体以退出码 2 退出；`--viewer` 需要 MuJoCo 本体 |
