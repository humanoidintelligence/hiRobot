# 更新日志

每个 hiRobot 版本用户看得到的变化，新的在上面。格式参照
[Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)。0.1.0 之前任何版本都可能有不兼容的接口改动，
这类改动列在"移除"或"变更"下，并标 **不兼容**。

## [0.0.5] - 2026-11-14

技能方法改名。关节可以写转轴和挂接的 link，Unitree G1 的关节带上了限位、力矩上限和转轴。

### 新增

- `Joint.axis` 和 `Joint.parent_link`，都可选，本体配置里可以写。`axis` 是关节转轴的单位向量，在该关节带动的 link 的坐标系里
  （同 URDF `<axis>` 和 MJCF `axis`），不是世界坐标系。`parent_link` 是关节挂在哪个 link 上（URDF `<parent>`）。
- Unitree G1 本体（`body/unitree/g1`）的 29 个关节都写上了 `position_limit`、`torque_limit`、`axis` 和 `parent_link`，取自 Unitree
  `unitree_ros` 里的 `g1_29dof_rev_1_0.urdf`。hiRobot 不按这些单关节限位裁剪命令，它们只用来描述机器人。

### 变更

- **不兼容**：技能方法改名。包含全部技能（层技能和机器人控制器自带的）的那个合集现在叫「robot skill」；
  机器人控制器自己运行的技能现在叫「onboard skill」（机载 skill）。层技能和 `RobotSkillSpec` / `RobotSkillResult` /
  `RobotSkillError` 类型名不变。

  | 0.0.4 | 0.0.5 |
  |---|---|
  | `list_skills()` | `list_robot_skills()` |
  | `run_skill(name, args)` | `run_robot_skill(name, args)` |
  | `stop_skill()` | `stop_robot_skill()` |
  | `skill_running` | `robot_skill_running` |
  | `has_robot_skills` | `has_onboard_skills` |
  | `list_robot_skills()` | `list_onboard_skills()` |
  | `run_robot_skill(name, args)` | `run_onboard_skill(name, args)` |
  | `stop_robot_skill()` | `stop_onboard_skill()` |
  | `_run_robot_skill`（驱动 SPI） | `_run_onboard_skill` |

  `CommandLease.run_skill` 改为 `run_robot_skill`，`CommandLease.run_robot_skill` 改为 `run_onboard_skill`。

  **注意：**`list_robot_skills`、`run_robot_skill` 和 `stop_robot_skill` 这三个名字还在，但现在指合集。按 0.0.4 写的代码调用它们
  会连层技能一起拿到，不会报错。覆写了 `list_robot_skills` 或 `_run_robot_skill` 的驱动要改成 `list_onboard_skills` /
  `_run_onboard_skill`，否则它的技能不会再出现。
- `Joint` 现在拒绝下限大于上限的 `position_limit`、不大于 0 的 `torque_limit` 和不是单位向量的 `axis`，带这类值的本体配置会加载失败。
  内置配置都能通过。
- `docs/setup/unitree.md`：G1 机载 skill 要在哪个 FSM 状态下用（手势只在 500、501 或 801）、在一台 G1 上验证过的起身顺序、
  `SetFsmId` 被忽略时也返回 0、机械臂错误码 7400–7404，以及 `videohub_pc4` 启动失败 30 次后头部相机为什么一直不恢复。

## [0.0.4] - 2026-11-13

支持 Franka FR3 和 AgileX PiPER，以及 ZED 和 Orbbec 相机。Franka FR3 改为通过 DROID 机器人服务控制，libfranka 驱动已移除。

### 新增

- **AgileX PiPER 机械臂**（新驱动 `agilex_piper_sdk`）：走 AgileX 的 `piper_sdk`，经 SocketCAN。读关节、法兰位姿和夹爪；
  POSITION 模式的关节运动由机械臂控制器按设定速度规划，夹爪走 `Action.gripper`。`connect()` 和读状态不往总线上发任何帧。
  配置 `body/agilex/piper` 和 `robots/agilex_piper`（机械臂、夹爪和一台 Orbbec 场景相机）。在真机 PiPER 上验证过：读状态、
  上移 10 cm 再回来、夹爪。`estop()` 还没在真机上验证。
- **Orbbec 相机**（新驱动 `orbbec_pyorbbecsdk`）：用 1.x 版 `pyorbbecsdk`，本地 USB，按 USB 端口或序列号选相机；彩色，
  可选对齐到彩色的深度（米）。配置 `camera/orbbec/dabai_dc1`。在 Dabai DC1 上验证过：640x480 和 1280x720，30 fps 带深度。
- **DROID 平台上的 Franka 机械臂**（新驱动 `franka_droid`）：[DROID](https://github.com/droid-dataset/droid) 机器人服务
  （zerorpc，端口 4242）的客户端，只用上游 DROID 就有的服务方法，所以 hiRobot 这边不需要实时内核和 libfranka，只要
  `pip install zerorpc`。读关节、末端位姿和夹爪；POSITION（`update_joints`）和 CARTESIAN（`update_pose`）控制，夹爪走
  `Action.gripper`。配置 `body/franka/fr3_droid` 和 `robots/franka_fr3_droid`（机械臂加三台 ZED 相机）。在带 Robotiq 2F-85
  的真机 FR3 上验证过：读状态、CARTESIAN 末端上移 10 cm 再回来、夹爪。POSITION、`reset(home_q)` 和 `estop` 还没在真机上验证。
- **Stereolabs ZED 相机**（新驱动 `stereolabs_zed`）：走 pyzed，本地 USB，按序列号选相机；左目彩色，可选对齐到它的深度（米）。
  配置 `camera/stereolabs/zed2i` 和 `camera/stereolabs/zed_mini`。在两台 ZED 2i 加一台 ZED Mini 同时运行时验证过
  （HD720、30 fps、带深度）。
- **技能合成一张表**：每个机器人都有 `Robot.list_skills()`、`run_skill(name, args)`、`stop_skill()` 和 `skill_running`，
  机器人技能和层技能一起列、一起调，调用方不用区分。层技能和机器人技能同名时层技能优先；组装里技能名带部件路径。
  `CommandLease.run_skill` 也有。原来按类别分的八个方法保留，给驱动和层用。见 [技能](docs/api.zh-CN.md#技能)。
- `docs/setup/agilex_piper.md`：PiPER 和它的 Orbbec 相机的设置指南。`docs/setup/franka.md` 只讲 DROID 机器人服务。
- `hirobot.geometry.transforms`：`euler_xyz_to_quat` 和 `quat_to_euler_xyz`（外旋 `xyz` 欧拉角，即 scipy 的 `"xyz"`）。

### 变更

- Franka FR3、AgileX PiPER 及其夹爪、Orbbec Dabai DC1、ZED 2i / ZED Mini 在 README 和 `docs/supported_robots.md`
  里标为已验证（✅），照片墙上加了它们的照片。
- `docs/api.zh-CN.md` 调整了小节顺序：新的第一节「Robot 一览」按用途列出 `Robot` 的全部成员；接着是快速示例、`make_robot`、
  `Robot` 和数据类型，给开发者的小节（驱动 SPI、扩展点）放到最后。`Robot` 表里补上了 `cameras` / `microphones` / `speakers`、
  `accepts_parts`、`claim` 和 `command_events`。
- README 顶部加了文档索引，和语言切换在同一行。

### 移除

- **不兼容**：`franka_libfranka` 驱动（`FrankaRobot`，基于 `pylibfranka`）和 `franka_fr3` / `body/franka/fr3`
  配置。它从没在真机上跑过。Franka FR3 通过 DROID 机器人服务控制（`franka_droid`、`franka_fr3_droid`）。
  `franka_ur5e_dual` 的左臂改为 `body/franka/fr3_droid`，名字（`left/joint1`、`left/ee`）不变。

## [0.0.3] - 2026-11-12

支持 UR 机械臂，新增两个驱动：Robotiq 夹爪和 Intel RealSense 相机。没有不兼容的改动。

### 新增

- **UR 机械臂上真机。** `ur_rtde` 驱动在真机 UR5（CB3）上验证过，单臂和双臂组装 `ur5e_dual` 都跑过：读关节和 TCP 位姿，
  CARTESIAN 模式（`servoL`）控制，`control_hz: 125`。`reset` / `estop` 和 POSITION / PD / VELOCITY 模式还没在真机上验证。
- `body/ur/ur5e` 加了 `flange` 挂载点（`wrist_3_link`），给末端执行器用。
- **Robotiq 2F 夹爪**（新驱动 `robotiq_urcap`）：通过 UR 控制箱上 Robotiq URCap 的 socket（端口 63352）通信，不需要额外接线和
  Python 包。`opening` 接口（0 合、1 开）。`activate` 默认关，因为激活时夹爪会完整开合一次。配置
  `end_effector/robotiq/2f85_urcap` 和 `robots/ur_ur5e_robotiq`。在真机 2F-85 上验证过。
- **Intel RealSense 相机**（新驱动 `intel_realsense`）：用 pyrealsense2，本机 USB 连接，输出彩色图和对齐到彩色的深度图。
  `depth_width` / `depth_height` 可以和彩色分辨率分开设（L515 接 USB 2 时深度只有 320×240）。配置
  `camera/intel/d435i_realsense` 和 `camera/intel/l515_realsense`。在 D435i（pyrealsense2 2.59.1）和 L515（2.54.2；更新的 SDK
  列不出 L515）上验证过，两台同时开各 30 fps。
- `RobotStatus.command`（`CommandProgress`：`mode`、`done`、`goal`、`clip_time`、`clip_duration`）：高层命令（BASE_POSE、
  MOTION_CLIP）的进度，由层往上报。组装里只有一个部件报时放在顶层，否则放在 `extra["<part>/command"]`。hiRobot 自带的驱动填 `None`。
- `docs/setup/ur.md`：加了真机验证状态、双臂配置、RealSense 相机、Robotiq 夹爪和 L515。
- `docs/api.md`：加了 `RobotStatus` 一节，`get_status` 列进 `Robot` 的方法表，RealSense 和 Robotiq 列进驱动表。

### 变更

- 层技能的四个方法（`list_layer_skills`、`run_layer_skill`、`stop_layer_skill`、`layer_skill_running`）移到 `Robot` 基类，
  没有层的机器人也能调，返回默认值。
- README 和 `docs/supported_robots.md` 里 UR5e、Robotiq 2F-85、RealSense L515 标为已验证（✅）；那里的相机表把
  "D435i / D405" 和 "L515" 分成两行。
- UR5e 的 MuJoCo 测试兼容新旧两版 MuJoCo Menagerie 模型（关节名和底座朝向不同）。

### 弃用

- 旧的单机配置格式（`embodiment:` + `body:`）还能加载，会发 `DeprecationWarning`；计划下一版删掉。

## [0.0.2] - 2026-11-11

嵌套组装树，重新安排了层次。

### 新增

- **组装树。** 机器人配置是一棵 `assembly:` 树：每个节点是一份设备文件或一份完整的单机配置，`make_robot` 把整棵树建成一个
  `RobotAssembly`。内置配置都改用这种格式；新增配置 `franka_ur5e_dual` 和 `ur5e_dual`。
- `RobotAssembly` 可以带子部件：几台完整的机器人当成一个 `Robot` 驱动，处于同一个坐标系。`robot.part("left")` 取一个部件；
  `placement:` 按部件解析；`overrides={"left/body": ...}` 覆盖一个部件的配置。
- 租约可以只持有某几个部件（`claim(..., parts=[...])`）；层可以只套在一个部件上（`layers: [{part: left, ...}]`）。
- 整棵树放进一个共享的 MuJoCo 世界（`sim:`）；安全门的仿真检查按树建（`safety.sim_check`）；
  `hirobot check --sim [--against-real]`。
- `toy` 本体可以当移动底盘（`mobile_base: true`）。
- **命令仲裁器**，在最外层的机器人上：`robot.claim(name, priority)` 返回租约；`TAKEOVER` > `LAYER_SKILL` > `CALLER`，
  高优先级抢占；租约可以带更严的 `envelope`；claim、抢占和拒绝记进审计（`robot.command_events()`）。
- **命令看门狗**：速度命令超过 `command_timeout_s` 没刷新就停下。
- `Layer` 基类和层技能（`run_layer_skill`，运行时以 `LAYER_SKILL` 持有租约）；文档加了"编写层"。
- 安全门：步长和跟踪误差上限、电量和温度检查、逐关节限位、逐夹爪范围、可选的 MuJoCo 推演检查。
- `hirobot view`：在 MuJoCo 里只读地显示真机的实测姿态。

### 变更

- 栈从下往上：驱动 → 一个 `RobotAssembly` → 硬件 `SafetyGate` → 层 → 命令仲裁器。
- `estop()` 先直接打到最底层驱动，再逐层清理；层只能看到 `inner` 的公开接口。
- **不兼容：** 驱动和层的钩子（SPI）改成 `_` 开头。
- **不兼容：** `src/hirobot` 下的模块分到 `core/`、`devices/`、`geometry/`、`loading/`、`assembly/`、`control/`、`sim/`。
  `from hirobot import ...` 不变；深路径 import 变了（如 `hirobot.kinematics` → `hirobot.geometry.kinematics`）。
- **不兼容：** `Action.members` 改名 `Action.parts`。
- README：加了硬件照片墙；支持的机器人列表移到 `docs/supported_robots.md`；快速开始移到架构前面；重画了架构图。
  `docs/api.md` 分成应用 API 和 SPI 两部分。

### 弃用

- 旧的单机配置格式（`embodiment:` + `body:`）经 `hirobot.loading.legacy` 还能加载，会发 `DeprecationWarning`。

### 移除

- **不兼容：** `RobotGroup` 和 `group:` 配置；改用带子部件的 `RobotAssembly`。
- **不兼容：** sim twin（`TwinRobot`、mirror / shadow、`sim_overrides`）；看真机改用 `hirobot view`。

## [0.0.1] - 2026-11-09

首次对外发布：人形机器人和机械臂、仿真和真机共用一个 `Robot` 接口。控制代码、VLA 和 agent 对着 `Observation` / `Action`
写一次，就能在任何有驱动的机器人上运行。

### 新增

- **Robot 接口约定**：`connect` / `reset` / `get_observation` / `send_action` / `set_gains` / `disconnect`，加上 `get_status`
  （电量、温度、关节故障）、可以从任意线程打断正在运行的 `reset` 或 `connect` 的 `estop()`，以及列出驱动接受什么的
  `action_modes` / `motion_spaces`。`Observation.stamps` 给每个来源（本体、手、相机）单独的时间戳和序号。
- **安全**：每个动作到达驱动之前都经过 `SafetyGate`（形状、有限值、关节和夹爪范围）；被拒绝时抛出 `SafetyError`。
- **配置**：机器人用 YAML 描述（按名字的关节、浮动基座、末端执行器、相机、控制频率），每个部件一个文件，用 `defaults:` 组合。
  `make_robot("unitree_g1")` 建出整个栈。命令行有 `hirobot info`、`hirobot configs` 和 `hirobot sim`。
- **末端执行器**：灵巧手和夹爪共用一个驱动接口约定。手是 `joint_target` 里的关节；夹爪是 [0, 1] 的开合度
  （`Observation.gripper` / `Action.gripper`，0 合、1 开），任何控制模式下都能用。
- **仿真**：MuJoCo 机器人，名字、关节顺序和相机都和真机一致，可以单独用，也可以当真机的 `mirror` 孪生，或者当 `shadow`
  从真机当前姿态预演动作。测试用不依赖任何包的 `toy` 驱动。
- **多台机器人当一台**：`RobotGroup` 把几台完整的机器人（可以是不同厂商）和场景相机组合成一个 `Robot`，处于同一个世界坐标系，
  带成员之间的距离检查。
- **扩展点**：从自己的包通过 `hirobot.drivers` entry point 注册本体驱动、末端执行器、相机、音频设备和 `layers:`（如全身控制器）。
- **机器人技能**：厂商自带的动作（G1 行走、手臂手势）按名字启动，每个带说明和参数的 JSON Schema。
- 真机验证过：Unitree G1 和 H1 本体；Unitree Dex3-1 手（G1）；Inspire RH56DFX 手（H1）；AgiBot G1 本体、两个 Omnipicker 夹爪
  和底盘，以及它的头部（RealSense D457）和腕部（RealSense D405）相机；Unitree G1 和 H1 上的 RealSense D435i 头部相机。
  Fourier GRx、AgiBot X1、Franka FR3 和 UR5e 的驱动已包含，还没在真机上验证。

[0.0.5]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.4...v0.0.5
[0.0.4]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.3...v0.0.4
[0.0.3]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.2...v0.0.3
[0.0.2]: https://github.com/humanoidintelligence/hiRobot/compare/v0.0.1...v0.0.2
[0.0.1]: https://github.com/humanoidintelligence/hiRobot/releases/tag/v0.0.1
