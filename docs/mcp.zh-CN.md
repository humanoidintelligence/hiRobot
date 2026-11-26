# 用 AI agent 控制机器人（MCP）

[English](./mcp.md) | 简体中文

`hirobot mcp` 把一台机器人作为 [MCP](https://modelcontextprotocol.io) 服务器提供出来。
这样 LLM agent（Claude Code、Claude Desktop 或任何 MCP 客户端）就能读取机器人的状态和相机图像，
在你允许时还能给它发一小段一小段的动作。每条命令都和其他调用方一样，经过机器人的 `send_action`、
它的 `SafetyGate` 和一个命令租约。

本页是使用指南。工具列表、`mcp:` 配置键和 Python 接口在参考手册里：[MCP 服务器](./api.zh-CN.md#mcp-服务器)。

- [安装](#安装)
- [接入客户端](#接入客户端)
- [在仿真里第一次跑通](#在仿真里第一次跑通)
- [在真机上开放运动](#在真机上开放运动)
- [常见问题](#常见问题)

## 安装

MCP 服务器要 Python 3.10 或更新版本（`mcp` SDK 的要求）和 `mcp` 这个 extra，它同时装上用于相机图像的 Pillow：

```bash
pip install -e '.[mcp]'
hirobot mcp --help
```

在 Python 3.9 上这个 extra 装的时候不带 `mcp` 包，`hirobot mcp` 会提示这一点并退出。

## 接入客户端

用 stdio（默认）时，客户端自己启动 `hirobot mcp <config>`，通过这个进程的 stdin 和 stdout 通信，
你不用手动启动服务器。用 HTTP 时，你先启动服务器，客户端再连接一个 URL。

默认只开放 `observe` 工具（`describe_robot`、`get_observation`、`get_robot_status`）。要让 agent
移动机器人，加上 `--tools observe act`，或在配置里写 `mcp: tools:`。

### Claude Code

```bash
claude mcp add hirobot -- hirobot mcp ur_ur5e --tools observe act
claude mcp list
```

`--` 后面的就是 Claude Code 要运行的命令。不加 `-s` 时服务器在 `local` 范围：只对你、只在这个项目里。
`-s project` 则把它写进当前目录的 `.mcp.json`，可以随仓库共享：

```bash
claude mcp add -s project hirobot -- hirobot mcp ur_ur5e --tools observe act
```

```json
{
  "mcpServers": {
    "hirobot": {
      "type": "stdio",
      "command": "hirobot",
      "args": ["mcp", "ur_ur5e", "--tools", "observe", "act"],
      "env": {}
    }
  }
}
```

project 范围的服务器第一次要批准：下次在这个目录里启动 `claude` 时它会询问。在那之前
`claude mcp get hirobot` 显示它在等待批准。[`examples/mcp/mcp.json`](../examples/mcp/mcp.json)
是下面仿真示例现成的文件：把它复制为仓库根目录的 `.mcp.json`。

### Claude Desktop 和其他客户端

大多数客户端接受同样的 `mcpServers` JSON。在 Claude Desktop 里它写在
`claude_desktop_config.json`（Settings、Developer、Edit Config）：

```json
{
  "mcpServers": {
    "hirobot": {
      "command": "/absolute/path/to/env/bin/hirobot",
      "args": ["mcp", "/absolute/path/to/hiRobot/examples/mcp/ur5e_sim.yaml"]
    }
  }
}
```

- 客户端用它自己的 `PATH` 和工作目录启动命令，不是你 shell 的。如果 `hirobot` 不在那个 `PATH` 上
  （客户端不会激活的 conda 环境或 virtualenv），就写这个环境里 `hirobot` 可执行文件的绝对路径
  （在环境里运行 `which hirobot` 可以看到）。
- 相对于你 shell 目录的配置文件路径，从客户端的工作目录可能找不到。用绝对路径。`ur_ur5e` 这样的内置配置名在哪里都能用。
- 改完客户端配置后重启客户端。

### HTTP

你自己启动服务器，再让客户端指向它的 URL：

```bash
hirobot mcp examples/mcp/ur5e_sim.yaml --transport http --port 8000
# serves http://127.0.0.1:8000/mcp (streamable HTTP)
claude mcp add --transport http hirobot http://127.0.0.1:8000/mcp
```

服务器一直保持机器人连接，直到你用 Ctrl-C 停止它。它只监听 `127.0.0.1`；改 `--host` 之前先看
[在真机上开放运动](#在真机上开放运动)。`--path` 改 `/mcp` 这个路径。

## 在仿真里第一次跑通

[`examples/mcp/ur5e_sim.yaml`](../examples/mcp/ur5e_sim.yaml) 是内置的 UR5e（`ur_ur5e`，一台不需要
MuJoCo 的仿真机械臂），开放了动作工具，每次调用最多 10 个动作，并有一个安全包络：任何离上一个目标超过
0.05 rad 的关节目标都被拒绝：

```yaml
defaults: [ur_ur5e]
mcp:
  tools: [observe, act]
  max_steps: 10
  envelope: {max_joint_step: 0.05}
```

1. 把它接到客户端，例如在仓库根目录下：

   ```bash
   claude mcp add hirobot -- hirobot mcp examples/mcp/ur5e_sim.yaml
   ```

   客户端列出六个工具：`describe_robot`、`get_observation`、`get_robot_status`、`send_actions`、`hold`、`stop`。

2. 让 agent 描述这台机器人。它调用 `describe_robot`，返回关节（按关节列表使用的顺序）、动作模式和限制（有删节）：

   ```json
   {"name": "ur5e", "kind": "arm", "control_hz": 125, "floating_base": false,
    "joints": [{"name": "shoulder_pan", "position_limit": null, "torque_limit": null}, "..."],
    "end_effectors": [{"name": "tcp", "has_gripper": false}], "cameras": [],
    "action_modes": ["position", "pd", "velocity", "torque"],
    "tools": ["describe_robot", "get_observation", "get_robot_status", "send_actions", "hold", "stop"],
    "max_steps": 10}
   ```

3. 询问当前状态。`get_observation` 按名字返回关节：

   ```json
   {"observation": {
     "joint_pos": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.0,
                   "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0},
     "joint_vel": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.0,
                   "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0},
     "stamps": {"body": {"t": 0.0, "seq": 1}}}}
   ```

4. 让它把肘关节弯一点。一次 `send_actions` 调用如下。`position` 模式下 `joint_target` 设置每个关节：
   按 `describe_robot` 顺序的列表，或写全所有关节的 `{关节名: 值}`。

   ```json
   {"actions": [{"mode": "position",
                 "joint_target": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.04,
                                  "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0}}],
    "images": false}
   ```

   动作每个控制周期发一个（这里是 125 Hz），然后机器人保持不动。结果给出发了几个、为什么停下，以及新的观测：

   ```json
   {"sent": 1, "of": 1, "stopped": "done",
    "observation": {"joint_pos": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.04,
                                  "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0}, "...": "..."}}
   ```

5. 让它做一个大动作，例如一步把肘关节转到 0.5 rad。安全包络拒绝它，机器人保持不动，agent 读到原因：

   ```json
   {"sent": 0, "of": 1, "stopped": "refused",
    "error": "SafetyError: envelope of lease 'mcp': 1 joint target(s) move more than max_joint_step 0.050 rad from the last target; largest: 'elbow' by +0.460 rad",
    "observation": {"...": "..."}}
   ```

   好的 agent 接着会把这个动作拆成每步不超过 0.05 rad、每次调用最多 10 步。

6. `hold` 让机器人停在当前位置不动。`stop` 在下一步之前打断还在运行的 `send_actions`，然后保持不动。

[`examples/mcp/client.py`](../examples/mcp/client.py) 用一个 Python MCP 客户端跑同样的步骤，不涉及 LLM：
它通过 stdio 启动服务器，列出工具，调用 `describe_robot`，并发送一个动作。

### 带相机的人形机器人

MuJoCo 里的 G1 有头部相机，所以 `get_observation` 还返回一张 agent 能看的 JPEG。它需要 `sim` extra 和
G1 模型（见[仿真 G1](./setup/unitree.zh-CN.md#仿真-g1无机器人)）：

```bash
pip install -e '.[sim,mcp]'
claude mcp add hirobot-g1 -- hirobot mcp unitree_g1_mujoco
```

这样启动是只读观测。加 `--tools observe act` 后 agent 还可以动它的 29 个关节，骨盆像在龙门架上一样固定。
调用里 `images: false` 不要图像，`cameras: [...]` 选其中几个相机。

## 在真机上开放运动

MCP 服务器不增加通往硬件的新路径：agent 的动作和其他程序的动作一样被检查。变的是由谁决定发什么，
而模型可能出错。分阶段开放 `act`。

1. **先只观测。** 用默认工具提供真机配置，检查 agent 看到的关节、相机和状态是否正确。驱动有只读模式时就用它，
   并加 `--no-reset`。不加这个选项时 `hirobot mcp` 连接后会调用 `reset()`，它会升起增益并命令实测姿态，
   只读驱动会拒绝：

   ```bash
   hirobot mcp examples/unitree_g1_read_only.yaml --no-reset
   ```

2. **先在仿真里演练**，用同一配置的 `*_mujoco` 孪生和同样的 `mcp:` 段，直到 agent 发出的动作段看起来合理。

3. **收紧 MCP 租约。** `mcp: envelope:` 只给 agent 的命令设限制，比机器人自己的 gate 更严（见
   [安全包络](./api.zh-CN.md#安全包络)）。它只能收紧：比机器人 gate 宽的项被忽略并给出警告。

   ```yaml
   mcp:
     tools: [observe, act]
     max_steps: 5                   # short chunks: the agent looks again after each
     envelope:
       allowed_modes: [position]    # no velocity / torque from the agent
       max_joint_step: 0.03         # rad per command, from the last target
       joint_limit: 1.5             # |target| <= 1.5 rad
   ```

4. **保留机器人自己的限制。** 机器人配置的 `safety:` 段（见 [SafetyGate](./api.zh-CN.md#safetygate)）
   对每个调用方都生效，包括 agent：`max_joint_step`、`max_joint_error`、`joint_limit`、电量和温度阈值。

5. **从软开始。** 对人形机器人：吊在龙门架上，用低增益（驱动的 `default_kp`），并按它的安装指南做上电步骤，
   例如 [Unitree：上电与安全](./setup/unitree.zh-CN.md#3-上电与安全)。

6. **急停留在人手里。** 故意没有 `estop` 工具：急停是给人或看门狗用的，不是给模型用的。`stop` 只打断当前动作段并保持不动。
   把硬件急停放在手边，并准备好软件急停（`robot.estop()` 或厂商工具）。

7. **规划接管。** 命令仲裁只在一个进程内起作用，所以必须能从 agent 手里接管机器人的遥操作或应用，要和服务器在同一个进程里运行：
   在 Python 里构建机器人，用 `serve_mcp` 提供服务（见[在 Python 里用](./api.zh-CN.md#在-python-里用)），
   由另一段代码调用 `robot.claim(name, priority=Priority.TAKEOVER)`（见[命令仲裁](./api.zh-CN.md#命令仲裁)）。
   机器人 hold 一次，之后 agent 的每条命令都被拒绝：

   ```text
   the MCP lease was preempted by 'teleop'; the robot is someone else's now
   ```

   即使接管的租约释放了，MCP 服务器也不会再次 claim 机器人。`get_robot_status` 显示 `"mcp_lease": "preempted"`。
   要把控制交回去，重启服务器。用普通的 `hirobot mcp` 时没有别人能接管；停止服务器（Ctrl-C，或关闭客户端）会断开机器人。

8. **传输只留在本机。** 优先用 stdio：只有启动服务器的那个客户端能和它通信。HTTP 监听 `127.0.0.1`。
   其他任何 `--host` 都让能访问该端口的人命令机器人（服务器会打一条警告）；不要在你控制不了的网络上用。

## 常见问题

**`hirobot mcp: the MCP server needs the mcp package`。** 在 Python 3.10 或更新版本上用
`pip install -e '.[mcp]'` 安装这个 extra。在 Python 3.9 上这个 extra 会跳过 `mcp`；换更新的 Python。

**客户端没有运动工具。** 默认只开放 `observe`。在命令里加 `--tools observe act`，或在配置里写
`mcp: tools: [observe, act]`。调用一个没开放的工具会返回
`tool 'send_actions' is not enabled (mcp: tools: ['observe'])`。

**Claude Code 里完全看不到这些工具。** 运行 `claude mcp list` 和 `claude mcp get hirobot`。
来自 `.mcp.json` 的 project 范围服务器在你于该目录启动 `claude` 并批准之前一直处于等待状态。

**客户端启动不了 `hirobot`。** 客户端不用你 shell 的 `PATH` 和工作目录。用环境里 `hirobot` 的绝对路径，
以及绝对的配置路径或内置名。路径写错时服务器立刻退出，并在 stderr 打出 `hirobot mcp: config file not found: ...`。

**工具结果设了 `isError`。** 文本给出原因。agent 写错的调用（未知关节、缺参数、动作太多）这样返回，
在发出任何东西之前就被拒绝的命令（另一个租约持有机器人、MCP 租约被抢占）也这样返回。
`send_actions` 中没能发出的一步不一样：结果不是错误，它带 `"stopped": "refused"` 和写着原因的 `error`，机器人保持不动。常见原因：

| 文本开头 | 原因 |
|---|---|
| `SafetyError: envelope of lease 'mcp':` | `mcp: envelope:` 的限制拒绝了这个动作 |
| `SafetyError:`（其他） | 机器人的 `safety:` gate 拒绝了它 |
| `ValueError: 1 validation error for Action` | 动作不符合它的模式，例如 `position` 模式下 `joint_target` 映射只写了部分关节 |
| `the MCP lease was preempted by '<name>'` | 进程里另一个调用方以更高优先级拿走了机器人；重启服务器把它交回来 |
| `RobotBusyError: ... is held by '<name>'` | agent 第一次发命令时另一个租约持有机器人；那个调用方必须先释放 |

**机器人忙。** MCP 服务器在第一条命令时才拿它的租约（名为 `mcp`），不是启动时。如果另一个调用方持有同等或更高优先级的租约，
命令会以 `RobotBusyError` 被拒绝；那个租约释放后就能用。

**日志。** `hirobot mcp` 把日志写到 stderr，包括每次拒绝。用 stdio 时 stdout 是协议流：在你自己包装
`serve_mcp` 的程序里，不要往 stdout 打印，否则客户端会收到损坏的消息。
