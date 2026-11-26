# Control a robot from an AI agent (MCP)

English | [简体中文](./mcp.zh-CN.md)

`hirobot mcp` serves a robot as an [MCP](https://modelcontextprotocol.io) server.
An LLM agent (Claude Code, Claude Desktop, or any MCP client) can then read the
robot's state and camera images and, if you allow it, send it short chunks of
actions. Every command goes through the robot's `send_action`, its `SafetyGate`
and a command lease, like any other caller.

This page is the how-to guide. The tool list, the `mcp:` config keys and the
Python API are in the reference: [MCP server](./api.md#mcp-server).

- [Install](#install)
- [Connect a client](#connect-a-client)
- [First run in simulation](#first-run-in-simulation)
- [Allowing motion on a real robot](#allowing-motion-on-a-real-robot)
- [Troubleshooting](#troubleshooting)

## Install

The MCP server needs Python 3.10 or newer (the `mcp` SDK does) and the `mcp`
extra, which also brings Pillow for the camera images:

```bash
pip install -e '.[mcp]'
hirobot mcp --help
```

On Python 3.9 the extra installs without the `mcp` package, and `hirobot mcp`
says so and exits.

## Connect a client

Over stdio (the default) the client starts `hirobot mcp <config>` itself and
talks to it over the process's stdin and stdout. You don't start the server by
hand. Over HTTP you start the server and the client connects to a URL.

By default only the `observe` tools are on (`describe_robot`,
`get_observation`, `get_robot_status`). To let the agent move the robot, add
`--tools observe act` or set `mcp: tools:` in the config.

### Claude Code

```bash
claude mcp add hirobot -- hirobot mcp ur_ur5e --tools observe act
claude mcp list
```

Everything after `--` is the command Claude Code runs. Without `-s` the server
is in the `local` scope: only you, only this project. `-s project` writes it to
`.mcp.json` in the current directory instead, to share with the repository:

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

A project-scope server must be approved once: the next time you start `claude`
in that directory it asks. Until then `claude mcp get hirobot` shows it as
pending approval. [`examples/mcp/mcp.json`](../examples/mcp/mcp.json) is a
ready-made file for the simulation example below: copy it to `.mcp.json` in
the repository root.

### Claude Desktop and other clients

Most clients take the same `mcpServers` JSON. In Claude Desktop it goes in
`claude_desktop_config.json` (Settings, Developer, Edit Config):

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

- The client starts the command with its own `PATH` and working directory,
  not your shell's. If `hirobot` is not on that `PATH` (a conda env or
  virtualenv the client does not activate), give the absolute path to the
  env's `hirobot` executable (`which hirobot` in the env prints it).
- A config file path relative to your shell's directory may not resolve from
  the client's working directory. Use an absolute path. Built-in config
  names such as `ur_ur5e` work from anywhere.
- Restart the client after editing its config.

### HTTP

Start the server yourself, then point the client at its URL:

```bash
hirobot mcp examples/mcp/ur5e_sim.yaml --transport http --port 8000
# serves http://127.0.0.1:8000/mcp (streamable HTTP)
claude mcp add --transport http hirobot http://127.0.0.1:8000/mcp
```

The server keeps the robot connected until you stop it with Ctrl-C. It
listens on `127.0.0.1` only; see [Allowing motion on a real robot](#allowing-motion-on-a-real-robot)
before you change `--host`. `--path` changes `/mcp`.

## First run in simulation

[`examples/mcp/ur5e_sim.yaml`](../examples/mcp/ur5e_sim.yaml) is the built-in UR5e
(`ur_ur5e`, a simulated arm that needs no MuJoCo) with the action tools on,
up to 10 actions per call, and an envelope that refuses any joint target more
than 0.05 rad from the last one:

```yaml
defaults: [ur_ur5e]
mcp:
  tools: [observe, act]
  max_steps: 10
  envelope: {max_joint_step: 0.05}
```

1. Connect it to your client, e.g. from the repository root:

   ```bash
   claude mcp add hirobot -- hirobot mcp examples/mcp/ur5e_sim.yaml
   ```

   The client lists six tools: `describe_robot`, `get_observation`,
   `get_robot_status`, `send_actions`, `hold`, `stop`.

2. Ask the agent to describe the robot. It calls `describe_robot`, which
   returns the joints in the order joint lists use, the action modes and the
   limits (shortened):

   ```json
   {"name": "ur5e", "kind": "arm", "control_hz": 125, "floating_base": false,
    "joints": [{"name": "shoulder_pan", "position_limit": null, "torque_limit": null}, "..."],
    "end_effectors": [{"name": "tcp", "has_gripper": false}], "cameras": [],
    "action_modes": ["position", "pd", "velocity", "torque"],
    "tools": ["describe_robot", "get_observation", "get_robot_status", "send_actions", "hold", "stop"],
    "max_steps": 10}
   ```

3. Ask for the current state. `get_observation` returns the joints by name:

   ```json
   {"observation": {
     "joint_pos": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.0,
                   "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0},
     "joint_vel": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.0,
                   "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0},
     "stamps": {"body": {"t": 0.0, "seq": 1}}}}
   ```

4. Ask it to bend the elbow a little. A `send_actions` call looks like this.
   In `position` mode `joint_target` sets every joint: a list in
   `describe_robot` order, or `{joint name: value}` naming all of them.

   ```json
   {"actions": [{"mode": "position",
                 "joint_target": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.04,
                                  "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0}}],
    "images": false}
   ```

   The actions run one per control tick (125 Hz here), then the robot holds.
   The result says how many were sent and why it stopped, plus the new
   observation:

   ```json
   {"sent": 1, "of": 1, "stopped": "done",
    "observation": {"joint_pos": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.04,
                                  "wrist_1": 0.0, "wrist_2": 0.0, "wrist_3": 0.0}, "...": "..."}}
   ```

5. Ask for a large move, e.g. the elbow to 0.5 rad in one step. The envelope
   refuses it, the robot holds, and the agent reads why:

   ```json
   {"sent": 0, "of": 1, "stopped": "refused",
    "error": "SafetyError: envelope of lease 'mcp': 1 joint target(s) move more than max_joint_step 0.050 rad from the last target; largest: 'elbow' by +0.460 rad",
    "observation": {"...": "..."}}
   ```

   A good agent then splits the move into steps of at most 0.05 rad, up to 10
   per call.

6. `hold` keeps the robot still where it is. `stop` interrupts a
   `send_actions` that is still running, before its next step, then holds.

[`examples/mcp/client.py`](../examples/mcp/client.py) runs the same steps from
a Python MCP client, with no LLM involved: it starts the server over stdio,
lists the tools, calls `describe_robot` and sends one action.

### A humanoid with a camera

The MuJoCo G1 has a head camera, so `get_observation` also returns a JPEG the
agent can look at. It needs the `sim` extra and the G1 model (see
[Simulated G1](./setup/unitree.md#simulated-g1-no-robot)):

```bash
pip install -e '.[sim,mcp]'
claude mcp add hirobot-g1 -- hirobot mcp unitree_g1_mujoco
```

This starts observe-only. With `--tools observe act` the agent may also move
its 29 joints, with the pelvis welded in place as on a gantry.
`images: false` in a call skips the images, and `cameras: [...]` picks some.

## Allowing motion on a real robot

The MCP server adds no new path to the hardware: an action from the agent is
checked like one from any other program. What changes is who decides what to
send, and a model can be wrong. Open `act` in stages.

1. **Observe first.** Serve the real config with the default tools and check
   that the agent sees the right joints, cameras and status. Where the driver
   has a read-only mode, use it, with `--no-reset`. Without that option
   `hirobot mcp` calls `reset()` on connect, which ramps the gains and
   commands the measured pose, and a read-only driver refuses it:

   ```bash
   hirobot mcp examples/unitree_g1_read_only.yaml --no-reset
   ```

2. **Rehearse in simulation** with the same config's `*_mujoco` twin and the
   same `mcp:` section, until the agent's chunks look sensible.

3. **Tighten the MCP lease.** `mcp: envelope:` sets limits for the agent's
   commands only, stricter than the robot's own gate (see
   [Envelope](./api.md#envelope)). It can only tighten: an item looser than
   the robot's gate is ignored with a warning.

   ```yaml
   mcp:
     tools: [observe, act]
     max_steps: 5                   # short chunks: the agent looks again after each
     envelope:
       allowed_modes: [position]    # no velocity / torque from the agent
       max_joint_step: 0.03         # rad per command, from the last target
       joint_limit: 1.5             # |target| <= 1.5 rad
   ```

4. **Keep the robot's own limits.** The `safety:` section of the robot config
   (see [SafetyGate](./api.md#safetygate)) applies to every caller, the agent
   included: `max_joint_step`, `max_joint_error`, `joint_limit`, battery and
   temperature cut-offs.

5. **Start soft.** For a humanoid: hang it on a gantry, use low gains (the
   driver's `default_kp`), and follow its setup guide's power-on steps, e.g.
   [Unitree: power-on and safety](./setup/unitree.md#3-power-on-and-safety).

6. **A person keeps the e-stop.** There is no `estop` tool, on purpose: an
   e-stop is for a person or a watchdog, not for the model. `stop` only
   interrupts the current chunk and holds. Keep the hardware e-stop in reach
   and a software stop (`robot.estop()`, or the vendor's tool) ready.

7. **Plan the takeover.** The command arbiter works inside one process, so a
   teleoperation or app that must be able to take the robot from the agent
   runs in the same process as the server: build the robot in Python, serve
   it with `serve_mcp` (see [From Python](./api.md#from-python)), and let the
   other code call `robot.claim(name, priority=Priority.TAKEOVER)` (see
   [Command arbiter](./api.md#command-arbiter)). The robot holds once, and
   every later command from the agent is refused:

   ```text
   the MCP lease was preempted by 'teleop'; the robot is someone else's now
   ```

   The MCP server does not claim the robot again, even after the takeover
   lease is released. `get_robot_status` shows `"mcp_lease": "preempted"`.
   To give control back, restart the server. With plain `hirobot mcp`
   nothing else can take over; stopping the server (Ctrl-C, or closing the
   client) disconnects the robot.

8. **Keep the transport local.** Prefer stdio: only the client that started
   the server can talk to it. HTTP listens on `127.0.0.1`. Any other `--host`
   lets everyone who can reach that port command the robot (the server logs a
   warning); never use one on a network you don't control.

## Troubleshooting

**`hirobot mcp: the MCP server needs the mcp package`.** Install the extra
with `pip install -e '.[mcp]'` on Python 3.10 or newer. On Python 3.9 the
extra skips `mcp`; use a newer Python.

**The client shows no motion tools.** Only `observe` is on by default. Add
`--tools observe act` to the command, or `mcp: tools: [observe, act]` to the
config. A call to a tool that is off returns
`tool 'send_actions' is not enabled (mcp: tools: ['observe'])`.

**The tools don't show up at all in Claude Code.** Run `claude mcp list` and
`claude mcp get hirobot`. A project-scope server from `.mcp.json` stays
pending until you approve it when `claude` starts in that directory.

**The client can't start `hirobot`.** The client does not use your shell's
`PATH` or working directory. Use the absolute path to the env's `hirobot`,
and an absolute config path or a built-in name. A wrong path ends the server
at once with `hirobot mcp: config file not found: ...` on stderr.

**A tool result has `isError` set.** The text says why. A call the agent got
wrong (an unknown joint, a missing argument, too many actions) comes back
this way, and so does a command refused before anything was sent (another
lease holds the robot, the MCP lease was preempted).
A step `send_actions` could not send is different: the result is not an
error, it has `"stopped": "refused"` and `error` with the reason, and the
robot holds. Common reasons:

| Text starts with | Cause |
|---|---|
| `SafetyError: envelope of lease 'mcp':` | The `mcp: envelope:` limits refused the action |
| `SafetyError:` (other) | The robot's `safety:` gate refused it |
| `ValueError: 1 validation error for Action` | The action is malformed for its mode, e.g. a `joint_target` mapping that names only some joints in `position` mode |
| `the MCP lease was preempted by '<name>'` | Another caller in the process took the robot with a higher priority; restart the server to hand it back |
| `RobotBusyError: ... is held by '<name>'` | Another lease held the robot when the agent first commanded it; that caller must release it |

**Robot busy.** The MCP server takes its lease, named `mcp`, on the first
command, not at start. If another caller holds a lease of equal or higher
priority, the command is refused with `RobotBusyError`; it works once that
lease is released.

**Logs.** `hirobot mcp` logs to stderr, including each refusal. Under stdio,
stdout is the protocol stream: in your own wrapper around `serve_mcp`, never
print to stdout, or the client sees corrupt messages.
