# Fourier GR1 setup (GR1T1 / GR1T2)

English | [简体中文](./fourier.zh-CN.md)

Driver: `fourier_grx` (`src/hirobot/drivers/body/fourier/grx/`). Wraps
`fourier_grx_client.RobotClient`, which connects over Zenoh to a GRX server
(default `server_ip:7447`). GR1T1 and GR1T2 share the same 32-joint API; only
the embodiment config differs.

## 1. Prerequisites

Install the client ([fourier-grx-client](https://github.com/FFTAI/fourier-grx-client)):

```bash
pip install fourier-grx-client
```

**The GRX server must already be running on the robot** (provided by the
vendor); the client only connects to it.

> Units: since client v0.2.0 every user-facing value is in **radians**. The
> driver reads and writes radians and never passes `degrees=True`. The client
> reports IMU angular velocity in deg/s; the driver converts it to rad/s.

## 2. Network

- The host must reach the GRX server's IP (often `localhost` on the robot
  itself, or an IP on the same subnet).
- The port is fixed at **7447** (TCP / Zenoh).
- `namespace` is resolved by default from the `GRX_ROBOT_ID` environment
  variable or `~/.fourier/robot_id.yaml`; you usually don't need to set it.

## 3. Power-on and safety

1. The robot is hung or supported, the area is clear, and the e-stop is within reach.
2. Start the GRX server.
3. **Enable the motors:** commands only take effect after `set_enable(True)`.
   The driver enables in `connect()` and `reset()` (`enable_on_connect=True`).
4. **Power down:** the client's `close()` does **not** disable the motors. The
   driver's `disconnect()` calls `set_enable(False)` before `close()`; always
   call it when you are done.
5. To calibrate zero positions, call `client.calibrate_sensors()` on site (not
   wrapped by this driver; do it manually when needed).

## 4. Configuration

`fourier_gr1t1`:

```yaml
body:
  driver: fourier_grx
  server_ip: localhost      # GRX server address
  # namespace: gr/my_robot  # usually resolved automatically
  # freq: 400               # state refresh rate
  # enable_on_connect: true
```

The joint order follows the 32-joint list in `config_GR1_T1.yaml` /
`config_GR1_T2.yaml`. Gain arrays must have exactly 32 entries, or the client
**silently ignores** them.

## 5. Minimal loop

Check the connection with the basic loop from the
[Quick start](../../README.md#simulation), run on your local config (built-in
config plus the driver and address above): read an observation, send the
current joint positions back as a POSITION target, and hold.

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

What the driver does: `connect()` builds a `RobotClient` (which connects to
Zenoh in its constructor and blocks until the first state arrives) and enables
the motors. `get_observation()` reads `joint_positions/velocities/efforts`
(rad) and the IMU. `send_action()` uses the high-rate path
`control_joints("position", ..., degrees=False)`.

## 6. Common pitfalls

- **Enable first:** without enabling, the motors ignore commands.
- **Don't use `move_joints` at high rate:** the hot control loop uses
  `control_joints` (a single lightweight publish, REAL_TIME + DROP).
  `move_joints(duration>0)` / `movej` / `movel` start a thread or block on
  planning and are unsuitable at high rate. The driver uses `control_joints`.
- **Gain length:** kp/kd must be length-32 arrays, or they are ignored.
- **Cannot connect:** building a `RobotClient` runs a liveness check
  (`robot/info`) and raises `FourierConnectionError` on failure. Check that the
  server is running and that the IP and port 7447 are reachable.
- **Dexterous hands:** the GR1 hands use a separate interface
  (`fourier_dhx.sdk.DexHand`) that this driver does not cover; integrate it
  separately if needed.
