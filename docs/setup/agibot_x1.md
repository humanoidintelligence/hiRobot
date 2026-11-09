# AgiBot X1 setup (ROS2 / AimRT bridge)

English | [简体中文](./agibot_x1.zh-CN.md)

Driver: `agibot_aimrt` (`src/hirobot/drivers/body/agibot/aimrt/`).

**Unlike the other vendors, AgiBot X1 has no Python API.**
[agibot_x1_infer](https://github.com/AgibotTech/agibot_x1_infer)
(xyber_x1_infer) is a pure C++ / AimRT stack. The practical way to drive X1
from hiRobot is a ROS2 node that talks to it through AimRT's `ros2` channel
backend. The driver is exactly such an **rclpy bridge**:

- It subscribes to `/joint_states` (`sensor_msgs/JointState`) and `/imu/data`
  (`sensor_msgs/Imu`) for observations.
- It publishes `/joint_cmd` (`my_ros2_proto/JointCommand`: name / position /
  velocity / effort / **stiffness** / **damping**) for commands, where
  `stiffness` / `damping` are the PD gains kp / kd.

## ⚠️ Bridge prerequisites (must hold on the robot side, or the driver fails at connect)

1. **Enable the ros2 subscribe backend:** in the AimRT config,
   `sub_topics_options` defaults to `local` only, so an external `/joint_cmd`
   never gets in. Change it to `[local, ros2]`. (On the publish side,
   `pub_topics_options` already includes `ros2` by default, so state reaches
   external nodes.)
2. **Run only the low-level driver module:** the AimRT package you run must
   expose the low-level driver (`DcuDriverModule` on hardware, `SimModule` in
   sim) and **drop the bundled `ControlModule`**. Otherwise `ControlModule`
   (the built-in RL walking controller) also publishes `/joint_cmd` and fights
   you for control.
3. **Build the custom messages:** `my_ros2_proto` must be built with
   `colcon build` and sourced, so Python can
   `from my_ros2_proto.msg import JointCommand`.

Once these three hold, the driver works; the observation / action mapping
itself is complete.

## 1. Prerequisites

- A ROS2 (Humble) environment with `rclpy` and `sensor_msgs`.
- `my_ros2_proto` (from `agibot_x1_infer`) built and sourced.
- Robot side: build the AimRT stack with `agibot_x1_infer`'s `build.sh`, apply
  the bridge prerequisites above, then launch with `run.sh` (hardware) or
  `run_sim.sh` (sim).

## 2. Power-on and safety

- It is a humanoid: it must be hung or supported, with the e-stop ready.
- Mode switching, zeroing, etc. are handled by the AimRT stack; the driver's
  `reset()` is a no-op.
- `send_action` matches joints by **name** (not index), so the joint names in
  the config must match the AimRT stack.

## 3. Configuration

`agibot_x1`:

```yaml
body:
  driver: agibot_aimrt
  # node_name: hirobot_agibot_x1
  # default_kp: 100.0        # fallback stiffness
  # default_kd: 2.0          # fallback damping
  # spin_timeout: 5.0        # seconds to wait for the first /joint_states
```

The joint order follows the 29-joint `joint_list` of the ControlModule in
`rl_x1.yaml` (waist 3 + arms 14 + legs 12). If you drive the DCU layer directly
it has 32 joints (adds left/right claws, in a different order); adjust the
config accordingly.

## 4. Minimal loop

Check the connection with the basic loop from the
[Quick start](../../README.md#simulation), run on your local config (built-in
config plus the driver and address above): read an observation, send the
current joint positions back as a POSITION target, and hold.

```python
from hirobot import Action, ControlMode, make_robot

robot = make_robot("my_x1.yaml")   # your local config with the driver and address
robot.connect()
robot.reset()
obs = robot.get_observation()
robot.send_action(Action(mode=ControlMode.POSITION, joint_target=obs.joint_pos))
robot.hold()
robot.disconnect()
```

What the driver does: `connect()` starts an rclpy node, creates the publisher
and subscribers, and spins until the first `/joint_states` arrives (on timeout
after `spin_timeout` it raises an error pointing to this guide).
`get_observation()` assembles the cached JointState (position / velocity /
effort) and Imu. `send_action()` (POSITION / PD) publishes a JointCommand with
`stiffness/damping` set from kp/kd.

## 5. Common pitfalls

- **`connect` raises ImportError:** rclpy / sensor_msgs is missing, or
  `my_ros2_proto` has not been built or sourced.
- **Timeout waiting for /joint_states:** the AimRT process is not running, or
  the low-level driver module is not publishing state.
- **Commands are sent but the robot does not move:** most likely prerequisite 1
  (the subscribe side has no ros2 backend) or prerequisite 2 (`ControlModule`
  is still competing for `/joint_cmd`).
- **Only teleop walking works, no joint targets:** you are connected through
  the official ControlModule path (`/cmd_vel` plus mode-trigger topics), which
  only walks with the built-in RL policy and is not joint-level VLA control.
  Joint-level control requires bypassing ControlModule and talking directly to
  DcuDriverModule / SimModule.
- **Joint name mismatch:** `/joint_cmd` matches by name; misspelled joint names
  in the config are ignored.
