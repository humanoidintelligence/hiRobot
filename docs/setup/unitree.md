# Unitree setup (G1 / H1-2 / H2 / H1)

English | [简体中文](./unitree.zh-CN.md)

Driver: `unitree_sdk2` (`src/hirobot/drivers/body/unitree/sdk2/`). Talks over the SDK2 DDS
interface: publishes `rt/lowcmd` and subscribes to `rt/lowstate`.

- **G1 / H1-2 / H2** use the `unitree_hg` message family (`msg_family: hg`, the default).
- **Legacy H1 (19 DoF)** uses the `unitree_go` message family (`msg_family: go`).
- **G1 robot skills** (walk, sit, arm gestures run by the built-in controller)
  are an alternative to joint control: `builtin_skills: g1`, see
  [Robot skills (G1)](#robot-skills-g1).

## 1. Prerequisites

Install the Python SDK ([unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)):

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

It depends on `cyclonedds`. The driver can be constructed and run in sim
without the SDK; only a hardware `connect()` needs it.

## 2. Network

- Connect the robot to the host with an Ethernet cable and put the host NIC on
  the robot's subnet (Unitree robots default to `192.168.123.x`).
- Note your NIC name (e.g. `eth0` / `enp3s0`) and set it as
  `body.network_interface` in the config. It is the second argument of
  `ChannelFactoryInitialize(0, <iface>)`. If unset, the SDK auto-detects.

## 3. Power-on and safety

1. The robot is **hung or supported**, with the area around it clear.
2. After boot, enter a **damping / low-gain** state first. A built-in motion
   service (sport / locomotion / `ai`) owns `rt/lowcmd` while it runs; release
   it from the app or remote, or pass `--release-motion-mode` to
   `scripts/unitree_setup.py` (section 6), which also shows whether one is
   active.
3. Keep the e-stop within reach, and run `scripts/unitree_estop.py` as a
   software stop (section 6, step 2).
4. hiRobot's `reset()` reads the **measured joint pose** and ramps the gains up
   over `ramp_seconds` (default 2 s) while holding that pose, so nothing jumps.
   Watch this phase closely the first time.

## 4. Configuration

`unitree_g1` (H1-2 / H2 / H1 likewise, with their own joint
table and `msg_family`):

```yaml
body:
  driver: unitree_sdk2
  network_interface: eth0   # your NIC name
  # msg_family: hg          # G1/H1-2/H2 = hg (default); legacy H1 = go
  # domain_id: 0
  # default_kp: 60.0        # fallback when the policy sends no gains
  # default_kd: 1.0
  # ramp_seconds: 2.0
```

The joint order follows the `JointIndex` enum in each model's low-level SDK
example (see the comment at the top of each config). The policy's
`joint_target` must use the same order.

### Head camera (G1)

The G1 config declares a `head` camera with `driver: {type: unitree_videohub}`.
With `driver: unitree_sdk2`, `make_robot` wraps the body in a `RobotAssembly`, and every
observation then carries `images["head"]` (1920x1080 RGB uint8, the D435i colour
stream) and `stamps["head"]` (receive time on `time.monotonic()` and frame number).

The frames come from Unitree's `videohub_pc4` service on PC2, over the same DDS
NIC as the body, so nothing extra runs on the robot. The source only reads, so
it also works in `read_only` and in `sim: shadow`. It needs `opencv-python`
(`pip install -e '.[camera]'`). Measured from a desktop on the robot LAN: about
24 new frames/s, median frame age 21 ms. If the camera gives no frame within 3 s
of `connect()`, a warning is logged and `images` lacks `head` until frames
arrive; the body keeps working.

### Head camera (H1)

The H1 has no videohub service. Its head RealSense D435i is read by Unitree's
teleoperation image server, which runs on the computer the camera is plugged
into and publishes each colour frame as one JPEG over ZeroMQ PUB on port 5555
(`image_server.py` in Unitree's avp_teleoperate / xr_teleoperate). The
`unitree_h1` and `unitree_h1_inspire` configs declare a `head` camera with
`driver: {type: unitree_image_server, address: tcp://192.168.123.162:5555}`
(PC2, 1280x720), where avp_teleoperate runs the server on the H1. The source only subscribes, so it works in `read_only` and in
`sim: shadow`. It needs `opencv-python` and `pyzmq` (`pip install -e '.[camera]'`).

If the camera is plugged into another computer, or that computer has no image
server, run `scripts/camera_image_server.py` there. It needs only Python 3.8+,
`opencv-python` and `pyzmq`, and sends the same messages. A D435i shows up as
several V4L2 nodes; the colour stream is `video-index0` under `/dev/v4l/by-id/`:

```bash
# on the computer with the camera (e.g. a Jetson Orin), from a copy of the script
python3 camera_image_server.py --width 640 --height 480 --fps 30 \
  --device /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i-video-index0
```

Then point the config at it in a local file:

```yaml
defaults: [unitree_h1_inspire]
embodiment:
  cameras:
    - name: head
      width: 640
      height: 480
      driver: {type: unitree_image_server, address: "tcp://<camera computer>:5555"}
```

`header: true` reads the image server's `Unit_Test` messages (a timestamp and
frame id before the JPEG), and `crop: [x, y, width, height]` takes one camera out
of a frame where the server put several side by side. Check that the camera
enumerates as USB 3 (`cat /sys/bus/usb/devices/<port>/speed` prints 5000). On
USB 2 (480) the D435i colour stream manages about 28 frames/s at 640x480 but
only about 10 at 1280x720.

### Microphone and speaker (G1)

The G1 configs declare a `head` microphone (`unitree_voice_mic`) and a `head`
speaker (`unitree_voice_speaker`). Both sit behind the G1's audio board at
192.168.123.161; the Jetson sound card on PC2 is not connected to them (it
records only zeros). With `driver: unitree_sdk2`, `make_robot` attaches both to the
`RobotAssembly`, which opens them on `connect()` and closes them on
`disconnect()`. Audio is not part of `Observation`; use the devices directly:

```python
mic = robot.microphones["head"]
chunk = mic.read(timeout=1.0)        # AudioChunk: 16 kHz mono int16 pcm, stamp
spk = robot.speakers["head"]
played = spk.play(pcm, sample_rate=24000)   # blocks; returns seconds played
spk.stop()                                  # from another thread: cuts play() short
```

- **Microphone**: raw PCM multicast over UDP to `239.168.123.161:5555`, 16 kHz
  mono int16, one 5120-byte packet (160 ms) at a time, as in Unitree's C++
  `g1_audio_client_example.cpp`. The driver only listens, so it works in
  `read_only` and `sim: shadow`, and it needs no SDK. It joins the group on the
  address of `network_interface`. Up to 10 s of audio is queued for a slow
  reader; older chunks are then dropped (counted in `dropped`).
- **Speaker**: the SDK `voice` service (`AudioClient.PlayStream` / `PlayStop`).
  The robot buffers what it is sent, so `play()` sends 1 s blocks at most 1 s
  ahead of playback and keeps its own clock; `stop()` calls `PlayStop`, which
  flushes the buffer at once. Measured: 6 s queued, a stop 2 s in cut the sound
  at about 2 s. Input at another rate (TTS is often 22.05 or 24 kHz) is
  resampled to 16 kHz. `volume: 60` is set on `connect()` and the robot's
  previous volume restored on `disconnect()`; the factory volume was 100, and
  30 was too quiet next to the robot. The speaker sends sound, so it is
  attached in `read_only` too; leave it out of the config if that matters.

If the voice service does not answer, `connect()` logs a warning and `play()`
raises; the body keeps working. Likewise, a microphone without packets within
2 s of `connect()` only logs a warning. One G1 we tested sent no microphone
packets at all, not even to PC2, while another streamed at once: if `mic.chunks`
stays at 0, check the robot, not the host.

Check from the host without the body driver:

```bash
python - <<'EOF'
import time
from hirobot.drivers.audio import UnitreeG1Microphone
mic = UnitreeG1Microphone("head", network_interface="eth0")
mic.open(); time.sleep(3); print(mic.chunks, "chunks"); mic.close()   # ~19 in 3 s
EOF
```

### Dex3 hands (G1)

`unitree_g1_dex3` is the G1 with two Dex3-1 three-finger
hands: 43 joints, the 29 body joints followed by 7 per hand in Dex3 motor order
(thumb 0-2, middle 0-1, index 0-1). Joint names and limits come from unitree_ros
`g1_29dof_with_hand_rev_1_0.urdf`. The two hands are mirrored: left fingers
close towards negative angles, right fingers towards positive. All joints at 0
is the fully open hand.

Each hand is an end-effector that lists its joints and a hand driver:

```yaml
end_effectors:
  - name: left_hand
    joints: [left_hand_thumb_0, ..., left_hand_index_1]
    driver: {type: unitree_handcmd, side: left}
```

With `driver: unitree_sdk2`, `make_robot` gives the body driver only the 29 body
joints and builds one `unitree_handcmd` driver per hand. The `RobotAssembly`
splits each 43-joint action by joint name: `rt/lowcmd` carries the body, and
each hand gets its 7 joints on `rt/dex3/<side>/cmd`, clipped to the joint's
`position_limit`. Hands accept position targets only; a `TORQUE` action is
rejected. Gains default to kp 1.5, kd 0.2 (the values Unitree's xr_teleoperate
uses). `disconnect()` damps both hands: zero gains with the timeout flag, as in
the SDK example. `read_only: true` on the body makes the hands read-only too, so
`sim: shadow` publishes nothing on any topic.

The observation's `joint_pos` / `joint_vel` cover all 43 joints. Hand torque is
NaN: the `tau_est` the hand reports is not in Nm (the values are multiples of
about 1674.6). The raw pressure-sensor arrays are in
`extra["<hand>_pressure"]`, shape `(9, 12)`.

`unitree_g1_dex3_mujoco` runs the same 43 joints in MuJoCo
with `g1_29dof_with_hand_rev_1_0.xml` (from the same unitree_ros checkout as
section 5). There the fingers are model joints and no hand driver runs. The
finger joints have no rotor inertia in the MJCF; the config adds 0.001 (an
estimate) to keep the light finger links stable at kp 1.5.

`scripts/dex3_setup.py` checks the hands without ever commanding the body:

```bash
# Read-only: state rate, joint positions in their range, motor temperature, pressure
python scripts/dex3_setup.py status --config unitree_g1_dex3 --iface eth0
# Commands one hand (keep the fingers clear): ramp kp from 0, move on a
# minimum-jerk path, hold, move back, damp
python scripts/dex3_setup.py wiggle --config unitree_g1_dex3 --iface eth0 \
    --hand left --pose open          # or --pose mid --fraction 0.3
```

`wiggle` aborts to damping if a joint lags its path by more than
`--max-track-error` (default 0.4 rad) or the state goes stale. `--pose open`
sends every joint to 0, kept 0.05 rad inside the range where 0 is a hard stop.
Measured on the left hand: the full-open move (up to 1.53 rad of travel in 3 s)
tracked within 0.16 rad, and after the move back the fingers stopped up to about
0.1 rad short of the start, because kp 1.5 does not push the last part of the
way.

### Inspire hands (H1)

`unitree_h1_inspire` is the 19-DoF H1 plus two Inspire
RH56DFX hands (31 joints). The hands are not on the robot's DDS bus directly.
Both sit on one RS485 bus (right hand = Inspire ID 1, left = ID 2) behind a
USB-serial adapter. A bridge program on the computer the adapter is plugged
into turns that bus into DDS topics. Two bridges exist, with different topics
and message types, so each has its own hand driver: `unitree_motorcmds` for
Unitree's service (the config's default) and `inspire_sdkpy` for the
inspire_sdkpy bridge. Set it as the `type:` of each hand's `driver:`.

#### Versions this was tested with

Checked on an H1 on 2026-11-07. Other versions may work, but the commands
below were run with these.

| Item | Version |
|---|---|
| Robot | Unitree H1 (the original 19-DoF H1, not H1-2), `unitree_go` message family |
| Hands | Inspire RH56DFX, 6 DoF each, RS485 at 115200 baud, right ID 1, left ID 2 |
| USB-serial adapter | QinHeng CH340 (`1a86:7523`), `/dev/ttyUSB0` on PC2 |
| Bridge computer | H1 PC2, `192.168.123.162`: x86_64, Ubuntu 22.04.4 LTS, kernel 6.5.2-rt8, user `unitree`, DDS NIC `eth0` |
| Bridge A (Unitree) | `h1_inspire_service` **1.0.1** (C++), built against the `unitree_sdk2` in `/opt/unitree_robotics`; spdlog 1.9.2, Boost 1.74, Eigen 3.4.0 |
| Bridge B (inspire_sdkpy) | `inspire_sdkpy` 1.0.0 from [inspire_hand_ws](https://github.com/NaCl-1374/inspire_hand_ws) commit `fc75490` plus the per-hand callback patch below; pymodbus 3.6.9, pyserial 3.5, cyclonedds 0.10.2 (C library 0.10.5), unitree_sdk2py 1.0.1 (`986f39d`), Python 3.11.13 |
| Workstation (hiRobot) | Ubuntu 24.04.2, Python 3.11.18, cyclonedds 0.10.2, unitree_sdk2py 1.0.1 (`a035ade`), a NIC on `192.168.123.0/24` |

hiRobot needs only `unitree_sdk2py` and its `cyclonedds`. The driver declares
the inspire_sdkpy message types itself, so inspire_sdkpy is only needed on the
bridge computer.

On the H1 tested, the hands' adapter is on PC2. The two Orin NX boards at
`.166` / `.167` have no USB devices attached.

Run one bridge at a time; both open `/dev/ttyUSB0`.

#### Bridge A: Unitree h1_inspire_service

One topic pair for both hands: command `unitree_go.msg.dds_.MotorCmds_` on
`rt/inspire/cmd`, state `unitree_go.msg.dds_.MotorStates_` on
`rt/inspire/state`. Each message holds 12 motors: 0-5 the right hand, 6-11 the
left. Only `q` is used. Config: `type: unitree_motorcmds` with the topics,
`num_motors: 12` and `motor_offset: 6` (left) or `0` (right), as in
`unitree_h1_inspire.yaml`. The same driver serves any hand behind a Unitree
serial-to-DDS service (Dex1-1, BrainCo, Linker) with its own topics.

Use version 1.0.1. It writes to the hands only while commands keep arriving
and stops 1 s after the last one. Version 1.0.2 (Unitree's published
[dfx_inspire_service](https://github.com/unitreerobotics/dfx_inspire_service);
its CHANGELOG says "No longer detecting upper level disconnections, still
continuously sending commands") writes its command buffer from the first
cycle. That buffer starts at `q = 0`, so starting 1.0.2 with no command
publisher running closes every finger fully. A prebuilt 1.0.2 found on one
PC2 also failed to start with `libddsc.so.0: cannot open shared object file`.

Get the 1.0.1 source of `h1_inspire_service` onto PC2 (from Unitree) and build
it out of tree so the source directory is left alone:

```bash
# on PC2
SRC=<path to the h1_inspire_service 1.0.1 source>
mkdir -p ~/h1_inspire_build && cd ~/h1_inspire_build
export LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/unitree_robotics/lib/x86_64:/usr/local/lib
cmake $SRC -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD_LIBRARIES=-lfmt \
  -DCMAKE_CXX_FLAGS="-I/opt/unitree_robotics/include -I/opt/unitree_robotics/include/ddscxx -I/opt/unitree_robotics/include/iceoryx/v2.0.2"
make -j4 inspire_hand
LD_LIBRARY_PATH=/opt/unitree_robotics/lib/x86_64:/usr/local/lib \
  ./inspire_hand -s /dev/ttyUSB0 --network eth0
```

`-lfmt` is needed because Ubuntu's spdlog 1.9.2 uses an external fmt. This
1.0.1 copy defaults `-s` to `/dev/h1_newhand`, a udev name PC2 does not have,
so always pass `-s /dev/ttyUSB0`. The user `unitree` can open the port without
sudo. State arrives at about 96 Hz.

#### Bridge B: inspire_sdkpy

One topic pair per hand: command `inspire.inspire_hand_ctrl` on
`rt/inspire_hand/ctrl/l` and `/r`, state `inspire.inspire_hand_state` on
`rt/inspire_hand/state/l` and `/r`. The driver writes `angle_set` with
`mode = 1` (angle mode) and reads `angle_act`. Config:
`driver: {type: inspire_sdkpy, side: left}`.

Run hiRobot's launcher, `scripts/inspire_serial_to_dds.py`, on PC2. The
stock `Headless_driver_485_double.py` reads only `pos_act` (actuator position,
about 0-2000) and leaves `angle_act` at 0; the driver warns when it sees that.
The launcher reads `pos_act` and `angle_act`, and the other state fields only
with `--all-states`. State arrives at about 29 Hz per hand.

Upstream `fc75490` subscribes both ctrl topics to one callback that writes
every message to both hand IDs, so a command on `/l` also moves the right
hand. Patch `inspire_hand_sdk/inspire_sdkpy/inspire_sdk_double.py` in your
inspire_hand_ws checkout to two callbacks, `write_registers_callback_l` / `_r`, each writing
only its own ID. The launcher refuses to start without that patch.

```bash
# workstation: copy the launcher over
scp scripts/inspire_serial_to_dds.py unitree@192.168.123.162:~/
# on PC2
export CYCLONEDDS_HOME=<cyclonedds install prefix>
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$CYCLONEDDS_HOME/lib
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
python -u ~/inspire_serial_to_dds.py   # the Python that has inspire_sdkpy \
    --serial /dev/ttyUSB0 --left-id 2 --right-id 1
```

Starting the launcher moves nothing. It clears the hands' error flags and then
only reads. Fingers move when a ctrl message arrives. A copy started in the
background from a script (`nohup ... &`) ignores SIGINT, so stop it with
`kill <pid>` (SIGTERM).

#### Units, ordering and behaviour (both bridges)

Both bridges drive the same Inspire registers (angle set 1486, angle actual
1546, 0-1000), so the driver's units do not depend on the bridge. Per hand the
joint order is pinky, ring, middle, index, thumb bend, thumb rotation. Each
finger is normalised to [0, 1]: 0 = closed, 1 = open (register / 1000). The
driver clips commands to [0, 1].

With bridge A, one message sets both hands, so the two hand drivers share one
publisher and the 12 current targets. Commanding one hand rewrites its six
slots and resends the other hand's last target. If the other hand has not been
commanded yet, its slots get its measured pose, so it does not move. If the
state does not carry all 12 motors, `command()` raises rather than guess. With
bridge B each hand has its own topic, so commanding one never touches the
other.

Neither bridge has a passive mode. A hand holds its last target when commands
stop, so `damp()` sends nothing. State carries only positions (velocity 0,
torque NaN). Extras: bridge A gives `extra["lost"]`, the failed serial reads of
that hand since the service started (a count that keeps growing means the hand
is not answering). Bridge B gives `extra["err"]` (per-finger error codes) and
`extra["pos_act"]`.

If `connect()` raises `no state on <topic>`, the bridge is not running, cannot
open its serial port, or is on a different DDS NIC or domain.

#### Example: open both hands and return

`scripts/inspire_hand_demo.py` reads the current pose, ramps both hands to
fully open over 3 s, holds 1 s, ramps back over 3 s and prints how far each
hand ended from where it started. Only the hands move. Ctrl-C during the
motion sends the starting pose back.

```bash
python scripts/inspire_hand_demo.py --iface eth0 --bridge inspire_sdkpy   # bridge B running
python scripts/inspire_hand_demo.py --iface eth0 --bridge unitree         # bridge A running
```

Measured on an H1 on 2026-11-07, both bridges: fully open read 0.98-1.00
on every finger (thumb rotation stops at about 0.98), and both hands ended
within 0.004 of their starting pose.

The legacy H1 leaves motor slot 9 unused. With `msg_family: go` and 19 joints,
the body driver maps joints to slots `0-8, 10-19`. Set `motor_index` under
`body:` to override this.

### Robot skills (G1)

Instead of streaming joint targets, hiRobot can start the G1's built-in
motions by name (`list_robot_skills` / `run_robot_skill` / `stop_robot_skill`,
see the [API reference](../api.md#robot-skills)). Enable them on the body:

```yaml
body:
  driver: unitree_sdk2
  network_interface: eth0
  builtin_skills: g1
```

The built-in motion service (`ai`) then owns `rt/lowcmd`, so the driver creates
no `rt/lowcmd` publisher: `send_action` raises `SafetyError` and `reset()` does
nothing. `builtin_skills` cannot be combined with `release_motion_mode` or
`read_only`, and `sim: shadow` drops it. The `ai` service must be active; after
a `--release-motion-mode` run, bring it back with `unitree_setup.py restore`
(section 6, step 5). Otherwise every skill fails with RPC code 3102 and an error
that says so.

The list is fixed in `src/hirobot/drivers/body/unitree/sdk2/g1_skills.py`, taken
from unitree_sdk2_python (a035ade, 2025-11-28). `list_robot_skills()` returns it
without a robot connected; hiRobot does not ask the robot.

| Skill | Arguments | SDK call |
|---|---|---|
| `start_locomotion` | | `LocoClient.SetFsmId(200)`: enter the balance controller |
| `walk` | `vx` (±0.5 m/s), `vy` (±0.3 m/s), `vyaw` (±0.8 rad/s), `seconds` (0 to 10, default 1) | `LocoClient.SetVelocity` |
| `high_stand` / `low_stand` | | `LocoClient.SetStandHeight` |
| `sit` | | `SetFsmId(3)` |
| `squat_stand` | | `SetFsmId(706)`: squat down, or stand up from a squat |
| `lie_stand_up` | | `SetFsmId(702)`: stand up from lying down |
| `damp` | | `SetFsmId(1)`: limp with damping; the robot falls unless supported |
| `shake_hand`, `high_five`, `hug`, `high_wave`, `face_wave`, `clap`, `hands_up`, `right_hand_up`, `heart`, `right_heart`, `two_hand_kiss`, `left_kiss`, `right_kiss`, `reject`, `x_ray` | | `G1ArmActionClient.ExecuteAction(id)`, ids from the SDK's `action_map` |

`stop_robot_skill()` sends `StopMove` and then `release arm` (action 99), and
tries both even if one fails. The walking limits are deliberately below what
the controller accepts.

Left out on purpose: `ZeroTorque` (FSM 0, the robot drops with no damping),
`SetBalanceMode`, the `LocoClient` wave and shake-hand task ids (the gestures
cover them), and dances (the SDK has no call for them). The robot's
`GetActionList` reports more gestures than the SDK's 15; those are not
included. hiRobot does not check a gesture's own preconditions either, so a
gesture the controller cannot run in its current state shows up as a refusal.

None of these skills has been run on a robot yet. Try them with the robot hung
first.

## 5. MuJoCo (G1, optional)

Driving the real robot needs no MuJoCo. With it, the G1 runs in simulation on
its own (no robot at all), or as a twin next to the real one. Install MuJoCo and
fetch the G1 model from [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
(not shipped with hiRobot):

```bash
pip install -e '.[sim]'          # mujoco
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/unitreerobotics/unitree_ros.git ~/.cache/hirobot/unitree_ros
git -C ~/.cache/hirobot/unitree_ros sparse-checkout set robots/g1_description
```

### Simulated G1 (no robot)

| Config | Base | Use |
|---|---|---|
| `unitree_g1_mujoco` | pelvis welded at 1.0 m, like the robot on its gantry | joint control, arm motions |
| `unitree_g1_mujoco_standing` | free, feet on the floor | balance controllers (hiWBC); plain PD falls in about 1 s |
| `unitree_g1_dex3_mujoco` | welded | G1 with both Dex3 hands (43 joints, fingers simulated) |

Each is the real robot's config plus a MuJoCo overlay (`sim/mujoco/unitree_g1`):
`defaults: [robots/unitree_g1, sim/mujoco/unitree_g1]`. The overlay changes only
`body:` (driver, model, gains, armature), so joint names and order, the `head`
camera and the `Robot` interface are the same as on the real G1. Your own G1
config goes into simulation the same way: add the overlay to its `defaults:`.

```bash
hirobot sim unitree_g1_mujoco --viewer     # hold the pose for 3 s, report drift and images
```

The head camera is rendered from its mount (`d435_joint` in unitree_ros's URDF,
nominal D435i field of view, not calibrated) at the real 1920x1080, about 1 ms a
frame on an RTX 4090. Rendering needs OpenGL: with no display set
`MUJOCO_GL=egl` (or `osmesa`); without a working context the robot runs with no
images and logs a warning. The real camera and audio drivers are not started.

`ee_poses` holds `left_hand` / `right_hand`: the wrist yaw links' poses in the
pelvis frame (`frame:` in the G1's `end_effectors`). The simulated G1 does not
take `CARTESIAN` targets.

The simulation has no microphone or speaker of its own. To test audio code,
replace the drivers with stand-ins in your config; a new `type` replaces the
whole `driver:` entry:

```yaml
defaults: [unitree_g1_mujoco]
embodiment:
  microphones: [{name: head, driver: {type: wav_mic, path: ~/clips/hello.wav}}]
  speakers:    [{name: head, driver: {type: wav_speaker, path: /tmp/g1_said.wav}}]
```

`sounddevice_mic` / `sounddevice_speaker` use the computer's own microphone and
speaker instead.

What differs from the real robot: images (no noise, exposure or compression,
an empty scene), dynamics (motor model, friction, latency), IMU noise. The
built-in skills do not exist in simulation.

### Twin next to the real robot

The twin adds two things: rehearse a motion in simulation from the robot's
current pose before running it, and watch the measured state in a 3D viewer
while it runs. It renders no cameras: images come from the real ones. Add a
`sim:` section to your `driver: unitree_sdk2` config:

```yaml
sim:
  mode: "off"        # quote it: a bare off is read as a YAML boolean
  model: ~/.cache/hirobot/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.xml
  viewer: true
  fixed_base: true   # pelvis welded in place, like the robot on its gantry
  base_height: 1.0
  armature: [...]    # copy the list from the built-in sim/mujoco/unitree_g1
```

| `mode` | Real robot | Simulation |
|---|---|---|
| `off` (default) | commanded | not used; MuJoCo is not imported |
| `mirror` | commanded | shows the measured state (kinematic copy, no physics) |
| `shadow` | **read-only**: no `rt/lowcmd` publisher is created | starts from the measured pose and runs every action |

`scripts/unitree_setup.py hold|move --sim <mode>` and
`make_robot(cfg, sim_overrides={"mode": "shadow"})` override the config. In
`shadow` mode the script ignores `--release-motion-mode`, so a running built-in
service is left alone (`make_robot` refuses `release_motion_mode` with
`shadow`). The twin's gains (`default_kp` / `default_kd`) are copied
from the `body:` section unless `sim:` sets its own.

### Model notes

The unitree_ros MJCF has no joint armature (rotor inertia). Without it the light
wrist links oscillate under PD at the 2 ms timestep, so the G1 configs add an
estimated per-joint `armature:` list. The model also has no joint friction, so
the sim tracks slightly better than the hardware (see the comparison in section 6).

## 6. First motion, step by step (G1 elbow)

A procedure for the first commanded motion: lift the right forearm from hanging
(about 1.33 rad) to horizontal (0.0 rad), hold, and lower it again, while every
other joint holds its pose. Each step either reads only or commands the robot;
the commanding steps are marked.

Write a local config (say `my_g1.yaml`, or copy
`examples/unitree_g1_real.yaml`) that builds on the built-in `unitree_g1`
and sets `body.driver: unitree_sdk2`, the NIC, and per-joint gains. These follow
RoboJuDo's G1 stiffness / damping and hold the hung robot without shaking:

```yaml
defaults: [unitree_g1]   # joints, camera and audio from the built-in config
body:
  driver: unitree_sdk2
  network_interface: eth0
  ramp_seconds: 3.0
  damping_kd: 8.0
  #            hip_p hip_r hip_y knee ank_p ank_r   (left, then right)
  default_kp: [100, 100, 100, 150, 40, 40,
               100, 100, 100, 150, 40, 40,
               200, 200, 200,                        # waist yaw / roll / pitch
               40, 40, 40, 40, 20, 20, 20,           # left arm
               40, 40, 40, 40, 20, 20, 20]           # right arm
  default_kd: [5, 5, 5, 5, 2, 2,
               5, 5, 5, 5, 2, 2,
               6, 6, 6,
               2, 2, 2, 2, 2, 2, 2,
               2, 2, 2, 2, 2, 2, 2]
```

**Step 1: read the state (read-only).**

```bash
python scripts/unitree_setup.py status --config my_g1.yaml --iface eth0
```

Check that `lowstate` arrives (about 1 kHz on a G1), the IMU roll / pitch
match how the robot hangs, every joint reads a plausible angle (`right_elbow` about 1.33
with the arm hanging), and motor temperatures are well below 60 °C. The last
line shows whether a built-in motion service owns `rt/lowcmd`: `'name': ''`
means none does; `'ai'` or similar means it must be released (step 4).

**Step 2: arm the software e-stop (read-only until triggered).** In a second
terminal:

```bash
python scripts/unitree_estop.py --iface eth0
```

It only listens until Ctrl-C. Then it stops `unitree_setup.py`, publishes
damping at 500 Hz, and asks an active built-in service to damp. Start it before
any commanding step. It needs the DDS link to work, so keep the physical e-stop
or remote in hand as well.

**Step 3: rehearse in the twin (read-only).**

```bash
python scripts/unitree_setup.py move --config my_g1.yaml --iface eth0 \
    --joint right_elbow --target 0.0 --ramp 3 --sim shadow
```

The sim starts from the measured pose and runs the full move: a 3 s gain ramp,
4 s out on a minimum-jerk trajectory, 2 s hold, 4 s back, 1 s settle. The
printed table and the final line (`Sim twin left in damping mode; the real
robot was never commanded.`) should show tracking within a few hundredths of a
radian and no other joint drifting. If it aborts here, fix the target or gains
before going near the hardware.

**Step 4: run it on the robot (commands the robot).** The robot is hung with
its feet off the ground, the area around the right arm is clear, and someone
holds the e-stop.

```bash
python scripts/unitree_setup.py move --config my_g1.yaml --iface eth0 \
    --joint right_elbow --target 0.0 --ramp 3 --release-motion-mode --sim mirror \
    --log elbow.csv
```

`--release-motion-mode` releases the built-in service first (omit it if step 1
showed none). `--sim mirror` is optional: it shows the measured pose in the
viewer while the real robot moves; `--sim off` (or no `sim:` section) runs
without MuJoCo. The script aborts to damping if the elbow lags its trajectory
by more than `--max-track-error` (0.3 rad) or any other joint drifts more than
`--max-error` (0.3 rad), and always ends in damping mode.

Results from one G1 run, real robot versus the step 3 rehearsal:

| | Real robot | Twin (`shadow`) |
|---|---|---|
| Steady state at target 0.0 | 0.048 rad | 0.046 rad |
| Elbow torque while holding | −1.69 to −1.75 Nm | −1.82 Nm |
| Worst tracking error | 0.083 rad | 0.064 rad |
| Peak elbow torque | 2.19 Nm | 1.85 Nm |
| Worst drift of other joints | 0.020 rad | 0.043 rad |
| Back at start (1.328 commanded) | 1.310 rad | 1.314 rad |

The steady offset is gravity against kp = 40: about 1.75 Nm / 40 ≈ 0.044 rad.
Static values agree closely. While moving, the real elbow lags about 0.02 rad
more, which fits the joint friction the model lacks.

**Step 5: hand control back (commands the robot).** After a run with
`--release-motion-mode`, nothing owns `rt/lowcmd` and the robot stays limp.
Give it back to the built-in service:

```bash
python scripts/unitree_setup.py restore --config my_g1.yaml --iface eth0
```

It selects the `ai` service through the motion switcher, waits until it
reports active, and asks it to enter its damping state. Or re-enter the
service from the remote or app.

Other joints follow the same procedure; `--max-travel` (1.6 rad) refuses
targets far from the measured pose. `hold` runs steps 3 and 4 without a move,
holding the measured pose for `--seconds` (useful to check the gains first).

### Legacy H1 (19 DoF)

The same procedure ran on a hung legacy H1, body only. What differs from the G1:

- **Messages:** `msg_family: go`. `rt/lowstate` arrives at about 500 Hz. Pass
  `--msg-family go` to the e-stop: `python scripts/unitree_estop.py --iface eth0 --msg-family go`.
- **Gains:** legs and torso follow RoboJuDo's H1 stiffness / damping. The arms use the
  SDK example's weak-motor gains (kp 60, kd 1.5), softer than RoboJuDo's 100, for a
  first run. Order is the SDK slot order of `unitree_h1`:

  ```yaml
  body:
    driver: unitree_sdk2
    network_interface: eth0
    msg_family: go
    ramp_seconds: 3.0
    damping_kd: 8.0
    #            r_hip_roll r_hip_pitch r_knee l_hip_roll l_hip_pitch l_knee waist l_hip_yaw r_hip_yaw
    default_kp: [200, 200, 300, 200, 200, 300, 300, 200, 200,
                 40, 40,                  # left_ankle, right_ankle
                 60, 60, 60, 60,          # right arm (shoulder p / r / y, elbow)
                 60, 60, 60, 60]          # left arm
    default_kd: [5, 5, 6, 5, 5, 6, 6, 5, 5,
                 2, 2,
                 1.5, 1.5, 1.5, 1.5,
                 1.5, 1.5, 1.5, 1.5]
  ```

- **Twin model:** fetch `robots/h1_description` from unitree_ros the same way as
  in section 5. Its `mjcf/h1.xml` needs edits in a local copy before the twin
  loads it. The model names the waist `torso_joint`, but the config calls it
  `waist_yaw`. Its `home` keyframe also no longer fits once `fixed_base` removes
  the free joint. Set `meshdir` to the absolute `meshes/` path, since the copy
  lives elsewhere:

  ```bash
  M=~/.cache/hirobot/unitree_ros/robots/h1_description
  sed -e "s|meshdir=\"../meshes\"|meshdir=\"$M/meshes\"|" \
      -e 's/"torso_joint"/"waist_yaw_joint"/g' \
      -e '/<keyframe>/,/<\/keyframe>/d' $M/mjcf/h1.xml > h1_sim.xml
  ```

  Then point `sim.model` at `h1_sim.xml`, with `fixed_base: true` and `base_height: 1.2`.
- **Arm pose:** `right_elbow` reads about 1.52 rad with the arm hanging. Smaller
  angles raise the forearm, and 0.0 is horizontal forward. The test moved it to
  0.8 rad, about 40° up from hanging.
- **Handing control back:** `restore` has not been run on the H1. Re-enter the
  built-in service from the remote or app.

Results of `move --joint right_elbow --target 0.8` on the H1, real robot versus
the `shadow` rehearsal:

| | Real robot | Twin (`shadow`) |
|---|---|---|
| Steady state at target 0.8 | 0.835 rad | 0.857 rad |
| Worst tracking error | 0.043 rad | 0.172 rad |
| Peak elbow torque | 2.35 Nm | 9.89 Nm |
| Worst drift of other joints | 0.017 rad | 0.224 rad |
| Back at start (1.523 commanded) | 1.519 rad | 1.508 rad |

A `hold` of 5 s beforehand tracked within 0.001 rad with all 19 motors enabled.
Unlike the G1, the twin tracks *worse* than the hardware: the unitree_ros H1 model
was used without tuned armature or friction, so take its numbers as a
conservative rehearsal, not a prediction.

## 7. A control loop

Once a single-joint move works, the same driver runs any control loop that
uses the `Robot` interface. Keep the robot hung for the first runs, and
rehearse new motions with `sim: {mode: shadow}` first. The basic loop is in the
[Quick start](../../README.md#simulation); on the real robot, pace it at
`robot.embodiment.control_hz`.

What the driver does: `connect()` initializes DDS, creates the publisher and
subscriber, and waits for the first `LowState`. `get_observation()` reads
`motor_state[i].q/dq/tau_est` and the IMU (quaternion / gyroscope /
accelerometer). `send_action()` fills `q/dq/tau/kp/kd` of `motor_cmd[i]` for the
action's mode, computes the CRC, and publishes.

## 8. Common pitfalls

- **CRC is mandatory:** every `LowCmd` must be passed through `crc.Crc(cmd)`
  before publishing, or the firmware silently drops it. The driver handles this.
- **Echo `mode_machine`:** with `hg`, every command must carry the
  `mode_machine` value read from `LowState`; a wrong value is rejected. The
  driver handles this.
- **High-level mode owns the bus:** while a built-in motion service (sport /
  locomotion / `ai`) is active it owns `rt/lowcmd` and your commands have no
  effect. `unitree_setup.py status` shows it; `--release-motion-mode`
  releases it and `restore` brings it back. To use the built-in motions
  instead of joint commands, see [Robot skills (G1)](#robot-skills-g1).
- **Robot skill fails with RPC code 3102:** no built-in service answered,
  usually because it was released. Run `unitree_setup.py restore`.
- **No LowState:** `connect()` times out after about 5 s. Check the NIC name,
  the subnet, and that the robot is powered on.
- **Joint count / order:** G1 = 29, H1-2 = 27, H2 = 31, H1 = 19. Using the wrong
  config gives a dimension mismatch.
- **Parallel ankles (`mode_pr` on hg):** on some models the ankle runs in PR or
  AB mode, PR by default. AB (parallel) control requires extending the driver
  to set `mode_pr`.
- **Legacy H1 is in the go family:** set `msg_family: go`, or the message types
  will not match.
- **Head camera error 3102:** `RPC_ERR_CLIENT_SEND` means no subscriber took the
  request, so the `videohub_pc4` service on PC2 (192.168.123.164) is not
  running; the network and the camera itself may be fine. The D435i can be
  opened by one process at a time, so check whether another camera service on
  PC2 holds it (`ps -ef | grep -i camera`, `lsusb`).
