# Unitree 设置指南（G1 / H1-2 / H2 / H1）

[English](./unitree.md) | 简体中文

驱动：`unitree_sdk2`（`src/hirobot/drivers/body/unitree/sdk2/`）。通过 SDK2 的 DDS
接口通信：发布 `rt/lowcmd`，订阅 `rt/lowstate`。

- **G1 / H1-2 / H2** 使用 `unitree_hg` 消息族（`msg_family: hg`，默认值）。
- **旧版 H1（19 自由度）** 使用 `unitree_go` 消息族（`msg_family: go`）。
- **G1 的机器人 skill**（行走、坐下、手臂手势，由内置控制器执行）
  可以替代关节控制：`builtin_skills: g1`，见
  [机器人 skill（G1）](#机器人-skillg1)。

## 1. 前置条件

安装 Python SDK（[unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)）：

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

它依赖 `cyclonedds`。没有 SDK 也能构造驱动并在仿真中运行；只有连接硬件的
`connect()` 才需要它。

## 2. 网络

- 用网线把机器人连到主机，并把主机网卡设到机器人所在的子网
  （Unitree 机器人默认是 `192.168.123.x`）。
- 记下网卡名（例如 `eth0` / `enp3s0`），在配置里设为
  `body.network_interface`。它就是
  `ChannelFactoryInitialize(0, <iface>)` 的第二个参数。不设置时由 SDK 自动检测。

## 3. 上电与安全

1. 机器人处于**吊起或有支撑**的状态，周围区域已清空。
2. 开机后先进入**阻尼 / 低增益**状态。内置运动服务（sport / locomotion / `ai`）
   运行期间占用 `rt/lowcmd`；在 app 或遥控器上释放它，或给
   `scripts/unitree_setup.py` 传 `--release-motion-mode`（第 6 节），该脚本也会显示
   是否有内置服务处于活动状态。
3. 让急停始终触手可及，并运行 `scripts/unitree_estop.py` 作为
   软件急停（第 6 节，第 2 步）。
4. hiRobot 的 `reset()` 读取**实测关节姿态**，在保持该姿态的同时用
   `ramp_seconds`（默认 2 s）把增益斜坡升上去，所以不会有任何跳动。
   第一次运行时要密切观察这一阶段。

## 4. 配置

`unitree_g1`（H1-2 / H2 / H1 同理，各有自己的关节
表和 `msg_family`）：

```yaml
body:
  driver: unitree_sdk2
  network_interface: eth0   # 你的网卡名
  # msg_family: hg          # G1/H1-2/H2 = hg（默认）；旧版 H1 = go
  # domain_id: 0
  # default_kp: 60.0        # policy 不发送增益时的默认值
  # default_kd: 1.0
  # ramp_seconds: 2.0
```

关节顺序遵循各型号底层 SDK 示例中的 `JointIndex` 枚举（见每个配置文件
顶部的注释）。policy 的 `joint_target` 必须使用同样的顺序。

### 头部相机（G1）

G1 配置声明了一个 `head` 相机，`driver: {type: unitree_videohub}`。
使用 `driver: unitree_sdk2` 时，`make_robot` 把本体包进一个 `RobotAssembly`，此后每个
观测都带有 `images["head"]`（1920x1080 RGB uint8，D435i 的彩色流）
和 `stamps["head"]`（接收时间，`time.monotonic()`，以及帧序号）。

画面来自 PC2 上 Unitree 的 `videohub_pc4` 服务，走和本体相同的 DDS
网卡，所以机器人上不需要额外运行任何程序。这个图像源只读取，因此
在 `read_only` 下也能用。它需要 `opencv-python`
（`pip install -e '.[camera]'`）。在机器人局域网内的一台台式机上实测：每秒约
24 个新帧，帧龄中位数 21 ms。如果 `connect()` 后 3 s 内相机没有给出任何帧，
会记录一条警告，`images` 中不含 `head`，直到有帧到达；本体照常工作。

### 头部相机（H1）

H1 没有 videohub 服务。它头部的 RealSense D435i 由 Unitree 的
遥操作图像服务器读取，该服务器运行在相机所插的那台计算机上，
把每个彩色帧作为一张 JPEG 通过 ZeroMQ PUB 发布到 5555 端口
（Unitree avp_teleoperate / xr_teleoperate 中的 `image_server.py`）。
`unitree_h1` 和 `unitree_h1_inspire` 配置声明了一个 `head` 相机，
`driver: {type: unitree_image_server, address: tcp://192.168.123.162:5555}`
（PC2，1280x720），H1 上由 avp_teleoperate 在这里运行该服务器。这个图像源只订阅，所以在 `read_only` 下也能用。它需要 `opencv-python` 和 `pyzmq`（`pip install -e '.[camera]'`）。

如果相机插在另一台计算机上，或者那台计算机上没有图像
服务器，就在那里运行 `scripts/camera_image_server.py`。它只需要 Python 3.8+、
`opencv-python` 和 `pyzmq`，发送的消息与上面相同。一台 D435i 会出现为
多个 V4L2 节点；彩色流是 `/dev/v4l/by-id/` 下的 `video-index0`：

```bash
# 在接相机的计算机上（例如 Jetson Orin），使用脚本的一份拷贝
python3 camera_image_server.py --width 640 --height 480 --fps 30 \
  --device /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i-video-index0
```

然后在本地配置文件里把配置指向它：

```yaml
defaults: [unitree_h1_inspire]
embodiment:
  cameras:
    - name: head
      width: 640
      height: 480
      driver: {type: unitree_image_server, address: "tcp://<camera computer>:5555"}
```

`header: true` 读取图像服务器的 `Unit_Test` 消息（JPEG 前面带有时间戳和
帧 id），`crop: [x, y, width, height]` 从服务器把多个相机并排拼在一起的帧中
取出其中一个相机。检查相机是否以 USB 3 枚举（`cat /sys/bus/usb/devices/<port>/speed` 输出 5000）。在
USB 2（480）上，D435i 彩色流在 640x480 下约 28 帧/s，但在 1280x720 下
只有约 10 帧/s。

### 麦克风与扬声器（G1）

G1 配置声明了一个 `head` 麦克风（`unitree_voice_mic`）和一个 `head`
扬声器（`unitree_voice_speaker`）。两者都在 G1 位于
192.168.123.161 的音频板后面；PC2 上的 Jetson 声卡没有接到它们（它
只录到全零）。使用 `driver: unitree_sdk2` 时，`make_robot` 把两者都挂到
`RobotAssembly` 上，后者在 `connect()` 时打开它们，在
`disconnect()` 时关闭。音频不属于 `Observation`；请直接使用这些设备：

```python
mic = robot.microphones["head"]
chunk = mic.read(timeout=1.0)        # AudioChunk：16 kHz 单声道 int16 pcm，带时间戳
spk = robot.speakers["head"]
played = spk.play(pcm, sample_rate=24000)   # 阻塞；返回播放的秒数
spk.stop()                                  # 从另一个线程调用：提前结束 play()
```

- **麦克风**：原始 PCM 通过 UDP 组播到 `239.168.123.161:5555`，16 kHz
  单声道 int16，每次一个 5120 字节的包（160 ms），与 Unitree 的 C++
  `g1_audio_client_example.cpp` 相同。驱动只监听，所以在
  `read_only` 下也能用，而且不需要 SDK。它在
  `network_interface` 的地址上加入组播组。最多为读得慢的一方缓存 10 s
  音频；超出后丢弃较旧的块（计入 `dropped`）。
- **扬声器**：SDK 的 `voice` 服务（`AudioClient.PlayStream` / `PlayStop`）。
  机器人会缓冲发给它的内容，所以 `play()` 以 1 s 为一块发送，最多比播放进度
  提前 1 s，并自己计时；`stop()` 调用 `PlayStop`，它会
  立即清空缓冲区。实测：排队 6 s，在第 2 s 时停止，声音在
  约 2 s 处被切断。其他采样率的输入（TTS 常见 22.05 或 24 kHz）会
  重采样到 16 kHz。`connect()` 时设置 `volume: 60`，`disconnect()` 时恢复机器人
  原来的音量；出厂音量是 100，而
  30 在机器人旁边太小。扬声器会发出声音，所以在 `read_only` 下也会
  挂上；如果这一点要紧，就把它从配置中去掉。

如果 voice 服务没有响应，`connect()` 记录一条警告，`play()`
抛出异常；本体照常工作。同样，`connect()` 后 2 s 内没收到包的
麦克风也只记录一条警告。我们测试的一台 G1 完全没有发出麦克风
数据包，连 PC2 也收不到，而另一台立刻就开始传输：如果 `mic.chunks`
一直是 0，请检查机器人，而不是主机。

不用本体驱动，从主机上检查：

```bash
python - <<'EOF'
import time
from hirobot.drivers.audio import UnitreeG1Microphone
mic = UnitreeG1Microphone("head", network_interface="eth0")
mic.open(); time.sleep(3); print(mic.chunks, "chunks"); mic.close()   # 3 s 内约 19 个
EOF
```

### Dex3 灵巧手（G1）

`unitree_g1_dex3` 是带两只 Dex3-1 三指灵巧手的
G1：43 个关节，即 29 个本体关节，后接每只手 7 个、按 Dex3 电机顺序排列的关节
（thumb 0-2、middle 0-1、index 0-1）。关节名和限位来自 unitree_ros 的
`g1_29dof_with_hand_rev_1_0.urdf`。两只手互为镜像：左手手指
朝负角度方向闭合，右手手指朝正角度方向闭合。所有关节为 0
即手完全张开。

每只手是一个末端执行器，列出它的关节和一个手部驱动：

```yaml
end_effectors:
  - name: left_hand
    joints: [left_hand_thumb_0, ..., left_hand_index_1]
    driver: {type: unitree_handcmd, side: left}
```

使用 `driver: unitree_sdk2` 时，`make_robot` 只把 29 个本体
关节交给本体驱动，并为每只手构建一个 `unitree_handcmd` 驱动。`RobotAssembly`
按关节名拆分每个 43 关节的动作：`rt/lowcmd` 承载本体部分，
每只手的 7 个关节通过 `rt/dex3/<side>/cmd` 发送，并被裁剪到该关节的
`position_limit` 内。灵巧手只接受位置目标；`TORQUE` 动作会被
拒绝。增益默认 kp 1.5、kd 0.2（Unitree xr_teleoperate
使用的值）。`disconnect()` 让两只手进入阻尼：零增益并置超时标志，与
SDK 示例相同。本体上设 `read_only: true` 时灵巧手也变为只读，所以
只读的机器人不会在任何话题上发布任何内容。

观测的 `joint_pos` / `joint_vel` 覆盖全部 43 个关节。手部力矩是
NaN：手上报的 `tau_est` 单位不是 Nm（数值是
约 1674.6 的倍数）。原始压力传感器数组在
`extra["<hand>_pressure"]` 中，形状 `(9, 12)`。

`unitree_g1_dex3_mujoco` 在 MuJoCo 中运行同样的 43 个关节，
使用 `g1_29dof_with_hand_rev_1_0.xml`（来自与第 5 节
相同的 unitree_ros checkout）。在仿真中手指是模型关节，不运行手部驱动。MJCF
中的手指关节没有转子惯量；配置加了 0.001（估计值），
让轻质手指连杆在 kp 1.5 下保持稳定。

`scripts/dex3_setup.py` 检查灵巧手，且从不向本体发送命令：

```bash
# 只读：状态频率、关节位置是否在范围内、电机温度、压力
python scripts/dex3_setup.py status --config unitree_g1_dex3 --iface eth0
# 向一只手发送命令（手指周围保持无遮挡）：kp 从 0 斜坡上升，沿
# 最小加加速度路径运动，保持，回位，阻尼
python scripts/dex3_setup.py wiggle --config unitree_g1_dex3 --iface eth0 \
    --hand left --pose open          # 或 --pose mid --fraction 0.3
```

如果某个关节落后其路径超过
`--max-track-error`（默认 0.4 rad），或者状态过期，`wiggle` 会中止并进入阻尼。`--pose open`
把每个关节送到 0；对于 0 恰好是硬限位的关节，目标保持在范围内侧 0.05 rad 处。
在左手上实测：完全张开的运动（3 s 内最多 1.53 rad 行程）
跟踪误差在 0.16 rad 以内；回位之后手指最多停在距起点约
0.1 rad 处，因为 kp 1.5 推不动最后那一段
行程。

### Inspire 灵巧手（H1）

`unitree_h1_inspire` 是 19 自由度的 H1 加两只 Inspire
RH56DFX 灵巧手（31 个关节）。灵巧手并不直接在机器人的 DDS 总线上。
两只手位于同一条 RS485 总线上（右手 = Inspire ID 1，左手 = ID 2），通过一个
USB 转串口适配器连接。适配器所插的计算机上运行一个桥接程序，
把这条总线转换为 DDS 话题。现有两种桥接程序，话题
和消息类型不同，所以各有自己的手部驱动：`unitree_motorcmds` 对应
Unitree 的服务（配置的默认值），`inspire_sdkpy` 对应
inspire_sdkpy 桥接程序。把它设为每只手 `driver:` 的 `type:`。

#### 测试所用的版本

于 2026-11-07 在一台 H1 上检查。其他版本也可能可用，但下面的命令
是用这些版本运行的。

| 项目 | 版本 |
|---|---|
| 机器人 | Unitree H1（初代 19 自由度 H1，不是 H1-2），`unitree_go` 消息族 |
| 灵巧手 | Inspire RH56DFX，每只 6 自由度，RS485 115200 波特，右手 ID 1，左手 ID 2 |
| USB 转串口适配器 | QinHeng CH340（`1a86:7523`），PC2 上的 `/dev/ttyUSB0` |
| 桥接计算机 | H1 PC2，`192.168.123.162`：x86_64，Ubuntu 22.04.4 LTS，内核 6.5.2-rt8，用户 `unitree`，DDS 网卡 `eth0` |
| 桥接程序 A（Unitree） | `h1_inspire_service` **1.0.1**（C++），基于 `/opt/unitree_robotics` 中的 `unitree_sdk2` 构建；spdlog 1.9.2，Boost 1.74，Eigen 3.4.0 |
| 桥接程序 B（inspire_sdkpy） | `inspire_sdkpy` 1.0.0，来自 [inspire_hand_ws](https://github.com/NaCl-1374/inspire_hand_ws) commit `fc75490`，外加下文的逐手回调补丁；pymodbus 3.6.9，pyserial 3.5，cyclonedds 0.10.2（C 库 0.10.5），unitree_sdk2py 1.0.1（`986f39d`），Python 3.11.13 |
| 工作站（hiRobot） | Ubuntu 24.04.2，Python 3.11.18，cyclonedds 0.10.2，unitree_sdk2py 1.0.1（`a035ade`），一块位于 `192.168.123.0/24` 的网卡 |

hiRobot 只需要 `unitree_sdk2py` 及其 `cyclonedds`。驱动自己声明了
inspire_sdkpy 的消息类型，所以只有桥接计算机上才需要 inspire_sdkpy。

在测试的那台 H1 上，灵巧手的适配器接在 PC2 上。位于
`.166` / `.167` 的两块 Orin NX 板上没有接任何 USB 设备。

一次只运行一个桥接程序；两者都会打开 `/dev/ttyUSB0`。

#### 桥接程序 A：Unitree h1_inspire_service

两只手共用一对话题：命令 `unitree_go.msg.dds_.MotorCmds_` 走
`rt/inspire/cmd`，状态 `unitree_go.msg.dds_.MotorStates_` 走
`rt/inspire/state`。每条消息包含 12 个电机：0-5 是右手，6-11 是
左手。只使用 `q`。配置：`type: unitree_motorcmds`，加上这两个话题、
`num_motors: 12` 以及 `motor_offset: 6`（左手）或 `0`（右手），见
`unitree_h1_inspire.yaml`。同一个驱动也适用于任何接在 Unitree
串口转 DDS 服务后面的手（Dex1-1、BrainCo、Linker），使用各自的话题即可。

使用 1.0.1 版。它只在命令持续到达时才写入灵巧手，
并在最后一条命令之后 1 s 停止。1.0.2 版（Unitree 发布的
[dfx_inspire_service](https://github.com/unitreerobotics/dfx_inspire_service)；
其 CHANGELOG 写道 "No longer detecting upper level disconnections, still
continuously sending commands"）从第一个周期起就写入自己的命令缓冲区。
该缓冲区初始为 `q = 0`，所以在没有命令发布者运行时启动 1.0.2，
所有手指都会完全闭合。在一台 PC2 上找到的一个预编译 1.0.2
也无法启动，报错 `libddsc.so.0: cannot open shared object file`。

把 `h1_inspire_service` 的 1.0.1 源码放到 PC2 上（向 Unitree 获取），并在
源码树外构建，以免改动源码目录：

```bash
# 在 PC2 上
SRC=<path to the h1_inspire_service 1.0.1 source>
mkdir -p ~/h1_inspire_build && cd ~/h1_inspire_build
export LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/unitree_robotics/lib/x86_64:/usr/local/lib
cmake $SRC -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD_LIBRARIES=-lfmt \
  -DCMAKE_CXX_FLAGS="-I/opt/unitree_robotics/include -I/opt/unitree_robotics/include/ddscxx -I/opt/unitree_robotics/include/iceoryx/v2.0.2"
make -j4 inspire_hand
LD_LIBRARY_PATH=/opt/unitree_robotics/lib/x86_64:/usr/local/lib \
  ./inspire_hand -s /dev/ttyUSB0 --network eth0
```

需要 `-lfmt`，因为 Ubuntu 的 spdlog 1.9.2 使用外部的 fmt。这份
1.0.1 把 `-s` 默认设为 `/dev/h1_newhand`，这是 PC2 上没有的 udev 名称，
所以务必传 `-s /dev/ttyUSB0`。用户 `unitree` 不用
sudo 就能打开该端口。状态约以 96 Hz 到达。

#### 桥接程序 B：inspire_sdkpy

每只手一对话题：命令 `inspire.inspire_hand_ctrl` 走
`rt/inspire_hand/ctrl/l` 和 `/r`，状态 `inspire.inspire_hand_state` 走
`rt/inspire_hand/state/l` 和 `/r`。驱动以
`mode = 1`（角度模式）写 `angle_set`，并读取 `angle_act`。配置：
`driver: {type: inspire_sdkpy, side: left}`。

在 PC2 上运行 hiRobot 的启动脚本 `scripts/inspire_serial_to_dds.py`。原版的
`Headless_driver_485_double.py` 只读取 `pos_act`（执行器位置，
约 0-2000），`angle_act` 一直为 0；驱动看到这种情况时会发出警告。
启动脚本读取 `pos_act` 和 `angle_act`，其他状态字段只有
加 `--all-states` 时才读取。每只手的状态约以 29 Hz 到达。

上游 `fc75490` 把两个 ctrl 话题都订阅到同一个回调，该回调把
每条消息写到两个手的 ID，所以发到 `/l` 的命令也会让右手
动起来。请在你的 inspire_hand_ws checkout 中修补
`inspire_hand_sdk/inspire_sdkpy/inspire_sdk_double.py`，改成两个回调 `write_registers_callback_l` / `_r`，各自只写
自己的 ID。没有这个补丁，启动脚本会拒绝启动。

```bash
# 工作站：把启动脚本拷过去
scp scripts/inspire_serial_to_dds.py unitree@192.168.123.162:~/
# 在 PC2 上
export CYCLONEDDS_HOME=<cyclonedds install prefix>
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$CYCLONEDDS_HOME/lib
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General></Domain></CycloneDDS>'
python -u ~/inspire_serial_to_dds.py   # 装有 inspire_sdkpy 的 Python \
    --serial /dev/ttyUSB0 --left-id 2 --right-id 1
```

启动脚本本身不会让任何东西动。它清除灵巧手的错误标志，之后
只读取。ctrl 消息到达时手指才会动。从脚本中后台启动的副本
（`nohup ... &`）会忽略 SIGINT，所以要用
`kill <pid>`（SIGTERM）停止它。

#### 单位、顺序与行为（两种桥接程序通用）

两种桥接程序驱动的是同一组 Inspire 寄存器（角度设定 1486，实际角度
1546，0-1000），所以驱动的单位与桥接程序无关。每只手的
关节顺序是小指、无名指、中指、食指、拇指弯曲、拇指旋转。每根
手指归一化到 [0, 1]：0 = 闭合，1 = 张开（寄存器值 / 1000）。驱动
把命令裁剪到 [0, 1]。

使用桥接程序 A 时，一条消息同时设置两只手，所以两个手部驱动共用一个
发布者和 12 个当前目标。给一只手发命令会改写它的六个
槽位，并重发另一只手的上一次目标。如果另一只手还没有收到过
命令，它的槽位填入它的实测姿态，所以它不会动。如果
状态里没有全部 12 个电机，`command()` 会抛出异常，而不是去猜。使用
桥接程序 B 时每只手有自己的话题，所以给一只手发命令永远不会影响
另一只。

两种桥接程序都没有被动模式。命令停止后，手会保持最后一次目标，
所以 `damp()` 什么也不发送。状态只包含位置（速度为 0，
力矩为 NaN）。额外字段：桥接程序 A 提供 `extra["lost"]`，即该服务启动以来
这只手串口读取失败的次数（这个计数不断增长说明这只手
没有应答）。桥接程序 B 提供 `extra["err"]`（每根手指的错误码）和
`extra["pos_act"]`。

如果 `connect()` 抛出 `no state on <topic>`，说明桥接程序没有运行、无法
打开它的串口，或者位于不同的 DDS 网卡或 domain 上。

#### 示例：张开两只手再复原

`scripts/inspire_hand_demo.py` 读取当前姿态，用 3 s 把两只手斜坡过渡到
完全张开，保持 1 s，再用 3 s 斜坡回去，并打印每只
手最终离起始位置多远。只有手会动。在
运动过程中按 Ctrl-C 会把起始姿态发回去。

```bash
python scripts/inspire_hand_demo.py --iface eth0 --bridge inspire_sdkpy   # 桥接程序 B 在运行
python scripts/inspire_hand_demo.py --iface eth0 --bridge unitree         # 桥接程序 A 在运行
```

2026-11-07 在一台 H1 上实测，两种桥接程序都是：完全张开时每根手指读数为 0.98-1.00
（拇指旋转停在约 0.98），两只手最终都
在起始姿态的 0.004 以内。

旧版 H1 不使用电机槽位 9。在 `msg_family: go` 且 19 个关节时，
本体驱动把关节映射到槽位 `0-8, 10-19`。在
`body:` 下设置 `motor_index` 可以覆盖这一映射。

### 机器人 skill（G1）

hiRobot 可以不流式发送关节目标，而是按名字启动 G1 的内置
动作（`list_robot_skills` / `run_robot_skill` / `stop_robot_skill`，
见 [API 参考](../api.zh-CN.md#机器人-skill)）。在本体上启用：

```yaml
body:
  driver: unitree_sdk2
  network_interface: eth0
  builtin_skills: g1
```

此时由内置运动服务（`ai`）占用 `rt/lowcmd`，所以驱动不创建
`rt/lowcmd` 发布者：`send_action` 抛出 `SafetyError`，`reset()` 什么
也不做。`builtin_skills` 不能与 `release_motion_mode` 或
`read_only` 同时使用。`ai` 服务必须处于活动状态；运行过
`--release-motion-mode` 之后，用 `unitree_setup.py restore` 把它恢复
（第 6 节，第 5 步）。否则每个 skill 都会失败，返回 RPC 错误码 3102，并给出
说明这一点的错误信息。

列表固定在 `src/hirobot/drivers/body/unitree/sdk2/g1_skills.py` 中，取自
unitree_sdk2_python（a035ade，2025-11-28）。`list_robot_skills()` 不连接机器人
也能返回它；hiRobot 不向机器人查询。

| Skill | 参数 | SDK 调用 |
|---|---|---|
| `start_locomotion` | | `LocoClient.SetFsmId(200)`：进入平衡控制器 |
| `walk` | `vx`（±0.5 m/s），`vy`（±0.3 m/s），`vyaw`（±0.8 rad/s），`seconds`（0 到 10，默认 1） | `LocoClient.SetVelocity` |
| `high_stand` / `low_stand` | | `LocoClient.SetStandHeight` |
| `sit` | | `SetFsmId(3)` |
| `squat_stand` | | `SetFsmId(706)`：蹲下，或从蹲姿站起 |
| `lie_stand_up` | | `SetFsmId(702)`：从躺姿站起 |
| `damp` | | `SetFsmId(1)`：带阻尼瘫软；没有支撑时机器人会摔倒 |
| `shake_hand`, `high_five`, `hug`, `high_wave`, `face_wave`, `clap`, `hands_up`, `right_hand_up`, `heart`, `right_heart`, `two_hand_kiss`, `left_kiss`, `right_kiss`, `reject`, `x_ray` | | `G1ArmActionClient.ExecuteAction(id)`，id 取自 SDK 的 `action_map` |

`stop_robot_skill()` 先发送 `StopMove`，再发送 `release arm`（动作 99），即使其中一个
失败也会两个都尝试。行走限值有意设得低于
控制器能接受的范围。

有意未包含的：`ZeroTorque`（FSM 0，机器人会在没有阻尼的情况下瘫倒），
`SetBalanceMode`，`LocoClient` 的挥手和握手任务 id（手势已
覆盖它们），以及舞蹈（SDK 没有对应的调用）。机器人的
`GetActionList` 报告的手势比 SDK 的 15 个多；那些没有
收录。hiRobot 也不检查手势自身的前置条件，所以控制器在当前状态下无法执行的
手势会表现为一次拒绝。

这些 skill 都还没有在机器人上运行过。先在机器人吊起的状态下
尝试。

## 5. MuJoCo（G1，可选）

驱动真机不需要 MuJoCo。有了它，G1 可以单独在仿真中运行
（完全不需要机器人），或者从真机姿态起步，在上真机之前预演一个动作。安装 MuJoCo 并
从 [unitree_ros](https://github.com/unitreerobotics/unitree_ros) 获取 G1 模型
（hiRobot 不附带）：

```bash
pip install -e '.[sim]'          # mujoco
git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/unitreerobotics/unitree_ros.git ~/.cache/hirobot/unitree_ros
git -C ~/.cache/hirobot/unitree_ros sparse-checkout set robots/g1_description
```

### 仿真 G1（无机器人）

| 配置 | 基座 | 用途 |
|---|---|---|
| `unitree_g1_mujoco` | 骨盆焊接在 1.0 m 高处，就像机器人吊在龙门架上 | 关节控制、手臂动作 |
| `unitree_g1_mujoco_standing` | 自由，双脚着地 | 平衡控制器（hiWBC）；单纯 PD 约 1 s 就会摔倒 |
| `unitree_g1_dex3_mujoco` | 焊接 | 带两只 Dex3 灵巧手的 G1（43 个关节，手指也仿真） |

每个都是真机配置加一个 MuJoCo 覆盖层（`sim/mujoco/unitree_g1`）：
`defaults: [robots/unitree_g1, sim/mujoco/unitree_g1]`。覆盖层只改动
`body:`（驱动、模型、增益、armature），所以关节名和顺序、`head`
相机以及 `Robot` 接口都与真实 G1 相同。你自己的 G1
配置也用同样的方式放进仿真：把覆盖层加到它的 `defaults:` 里。

```bash
hirobot sim unitree_g1_mujoco --viewer     # 保持姿态 3 s，报告漂移和图像
```

头部相机从它的安装位置渲染（unitree_ros URDF 中的 `d435_joint`，
D435i 标称视场角，未标定），分辨率为真实的 1920x1080，在 RTX 4090 上每帧约 1 ms。
渲染需要 OpenGL：没有显示器时设置
`MUJOCO_GL=egl`（或 `osmesa`）；没有可用的上下文时机器人照常运行，但没有
图像，并记录一条警告。真实的相机和音频驱动不会启动。

`ee_poses` 包含 `left_hand` / `right_hand`：手腕 yaw 连杆在
骨盆坐标系下的位姿（G1 `end_effectors` 中的 `frame:`）。仿真 G1 不
接受 `CARTESIAN` 目标。

仿真没有自己的麦克风和扬声器。要测试音频代码，
在你的配置中把驱动换成替身；新的 `type` 会替换
整个 `driver:` 条目：

```yaml
defaults: [unitree_g1_mujoco]
embodiment:
  microphones: [{name: head, driver: {type: wav_mic, path: ~/clips/hello.wav}}]
  speakers:    [{name: head, driver: {type: wav_speaker, path: /tmp/g1_said.wav}}]
```

`sounddevice_mic` / `sounddevice_speaker` 则使用计算机自己的麦克风和
扬声器。

与真机的差异：图像（没有噪声、曝光或压缩，
场景为空）、动力学（电机模型、摩擦、延迟）、IMU 噪声。
内置 skill 在仿真中不存在。

### 从真机姿态起步预演

在真机上运行一个动作之前，先让它在 MuJoCo 中从真机当前的姿态出发跑一遍。
[`examples/rehearse_in_sim.py`](../../examples/rehearse_in_sim.py) 用两个独立的机器人完成这件事：

1. 用 `read_only: true` 打开真机（不创建 `rt/lowcmd` 发布者），读取一次观测，然后断开。
2. 把 MuJoCo 配置（`unitree_g1_mujoco`）建成第二个机器人，用 MuJoCo 驱动的 `set_state`
   把它放到实测姿态。
3. `reset()` 在保持这个姿态的同时把增益升上去；之后动作只在仿真中运行。

```bash
python examples/rehearse_in_sim.py my_g1.yaml unitree_g1_mujoco \
    --body network_interface=eth0 --joint right_elbow --target 0.0
```

不向真机发送任何内容，正在运行的内置服务也不会被动到。仿真使用 MuJoCo 配置的增益；
要用你自己的增益预演，把它们写进一份 `defaults: [unitree_g1_mujoco]` 的本地配置。
仿真从模型渲染头部相机，不是真实图像。要预演别的动作，把示例中的 `read_real_pose` 和
`sim_from_pose` 拷贝到你自己的代码里。在 MuJoCo 中实时显示真机状态的功能已在计划中
（`hirobot view <config>`）。

### 模型说明

unitree_ros 的 MJCF 没有关节 armature（转子惯量）。没有它，轻质的
手腕连杆在 2 ms 时间步长下会在 PD 控制下振荡，所以 G1 配置加了一个
估计的逐关节 `armature:` 列表。该模型也没有关节摩擦，所以
仿真的跟踪效果比硬件略好（见第 6 节中的对比）。

## 6. 第一次运动，逐步进行（G1 肘关节）

第一次命令运动的流程：把右前臂从下垂
（约 1.33 rad）抬到水平（0.0 rad），保持，再放下，同时其他所有
关节都保持各自的姿态。每一步要么只读，要么向机器人发送命令；
发送命令的步骤已标出。

写一个本地配置（比如 `my_g1.yaml`，或者拷贝
`examples/unitree_g1_real.yaml`），基于内置的 `unitree_g1`，
并设置 `body.driver: unitree_sdk2`、网卡和逐关节增益。这些增益沿用
RoboJuDo 的 G1 刚度 / 阻尼，能让吊着的机器人保持住而不抖动：

```yaml
defaults: [unitree_g1]   # 关节、相机和音频来自内置配置
body:
  driver: unitree_sdk2
  network_interface: eth0
  ramp_seconds: 3.0
  damping_kd: 8.0
  # command_timeout_s: 0.2  # 这么久没有新的 VELOCITY 就发零速（kp 0）；null 关掉
  #            hip_p hip_r hip_y knee ank_p ank_r   （先左后右）
  default_kp: [100, 100, 100, 150, 40, 40,
               100, 100, 100, 150, 40, 40,
               200, 200, 200,                        # waist yaw / roll / pitch
               40, 40, 40, 40, 20, 20, 20,           # 左臂
               40, 40, 40, 40, 20, 20, 20]           # 右臂
  default_kd: [5, 5, 5, 5, 2, 2,
               5, 5, 5, 5, 2, 2,
               6, 6, 6,
               2, 2, 2, 2, 2, 2, 2,
               2, 2, 2, 2, 2, 2, 2]
```

**第 1 步：读取状态（只读）。**

```bash
python scripts/unitree_setup.py status --config my_g1.yaml --iface eth0
```

检查：`lowstate` 有数据到达（G1 上约 1 kHz），IMU 的 roll / pitch
与机器人吊挂的姿态相符，每个关节读数都是合理的角度（手臂下垂时 `right_elbow` 约 1.33），
电机温度远低于 60 °C。最后
一行显示是否有内置运动服务占用 `rt/lowcmd`：`'name': ''`
表示没有；`'ai'` 或类似值表示必须先释放它（第 4 步）。

**第 2 步：启用软件急停（触发前只读）。** 在第二个
终端中：

```bash
python scripts/unitree_estop.py --iface eth0
```

它在按下 Ctrl-C 之前只监听。按下后它会停止 `unitree_setup.py`，以 500 Hz 发布
阻尼命令，并请求处于活动状态的内置服务进入阻尼。在
任何发送命令的步骤之前启动它。它依赖 DDS 链路正常工作，所以物理急停
或遥控器也要拿在手里。

**第 3 步：在仿真中预演（只读）。** 需要先完成第 5 节的 MuJoCo 设置。

```bash
python examples/rehearse_in_sim.py my_g1.yaml unitree_g1_mujoco \
    --body network_interface=eth0 --joint right_elbow --target 0.0
```

真机只被读取。仿真 G1 从实测姿态开始，升起增益，在 3 s（`--seconds`）内把肘关节移到目标。
最后一行打印真机的起始角度和仿真肘关节的终点。终点应接近目标：重力对抗 kp 40 会留下约
0.05 rad 的偏差。如果差得远，先修正目标或增益，再靠近硬件。

**第 4 步：在机器人上运行（向机器人发送命令）。** 机器人吊起，
双脚离地，右臂周围区域已清空，并且有人
握着急停。

```bash
python scripts/unitree_setup.py move --config my_g1.yaml --iface eth0 \
    --joint right_elbow --target 0.0 --ramp 3 --release-motion-mode \
    --log elbow.csv
```

`--release-motion-mode` 会先释放内置服务（如果第 1 步
显示没有内置服务，就省略它）。如果肘关节落后其轨迹
超过 `--max-track-error`（0.3 rad），或任何其他关节漂移超过
`--max-error`（0.3 rad），脚本会中止并进入阻尼，而且无论如何都以阻尼模式结束。

一次 G1 运行的结果，真机对比同一动作在 MuJoCo 中从真机姿态起步的运行：

| | 真机 | 仿真 |
|---|---|---|
| 目标 0.0 处的稳态 | 0.048 rad | 0.046 rad |
| 保持时的肘关节力矩 | −1.69 到 −1.75 Nm | −1.82 Nm |
| 最大跟踪误差 | 0.083 rad | 0.064 rad |
| 肘关节峰值力矩 | 2.19 Nm | 1.85 Nm |
| 其他关节的最大漂移 | 0.020 rad | 0.043 rad |
| 回到起点（命令值 1.328） | 1.310 rad | 1.314 rad |

稳态偏差是重力对抗 kp = 40 的结果：约 1.75 Nm / 40 ≈ 0.044 rad。
静态数值吻合得很好。运动中，真实肘关节多落后约 0.02 rad，
这与模型缺少的关节摩擦相符。

**第 5 步：交还控制权（向机器人发送命令）。** 用
`--release-motion-mode` 运行之后，没有任何服务占用 `rt/lowcmd`，机器人保持瘫软。
把控制权交还给内置服务：

```bash
python scripts/unitree_setup.py restore --config my_g1.yaml --iface eth0
```

它通过 motion switcher 选择 `ai` 服务，等到该服务
报告处于活动状态，再请求它进入阻尼状态。也可以从遥控器或 app 重新进入
该服务。

其他关节按同样的流程进行；`--max-travel`（1.6 rad）会拒绝
离实测姿态太远的目标。`hold` 执行第 4 步但不做运动，
在 `--seconds` 时间内保持实测姿态（适合先检查增益）。

### 旧版 H1（19 自由度）

在一台吊起的旧版 H1 上运行了同样的流程，只涉及本体。与 G1 的不同之处：

- **消息：** `msg_family: go`。`rt/lowstate` 约以 500 Hz 到达。给急停传
  `--msg-family go`：`python scripts/unitree_estop.py --iface eth0 --msg-family go`。
- **增益：** 腿和躯干沿用 RoboJuDo 的 H1 刚度 / 阻尼。手臂使用
  SDK 示例中弱电机的增益（kp 60，kd 1.5），比 RoboJuDo 的 100 更软，用于
  首次运行。顺序是 `unitree_h1` 的 SDK 槽位顺序：

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
                 60, 60, 60, 60,          # 右臂（shoulder p / r / y，elbow）
                 60, 60, 60, 60]          # 左臂
    default_kd: [5, 5, 6, 5, 5, 6, 6, 5, 5,
                 2, 2,
                 1.5, 1.5, 1.5, 1.5,
                 1.5, 1.5, 1.5, 1.5]
  ```

- **仿真模型：** 按第 5 节的方法从 unitree_ros 获取 `robots/h1_description`。
  它的 `mjcf/h1.xml` 需要在本地副本中修改后，MuJoCo 才能
  加载。模型把腰部命名为 `torso_joint`，而配置称之为
  `waist_yaw`。一旦 `fixed_base` 去掉自由关节，它的 `home` 关键帧也
  不再适用。由于副本放在别处，要把 `meshdir` 设为 `meshes/` 的绝对路径：

  ```bash
  M=~/.cache/hirobot/unitree_ros/robots/h1_description
  sed -e "s|meshdir=\"../meshes\"|meshdir=\"$M/meshes\"|" \
      -e 's/"torso_joint"/"waist_yaw_joint"/g' \
      -e '/<keyframe>/,/<\/keyframe>/d' $M/mjcf/h1.xml > h1_sim.xml
  ```

  然后写一份本地仿真配置，比如 `my_h1_mujoco.yaml`，用
  `python examples/rehearse_in_sim.py my_h1.yaml my_h1_mujoco.yaml ...` 预演：

  ```yaml
  defaults: [unitree_h1]
  assembly:
    parts:
      body:
        overrides:
          model: {mjcf: h1_sim.xml}   # kp / kd / armature：从真机配置照抄
  sim:
    fixed_base: [body]
    base_height: {body: 1.2}
    ramp_seconds: 2.0
    damping_kd: 8.0
  ```

- **手臂姿态：** 手臂下垂时 `right_elbow` 读数约 1.52 rad。角度越小
  前臂抬得越高，0.0 是水平朝前。测试中把它移到了
  0.8 rad，比下垂时抬高约 40°。
- **交还控制权：** 还没有在 H1 上运行过 `restore`。请从遥控器或 app 重新进入
  内置服务。

在 H1 上运行 `move --joint right_elbow --target 0.8` 的结果，真机对比
同一动作在 MuJoCo 中从真机姿态起步的运行：

| | 真机 | 仿真 |
|---|---|---|
| 目标 0.8 处的稳态 | 0.835 rad | 0.857 rad |
| 最大跟踪误差 | 0.043 rad | 0.172 rad |
| 肘关节峰值力矩 | 2.35 Nm | 9.89 Nm |
| 其他关节的最大漂移 | 0.017 rad | 0.224 rad |
| 回到起点（命令值 1.523） | 1.519 rad | 1.508 rad |

事先做的 5 s `hold` 在全部 19 个电机使能的情况下跟踪误差在 0.001 rad 以内。
与 G1 不同，这里仿真的跟踪效果*比硬件差*：使用的 unitree_ros H1 模型
没有调过 armature 或摩擦，所以应把它的数字当作
保守的预演，而不是预测。

## 7. 一个控制循环

单关节运动跑通之后，同一个驱动就能运行任何使用
`Robot` 接口的控制循环。前几次运行时让机器人保持吊起，并且
先在 MuJoCo 中从真机姿态起步预演新动作（[第 5 节](#从真机姿态起步预演)）。基本循环见
[快速开始](../../README.zh-CN.md#仿真)；在真机上，按
`robot.embodiment.control_hz` 控制循环节奏。

驱动做了什么：`connect()` 初始化 DDS，创建发布者和
订阅者，并等待第一条 `LowState`。`get_observation()` 读取
`motor_state[i].q/dq/tau_est` 和 IMU（四元数 / 陀螺仪 /
加速度计）。`send_action()` 按动作的模式填写 `motor_cmd[i]` 的 `q/dq/tau/kp/kd`，
计算 CRC，然后发布。

## 8. 常见问题

- **CRC 是必需的：** 每条 `LowCmd` 在发布前都必须经过 `crc.Crc(cmd)`，
  否则固件会静默丢弃它。驱动已处理这一点。
- **回传 `mode_machine`：** 使用 `hg` 时，每条命令都必须带上
  从 `LowState` 读到的 `mode_machine` 值；值不对会被拒绝。
  驱动已处理这一点。
- **高层模式占用总线：** 内置运动服务（sport /
  locomotion / `ai`）处于活动状态时，它占用 `rt/lowcmd`，你的命令不起
  作用。`unitree_setup.py status` 会显示这一点；`--release-motion-mode`
  释放它，`restore` 把它恢复。如果要使用内置动作
  而不是关节命令，见[机器人 skill（G1）](#机器人-skillg1)。
- **机器人 skill 失败，RPC 错误码 3102：** 没有内置服务应答，
  通常是因为它已被释放。运行 `unitree_setup.py restore`。
- **没有 LowState：** `connect()` 约 5 s 后超时。检查网卡名、
  子网，以及机器人是否已上电。
- **关节数量 / 顺序：** G1 = 29，H1-2 = 27，H2 = 31，H1 = 19。用错
  配置会导致维度不匹配。
- **并联踝关节（hg 上的 `mode_pr`）：** 在部分型号上，踝关节以 PR 或
  AB 模式运行，默认 PR。AB（并联）控制需要扩展驱动
  以设置 `mode_pr`。
- **旧版 H1 属于 go 消息族：** 设置 `msg_family: go`，否则消息类型
  会对不上。
- **头部相机错误 3102：** `RPC_ERR_CLIENT_SEND` 表示没有订阅者接收
  请求，即 PC2（192.168.123.164）上的 `videohub_pc4` 服务没有
  运行；网络和相机本身可能都没问题。D435i 同一时间只能
  被一个进程打开，所以要检查 PC2 上是否有其他相机服务占用了它
  （`ps -ef | grep -i camera`，`lsusb`）。
