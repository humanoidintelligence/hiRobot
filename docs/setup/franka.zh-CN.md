# Franka 设置指南（FR3 / Panda）

[English](./franka.md) | 简体中文

驱动：`franka_droid`。hiRobot 通过 DROID 机器人服务控制 Franka FR3，自己不经 libfranka 连机械臂。

在 [DROID](https://github.com/droid-dataset/droid) 平台上，机械臂和夹爪已经由 DROID 的机器人服务运行
（控制计算机上的 `scripts/server/run_server.py`，zerorpc，端口 4242）。`franka_droid` 驱动
（`src/hirobot/drivers/body/franka/droid/`）是这个服务的客户端，所以 hiRobot 不需要实时内核、libfranka，也不直连 FCI，
只需要 `pip install zerorpc`。它只调用上游 DROID 就有的方法（`get_robot_state`、`update_joints`、`update_pose`、
`update_gripper`），不启动也不停止服务的控制器。服务按 DROID 的方式启动。

## 配置

配置：`body/franka/fr3_droid`（机械臂，夹爪在 `ee` 坐标系上，和 DROID 一样 `control_hz: 15`）和
`robots/franka_fr3_droid`（这个本体，加一台 ZED Mini `wrist` 相机和两台 ZED 2i `left`、`right`）。
服务地址和相机序列号写进本地配置：

```yaml
defaults: [robots/franka_fr3_droid]

assembly:
  parts:
    body: {overrides: {driver: {type: franka_droid, ip: 172.16.0.2}}}
    wrist: {overrides: {driver: {serial: <腕部 ZED 序列号>}}}
    left: {overrides: {driver: {serial: <左侧 ZED 序列号>}}}
    right: {overrides: {driver: {serial: <右侧 ZED 序列号>}}}
```

ZED 相机用 `stereolabs_zed` 相机驱动，跑在插着相机的那台计算机上（要 ZED SDK、它的 `pyzed` wheel 和 NVIDIA GPU）。
一台 ZED 同一时间只能被一个进程打开，先停掉 DROID 自己的相机读取。

## 驱动做了什么

| | `franka_droid` |
|---|---|
| 参数 | `ip`（默认 `172.16.0.2`）、`port`（4242）、`timeout_s`（10）、`home_q`（`reset()` 用的 7 个关节角，可选）、`gripper`（`true`） |
| `get_observation()` | `joint_pos`、`joint_vel`、`ee_poses["ee"]`（DROID 的 `cartesian_position`，欧拉角 `xyz` 换成四元数）、`gripper["ee"]`（DROID 的 0 开 1 合换成 0 合 1 开） |
| `POSITION` | `update_joints(q, velocity=False, blocking=False)`：服务的控制器跟踪关节目标 |
| `CARTESIAN` | `ee_targets["ee"]` → `update_pose(pose, velocity=False, blocking=False)`；IK 或阻抗控制由服务做 |
| `Action.gripper["ee"]` | `update_gripper(1 - opening, velocity=False, blocking=False)`，两种模式都可以带 |
| `reset()` | 设了 `home_q` 时阻塞地 `update_joints` 过去；否则什么也不做 |
| `estop()` | 把测得的关节作为目标发出，机械臂停在原地。服务的控制器继续运行；要硬停用 Franka 自己的急停按钮 |

每个系统线程有自己的 zerorpc 客户端（zerorpc 跑在 gevent 上），所以别的线程调 `estop()` 不用等控制循环的调用。
`connect()` 先读一次状态，服务不应答时抛出 `ConnectionError`。

## 真机验证

在一台 DROID 平台上验证过（FR3、Robotiq 2F-85、三台 ZED，2026-11-12）：15 Hz 读状态，单次调用中位 2.4 ms；
`CARTESIAN` 以 15 Hz 流式发目标，末端上移 10 cm 再回来，最快 4 cm/s，跟踪误差不超过 6.3 mm，结束时离起点 1.1 mm；
夹爪到 0.50（测得 0.502）再回去；`make_robot` 同时读机械臂和三台 ZED（1280x720 彩色加深度）。

## 常见问题

服务和机械臂之间的连接断开后（服务报 `Socket is not connected`），读状态全部失败，直到服务恢复机械臂。
这由服务负责，`franka_droid` 不处理。
