# 使用说明（中文）

## 快速开始

1. 打开场景文件：Greedy.wbt。
2. 如果你想实现自己的算法，请修改 `/decision_making/waypoints_cruise.py` 里的 `mode_developing()`。

## `waypoints_cruise.py` 的运行方式

`waypoints_cruise.py` 默认每 10 帧运行一次。
如需修改频率，请打开 `/controllers/supervisor_controller/supervisor_controller.py` 并搜索 `CRUISE_INTERVAL_FRAMES`。

目前 decision making 有两种可用的运行模式和一种正在开发的模式。
如需切换模式，请在 `waypoints_cruise.py` 中搜索 `DEFAULT_MODE` 并修改。

## 可用的运行数据

运行时可获取以下数据：

- 当前机器人位置（含 bearing 角度）
- 小球位置与类型
- 障碍机器人位置
- 当前仿真时间（秒，来自 time.txt）

具体读取方式可参考 `waypoints_cruise.py` 中的 `mode_developing()`。

## 实时监控

如果需要实时查看数据和格式，请在 VS Code 中打开
`controllers/supervisor_controller/` 下对应的 `.txt` 文件，它们会实时更新。

演示视频：[Demo.mp4](Demo.mp4)

![](image-1.png)

### waypoints_history 说明

[controllers/supervisor_controller/waypoints_history.txt](controllers/supervisor_controller/waypoints_history.txt) 会记录所有主机器人目标点的历史轨迹。

- 每次程序启动时会追加一个新的会话头，包含时间戳和随机种子。
- 每条记录格式为：`(x, y, angle),  status`
	- `x, y` 是目标点坐标
	- `angle` 是目标朝向（角度），可能为 `None`
	- `status` 有两种：
		- `reached`：机器人到达该目标点
		- `cut`：目标点在到达前被新目标替换
- 文件会持续追加，便于回放和调试路径变化

## 当前进度

1. 目前只能获取到还在场内的小球位置，还缺少一个 `camera_detection` 函数，用于根据视场角和距离反推机器人真正能看到的球。
2. 还缺少一个把其他机器人坐标转换并模拟成雷达数据的函数。
3. 尚未研究避障算法。

---

# Decision Making Guide (English)

## Quick Start

1. Open the world file: Greedy.wbt.
2. To implement your own algorithm, edit `mode_developing()` in `waypoints_cruise.py`.

## How `waypoints_cruise.py` is executed

`waypoints_cruise.py` runs every 10 frames by default.
To change this interval, open `supervisor_controller.py` and search for `CRUISE_INTERVAL_FRAMES`.

There are currently two available decision making modes.
To switch modes, search for `DEFAULT_MODE` in `waypoints_cruise.py` and change it.

## Available runtime data

While the simulation is running, you can read live data such as:

- Current robot position (with bearing angle)
- Ball positions and types
- Obstacle robot positions
- Current simulation time in seconds (from time.txt)

See `mode_developing()` in `waypoints_cruise.py` for how to access these values.

## Real-time monitoring

To monitor live data and file formats, open the corresponding `.txt` files under
`controllers/supervisor_controller/` in VS Code. They update in real time.

Demo video: [Demo.mp4](Demo.mp4)

![](image-1.png)

### waypoints_history details

[controllers/supervisor_controller/waypoints_history.txt](controllers/supervisor_controller/waypoints_history.txt) records the full history of target waypoints for the main robot.

- A new session header is appended at startup, including a timestamp and random seed.
- Each record uses the format: `(x, y, angle),  status`
	- `x, y` are target coordinates
	- `angle` is the target orientation in degrees, or `None`
	- `status` values:
		- `reached`: the robot reached the waypoint
		- `cut`: the waypoint was replaced before being reached
- The file is append-only, useful for replay and debugging path changes

## Current progress

1. At the moment, only the positions of balls still in the field are available. A `camera_detection` function is still missing to infer which balls are actually visible based on field of view and distance.
2. A function is still missing to convert other robots' positions into simulated radar data.
3. Obstacle avoidance algorithms have not been researched yet.
