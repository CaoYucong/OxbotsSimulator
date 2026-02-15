# Oxbots Simulator

Webots simulation repository for UniBots. The current main workflow is multi-robot + supervisor + external decision script integration.

## ⚠️ Important: Pause + Reset before editing worlds

Webots starts running automatically after opening a world. Edits made while running are often not saved.

Please do this first:

1. Press **Pause** (⏸)
2. Press **Reset Simulation** (⏮)
3. Then edit and save

Otherwise, your changes may be lost after closing/resetting.

---

## 1) Environment

- Webots: https://cyberbotics.com
- World file version: `R2025a` (see `worlds/*.wbt`)

---

## 2) Quick Start (main workflow)

### Step 1. Open the main world

Open this in Webots:

```text
worlds/Decision_Making.wbt
```

### Step 2. Select active decision-making branch

Edit:

```text
who_is_developing.txt
```

Available values:

- `cyc` -> uses `decision_making_cyc/waypoints_cruise.py`
- `wly` -> uses `decision_making_wly/waypoints_cruise.py`

### Step 3. Run simulation

`supervisor_controller` will automatically:

- Start and manage main robot + obstacle robots
- Randomize initial ball positions (fixed seed)
- Execute selected `waypoints_cruise.py` every `15` frames
- Read/write real-time data files (positions, visible balls, status, speed, etc.)
- Start the field viewer (default `http://localhost:5001`)

---

## 3) Runtime architecture (`Decision_Making.wbt`)

- Main robot: `DEF MY_ROBOT`
- Obstacle robots: `DEF OBSTACLE_ROBOT_1/2/3`
- Controller: `controllers/supervisor_controller/supervisor_controller.py`
- Decision script: `decision_making_cyc/waypoints_cruise.py` or `decision_making_wly/waypoints_cruise.py`

Data flow:

1. Supervisor writes `controllers/supervisor_controller/real_time_data/*.txt` every frame
2. Decision script reads these files and writes back waypoints/status in its `real_time_data`
3. Main robot moves non-blocking based on `dynamic_waypoints.txt`

---

## 4) Key directories (current repository)

```text
OxbotsSimulator/
├── worlds/
│   ├── Decision_Making.wbt          # Main match/decision integration world (recommended)
│   ├── Arena_Development.wbt        # Arena/material development
│   ├── Position_Estimate_MATLAB.wbt # MATLAB pose estimation world (estimate_robot_pose)
│   └── moose_demo.wbt               # Webots moose demo (moose_path_following)
│
├── controllers/
│   ├── supervisor_controller/       # Main controller (multi-robot + balls + real-time data)
│   ├── other_go_random/             # Simple random-motion Python controller
│   ├── moose_path_following/        # C controller demo (with Makefile)
│   ├── estimate_robot_pose/         # MATLAB: camera/AprilTag pose and coordinate estimation
│   └── ball_detection/              # MATLAB: ball detection script
│
├── decision_making_cyc/             # CYC strategy scripts and runtime state files
├── decision_making_wly/             # WLY strategy scripts and runtime state files
│
├── protos/                          # Arena, balls, robots, AprilTag, etc.
├── textures/                        # AprilTag, arena, and robot textures
├── tools/
│   ├── field_viewer/                # Real-time visualization service (server.py + index.html)
│   └── camera_calibration/          # AprilTag/camera calibration tools (MATLAB)
│
├── who_is_developing.txt            # Strategy selection (cyc/wly)
├── Rulebook Unibots 2026 V1.1 - 02 Oct 2025.pdf
└── README.md
```

---

## 5) Real-time data files (commonly used)

Directory: `controllers/supervisor_controller/real_time_data/`

- `dynamic_waypoints.txt`: current target waypoint for main robot (written by decision script)
- `waypoint_status.txt`: `going/reached`
- `current_position.txt`: main robot position and heading
- `obstacle_robot.txt`: obstacle robot positions and headings
- `ball_position.txt`: in-field ball positions and types
- `visible_balls.txt`: balls visible to main robot
- `speed.txt`: main robot speed (m/s)
- `time.txt`: simulation time (seconds)
- `obstacle_plan.txt`: obstacle robot cyclic path
- `waypoints_history.txt`: waypoint history (`reached/cut`)

`decision_making_cyc/real_time_data/` and `decision_making_wly/real_time_data/` are used for internal strategy state and debugging outputs.

---

## 6) Other worlds and usage

- `worlds/Position_Estimate_MATLAB.wbt`: experiments for `estimate_robot_pose` controller
- `worlds/moose_demo.wbt`: `moose_path_following` demo scene
- `worlds/Arena_Development.wbt`: arena structure/material development

---

## 7) Troubleshooting

- Robot does not move as expected:
  - confirm `who_is_developing.txt` is `cyc` or `wly`
  - check the corresponding `decision_making_*/waypoints_cruise.py` is runnable
- Decision seems not updating:
  - inspect `controllers/supervisor_controller/real_time_data/dynamic_waypoints.txt`
  - inspect whether `waypoint_status.txt` keeps changing
- Field viewer is unreachable:
  - check whether local port `5001` is occupied

---

## 中文版

UniBots 仿真仓库（Webots）。当前主流程为多机器人 + supervisor + 外部决策脚本联动。

## ⚠️ 重要：编辑世界前先 Pause + Reset

Webots 打开 world 后会自动运行；运行中修改场景通常不会被保存。

请先执行：

1. 按 **Pause**（⏸）
2. 按 **Reset Simulation**（⏮）
3. 再开始编辑并保存

否则修改可能在关闭/重置后丢失。

---

## 1) 环境

- Webots: https://cyberbotics.com
- world 文件版本：`R2025a`（见 `worlds/*.wbt`）

---

## 2) 快速开始（主流程）

### Step 1. 打开主 world

在 Webots 中打开：

```text
worlds/Decision_Making.wbt
```

### Step 2. 选择当前决策开发分支

编辑根目录文件：

```text
who_is_developing.txt
```

可选值：

- `cyc` -> 使用 `decision_making_cyc/waypoints_cruise.py`
- `wly` -> 使用 `decision_making_wly/waypoints_cruise.py`

### Step 3. 直接运行仿真

`supervisor_controller` 会自动：

- 启动并管理主机器人与障碍机器人
- 随机化球初始位置（固定随机种子）
- 每 `15` 帧调用一次对应的 `waypoints_cruise.py`
- 读写实时数据文件（位置、可见球、状态、速度等）
- 启动 field viewer（默认 `http://localhost:5001`）

---

## 3) 运行架构（Decision_Making.wbt）

- 主机器人：`DEF MY_ROBOT`
- 障碍机器人：`DEF OBSTACLE_ROBOT_1/2/3`
- 控制器：`controllers/supervisor_controller/supervisor_controller.py`
- 决策脚本：`decision_making_cyc/waypoints_cruise.py` 或 `decision_making_wly/waypoints_cruise.py`

数据通道：

1. Supervisor 每帧写入 `controllers/supervisor_controller/real_time_data/*.txt`
2. 决策脚本读取这些数据并回写目标点/状态到对应 `real_time_data`
3. 主机器人根据 `dynamic_waypoints.txt` 非阻塞移动

---

## 4) 关键目录说明（当前仓库）

```text
OxbotsSimulator/
├── worlds/
│   ├── Decision_Making.wbt          # 主比赛/决策联调场景（推荐）
│   ├── Arena_Development.wbt        # 场地搭建与素材开发
│   ├── Position_Estimate_MATLAB.wbt # MATLAB 位姿估计场景（estimate_robot_pose）
│   └── moose_demo.wbt               # Webots moose 示例（moose_path_following）
│
├── controllers/
│   ├── supervisor_controller/       # 主控制器（多机器人+球+实时数据）
│   ├── other_go_random/             # 简单随机运动 Python 控制器
│   ├── moose_path_following/        # C 控制器示例（含 Makefile）
│   ├── estimate_robot_pose/         # MATLAB: 相机/AprilTag 位姿与坐标估计
│   └── ball_detection/              # MATLAB: 球检测脚本
│
├── decision_making_cyc/             # CYC 的策略脚本与运行态文件
├── decision_making_wly/             # WLY 的策略脚本与运行态文件
│
├── protos/                          # 场地、球、机器人、AprilTag 等 PROTO
├── textures/                        # AprilTag、场地与机器人纹理
├── tools/
│   ├── field_viewer/                # 实时可视化服务（server.py + index.html）
│   └── camera_calibration/          # AprilTag/相机标定工具（MATLAB）
│
├── who_is_developing.txt            # 决策脚本选择（cyc/wly）
├── Rulebook Unibots 2026 V1.1 - 02 Oct 2025.pdf
└── README.md
```

---

## 5) 实时数据文件（常用）

目录：`controllers/supervisor_controller/real_time_data/`

- `dynamic_waypoints.txt`：主机器人当前目标点（决策脚本写入）
- `waypoint_status.txt`：`going/reached`
- `current_position.txt`：主机器人位置与朝向
- `obstacle_robot.txt`：障碍机器人位置与朝向
- `ball_position.txt`：场内球位置与类型
- `visible_balls.txt`：主机器人视野内可见球
- `speed.txt`：主机器人速度（m/s）
- `time.txt`：仿真时间（秒）
- `obstacle_plan.txt`：障碍机器人循环路径
- `waypoints_history.txt`：目标点历史（reached/cut）

`decision_making_cyc/real_time_data/` 与 `decision_making_wly/real_time_data/` 用于策略内部状态缓存与调试输出。

---

## 6) 其他 world 的用途

- `worlds/Position_Estimate_MATLAB.wbt`：`estimate_robot_pose` 控制器相关实验
- `worlds/moose_demo.wbt`：`moose_path_following` 示例场景
- `worlds/Arena_Development.wbt`：场地结构与材质开发

---

## 7) 常见排查

- world 打开后机器人不按预期运动：
  - 确认 `who_is_developing.txt` 是否为 `cyc` 或 `wly`
  - 检查对应 `decision_making_*/waypoints_cruise.py` 是否可执行
- 决策似乎不更新：
  - 查看 `controllers/supervisor_controller/real_time_data/dynamic_waypoints.txt`
  - 查看 `waypoint_status.txt` 是否持续变化
- 可视化打不开：
  - 检查本地端口 `5001` 是否被占用


