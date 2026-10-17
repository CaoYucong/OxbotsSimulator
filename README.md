# Oxbots Simulator

Webots simulation repository for UniBots. The main workflow is multi-robot supervisor control plus external decision scripts, with an optional ROS2 stack for real-robot / Pi deployment.

## Important: Pause + Reset before editing worlds

Webots starts running automatically after opening a world. Edits made while running are often not saved.

1. Press **Pause** (⏸)
2. Press **Reset Simulation** (⏮)
3. Then edit and save

Otherwise, changes may be lost after closing or resetting.

---

## 1) Environment

- Webots: https://cyberbotics.com
- World file version: `R2025a` (see `worlds/*.wbt`)
- Optional ROS2: Jazzy (see `ROS/`)

---

## 2) Quick Start (main workflow)

### Step 1. Open the main world

Open in Webots:

```text
worlds/Decision_making.wbt
```

### Step 2. Select active decision-making branch

Edit root `config.json` (preferred). Legacy fallback: `who_is_developing.txt`.

Key fields:

| Field | Meaning |
| --- | --- |
| `develop_branch` | `cyc` / `wly` / `xjj` / `ros` |
| `data_flow` | `web` (HTTP via field viewer) or `file` |
| `run_on_pi` | Pull decisions from Pi when `true` |
| `pi_ip` | Raspberry Pi address for remote decision data |
| `camera` / `pose_estimation` | Feature toggles used by supervisor / tools |
| `default_linear_velocity` / `default_angular_velocity` | Motion defaults |

Branch → script mapping:

- `cyc` → `decision_making_cyc/waypoints_cruise.py`
- `wly` → `decision_making_wly/waypoints_cruise.py`
- `xjj` → `decision_making_xjj/waypoints_cruise.py`
- `ros` → `decision_making_ros/waypoints_cruise.py` (ROS-aligned strategy)

Example:

```json
{
  "develop_branch": "ros",
  "data_flow": "web",
  "run_on_pi": true,
  "pi_ip": "192.168.50.2"
}
```

### Step 2.1 Select strategy mode (per branch)

Most branches resolve mode as: `mode.txt` → CLI/`MODE` env → `DEFAULT_MODE` in `waypoints_cruise.py`.

- CYC: e.g. `improved_nearest_v3`
- WLY: e.g. `developing`
- XJJ: e.g. `improved_nearest`
- ROS script / ROS node: e.g. `improved_nearest_v3_5` (also set in `ROS/.../config/params.yaml` → `decision_node.mode`)

### Step 3. Run simulation

`supervisor_controller` will:

- Manage main robot + obstacle robots
- Randomize ball positions (seed from `controllers/supervisor_controller/random_seed.txt`)
- Invoke the selected `waypoints_cruise.py` on a fixed frame interval
- Exchange real-time data (files and/or web endpoints)
- Start the field viewer (default `http://localhost:5001`)

---

## 3) Runtime architecture (`Decision_making.wbt`)

- Main robot: `DEF MY_ROBOT`
- Obstacle robots: `DEF OBSTACLE_ROBOT_1/2/3`
- Controller: `controllers/supervisor_controller/supervisor_controller.py`
- Decision script: `decision_making_<branch>/waypoints_cruise.py`

Data flow (`data_flow: "web"`):

1. Supervisor publishes simulation state to field viewer HTTP endpoints
2. Decision script (or ROS stack on Pi) reads state and posts decisions / waypoints
3. Main robot follows the active dynamic waypoint non-blocking

Data flow (`data_flow: "file"`):

1. Supervisor writes `controllers/supervisor_controller/real_time_data/*.txt` every frame
2. Decision script reads those files and writes waypoints/status
3. Main robot moves from `dynamic_waypoints.txt`

---

## 4) Key directories

```text
OxbotsSimulator/
├── worlds/
│   ├── Decision_making.wbt          # Main match / decision integration (recommended)
│   ├── Arena_Development.wbt        # Arena / material development
│   ├── Position_Estimate_MATLAB.wbt # MATLAB pose estimation world
│   └── moose_demo.wbt               # Webots moose demo
│
├── controllers/
│   ├── supervisor_controller/       # Main controller (robots + balls + data)
│   ├── other_go_random/             # Simple random-motion Python controller
│   ├── moose_path_following/        # C controller demo
│   ├── estimate_robot_pose/         # MATLAB: camera / AprilTag pose
│   └── ball_detection/              # MATLAB: ball detection
│
├── decision_making_cyc/             # CYC strategy
├── decision_making_wly/             # WLY strategy
├── decision_making_xjj/             # XJJ strategy
├── decision_making_ros/             # ROS-aligned strategy scripts
│
├── ROS/                             # ROS2 Jazzy workspace + Pi deploy
│   ├── ros2_ws/src/unibots/         # `unibots` package (nodes + launch)
│   └── deploy_and_run_ros_pi.sh     # Sync / build / launch on Raspberry Pi
│
├── pose_estimation/                 # Standalone AprilTag / intrinsic helpers
├── protos/                          # Arena, balls, robots, AprilTag PROTOs
├── textures/                        # AprilTag, arena, robot textures
├── tools/
│   ├── field_viewer/                # Real-time visualization (server.py)
│   ├── camera_calibration_matlab/   # MATLAB calibration
│   ├── camera_calibration_python/   # Python calibration
│   ├── auto_photo_capture/          # Capture helpers
│   ├── testing/                     # Automation / loop tests
│   └── system_backup/               # Backup utilities
│
├── benchmarks/                      # UniBots benchmark spreadsheets
├── rpi5/                            # Pi-side motor helpers
├── config.json                      # Branch, data flow, Pi, feature flags
└── README.md
```

---

## 5) Real-time data files (commonly used)

Directory: `controllers/supervisor_controller/real_time_data/`

- `dynamic_waypoints.txt` — current target for main robot
- `waypoint_status.txt` — `going` / `reached`
- `current_position.txt` — main robot pose / heading
- `obstacle_robot.txt` — obstacle robot poses
- `ball_position.txt` — in-field ball positions / types
- `visible_balls.txt` — balls visible to main robot
- `speed.txt` — main robot speed (m/s)
- `time.txt` — simulation time (seconds)
- `obstacle_plan.txt` — obstacle cyclic path
- `waypoints_history.txt` — waypoint history (`reached` / `cut`)
- `radar_sensor.txt` — radar / ToF-style readings when used

Each `decision_making_*/real_time_data/` holds strategy-local state and debug outputs.

---

## 6) ROS2 stack (overview)

Full docs: [`ROS/README.md`](ROS/README.md) and [`ROS/ros2_ws/src/unibots/README.md`](ROS/ros2_ws/src/unibots/README.md).

```bash
cd ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

Or deploy to Pi:

```bash
./ROS/deploy_and_run_ros_pi.sh
```

Launched nodes (package `unibots`):

| Node | Role |
| --- | --- |
| `web_bridge_node` | HTTP mirror / field-viewer bridge |
| `front_camera_node` | USB or upstream camera → `/front_camera` |
| `ball_detection_node` | Local Roboflow inference → `/visible_balls`, overlays |
| `pose_estimation_camera_node` | AprilTag → `/current_position_camera` |
| `pose_estimation_sensor_fusion_node` | Camera + wheel/IMU fusion → `/current_position` |
| `decision_node` | Strategy → `/dynamic_waypoint`, `/decisions` |
| `motion_control_node` | Waypoint following + motors |
| `timer_node` | Simulation clock on `/time` |
| `reset_button_node` | Hardware reset / run control |

Config: `ROS/ros2_ws/src/unibots/config/params.yaml`.

---

## 7) Other worlds

- `worlds/Position_Estimate_MATLAB.wbt` — `estimate_robot_pose` experiments
- `worlds/moose_demo.wbt` — `moose_path_following` demo
- `worlds/Arena_Development.wbt` — arena structure / materials

---

## 8) Troubleshooting

- Robot does not move as expected:
  - confirm `config.json` → `develop_branch` is `cyc` / `wly` / `xjj` / `ros`
  - check the matching `decision_making_*/waypoints_cruise.py` runs
- Decision seems stuck:
  - inspect `dynamic_waypoints.txt` / `waypoint_status.txt`, or field-viewer `/data/decisions`
  - if `run_on_pi: true`, confirm Pi IP and ROS launch are up
- Field viewer unreachable:
  - check port in `controllers/supervisor_controller/html_port.txt` (often `5001`)
- ROS camera / `cv_bridge` NumPy errors:
  - see troubleshooting in `ROS/ros2_ws/src/unibots/README.md`

---

## 中文版

UniBots 的 Webots 仿真仓库。主流程为多机器人 supervisor + 外部决策脚本；可选 ROS2 栈用于真机 / 树莓派部署。

## 重要：编辑世界前先 Pause + Reset

Webots 打开 world 后会自动运行；运行中修改场景通常不会被保存。

1. 按 **Pause**（⏸）
2. 按 **Reset Simulation**（⏮）
3. 再编辑并保存

否则修改可能在关闭/重置后丢失。

---

## 1) 环境

- Webots: https://cyberbotics.com
- world 文件版本：`R2025a`（见 `worlds/*.wbt`）
- 可选 ROS2：Jazzy（见 `ROS/`）

---

## 2) 快速开始（主流程）

### Step 1. 打开主 world

在 Webots 中打开：

```text
worlds/Decision_making.wbt
```

### Step 2. 选择决策开发分支

编辑根目录 `config.json`（推荐）。兼容旧文件：`who_is_developing.txt`。

主要字段：

| 字段 | 含义 |
| --- | --- |
| `develop_branch` | `cyc` / `wly` / `xjj` / `ros` |
| `data_flow` | `web`（经 field viewer HTTP）或 `file` |
| `run_on_pi` | 为 `true` 时从树莓派拉取决策 |
| `pi_ip` | 树莓派地址 |
| `camera` / `pose_estimation` | supervisor / 工具功能开关 |
| `default_linear_velocity` / `default_angular_velocity` | 默认运动参数 |

分支 → 脚本：

- `cyc` → `decision_making_cyc/waypoints_cruise.py`
- `wly` → `decision_making_wly/waypoints_cruise.py`
- `xjj` → `decision_making_xjj/waypoints_cruise.py`
- `ros` → `decision_making_ros/waypoints_cruise.py`

示例：

```json
{
  "develop_branch": "ros",
  "data_flow": "web",
  "run_on_pi": true,
  "pi_ip": "192.168.50.2"
}
```

### Step 2.1 选择策略模式（按分支）

多数分支模式优先级：`mode.txt` → CLI/`MODE` → `waypoints_cruise.py` 中的 `DEFAULT_MODE`。

- CYC：如 `improved_nearest_v3`
- WLY：如 `developing`
- XJJ：如 `improved_nearest`
- ROS 脚本 / ROS 节点：如 `improved_nearest_v3_5`（也可在 `params.yaml` 的 `decision_node.mode` 设置）

### Step 3. 运行仿真

`supervisor_controller` 会：

- 管理主机器人与障碍机器人
- 按种子随机化球位置
- 按固定帧间隔调用对应 `waypoints_cruise.py`
- 通过文件和/或 Web 交换实时数据
- 启动 field viewer（默认 `http://localhost:5001`）

---

## 3) 运行架构（`Decision_making.wbt`）

- 主机器人：`DEF MY_ROBOT`
- 障碍机器人：`DEF OBSTACLE_ROBOT_1/2/3`
- 控制器：`controllers/supervisor_controller/supervisor_controller.py`
- 决策脚本：`decision_making_<branch>/waypoints_cruise.py`

`data_flow: "web"`：

1. Supervisor 将仿真状态发布到 field viewer HTTP
2. 决策脚本（或 Pi 上的 ROS）读取状态并回写决策/目标点
3. 主机器人非阻塞跟踪动态目标点

`data_flow: "file"`：

1. Supervisor 每帧写入 `controllers/supervisor_controller/real_time_data/*.txt`
2. 决策脚本读写这些文件
3. 主机器人根据 `dynamic_waypoints.txt` 移动

---

## 4) 关键目录

```text
OxbotsSimulator/
├── worlds/                          # Webots 场景
├── controllers/                     # supervisor 与其它控制器
├── decision_making_cyc|wly|xjj|ros/ # 各分支策略
├── ROS/                             # ROS2 Jazzy + Pi 部署
├── pose_estimation/                 # 独立 AprilTag / 内参工具
├── protos/ / textures/              # 模型与纹理
├── tools/                           # field viewer、标定、测试等
├── benchmarks/ / rpi5/              # 基准表与 Pi 辅助脚本
├── config.json                      # 分支、数据流、Pi、功能开关
└── README.md
```

更细的英文树见上方 English 第 4 节。

---

## 5) 实时数据文件（常用）

目录：`controllers/supervisor_controller/real_time_data/`

- `dynamic_waypoints.txt`：主机器人当前目标点
- `waypoint_status.txt`：`going` / `reached`
- `current_position.txt`：主机器人位置与朝向
- `obstacle_robot.txt`：障碍机器人位姿
- `ball_position.txt`：场内球位置与类型
- `visible_balls.txt`：主机器人可见球
- `speed.txt`：速度（m/s）
- `time.txt`：仿真时间（秒）
- `obstacle_plan.txt`：障碍循环路径
- `waypoints_history.txt`：目标点历史
- `radar_sensor.txt`：雷达 / ToF 读数（如启用）

各 `decision_making_*/real_time_data/` 存放策略内部状态与调试输出。

---

## 6) ROS2 栈（概览）

详见 [`ROS/README.md`](ROS/README.md) 与 [`ROS/ros2_ws/src/unibots/README.md`](ROS/ros2_ws/src/unibots/README.md)。

```bash
cd ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

或部署到树莓派：

```bash
./ROS/deploy_and_run_ros_pi.sh
```

启动节点见英文第 6 节表格；参数文件：`ROS/ros2_ws/src/unibots/config/params.yaml`。

---

## 7) 其他 world

- `worlds/Position_Estimate_MATLAB.wbt`：`estimate_robot_pose` 实验
- `worlds/moose_demo.wbt`：`moose_path_following` 示例
- `worlds/Arena_Development.wbt`：场地结构与材质开发

---

## 8) 常见排查

- 机器人不按预期运动：检查 `config.json` 的 `develop_branch` 与对应 `waypoints_cruise.py`
- 决策不更新：检查 `dynamic_waypoints.txt` / field viewer `/data/decisions`；若 `run_on_pi` 确认 Pi 与 ROS 已启动
- 可视化打不开：检查 `html_port.txt`（常为 `5001`）
- ROS `cv_bridge` / NumPy 问题：见 `ROS/ros2_ws/src/unibots/README.md`
