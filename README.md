# Oxbots Simulator

Webots simulation repository for UniBots. The main workflow is a multi-robot supervisor plus an external decision script. An optional ROS 2 Jazzy stack on a Raspberry Pi runs perception, the same style of strategy, and motor control for the real robot, and can feed decisions back into the simulator.

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
- Optional ROS 2: Jazzy (see `ROS/`)

---

## 2) Quick start

### Step 1. Open the main world

Open in Webots:

```text
worlds/Decision_making.wbt
```

### Step 2. Select the active decision branch

Edit the root `config.json`. Legacy fallback: `who_is_developing.txt`.

| Field | Meaning |
| --- | --- |
| `develop_branch` | `cyc` / `wly` / `xjj` / `ros` |
| `data_flow` | `web` (HTTP via the field viewer) or `file` |
| `run_on_pi` | When `true`, pull decisions from the Pi |
| `pi_ip` | Raspberry Pi address |
| `camera` / `pose_estimation` | Feature toggles used by the supervisor and tools |
| `default_linear_velocity` / `default_angular_velocity` | Motion defaults for the supervisor |

Branch → script:

- `cyc` → `decision_making_cyc/waypoints_cruise.py`
- `wly` → `decision_making_wly/waypoints_cruise.py`
- `xjj` → `decision_making_xjj/waypoints_cruise.py`
- `ros` → `decision_making_ros/waypoints_cruise.py`

Example:

```json
{
  "develop_branch": "ros",
  "data_flow": "web",
  "run_on_pi": true,
  "pi_ip": "192.168.50.2"
}
```

### Step 2.1 Select the strategy mode

Most branches resolve the mode as: `mode.txt` → CLI / `MODE` env → `DEFAULT_MODE` in that branch's `waypoints_cruise.py`.

- CYC: for example `improved_nearest_v3`
- WLY: for example `developing`
- XJJ: for example `improved_nearest`
- ROS script and ROS node: `improved_nearest_v3_5`, `planned`, or `exploration_test`

On the Pi, the live mode is `decision_node.mode` in `ROS/ros2_ws/src/unibots/config/params.yaml`. The node calls `decide_from_ros_state()` in `decision_cruise.py`. If that parameter is empty, the planner falls back to `DEFAULT_MODE` (`improved_nearest_v3_5`).

### Step 3. Run the simulation

`controllers/supervisor_controller/supervisor_controller.py` will:

- Manage the main robot and the obstacle robots
- Randomize ball positions (seed from `controllers/supervisor_controller/random_seed.txt`)
- Invoke the selected `waypoints_cruise.py` on a fixed frame interval
- Exchange state through files and/or HTTP
- Start the field viewer (default `http://localhost:5001`, port in `controllers/supervisor_controller/html_port.txt`)

### Step 4. Optional: run the ROS stack

Local:

```bash
cd ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

Or sync, build, and launch on the Pi:

```bash
./ROS/deploy_and_run_ros_pi.sh
```

Keep `config.json` `pi_ip` / `run_on_pi` consistent with the host and port the ROS web bridge is serving.

---

## 3) Runtime architecture (`Decision_making.wbt`)

- Main robot: `DEF MY_ROBOT`
- Obstacle robots: `DEF OBSTACLE_ROBOT_1/2/3`
- Controller: `controllers/supervisor_controller/supervisor_controller.py`
- Decision script: `decision_making_<branch>/waypoints_cruise.py`

`data_flow: "web"`:

1. The supervisor publishes simulation state to field-viewer HTTP endpoints.
2. The decision script, or the ROS stack on the Pi when `run_on_pi` is true, reads that state and posts decisions / waypoints.
3. The main robot follows the active dynamic waypoint without blocking the supervisor loop.

`data_flow: "file"`:

1. The supervisor writes `controllers/supervisor_controller/real_time_data/*.txt` every frame.
2. The decision script reads those files and writes waypoints and status.
3. The main robot moves from `dynamic_waypoints.txt`.

The ROS package is a second runtime. It does not replace the supervisor inside Webots. The supervisor still owns the simulated robots. The Pi stack owns the camera, AprilTag pose, ball detector, strategy, and motors, and exposes decisions over HTTP so the simulator can consume them.

---

## 4) Repository structure

```text
OxbotsSimulator/
├── worlds/
│   ├── Decision_making.wbt          # Main match / decision integration
│   ├── Arena_Development.wbt        # Arena and material development
│   ├── Position_Estimate_MATLAB.wbt # MATLAB pose-estimation world
│   └── moose_demo.wbt               # Webots moose demo
│
├── controllers/
│   ├── supervisor_controller/       # Match supervisor: robots, balls, data, viewer
│   ├── other_go_random/             # Simple random-motion Python controller
│   ├── moose_path_following/        # C path-following demo
│   ├── estimate_robot_pose/         # MATLAB camera / AprilTag pose
│   └── ball_detection/              # MATLAB ball detection
│
├── decision_making_cyc/             # CYC strategy
├── decision_making_wly/             # WLY strategy
├── decision_making_xjj/             # XJJ strategy
├── decision_making_ros/             # ROS-aligned strategy script for the simulator
│
├── ROS/                             # ROS 2 Jazzy workspace and Pi deploy
│   ├── deploy_and_run_ros_pi.sh
│   ├── README.md
│   └── ros2_ws/src/unibots/         # The only ament_python package
│
├── pose_estimation/                 # Standalone AprilTag / intrinsic helpers
├── protos/                          # Arena, balls, robots, AprilTag PROTOs
├── textures/                        # AprilTag, arena, and robot textures
├── tools/
│   ├── field_viewer/                # Real-time visualization (server.py)
│   ├── camera_calibration_matlab/
│   ├── camera_calibration_python/
│   ├── auto_photo_capture/
│   ├── testing/
│   └── system_backup/
│
├── benchmarks/                      # UniBots benchmark spreadsheets
├── rpi5/                            # Pi-side motor helpers (run_motor.py)
├── config.json                      # Branch, data flow, Pi address, feature flags
└── README.md
```

The ROS package layout:

```text
ROS/ros2_ws/src/unibots/
├── package.xml
├── setup.py
├── launch/unibots.launch.py
├── config/
│   ├── params.yaml
│   ├── real_camera_intrinsic.yaml
│   ├── simulation_camera_intrinsic.json
│   └── tag_world_map.json
├── unibots/                         # Importable Python module
│   ├── web_bridge_node.py
│   ├── front_camera_node.py
│   ├── ball_detection_node.py
│   ├── pose_estimation_camera_node.py
│   ├── pose_estimation_sensor_fusion_node.py
│   ├── decision_node.py
│   ├── decision_cruise.py
│   ├── motion_control_node.py
│   ├── timer_node.py
│   └── reset_button_node.py
└── README.md
```

The nested `unibots/unibots` folders are the normal ROS 2 Python layout: the outer folder is the package (`package.xml`, launch, config); the inner folder is the importable module.

---

## 5) Key files

### Simulator

| File | Role |
| --- | --- |
| `config.json` | Selects `develop_branch`, `data_flow`, `run_on_pi`, `pi_ip`, camera / pose toggles, and default velocities. The supervisor reads it at startup. |
| `worlds/Decision_making.wbt` | Match world: main robot, three obstacle robots, balls, arena. |
| `controllers/supervisor_controller/supervisor_controller.py` | Steps the match, writes or serves state, calls the branch script, and drives `MY_ROBOT` from the current waypoint. |
| `controllers/supervisor_controller/random_seed.txt` | Seed for ball randomization. |
| `controllers/supervisor_controller/html_port.txt` | Field-viewer port (often `5001`). |
| `controllers/supervisor_controller/real_time_data/*.txt` | Per-frame state when `data_flow` is `file`, and the files the supervisor still uses as the motion interface. |
| `decision_making_*/waypoints_cruise.py` | One strategy process per branch. Must return quickly; the supervisor calls it on a fixed frame interval. |
| `decision_making_*/real_time_data/` | Strategy-local state and debug output. |
| `tools/field_viewer/server.py` | HTTP view of the field, decisions, and camera. |
| `protos/`, `textures/` | Webots PROTOs and images referenced by the worlds. |
| `pose_estimation/` | Offline AprilTag and intrinsic helpers, separate from the ROS pose nodes. |
| `rpi5/run_motor.py` | Small Pi motor helper outside the ROS launch. |

### ROS package source

| File | Role |
| --- | --- |
| `ROS/ros2_ws/src/unibots/package.xml` | Package `unibots`, build type `ament_python`. Depends on `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `cv_bridge`. |
| `ROS/ros2_ws/src/unibots/setup.py` | Declares the nine `console_scripts` entry points and installs `launch/` plus the four files in `config/` into the package share directory. |
| `ROS/ros2_ws/src/unibots/launch/unibots.launch.py` | Starts every node with `config/params.yaml`. Copies `front_camera_node.use_real_sensor` into ball detection, camera pose, and fusion. When that flag is false, it points ball detection and camera pose at `simulation_camera_intrinsic.json`. If `ROBOFLOW_API_KEY` is set in the environment, it overrides the detector key. |
| `unibots/front_camera_node.py` | USB camera, or an upstream JPEG, published on `/front_camera`. |
| `unibots/pose_estimation_camera_node.py` | AprilTag PnP from `/front_camera`. Publishes the absolute robot pose and the debug image. |
| `unibots/pose_estimation_sensor_fusion_node.py` | Fuses the AprilTag pose with wheel-encoder odometry and publishes `/current_position`. |
| `unibots/ball_detection_node.py` | Local Roboflow inference. Projects detections into field coordinates and publishes `/visible_balls`. |
| `unibots/decision_node.py` | ROS wrapper around the planner. Subscribes to pose, balls, waypoint status, and time; publishes the next waypoint and decision payloads. |
| `unibots/decision_cruise.py` | Strategy library. Modes: `improved_nearest_v3_5`, `planned`, `exploration_test`. `decide_from_ros_state()` is the entry used by the node. The same file can also run as a script (`mode.txt` → CLI → `MODE` → `DEFAULT_MODE`). |
| `unibots/motion_control_node.py` | Waypoint follower. Drives the Pi motors, publishes `going` / `reached`, `moving` / `stopped`, and wheel `JointState`. |
| `unibots/timer_node.py` | Publishes `/time` while `/run` is on. |
| `unibots/reset_button_node.py` | Watches the Pi power button, publishes `/run`, and clears time and decision topics on reset. |
| `unibots/web_bridge_node.py` | Subscribes to the topics above and serves them over HTTP for the field viewer and the Webots supervisor. |
| `ROS/deploy_and_run_ros_pi.sh` | Rsync the workspace to the Pi, `colcon build`, and `ros2 launch`. Can also start the local Roboflow inference container. |
| `ROS/README.md`, `ROS/ros2_ws/src/unibots/README.md` | Package-level build notes and `cv_bridge` / port troubleshooting. |

`decision_making_ros/waypoints_cruise.py` is the simulator-side script selected by `develop_branch: "ros"`. The ROS node does not shell out to that file; it imports `decision_cruise.py` inside the package.

### ROS `config/` files

All four files are installed to `share/unibots/config/` by `setup.py`. Nodes resolve them from the package share directory after `colcon build`.

**`params.yaml`** is the only ROS parameter file the launch loads into every node. Each top-level key is a node name; parameters live under `ros__parameters`.

| Section | What it controls |
| --- | --- |
| `web_bridge_node` | Local HTTP bind (`local_host` / `local_port`, default `0.0.0.0:5003`) and `allow_local_port_fallback` (bind another port if `5003` is taken). The running node serves ROS topics it subscribes to, including `/current_position` from fusion. `web_debug: true` also POSTs `/decisions` and `/decision_making_data` onward. `remote_host`, `remote_port`, and `poll_hz` are still in the YAML; this node does not declare them. |
| `front_camera_node` | `use_real_sensor`: USB camera when true, upstream JPEG (`camera_remote_host`, `camera_remote_port`, `camera_path`) when false. Capture size, published size (`output_width` / `output_height`), poll rate, and the stop-gate (`capture_when_stopped_only` on `/robot_motion_status`). Real-camera intrinsics must match the published resolution. |
| `ball_detection_node` | Inference on/off, Roboflow URL and model id (`local_inference_url`, `local_model_id`), confidence, infer rate, ball diameters and camera mounting offsets used to turn a bbox into a field position. Inference runs only while `/robot_motion_status` is `stopped`. |
| `pose_estimation_camera` | AprilTag on/off, `/front_camera` input, `tick_hz`, camera offset from the robot origin, optional Kalman filter, and the same stop-gate. Tag geometry comes from `tag_world_map.json`, not from this YAML. |
| `pose_estimation_sensor_fusion` | Camera pose topic, wheel `JointState` topic, output topic `/current_position`, wheel calibration (`counts_per_rev`, count-per-meter, count-diff-per-degree), and the robot-origin offset ahead of the rear axle. |
| `decision_node` | `mode`, `fallback_tick_hz` (the timer that actually runs; `tick_hz` is unused), default speed, target distance filters, `home_colour`, and the four home poses `[x, y, degrees]`. |
| `motion_control_node` | Control rate, PWM and heading gains, rotate/drive tolerances, pause times between rotate and drive, encoder counts per revolution, and the wheel-state publish rate. |
| `timer` | `/time` publish rate. The clock advances only while `/run` is on. |

`exploration_heading_scan_enabled` is declared on ball detection, decision, and motion control. Keep the three copies the same.

Two blocks in `params.yaml` are leftover relative to the code that is running. Under `pose_estimation_sensor_fusion`, the ToF, MPU6050, and IMU-fusion keys are not declared by the fusion node, which only reads camera-pose and wheel-odometry parameters. Under `web_bridge_node`, `remote_host`, `remote_port`, `poll_hz`, and `pose_estimation_enabled` are not declared either. Undeclared keys do not change those nodes.

**`real_camera_intrinsic.yaml`** is the calibrated USB camera model: a 3×3 `camera_matrix`, four fisheye `distortion_coefficients`, `distortion_model: fisheye`, and the image size those numbers belong to (currently 960×540, matching `front_camera_node` `output_width` / `output_height`). Pose estimation and ball detection use it when `use_real_sensor` is true. Recalibrate and replace this file if the camera, lens, or published resolution changes.

**`simulation_camera_intrinsic.json`** is the Webots front-camera model: image size, horizontal FOV, pinhole `K`, and a plumb-bob distortion (zeros in the current file). `unibots.launch.py` passes this path to ball detection and camera pose only when `use_real_sensor` is false.

**`tag_world_map.json`** is the AprilTag field map. It records 24 tags (ids 0–23), units in meters, corner order `BL, BR, TR, TL`, and each tag's four world corners. The file header points at `controllers/estimate_robot_pose/tagWorldMap.mat` as the source. `pose_estimation_camera_node` loads it for PnP. An empty `tag_map_path` parameter means "use this packaged file".

---

## 6) ROS system logic

Package name `unibots`, distro ROS 2 Jazzy, one launch file, nine nodes.

### Nodes

| Executable | Node name | Role |
| --- | --- | --- |
| `web_bridge_node` | `web_bridge_node` | HTTP mirror of pose, balls, time, decisions, and camera images |
| `front_camera_node` | `front_camera_node` | USB or upstream JPEG → `/front_camera` |
| `ball_detection_node` | `ball_detection_node` | Roboflow inference → `/visible_balls` and overlays |
| `pose_estimation_camera_node` | `pose_estimation_camera` | AprilTag PnP → `/current_position_camera` |
| `pose_estimation_sensor_fusion_node` | `pose_estimation_sensor_fusion` | Camera pose + wheel odometry → `/current_position` |
| `decision_node` | `decision_node` | Strategy → `/dynamic_waypoint` and `/decisions` |
| `motion_control_node` | `motion_control_node` | Follow the waypoint, drive motors |
| `timer_node` | `timer` | Simulation clock on `/time` |
| `reset_button_node` | `reset_button_node` | Power button → `/run`, and reset of clock / decisions |

### Topic flow

`/run` is the match gate. `reset_button_node` publishes it from the Pi power button. On reset it also clears `/time`, `/decisions`, `/decision_making_data`, and `/dynamic_waypoints_type`. `timer_node` advances `/time` only while `/run` is on. Camera capture, AprilTag, ball inference, decision, and motion all subscribe to `/run`.

```text
reset_button ── /run ──► timer ── /time
                 │
USB camera, or upstream JPEG
        │
        ▼
front_camera_node ── /front_camera          sensor_msgs/Image
        │
        ├──► pose_estimation_camera
        │       /current_position_camera     absolute AprilTag pose (PoseStamped)
        │       /camera_pose                 camera pose used by ball projection
        │       /camera_pose_history, /pose_history
        │       /processed_image, /num_tags_detected
        │
        └──► ball_detection_node
                /visible_balls               field positions, std_msgs/String
                /ball_pose, /ball_detections
                /ball_detection_image

/current_position_camera + /wheel_joint_states
        │
        ▼
pose_estimation_sensor_fusion ── /current_position
        │
        ▼
decision_node
        /dynamic_waypoint            "(x, y, theta_deg)"
        /dynamic_waypoints_type      pingball / steelball / planned / ...
        /mode
        /decisions, /decision_making_data
        /collision_avoiding_waypoint
        │
        ▼
motion_control_node ── Pi motors
        /waypoint_status             going | reached
        /robot_motion_status         moving | stopped
        /wheel_joint_states          encoder position and velocity
        /exploration_phase
```

Closed loops:

- `/waypoint_status` goes back to `decision_node`, which picks the next target after `reached`.
- `/wheel_joint_states` goes back to fusion, which dead-reckons from the last AprilTag fix: current pose = camera anchor plus wheel motion since that camera timestamp.
- `/robot_motion_status` gates perception. With the current parameters, the camera captures, and ball detection and AprilTag run, only while the status is `stopped`. The robot looks between moves.
- `/dynamic_waypoints_type` and `/exploration_phase` coordinate the optional heading scan across decision, motion, and ball detection.
- Ball projection uses `/camera_pose` and `/camera_pose_history` from the AprilTag node. The fused `/current_position` is what decision and motion follow.

`decision_node` also subscribes to `/radar_sensor`. No node in this package publishes that topic. `web_bridge_node` mirrors `/radar_sensor` onto `/data/radar_sensor` when a message arrives.

`web_bridge_node` is outside the control loop. It subscribes to `/current_position`, `/visible_balls`, `/waypoint_status`, `/time`, `/radar_sensor`, `/pose_history`, `/decisions`, `/decision_making_data`, `/front_camera`, `/processed_image`, and `/ball_detection_image`, and serves them on the local HTTP port (default `5003`). The field viewer and the Webots supervisor read those endpoints when `run_on_pi` is true. `/decisions` and `/decision_making_data` on the viewer redirect to the main page, which embeds the JSON.

### How configuration is applied

1. `unibots.launch.py` reads `share/unibots/config/params.yaml`.
2. Every node receives that file as its parameter set.
3. The launch file then overrides a few keys: `use_real_sensor` is copied from `front_camera_node` onto ball detection, camera pose, and fusion; simulation intrinsics are injected when the real sensor is off; `ROBOFLOW_API_KEY` overrides the detector key when the environment variable is set.
4. Each node calls `declare_parameter` and reads the value. A key present in the YAML but never declared by the node is not used. That includes the leftover ToF / IMU block and the unused web-bridge upstream fields.
5. Root `config.json` is a different file. It configures the Webots supervisor (which branch, file vs web, which Pi). It does not replace `params.yaml`.

---

## 7) Real-time data files

Directory: `controllers/supervisor_controller/real_time_data/`

| File | Contents |
| --- | --- |
| `dynamic_waypoints.txt` | Current target for the main robot |
| `waypoint_status.txt` | `going` / `reached` |
| `current_position.txt` | Main robot pose and heading |
| `obstacle_robot.txt` | Obstacle robot poses |
| `ball_position.txt` | In-field ball positions and types |
| `visible_balls.txt` | Balls visible to the main robot |
| `speed.txt` | Main robot speed (m/s) |
| `time.txt` | Simulation time (seconds) |
| `obstacle_plan.txt` | Obstacle cyclic path |
| `waypoints_history.txt` | Waypoint history (`reached` / `cut`) |
| `radar_sensor.txt` | Radar / ToF-style readings when used |

On the ROS side the same ideas are topics (`/dynamic_waypoint`, `/waypoint_status`, `/current_position`, `/visible_balls`, `/time`) plus the HTTP mirror, rather than these text files.

---

## 8) Other worlds

- `worlds/Position_Estimate_MATLAB.wbt` — `estimate_robot_pose` experiments
- `worlds/moose_demo.wbt` — `moose_path_following` demo
- `worlds/Arena_Development.wbt` — arena structure and materials

---

## 9) Troubleshooting

- The robot does not move as expected: confirm `config.json` → `develop_branch` is `cyc`, `wly`, `xjj`, or `ros`, and that the matching `waypoints_cruise.py` is the one the supervisor starts.
- Decisions look stuck: inspect `dynamic_waypoints.txt` / `waypoint_status.txt`, or the field viewer `/data/decisions`. If `run_on_pi` is true, confirm the Pi address and that `ros2 launch unibots unibots.launch.py` is up. The supervisor and the web bridge must agree on host and port.
- The field viewer does not open: check `controllers/supervisor_controller/html_port.txt` (often `5001`). The ROS mirror is a different port (`params.yaml` `local_port`, often `5003`).
- ROS `cv_bridge` / NumPy errors, or `Address already in use` on the web bridge: see `ROS/ros2_ws/src/unibots/README.md`.
- Pose or ball positions are wrong on the real camera: `real_camera_intrinsic.yaml` image size must match `front_camera_node` `output_width` / `output_height`.

---

## 中文版

UniBots 的 Webots 仿真仓库。主流程是多机器人 supervisor 加外部决策脚本。可选的 ROS 2 Jazzy 栈跑在树莓派上，负责感知、同类策略和电机控制，并可以把决策送回仿真。

## 重要：编辑世界前先 Pause + Reset

Webots 打开 world 后会自动运行。运行中修改场景通常不会被保存。

1. 按 **Pause**（⏸）
2. 按 **Reset Simulation**（⏮）
3. 再编辑并保存

否则修改可能在关闭或重置后丢失。

---

## 1) 环境

- Webots: https://cyberbotics.com
- world 文件版本：`R2025a`（见 `worlds/*.wbt`）
- 可选 ROS 2：Jazzy（见 `ROS/`）

---

## 2) 快速开始

### Step 1. 打开主 world

在 Webots 中打开：

```text
worlds/Decision_making.wbt
```

### Step 2. 选择决策分支

编辑根目录 `config.json`。旧文件回退：`who_is_developing.txt`。

| 字段 | 含义 |
| --- | --- |
| `develop_branch` | `cyc` / `wly` / `xjj` / `ros` |
| `data_flow` | `web`（经 field viewer 的 HTTP）或 `file` |
| `run_on_pi` | 为 `true` 时从树莓派拉取决策 |
| `pi_ip` | 树莓派地址 |
| `camera` / `pose_estimation` | supervisor 和工具使用的功能开关 |
| `default_linear_velocity` / `default_angular_velocity` | supervisor 的默认运动参数 |

分支对应脚本：

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

### Step 2.1 选择策略模式

多数分支的优先级是：`mode.txt` → 命令行 / 环境变量 `MODE` → 该分支 `waypoints_cruise.py` 里的 `DEFAULT_MODE`。

- CYC：例如 `improved_nearest_v3`
- WLY：例如 `developing`
- XJJ：例如 `improved_nearest`
- ROS 脚本和 ROS 节点：`improved_nearest_v3_5`、`planned` 或 `exploration_test`

在树莓派上，实际模式是 `ROS/ros2_ws/src/unibots/config/params.yaml` 里的 `decision_node.mode`。节点调用 `decision_cruise.py` 的 `decide_from_ros_state()`。该参数为空时，规划器回退到 `DEFAULT_MODE`（`improved_nearest_v3_5`）。

### Step 3. 运行仿真

`controllers/supervisor_controller/supervisor_controller.py` 会：

- 管理主机器人和障碍机器人
- 按种子随机摆放球（种子在 `controllers/supervisor_controller/random_seed.txt`）
- 按固定帧间隔调用选中的 `waypoints_cruise.py`
- 通过文件和/或 HTTP 交换状态
- 启动 field viewer（默认 `http://localhost:5001`，端口写在 `controllers/supervisor_controller/html_port.txt`）

### Step 4. 可选：启动 ROS 栈

本机：

```bash
cd ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

或者同步到树莓派、编译并启动：

```bash
./ROS/deploy_and_run_ros_pi.sh
```

`config.json` 里的 `pi_ip` / `run_on_pi` 需要和 ROS web bridge 实际监听的地址、端口一致。

---

## 3) 运行架构（`Decision_making.wbt`）

- 主机器人：`DEF MY_ROBOT`
- 障碍机器人：`DEF OBSTACLE_ROBOT_1/2/3`
- 控制器：`controllers/supervisor_controller/supervisor_controller.py`
- 决策脚本：`decision_making_<branch>/waypoints_cruise.py`

`data_flow: "web"`：

1. Supervisor 把仿真状态发到 field viewer 的 HTTP 接口。
2. 决策脚本读取该状态并回写决策和目标点。`run_on_pi` 为 true 时，由树莓派上的 ROS 栈提供决策。
3. 主机器人以非阻塞方式跟踪当前动态目标点。

`data_flow: "file"`：

1. Supervisor 每帧写入 `controllers/supervisor_controller/real_time_data/*.txt`。
2. 决策脚本读取这些文件，并写回目标点和状态。
3. 主机器人根据 `dynamic_waypoints.txt` 运动。

ROS 包是另一套运行时，不替代 Webots 里的 supervisor。Supervisor 仍然掌管仿真机器人。树莓派栈掌管相机、AprilTag 位姿、球检测、策略和电机，并通过 HTTP 把决策暴露出来，供仿真读取。

---

## 4) 仓库结构

```text
OxbotsSimulator/
├── worlds/
│   ├── Decision_making.wbt          # 主比赛 / 决策联调
│   ├── Arena_Development.wbt        # 场地与材质
│   ├── Position_Estimate_MATLAB.wbt # MATLAB 位姿估计
│   └── moose_demo.wbt               # Webots moose 示例
│
├── controllers/
│   ├── supervisor_controller/       # 比赛 supervisor：机器人、球、数据、可视化
│   ├── other_go_random/             # 简单随机运动
│   ├── moose_path_following/        # C 语言路径跟踪示例
│   ├── estimate_robot_pose/         # MATLAB 相机 / AprilTag 位姿
│   └── ball_detection/              # MATLAB 球检测
│
├── decision_making_cyc/             # CYC 策略
├── decision_making_wly/             # WLY 策略
├── decision_making_xjj/             # XJJ 策略
├── decision_making_ros/             # 与 ROS 对齐的仿真侧策略脚本
│
├── ROS/                             # ROS 2 Jazzy 工作空间与树莓派部署
│   ├── deploy_and_run_ros_pi.sh
│   ├── README.md
│   └── ros2_ws/src/unibots/         # 唯一的 ament_python 包
│
├── pose_estimation/                 # 独立的 AprilTag / 内参工具
├── protos/                          # 场地、球、机器人、AprilTag PROTO
├── textures/                        # AprilTag、场地、机器人纹理
├── tools/
│   ├── field_viewer/                # 实时可视化（server.py）
│   ├── camera_calibration_matlab/
│   ├── camera_calibration_python/
│   ├── auto_photo_capture/
│   ├── testing/
│   └── system_backup/
│
├── benchmarks/                      # UniBots 基准表
├── rpi5/                            # 树莓派电机辅助脚本（run_motor.py）
├── config.json                      # 分支、数据流、树莓派地址、功能开关
└── README.md
```

ROS 包内部：

```text
ROS/ros2_ws/src/unibots/
├── package.xml
├── setup.py
├── launch/unibots.launch.py
├── config/
│   ├── params.yaml
│   ├── real_camera_intrinsic.yaml
│   ├── simulation_camera_intrinsic.json
│   └── tag_world_map.json
├── unibots/                         # 可 import 的 Python 模块
│   ├── web_bridge_node.py
│   ├── front_camera_node.py
│   ├── ball_detection_node.py
│   ├── pose_estimation_camera_node.py
│   ├── pose_estimation_sensor_fusion_node.py
│   ├── decision_node.py
│   ├── decision_cruise.py
│   ├── motion_control_node.py
│   ├── timer_node.py
│   └── reset_button_node.py
└── README.md
```

两层 `unibots` 是 ROS 2 Python 包的常规布局：外层是包（`package.xml`、launch、config），内层是可导入模块。

---

## 5) 关键文件

### 仿真侧

| 文件 | 作用 |
| --- | --- |
| `config.json` | 选择 `develop_branch`、`data_flow`、`run_on_pi`、`pi_ip`、相机/位姿开关和默认速度。Supervisor 启动时读取。 |
| `worlds/Decision_making.wbt` | 比赛场景：主机器人、三台障碍机器人、球、场地。 |
| `controllers/supervisor_controller/supervisor_controller.py` | 推进比赛、写出或提供状态、调用分支脚本，并让 `MY_ROBOT` 跟踪当前目标点。 |
| `controllers/supervisor_controller/random_seed.txt` | 球随机摆放的种子。 |
| `controllers/supervisor_controller/html_port.txt` | Field viewer 端口（常为 `5001`）。 |
| `controllers/supervisor_controller/real_time_data/*.txt` | `data_flow` 为 `file` 时的逐帧状态，也是 supervisor 仍在使用的运动接口文件。 |
| `decision_making_*/waypoints_cruise.py` | 每个分支一个策略进程。必须尽快返回；supervisor 按固定帧间隔调用它。 |
| `decision_making_*/real_time_data/` | 策略自己的状态和调试输出。 |
| `tools/field_viewer/server.py` | 场地、决策和相机的 HTTP 可视化。 |
| `protos/`、`textures/` | world 引用的 Webots PROTO 和贴图。 |
| `pose_estimation/` | 离线 AprilTag 和内参工具，与 ROS 位姿节点分开。 |
| `rpi5/run_motor.py` | ROS launch 之外的小型电机辅助脚本。 |

### ROS 包源码

| 文件 | 作用 |
| --- | --- |
| `ROS/ros2_ws/src/unibots/package.xml` | 包名 `unibots`，构建类型 `ament_python`。依赖 `rclpy`、`std_msgs`、`geometry_msgs`、`sensor_msgs`、`cv_bridge`。 |
| `ROS/ros2_ws/src/unibots/setup.py` | 声明九个 `console_scripts` 入口，并把 `launch/` 和 `config/` 下四个文件安装到包的 share 目录。 |
| `ROS/ros2_ws/src/unibots/launch/unibots.launch.py` | 用 `config/params.yaml` 启动全部节点。把 `front_camera_node.use_real_sensor` 复制给球检测、相机位姿和融合。该标志为 false 时，把球检测和相机位姿指向 `simulation_camera_intrinsic.json`。环境变量 `ROBOFLOW_API_KEY` 若已设置，会覆盖检测器的 key。 |
| `unibots/front_camera_node.py` | USB 相机或上游 JPEG，发布到 `/front_camera`。 |
| `unibots/pose_estimation_camera_node.py` | 用 `/front_camera` 做 AprilTag PnP，发布绝对机器人位姿和调试图。 |
| `unibots/pose_estimation_sensor_fusion_node.py` | 把 AprilTag 位姿和轮式编码器里程融合，发布 `/current_position`。 |
| `unibots/ball_detection_node.py` | 本地 Roboflow 推理，把检测框投影到场地坐标，发布 `/visible_balls`。 |
| `unibots/decision_node.py` | 规划器的 ROS 包装。订阅位姿、球、目标点状态和时间，发布下一目标点和决策内容。 |
| `unibots/decision_cruise.py` | 策略库。模式：`improved_nearest_v3_5`、`planned`、`exploration_test`。节点使用的入口是 `decide_from_ros_state()`。同一文件也可以当脚本跑（`mode.txt` → 命令行 → `MODE` → `DEFAULT_MODE`）。 |
| `unibots/motion_control_node.py` | 目标点跟踪。驱动树莓派电机，发布 `going` / `reached`、`moving` / `stopped`，以及轮子 `JointState`。 |
| `unibots/timer_node.py` | 在 `/run` 为开时发布 `/time`。 |
| `unibots/reset_button_node.py` | 监听树莓派电源键，发布 `/run`，复位时清空时间和决策话题。 |
| `unibots/web_bridge_node.py` | 订阅上述话题，通过 HTTP 提供给 field viewer 和 Webots supervisor。 |
| `ROS/deploy_and_run_ros_pi.sh` | 把工作空间 rsync 到树莓派，执行 `colcon build` 和 `ros2 launch`。也可以拉起本地 Roboflow 推理容器。 |
| `ROS/README.md`、`ROS/ros2_ws/src/unibots/README.md` | 包级编译说明，以及 `cv_bridge` 和端口占用的排查。 |

`decision_making_ros/waypoints_cruise.py` 是 `develop_branch: "ros"` 时仿真侧调用的脚本。ROS 节点不会再去执行这个文件，而是直接 import 包内的 `decision_cruise.py`。

### ROS `config/` 里的文件

这四个文件由 `setup.py` 安装到 `share/unibots/config/`。`colcon build` 之后，节点从包的 share 目录读取它们。

**`params.yaml`** 是 launch 加载进每个节点的唯一 ROS 参数文件。顶层键是节点名，参数写在 `ros__parameters` 下面。

| 段落 | 控制什么 |
| --- | --- |
| `web_bridge_node` | 本地 HTTP 绑定（`local_host` / `local_port`，默认 `0.0.0.0:5003`）和 `allow_local_port_fallback`（`5003` 被占用时改绑其他端口）。正在运行的节点只提供自己订阅到的 ROS 话题，其中包括融合节点的 `/current_position`。`web_debug: true` 时还会把 `/decisions` 和 `/decision_making_data` 再 POST 出去。YAML 里仍有 `remote_host`、`remote_port` 和 `poll_hz`，这个节点没有声明它们。 |
| `front_camera_node` | `use_real_sensor`：true 用 USB 相机，false 用上游 JPEG（`camera_remote_host`、`camera_remote_port`、`camera_path`）。还有采集分辨率、发布分辨率（`output_width` / `output_height`）、轮询频率，以及停车门控（`capture_when_stopped_only`，看 `/robot_motion_status`）。真机内参必须和发布分辨率一致。 |
| `ball_detection_node` | 推理开关、Roboflow 地址和模型（`local_inference_url`、`local_model_id`）、置信度、推理频率、球直径和相机安装偏移（用来把检测框换成场地坐标）。只在 `/robot_motion_status` 为 `stopped` 时推理。 |
| `pose_estimation_camera` | AprilTag 开关、输入 `/front_camera`、`tick_hz`、相机相对机器人原点的偏移、可选卡尔曼滤波，以及同样的停车门控。标签几何不在这个 YAML 里，而在 `tag_world_map.json`。 |
| `pose_estimation_sensor_fusion` | 相机位姿话题、轮子 `JointState` 话题、输出话题 `/current_position`、轮子标定（`counts_per_rev`、每米计数、每度计数差），以及机器人原点相对后轴的前向偏移。 |
| `decision_node` | `mode`、`fallback_tick_hz`（真正在跑的定时器；`tick_hz` 未使用）、默认速度、目标距离过滤、`home_colour`，以及四个 home 位姿 `[x, y, 角度]`。 |
| `motion_control_node` | 控制频率、PWM 和航向增益、旋转/直行容差、旋转与直行之间的停顿、编码器每圈计数、轮子状态发布频率。 |
| `timer` | `/time` 的发布频率。只有 `/run` 为开时时钟才往前走。 |

`exploration_heading_scan_enabled` 在球检测、决策和运动控制里各有一份。三处要保持一致。

`params.yaml` 里有两处和当前代码不一致的遗留参数。`pose_estimation_sensor_fusion` 下面的 ToF、MPU6050 和 IMU 融合键，融合节点没有声明，它只读取相机位姿和轮式里程参数。`web_bridge_node` 下面的 `remote_host`、`remote_port`、`poll_hz` 和 `pose_estimation_enabled` 同样没有被声明。未声明的键不会改变这些节点的行为。

**`real_camera_intrinsic.yaml`** 是 USB 相机标定：3×3 的 `camera_matrix`、四个鱼眼 `distortion_coefficients`、`distortion_model: fisheye`，以及这组数对应的图像尺寸（当前 960×540，与 `front_camera_node` 的 `output_width` / `output_height` 一致）。`use_real_sensor` 为 true 时，位姿估计和球检测使用它。相机、镜头或发布分辨率变了，就需要重新标定并替换这个文件。

**`simulation_camera_intrinsic.json`** 是 Webots 前相机模型：图像尺寸、水平视场角、针孔 `K`，以及 plumb-bob 畸变（当前文件里系数为零）。只有 `use_real_sensor` 为 false 时，`unibots.launch.py` 才把这个路径传给球检测和相机位姿节点。

**`tag_world_map.json`** 是 AprilTag 场地地图。共 24 个标签（id 0–23），单位米，角点顺序 `BL, BR, TR, TL`，每个标签有四个世界坐标角点。文件头注明来源是 `controllers/estimate_robot_pose/tagWorldMap.mat`。`pose_estimation_camera_node` 用它做 PnP。`tag_map_path` 为空表示使用这份随包安装的文件。

---

## 6) ROS 系统逻辑

包名 `unibots`，发行版 ROS 2 Jazzy，一个 launch 文件，九个节点。

### 节点

| 可执行文件 | 节点名 | 作用 |
| --- | --- | --- |
| `web_bridge_node` | `web_bridge_node` | 把位姿、球、时间、决策和相机图像做成 HTTP 镜像 |
| `front_camera_node` | `front_camera_node` | USB 或上游 JPEG → `/front_camera` |
| `ball_detection_node` | `ball_detection_node` | Roboflow 推理 → `/visible_balls` 和叠加图 |
| `pose_estimation_camera_node` | `pose_estimation_camera` | AprilTag PnP → `/current_position_camera` |
| `pose_estimation_sensor_fusion_node` | `pose_estimation_sensor_fusion` | 相机位姿 + 轮式里程 → `/current_position` |
| `decision_node` | `decision_node` | 策略 → `/dynamic_waypoint` 和 `/decisions` |
| `motion_control_node` | `motion_control_node` | 跟踪目标点并驱动电机 |
| `timer_node` | `timer` | `/time` 上的仿真时钟 |
| `reset_button_node` | `reset_button_node` | 电源键 → `/run`，并复位时钟和决策 |

### 话题流

`/run` 是比赛开关。`reset_button_node` 根据树莓派电源键发布它。复位时还会清空 `/time`、`/decisions`、`/decision_making_data` 和 `/dynamic_waypoints_type`。`timer_node` 只在 `/run` 为开时推进 `/time`。相机采集、AprilTag、球检测、决策和运动控制都订阅 `/run`。

```text
reset_button ── /run ──► timer ── /time
                 │
USB 相机，或上游 JPEG
        │
        ▼
front_camera_node ── /front_camera          sensor_msgs/Image
        │
        ├──► pose_estimation_camera
        │       /current_position_camera     AprilTag 绝对位姿 (PoseStamped)
        │       /camera_pose                 球投影使用的相机位姿
        │       /camera_pose_history, /pose_history
        │       /processed_image, /num_tags_detected
        │
        └──► ball_detection_node
                /visible_balls               场地坐标，std_msgs/String
                /ball_pose, /ball_detections
                /ball_detection_image

/current_position_camera + /wheel_joint_states
        │
        ▼
pose_estimation_sensor_fusion ── /current_position
        │
        ▼
decision_node
        /dynamic_waypoint            "(x, y, theta_deg)"
        /dynamic_waypoints_type      pingball / steelball / planned / ...
        /mode
        /decisions, /decision_making_data
        /collision_avoiding_waypoint
        │
        ▼
motion_control_node ── 树莓派电机
        /waypoint_status             going | reached
        /robot_motion_status         moving | stopped
        /wheel_joint_states          编码器位置和速度
        /exploration_phase
```

闭环：

- `/waypoint_status` 回到 `decision_node`。状态变为 `reached` 后，决策选择下一个目标。
- `/wheel_joint_states` 回到融合节点。融合节点从最近一次 AprilTag 修正往前推算：当前位姿 = 相机锚点 + 该相机时间戳之后的轮子运动。
- `/robot_motion_status` 给感知做门控。按当前参数，相机只在状态为 `stopped` 时采集，球检测和 AprilTag 也只在停车时运行。机器人在两次运动之间观察场地。
- `/dynamic_waypoints_type` 和 `/exploration_phase` 在决策、运动和球检测之间协调可选的朝向扫描。
- 球的场地坐标使用 AprilTag 节点的 `/camera_pose` 和 `/camera_pose_history`。决策和运动跟踪的是融合后的 `/current_position`。

`decision_node` 还订阅 `/radar_sensor`。本包里没有节点发布这个话题。有消息到来时，`web_bridge_node` 会把它镜像到 `/data/radar_sensor`。

`web_bridge_node` 不在控制回路里。它订阅 `/current_position`、`/visible_balls`、`/waypoint_status`、`/time`、`/radar_sensor`、`/pose_history`、`/decisions`、`/decision_making_data`、`/front_camera`、`/processed_image` 和 `/ball_detection_image`，并在本地 HTTP 端口提供它们（默认 `5003`）。`run_on_pi` 为 true 时，field viewer 和 Webots supervisor 读这些接口。Viewer 上的 `/decisions` 和 `/decision_making_data` 会重定向到主页，主页里嵌着对应 JSON。

### 配置如何生效

1. `unibots.launch.py` 读取 `share/unibots/config/params.yaml`。
2. 每个节点都收到这一份参数。
3. Launch 再覆盖少数键：把 `front_camera_node` 的 `use_real_sensor` 复制到球检测、相机位姿和融合；真传感器关闭时注入仿真内参；环境变量 `ROBOFLOW_API_KEY` 已设置时覆盖检测器 key。
4. 每个节点用 `declare_parameter` 读取自己声明过的值。YAML 里有、但节点没有声明的键不会被使用。这包括遗留的 ToF / IMU 段，以及 web bridge 里未使用的上游地址字段。
5. 根目录 `config.json` 是另一个文件。它配置的是 Webots supervisor（哪个分支、文件还是网页、哪台树莓派）。它不替代 `params.yaml`。

---

## 7) 实时数据文件

目录：`controllers/supervisor_controller/real_time_data/`

| 文件 | 内容 |
| --- | --- |
| `dynamic_waypoints.txt` | 主机器人当前目标点 |
| `waypoint_status.txt` | `going` / `reached` |
| `current_position.txt` | 主机器人位置和朝向 |
| `obstacle_robot.txt` | 障碍机器人位姿 |
| `ball_position.txt` | 场内球的位置和类型 |
| `visible_balls.txt` | 主机器人可见的球 |
| `speed.txt` | 主机器人速度（m/s） |
| `time.txt` | 仿真时间（秒） |
| `obstacle_plan.txt` | 障碍机器人的循环路径 |
| `waypoints_history.txt` | 目标点历史（`reached` / `cut`） |
| `radar_sensor.txt` | 启用时的雷达 / ToF 读数 |

ROS 侧用话题表达同样的量（`/dynamic_waypoint`、`/waypoint_status`、`/current_position`、`/visible_balls`、`/time`），再加 HTTP 镜像，而不是这些文本文件。

---

## 8) 其他 world

- `worlds/Position_Estimate_MATLAB.wbt`：`estimate_robot_pose` 实验
- `worlds/moose_demo.wbt`：`moose_path_following` 示例
- `worlds/Arena_Development.wbt`：场地结构和材质

---

## 9) 常见排查

- 机器人不按预期运动：确认 `config.json` 的 `develop_branch` 是 `cyc`、`wly`、`xjj` 或 `ros`，并且 supervisor 启动的是对应的 `waypoints_cruise.py`。
- 决策看起来不更新：查看 `dynamic_waypoints.txt` / `waypoint_status.txt`，或 field viewer 的 `/data/decisions`。若 `run_on_pi` 为 true，确认树莓派地址，以及 `ros2 launch unibots unibots.launch.py` 已启动。Supervisor 和 web bridge 的主机、端口要一致。
- Field viewer 打不开：查看 `controllers/supervisor_controller/html_port.txt`（常为 `5001`）。ROS 镜像是另一个端口（`params.yaml` 的 `local_port`，常为 `5003`）。
- ROS `cv_bridge` / NumPy 报错，或 web bridge 提示 `Address already in use`：见 `ROS/ros2_ws/src/unibots/README.md`。
- 真机位姿或球位置不对：`real_camera_intrinsic.yaml` 的图像尺寸必须和 `front_camera_node` 的 `output_width` / `output_height` 一致。
