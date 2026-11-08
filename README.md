# Oxbots

UniBots software for the physical robot. Perception, decision making, and motor control all run from `ROS/`: one ROS 2 Jazzy Python package, `unibots`, deployed on a Raspberry Pi.

Most other directories are the Webots simulation used before the first physical robot existed. They remain in the tree so the early match logic can still be read. That simulation has not been maintained for a long time and may no longer run. How it was used is at the end of this file.

## Layout

```text
OxbotsSimulator/
├── ROS/                         # The robot system: perception, decisions, motors, parameters, deploy
│   ├── deploy_and_run_ros_pi.sh
│   └── ros2_ws/src/unibots/
├── tools/                       # Calibration, photo capture, field view, SD backup, old sim batch runs
├── rpi5/run_motor.py            # Early keyboard motor jog, not part of the ROS launch
├── worlds/                      # Webots worlds (unmaintained)
├── controllers/                 # Webots controllers, including the supervisor (unmaintained)
├── decision_making_cyc|wly|xjj|ros/   # Strategy scripts for the simulator (unmaintained)
├── pose_estimation/             # Offline AprilTag script aimed at the simulated camera (unmaintained)
├── protos/  textures/           # Webots models and images
├── benchmarks/                  # Early benchmark spreadsheets
└── config.json                  # Branch and data-flow settings the simulator supervisor used to read
```

Changes to robot behavior, sensors, or parameters belong in `ROS/ros2_ws/src/unibots/`. Scripts in `tools/` are helpers and are not started by `ros2 launch`.

---

## ROS system

ROS 2 Jazzy, package `unibots`, build type `ament_python`. One launch file starts nine nodes.

```text
ROS/ros2_ws/src/unibots/
├── package.xml
├── setup.py                         # Nine entry points; installs launch and config
├── launch/unibots.launch.py
├── config/
│   ├── params.yaml                  # Per-node parameters
│   ├── real_camera_intrinsic.yaml   # Real fisheye intrinsics
│   ├── simulation_camera_intrinsic.json
│   └── tag_world_map.json           # Field coordinates of 24 AprilTags
└── unibots/
    ├── front_camera_node.py
    ├── pose_estimation_camera_node.py
    ├── pose_estimation_sensor_fusion_node.py
    ├── ball_detection_node.py
    ├── decision_node.py
    ├── decision_cruise.py           # Strategy library called by the node
    ├── motion_control_node.py
    ├── timer_node.py
    ├── reset_button_node.py
    └── web_bridge_node.py
```

The outer `unibots` directory is the ROS package. The inner `unibots` directory is the importable Python module.

### Build and run

On a machine with ROS 2 Jazzy:

```bash
cd ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

Sync the workspace to the Pi, build, and launch:

```bash
./ROS/deploy_and_run_ros_pi.sh
```

That script can also start a local Roboflow inference container on the Pi (default `127.0.0.1:9001`). The ball detector calls that address.

### Nodes

| Executable | Node name | Role |
| --- | --- | --- |
| `reset_button_node` | `reset_button_node` | Reads the Pi power button, publishes `/run`, and clears the clock and decision topics on reset |
| `timer_node` | `timer` | Publishes `/time` while `/run` is on |
| `front_camera_node` | `front_camera_node` | Publishes the USB camera on `/front_camera` |
| `pose_estimation_camera_node` | `pose_estimation_camera` | AprilTag PnP, publishes the absolute pose |
| `pose_estimation_sensor_fusion_node` | `pose_estimation_sensor_fusion` | AprilTag pose plus wheel odometry, publishes `/current_position` |
| `ball_detection_node` | `ball_detection_node` | Local Roboflow inference, publishes visible balls in field coordinates |
| `decision_node` | `decision_node` | Picks the next waypoint for the active mode |
| `motion_control_node` | `motion_control_node` | Follows the waypoint and drives the motors |
| `web_bridge_node` | `web_bridge_node` | Serves those topics over HTTP for the field page |

### Topic flow

`/run` is the match gate. `reset_button_node` publishes it from the power button. A reset also clears `/time`, `/decisions`, `/decision_making_data`, and `/dynamic_waypoints_type`. `timer_node` advances `/time` only while `/run` is on. The camera, AprilTag, ball detector, decision node, and motion node all subscribe to `/run`.

```text
reset_button ── /run ──► timer ── /time
                 │
USB camera
        │
        ▼
front_camera_node ── /front_camera          sensor_msgs/Image
        │
        ├──► pose_estimation_camera
        │       /current_position_camera     absolute AprilTag pose
        │       /camera_pose                 camera pose used to project balls
        │       /camera_pose_history, /pose_history
        │       /processed_image, /num_tags_detected
        │
        └──► ball_detection_node
                /visible_balls               field coordinates
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
        /dynamic_waypoints_type
        /mode
        /decisions, /decision_making_data
        /collision_avoiding_waypoint
        │
        ▼
motion_control_node ── motors
        /waypoint_status             going | reached
        /robot_motion_status         moving | stopped
        /wheel_joint_states          encoder angle and angular velocity
        /exploration_phase
```

The loops that matter:

- `/waypoint_status` returns to the decision node. After `reached`, the planner chooses the next target.
- `/wheel_joint_states` returns to fusion. The published pose is the latest AprilTag anchor plus wheel motion since that frame's timestamp. The driven wheels sit on the rear axle, and the robot origin is about 40 mm forward of the axle midpoint. Fusion converts odometry into that origin.
- `/robot_motion_status` gates perception. With the current parameters the camera captures, and ball detection and AprilTag run, only while the status is `stopped`. The robot looks between moves.
- Ball positions in the field frame come from `/camera_pose` and `/camera_pose_history`. Decision and motor control follow the fused `/current_position`.
- `/dynamic_waypoints_type` and `/exploration_phase` carry the optional heading-scan phase among decision, motion, and ball detection. The switch `exploration_heading_scan_enabled` is currently off. Change all three copies together.

`decision_node` also subscribes to `/radar_sensor`. No node in this package publishes it. When a message does arrive, `web_bridge_node` exposes it at HTTP `/data/radar_sensor`.

`web_bridge_node` sits outside the control loop. It subscribes to pose, visible balls, waypoint status, time, radar, pose history, decisions, and the three image topics, and serves them on a local port (default `5003`). If `5003` is taken, it binds a free port. With `web_debug: true` it also POSTs the decision JSON onward.

### Strategy

On each tick, `decision_node` calls `decide_from_ros_state()` in `decision_cruise.py`. The mode is `decision_node.mode` in `params.yaml`:

- `improved_nearest_v3_5` (default)
- `planned`
- `exploration_test`

An empty parameter falls back to `DEFAULT_MODE` in `decision_cruise.py`, which is also `improved_nearest_v3_5`. `home_colour` selects the end-of-match home pose from `home_yellow`, `home_green`, `home_purple`, and `home_orange`, each stored as `[x, y, degrees]`.

Target filters live on the same node: field bound `target_field_bound_m`, skip targets closer than `target_min_distance_m` to the robot, and skip retargets closer than `target_retarget_min_distance_m` to the current waypoint. The timer that actually runs is `fallback_tick_hz` (currently 1 Hz). `tick_hz` in the same file is unused.

### How configuration is applied

1. `setup.py` installs the four files in `config/` to `share/unibots/config/`.
2. `unibots.launch.py` passes `params.yaml` to every node.
3. The launch file then overrides a few keys. It copies `front_camera_node.use_real_sensor` onto ball detection, camera pose, and fusion. When that value is false, it passes the simulation intrinsic file to ball detection and camera pose. When `ROBOFLOW_API_KEY` is set in the environment, it overrides the detector key.
4. A node reads only parameters it declares. Extra keys in the YAML have no effect.

What each block of `params.yaml` controls:

| Block | Role |
| --- | --- |
| `front_camera_node` | `use_real_sensor: true` opens the USB camera. Capture size, published size (`output_width` / `output_height`, currently 960×540), poll rate, and whether capture waits until the robot is stopped. Real intrinsics must match the published size. |
| `ball_detection_node` | Inference on/off, Roboflow URL and model (default `http://127.0.0.1:9001`, `unibot-ball-detection/1`), confidence, infer rate, ball diameters, camera height, and the camera offset in the robot frame. Those geometry values turn a bounding box into a field position. Inference runs only while stopped. |
| `pose_estimation_camera` | AprilTag on/off, processing rate `tick_hz`, forward offset of the camera from the robot origin, optional Kalman filter. Tag world coordinates are not here; they are in `tag_world_map.json`. |
| `pose_estimation_sensor_fusion` | Inputs `/current_position_camera` and `/wheel_joint_states`, output `/current_position`. Calibration is encoder counts per revolution, counts per metre of straight travel, left/right count difference per degree of in-place rotation, and the origin's forward offset from the rear axle. |
| `decision_node` | Mode, decision rate, default speed, target filters, home colour, and the four home poses. |
| `motion_control_node` | Control rate, drive and turn PWM, heading tolerances, the looser tolerance used near the target, pauses between rotate and drive, encoder counts per revolution, and the wheel-state publish rate. |
| `timer` | Publish rate of `/time`. The clock advances only while `/run` is on. |
| `web_bridge_node` | The values the running node uses are the local bind address, port fallback, and `web_debug`. |

Two groups of keys are still in the YAML and are not declared by the current code, so they do not change node behavior: the ToF, MPU6050, and IMU-fusion keys under `pose_estimation_sensor_fusion`, and `remote_host`, `remote_port`, `poll_hz`, and `pose_estimation_enabled` under `web_bridge_node`.

The four config files:

- **`params.yaml`** holds every node parameter in the table above. Rates, speeds, home poses, and detection thresholds are changed here, then the launch is started again. With `--symlink-install`, editing this YAML in the source tree and relaunching is enough. A rebuild is unnecessary.
- **`real_camera_intrinsic.yaml`** is the USB camera calibration: a 3×3 camera matrix, four fisheye distortion coefficients, `distortion_model: fisheye`, and the image size those numbers belong to (currently 960×540). Pose estimation and ball detection use it when `use_real_sensor` is true. Replace it after a camera, lens, or published-resolution change, using the calibration scripts under `tools/`.
- **`simulation_camera_intrinsic.json`** is a pinhole model (distortion coefficients are currently zero). The launch file passes it to ball detection and camera pose only when `use_real_sensor` is false. On the physical robot that flag stays true, so this file is not part of the onboard calculation.
- **`tag_world_map.json`** is the field AprilTag map: 24 tags, ids 0–23, units in metres, corner order `BL, BR, TR, TL`. `pose_estimation_camera_node` uses it for PnP. An empty `tag_map_path` means this packaged file.

### What each source file does

| File | Role |
| --- | --- |
| `package.xml` | Package metadata. Depends on `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, and `cv_bridge`. |
| `setup.py` | `console_scripts` entry points, and installation of the launch file plus the four config files. |
| `launch/unibots.launch.py` | Starts the nine nodes, loads `params.yaml`, and applies the intrinsic and Roboflow overrides described above. |
| `front_camera_node.py` | Opens the USB camera, publishes `/front_camera`, and can wait until the robot is stopped. |
| `pose_estimation_camera_node.py` | Finds AprilTags and solves PnP against `tag_world_map.json`. |
| `pose_estimation_sensor_fusion_node.py` | Dead-reckons with the encoders between AprilTag fixes. |
| `ball_detection_node.py` | Sends frames to the local inference server and projects boxes with the camera pose. |
| `decision_node.py` | Collects topics, calls the planner, publishes the waypoint and the decision JSON. |
| `decision_cruise.py` | Strategy implementation. The node enters through `decide_from_ros_state()`. |
| `motion_control_node.py` | Turns to the target heading, drives forward, publishes `reached`, and publishes encoder `JointState`. |
| `timer_node.py` | Match clock. |
| `reset_button_node.py` | Power-button start, stop, and reset. Needs `evdev`. |
| `web_bridge_node.py` | HTTP mirror of the topics. |
| `ROS/deploy_and_run_ros_pi.sh` | Rsyncs the workspace from the development machine to the Pi, builds, and launches. |
| `ROS/README.md`, `ROS/ros2_ws/src/unibots/README.md` | Package notes, plus `cv_bridge` / NumPy and port-in-use troubleshooting. |

`rpi5/run_motor.py` is an earlier script that toggles GPIO from the keyboard, one wheel at a time. Motion on the robot now goes through `motion_control_node`. Do not run both against the same pins.

### Troubleshooting

- Pose or ball positions look wrong: the image size in `real_camera_intrinsic.yaml` must equal `front_camera_node` `output_width` / `output_height`.
- Ball detection stays empty: the inference server on the Pi at `127.0.0.1:9001` has to be running, and the robot has to be `stopped`. Inference does not run while the robot is moving.
- `cv_bridge` fails under NumPy 2, or the web bridge reports the address is already in use: see `ROS/ros2_ws/src/unibots/README.md`.

---

## tools

These scripts are not part of `ros2 launch`. Calibration and the SD-card backup are still for the physical robot. `testing/` only drives the old Webots simulator described at the end of this file.

### Camera calibration

The fisheye intrinsics on the robot come from this set. Print a board, capture images, calibrate, and copy the resulting YAML over `ROS/ros2_ws/src/unibots/config/real_camera_intrinsic.yaml`.

| File | Role |
| --- | --- |
| `tools/camera_calibration_python/generate_chessboard.py` | Writes an A4 landscape chessboard image, defaulting to a 9×6 inner-corner grid. Print at actual size. Measure a square with a ruler and pass that length in metres as `--square`. |
| `tools/camera_calibration_python/calibrate.py` | Pinhole calibration. Takes an image directory and the inner-corner counts, and writes a ROS-style `camera_info.yaml`. |
| `tools/camera_calibration_python/calibrate_fisheye.py` | Fisheye calibration. Writes a YAML with `camera_matrix`, `distortion_coefficients`, and the image size. The wide-angle lens on the robot uses this script, not `calibrate.py`. |
| `tools/auto_photo_capture/auto_photo_capture.py` | Opens a preview and saves 1920×1080 JPEGs on an interval into `photos/` next to the script. Press `q` or `ESC` in the preview to stop. Use it to collect calibration images. |
| `tools/camera_calibration_matlab/run_calibration.m` | Full MATLAB calibration from AprilTags (tag36h11): detect corners, estimate intrinsics and extrinsics, plot reprojection error, and write undistorted images. |
| `tools/camera_calibration_matlab/Generate_Apriltag_Board.m` | Builds a 5×8 tag36h11 calibration board and writes `aprilTagBoard.png`. |
| `tools/camera_calibration_matlab/helper*.m`, `initiation.m` | Corner detection and coordinate conversion used by the two scripts above. |

### Field view

`tools/field_viewer/server.py` is an HTTP page with no third-party dependencies. A browser shows the field, robots, balls, and decision JSON. The port comes from `controllers/supervisor_controller/html_port.txt`, default `5001`. The server reads the root `config.json`. With `run_on_pi` true it requests data from `pi_ip`. Otherwise it reads the text files the simulator supervisor writes under `real_time_data/`. The page can still be started. The simulator feed it falls back to is part of the unmaintained section at the end.

### Raspberry Pi SD card

`tools/system_backup/rpi_img.sh` backs up a Pi SD card to a compressed image on macOS, or writes an image back. Backup is `-b -d /dev/diskN`. Restore is `-r -d /dev/diskN -f <image>`. Restore erases the target disk. Check the disk number with `diskutil list` first.

### Simulator batch runs

These only loop the old Webots setup and are as unmaintained as the simulator:

- `tools/testing/webots_auto_loop_crossplatform.py` starts Webots repeatedly, waits for the supervisor exit text, then starts the next run. It can sweep modes, avoidance levels, and random seeds into a CSV.
- `tools/testing/run_parallel_3.sh` and `run_parallel_4.sh` copy the repo several times and run that loop in parallel.

---

## Simulator (unmaintained; it may no longer run)

Before the first physical robot was built, match logic was developed in Webots. These directories are what remains: `worlds/`, `controllers/`, `decision_making_cyc/`, `decision_making_wly/`, `decision_making_xjj/`, `decision_making_ros/`, `pose_estimation/`, `protos/`, `textures/`, and the root `config.json`.

This simulation has not been maintained for a long time. The worlds, the supervisor, and the strategy scripts were not updated alongside the later ROS nodes. Opening it as described below can miss files, disagree with current interfaces, or leave the robot unmoved. For how the robot decides and drives today, use `ROS/`.

What the old workflow was:

1. Open `worlds/Decision_making.wbt` in Webots R2025a. The main robot is `MY_ROBOT`, with three obstacle robots. The controller is `controllers/supervisor_controller/supervisor_controller.py`.
2. Before editing a world, press Pause, then Reset Simulation. Edits made while the simulation is running are often not saved.
3. `develop_branch` in the root `config.json` selects a strategy script. `cyc`, `wly`, `xjj`, and `ros` map to `decision_making_<branch>/waypoints_cruise.py`. `data_flow: web` uses the field viewer's HTTP API. `data_flow: file` uses `controllers/supervisor_controller/real_time_data/*.txt`. With `run_on_pi` true, the supervisor fetches decisions from the Pi instead.
4. The ball-randomization seed is `controllers/supervisor_controller/random_seed.txt`. Files that workflow used include `dynamic_waypoints.txt`, `waypoint_status.txt`, `current_position.txt`, `ball_position.txt`, `visible_balls.txt`, and `time.txt`.
5. `pose_estimation/pose_estimation.py` runs AprilTag offline on the simulated front-camera image. `pose_estimation/generate_camera_intrinsics.py` derives a theoretical intrinsic matrix from the Webots camera field of view. That is a different calibration from `real_camera_intrinsic.yaml` on the robot.

Other worlds: `worlds/Arena_Development.wbt` is the arena materials, `worlds/Position_Estimate_MATLAB.wbt` goes with `controllers/estimate_robot_pose/`, and `worlds/moose_demo.wbt` goes with `controllers/moose_path_following/`. `controllers/ball_detection/` is the MATLAB ball detector. `controllers/other_go_random/` is a random-motion controller.

---

# 中文版

UniBots 实物机器人的软件仓库。机器人上运行的感知、决策和电机控制都在 `ROS/` 里，是一个 ROS 2 Jazzy 的 Python 包 `unibots`，部署在树莓派上。

仓库里其余大部分目录是第一台实物机器人搭起来之前用的 Webots 仿真。那些目录还留在仓库里，方便对照早期比赛逻辑，但仿真侧已经很久没有维护，不保证还能跑。仿真的用法放在本文最后。

## 项目结构

```text
OxbotsSimulator/
├── ROS/                         # 机器人系统。感知、决策、电机、参数、部署都在这里
│   ├── deploy_and_run_ros_pi.sh
│   └── ros2_ws/src/unibots/
├── tools/                       # 标定、拍照、场地查看、SD 卡备份、早期仿真批测
├── rpi5/run_motor.py            # 早期手动键盘控电机，不在 ROS launch 里
├── worlds/                      # Webots 场景（未维护）
├── controllers/                 # Webots 控制器，含 supervisor（未维护）
├── decision_making_cyc|wly|xjj|ros/   # 仿真用的策略脚本（未维护）
├── pose_estimation/             # 仿真相机的离线 AprilTag 脚本（未维护）
├── protos/  textures/           # Webots 模型和贴图
├── benchmarks/                  # 早期基准表
└── config.json                  # 当年仿真 supervisor 读的分支和数据流配置
```

改机器人行为、传感器或参数，只需要看 `ROS/ros2_ws/src/unibots/`。`tools/` 是配套脚本，不参与 `ros2 launch`。

---

## ROS 系统

发行版 ROS 2 Jazzy，包名 `unibots`，构建类型 `ament_python`。一个 launch 文件拉起九个节点。

```text
ROS/ros2_ws/src/unibots/
├── package.xml
├── setup.py                         # 九个节点入口，并安装 launch 和 config
├── launch/unibots.launch.py
├── config/
│   ├── params.yaml                  # 各节点参数
│   ├── real_camera_intrinsic.yaml   # 真机鱼眼内参
│   ├── simulation_camera_intrinsic.json
│   └── tag_world_map.json           # 24 个 AprilTag 的场地坐标
└── unibots/
    ├── front_camera_node.py
    ├── pose_estimation_camera_node.py
    ├── pose_estimation_sensor_fusion_node.py
    ├── ball_detection_node.py
    ├── decision_node.py
    ├── decision_cruise.py           # 策略库，节点直接调用
    ├── motion_control_node.py
    ├── timer_node.py
    ├── reset_button_node.py
    └── web_bridge_node.py
```

外层 `unibots` 是 ROS 包，内层 `unibots` 是可 import 的 Python 模块。

### 编译和启动

在装有 ROS 2 Jazzy 的机器上：

```bash
cd ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

把工作空间同步到树莓派、编译并启动：

```bash
./ROS/deploy_and_run_ros_pi.sh
```

这个脚本也可以在树上拉起本地 Roboflow 推理容器（默认 `127.0.0.1:9001`）。球检测节点默认请求这个地址。

### 节点

| 可执行文件 | 节点名 | 作用 |
| --- | --- | --- |
| `reset_button_node` | `reset_button_node` | 读树莓派电源键，发布 `/run`；复位时清空时钟和决策话题 |
| `timer_node` | `timer` | `/run` 为开时发布 `/time` |
| `front_camera_node` | `front_camera_node` | USB 相机发布到 `/front_camera` |
| `pose_estimation_camera_node` | `pose_estimation_camera` | AprilTag PnP，发布绝对位姿 |
| `pose_estimation_sensor_fusion_node` | `pose_estimation_sensor_fusion` | AprilTag 位姿加轮式里程，发布 `/current_position` |
| `ball_detection_node` | `ball_detection_node` | 本地 Roboflow 推理，发布场地坐标上的可见球 |
| `decision_node` | `decision_node` | 按模式选下一目标点 |
| `motion_control_node` | `motion_control_node` | 跟踪目标点并驱动电机 |
| `web_bridge_node` | `web_bridge_node` | 把上述话题做成 HTTP，给场地页面看 |

### 话题流

`/run` 是比赛开关。`reset_button_node` 根据电源键发布它。复位时还会清空 `/time`、`/decisions`、`/decision_making_data` 和 `/dynamic_waypoints_type`。`timer_node` 只在 `/run` 为开时推进 `/time`。相机、AprilTag、球检测、决策和运动控制都订阅 `/run`。

```text
reset_button ── /run ──► timer ── /time
                 │
USB 相机
        │
        ▼
front_camera_node ── /front_camera          sensor_msgs/Image
        │
        ├──► pose_estimation_camera
        │       /current_position_camera     AprilTag 绝对位姿
        │       /camera_pose                 球投影用的相机位姿
        │       /camera_pose_history, /pose_history
        │       /processed_image, /num_tags_detected
        │
        └──► ball_detection_node
                /visible_balls               场地坐标
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
        /dynamic_waypoints_type
        /mode
        /decisions, /decision_making_data
        /collision_avoiding_waypoint
        │
        ▼
motion_control_node ── 电机
        /waypoint_status             going | reached
        /robot_motion_status         moving | stopped
        /wheel_joint_states          编码器角度和角速度
        /exploration_phase
```

几条闭环：

- `/waypoint_status` 回到决策。状态变成 `reached` 之后，决策再选下一个目标。
- `/wheel_joint_states` 回到融合。当前位姿 = 最近一次 AprilTag 锚点 + 该帧时间戳之后的轮子位移。轮子在后轴，机器人原点在后轴中点前方约 40 mm，融合节点按这个几何把里程换到原点。
- `/robot_motion_status` 给感知做门控。按现在的参数，相机只在 `stopped` 时采集，球检测和 AprilTag 也只在停车时跑。机器人在两次运动之间看场地。
- 球的场地坐标用的是 `/camera_pose` 和 `/camera_pose_history`。决策和电机跟踪的是融合后的 `/current_position`。
- `/dynamic_waypoints_type` 和 `/exploration_phase` 在决策、运动和球检测之间传递可选的朝向扫描阶段。这个功能的开关 `exploration_heading_scan_enabled` 目前是关的，三处参数要一起改。

`decision_node` 还订阅 `/radar_sensor`。当前包里没有节点发布它。有消息时，`web_bridge_node` 会把它放到 HTTP 的 `/data/radar_sensor`。

`web_bridge_node` 不在控制回路里。它订阅位姿、可见球、目标点状态、时间、雷达、位姿历史、决策和三路图像，在本地端口提供 HTTP（默认 `5003`）。`5003` 被占用时，默认改绑一个空闲端口。`web_debug: true` 时，还会把决策 JSON 再 POST 出去。

### 策略

`decision_node` 每个周期调用 `decision_cruise.py` 里的 `decide_from_ros_state()`。模式由 `params.yaml` 的 `decision_node.mode` 决定：

- `improved_nearest_v3_5`（默认）
- `planned`
- `exploration_test`

参数为空时用 `decision_cruise.py` 里的 `DEFAULT_MODE`（同样是 `improved_nearest_v3_5`）。`home_colour` 选择终局回家的颜色，对应四个 `[x, y, 角度]`：`home_yellow`、`home_green`、`home_purple`、`home_orange`。

目标过滤也在这个节点的参数里：场地边界 `target_field_bound_m`、离机器人太近则跳过 `target_min_distance_m`、离当前目标点太近则不改目标 `target_retarget_min_distance_m`。决策定时器用的是 `fallback_tick_hz`（当前 1 Hz）。同文件里的 `tick_hz` 没有被定时器使用。

### 配置怎么生效

1. `setup.py` 把 `config/` 里的四个文件安装到 `share/unibots/config/`。
2. `unibots.launch.py` 把 `params.yaml` 交给每一个节点。
3. Launch 再覆盖少数键：把 `front_camera_node.use_real_sensor` 复制给球检测、相机位姿和融合；这个值为 false 时，把仿真内参路径传给球检测和相机位姿；环境变量 `ROBOFLOW_API_KEY` 已设置时覆盖检测器 key。
4. 节点只读取自己 `declare_parameter` 过的键。YAML 里多出来的键不会生效。

`params.yaml` 各段在控制什么：

| 段 | 作用 |
| --- | --- |
| `front_camera_node` | `use_real_sensor: true` 打开 USB 相机。采集分辨率、发布分辨率（`output_width` / `output_height`，当前 960×540）、轮询频率，以及是否只在停车时采集。真机内参必须和发布分辨率一致。 |
| `ball_detection_node` | 推理开关、Roboflow 地址和模型（默认 `http://127.0.0.1:9001`，`unibot-ball-detection/1`）、置信度、推理频率、球直径、相机高度和相机在机器人坐标系下的偏移。这些几何量用来把检测框换成场地坐标。推理只在停车时跑。 |
| `pose_estimation_camera` | AprilTag 开关、处理频率 `tick_hz`、相机相对机器人原点的前向偏移、可选卡尔曼滤波。标签的世界坐标不在这里，而在 `tag_world_map.json`。 |
| `pose_estimation_sensor_fusion` | 输入 `/current_position_camera` 和 `/wheel_joint_states`，输出 `/current_position`。标定包括每圈编码器计数、直线每米计数、原地旋转每度的左右轮计数差，以及原点相对后轴的前向偏移。 |
| `decision_node` | 模式、决策频率、默认速度、目标过滤、回家颜色和四个 home 位姿。 |
| `motion_control_node` | 控制频率、直行和转向 PWM、航向容差、接近目标时放宽的容差、旋转与直行之间的停顿、编码器每圈计数、轮子状态发布频率。 |
| `timer` | `/time` 的发布频率。时钟只在 `/run` 为开时前进。 |
| `web_bridge_node` | 正在生效的是本地绑定地址、端口回退和 `web_debug`。 |

有两处参数还写在 YAML 里，但当前代码没有声明，因此不会改变节点行为：`pose_estimation_sensor_fusion` 下面的 ToF、MPU6050 和 IMU 融合项；`web_bridge_node` 下面的 `remote_host`、`remote_port`、`poll_hz` 和 `pose_estimation_enabled`。

四个配置文件：

- **`params.yaml`**：上面这张表的全部节点参数。改频率、速度、回家点、检测阈值，改这个文件，然后重新 launch。`--symlink-install` 时，改源码树里的这份 YAML 后重新 launch 即可，不必再 build。
- **`real_camera_intrinsic.yaml`**：USB 相机标定结果。3×3 相机矩阵、四个鱼眼畸变系数、`distortion_model: fisheye`，以及对应图像尺寸（当前 960×540）。`use_real_sensor` 为 true 时，位姿和球检测用它。换相机、镜头或发布分辨率后要重新标定并替换这个文件。
- **`simulation_camera_intrinsic.json`**：一份针孔模型（当前畸变系数为零）。只有 `use_real_sensor` 为 false 时，launch 才把它传给球检测和相机位姿。实物机器人保持 `use_real_sensor: true`，这份文件不参与真机计算。
- **`tag_world_map.json`**：场地 AprilTag。24 个标签，id 0–23，单位米，角点顺序 `BL, BR, TR, TL`。`pose_estimation_camera_node` 用它做 PnP。`tag_map_path` 留空就是用这份随包文件。

### 源码里各文件做什么

| 文件 | 作用 |
| --- | --- |
| `package.xml` | 包元数据。依赖 `rclpy`、`std_msgs`、`geometry_msgs`、`sensor_msgs`、`cv_bridge`。 |
| `setup.py` | `console_scripts` 入口，并把 launch 和四个 config 文件装进 share。 |
| `launch/unibots.launch.py` | 启动九个节点，加载 `params.yaml`，并按上一节覆盖内参和 Roboflow key。 |
| `front_camera_node.py` | 打开 USB 相机，按停车状态决定是否出图，发布 `/front_camera`。 |
| `pose_estimation_camera_node.py` | 在图像里找 AprilTag，对照 `tag_world_map.json` 做 PnP。 |
| `pose_estimation_sensor_fusion_node.py` | 用编码器在两次 AprilTag 修正之间推算位姿。 |
| `ball_detection_node.py` | 把图像送给本地推理服务，再用相机位姿把框投影到场地。 |
| `decision_node.py` | 收话题、调用策略、发布目标点和决策 JSON。 |
| `decision_cruise.py` | 策略实现。节点走 `decide_from_ros_state()`。 |
| `motion_control_node.py` | 先转到目标航向，再直行，到点后发 `reached`，并发布编码器 `JointState`。 |
| `timer_node.py` | 比赛时钟。 |
| `reset_button_node.py` | 电源键启停和复位。依赖 `evdev`。 |
| `web_bridge_node.py` | 话题到 HTTP 的镜像。 |
| `ROS/deploy_and_run_ros_pi.sh` | 从开发机 rsync 工作空间到树莓派，编译并 launch。 |
| `ROS/README.md`、`ROS/ros2_ws/src/unibots/README.md` | 包级说明，以及 `cv_bridge` / NumPy、端口占用的排查。 |

`rpi5/run_motor.py` 是更早的键盘直接拨 GPIO 的脚本，四个轮子单独点动。现在车上的运动走 `motion_control_node`，不要和它同时占同一组引脚。

### 排查

- 位姿或球的位置不对：`real_camera_intrinsic.yaml` 的宽高要等于 `front_camera_node` 的 `output_width` / `output_height`。
- 球检测没有结果：确认树上 `127.0.0.1:9001` 的推理服务在跑，并且机器人处于 `stopped`。推理在运动过程中不会跑。
- `cv_bridge` 报 NumPy 2 不兼容，或 web bridge 报地址已被占用：看 `ROS/ros2_ws/src/unibots/README.md`。

---

## tools

这些脚本不在 `ros2 launch` 里。标定和备份仍然是给实物机器人用的。`testing/` 只驱动当年的 Webots，依赖文末那套未维护的仿真。

### 相机标定

真机鱼眼内参来自这组脚本。流程是：打印棋盘，拍照，标定，把得到的 YAML 抄到 `ROS/ros2_ws/src/unibots/config/real_camera_intrinsic.yaml`。

| 文件 | 作用 |
| --- | --- |
| `tools/camera_calibration_python/generate_chessboard.py` | 生成一张 A4 横向棋盘图，默认对应内角点 9×6。打印时用实际大小。打印后用尺子量方格边长，把米制长度传给标定脚本的 `--square`。 |
| `tools/camera_calibration_python/calibrate.py` | 普通针孔模型。输入一个图片目录和棋盘内角点数，写出 ROS 风格的 `camera_info.yaml`。 |
| `tools/camera_calibration_python/calibrate_fisheye.py` | 鱼眼模型，输出同样带 `camera_matrix`、`distortion_coefficients` 和图像尺寸的 YAML。车上的广角镜头用这一份，而不是 `calibrate.py`。 |
| `tools/auto_photo_capture/auto_photo_capture.py` | 打开相机预览，按间隔把 1920×1080 的 JPEG 存进该目录下的 `photos/`。预览窗口里按 `q` 或 `ESC` 停止。用来收集标定图片。 |
| `tools/camera_calibration_matlab/run_calibration.m` | 用 AprilTag（tag36h11）做整套 MATLAB 标定：检测角点、估计内外参、看重投影误差、导出去畸变图。 |
| `tools/camera_calibration_matlab/Generate_Apriltag_Board.m` | 生成 5×8 的 tag36h11 标定板图片 `aprilTagBoard.png`。 |
| `tools/camera_calibration_matlab/helper*.m`、`initiation.m` | 上面两个脚本用的角点检测和坐标转换。 |

### 场地查看

`tools/field_viewer/server.py` 是一个不依赖第三方库的 HTTP 页面，浏览器里画场地、机器人、球和决策 JSON。端口读 `controllers/supervisor_controller/html_port.txt`，默认 `5001`。它读根目录 `config.json`：`run_on_pi` 为 true 时向 `pi_ip` 要数据，否则读仿真 supervisor 写在 `real_time_data/` 里的文件。页面本身还能开；它依赖的仿真数据源属于文末未维护的部分。

### 树莓派 SD 卡

`tools/system_backup/rpi_img.sh` 在 macOS 上把树莓派 SD 卡备份成压缩镜像，或从镜像写回。备份是 `-b -d /dev/diskN`，恢复是 `-r -d /dev/diskN -f 镜像`。恢复会清掉目标盘。运行前用 `diskutil list` 核对盘号。

### 仿真批测

只用于当年的 Webots 循环跑分，现在随仿真一起停更：

- `tools/testing/webots_auto_loop_crossplatform.py` 反复启动 Webots，等到 supervisor 退出文字后再开下一轮，并可按模式、避障档和随机种子写 CSV。
- `tools/testing/run_parallel_3.sh` 和 `run_parallel_4.sh` 复制多份仓库，并行跑上面的循环。

---

## 仿真（久未维护，不保证还能用）

第一台实物机器人做好之前，比赛逻辑是在 Webots 里调的。下面这些目录从那时候留到现在：`worlds/`、`controllers/`、`decision_making_cyc/`、`decision_making_wly/`、`decision_making_xjj/`、`decision_making_ros/`、`pose_estimation/`、`protos/`、`textures/`，以及根目录 `config.json`。

这套仿真已经很久没有维护。World、supervisor 和策略脚本没有跟着后来的 ROS 节点一起改。按下面的步骤打开，可能会缺文件、对不上接口，或者机器人根本不动。需要看当前机器人怎么决策、怎么动，以 `ROS/` 为准。

当时的用法是：

1. 用 Webots R2025a 打开 `worlds/Decision_making.wbt`。主机器人是 `MY_ROBOT`，另外有三台障碍机器人。控制器是 `controllers/supervisor_controller/supervisor_controller.py`。
2. 编辑 world 之前先按 Pause，再按 Reset Simulation，否则运行中的修改经常存不下来。
3. 根目录 `config.json` 的 `develop_branch` 选择策略脚本：`cyc`、`wly`、`xjj`、`ros` 分别对应 `decision_making_<分支>/waypoints_cruise.py`。`data_flow` 为 `web` 时走 field viewer 的 HTTP，为 `file` 时走 `controllers/supervisor_controller/real_time_data/*.txt`。`run_on_pi` 为 true 时，supervisor 改为向树莓派要决策。
4. 球的随机种子在 `controllers/supervisor_controller/random_seed.txt`。常用文本包括 `dynamic_waypoints.txt`、`waypoint_status.txt`、`current_position.txt`、`ball_position.txt`、`visible_balls.txt`、`time.txt`。
5. `pose_estimation/pose_estimation.py` 从仿真的前相机图做离线 AprilTag。`pose_estimation/generate_camera_intrinsics.py` 按 Webots 相机的视场角推一份理论内参。这和车上 `real_camera_intrinsic.yaml` 不是同一份标定。

其他 world：`worlds/Arena_Development.wbt` 是场地材质，`worlds/Position_Estimate_MATLAB.wbt` 配合 `controllers/estimate_robot_pose/`，`worlds/moose_demo.wbt` 配合 `controllers/moose_path_following/`。`controllers/ball_detection/` 是 MATLAB 球检测，`controllers/other_go_random/` 是一个随机运动控制器。

---
