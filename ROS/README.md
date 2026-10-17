# UniBots ROS2 ↔ Simulator Bridge

ROS2 Jazzy workspace that bridges the Webots simulator (or real hardware) to perception, decision-making, and motion control.

```text
ROS/
├── deploy_and_run_ros_pi.sh     # rsync + colcon build + launch on Raspberry Pi
├── GPIO-Pinout-Diagram-2.png    # Pi GPIO reference
└── ros2_ws/
    └── src/
        └── unibots/             # single ament_python package
            ├── package.xml
            ├── setup.py
            ├── launch/unibots.launch.py
            ├── config/          # params.yaml, camera intrinsics, tag map
            └── unibots/         # Python module (nodes)
```

The nested `unibots/unibots` layout is normal for ROS2 Python packages:

- outer folder: package root (`package.xml`, `setup.py`, launch, config)
- inner folder: importable Python module

---

## Package

- Name: `unibots`
- Distro: ROS2 **Jazzy**
- Build type: `ament_python`

### Nodes (launched by `unibots.launch.py`)

| Executable | Role |
| --- | --- |
| `web_bridge_node` | HTTP mirror of simulation / viewer data; aggregates ROS topics for the field viewer |
| `front_camera_node` | USB camera or upstream JPEG → `/front_camera` |
| `ball_detection_node` | Local Roboflow inference → overlays, `/visible_balls`, `/ball_pose` |
| `pose_estimation_camera_node` | AprilTag PnP → `/current_position_camera`, `/processed_image` |
| `pose_estimation_sensor_fusion_node` | Camera + wheel odometry / IMU / ToF → `/current_position`, `/radar_sensor` |
| `decision_node` | Strategy (`decision_cruise`) → `/dynamic_waypoint`, `/decisions`, `/decision_making_data` |
| `motion_control_node` | Follow waypoints, drive motors, publish `/waypoint_status`, `/wheel_joint_states` |
| `timer_node` | Simulation clock on `/time` (gated by `/run`) |
| `reset_button_node` | Hardware button: reset clock / decisions / run state |

### Typical topic flow

```text
front_camera ──► ball_detection ──► /visible_balls ──┐
              └► pose_estimation_camera ──► /current_position_camera
                        │
                        ▼
              pose_estimation_sensor_fusion ──► /current_position
                        │
                        ▼
              decision_node ──► /dynamic_waypoint ──► motion_control_node
                        │                                    │
                        └── /decisions ──► web_bridge ◄──────┘ /waypoint_status
```

---

## Build and run (local)

```bash
cd ~/OxbotsSimulator/ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

## Deploy to Raspberry Pi

From the Mac (preferred helper):

```bash
./ROS/deploy_and_run_ros_pi.sh          # foreground
./ROS/deploy_and_run_ros_pi.sh --detach # background log on Pi
```

Manual sync:

```bash
rsync -avz --delete \
  --exclude '.git' \
  --exclude 'build' --exclude 'install' --exclude 'log' \
  ~/OxbotsSimulator/ROS/ros2_ws/ \
  <pi_user>@<pi_ip>:~/OxbotsSimulator/ROS/ros2_ws/
```

Then on the Pi:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/OxbotsSimulator/ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

---

## Parameters

Primary file: `ros2_ws/src/unibots/config/params.yaml`

Also shipped:

- `real_camera_intrinsic.yaml` — real USB camera intrinsics
- `simulation_camera_intrinsic.json` — sim camera intrinsics (when `use_real_sensor: false`)
- `tag_world_map.json` — AprilTag world map

Common defaults (check the YAML for the live values):

- Upstream simulator: `remote_host` / `remote_port` (often `192.168.50.2:5003`)
- Local HTTP mirror: `local_host` / `local_port` (`0.0.0.0:5003`)
- Decision mode: `decision_node.mode` (e.g. `improved_nearest_v3_5`)
- `front_camera_node.use_real_sensor` — USB vs upstream JPEG

Root `config.json` (`develop_branch`, `pi_ip`, `run_on_pi`, …) selects how the **Webots supervisor** talks to this stack; keep Pi IP and ports consistent.

---

## Related repo pieces

- Simulator bridge consumer: `decision_making_ros/waypoints_cruise.py`
- Field viewer: `tools/field_viewer/`
- Standalone pose helpers: `pose_estimation/`
- Node-level troubleshooting (`cv_bridge` / NumPy, port in use): [`ros2_ws/src/unibots/README.md`](ros2_ws/src/unibots/README.md)

---

## 中文版

面向 Webots 仿真（或真机）的 ROS2 Jazzy 桥接工作区：感知、决策、运动控制一体。

```text
ROS/
├── deploy_and_run_ros_pi.sh     # 同步 / 编译 / 在树莓派上启动
├── GPIO-Pinout-Diagram-2.png
└── ros2_ws/src/unibots/         # ament_python 包（节点 + launch + 配置）
```

### 包与节点

- 包名：`unibots`；发行版：ROS2 **Jazzy**
- 启动文件：`launch/unibots.launch.py`
- 节点职责见上方英文表格（`web_bridge`、相机、球检测、位姿融合、决策、运动控制、定时器、复位按键）

### 本地编译运行

```bash
cd ~/OxbotsSimulator/ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots unibots.launch.py
```

### 部署到树莓派

```bash
./ROS/deploy_and_run_ros_pi.sh
```

参数文件：`ros2_ws/src/unibots/config/params.yaml`。与仿真联调时，请保持根目录 `config.json` 中的 `pi_ip` / `run_on_pi` 与本栈端口一致。

更细的节点说明与排错：[`ros2_ws/src/unibots/README.md`](ros2_ws/src/unibots/README.md)。
