# UniBots ROS2 ↔ Simulator File Bridge

This directory contains a ROS2 Jazzy package implementing file-based integration only:

- Simulator writes state files in `sim_dir`
- `file_bridge_node` reads files and publishes ROS topics
- `decision_node` computes commands from ROS topics
- `file_bridge_node` writes command files atomically

## Package

- `unibots_file_bridge`

## Topics

Published by `file_bridge_node`:

- `/sim/current_position` (`geometry_msgs/PoseStamped`)
- `/sim/visible_balls` (`std_msgs/String`, JSON payload)
- `/sim/time` (`std_msgs/String`)

Subscribed by `file_bridge_node`:

- `/sim/dynamic_waypoints_cmd` (`std_msgs/String`)
- `/sim/speed_cmd` (`geometry_msgs/Twist`)

## File Contract

Input files (simulator → ROS):

- `current_position.txt`: `x y theta` (radians)
- `visible_balls.txt`: JSON array
- `time.txt`: seconds from match start

Output files (ROS → simulator):

- `dynamic_waypoints.txt`
- `speed.txt`

## Build and Run (ROS2 Jazzy)

```bash
cd ~/OxbotsSimulator/ROS
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots_file_bridge unibots_file_bridge.launch.py
```

## Parameters

Default parameters are in:

- `unibots_file_bridge/config/params.yaml`

Set `sim_dir` to your simulator shared directory (for your mode: `unibots_simulation`).
