# UniBots ROS2 ↔ Simulator Bridge

This directory is now organized as a standard ROS2 workspace:

```text
ROS/
└── ros2_ws/
		└── src/
				└── unibots_bridge/
						├── package.xml
						├── setup.py
						├── launch/
						├── config/
						└── unibots_bridge/
```

The inner `unibots_bridge/unibots_bridge` is normal for ROS2 Python packages:
- outer folder: ROS package root (`package.xml`, `setup.py`)
- inner folder: Python module source code

## Package

- `unibots_bridge`

## Build and Run (ROS2 Jazzy)

```bash
cd ~/OxbotsSimulator/ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots_bridge unibots_bridge.launch.py
```

## Parameters

- `src/unibots_bridge/config/params.yaml`

Main defaults:
- `remote_host: 192.168.50.1`
- `remote_port: 5003`
- `local_host: 127.0.0.1`
- `local_port: 5003`

## Sync to Raspberry Pi

From your Mac:

```bash
rsync -avz --delete \
	--exclude '.git' \
	--exclude 'build' --exclude 'install' --exclude 'log' \
	~/OxbotsSimulator/ROS/ros2_ws/ \
	<pi_user>@<pi_ip>:~/OxbotsSimulator/ROS/ros2_ws/
```

Then on Raspberry Pi:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/OxbotsSimulator/ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch unibots_bridge unibots_bridge.launch.py
```
