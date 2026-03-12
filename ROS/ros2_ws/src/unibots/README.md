# unibots

ROS2 Jazzy Python package providing:

- `web_bridge_node`: HTTP mirror bridge for simulation data + front camera publisher
- `decision_node`: decision node that consumes topics and publishes decisions
- `pose_estimation`: optional pose estimation from front camera

## Notes

- `web_bridge_node` polls upstream endpoints:
	- `http://192.168.50.1:5003/simulation_data`
	- `http://192.168.50.1:5003/data/simulation_data`
- Then serves mirrored data on local endpoints:
	- `http://127.0.0.1:5003/simulation_data`
	- `http://127.0.0.1:5003/data/simulation_data`
- Cache is updated only when upstream content changes.
- Front camera JPEG is fetched from `/front_camera` and published to `/front_camera`.
- Topics published by `web_bridge_node`:
	- `/visible_balls`, `/waypoint_status`, `/time`, `/front_camera`
	- `/current_position` only when `pose_estimation` is disabled
- Topics published by `pose_estimation`:
	- `/current_position` when `pose_estimation` is enabled
- Topics published by `decision_node`:
	- `/decisions`, `/decision_making_data`

## Troubleshooting: `cv_bridge` fails with NumPy 2.x

If you see an error like:

- `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

it usually means ROS Jazzy's binary `cv_bridge` is being loaded together with a user-installed NumPy 2.x from `~/.local`, which is ABI-incompatible.

This package launch file sets:

- `PYTHONNOUSERSITE=1`

for all nodes to avoid importing user-site packages by default.

If the issue still appears, run:

- `python3 -m pip uninstall -y numpy`
- `python3 -m pip install --user "numpy<2"`

Then rebuild and relaunch:

- `cd ~/OxbotsSimulator/ROS/ros2_ws`
- `colcon build --packages-select unibots --symlink-install`
- `source install/setup.bash`
- `ros2 launch unibots unibots.launch.py`
