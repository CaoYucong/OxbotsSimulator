# unibots

ROS2 Jazzy Python package providing:

- `web_bridge_node`: HTTP mirror bridge for simulation data + web viewer endpoints
- `radar_sensor_node`: standalone radar sensor fetcher/publisher (`/radar_sensor`)
- `front_camera_node`: standalone front camera fetcher/publisher (`/front_camera`)
- `ball_detection_node`: Roboflow ball detector + processed image overlay
- `decision_node`: decision node that consumes topics and publishes decisions
- `pose_estimation`: optional pose estimation from front camera

## Notes

- `web_bridge_node` polls upstream endpoints:
	- `http://192.168.50.1:5003/simulation_data`
	- `http://192.168.50.1:5003/data/simulation_data`
- Then serves mirrored data on local endpoints:
	- `http://127.0.0.1:5003/simulation_data`
	- `http://127.0.0.1:5003/data/simulation_data`
- Field viewer main page also includes the live `decisions` and `decision_making_data` JSON views; `/decisions` and `/decision_making_data` now redirect to `/`.
- Cache is updated only when upstream content changes.
- If local mirror port `5003` is occupied, `web_bridge_node` falls back to an available local port by default (`allow_local_port_fallback: true`).
- Front camera JPEG is fetched by `front_camera_node` and published to `/front_camera`.
- Topics published by `web_bridge_node`:
	- `/visible_balls`, `/waypoint_status`, `/time`
	- `/current_position` only when `pose_estimation` is disabled
	- (`/radar_sensor` publishing has been moved out to `radar_sensor_node`)
	- (`/front_camera` publishing has been moved out to `front_camera_node`)
- Topics published by `radar_sensor_node`:
	- `/radar_sensor` (fetched from upstream `/data/simulation_data` field `radar_sensor`)
- Topics published by `front_camera_node`:
	- `/front_camera` (fetched from upstream camera endpoint)
- Topics published by `ball_detection_node`:
	- `/ball_detection_image` (always published; detections overlay on success, low-res error image on failure/no camera)
	- `/ball_pose` (best detected ball 2D pose in normalized image coordinates)
	- `/ball_detections` (all detections JSON, includes confidence)

### Ball detection inference location

- Ball detection uses local inference on Raspberry Pi only.
- Default endpoint is `http://127.0.0.1:9001` with model route `unibot-ball-detection-instant-1/1`.
- Topics published by `pose_estimation`:
	- `/current_position` when `pose_estimation` is enabled
- Topics published by `decision_node`:
	- `/decisions`, `/decision_making_data`
- Topics subscribed by `motion_control_node`:
	- `/current_position`, `/speed`, `/time`

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

## Troubleshooting: `Address already in use` on `web_bridge_node`

If `web_bridge_node` logs `OSError: [Errno 98] Address already in use`, another process is already listening on `local_port` (default `5003`).

- Default behavior is to auto-fallback to a free local port.
- To force strict binding and fail instead, set `allow_local_port_fallback: false` in `config/params.yaml`.
- To keep using `5003`, stop the process currently bound to that port before relaunching.
