# Raspberry Pi Web Runtime (No ROS)

This runtime migrates the ROS nodes to a pure Python Web stack:

- `web_bridge`: mirrors two upstream sources
  - `/data/simulation_data`
  - `/data/front_camera`
- `decision_making`: directly reuses `decision_making_ros/waypoints_cruise.py`
- `pose_estimation`: directly reuses `pose_estimation/pose_estimation.py` core functions

A base orchestrator wakes all three nodes at a fixed frequency (default `10Hz`).

## Files

- `rpi5/web_runtime/web_bridge.py`
- `rpi5/web_runtime/decision_making_node.py`
- `rpi5/web_runtime/pose_estimation_node.py`
- `rpi5/web_runtime/run_web_runtime.py`

## Dependencies (Raspberry Pi)

```bash
sudo apt-get update
sudo apt-get install -y python3-opencv
```

If you use a virtual environment, activate it first.

## Run

From workspace root:

```bash
python3 rpi5/web_runtime/run_web_runtime.py --hz 10 --remote-host 192.168.50.1 --remote-port 5003
```

Optional:

```bash
python3 rpi5/web_runtime/run_web_runtime.py --hz 10 --disable-pose
```

## Runtime behavior

- Local mirror server starts at `http://127.0.0.1:5003` by default.
- `decision_making` reads and writes through Web endpoints exactly like the ROS Web data flow.
- `pose_estimation` runs each tick and posts robot pose into `/data/decision_making_data` key `pose_estimation_robot_pose`.

## Notes

- `config.json` should keep `"data_flow": "web"`.
- If your upstream viewer host/port changes, pass `--remote-host` and `--remote-port`.
