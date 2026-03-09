#!/usr/bin/env python3
"""10Hz orchestrator for web_bridge + decision_making + pose_estimation on Raspberry Pi."""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

from web_bridge import WebBridgeNode
from decision_making_node import DecisionMakingNode
from pose_estimation_node import PoseEstimationNode


def _load_config(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    default_config = root / "config.json"

    parser = argparse.ArgumentParser()
    parser.add_argument("--hz", type=float, default=10.0, help="Main loop frequency (default: 10)")
    parser.add_argument("--config", type=Path, default=default_config, help="Path to config.json")
    parser.add_argument("--remote-host", default="192.168.50.1", help="Upstream simulation host")
    parser.add_argument("--remote-port", type=int, default=5003, help="Upstream simulation port")
    parser.add_argument("--local-host", default="127.0.0.1", help="Local mirror host")
    parser.add_argument("--local-port", type=int, default=5003, help="Local mirror port")
    parser.add_argument("--disable-pose", action="store_true", help="Disable pose_estimation tick")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = _load_config(args.config)

    default_speed = float(config.get("default_linear_velocity", 3.0))

    stop = False

    def _handle_stop(_sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    project_root = Path(__file__).resolve().parents[2]

    web_bridge = WebBridgeNode(
        remote_host=args.remote_host,
        remote_port=args.remote_port,
        local_host=args.local_host,
        local_port=args.local_port,
        request_timeout=1.0,
        camera_remote_host=args.remote_host,
        camera_remote_port=args.remote_port,
        camera_path="/data/front_camera",
        camera_request_timeout=1.0,
        default_speed=default_speed,
    )

    decision = DecisionMakingNode(project_root=project_root)
    pose = PoseEstimationNode(
        project_root=project_root,
        bridge_host=args.local_host,
        bridge_port=args.local_port,
        enabled=(not args.disable_pose),
    )

    hz = args.hz if args.hz > 0 else 10.0
    period = 1.0 / hz
    print(f"[runtime] started at {hz:.2f} Hz")

    try:
        next_tick = time.monotonic()
        while not stop:
            web_bridge.tick()
            decision.tick()
            pose.tick()

            next_tick += period
            sleep_sec = next_tick - time.monotonic()
            if sleep_sec > 0:
                time.sleep(sleep_sec)
            else:
                # Loop overrun: reset schedule anchor.
                next_tick = time.monotonic()
    finally:
        web_bridge.shutdown()
        print("[runtime] stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
