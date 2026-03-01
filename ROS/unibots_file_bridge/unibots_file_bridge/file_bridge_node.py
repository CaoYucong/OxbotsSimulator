import json
import math
import os
import re
import tempfile
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String


class FileBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('file_bridge_node')

        self.declare_parameter('sim_dir', '/tmp/unibots_simulation')
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('frame_id', 'map')

        self.sim_dir = Path(self.get_parameter('sim_dir').get_parameter_value().string_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.current_position_file = self.sim_dir / 'current_position.txt'
        self.visible_balls_file = self.sim_dir / 'visible_balls.txt'
        self.visible_balls_json_file = self.sim_dir / 'visible_balls.json'
        self.time_file = self.sim_dir / 'time.txt'

        self.dynamic_waypoints_file = self.sim_dir / 'dynamic_waypoints.txt'
        self.speed_file = self.sim_dir / 'speed.txt'

        self.pub_current_position = self.create_publisher(PoseStamped, '/sim/current_position', 10)
        self.pub_visible_balls = self.create_publisher(String, '/sim/visible_balls', 10)
        self.pub_time = self.create_publisher(String, '/sim/time', 10)

        self.create_subscription(String, '/sim/dynamic_waypoints_cmd', self._on_dynamic_waypoints_cmd, 10)
        self.create_subscription(Twist, '/sim/speed_cmd', self._on_speed_cmd, 10)

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)

        self.get_logger().info(f'file_bridge_node started, sim_dir={self.sim_dir}, poll_hz={poll_hz}')

    def _poll_once(self) -> None:
        self._publish_current_position()
        self._publish_visible_balls()
        self._publish_time()

    def _publish_current_position(self) -> None:
        try:
            content = self.current_position_file.read_text(encoding='utf-8').strip()
        except Exception:
            return

        if not content:
            return

        parsed = self._parse_current_position(content)
        if parsed is None:
            return
        x, y, theta = parsed

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        half = theta * 0.5
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)

        self.pub_current_position.publish(msg)

    def _parse_current_position(self, content: str):
        nums = re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', content or '')
        if len(nums) < 3:
            return None
        try:
            x = float(nums[0])
            y = float(nums[1])
            theta = float(nums[2])
        except Exception:
            return None

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
            return None

        if abs(theta) > (2.0 * math.pi + 1.0):
            theta = math.radians(theta)

        return x, y, theta

    def _publish_visible_balls(self) -> None:
        payload = self._load_visible_balls_payload()
        if payload is None:
            return
        out = String()
        out.data = payload
        self.pub_visible_balls.publish(out)

    def _load_visible_balls_payload(self):
        raw = None
        try:
            raw = self.visible_balls_json_file.read_text(encoding='utf-8').strip()
        except Exception:
            raw = None

        if raw:
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, list):
                    return json.dumps(parsed, separators=(',', ':'))
            except Exception:
                pass

        try:
            raw = self.visible_balls_file.read_text(encoding='utf-8').strip()
        except Exception:
            return None

        if not raw:
            return json.dumps([])

        try:
            parsed = json.loads(raw)
            if isinstance(parsed, list):
                return json.dumps(parsed, separators=(',', ':'))
        except Exception:
            pass

        balls = []
        for line in raw.splitlines():
            text = line.strip()
            if not text:
                continue
            nums = re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', text)
            if len(nums) < 2:
                continue
            try:
                x = float(nums[0])
                y = float(nums[1])
            except Exception:
                continue
            parts = [p.strip() for p in text.strip('()').split(',')]
            typ = 'BALL'
            if len(parts) >= 3:
                typ = str(parts[2]).strip().strip("'\"") or 'BALL'
            balls.append({'x': x, 'y': y, 'type': typ})

        return json.dumps(balls, separators=(',', ':'))

    def _publish_time(self) -> None:
        try:
            raw = self.time_file.read_text(encoding='utf-8').strip()
        except Exception:
            return

        if not raw:
            return

        try:
            _ = float(raw)
        except Exception:
            return

        out = String()
        out.data = raw
        self.pub_time.publish(out)

    def _on_dynamic_waypoints_cmd(self, msg: String) -> None:
        payload = (msg.data or '').strip()
        if not payload:
            return
        self._atomic_write(self.dynamic_waypoints_file, payload + '\n')

    def _on_speed_cmd(self, msg: Twist) -> None:
        speed = float(msg.linear.x)
        if not math.isfinite(speed):
            return
        self._atomic_write(self.speed_file, f'{speed:.6f}\n')

    def _atomic_write(self, target: Path, content: str) -> None:
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=str(target.parent),
                delete=False,
            ) as tmp:
                tmp.write(content)
                tmp.flush()
                os.fsync(tmp.fileno())
                tmp_path = Path(tmp.name)
            os.replace(tmp_path, target)
        except Exception as exc:
            self.get_logger().warn(f'atomic write failed for {target}: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FileBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
