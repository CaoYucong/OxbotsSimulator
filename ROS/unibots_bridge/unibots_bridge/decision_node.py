import math
import json
import urllib.request
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from . import waypoints_cruise_ros as planner


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('decision_node')

        self.declare_parameter('tick_hz', 10.0)
        self.declare_parameter('mode', 'mode_improved_nearest_v3_5')
        self.declare_parameter('default_speed', 0.3)
        self.declare_parameter('decisions_post_url', 'http://127.0.0.1:5003/data/decisions')
        self.declare_parameter('decisions_post_timeout', 0.2)

        tick_hz = float(self.get_parameter('tick_hz').get_parameter_value().double_value)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.default_speed = float(self.get_parameter('default_speed').get_parameter_value().double_value)
        self.decisions_post_url = self.get_parameter('decisions_post_url').get_parameter_value().string_value
        self.decisions_post_timeout = float(self.get_parameter('decisions_post_timeout').get_parameter_value().double_value)
        self._post_error_logged = False

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_theta: Optional[float] = None
        self.visible_balls_json: str = '[]'
        self.sim_time_seconds: float = 0.0
        self.waypoint_status: str = 'going'

        self.pub_waypoint_cmd = self.create_publisher(String, '/sim/dynamic_waypoints_cmd', 10)
        self.pub_speed_cmd = self.create_publisher(Twist, '/sim/speed_cmd', 10)

        self.create_subscription(PoseStamped, '/sim/current_position', self._on_current_position, 10)
        self.create_subscription(String, '/sim/visible_balls', self._on_visible_balls, 10)
        self.create_subscription(String, '/sim/waypoint_status', self._on_waypoint_status, 10)
        self.create_subscription(String, '/sim/time', self._on_time, 10)

        period = 0.1 if tick_hz <= 0.0 else (1.0 / tick_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(f'decision_node started, mode={self.mode}, tick_hz={tick_hz}')

    def _on_current_position(self, msg: PoseStamped) -> None:
        self.current_x = float(msg.pose.position.x)
        self.current_y = float(msg.pose.position.y)

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def _on_visible_balls(self, msg: String) -> None:
        self.visible_balls_json = msg.data if msg.data else '[]'

    def _on_time(self, msg: String) -> None:
        try:
            self.sim_time_seconds = float(msg.data)
        except Exception:
            pass

    def _on_waypoint_status(self, msg: String) -> None:
        text = (msg.data or '').strip().lower()
        if text:
            self.waypoint_status = text

    def _tick(self) -> None:
        if self.current_x is None or self.current_y is None or self.current_theta is None:
            return

        result = planner.decide_from_ros_state(
            current_x=self.current_x,
            current_y=self.current_y,
            current_theta=self.current_theta,
            visible_balls_json=self.visible_balls_json,
            sim_time_seconds=self.sim_time_seconds,
            waypoint_status=self.waypoint_status,
            mode=self.mode,
            default_speed=self.default_speed,
        )

        if not result:
            return

        waypoint = result.get('dynamic_waypoint')
        speed = result.get('speed')

        if waypoint is not None and len(waypoint) >= 3:
            x, y, theta = float(waypoint[0]), float(waypoint[1]), float(waypoint[2])
            msg = String()
            msg.data = f'({x:.6f}, {y:.6f}, {theta:.6f})'
            self.pub_waypoint_cmd.publish(msg)

        if speed is not None:
            tw = Twist()
            tw.linear.x = float(speed)
            self.pub_speed_cmd.publish(tw)

        if waypoint is not None:
            self._post_decisions(waypoint, speed)

    def _post_decisions(self, waypoint, speed) -> None:
        try:
            x, y, theta = float(waypoint[0]), float(waypoint[1]), float(waypoint[2])
            theta_deg = math.degrees(theta)
            payload = {
                'dynamic_waypoints': f'({x:.6f}, {y:.6f}, {theta_deg:.6f})',
                'speed': f'{float(speed if speed is not None else self.default_speed):.6f}',
            }
            body = json.dumps(payload).encode('utf-8')
            req = urllib.request.Request(
                self.decisions_post_url,
                data=body,
                method='POST',
                headers={'Content-Type': 'application/json', 'Content-Length': str(len(body))},
            )
            with urllib.request.urlopen(req, timeout=self.decisions_post_timeout) as res:
                res.read()
            self._post_error_logged = False
        except Exception as exc:
            if not self._post_error_logged:
                self.get_logger().warn(f'failed to POST decisions to {self.decisions_post_url}: {exc}')
                self._post_error_logged = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
