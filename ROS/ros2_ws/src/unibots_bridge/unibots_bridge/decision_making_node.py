import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from . import waypoints_cruise_ros as planner


class DecisionMakingNode(Node):
    def __init__(self) -> None:
        super().__init__('decision_making')

        self.declare_parameter('tick_hz', 10.0)
        self.declare_parameter('mode', 'improved_nearest_v3_5')
        self.declare_parameter('default_speed', 0.3)

        tick_hz = float(self.get_parameter('tick_hz').get_parameter_value().double_value)
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.default_speed = float(self.get_parameter('default_speed').get_parameter_value().double_value)
        self._mode_warned = False

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_theta: Optional[float] = None
        self.visible_balls_text: str = '[]'
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

        self.get_logger().info(
            f'decision_making started, mode={self.mode}, tick_hz={tick_hz} '
            f'(full mode pipeline enabled)'
        )

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
        self.visible_balls_text = msg.data if msg.data else '[]'

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
        planner._refresh_sim_data()
        planner._refresh_decisions_data()
        planner._bootstrap_stack_data()

        mode_key = (self.mode or '').strip().lower()
        handler = planner._MODE_HANDLERS.get(mode_key)
        if handler is None:
            if not self._mode_warned:
                self.get_logger().warn(f'unknown mode: {mode_key}')
                self._mode_warned = True
            return

        self._mode_warned = False
        try:
            handler()
        except Exception as exc:
            self.get_logger().warn(f'decision handler failed: {exc}')
            return

        planner._refresh_decisions_data()

        waypoint_deg = planner._read_dynamic_waypoints()
        if waypoint_deg is not None and len(waypoint_deg) >= 2:
            x = float(waypoint_deg[0])
            y = float(waypoint_deg[1])
            bearing_deg = waypoint_deg[2] if len(waypoint_deg) >= 3 else None
            theta = math.radians(float(bearing_deg)) if bearing_deg is not None else 0.0
            waypoint_msg = String()
            waypoint_msg.data = f'({x:.6f}, {y:.6f}, {theta:.6f})'
            self.pub_waypoint_cmd.publish(waypoint_msg)

        speed = planner._get_decision_value('speed')
        try:
            speed_value = float(speed) if speed is not None else self.default_speed
        except Exception:
            speed_value = self.default_speed
        speed_msg = Twist()
        speed_msg.linear.x = speed_value
        self.pub_speed_cmd.publish(speed_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionMakingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
