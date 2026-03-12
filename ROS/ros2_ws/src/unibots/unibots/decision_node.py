import json
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from . import decision_cruise as planner


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('decision_node')

        self.declare_parameter('tick_hz', 10.0)
        self.declare_parameter('mode', '')
        self.declare_parameter('default_speed', 0.3)

        tick_hz = float(self.get_parameter('tick_hz').get_parameter_value().double_value)
        self._mode_param = self.get_parameter('mode').get_parameter_value().string_value
        self._default_speed = float(self.get_parameter('default_speed').get_parameter_value().double_value)

        self._current_x: Optional[float] = None
        self._current_y: Optional[float] = None
        self._current_theta: Optional[float] = None
        self._visible_balls_text = ''
        self._radar_sensor_text = ''
        self._waypoint_status = 'going'
        self._sim_time_seconds = 0.0

        self.create_subscription(PoseStamped, '/current_position', self._on_current_position, 10)
        self.create_subscription(String, '/visible_balls', self._on_visible_balls, 10)
        self.create_subscription(String, '/radar_sensor', self._on_radar_sensor, 10)
        self.create_subscription(String, '/waypoint_status', self._on_waypoint_status, 10)
        self.create_subscription(String, '/time', self._on_time, 10)

        self._pub_decisions = self.create_publisher(String, '/decisions', 10)
        self._pub_decision_making = self.create_publisher(String, '/decision_making_data', 10)

        self._mode_warned = False
        period = 0.1 if tick_hz <= 0.0 else (1.0 / tick_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'decision_node started, tick_hz={tick_hz}, input_source=topics'
        )

    def _on_current_position(self, msg: PoseStamped) -> None:
        self._current_x = float(msg.pose.position.x)
        self._current_y = float(msg.pose.position.y)

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self._current_theta = math.atan2(siny_cosp, cosy_cosp)

    def _on_visible_balls(self, msg: String) -> None:
        self._visible_balls_text = msg.data if msg.data else ''

    def _on_radar_sensor(self, msg: String) -> None:
        self._radar_sensor_text = msg.data if msg.data else ''

    def _on_waypoint_status(self, msg: String) -> None:
        text = (msg.data or '').strip().lower()
        if text:
            self._waypoint_status = text

    def _on_time(self, msg: String) -> None:
        try:
            self._sim_time_seconds = float(msg.data)
        except Exception:
            pass

    def _resolve_mode(self) -> str:
        mode = (self._mode_param or '').strip().lower()
        if mode:
            return mode
        mode = planner._read_mode(planner.MODE_FILE) or os.environ.get('MODE') or planner.DEFAULT_MODE
        return (mode or '').strip().lower()

    def _tick(self) -> None:
        if self._current_x is None or self._current_y is None or self._current_theta is None:
            return

        mode_key = self._resolve_mode()
        result = planner.decide_from_ros_state(
            current_x=self._current_x,
            current_y=self._current_y,
            current_theta=self._current_theta,
            visible_balls_json=self._visible_balls_text,
            radar_sensor_text=self._radar_sensor_text,
            sim_time_seconds=self._sim_time_seconds,
            waypoint_status=self._waypoint_status,
            mode=mode_key,
            default_speed=self._default_speed,
        )
        if result is None:
            return

        waypoint = result.get('dynamic_waypoint')
        speed = result.get('speed')
        if waypoint is None or speed is None:
            return

        wx, wy, theta = waypoint
        if theta is None:
            waypoint_text = f'({wx:.6f}, {wy:.6f}, None)'
        else:
            waypoint_text = f'({wx:.6f}, {wy:.6f}, {math.degrees(theta):.6f})'

        payload = {
            'dynamic_waypoints': waypoint_text,
            'speed': f'{float(speed):.6f}',
        }
        self._publish_decisions(payload)
        self._publish_decision_making_data()

    def _publish_decisions(self, payload: dict) -> None:
        try:
            body = json.dumps(payload, ensure_ascii=True, separators=(',', ':'))
        except Exception:
            return
        msg = String()
        msg.data = body
        self._pub_decisions.publish(msg)

    def _publish_decision_making_data(self) -> None:
        merged = {}
        if getattr(planner, 'DECISION_MAKING_DATA_CACHE', None):
            merged.update(planner.DECISION_MAKING_DATA_CACHE)
        if getattr(planner, 'DECISION_MAKING_DATA_LOCAL_CACHE', None):
            merged.update(planner.DECISION_MAKING_DATA_LOCAL_CACHE)
        try:
            body = json.dumps(merged, ensure_ascii=True, separators=(',', ':'))
        except Exception:
            return
        msg = String()
        msg.data = body
        self._pub_decision_making.publish(msg)



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
