import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from . import decision_cruise as planner


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('decision_node')

        self.declare_parameter('tick_hz', 1.0)
        self.declare_parameter('fallback_tick_hz', 1.0)
        self.declare_parameter('mode', '')
        self.declare_parameter('default_speed', 0.3)
        self.declare_parameter('time_topic', '/time')

        tick_hz = float(self.get_parameter('tick_hz').get_parameter_value().double_value)  # kept for launch-file compat, unused
        fallback_tick_hz = float(self.get_parameter('fallback_tick_hz').get_parameter_value().double_value)
        self._mode_param = self.get_parameter('mode').get_parameter_value().string_value
        self._default_speed = float(self.get_parameter('default_speed').get_parameter_value().double_value)
        self._time_topic = self.get_parameter('time_topic').get_parameter_value().string_value or '/time'

        self._current_x: Optional[float] = 0.0
        self._current_y: Optional[float] = 0.0
        self._current_theta: Optional[float] = 0.0
        self._visible_balls_text = ''
        self._radar_sensor_text = ''
        self._waypoint_status = 'going'
        self._sim_time_seconds: Optional[float] = None
        self._run_enabled: bool = False

        self.create_subscription(PoseStamped, '/current_position', self._on_current_position, 10)
        self.create_subscription(String, '/visible_balls', self._on_visible_balls, 10)
        self.create_subscription(String, '/radar_sensor', self._on_radar_sensor, 10)
        self.create_subscription(String, '/waypoint_status', self._on_waypoint_status, 10)
        self.create_subscription(String, self._time_topic, self._on_time, 10)
        self.create_subscription(String, '/run', self._on_run, 10)

        self._pub_decisions = self.create_publisher(String, '/decisions', 10)
        self._pub_decision_making = self.create_publisher(String, '/decision_making_data', 10)
        self._pub_dynamic_waypoint = self.create_publisher(String, '/dynamic_waypoint', 10)
        self._pub_dynamic_waypoints_type = self.create_publisher(String, '/dynamic_waypoints_type', 10)
        self._pub_collision_avoiding_waypoint = self.create_publisher(String, '/collision_avoiding_waypoint', 10)
        self._pub_mode = self.create_publisher(String, '/mode', 10)

        self._mode_warned = False
        fallback_period = 1.0 if fallback_tick_hz <= 0.0 else (1.0 / fallback_tick_hz)
        self.create_timer(fallback_period, self._tick)

        self.get_logger().info(
            f'decision_node started, tick_hz={fallback_tick_hz} (fixed), input_source=topics, time_topic={self._time_topic}'
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

    def _on_run(self, msg: String) -> None:
        self._run_enabled = (msg.data or '').strip().lower() != 'off'

    def _resolve_mode(self) -> str:
        mode = (self._mode_param or '').strip().lower()
        return mode or planner.DEFAULT_MODE

    def _tick(self) -> None:
        if self._current_x is None or self._current_y is None or self._current_theta is None:
            return
        if self._sim_time_seconds is None:
            return

        mode_key = self._resolve_mode()
        result = planner.decide_from_ros_state(
            current_x=self._current_x,
            current_y=self._current_y,
            current_theta=self._current_theta,
            visible_balls_json=self._visible_balls_text,
            radar_sensor_text=self._radar_sensor_text,
            sim_time_seconds=float(self._sim_time_seconds),
            waypoint_status=self._waypoint_status,
            mode=mode_key,
            default_speed=self._default_speed,
        )
        if result is None:
            return

        waypoint = result.get('dynamic_waypoint')
        collision_waypoint = result.get('collision_avoiding_waypoint')
        speed = result.get('speed')
        if waypoint is None or speed is None:
            return

        waypoint_text = self._format_waypoint_text(waypoint)
        collision_waypoint_text = '' if collision_waypoint is None else self._format_waypoint_text(collision_waypoint)

        payload = {
            'dynamic_waypoints': waypoint_text,
            'collision_avoiding_waypoint': collision_waypoint_text,
            'speed': f'{float(speed):.6f}',
        }
        self._publish_decisions(payload)
        self._publish_dynamic_waypoint(waypoint_text)
        self._publish_dynamic_waypoints_type()
        self._publish_collision_avoiding_waypoint(collision_waypoint_text)
        self._publish_decision_making_data()
        self._publish_mode(mode_key)

    def _publish_dynamic_waypoints_type(self) -> None:
        cache = getattr(planner, 'DECISION_MAKING_DATA_LOCAL_CACHE', {})
        wp_type = str(cache.get('dynamic_waypoints_type', '')).strip()
        if not wp_type:
            cache2 = getattr(planner, 'DECISION_MAKING_DATA_CACHE', {})
            wp_type = str(cache2.get('dynamic_waypoints_type', '')).strip()
        msg = String()
        msg.data = wp_type
        self._pub_dynamic_waypoints_type.publish(msg)

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._pub_mode.publish(msg)

    def _format_waypoint_text(self, waypoint) -> str:
        wx, wy, theta = waypoint
        if theta is None:
            return f'({wx:.6f}, {wy:.6f}, None)'
        return f'({wx:.6f}, {wy:.6f}, {math.degrees(theta):.6f})'

    def _publish_collision_avoiding_waypoint(self, waypoint_text: str) -> None:
        msg = String()
        msg.data = waypoint_text
        self._pub_collision_avoiding_waypoint.publish(msg)

    def _publish_dynamic_waypoint(self, waypoint_text: str) -> None:
        msg = String()
        msg.data = waypoint_text
        self._pub_dynamic_waypoint.publish(msg)

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
