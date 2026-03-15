import json
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RadarSensorNode(Node):
    def __init__(self) -> None:
        super().__init__('radar_sensor_node')

        self.declare_parameter('remote_host', '192.168.50.1')
        self.declare_parameter('remote_port', 5003)
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('request_timeout', 1.0)

        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)

        if self.request_timeout <= 0.0:
            self.request_timeout = 1.0

        self._url = f'http://{self.remote_host}:{self.remote_port}/data/simulation_data'
        self._pub_radar_sensor = self.create_publisher(String, '/radar_sensor', 10)
        self._last_radar_text = None
        self._error_logged = False

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)

        self.get_logger().info(
            f'radar_sensor_node started; upstream={self._url}, poll_hz={poll_hz}'
        )

    def _poll_once(self) -> None:
        try:
            with urllib.request.urlopen(self._url, timeout=self.request_timeout) as res:
                body = res.read()
            payload = json.loads(body.decode('utf-8', errors='ignore'))
            if not isinstance(payload, dict):
                raise ValueError('simulation_data payload must be a JSON object')
        except Exception as exc:
            if not self._error_logged:
                self.get_logger().warn(f'upstream fetch failed for {self._url}: {exc}')
                self._error_logged = True
            return

        self._error_logged = False
        radar_text = str(payload.get('radar_sensor', '')).strip()
        if not radar_text:
            return
        if radar_text == self._last_radar_text:
            return

        msg = String()
        msg.data = radar_text
        self._pub_radar_sensor.publish(msg)
        self._last_radar_text = radar_text


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
