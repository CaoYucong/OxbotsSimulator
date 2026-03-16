import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimerNode(Node):
    def __init__(self) -> None:
        super().__init__('timer')

        self.declare_parameter('publish_hz', 10.0)
        publish_hz = float(self.get_parameter('publish_hz').get_parameter_value().double_value)

        self._start_mono = time.monotonic()
        self._pub_time = self.create_publisher(String, '/time', 10)

        period = 0.1 if publish_hz <= 0.0 else (1.0 / publish_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(f'timer node started, publish_hz={publish_hz}')

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._start_mono
        msg = String()
        msg.data = f'{elapsed:.6f}'
        self._pub_time.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
