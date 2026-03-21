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
        self._run_enabled: bool = False
        self._pub_time = self.create_publisher(String, '/time', 10)
        self.create_subscription(String, '/run', self._on_run, 10)

        period = 0.1 if publish_hz <= 0.0 else (1.0 / publish_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(f'timer node started, publish_hz={publish_hz}')

    def _on_run(self, msg: String) -> None:
        enabled = (msg.data or '').strip().lower() != 'off'
        if enabled and not self._run_enabled:
            self._start_mono = time.monotonic()  # restart clock from 0
        self._run_enabled = enabled

    def _tick(self) -> None:
        if not self._run_enabled:
            return
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
