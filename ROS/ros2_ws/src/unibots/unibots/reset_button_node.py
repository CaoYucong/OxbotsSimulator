"""
reset_button_node.py — Publishes /run=on after a short startup delay so other
nodes begin processing without a hardware button press.

Usage:
  ros2 run unibots reset_button_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ResetButtonNode(Node):
    def __init__(self) -> None:
        super().__init__('reset_button_node')

        self.declare_parameter('startup_delay_sec', 5.0)

        self._pub_run = self.create_publisher(String, '/run', 10)
        self._started = False

        delay = self.get_parameter('startup_delay_sec').get_parameter_value().double_value
        self.get_logger().info(f'reset_button_node started — will publish /run=on in {delay:.1f}s')
        self.create_timer(delay, self._start_run)

    def _start_run(self) -> None:
        if self._started:
            return
        self._started = True

        run_on = String()
        run_on.data = 'on'
        self._pub_run.publish(run_on)

        msg = 'Started: /run=on'
        print(f'[reset_button_node] {msg}', flush=True)
        self.get_logger().info(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResetButtonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
