"""
reset_button_node.py — Match start / reset via dedicated GPIO 24.

GPIO 24 is a dedicated match-control line:
  LOW  (0 V)   → reset  (/time=0, clear waypoints, /run=off)
  HIGH (3.3 V) → start competition (/time=0, clear waypoints, /run=on);
                 the line stays HIGH for the whole match.

Usage:
  ros2 run unibots reset_button_node
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from gpiozero import DigitalInputDevice
    _GPIO_OK = True
except ImportError:
    _GPIO_OK = False

DEFAULT_GPIO_PIN = 24


class ResetButtonNode(Node):
    def __init__(self) -> None:
        super().__init__('reset_button_node')

        self.declare_parameter('gpio_pin', DEFAULT_GPIO_PIN)
        self.declare_parameter('poll_hz', 20.0)
        self.declare_parameter('debounce_sec', 0.05)
        self.declare_parameter('time_topic', '/time')
        self.declare_parameter('decisions_topic', '/decisions')
        self.declare_parameter('decision_making_data_topic', '/decision_making_data')
        self.declare_parameter('dynamic_waypoints_type_topic', '/dynamic_waypoints_type')

        self._pub_time = self.create_publisher(
            String,
            self.get_parameter('time_topic').get_parameter_value().string_value or '/time',
            10,
        )
        self._pub_decisions = self.create_publisher(
            String,
            self.get_parameter('decisions_topic').get_parameter_value().string_value or '/decisions',
            10,
        )
        self._pub_dm_data = self.create_publisher(
            String,
            self.get_parameter('decision_making_data_topic').get_parameter_value().string_value
            or '/decision_making_data',
            10,
        )
        self._pub_wpt_type = self.create_publisher(
            String,
            self.get_parameter('dynamic_waypoints_type_topic').get_parameter_value().string_value
            or '/dynamic_waypoints_type',
            10,
        )
        self._pub_run = self.create_publisher(String, '/run', 10)

        self._is_running: bool = False
        self._watch_alive: bool = False
        self._gpio_level: str = 'unknown'

        if not _GPIO_OK:
            self.get_logger().error(
                'gpiozero not available — install with: sudo apt install python3-gpiozero'
            )
            return

        pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        self._thread = threading.Thread(target=self._watch_gpio, daemon=True)
        self._thread.start()
        print(f'[reset_button_node] started — monitoring GPIO {pin} (LOW=reset, HIGH=start)', flush=True)
        self.get_logger().info(f'reset_button_node started — monitoring GPIO {pin}')
        self.create_timer(5.0, self._status_tick)

    def _status_tick(self) -> None:
        state = 'running' if self._is_running else 'stopped'
        watching = 'watching' if self._watch_alive else 'NOT watching (thread dead or no GPIO)'
        print(
            f'[reset_button_node] status: {state}, GPIO={self._gpio_level}, thread: {watching}',
            flush=True,
        )

    def _watch_gpio(self) -> None:
        pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value
        debounce = self.get_parameter('debounce_sec').get_parameter_value().double_value
        poll_hz = self.get_parameter('poll_hz').get_parameter_value().double_value
        interval = 1.0 / poll_hz if poll_hz > 0.0 else 0.05

        try:
            gpio = DigitalInputDevice(pin, pull_up=True)
        except Exception as exc:
            print(f'[reset_button_node] ERROR opening GPIO {pin}: {exc}', flush=True)
            self.get_logger().error(f'Failed to open GPIO {pin}: {exc}')
            return

        self._watch_alive = True
        stable = gpio.value
        pending = stable
        pending_since = time.monotonic()
        self._gpio_level = 'HIGH' if stable else 'LOW'

        if stable:
            self._reset_and_run()
        else:
            self._reset_stop()

        try:
            while rclpy.ok():
                reading = gpio.value
                now = time.monotonic()
                if reading != pending:
                    pending = reading
                    pending_since = now
                elif (now - pending_since) >= debounce and pending != stable:
                    stable = pending
                    self._gpio_level = 'HIGH' if stable else 'LOW'
                    if stable:
                        print('[reset_button_node] GPIO HIGH → start competition', flush=True)
                        self.get_logger().info('GPIO HIGH → start competition')
                        self._reset_and_run()
                    else:
                        print('[reset_button_node] GPIO LOW → reset', flush=True)
                        self.get_logger().info('GPIO LOW → reset')
                        self._reset_stop()
                time.sleep(interval)
        except Exception as exc:
            print(f'[reset_button_node] ERROR in GPIO watch loop: {exc}', flush=True)
            self.get_logger().error(f'GPIO watch loop failed: {exc}')
        finally:
            gpio.close()
            self._watch_alive = False

    def _reset_stop(self) -> None:
        t = String()
        t.data = '0.0'
        self._pub_time.publish(t)

        empty = String()
        empty.data = ''
        self._pub_wpt_type.publish(empty)

        run_off = String()
        run_off.data = 'off'
        self._pub_run.publish(run_off)
        self._is_running = False

        msg = 'Reset: /time="0.0", /dynamic_waypoints_type="", /run=off'
        print(f'[reset_button_node] {msg}', flush=True)
        self.get_logger().info(msg)

    def _reset_and_run(self) -> None:
        if self._is_running:
            return

        t = String()
        t.data = '0.0'
        self._pub_time.publish(t)

        empty = String()
        empty.data = ''
        self._pub_wpt_type.publish(empty)

        run_on = String()
        run_on.data = 'on'
        self._pub_run.publish(run_on)
        self._is_running = True

        msg = 'Reset + started: /time="0.0", /dynamic_waypoints_type="", /run=on'
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
