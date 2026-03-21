"""
reset_button_node.py — Repurposes the hardware power button as a ROS 2
reset trigger.  When pressed, it publishes reset values to /time and the
decision-making topics so the planner restarts cleanly without rebooting.

Prerequisites (run setup_reset_button.sh once):
  1. python3-evdev installed
  2. /etc/systemd/logind.conf  →  HandlePowerKey=ignore
  3. systemd-logind restarted

Usage:
  ros2 run unibots reset_button_node
"""

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import evdev
    from evdev import InputDevice, categorize, ecodes as ev
    _EVDEV_OK = True
except ImportError:
    _EVDEV_OK = False


def _find_power_button() -> Optional["InputDevice"]:
    """Return the first input device that reports KEY_POWER, or None."""
    for path in evdev.list_devices():
        try:
            dev = InputDevice(path)
            caps = dev.capabilities()
            if ev.EV_KEY in caps and ev.KEY_POWER in caps[ev.EV_KEY]:
                return dev
        except Exception:
            pass
    return None


class ResetButtonNode(Node):
    def __init__(self) -> None:
        super().__init__('reset_button_node')

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

        if not _EVDEV_OK:
            self.get_logger().error(
                'evdev not available — install with: sudo apt install python3-evdev'
            )
            return

        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()
        self.get_logger().info('reset_button_node started — waiting for power-button press')

    # ------------------------------------------------------------------
    def _watch(self) -> None:
        dev = _find_power_button()
        if dev is None:
            self.get_logger().warn(
                'Power button device not found in /dev/input. '
                'Is HandlePowerKey=ignore set in logind.conf?'
            )
            return

        self.get_logger().info(f'Monitoring: {dev.path}  ({dev.name})')
        for event in dev.read_loop():
            if event.type == ev.EV_KEY:
                key_event = categorize(event)
                # Fire on key-up to avoid repeated triggers during hold
                if (
                    key_event.scancode == ev.KEY_POWER
                    and key_event.keystate == key_event.key_up
                ):
                    self.get_logger().info('Power button released — resetting topics')
                    self._reset()

    def _reset(self) -> None:
        # Reset sim time to 0
        t = String()
        t.data = '0.0'
        self._pub_time.publish(t)

        # Clear decision state
        empty = String()
        empty.data = ''
        self._pub_decisions.publish(empty)
        self._pub_dm_data.publish(empty)
        self._pub_wpt_type.publish(empty)

        self.get_logger().info(
            'Reset published: /time="0.0", /decisions="", '
            '/decision_making_data="", /dynamic_waypoints_type=""'
        )


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
