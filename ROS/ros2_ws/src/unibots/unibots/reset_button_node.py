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
    """Return the input device for the hardware power button, or None.

    Strategy:
      1. Prefer a device whose name contains 'pwr' (e.g. 'pwr_button').
      2. Fall back to the first device that reports KEY_POWER capability,
         skipping HDMI/CEC devices (which also advertise KEY_POWER but are
         not the physical button).
    """
    candidates = []
    for path in evdev.list_devices():
        try:
            dev = InputDevice(path)
            name_lower = dev.name.lower()
            caps = dev.capabilities()
            has_power_key = ev.EV_KEY in caps and ev.KEY_POWER in caps[ev.EV_KEY]
            if not has_power_key:
                continue
            # Highest priority: device explicitly named as power button
            if 'pwr' in name_lower or 'power_button' in name_lower:
                return dev
            # Skip HDMI/CEC devices — they list KEY_POWER for TV-remote power
            if 'hdmi' in name_lower or 'cec' in name_lower:
                continue
            candidates.append(dev)
        except Exception:
            pass
    return candidates[0] if candidates else None


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
        self._pub_run = self.create_publisher(String, '/run', 10)

        if not _EVDEV_OK:
            self.get_logger().error(
                'evdev not available — install with: sudo apt install python3-evdev'
            )
            return

        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()
        self._is_running: bool = False
        self._watch_alive: bool = False
        print('[reset_button_node] started — waiting for power-button press', flush=True)
        self.get_logger().info('reset_button_node started — waiting for power-button press')
        self.create_timer(5.0, self._status_tick)

    def _status_tick(self) -> None:
        state = 'running' if self._is_running else 'stopped'
        watching = 'watching' if self._watch_alive else 'NOT watching (thread dead or no device)'
        print(f'[reset_button_node] status: {state}, button thread: {watching}', flush=True)

    # ------------------------------------------------------------------
    def _watch(self) -> None:
        try:
            dev = _find_power_button()
        except Exception as exc:
            print(f'[reset_button_node] ERROR finding power button: {exc}', flush=True)
            return

        if dev is None:
            msg = 'Power button device not found in /dev/input. Is HandlePowerKey=ignore set in logind.conf? Is user in input group?'
            print(f'[reset_button_node] WARN: {msg}', flush=True)
            self.get_logger().warn(msg)
            return

        print(f'[reset_button_node] Monitoring: {dev.path}  ({dev.name})', flush=True)
        self.get_logger().info(f'Monitoring: {dev.path}  ({dev.name})')
        self._watch_alive = True
        try:
            dev.grab()  # exclusively grab device so systemd-logind can't consume events
            print('[reset_button_node] Device grabbed (exclusive access)', flush=True)
            for event in dev.read_loop():
                print(f'[reset_button_node] RAW event: type={event.type} code={event.code} value={event.value}', flush=True)
                if event.type == ev.EV_KEY:
                    key_event = categorize(event)
                    print(
                        f'[reset_button_node] KEY event: scancode={key_event.scancode} '
                        f'keystate={key_event.keystate} KEY_POWER={ev.KEY_POWER}',
                        flush=True,
                    )
                    if (
                        key_event.scancode == ev.KEY_POWER
                        and key_event.keystate == key_event.key_up
                    ):
                        if self._is_running:
                            print('[reset_button_node] Power button: running → stopping', flush=True)
                            self.get_logger().info('Power button: running → stopping')
                            self._stop()
                        else:
                            print('[reset_button_node] Power button: stopped → resetting and starting', flush=True)
                            self.get_logger().info('Power button: stopped → resetting and starting')
                            self._reset_and_run()
        except Exception as exc:
            print(f'[reset_button_node] ERROR in read_loop: {exc}', flush=True)
            print('[reset_button_node] Hint: try: sudo usermod -aG input $USER  (then re-login)', flush=True)
        finally:
            self._watch_alive = False

    def _stop(self) -> None:
        run_off = String()
        run_off.data = 'off'
        self._pub_run.publish(run_off)
        self._is_running = False
        print('[reset_button_node] Stopped: /run=off', flush=True)
        self.get_logger().info('Stopped: /run=off')

    def _reset_and_run(self) -> None:
        # Reset sim time to 0
        t = String()
        t.data = '0.0'
        self._pub_time.publish(t)

        # Clear waypoint type
        empty = String()
        empty.data = ''
        self._pub_wpt_type.publish(empty)

        # Start all nodes
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
