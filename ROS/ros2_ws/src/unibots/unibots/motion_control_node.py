from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from gpiozero import DigitalOutputDevice
except Exception:
    DigitalOutputDevice = None


WHEELS: Dict[int, Tuple[int, int]] = {
    1: (5, 6),
    2: (13, 19),
    3: (17, 18),
    4: (22, 23),
    5: (24, 25),
    6: (27, 4),
}


def set_wheel_state(devices: dict, wheel_id: int, state: str) -> None:
    a, b = devices[wheel_id]
    if state == 'f':
        a.on()
        b.off()
    elif state == 'r':
        a.off()
        b.on()
    else:
        a.off()
        b.off()


def apply_pattern(devices: dict, pattern: dict) -> None:
    for wheel_id, state in pattern.items():
        set_wheel_state(devices, wheel_id, state)


def close_devices(devices: dict) -> None:
    for a, b in devices.values():
        try:
            a.off()
        except Exception:
            pass
        try:
            b.off()
        except Exception:
            pass
        try:
            a.close()
        except Exception:
            pass
        try:
            b.close()
        except Exception:
            pass


class MotionControlNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_control_node')

        self.declare_parameter('control_hz', 10.0)

        control_hz = float(self.get_parameter('control_hz').get_parameter_value().double_value)
        self._sim_time_seconds: float | None = None

        if DigitalOutputDevice is None:
            raise RuntimeError('gpiozero is not available. Please install python3-gpiozero on Raspberry Pi.')

        self._devices = {
            wheel_id: (
                DigitalOutputDevice(pin_a, initial_value=False),
                DigitalOutputDevice(pin_b, initial_value=False),
            )
            for wheel_id, (pin_a, pin_b) in WHEELS.items()
        }

        self._forward_pattern = {wheel_id: 'f' for wheel_id in WHEELS}
        self._reverse_pattern = {wheel_id: 'r' for wheel_id in WHEELS}
        self._stop_pattern = {wheel_id: 's' for wheel_id in WHEELS}
        self._left_strafe_pattern = dict(self._stop_pattern)
        self._right_strafe_pattern = dict(self._stop_pattern)
        self._left_strafe_pattern.update({
            1: 'r',
            2: 'f',
            3: 'f',
            4: 'r',
        })
        self._right_strafe_pattern.update({
            1: 'f',
            2: 'r',
            3: 'r',
            4: 'f',
        })

        self.create_subscription(String, '/time', self._on_time, 10)

        period = 0.1 if control_hz <= 0.0 else (1.0 / control_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'motion_control_node started, time-gated control enabled, control_hz={control_hz}'
        )

    def _on_time(self, msg: String) -> None:
        try:
            self._sim_time_seconds = float(msg.data)
        except Exception:
            pass

    def _move_forward(self) -> None:
        apply_pattern(self._devices, self._forward_pattern)

    def _move_backward(self) -> None:
        apply_pattern(self._devices, self._reverse_pattern)

    def _stop(self) -> None:
        apply_pattern(self._devices, self._stop_pattern)

    def _move_left(self) -> None:
        apply_pattern(self._devices, self._left_strafe_pattern)

    def _move_right(self) -> None:
        apply_pattern(self._devices, self._right_strafe_pattern)

    def _tick(self) -> None:
        sim_time_seconds = self._sim_time_seconds
        if sim_time_seconds is not None and sim_time_seconds < 20:
            if sim_time_seconds % 4 < 2:
                self._move_forward()
            else:
                self._move_backward()
        else:
            self._stop()

    def shutdown_motors(self) -> None:
        if hasattr(self, '_devices'):
            apply_pattern(self._devices, self._stop_pattern)
            close_devices(self._devices)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()