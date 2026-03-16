from typing import Dict, Tuple

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

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
        self.declare_parameter('position_topic', '/current_position')
        self.declare_parameter('speed_topic', '/speed')
        self.declare_parameter('target_heading_deg', 10.0)
        self.declare_parameter('heading_deadband_min_deg', 9.0)
        self.declare_parameter('heading_deadband_max_deg', 11.0)

        control_hz = float(self.get_parameter('control_hz').get_parameter_value().double_value)
        position_topic = self.get_parameter('position_topic').get_parameter_value().string_value or '/current_position'
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value or '/speed'
        self._target_heading_deg = float(self.get_parameter('target_heading_deg').get_parameter_value().double_value)
        self._heading_deadband_min_deg = float(self.get_parameter('heading_deadband_min_deg').get_parameter_value().double_value)
        self._heading_deadband_max_deg = float(self.get_parameter('heading_deadband_max_deg').get_parameter_value().double_value)

        self._sim_time_seconds: float | None = None
        self._current_x: float | None = None
        self._current_y: float | None = None
        self._current_heading_deg: float | None = None
        self._speed_mps: float = 0.0

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
        self.clock_rotate = dict(self._stop_pattern)
        self.anticlock_rotate = dict(self._stop_pattern)
        self.clock_rotate.update({
            1: 'f',
            2: 'r',
            3: 'f',
            4: 'r',
        })
        self.anticlock_rotate.update({
            1: 'r',
            2: 'f',
            3: 'r',
            4: 'f',
        })

        self.create_subscription(String, '/time', self._on_time, 10)
        self.create_subscription(PoseStamped, position_topic, self._on_current_position, 10)
        self.create_subscription(Float32, speed_topic, self._on_speed, 10)

        period = 0.1 if control_hz <= 0.0 else (1.0 / control_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'motion_control_node started, control_hz={control_hz}, position_topic={position_topic}, speed_topic={speed_topic}, target_heading_deg={self._target_heading_deg}, deadband=[{self._heading_deadband_min_deg}, {self._heading_deadband_max_deg}]'
        )

    def _on_time(self, msg: String) -> None:
        try:
            self._sim_time_seconds = float(msg.data)
        except Exception:
            pass

    def _on_current_position(self, msg: PoseStamped) -> None:
        self._current_x = float(msg.pose.position.x)
        self._current_y = float(msg.pose.position.y)
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self._current_heading_deg = math.degrees(yaw_rad)

    def _on_speed(self, msg: Float32) -> None:
        try:
            self._speed_mps = float(msg.data)
        except Exception:
            self._speed_mps = 0.0

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

    def _clock_rotate(self) -> None:
        apply_pattern(self._devices, self.clock_rotate)

    def _anticlock_rotate(self) -> None:
        apply_pattern(self._devices, self.anticlock_rotate)

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def _tick(self) -> None:

    #     if self._current_heading_deg is None:
    #         self._stop()
    #         return

    #     heading_deg = self._normalize_angle_deg(self._current_heading_deg)
    #     if self._heading_deadband_min_deg <= heading_deg <= self._heading_deadband_max_deg:
    #         self._stop()
    #         return

    #     angle_error_deg = self._normalize_angle_deg(self._target_heading_deg - heading_deg)
    #     if angle_error_deg > 0.0:
    #         self._clock_rotate()
    #     else:
    #         self._anticlock_rotate()

    # def shutdown_motors(self) -> None:
    #     if hasattr(self, '_devices'):
    #         apply_pattern(self._devices, self._stop_pattern)
    #         close_devices(self._devices)
        self._stop()

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