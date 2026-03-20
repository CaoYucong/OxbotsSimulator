import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice
except Exception:
    DigitalOutputDevice = None
    PWMOutputDevice = None


DRIVE_WHEEL_SPEED: float = 0.8  # PWM duty cycle for drive wheels 1-4 (0.0 – 1.0)

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
    is_pwm = PWMOutputDevice is not None and isinstance(a, PWMOutputDevice)
    speed = DRIVE_WHEEL_SPEED if is_pwm else 1
    if state == 'f':
        a.value = speed
        b.value = 0
    elif state == 'r':
        a.value = 0
        b.value = speed
    else:
        a.value = 0
        b.value = 0


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
        self.declare_parameter('dynamic_waypoint_topic', '/dynamic_waypoint')
        self.declare_parameter('collision_avoiding_waypoint_topic', '/collision_avoiding_waypoint')
        self.declare_parameter('waypoint_status_topic', '/waypoint_status')
        self.declare_parameter('target_heading_deg', 10.0)
        self.declare_parameter('heading_deadband_min_deg', 9.0)
        self.declare_parameter('heading_deadband_max_deg', 11.0)

        control_hz = float(self.get_parameter('control_hz').get_parameter_value().double_value)
        position_topic = self.get_parameter('position_topic').get_parameter_value().string_value or '/current_position'
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value or '/speed'
        dynamic_waypoint_topic = self.get_parameter('dynamic_waypoint_topic').get_parameter_value().string_value or '/dynamic_waypoint'
        collision_avoiding_waypoint_topic = self.get_parameter('collision_avoiding_waypoint_topic').get_parameter_value().string_value or '/collision_avoiding_waypoint'
        waypoint_status_topic = self.get_parameter('waypoint_status_topic').get_parameter_value().string_value or '/waypoint_status'
        self._target_heading_deg = float(self.get_parameter('target_heading_deg').get_parameter_value().double_value)
        self._heading_deadband_min_deg = float(self.get_parameter('heading_deadband_min_deg').get_parameter_value().double_value)
        self._heading_deadband_max_deg = float(self.get_parameter('heading_deadband_max_deg').get_parameter_value().double_value)

        self._sim_time_seconds: float | None = None
        self._start_mono_seconds = time.monotonic()
        self._warned_missing_time = False
        self._current_x: float | None = None
        self._current_y: float | None = None
        self._current_heading_deg: float | None = None
        self._speed_mps: float = 0.0
        self._dynamic_waypoint: tuple[float, float, Optional[float]] | None = None
        self._collision_avoiding_waypoint: tuple[float, float, Optional[float]] | None = None
        self._destination_waypoint: tuple[float, float, Optional[float]] | None = None

        if DigitalOutputDevice is None:
            raise RuntimeError('gpiozero is not available. Please install python3-gpiozero on Raspberry Pi.')

        # Wheels 1-4: PWM drive wheels at DRIVE_WHEEL_SPEED
        # Wheels 5 & 6: always-on digital wheels (full speed)
        ALWAYS_ON_WHEELS = {5, 6}
        DRIVE_WHEELS = {1, 2, 3, 4}

        self._devices = {}
        for wheel_id, (pin_a, pin_b) in WHEELS.items():
            if wheel_id in DRIVE_WHEELS:
                self._devices[wheel_id] = (
                    PWMOutputDevice(pin_a, initial_value=0),
                    PWMOutputDevice(pin_b, initial_value=0),
                )
            else:
                self._devices[wheel_id] = (
                    DigitalOutputDevice(pin_a, initial_value=(wheel_id in ALWAYS_ON_WHEELS)),
                    DigitalOutputDevice(pin_b, initial_value=False),
                )

        self._forward_pattern = {wheel_id: 'f' for wheel_id in WHEELS}
        self._reverse_pattern = {wheel_id: 'r' for wheel_id in WHEELS}
        self._reverse_pattern[5] = 'f'  # wheel 5 (pins 24, 25) always spins forward
        self._reverse_pattern[6] = 'f'  # wheel 6 (pins 27, 4) always spins forward
        self._stop_pattern = {wheel_id: 's' for wheel_id in WHEELS}
        self._stop_pattern[5] = 'f'  # wheel 5 (pins 24, 25) always spins forward
        self._stop_pattern[6] = 'f'  # wheel 6 (pins 27, 4) always spins forward
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
        self.create_subscription(String, dynamic_waypoint_topic, self._on_dynamic_waypoint, 10)
        self.create_subscription(String, collision_avoiding_waypoint_topic, self._on_collision_avoiding_waypoint, 10)
        self._pub_waypoint_status = self.create_publisher(String, waypoint_status_topic, 10)

        period = 0.1 if control_hz <= 0.0 else (1.0 / control_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'motion_control_node started, control_hz={control_hz}, position_topic={position_topic}, speed_topic={speed_topic}, dynamic_waypoint_topic={dynamic_waypoint_topic}, collision_avoiding_waypoint_topic={collision_avoiding_waypoint_topic}, waypoint_status_topic={waypoint_status_topic}, target_heading_deg={self._target_heading_deg}, deadband=[{self._heading_deadband_min_deg}, {self._heading_deadband_max_deg}]'
        )

    def _on_time(self, msg: String) -> None:
        try:
            self._sim_time_seconds = float(msg.data)
            self._warned_missing_time = False
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

    @staticmethod
    def _parse_waypoint_text(text: str) -> tuple[float, float, Optional[float]] | None:
        raw = (text or '').strip()
        if not raw:
            return None
        if raw.startswith('(') and raw.endswith(')'):
            raw = raw[1:-1]
        parts = [part.strip() for part in raw.split(',')]
        if len(parts) < 2:
            return None
        try:
            x = float(parts[0])
            y = float(parts[1])
        except Exception:
            return None
        theta: Optional[float] = None
        if len(parts) >= 3:
            theta_text = parts[2]
            if theta_text and theta_text.lower() != 'none':
                try:
                    theta = float(theta_text)
                except Exception:
                    theta = None
        return (x, y, theta)

    def _update_destination_waypoint(self) -> None:
        wp = self._collision_avoiding_waypoint
        if wp is not None and not (wp[0] == 0.0 and wp[1] == 0.0 and (wp[2] is None or wp[2] == 0.0)):
            self._destination_waypoint = wp
            return
        self._destination_waypoint = self._dynamic_waypoint

    def _on_dynamic_waypoint(self, msg: String) -> None:
        self._dynamic_waypoint = self._parse_waypoint_text(msg.data)
        self._update_destination_waypoint()

    def _on_collision_avoiding_waypoint(self, msg: String) -> None:
        self._collision_avoiding_waypoint = self._parse_waypoint_text(msg.data)
        self._update_destination_waypoint()

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

    def _publish_waypoint_status(self) -> None:
        status = 'going'

        if (
            self._destination_waypoint is not None
            and self._current_x is not None
            and self._current_y is not None
        ):
            dest_x, dest_y, dest_heading_deg = self._destination_waypoint
            reached_xy = (
                abs(self._current_x - dest_x) <= 0.1
                and abs(self._current_y - dest_y) <= 0.1
            )

            status = 'reached' if reached_xy else 'going'

        cur_x_text = 'None' if self._current_x is None else f'{self._current_x:.3f}'
        cur_y_text = 'None' if self._current_y is None else f'{self._current_y:.3f}'
        cur_heading_text = 'None' if self._current_heading_deg is None else f'{self._current_heading_deg:.1f}'
        if self._destination_waypoint is None:
            dest_text = 'None'
        else:
            dx, dy, dtheta = self._destination_waypoint
            dtheta_text = 'None' if dtheta is None else f'{float(dtheta):.1f}'
            dest_text = f'({dx:.3f}, {dy:.3f}, {dtheta_text})'

        # self.get_logger().info(
        #     f'[waypoint_status_debug] current=({cur_x_text}, {cur_y_text}, {cur_heading_text}), '
        #     f'destination={dest_text}, status={status}'
        # )

        msg = String()
        msg.data = status
        self._pub_waypoint_status.publish(msg)

    def _tick(self) -> None:
        self._publish_waypoint_status()
        if self._current_x is None or self._current_y is None or self._current_heading_deg is None:
            # No AprilTag visible — reverse at full speed to find a tag
            self._move_backward()
            return
        if self._destination_waypoint is None:
            self._stop()
            return

        dest_x, dest_y, _ = self._destination_waypoint
        dx = dest_x - self._current_x
        dy = dest_y - self._current_y
        distance = math.hypot(dx, dy)

        if distance <= 0.1:
            self._stop()
            return

        target_heading_deg = math.degrees(math.atan2(dy, dx))
        heading_error_deg = self._normalize_angle_deg(target_heading_deg - self._current_heading_deg)

        if heading_error_deg > 15.0:
            self._clock_rotate()
        elif heading_error_deg < -15.0:
            self._anticlock_rotate()
        else:
            self._move_forward()

        # sim_time = self._sim_time_seconds
        # if sim_time is None:
        #     sim_time = time.monotonic() - self._start_mono_seconds
        #     if not self._warned_missing_time:
        #         self.get_logger().warn('No /time message received yet; falling back to local monotonic clock for rotation sequence.')
        #         self._warned_missing_time = True

        # if sim_time < 20.0:
        #     if sim_time % 4.0 < 2.0:
        #         self._clock_rotate()
        #     else:
        #         self._anticlock_rotate()
        # else:
        #     self._stop()

    def shutdown_motors(self) -> None:
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