import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

try:
    from gpiozero import AngularServo, DigitalOutputDevice, PWMOutputDevice
except Exception:
    AngularServo = None
    DigitalOutputDevice = None
    PWMOutputDevice = None

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None

SERVO_PIN: int = 12
SCOOP_OPEN_ANGLE: float = 5.0   # servo angle to open the scoop
SCOOP_CLOSED_ANGLE: float = 179.0  # servo angle when closed

MOTION_DEBUG: bool = False  # set True to enable nav debug logs


FORWARD_SPEED: float = 0.7            # PWM duty cycle for forward movement (wheels 1-4)
LEFT_FORWARD_SPEED: float = 0.7       # PWM duty cycle for forward movement — left wheels (2, 4)
RIGHT_FORWARD_SPEED: float = 0.7      # PWM duty cycle for forward movement — right wheels (1, 3)
BACKWARD_SPEED: float = 0.7           # PWM duty cycle for backward movement (wheels 1-4)
LEFT_STRAFE_SPEED: float = 0.8        # PWM duty cycle for left strafe
RIGHT_STRAFE_SPEED: float = 0.8       # PWM duty cycle for right strafe
CLOCK_ROTATE_FRONT_SPEED: float = 1.0  # PWM duty cycle for clockwise rotation — front wheels (1, 2)
CLOCK_ROTATE_REAR_SPEED: float = 1.0   # PWM duty cycle for clockwise rotation — rear wheels (3, 4)
ANTICLOCK_ROTATE_FRONT_SPEED: float = 1.0  # PWM duty cycle for anti-clockwise rotation — front wheels (1, 2)
ANTICLOCK_ROTATE_REAR_SPEED: float = 1.0   # PWM duty cycle for anti-clockwise rotation — rear wheels (3, 4)
ROTATE_BURST_SMALL_MS: float = 20.0   # burst duration for heading error < 30 deg
ROTATE_BURST_MED_MS: float = 40.0     # burst duration for heading error 30–60 deg
ROTATE_BURST_LARGE_MS: float = 70.0  # burst duration for heading error > 60 deg

WHEELS: Dict[int, Tuple[int, int]] = {
    1: (5, 6),
    2: (13, 19),
    3: (17, 18),
    4: (22, 23),
    5: (24, 25),
    6: (4, 27),
}


def set_wheel_state(devices: dict, wheel_id: int, state: str, speed: float = FORWARD_SPEED) -> None:
    a, b = devices[wheel_id]
    is_pwm = PWMOutputDevice is not None and isinstance(a, PWMOutputDevice)
    s = speed if is_pwm else 1
    if state == 'f':
        a.value = s
        b.value = 0
    elif state == 'r':
        a.value = 0
        b.value = s
    else:
        a.value = 0
        b.value = 0


def apply_pattern(
    devices: dict,
    pattern: dict,
    speed: float = FORWARD_SPEED,
    speed_map: Optional[Dict[int, float]] = None,
) -> None:
    for wheel_id, state in pattern.items():
        s = speed_map[wheel_id] if (speed_map and wheel_id in speed_map) else speed
        set_wheel_state(devices, wheel_id, state, s)


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
        self.declare_parameter('scoop_state_topic', '/scoop_state')
        self.declare_parameter('num_tags_detected_topic', '/num_tags_detected')
        self.declare_parameter('target_heading_deg', 10.0)
        self.declare_parameter('heading_deadband_min_deg', 9.0)
        self.declare_parameter('heading_deadband_max_deg', 11.0)

        control_hz = float(self.get_parameter('control_hz').get_parameter_value().double_value)
        position_topic = self.get_parameter('position_topic').get_parameter_value().string_value or '/current_position'
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value or '/speed'
        dynamic_waypoint_topic = self.get_parameter('dynamic_waypoint_topic').get_parameter_value().string_value or '/dynamic_waypoint'
        collision_avoiding_waypoint_topic = self.get_parameter('collision_avoiding_waypoint_topic').get_parameter_value().string_value or '/collision_avoiding_waypoint'
        waypoint_status_topic = self.get_parameter('waypoint_status_topic').get_parameter_value().string_value or '/waypoint_status'
        scoop_state_topic = self.get_parameter('scoop_state_topic').get_parameter_value().string_value or '/scoop_state'
        num_tags_detected_topic = self.get_parameter('num_tags_detected_topic').get_parameter_value().string_value or '/num_tags_detected'
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
        self._dynamic_waypoints_type: str = ''
        self._destination_waypoint_type: str = ''
        self._num_tags_detected: Optional[int] = None
        self._scoop_state: Optional[str] = None

        if DigitalOutputDevice is None:
            raise RuntimeError('gpiozero is not available. Please install python3-gpiozero on Raspberry Pi.')

        self._servo = None
        self._servo_backend = 'none'
        # Prefer gpiozero AngularServo (works with lgpio on Pi OS Bookworm/Pi 5).
        if AngularServo is not None:
            try:
                self._servo = AngularServo(
                    SERVO_PIN,
                    min_angle=0,
                    max_angle=180,
                    min_pulse_width=0.0005,
                    max_pulse_width=0.0025,
                    frame_width=0.02,
                )
                self._servo_backend = 'gpiozero'
            except Exception as exc:
                self.get_logger().warn(f'AngularServo init failed on GPIO{SERVO_PIN}: {exc}')

        # Fallback for older images that still provide RPi.GPIO.
        if self._servo is None and GPIO is not None:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(SERVO_PIN, GPIO.OUT)
                self._servo = GPIO.PWM(SERVO_PIN, 50)
                self._servo.start(0)
                self._servo_backend = 'rpi_gpio'
            except Exception as exc:
                self.get_logger().warn(f'RPi.GPIO servo init failed on GPIO{SERVO_PIN}: {exc}')

        if self._servo is None:
            self.get_logger().warn('Scoop servo is disabled: no usable GPIO backend found.')

        # All wheels use PWM; wheels 5 & 6 are held at full speed (1.0) by set_wheel_state

        self._devices = {}
        for wheel_id, (pin_a, pin_b) in WHEELS.items():
            self._devices[wheel_id] = (
                PWMOutputDevice(pin_a, initial_value=0),
                PWMOutputDevice(pin_b, initial_value=0),
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
        self.create_subscription(String, '/dynamic_waypoints_type', self._on_dynamic_waypoints_type, 10)
        self.create_subscription(Int32, num_tags_detected_topic, self._on_num_tags_detected, 10)
        self._pub_waypoint_status = self.create_publisher(String, waypoint_status_topic, 10)
        self._pub_scoop_state = self.create_publisher(String, scoop_state_topic, 10)

        # Force scoop to closed at startup to avoid inheriting previous servo position.
        self._close_scoop()

        period = 0.1 if control_hz <= 0.0 else (1.0 / control_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'motion_control_node started, control_hz={control_hz}, position_topic={position_topic}, speed_topic={speed_topic}, dynamic_waypoint_topic={dynamic_waypoint_topic}, collision_avoiding_waypoint_topic={collision_avoiding_waypoint_topic}, waypoint_status_topic={waypoint_status_topic}, scoop_state_topic={scoop_state_topic}, target_heading_deg={self._target_heading_deg}, deadband=[{self._heading_deadband_min_deg}, {self._heading_deadband_max_deg}]'
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

    def _on_num_tags_detected(self, msg: Int32) -> None:
        self._num_tags_detected = int(msg.data)

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
            self._destination_waypoint_type = 'collision'
            return
        self._destination_waypoint = self._dynamic_waypoint
        self._destination_waypoint_type = self._dynamic_waypoints_type

    @staticmethod
    def _normalize_waypoint_type(raw: str) -> str:
        """Normalise legacy 'ball' to 'pingball'; 'steelball' and 'pingball' pass through."""
        t = (raw or '').strip().lower()
        if t == 'ball':
            return 'pingball'
        return t

    def _on_dynamic_waypoints_type(self, msg: String) -> None:
        self._dynamic_waypoints_type = self._normalize_waypoint_type(msg.data)
        self._update_destination_waypoint()

    def _on_dynamic_waypoint(self, msg: String) -> None:
        self._dynamic_waypoint = self._parse_waypoint_text(msg.data)
        self._update_destination_waypoint()

    def _on_collision_avoiding_waypoint(self, msg: String) -> None:
        self._collision_avoiding_waypoint = self._parse_waypoint_text(msg.data)
        self._update_destination_waypoint()

    def _set_scoop_angle(self, angle: float) -> None:
        if self._servo is not None:
            if self._servo_backend == 'gpiozero':
                clipped = max(0.0, min(180.0, float(angle)))
                self._servo.angle = clipped
            else:
                duty = 2.0 + (angle / 18.0)
                self._servo.ChangeDutyCycle(duty)

    def _publish_scoop_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._pub_scoop_state.publish(msg)

    def _depower_scoop(self) -> None:
        if self._servo is None:
            return
        try:
            if self._servo_backend == 'gpiozero':
                # Stop PWM pulses to reduce holding jitter.
                self._servo.detach()
            else:
                # Keep PWM instance alive, but output 0 duty to depower the servo signal.
                self._servo.ChangeDutyCycle(0)
        except Exception:
            pass

    def _set_scoop_state(self, state: str) -> bool:
        target = (state or '').strip().lower()
        if target not in ('open', 'closed'):
            return False
        if self._scoop_state == target:
            return False

        if target == 'open':
            self._set_scoop_angle(SCOOP_OPEN_ANGLE)
        else:
            self._set_scoop_angle(SCOOP_CLOSED_ANGLE)

        # Allow servo time to reach target, then depower to avoid jitter.
        time.sleep(1.0)
        self._depower_scoop()

        self._scoop_state = target
        self._publish_scoop_state(target)
        return True

    def _open_scoop(self) -> None:
        self._set_scoop_state('open')

    def _close_scoop(self) -> None:
        self._set_scoop_state('closed')

    def _nav_log(self, msg: str) -> None:
        if MOTION_DEBUG:
            self.get_logger().info(msg)

    def _log_motor_states(self, label: str) -> None:
        pass  # disabled; re-enable by restoring the log call below
        # parts = []
        # for wid, (a, b) in sorted(self._devices.items()):
        #     parts.append(f'W{wid}(a={a.value:.2f},b={b.value:.2f})')
        # self.get_logger().info(f'[motor] {label}: {" ".join(parts)}')

    def _move_forward(
        self,
        left_speed: float = LEFT_FORWARD_SPEED,
        right_speed: float = RIGHT_FORWARD_SPEED,
    ) -> None:
        apply_pattern(
            self._devices,
            self._forward_pattern,
            speed_map={1: right_speed, 2: left_speed, 3: right_speed, 4: left_speed, 5: 1.0, 6: 1.0},
        )
        self._log_motor_states('forward')

    def _move_backward(self) -> None:
        apply_pattern(self._devices, self._reverse_pattern, BACKWARD_SPEED, speed_map={5: 1.0, 6: 1.0})
        self._log_motor_states('backward')

    def _stop(self) -> None:
        apply_pattern(self._devices, self._stop_pattern, speed_map={5: 1.0, 6: 1.0})
        self._log_motor_states('stop')

    def _move_left(self) -> None:
        apply_pattern(self._devices, self._left_strafe_pattern, LEFT_STRAFE_SPEED, speed_map={5: 1.0, 6: 1.0})
        self._log_motor_states('left')

    def _move_right(self) -> None:
        apply_pattern(self._devices, self._right_strafe_pattern, RIGHT_STRAFE_SPEED, speed_map={5: 1.0, 6: 1.0})
        self._log_motor_states('right')

    def _clock_rotate(self) -> None:
        apply_pattern(
            self._devices,
            self.clock_rotate,
            speed_map={1: CLOCK_ROTATE_FRONT_SPEED, 2: CLOCK_ROTATE_FRONT_SPEED,
                       3: CLOCK_ROTATE_REAR_SPEED, 4: CLOCK_ROTATE_REAR_SPEED, 5: 1.0, 6: 1.0},
        )
        self._log_motor_states('clock_rotate')

    def _anticlock_rotate(self) -> None:
        apply_pattern(
            self._devices,
            self.anticlock_rotate,
            speed_map={1: ANTICLOCK_ROTATE_FRONT_SPEED, 2: ANTICLOCK_ROTATE_FRONT_SPEED,
                       3: ANTICLOCK_ROTATE_REAR_SPEED, 4: ANTICLOCK_ROTATE_REAR_SPEED, 5: 1.0, 6: 1.0},
        )
        self._log_motor_states('anticlock_rotate')

    def _rotate_burst(self, clockwise: bool, heading_error_deg: float = 0.0) -> None:
        abs_err = abs(heading_error_deg)
        if abs_err < 30.0:
            burst_ms = ROTATE_BURST_SMALL_MS
        elif abs_err < 60.0:
            burst_ms = ROTATE_BURST_MED_MS
        else:
            burst_ms = ROTATE_BURST_LARGE_MS
        if clockwise:
            self._clock_rotate()
        else:
            self._anticlock_rotate()
        time.sleep(burst_ms / 1000.0)
        self._stop()

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
                math.hypot(dest_x - self._current_x, dest_y - self._current_y) <= 0.15
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
        # Highest priority: no AprilTag visible — reverse unconditionally to find a tag.
        if self._num_tags_detected is not None and self._num_tags_detected == 0 and self._sim_time_seconds is not None and self._sim_time_seconds > 20.0:
            self._move_backward()
            self._publish_waypoint_status()
            return

        self._publish_waypoint_status()
        if self._destination_waypoint is None:
            self._stop()
            return

        if self._current_x is None or self._current_y is None or self._current_heading_deg is None:
            return

        dest_x, dest_y, _ = self._destination_waypoint
        dx = dest_x - self._current_x
        dy = dest_y - self._current_y
        distance = math.hypot(dx, dy)

        if distance <= 0.1:
            self._stop()
            return

        self._nav_log(
            f'[nav] pos=({self._current_x:.3f}, {self._current_y:.3f}) heading={self._current_heading_deg:.1f}° '
            f'dest=({dest_x:.3f}, {dest_y:.3f}) dist={distance:.3f} type={self._destination_waypoint_type}'
        )

        if abs(dest_x) <= 0.7 and abs(dest_y) <= 0.7:
            # --- Waypoint inside field (|x| <= 0.7 and |y| <= 0.7): rotate to face target, then move forward ---
            target_heading_deg = math.degrees(math.atan2(dy, dx))
            heading_error_deg = -self._normalize_angle_deg(target_heading_deg - self._current_heading_deg)
            self._nav_log(
                f'[nav:inside] target_heading={target_heading_deg:.1f}° heading_error={heading_error_deg:.1f}°'
            )
            if heading_error_deg > 10.0:
                self._nav_log(f'[nav:inside] rotating anti-clockwise (error={heading_error_deg:.1f}°)')
                self._rotate_burst(clockwise=False, heading_error_deg=heading_error_deg)
            elif heading_error_deg < -10.0:
                self._nav_log(f'[nav:inside] rotating clockwise (error={heading_error_deg:.1f}°)')
                self._rotate_burst(clockwise=True, heading_error_deg=heading_error_deg)
            else:
                self._nav_log('[nav:inside] moving forward')
                self._move_forward(left_speed=LEFT_FORWARD_SPEED, right_speed=RIGHT_FORWARD_SPEED)
        else:
            # --- Waypoint outside field (|x| > 0.65 or |y| > 0.65) ---
            # Intermediate point: clamp each coordinate to [-0.65, 0.65]
            mid_x = max(-0.65, min(0.65, dest_x))
            mid_y = max(-0.65, min(0.65, dest_y))
            self._nav_log(f'[nav:outside] mid=({mid_x:.3f}, {mid_y:.3f})')
            # Step 1: navigate to intermediate point first
            if not (abs(self._current_x - mid_x) < 0.1 and abs(self._current_y - mid_y) < 0.1):
                dx_mid = mid_x - self._current_x
                dy_mid = mid_y - self._current_y
                target_heading_deg = math.degrees(math.atan2(dy_mid, dx_mid))
                heading_error_deg = -self._normalize_angle_deg(target_heading_deg - self._current_heading_deg)
                self._nav_log(
                    f'[nav:outside:step1] heading_to_mid={target_heading_deg:.1f}° error={heading_error_deg:.1f}°'
                )
                if heading_error_deg > 10.0:
                    self._nav_log(f'[nav:outside:step1] rotating anti-clockwise')
                    self._rotate_burst(clockwise=False, heading_error_deg=heading_error_deg)
                elif heading_error_deg < -10.0:
                    self._nav_log(f'[nav:outside:step1] rotating clockwise')
                    self._rotate_burst(clockwise=True, heading_error_deg=heading_error_deg)
                else:
                    self._nav_log('[nav:outside:step1] moving forward to mid')
                    self._move_forward(left_speed=LEFT_FORWARD_SPEED, right_speed=RIGHT_FORWARD_SPEED)
            else:
                # Step 2: at intermediate point — approach wall
                self._nav_log(f'[nav:outside:step2] at mid, type={self._destination_waypoint_type}')
                if self._destination_waypoint_type == 'home':
                    # Face away from wall (back toward wall), then reverse 1 s
                    target_heading_deg = math.degrees(math.atan2(dy, dx)) + 180.0
                    heading_error_deg = -self._normalize_angle_deg(target_heading_deg - self._current_heading_deg)
                    self._nav_log(
                        f'[nav:home] face_away_heading={target_heading_deg:.1f}° error={heading_error_deg:.1f}°'
                    )
                    if heading_error_deg > 20.0:
                        self._nav_log('[nav:home] rotating anti-clockwise to face away')
                        self._rotate_burst(clockwise=False, heading_error_deg=heading_error_deg)
                    elif heading_error_deg < -20.0:
                        self._nav_log('[nav:home] rotating clockwise to face away')
                        self._rotate_burst(clockwise=True, heading_error_deg=heading_error_deg)
                    else:
                        self._nav_log('[nav:home] reversing into wall')
                        self._move_backward()
                        time.sleep(2.0)
                        # self._stop()
                        self._nav_log('[nav:home] opening scoop')
                        self._open_scoop()
                        time.sleep(1.0)
                        self._nav_log('[nav:home] closing scoop')
                        self._close_scoop()
                        time.sleep(1.0)
                        self._nav_log('[nav:home] publishing reached')
                        reached_msg = String()
                        reached_msg.data = 'reached'
                        self._pub_waypoint_status.publish(reached_msg)
                else:
                    # Face the wall, full forward 1 s, then signal reached
                    target_heading_deg = math.degrees(math.atan2(dy, dx))
                    heading_error_deg = -self._normalize_angle_deg(target_heading_deg - self._current_heading_deg)
                    self._nav_log(
                        f'[nav:wall] face_wall_heading={target_heading_deg:.1f}° error={heading_error_deg:.1f}°'
                    )
                    if heading_error_deg > 10.0:
                        self._nav_log('[nav:wall] rotating anti-clockwise to face wall')
                        self._rotate_burst(clockwise=False, heading_error_deg=heading_error_deg)
                    elif heading_error_deg < -10.0:
                        self._nav_log('[nav:wall] rotating clockwise to face wall')
                        self._rotate_burst(clockwise=True, heading_error_deg=heading_error_deg)
                    else:
                        self._nav_log('[nav:wall] driving into wall')
                        self._move_forward(left_speed=0.8, right_speed=0.8)
                        time.sleep(2.0)
                        self._stop()
                        self._nav_log('[nav:wall] publishing reached')
                        reached_msg = String()
                        reached_msg.data = 'reached'
                        self._pub_waypoint_status.publish(reached_msg)

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
        if self._servo is not None:
            try:
                if self._servo_backend == 'gpiozero':
                    self._servo.detach()
                    self._servo.close()
                else:
                    self._servo.stop()
            except Exception:
                pass
        if GPIO is not None and self._servo_backend == 'rpi_gpio':
            try:
                GPIO.cleanup(SERVO_PIN)
            except Exception:
                pass
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