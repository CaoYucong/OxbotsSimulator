"""Motion control node for the Cytron MDD3A dual DC motor driver.

Hardware:
  - Raspberry Pi 5
  - Cytron MDD3A dual DC motor driver (dual-PWM input logic, NOT PWM+DIR)
  - 2x JGA25-371 12V encoder gear motors
  - 3S LiPo battery -> MDD3A VB+/VB-
  - Separate 5V buck converter powers the Pi

Control wiring (BCM numbering):
  - Left motor : M1A = GPIO18, M1B = GPIO19
  - Right motor: M2A = GPIO12, M2B = GPIO13
  - Pi GND -> MDD3A GND

MDD3A dual-PWM logic (per channel):
  - Forward : MxA = PWM(speed), MxB = 0
  - Reverse : MxA = 0,          MxB = PWM(speed)
  - Brake   : MxA = 0,          MxB = 0

Encoders (BCM numbering, 3.3V supply to protect the Pi):
  - Left  encoder: A = GPIO17, B = GPIO27
  - Right encoder: A = GPIO22, B = GPIO23

Monitor Output:
grep --line-buffered 'motion_control_node'

Wheel state output (for differential-drive odometry):
  - Topic: /wheel_joint_states (sensor_msgs/JointState)
  - Joints: left_wheel_joint, right_wheel_joint
  - position: cumulative wheel angle (rad), forward = positive
  - velocity: wheel angular velocity (rad/s)

Motion status output:
  - Topic: /robot_motion_status (std_msgs/String)
  - 'moving'  when either motor is being driven (non-zero PWM)
  - 'stopped' when neither wheel has a drive command

Waypoint following:
  - Inputs:
      /current_position (geometry_msgs/PoseStamped) -> robot x, y, heading
      /dynamic_waypoint (std_msgs/String '(x, y, theta_deg)') -> target x, y
  - Behavior (face-then-drive):
      * If the heading error to the target is outside +/-5 deg, rotate in place
        to face the target.
      * If within +/-5 deg, drive straight forward.
      * Stop once |dx| and |dy| to the target are both within +/-0.1 m.
  - Waypoint status output:
      /waypoint_status (std_msgs/String)
        'reached' when |dx| and |dy| are both within +/-0.1 m, else 'going'.

"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from gpiozero import PWMOutputDevice, RotaryEncoder
except Exception:
    PWMOutputDevice = None
    RotaryEncoder = None


# --- Motor control pins (BCM numbering, MDD3A channel labels) ---
M1A: int = 18
M1B: int = 19
M2A: int = 12
M2B: int = 13
# Physical wheel -> MDD3A channel (M2=left, M1=right on this robot's motor harness)
LEFT_MOTOR_A: int = M2A
LEFT_MOTOR_B: int = M2B
RIGHT_MOTOR_A: int = M1A
RIGHT_MOTOR_B: int = M1B

# --- Encoder pins (BCM numbering) ---
LEFT_ENC_A: int = 17
LEFT_ENC_B: int = 27
RIGHT_ENC_A: int = 22
RIGHT_ENC_B: int = 23

FULL_SPEED: float = 1.0  # PWM duty cycle for full-speed forward/reverse
ROTATE_RAMP_S: float = 0.1  # soft-start ramp duration after rotate() begins
ROTATE_RAMP_START_SPEED: float = 0.2  # PWM duty cycle at rotate() start
ROTATE_SLOW_SPEED: float = 0.1  # PWM duty cycle when close to target
ROTATE_SLOWDOWN_COUNTS: int = 800  # switch to slow speed when |error| < this
ROTATE_POLL_S: float = 0.01  # closed-loop polling interval for rotate()

COUNTS_PER_REV: float = 488.0  # quadrature counts per output-shaft revolution
WHEEL_DIAMETER: float = 0.047  # wheel diameter in meters
WHEEL_STATE_HZ: float = 50.0  # JointState publish rate for odometry
WHEEL_JOINT_STATE_TOPIC: str = '/wheel_joint_states'
LEFT_WHEEL_JOINT_NAME: str = 'left_wheel_joint'
RIGHT_WHEEL_JOINT_NAME: str = 'right_wheel_joint'
ENCODER_DEBUG_HZ: float = 10.0  # how often to print encoder debug
MOTION_STATUS_TOPIC: str = '/robot_motion_status'  # 'moving' / 'stopped'
MOTION_STATUS_HZ: float = 20.0  # how often to publish robot motion status

# --- Waypoint-following control ---
POSITION_TOPIC: str = '/current_position'  # geometry_msgs/PoseStamped robot pose
WAYPOINT_TOPIC: str = '/dynamic_waypoint'  # std_msgs/String '(x, y, theta_deg)'
WAYPOINT_STATUS_TOPIC: str = '/waypoint_status'  # 'going' / 'reached'
CONTROL_HZ: float = 10.0  # waypoint-following control loop rate
DRIVE_SPEED: float = 0.5  # PWM duty cycle when driving forward toward the waypoint
ROTATE_SPEED: float = 0.4  # PWM duty cycle when rotating in place to face the waypoint
HEADING_TOLERANCE_DEG: float = 5.0  # face the target when |heading error| <= this
POSITION_TOLERANCE_M: float = 0.1  # 'reached' when |dx| and |dy| are both <= this


def _normalize_angle(angle: float) -> float:
    """Wrap an angle (rad) to the range (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _parse_waypoint(text: str) -> tuple[float, float] | None:
    """Parse '(x, y, theta_deg)' (theta optional/None) into (x, y) or None."""
    if not text:
        return None
    cleaned = text.strip().strip('()')
    if not cleaned:
        return None
    parts = [p.strip() for p in cleaned.split(',')]
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except (TypeError, ValueError):
        return None


class Motor:
    """A single MDD3A channel driven in dual-PWM mode."""

    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        encoder=None,
        *,
        invert_encoder: bool = False,
    ) -> None:
        self._a = PWMOutputDevice(pin_a, initial_value=0.0)
        self._b = PWMOutputDevice(pin_b, initial_value=0.0)
        self._enc = encoder
        self._invert_encoder = invert_encoder
        self._rotate_rc: int | None = None
        self._rotate_speed: float = FULL_SPEED
        self._rotate_tolerance: int = 1
        self._rotate_started_at: float | None = None

    @property
    def is_rotating(self) -> bool:
        return self._rotate_rc is not None

    @property
    def is_active(self) -> bool:
        """True when this motor is currently being driven (non-zero PWM output)."""
        return self._a.value > 0.0 or self._b.value > 0.0

    def _encoder_count(self) -> int:
        if self._enc is None:
            raise RuntimeError('Motor has no encoder; cannot rotate to count.')
        steps = int(self._enc.steps)
        return -steps if self._invert_encoder else steps

    def _current_rotate_speed(self, error: int) -> float:
        if abs(error) < ROTATE_SLOWDOWN_COUNTS:
            rotate_speed = ROTATE_SLOW_SPEED + (abs(error) / ROTATE_SLOWDOWN_COUNTS) * (1 - ROTATE_SLOW_SPEED)
            return rotate_speed

        if self._rotate_started_at is not None:
            elapsed = time.monotonic() - self._rotate_started_at
            if elapsed < ROTATE_RAMP_S:
                t = elapsed / ROTATE_RAMP_S
                return (
                    ROTATE_RAMP_START_SPEED
                    + t * (self._rotate_speed - ROTATE_RAMP_START_SPEED)
                )

        return self._rotate_speed

    def rotate(self, rc: int, speed: float = FULL_SPEED, tolerance: int = 1) -> None:
        """Start non-blocking move to encoder count rc (reference count)."""
        self._rotate_rc = rc
        self._rotate_speed = max(0.0, min(1.0, speed))
        self._rotate_tolerance = max(0, tolerance)
        self._rotate_started_at = time.monotonic()
        self.update_rotate()

    def cancel_rotate(self) -> None:
        self._rotate_rc = None
        self._rotate_started_at = None

    def update_rotate(self) -> bool:
        """Advance closed-loop rotate by one step. Returns True when idle or target reached."""
        if self._rotate_rc is None:
            return True

        error = self._rotate_rc - self._encoder_count()
        if abs(error) <= self._rotate_tolerance:
            self._rotate_rc = None
            self._rotate_started_at = None
            self.stop()
            return True

        speed = self._current_rotate_speed(error)
        if error > 0:
            self.forward(speed)
        else:
            self.reverse(speed)
        return False

    def forward(self, speed: float) -> None:
        self._a.value = max(0.0, min(1.0, speed))
        self._b.value = 0.0

    def reverse(self, speed: float) -> None:
        self._a.value = 0.0
        self._b.value = max(0.0, min(1.0, speed))

    def stop(self) -> None:
        self._rotate_rc = None
        self._rotate_started_at = None
        self._a.value = 0.0
        self._b.value = 0.0

    def close(self) -> None:
        try:
            self._a.off()
            self._b.off()
            self._a.close()
            self._b.close()
        except Exception:
            pass


class MotionControlNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_control_node')

        if PWMOutputDevice is None:
            raise RuntimeError('gpiozero is not available. Please install python3-gpiozero on the Raspberry Pi.')

        self.declare_parameter('time_topic', '/time')
        self.declare_parameter('counts_per_rev', COUNTS_PER_REV)
        self.declare_parameter('encoder_debug_hz', ENCODER_DEBUG_HZ)
        self.declare_parameter('wheel_state_hz', WHEEL_STATE_HZ)
        self.declare_parameter('wheel_state_topic', WHEEL_JOINT_STATE_TOPIC)
        self.declare_parameter('motion_status_topic', MOTION_STATUS_TOPIC)
        self.declare_parameter('motion_status_hz', MOTION_STATUS_HZ)
        self.declare_parameter('position_topic', POSITION_TOPIC)
        self.declare_parameter('waypoint_topic', WAYPOINT_TOPIC)
        self.declare_parameter('waypoint_status_topic', WAYPOINT_STATUS_TOPIC)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('drive_speed', DRIVE_SPEED)
        self.declare_parameter('rotate_speed', ROTATE_SPEED)
        self.declare_parameter('heading_tolerance_deg', HEADING_TOLERANCE_DEG)
        self.declare_parameter('position_tolerance_m', POSITION_TOLERANCE_M)
        time_topic = self.get_parameter('time_topic').get_parameter_value().string_value or '/time'
        self._counts_per_rev = float(self.get_parameter('counts_per_rev').get_parameter_value().double_value) or COUNTS_PER_REV
        encoder_debug_hz = float(self.get_parameter('encoder_debug_hz').get_parameter_value().double_value) or ENCODER_DEBUG_HZ
        wheel_state_hz = float(self.get_parameter('wheel_state_hz').get_parameter_value().double_value) or WHEEL_STATE_HZ
        wheel_state_topic = (
            self.get_parameter('wheel_state_topic').get_parameter_value().string_value
            or WHEEL_JOINT_STATE_TOPIC
        )
        motion_status_topic = (
            self.get_parameter('motion_status_topic').get_parameter_value().string_value
            or MOTION_STATUS_TOPIC
        )
        motion_status_hz = (
            float(self.get_parameter('motion_status_hz').get_parameter_value().double_value)
            or MOTION_STATUS_HZ
        )
        position_topic = (
            self.get_parameter('position_topic').get_parameter_value().string_value
            or POSITION_TOPIC
        )
        waypoint_topic = (
            self.get_parameter('waypoint_topic').get_parameter_value().string_value
            or WAYPOINT_TOPIC
        )
        waypoint_status_topic = (
            self.get_parameter('waypoint_status_topic').get_parameter_value().string_value
            or WAYPOINT_STATUS_TOPIC
        )
        control_hz = (
            float(self.get_parameter('control_hz').get_parameter_value().double_value)
            or CONTROL_HZ
        )
        self._drive_speed = (
            float(self.get_parameter('drive_speed').get_parameter_value().double_value)
            or DRIVE_SPEED
        )
        self._rotate_speed = (
            float(self.get_parameter('rotate_speed').get_parameter_value().double_value)
            or ROTATE_SPEED
        )
        self._heading_tolerance_deg = (
            float(self.get_parameter('heading_tolerance_deg').get_parameter_value().double_value)
            or HEADING_TOLERANCE_DEG
        )
        self._position_tolerance_m = (
            float(self.get_parameter('position_tolerance_m').get_parameter_value().double_value)
            or POSITION_TOLERANCE_M
        )

        self._run_enabled = False
        self._motion_started = False
        self._last_motion_status: str | None = None
        self._start_left_steps = 0
        self._start_right_steps = 0
        self._prev_left_count: int | None = None
        self._prev_right_count: int | None = None
        self._prev_wheel_state_time: float | None = None

        # Latest robot pose (/current_position) and target (/dynamic_waypoint).
        self._current_x: float | None = None
        self._current_y: float | None = None
        self._current_theta: float | None = None
        self._target_x: float | None = None
        self._target_y: float | None = None
        self._last_waypoint_status: str | None = None

        # Quadrature encoders (max_steps=0 -> unbounded accumulation).
        self._left_enc = None
        self._right_enc = None
        if RotaryEncoder is not None:
            try:
                self._left_enc = RotaryEncoder(LEFT_ENC_A, LEFT_ENC_B, max_steps=0)
                self._right_enc = RotaryEncoder(RIGHT_ENC_A, RIGHT_ENC_B, max_steps=0)
            except Exception as exc:
                self.get_logger().warn(f'Encoder init failed: {exc}')
        else:
            self.get_logger().warn('gpiozero RotaryEncoder unavailable; encoder debug disabled.')

        # Left encoder count is negated elsewhere on this robot; keep rotate() consistent.
        self._left = Motor(
            LEFT_MOTOR_A, LEFT_MOTOR_B, self._left_enc, invert_encoder=True
        )
        self._right = Motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, self._right_enc)

        self.create_subscription(String, time_topic, self._on_time, 10)
        self.create_subscription(String, '/run', self._on_run, 10)
        self.create_subscription(PoseStamped, position_topic, self._on_current_position, 10)
        self.create_subscription(String, waypoint_topic, self._on_waypoint, 10)

        self._wheel_state_pub = self.create_publisher(JointState, wheel_state_topic, 10)

        self._motion_status_pub = self.create_publisher(String, motion_status_topic, 10)
        status_period = 0.05 if motion_status_hz <= 0.0 else (1.0 / motion_status_hz)
        self.create_timer(status_period, self._publish_motion_status)

        self._waypoint_status_pub = self.create_publisher(String, waypoint_status_topic, 10)
        control_period = 0.1 if control_hz <= 0.0 else (1.0 / control_hz)
        self.create_timer(control_period, self._control_loop)

        if self._left_enc is not None and self._right_enc is not None:
            period = 0.2 if encoder_debug_hz <= 0.0 else (1.0 / encoder_debug_hz)
            self.create_timer(period, self._print_encoders)
            wheel_period = 0.02 if wheel_state_hz <= 0.0 else (1.0 / wheel_state_hz)
            self.create_timer(wheel_period, self._publish_wheel_joint_states)

        self.get_logger().info(
            f'motion_control_node started (MDD3A dual-PWM): '
            f'left=({LEFT_MOTOR_A},{LEFT_MOTOR_B}) right=({RIGHT_MOTOR_A},{RIGHT_MOTOR_B}), '
            f'following {waypoint_topic} from {position_topic} @ {control_hz:.1f} Hz '
            f'(heading_tol={self._heading_tolerance_deg:.1f}deg, pos_tol={self._position_tolerance_m:.2f}m, '
            f'drive={self._drive_speed:.2f}, rotate={self._rotate_speed:.2f}), '
            f'time_topic={time_topic}, counts_per_rev={self._counts_per_rev}, '
            f'wheel_state={wheel_state_topic} @ {wheel_state_hz:.1f} Hz, '
            f'motion_status={motion_status_topic} @ {motion_status_hz:.1f} Hz, '
            f'waypoint_status={waypoint_status_topic}'
        )

    def _on_current_position(self, msg: PoseStamped) -> None:
        self._current_x = float(msg.pose.position.x)
        self._current_y = float(msg.pose.position.y)

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self._current_theta = math.atan2(siny_cosp, cosy_cosp)

    def _on_waypoint(self, msg: String) -> None:
        parsed = _parse_waypoint(msg.data if msg.data else '')
        if parsed is None:
            return
        self._target_x, self._target_y = parsed

    def _control_loop(self) -> None:
        """Face-then-drive controller toward /dynamic_waypoint, with waypoint_status."""
        if not self._run_enabled:
            self._stop()
            return

        have_pose = (
            self._current_x is not None
            and self._current_y is not None
            and self._current_theta is not None
        )
        have_waypoint = self._target_x is not None and self._target_y is not None
        if not have_pose or not have_waypoint:
            self._stop()
            return

        dx = self._target_x - self._current_x
        dy = self._target_y - self._current_y

        # 'reached' when both x and y are within tolerance; otherwise still 'going'.
        if abs(dx) <= self._position_tolerance_m and abs(dy) <= self._position_tolerance_m:
            self._publish_waypoint_status('reached')
            self._stop()
            return

        self._publish_waypoint_status('going')

        # Heading error toward the target point, wrapped to (-pi, pi].
        desired_heading = math.atan2(dy, dx)
        heading_error = _normalize_angle(desired_heading - self._current_theta)

        if abs(heading_error) > math.radians(self._heading_tolerance_deg):
            # Rotate in place to face the target. Positive error = CCW (turn left):
            # right wheel forward, left wheel reverse (matches odometry yaw sign).
            if heading_error > 0.0:
                self._left.reverse(self._rotate_speed)
                self._right.forward(self._rotate_speed)
            else:
                self._left.forward(self._rotate_speed)
                self._right.reverse(self._rotate_speed)
        else:
            # Within the heading deadband: drive straight toward the target.
            self._left.forward(self._drive_speed)
            self._right.forward(self._drive_speed)

    def _publish_waypoint_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._waypoint_status_pub.publish(msg)

        if status != self._last_waypoint_status:
            self._last_waypoint_status = status
            self.get_logger().info(f'[waypoint_status] {status}')

    def _publish_motion_status(self) -> None:
        """Publish 'moving' if either motor is being driven, else 'stopped'."""
        moving = self._left.is_active or self._right.is_active
        status = 'moving' if moving else 'stopped'

        msg = String()
        msg.data = status
        self._motion_status_pub.publish(msg)

        if status != self._last_motion_status:
            self._last_motion_status = status
            self.get_logger().info(f'[motion_status] {status}')

    def _signed_wheel_counts(self) -> tuple[int, int]:
        """Encoder counts in robot frame (forward motion = positive for both wheels)."""
        left = -int(self._left_enc.steps)
        right = int(self._right_enc.steps)
        return left, right

    def _count_to_rad(self, count: int) -> float:
        return (count / self._counts_per_rev) * (2.0 * math.pi)

    def _wheel_revolutions(self) -> tuple[float, float]:
        left_steps = self._left_enc.steps - self._start_left_steps
        right_steps = self._right_enc.steps - self._start_right_steps
        return (
            left_steps / self._counts_per_rev,
            right_steps / self._counts_per_rev,
        )

    def _publish_wheel_joint_states(self) -> None:
        if self._left_enc is None or self._right_enc is None:
            return

        left_count, right_count = self._signed_wheel_counts()
        stamp = self.get_clock().now()
        now_s = stamp.nanoseconds * 1e-9

        left_vel = 0.0
        right_vel = 0.0
        if (
            self._prev_wheel_state_time is not None
            and self._prev_left_count is not None
            and self._prev_right_count is not None
        ):
            dt = now_s - self._prev_wheel_state_time
            if dt > 0.0:
                left_vel = self._count_to_rad(left_count - self._prev_left_count) / dt
                right_vel = self._count_to_rad(right_count - self._prev_right_count) / dt

        self._prev_left_count = left_count
        self._prev_right_count = right_count
        self._prev_wheel_state_time = now_s

        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name = [LEFT_WHEEL_JOINT_NAME, RIGHT_WHEEL_JOINT_NAME]
        msg.position = [
            self._count_to_rad(left_count),
            self._count_to_rad(right_count),
        ]
        msg.velocity = [left_vel, right_vel]
        self._wheel_state_pub.publish(msg)

    def _print_encoders(self) -> None:
        if self._left_enc is None or self._right_enc is None:
            return
        left_rev, right_rev = self._wheel_revolutions()
        left_steps = self._left_enc.steps
        right_steps = self._right_enc.steps
        left_rev_display = -left_rev
        left_cnt = -left_steps
        if self._motion_started:
            self.get_logger().info(
                f'[encoder] left={left_rev_display:+.3f} rev ({left_cnt} cnt), '
                f'right={right_rev:+.3f} rev ({right_steps} cnt)'
            )
        else:
            self.get_logger().info(
                f'[encoder] left={left_cnt} cnt, right={right_steps} cnt (idle)'
            )

    def _reset_motion_state(self) -> None:
        self._motion_started = False
        self._start_left_steps = 0
        self._start_right_steps = 0

    def _on_run(self, msg: String) -> None:
        enabled = (msg.data or '').strip().lower() != 'off'
        if not enabled:
            self._run_enabled = False
            self._reset_motion_state()
            self._stop()
            return
        self._run_enabled = True

    def _on_time(self, msg: String) -> None:
        if not self._run_enabled:
            return

        try:
            t = float(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Ignoring non-numeric /time message: {msg.data!r}')
            return

        if t <= 0.0:
            self._reset_motion_state()
            self._stop()
            return

        if not self._motion_started:
            if self._left_enc is not None and self._right_enc is not None:
                self._start_left_steps = self._left_enc.steps
                self._start_right_steps = self._right_enc.steps
            self._motion_started = True
            self.get_logger().info('/time > 0: waypoint-following control active')

    def _stop(self) -> None:
        self._left.stop()
        self._right.stop()

    def shutdown_motors(self) -> None:
        self._stop()
        self._left.close()
        self._right.close()
        for enc in (self._left_enc, self._right_enc):
            if enc is not None:
                try:
                    enc.close()
                except Exception:
                    pass


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
