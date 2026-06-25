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

Servo (BCM numbering):
  - Intake / mechanism servo: GPIO21
  - PWM is sent only while moving to a new angle, then the pin is released.

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
      /dynamic_waypoint (std_msgs/String '(x, y, theta_deg)') -> task target x, y
      /collision_avoiding_waypoint (std_msgs/String, optional) -> short-range escape
        target. When non-empty, motion control pursues it to 'reached' before resuming
        /dynamic_waypoint; if the escape point moves during pursuit, the target updates.
        After the escape point is reached, hold stopped for 2 s before publishing 'reached'.
  - Behavior (heading-gated drive toward each waypoint):
      * Rotate in place (proximity PWM + align hysteresis) until |heading error| is
        within heading_tolerance_deg (1 deg).
      * After heading enters tolerance, pause 1 s; if heading drifts out during
        that interval, rotate again instead of driving forward.
      * Drive forward with proportional differential steering (drive_heading_kp) to
        correct small heading drift without stopping.
      * Only stop to re-rotate in place when |heading error| exceeds
        drive_rotate_heading_deg while driving (far) or near_target_heading_tolerance_deg
        when within near_target_distance_m.
      * Within near_target_distance_m, use near_target_heading_tolerance_deg (wider than
        heading_tolerance_deg) for align / re-rotate so small noisy errors do not hunt,
        but large errors (e.g. target behind) still trigger in-place rotation.
      * Near-target in-place rotate PWM is scaled by near_target_rotate_pwm_scale (lower
        effective P-gain) to reduce overshoot on large apparent heading errors.
      * Pause: publish 'stopped', wait 1 s, then mark the waypoint 'reached'.
      * Dwell: publish 'stopped', wait 5 s, then accept the next /dynamic_waypoint.
      * Stop once the target in robot frame lies on forward axis in [0, -0.2] m
        and |lateral offset| <= 0.1 m.
  - Home waypoint type (/dynamic_waypoints_type = 'home'):
      * Expects a target with one axis ~0 and the other ~±0.9 m; invalid format
        falls back to the normal waypoint behavior above.
      * Phase 1 (approach): drive forward to the intermediate point on the
        non-zero axis at ±0.8 m (zero axis unchanged).
      * Phase 2 (rotate): turn in place to face the field center (0, 0).
      * Phase 2b (confirm): hold 2 s, then verify the robot is still at the
        intermediate point; if not, return to Phase 1 (approach).
      * Phase 3 (reverse): back up until |coordinate| on the home axis exceeds 0.9 m
        (same axis as the non-zero home target).
      * Phase 4 (servo): pulse GPIO21 to -90 deg, wait 1 s (no PWM), pulse to +90 deg,
        wait 3 s (no PWM), then publish waypoint 'reached'.
  - Waypoint status output:
      /waypoint_status (std_msgs/String)
        'reached' when the robot-frame stop band is satisfied, else 'going'.

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
    from gpiozero import AngularServo, PWMOutputDevice, RotaryEncoder
except Exception:
    AngularServo = None
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

SERVO_PIN: int = 21
SERVO_MIN_ANGLE_DEG: float = -90.0
SERVO_MAX_ANGLE_DEG: float = 90.0
SERVO_MIN_PULSE_WIDTH_S: float = 0.0005
SERVO_MAX_PULSE_WIDTH_S: float = 0.0025
SERVO_MOVE_SETTLE_S: float = 2.0  # keep PWM active while the servo moves (matches bench script)
SERVO_STARTUP_ANGLE_DEG: float = 90.0
SERVO_STARTUP_HOLD_S: float = 3.0
HOME_SERVO_FIRST_ANGLE_DEG: float = -90.0
HOME_SERVO_SECOND_ANGLE_DEG: float = 90.0
HOME_SERVO_FIRST_HOLD_S: float = 1.0
HOME_SERVO_SECOND_HOLD_S: float = 3.0

FULL_SPEED: float = 1.0  # PWM duty cycle for full-speed forward/reverse
ROTATE_RAMP_S: float = 0.5  # soft-start ramp duration after rotate() begins
ROTATE_RAMP_START_SPEED: float = 0.2  # PWM duty cycle at rotate() start
ROTATE_SLOW_SPEED: float = 0.05  # PWM duty cycle when close to target
ROTATE_SLOWDOWN_COUNTS: int = 30  # switch to slow speed when |encoder error| < this
ROTATE_HEADING_SLOWDOWN_DEG: float = 45.0  # linear PWM ramp below this |heading error| (deg)
ROTATE_PWM_HIGH: float = FULL_SPEED  # heading-rotate PWM at/above slowdown threshold
ROTATE_PWM_MIN: float = ROTATE_SLOW_SPEED  # heading-rotate PWM when |heading error| -> 0
ROTATE_ALIGN_HYSTERESIS_DEG: float = 2.0  # exit aligned only above tolerance + this
ROTATE_DIRECTION_FLIP_DEG: float = 3.0  # suppress direction reversals below this |error|
ROTATE_PWM_SLEW_PER_S: float = 3.0  # max |dPWM/dt| (0..1 scale); 0 disables slew limit
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
COLLISION_AVOIDING_WAYPOINT_TOPIC: str = '/collision_avoiding_waypoint'
WAYPOINT_STATUS_TOPIC: str = '/waypoint_status'  # 'going' / 'reached'
CONTROL_HZ: float = 50.0  # waypoint-following control loop rate
DRIVE_SPEED: float = 0.05  # PWM duty cycle when driving forward toward the waypoint
DRIVE_HEADING_KP: float = 0.8  # differential PWM per rad of heading error while driving
DRIVE_HEADING_MAX_DIFF: float = 0.35  # cap |left_pwm - right_pwm| during drive correction
DRIVE_ROTATE_HEADING_DEG: float = 15.0  # stop and in-place rotate if |error| exceeds this while driving
HEADING_TOLERANCE_DEG: float = 1.0  # rotate in place until |heading error| <= this
NEAR_TARGET_DISTANCE_M: float = 0.20  # use near_target_heading_tolerance below this range (0 disables)
NEAR_TARGET_HEADING_TOLERANCE_DEG: float = 10.0  # wider align band when near the target
NEAR_TARGET_ROTATE_PWM_SCALE: float = 0.5  # scale in-place rotate PWM when near (1 = same as far)
POSITION_TOLERANCE_M: float = 0.1  # 'reached' when |y_robot| <= this (lateral, meters)
LONGITUDINAL_STOP_MIN_M: float = -0.2  # 'reached' when x_robot >= this (meters)
LONGITUDINAL_STOP_MAX_M: float = 0.0  # 'reached' when x_robot <= this (meters)
PHASE_PAUSE_S: float = 1.0  # stopped dwell after heading aligns (and after drive)
COLLISION_REACHED_PAUSE_S: float = 2.0  # stopped dwell at escape point before 'reached'
WAYPOINT_REACHED_PAUSE_S: float = 5.0  # stopped dwell at waypoint before next target
POSE_STALE_STOP_S: float = 0.5  # stop motors if /current_position is older than this
WAYPOINT_TYPE_TOPIC: str = '/dynamic_waypoints_type'
HOME_INTERMEDIATE_M: float = 0.8  # intermediate stop on non-zero home axis before reverse
HOME_POSITION_CONFIRM_S: float = 2.0  # hold after rotate before reverse; re-approach if off mark
HOME_AXIS_STOP_ABS_M: float = 0.95  # reverse phase stops once |axis coord| exceeds this


def _normalize_angle(angle: float) -> float:
    """Wrap an angle (rad) to the range (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _world_to_robot(dx: float, dy: float, theta: float) -> tuple[float, float]:
    """Express world-frame delta (target - robot) in robot frame (x forward, y left)."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    x_forward = dx * cos_t + dy * sin_t
    y_left = -dx * sin_t + dy * cos_t
    return x_forward, y_left


def _ramp_speed(started_at: float | None, cruise: float) -> float:
    """Linear soft-start ramp from ROTATE_RAMP_START_SPEED to cruise over ROTATE_RAMP_S."""
    if started_at is None:
        return cruise
    elapsed = time.monotonic() - started_at
    if elapsed >= ROTATE_RAMP_S:
        return cruise
    t = elapsed / ROTATE_RAMP_S
    return ROTATE_RAMP_START_SPEED + t * (cruise - ROTATE_RAMP_START_SPEED)


def _proximity_pwm(
    error_abs: float,
    threshold: float,
    pwm_high: float,
    pwm_min: float,
) -> float:
    """PWM vs error: pwm_high at/above threshold, linear down to pwm_min at zero."""
    if threshold <= 0.0:
        return max(0.0, min(1.0, pwm_high))
    if error_abs >= threshold:
        pwm = pwm_high
    else:
        t = error_abs / threshold
        pwm = pwm_min + t * (pwm_high - pwm_min)
    return max(0.0, min(1.0, pwm))


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


def _parse_home_geometry(
    home_x: float,
    home_y: float,
    eps: float,
) -> tuple[float, float, float, float] | None:
    """Return (final_x, final_y, intermediate_x, intermediate_y) or None if invalid."""
    x_near_zero = abs(home_x) <= eps
    y_near_zero = abs(home_y) <= eps
    if x_near_zero and y_near_zero:
        return None
    if not x_near_zero and not y_near_zero:
        return None
    if x_near_zero:
        intermediate_x = home_x
        intermediate_y = math.copysign(HOME_INTERMEDIATE_M, home_y)
    else:
        intermediate_x = math.copysign(HOME_INTERMEDIATE_M, home_x)
        intermediate_y = home_y
    return home_x, home_y, intermediate_x, intermediate_y


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
        proximity = (
            ROTATE_SLOW_SPEED
            if abs(error) < ROTATE_SLOWDOWN_COUNTS
            else self._rotate_speed
        )
        ramp = _ramp_speed(self._rotate_started_at, self._rotate_speed)
        return min(proximity, ramp)

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
        self.declare_parameter(
            'collision_avoiding_waypoint_topic', COLLISION_AVOIDING_WAYPOINT_TOPIC
        )
        self.declare_parameter('waypoint_status_topic', WAYPOINT_STATUS_TOPIC)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('drive_speed', DRIVE_SPEED)
        self.declare_parameter('drive_heading_kp', DRIVE_HEADING_KP)
        self.declare_parameter('drive_heading_max_diff', DRIVE_HEADING_MAX_DIFF)
        self.declare_parameter('drive_rotate_heading_deg', DRIVE_ROTATE_HEADING_DEG)
        self.declare_parameter('rotate_heading_slowdown_deg', ROTATE_HEADING_SLOWDOWN_DEG)
        self.declare_parameter('rotate_pwm_high', ROTATE_PWM_HIGH)
        self.declare_parameter('rotate_pwm_min', ROTATE_PWM_MIN)
        self.declare_parameter('rotate_align_hysteresis_deg', ROTATE_ALIGN_HYSTERESIS_DEG)
        self.declare_parameter('rotate_direction_flip_deg', ROTATE_DIRECTION_FLIP_DEG)
        self.declare_parameter('rotate_pwm_slew_per_s', ROTATE_PWM_SLEW_PER_S)
        self.declare_parameter('heading_tolerance_deg', HEADING_TOLERANCE_DEG)
        self.declare_parameter('near_target_distance_m', NEAR_TARGET_DISTANCE_M)
        self.declare_parameter('near_target_heading_tolerance_deg', NEAR_TARGET_HEADING_TOLERANCE_DEG)
        self.declare_parameter('near_target_rotate_pwm_scale', NEAR_TARGET_ROTATE_PWM_SCALE)
        self.declare_parameter('position_tolerance_m', POSITION_TOLERANCE_M)
        self.declare_parameter('longitudinal_stop_min_m', LONGITUDINAL_STOP_MIN_M)
        self.declare_parameter('longitudinal_stop_max_m', LONGITUDINAL_STOP_MAX_M)
        self.declare_parameter('phase_pause_s', PHASE_PAUSE_S)
        self.declare_parameter('waypoint_reached_pause_s', WAYPOINT_REACHED_PAUSE_S)
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
        collision_waypoint_topic = (
            self.get_parameter('collision_avoiding_waypoint_topic').get_parameter_value().string_value
            or COLLISION_AVOIDING_WAYPOINT_TOPIC
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
        self._drive_heading_kp = (
            float(self.get_parameter('drive_heading_kp').get_parameter_value().double_value)
            or DRIVE_HEADING_KP
        )
        self._drive_heading_max_diff = (
            float(self.get_parameter('drive_heading_max_diff').get_parameter_value().double_value)
            or DRIVE_HEADING_MAX_DIFF
        )
        self._drive_rotate_heading_deg = (
            float(self.get_parameter('drive_rotate_heading_deg').get_parameter_value().double_value)
            or DRIVE_ROTATE_HEADING_DEG
        )
        self._rotate_heading_slowdown_deg = (
            float(self.get_parameter('rotate_heading_slowdown_deg').get_parameter_value().double_value)
            or ROTATE_HEADING_SLOWDOWN_DEG
        )
        self._rotate_pwm_high = (
            float(self.get_parameter('rotate_pwm_high').get_parameter_value().double_value)
            or ROTATE_PWM_HIGH
        )
        self._rotate_pwm_min = (
            float(self.get_parameter('rotate_pwm_min').get_parameter_value().double_value)
            or ROTATE_PWM_MIN
        )
        self._rotate_align_hysteresis_deg = (
            float(
                self.get_parameter('rotate_align_hysteresis_deg').get_parameter_value().double_value
            )
            or ROTATE_ALIGN_HYSTERESIS_DEG
        )
        self._rotate_direction_flip_deg = (
            float(self.get_parameter('rotate_direction_flip_deg').get_parameter_value().double_value)
            or ROTATE_DIRECTION_FLIP_DEG
        )
        self._rotate_pwm_slew_per_s = (
            float(self.get_parameter('rotate_pwm_slew_per_s').get_parameter_value().double_value)
            or ROTATE_PWM_SLEW_PER_S
        )
        self._heading_tolerance_deg = (
            float(self.get_parameter('heading_tolerance_deg').get_parameter_value().double_value)
            or HEADING_TOLERANCE_DEG
        )
        self._near_target_distance_m = float(
            self.get_parameter('near_target_distance_m').get_parameter_value().double_value
        )
        self._near_target_heading_tolerance_deg = (
            float(
                self.get_parameter('near_target_heading_tolerance_deg').get_parameter_value().double_value
            )
            or NEAR_TARGET_HEADING_TOLERANCE_DEG
        )
        self._near_target_rotate_pwm_scale = float(
            self.get_parameter('near_target_rotate_pwm_scale').get_parameter_value().double_value
        )
        self._position_tolerance_m = (
            float(self.get_parameter('position_tolerance_m').get_parameter_value().double_value)
            or POSITION_TOLERANCE_M
        )
        self._longitudinal_stop_min_m = float(
            self.get_parameter('longitudinal_stop_min_m').get_parameter_value().double_value
        )
        self._longitudinal_stop_max_m = float(
            self.get_parameter('longitudinal_stop_max_m').get_parameter_value().double_value
        )
        self._phase_pause_s = (
            float(self.get_parameter('phase_pause_s').get_parameter_value().double_value)
            or PHASE_PAUSE_S
        )
        self._waypoint_reached_pause_s = (
            float(self.get_parameter('waypoint_reached_pause_s').get_parameter_value().double_value)
            or WAYPOINT_REACHED_PAUSE_S
        )

        self._run_enabled = False
        self._motion_started = False
        self._last_motion_status: str | None = None
        self._start_left_steps = 0
        self._start_right_steps = 0
        self._prev_left_count: int | None = None
        self._prev_right_count: int | None = None
        self._prev_wheel_state_time: float | None = None
        self._last_pose_update_mono: float | None = None

        # Latest robot pose (/current_position) and navigation targets.
        self._current_x: float | None = None
        self._current_y: float | None = None
        self._current_theta: float | None = None
        self._dynamic_target_x: float | None = None
        self._dynamic_target_y: float | None = None
        self._collision_target: tuple[float, float] | None = None
        self._collision_pursuit_active: bool = False
        self._collision_reached_point: tuple[float, float] | None = None
        self._target_x: float | None = None
        self._target_y: float | None = None
        self._last_waypoint_status: str | None = None
        # Waypoint motion phases: navigate -> pause -> reached -> ready.
        # During navigate, forward is allowed only when heading is within tolerance.
        self._motion_phase: str | None = None
        self._needs_heading_settle: bool = False
        self._heading_settle_until: float | None = None
        self._pause_until: float | None = None
        self._pause_next_phase: str | None = None
        self._reached_pause_until: float | None = None
        self._pending_target: tuple[float, float] | None = None
        self._active_target: tuple[float, float] | None = None
        self._nav_motion_mode: str | None = None  # 'rotate' or 'drive' during navigate
        self._rotate_started_at: float | None = None
        self._drive_started_at: float | None = None
        self._rotate_last_cmd: float = 0.0
        self._heading_align_latched: bool = False
        self._waypoint_type: str = ''
        self._home_phase: str | None = None
        self._home_final: tuple[float, float] | None = None
        self._home_intermediate: tuple[float, float] | None = None
        self._home_servo_until: float | None = None
        self._home_confirm_until: float | None = None
        self._control_period = 0.1 if control_hz <= 0.0 else (1.0 / control_hz)

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

        self._servo = None
        self._init_startup_servo()

        self.create_subscription(String, time_topic, self._on_time, 10)
        self.create_subscription(String, '/run', self._on_run, 10)
        self.create_subscription(PoseStamped, position_topic, self._on_current_position, 10)
        self.create_subscription(String, waypoint_topic, self._on_waypoint, 10)
        self.create_subscription(String, collision_waypoint_topic, self._on_collision_waypoint, 10)
        self.create_subscription(String, WAYPOINT_TYPE_TOPIC, self._on_waypoint_type, 10)

        self._wheel_state_pub = self.create_publisher(JointState, wheel_state_topic, 10)

        self._motion_status_pub = self.create_publisher(String, motion_status_topic, 10)
        status_period = 0.05 if motion_status_hz <= 0.0 else (1.0 / motion_status_hz)
        self.create_timer(status_period, self._publish_motion_status)

        self._waypoint_status_pub = self.create_publisher(String, waypoint_status_topic, 10)
        control_period = self._control_period
        self.create_timer(control_period, self._control_loop)

        if self._left_enc is not None and self._right_enc is not None:
            period = 0.2 if encoder_debug_hz <= 0.0 else (1.0 / encoder_debug_hz)
            self.create_timer(period, self._print_encoders)
            wheel_period = 0.02 if wheel_state_hz <= 0.0 else (1.0 / wheel_state_hz)
            self.create_timer(wheel_period, self._publish_wheel_joint_states)

        self.get_logger().info(
            f'motion_control_node started (MDD3A dual-PWM): '
            f'left=({LEFT_MOTOR_A},{LEFT_MOTOR_B}) right=({RIGHT_MOTOR_A},{RIGHT_MOTOR_B}), '
            f'following {waypoint_topic} (task) + {collision_waypoint_topic} (escape priority) '
            f'from {position_topic} @ {control_hz:.1f} Hz '
            f'(heading_tol={self._heading_tolerance_deg:.1f}deg, '
            f'near_dist={self._near_target_distance_m:.2f}m@'
            f'{self._near_target_heading_tolerance_deg:.1f}deg, '
            f'near_rotate_scale={self._near_target_rotate_pwm_scale:.2f}, '
            f'drive_rotate={self._drive_rotate_heading_deg:.1f}deg, '
            f'lateral_tol={self._position_tolerance_m:.2f}m, '
            f'long_stop=[{self._longitudinal_stop_min_m:.2f}, {self._longitudinal_stop_max_m:.2f}]m, '
            f'drive={self._drive_speed:.2f}, drive_kp={self._drive_heading_kp:.2f}, '
            f'drive_max_diff={self._drive_heading_max_diff:.2f}, rotate_pwm=[{self._rotate_pwm_min:.2f}, '
            f'{self._rotate_pwm_high:.2f}]@{self._rotate_heading_slowdown_deg:.1f}deg), '
            f'time_topic={time_topic}, counts_per_rev={self._counts_per_rev}, '
            f'wheel_state={wheel_state_topic} @ {wheel_state_hz:.1f} Hz, '
            f'motion_status={motion_status_topic} @ {motion_status_hz:.1f} Hz, '
            f'waypoint_status={waypoint_status_topic}'
        )

    def _init_startup_servo(self) -> None:
        if AngularServo is None:
            self.get_logger().warn('gpiozero AngularServo unavailable; startup servo sequence skipped.')
            return
        try:
            self._set_servo_angle(SERVO_STARTUP_ANGLE_DEG)
            self.get_logger().info(
                f'AngularServo on GPIO{SERVO_PIN} moved to {SERVO_STARTUP_ANGLE_DEG:.0f} deg, '
                f'waiting {SERVO_STARTUP_HOLD_S:.1f} s with GPIO{SERVO_PIN} idle'
            )
            time.sleep(SERVO_STARTUP_HOLD_S)
            self.get_logger().info('Servo startup sequence complete')
        except Exception as exc:
            self.get_logger().warn(f'Servo init failed: {exc}')

    def _on_waypoint_type(self, msg: String) -> None:
        new_type = (msg.data or '').strip().lower()
        if new_type != self._waypoint_type and self._waypoint_type == 'home':
            self._reset_home_state()
        self._waypoint_type = new_type

    def _on_current_position(self, msg: PoseStamped) -> None:
        new_x = float(msg.pose.position.x)
        new_y = float(msg.pose.position.y)
        self._current_x = new_x
        self._current_y = new_y
        self._last_pose_update_mono = time.monotonic()

        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self._current_theta = math.atan2(siny_cosp, cosy_cosp)

    def _following_collision(self) -> bool:
        return self._collision_pursuit_active

    def _reset_nav_state(self) -> None:
        self._needs_heading_settle = False
        self._heading_settle_until = None
        self._heading_align_latched = False
        self._pause_until = None
        self._pause_next_phase = None
        self._reached_pause_until = None
        self._nav_motion_mode = None
        self._rotate_started_at = None
        self._drive_started_at = None
        self._reset_rotate_command_state()

    def _begin_collision_pursuit(self) -> None:
        """Preempt current navigation and pursue the escape point to reached."""
        if self._collision_target is None:
            return
        self._stop()
        self._reset_home_state()
        self._collision_pursuit_active = True
        self._collision_reached_point = None
        self._target_x, self._target_y = self._collision_target
        self._active_target = self._collision_target
        self._motion_phase = 'navigate'
        self._pending_target = None
        self._reset_nav_state()

    def _update_collision_target_during_pursuit(self) -> None:
        """Refresh the escape target while already pursuing collision avoidance."""
        if self._collision_target is None or not self._collision_pursuit_active:
            return
        prev = (self._target_x, self._target_y)
        self._target_x, self._target_y = self._collision_target
        self._active_target = self._collision_target
        if self._collision_target == prev:
            return
        if self._motion_phase in ('reached', 'ready', 'pause'):
            self._motion_phase = 'navigate'
        self._reset_nav_state()

    def _begin_collision_arrival_pause(self) -> None:
        """Stop at the escape point for COLLISION_REACHED_PAUSE_S before handoff."""
        self._stop()
        self._publish_motion_status_now('stopped')
        self._motion_phase = 'pause'
        self._pause_until = time.monotonic() + COLLISION_REACHED_PAUSE_S
        self._pause_next_phase = 'collision_handoff'
        self.get_logger().info(
            f'[collision] escape point reached, holding stopped for '
            f'{COLLISION_REACHED_PAUSE_S:.1f} s before reached'
        )

    def _handoff_to_dynamic_after_collision(self) -> None:
        """Collision escape point reached; resume the task waypoint."""
        if self._collision_target is not None:
            self._collision_reached_point = self._collision_target
        self._collision_pursuit_active = False
        self._pending_target = None
        self._publish_waypoint_status('reached')
        if self._dynamic_target_x is not None and self._dynamic_target_y is not None:
            self._target_x = self._dynamic_target_x
            self._target_y = self._dynamic_target_y
            self._active_target = (self._target_x, self._target_y)
            self._motion_phase = 'navigate'
            self._reset_nav_state()
            return
        self._begin_phase_pause('reached')

    def _sync_follow_target(self, *, immediate: bool = False) -> None:
        if self._collision_pursuit_active:
            return
        if self._dynamic_target_x is not None and self._dynamic_target_y is not None:
            new_target = (self._dynamic_target_x, self._dynamic_target_y)
        else:
            self._target_x = None
            self._target_y = None
            return

        if (
            not immediate
            and self._motion_phase == 'reached'
            and self._reached_pause_until is not None
            and time.monotonic() < self._reached_pause_until
        ):
            self._pending_target = new_target
            return

        prev = (self._target_x, self._target_y)
        self._target_x, self._target_y = new_target
        if new_target == prev:
            return

        self._active_target = new_target
        self._motion_phase = 'navigate'
        self._pending_target = None
        self._reset_nav_state()

    def _on_waypoint(self, msg: String) -> None:
        parsed = _parse_waypoint(msg.data if msg.data else '')
        if parsed is None:
            self._dynamic_target_x = None
            self._dynamic_target_y = None
        else:
            self._dynamic_target_x, self._dynamic_target_y = parsed
        if self._collision_pursuit_active:
            return
        self._sync_follow_target()

    def _on_collision_waypoint(self, msg: String) -> None:
        prev_collision = self._collision_target
        parsed = _parse_waypoint(msg.data if msg.data else '')
        self._collision_target = parsed

        if self._collision_target is None:
            self._collision_pursuit_active = False
            self._collision_reached_point = None
            if prev_collision is not None:
                self._sync_follow_target(immediate=True)
            return

        if self._collision_pursuit_active:
            if self._collision_target != prev_collision:
                self._update_collision_target_during_pursuit()
            return

        if self._collision_reached_point is not None:
            rx, ry = self._collision_reached_point
            cx, cy = self._collision_target
            if math.hypot(cx - rx, cy - ry) < 1e-3:
                return

        self._begin_collision_pursuit()

    def _reset_motion_phase(self) -> None:
        self._motion_phase = None
        self._collision_pursuit_active = False
        self._collision_reached_point = None
        self._needs_heading_settle = False
        self._heading_settle_until = None
        self._pause_until = None
        self._pause_next_phase = None
        self._reached_pause_until = None
        self._pending_target = None
        self._active_target = None
        self._nav_motion_mode = None
        self._rotate_started_at = None
        self._drive_started_at = None
        self._heading_align_latched = False
        self._reset_rotate_command_state()
        self._reset_home_state()

    def _reset_home_state(self) -> None:
        self._home_phase = None
        self._home_final = None
        self._home_intermediate = None
        self._home_servo_until = None
        self._home_confirm_until = None

    def _home_sequence_active(self) -> bool:
        return self._home_phase in ('approach', 'rotate', 'confirm', 'reverse')

    def _home_sequence_holding(self) -> bool:
        """True while home is in progress or finished and should not re-navigate."""
        return self._home_phase in (
            'approach', 'rotate', 'confirm', 'reverse', 'servo_90', 'servo_neg90', 'complete',
        )

    def _release_servo(self) -> None:
        if self._servo is None:
            return
        try:
            self._servo.close()
        except Exception:
            pass
        self._servo = None

    def _set_servo_angle(self, angle_deg: float) -> None:
        if AngularServo is None:
            self.get_logger().warn(f'Servo unavailable; cannot set GPIO{SERVO_PIN} to {angle_deg:.0f} deg')
            return
        self._release_servo()
        clamped = max(SERVO_MIN_ANGLE_DEG, min(SERVO_MAX_ANGLE_DEG, float(angle_deg)))
        servo = None
        try:
            servo = AngularServo(
                SERVO_PIN,
                min_angle=SERVO_MIN_ANGLE_DEG,
                max_angle=SERVO_MAX_ANGLE_DEG,
                min_pulse_width=SERVO_MIN_PULSE_WIDTH_S,
                max_pulse_width=SERVO_MAX_PULSE_WIDTH_S,
            )
            servo.angle = clamped
            if SERVO_MOVE_SETTLE_S > 0.0:
                time.sleep(SERVO_MOVE_SETTLE_S)
            self.get_logger().info(
                f'[servo] GPIO{SERVO_PIN} angle -> {clamped:.0f} deg '
                f'({SERVO_MOVE_SETTLE_S:.1f} s PWM), then released'
            )
        except Exception as exc:
            self.get_logger().warn(f'Servo set angle failed: {exc}')
        finally:
            if servo is not None:
                try:
                    servo.close()
                except Exception:
                    pass
            self._servo = None

    def _begin_home_servo_sequence(self) -> None:
        self._stop()
        self._publish_motion_status_now('stopped')
        self._home_phase = 'servo_neg90'
        self._motion_phase = 'navigate'
        self._set_servo_angle(HOME_SERVO_FIRST_ANGLE_DEG)
        self._home_servo_until = time.monotonic() + HOME_SERVO_FIRST_HOLD_S
        self.get_logger().info(
            f'[home] reverse complete, servo -> {HOME_SERVO_FIRST_ANGLE_DEG:.0f} deg, '
            f'waiting {HOME_SERVO_FIRST_HOLD_S:.1f} s with GPIO{SERVO_PIN} idle'
        )

    def _control_home_servo_sequence(self) -> None:
        now = time.monotonic()
        self._publish_waypoint_status('going')
        self._stop()
        self._publish_motion_status_now('stopped')

        if self._home_servo_until is None or now < self._home_servo_until:
            return

        if self._home_phase == 'servo_neg90':
            self._home_phase = 'servo_90'
            self._set_servo_angle(HOME_SERVO_SECOND_ANGLE_DEG)
            self._home_servo_until = now + HOME_SERVO_SECOND_HOLD_S
            self.get_logger().info(
                f'[home] servo -> {HOME_SERVO_SECOND_ANGLE_DEG:.0f} deg, '
                f'waiting {HOME_SERVO_SECOND_HOLD_S:.1f} s with GPIO{SERVO_PIN} idle'
            )
            return

        if self._home_phase == 'servo_90':
            self._home_phase = 'complete'
            self._home_servo_until = None
            if self._home_final is not None:
                self._active_target = self._home_final
            self.get_logger().info('[home] servo sequence complete, marking reached')
            self._begin_phase_pause('reached')

    def _init_home_sequence(self, final_x: float, final_y: float) -> None:
        parsed = _parse_home_geometry(final_x, final_y, self._position_tolerance_m)
        if parsed is None:
            self._reset_home_state()
            self.get_logger().warn(
                f'[home] invalid geometry ({final_x:.3f}, {final_y:.3f}); '
                'fallback to normal navigation'
            )
            return

        fx, fy, ix, iy = parsed
        self._home_final = (fx, fy)
        self._home_intermediate = (ix, iy)
        self._home_phase = 'approach'
        self._motion_phase = 'navigate'
        self._active_target = None
        self._needs_heading_settle = False
        self._heading_settle_until = None
        self._heading_align_latched = False
        self._nav_motion_mode = None
        self.get_logger().info(
            f'[home] sequence start: final=({fx:.3f}, {fy:.3f}) '
            f'intermediate=({ix:.3f}, {iy:.3f})'
        )

    def _begin_home_position_confirm(self) -> None:
        self._stop()
        self._publish_motion_status_now('stopped')
        self._home_phase = 'confirm'
        self._home_confirm_until = time.monotonic() + HOME_POSITION_CONFIRM_S
        self._motion_phase = 'navigate'
        self._needs_heading_settle = False
        self._heading_settle_until = None
        self._heading_align_latched = False
        self._nav_motion_mode = None
        self.get_logger().info(
            f'[home] rotate complete, holding {HOME_POSITION_CONFIRM_S:.1f} s to confirm position'
        )

    def _transition_home_phase(self, next_phase: str) -> None:
        self._home_phase = next_phase
        self._home_confirm_until = None
        self._needs_heading_settle = False
        self._heading_settle_until = None
        self._heading_align_latched = False
        self._nav_motion_mode = None
        self.get_logger().info(f'[home] phase -> {next_phase}')
        self._begin_phase_pause('navigate')

    def _home_reverse_reached(self) -> bool:
        """True once the robot has backed past HOME_AXIS_STOP_ABS_M on the home axis."""
        if self._home_final is None or self._current_x is None or self._current_y is None:
            return False

        fx, fy = self._home_final
        eps = self._position_tolerance_m
        if abs(fx) <= eps:
            axis_coord = self._current_y
            target_coord = fy
        else:
            axis_coord = self._current_x
            target_coord = fx

        if abs(axis_coord) <= HOME_AXIS_STOP_ABS_M:
            return False
        return math.copysign(1.0, axis_coord) == math.copysign(1.0, target_coord)

    def _reset_rotate_command_state(self) -> None:
        self._rotate_last_cmd = 0.0

    def _heading_error_rad(self, dx: float, dy: float) -> float:
        desired_heading = math.atan2(dy, dx)
        return _normalize_angle(desired_heading - self._current_theta)

    def _at_target(self, dx: float, dy: float) -> bool:
        """True when target lies in robot-frame stop band (x forward in [min, max], |y| <= lateral tol)."""
        x_forward, y_left = _world_to_robot(dx, dy, self._current_theta)
        return (
            self._longitudinal_stop_min_m <= x_forward <= self._longitudinal_stop_max_m
            and abs(y_left) <= self._position_tolerance_m
        )

    def _is_near_target(self, dx: float, dy: float) -> bool:
        """True when the target is close enough to use the relaxed heading tolerance."""
        if self._near_target_distance_m <= 0.0:
            return False
        return math.hypot(dx, dy) < self._near_target_distance_m

    def _heading_tolerance_deg_for(self, dx: float, dy: float) -> float:
        if self._is_near_target(dx, dy):
            return self._near_target_heading_tolerance_deg
        return self._heading_tolerance_deg

    def _needs_drive_rotate(self, heading_error: float, dx: float, dy: float) -> bool:
        """True when |heading error| requires in-place rotation while driving."""
        if self._is_near_target(dx, dy):
            limit = self._near_target_heading_tolerance_deg
        else:
            limit = self._drive_rotate_heading_deg
        if limit <= 0.0:
            return False
        return abs(math.degrees(heading_error)) > limit

    def _heading_rotate_aligned(self, dx: float, dy: float) -> bool:
        """Latched heading align with hysteresis to avoid hunting at the tolerance edge."""
        error_abs_deg = abs(math.degrees(self._heading_error_rad(dx, dy)))
        enter_tol = self._heading_tolerance_deg_for(dx, dy)
        exit_tol = enter_tol + self._rotate_align_hysteresis_deg
        if not self._heading_align_latched:
            if error_abs_deg <= enter_tol:
                self._heading_align_latched = True
        elif error_abs_deg > exit_tol:
            self._heading_align_latched = False
        return self._heading_align_latched

    def _apply_rotate_direction_hysteresis(self, cmd: float, error_deg: float) -> float:
        if cmd == 0.0 or self._rotate_direction_flip_deg <= 0.0:
            return cmd
        if abs(error_deg) >= self._rotate_direction_flip_deg:
            return cmd
        if self._rotate_last_cmd == 0.0:
            return cmd
        if (cmd > 0.0) == (self._rotate_last_cmd > 0.0):
            return cmd
        return math.copysign(abs(cmd), self._rotate_last_cmd)

    def _apply_rotate_pwm_slew(self, cmd: float) -> float:
        if self._rotate_pwm_slew_per_s <= 0.0:
            self._rotate_last_cmd = cmd
            return cmd
        max_delta = self._rotate_pwm_slew_per_s * self._control_period
        low = self._rotate_last_cmd - max_delta
        high = self._rotate_last_cmd + max_delta
        cmd = max(low, min(high, cmd))
        self._rotate_last_cmd = cmd
        return cmd

    def _heading_rotate_command(self, heading_error: float, dx: float, dy: float) -> float:
        """Signed PWM for in-place heading rotation (positive = CCW / turn left).

        Proximity PWM is proportional (saturated) in |error|; near the target the output
        is scaled by near_target_rotate_pwm_scale to reduce overshoot.
        """
        error_deg = math.degrees(heading_error)
        error_abs_deg = abs(error_deg)

        proximity = _proximity_pwm(
            error_abs_deg,
            self._rotate_heading_slowdown_deg,
            self._rotate_pwm_high,
            self._rotate_pwm_min,
        )
        ramp = _ramp_speed(self._rotate_started_at, self._rotate_pwm_high)
        pwm_cap = min(proximity, ramp)
        if self._is_near_target(dx, dy) and self._near_target_rotate_pwm_scale < 1.0:
            pwm_cap *= max(0.0, self._near_target_rotate_pwm_scale)
        if pwm_cap <= 0.0:
            self._rotate_last_cmd = 0.0
            return 0.0

        u = math.copysign(pwm_cap, heading_error)
        u = self._apply_rotate_direction_hysteresis(u, error_deg)
        return self._apply_rotate_pwm_slew(u)

    def _drive_pwm(self) -> float:
        return _ramp_speed(self._drive_started_at, self._drive_speed)

    def _drive_wheel_speeds(self, heading_error: float) -> tuple[float, float]:
        """Forward PWM for left/right wheels with proportional heading correction."""
        base = self._drive_pwm()
        if self._drive_heading_kp <= 0.0 or self._drive_heading_max_diff <= 0.0:
            return base, base

        # Positive heading_error -> target left -> turn left -> slow left, speed up right.
        diff = self._drive_heading_kp * heading_error
        diff = max(-self._drive_heading_max_diff, min(self._drive_heading_max_diff, diff))
        left = max(0.0, min(1.0, base - diff))
        right = max(0.0, min(1.0, base + diff))
        return left, right

    def _reverse_wheel_speeds(self, heading_error: float) -> tuple[float, float]:
        """Reverse PWM for left/right wheels with proportional heading correction."""
        return self._drive_wheel_speeds(heading_error)

    def _begin_nav_motion(self, mode: str) -> None:
        if self._nav_motion_mode == mode:
            return
        self._nav_motion_mode = mode
        now = time.monotonic()
        if mode == 'rotate':
            self._rotate_started_at = now
            self._reset_rotate_command_state()
        elif mode == 'drive':
            self._drive_started_at = now

    def _rotate_toward_heading(self, heading_error: float, dx: float, dy: float) -> None:
        cmd = self._heading_rotate_command(heading_error, dx, dy)
        if cmd == 0.0:
            self._left.stop()
            self._right.stop()
            return
        speed = abs(cmd)
        # Positive cmd = CCW (turn left): right wheel forward, left wheel reverse.
        if cmd > 0.0:
            self._left.reverse(speed)
            self._right.forward(speed)
        else:
            self._left.forward(speed)
            self._right.reverse(speed)

    def _begin_phase_pause(self, next_phase: str) -> None:
        """Stop motors, publish 'stopped', and wait before entering the next phase."""
        self._stop()
        self._publish_motion_status_now('stopped')
        self._motion_phase = 'pause'
        self._pause_until = time.monotonic() + self._phase_pause_s
        self._pause_next_phase = next_phase

    def _navigate_forward_toward(self, dx: float, dy: float) -> None:
        """Rotate, settle, then drive forward toward (dx, dy) in world frame."""
        heading_error = self._heading_error_rad(dx, dy)

        already_driving = self._nav_motion_mode == 'drive'
        if already_driving:
            aligned = not self._needs_drive_rotate(heading_error, dx, dy)
        else:
            aligned = self._heading_rotate_aligned(dx, dy)
        if not aligned:
            self._needs_heading_settle = True
            self._heading_settle_until = None
            self._begin_nav_motion('rotate')
            self._rotate_toward_heading(heading_error, dx, dy)
            return

        if self._needs_heading_settle:
            now = time.monotonic()
            if self._heading_settle_until is None:
                self._stop()
                self._publish_motion_status_now('stopped')
                self._heading_settle_until = now + self._phase_pause_s
                return
            if now < self._heading_settle_until:
                self._stop()
                self._publish_motion_status_now('stopped')
                return
            self._needs_heading_settle = False
            self._heading_settle_until = None

        self._begin_nav_motion('drive')
        left_speed, right_speed = self._drive_wheel_speeds(heading_error)
        self._left.forward(left_speed)
        self._right.forward(right_speed)

    def _control_home_sequence(self) -> None:
        """Three-phase home return: approach intermediate, face origin, reverse to final."""
        if (
            self._home_phase is None
            or self._home_final is None
            or self._home_intermediate is None
            or self._current_x is None
            or self._current_y is None
            or self._current_theta is None
        ):
            return

        if self._motion_phase is None:
            self._motion_phase = 'navigate'

        if self._motion_phase != 'navigate':
            return

        self._publish_waypoint_status('going')

        if self._home_phase == 'approach':
            ix, iy = self._home_intermediate
            dx = ix - self._current_x
            dy = iy - self._current_y
            if self._at_target(dx, dy):
                self._transition_home_phase('rotate')
                return
            self._navigate_forward_toward(dx, dy)
            return

        if self._home_phase == 'rotate':
            dx = -self._current_x
            dy = -self._current_y
            heading_error = self._heading_error_rad(dx, dy)
            if self._heading_rotate_aligned(dx, dy):
                self._begin_home_position_confirm()
                return
            self._needs_heading_settle = False
            self._heading_settle_until = None
            self._begin_nav_motion('rotate')
            self._rotate_toward_heading(heading_error, dx, dy)
            return

        if self._home_phase == 'confirm':
            self._stop()
            self._publish_motion_status_now('stopped')
            now = time.monotonic()
            if self._home_confirm_until is not None and now < self._home_confirm_until:
                return

            ix, iy = self._home_intermediate
            dx = ix - self._current_x
            dy = iy - self._current_y
            if self._at_target(dx, dy):
                self.get_logger().info(
                    f'[home] position confirmed at ({self._current_x:.3f}, {self._current_y:.3f})'
                )
                self._transition_home_phase('reverse')
            else:
                self.get_logger().info(
                    f'[home] off intermediate mark at ({self._current_x:.3f}, {self._current_y:.3f}), '
                    f'retry approach to ({ix:.3f}, {iy:.3f})'
                )
                self._transition_home_phase('approach')
            return

        if self._home_phase == 'reverse':
            fx, fy = self._home_final
            if self._home_reverse_reached():
                self._active_target = (fx, fy)
                self.get_logger().info(
                    f'[home] arrived at final ({fx:.3f}, {fy:.3f}), '
                    f'pose=({self._current_x:.3f}, {self._current_y:.3f})'
                )
                self._begin_home_servo_sequence()
                return

            dx = -self._current_x
            dy = -self._current_y
            heading_error = self._heading_error_rad(dx, dy)
            already_driving = self._nav_motion_mode == 'drive'
            if already_driving:
                aligned = not self._needs_drive_rotate(heading_error, dx, dy)
            else:
                aligned = self._heading_rotate_aligned(dx, dy)
            if not aligned:
                self._needs_heading_settle = False
                self._heading_settle_until = None
                self._begin_nav_motion('rotate')
                self._rotate_toward_heading(heading_error, dx, dy)
                return

            self._begin_nav_motion('drive')
            left_speed, right_speed = self._reverse_wheel_speeds(-heading_error)
            self._left.reverse(left_speed)
            self._right.reverse(right_speed)

    def _control_loop(self) -> None:
        """Heading-gated controller: rotate until aligned, then drive; re-rotate as needed."""
        if not self._run_enabled:
            self._reset_motion_phase()
            self._stop()
            self._publish_motion_status_now('stopped')
            return

        have_pose = (
            self._current_x is not None
            and self._current_y is not None
            and self._current_theta is not None
        )
        have_waypoint = self._target_x is not None and self._target_y is not None
        if not have_pose or not have_waypoint:
            self._reset_motion_phase()
            self._stop()
            return

        if self._motion_started:
            pose_never_received = self._last_pose_update_mono is None
            pose_age_s = (
                None
                if pose_never_received
                else time.monotonic() - self._last_pose_update_mono
            )
            if pose_never_received or (
                pose_age_s is not None and pose_age_s > POSE_STALE_STOP_S
            ):
                if self._left.is_active or self._right.is_active:
                    reason = 'never received' if pose_never_received else f'{pose_age_s:.2f}s stale'
                    self.get_logger().warn(
                        f'[pose_stale] /current_position {reason}; stopping motors'
                    )
                self._stop()
                self._publish_motion_status_now('stopped')
                return

        if self._waypoint_type == 'home' and not self._following_collision():
            home_target = (self._target_x, self._target_y)
            if self._home_phase == 'complete' and home_target == self._home_final:
                pass
            elif home_target != self._home_final or not self._home_sequence_holding():
                self._init_home_sequence(self._target_x, self._target_y)

        target = (self._target_x, self._target_y)
        use_normal_target_tracking = not (
            self._waypoint_type == 'home' and self._home_sequence_holding()
        ) and not self._following_collision()
        if use_normal_target_tracking and self._active_target != target:
            if self._motion_phase == 'reached':
                self._pending_target = target
            elif self._motion_phase != 'ready':
                self._active_target = target
                self._motion_phase = 'navigate'
                self._needs_heading_settle = False
                self._heading_settle_until = None
                self._heading_align_latched = False
                self._pause_until = None
                self._pause_next_phase = None

        dx = self._target_x - self._current_x
        dy = self._target_y - self._current_y
        at_target = self._at_target(dx, dy)

        if self._motion_phase == 'pause':
            self._stop()
            self._publish_motion_status_now('stopped')
            if self._pause_until is not None and time.monotonic() >= self._pause_until:
                next_phase = self._pause_next_phase
                self._pause_until = None
                self._pause_next_phase = None
                if next_phase == 'collision_handoff':
                    self._handoff_to_dynamic_after_collision()
                    return
                self._motion_phase = next_phase
                if next_phase == 'reached':
                    self._reached_pause_until = time.monotonic() + self._waypoint_reached_pause_s
            return

        if self._motion_phase == 'reached':
            self._publish_waypoint_status('reached')
            self._stop()
            self._publish_motion_status_now('stopped')
            if self._reached_pause_until is not None and time.monotonic() >= self._reached_pause_until:
                self._reached_pause_until = None
                self._motion_phase = 'ready'
                if self._pending_target is not None:
                    self._target_x, self._target_y = self._pending_target
                    self._pending_target = None
            return

        if self._motion_phase == 'ready':
            self._publish_waypoint_status('reached')
            self._stop()
            self._publish_motion_status_now('stopped')
            if self._pending_target is not None:
                self._target_x, self._target_y = self._pending_target
                self._pending_target = None
                target = (self._target_x, self._target_y)
            if (
                self._active_target != target
                and not (
                    self._waypoint_type == 'home' and self._home_phase == 'complete'
                )
            ):
                self._active_target = target
                self._motion_phase = 'navigate'
                self._needs_heading_settle = False
                self._heading_settle_until = None
            return

        if self._waypoint_type == 'home' and self._home_phase in ('servo_90', 'servo_neg90'):
            self._control_home_servo_sequence()
            return

        if self._waypoint_type == 'home' and self._home_phase == 'complete':
            self._publish_waypoint_status('reached')
            self._stop()
            self._publish_motion_status_now('stopped')
            return

        if self._waypoint_type == 'home' and self._home_sequence_active():
            self._control_home_sequence()
            return

        if at_target:
            self._needs_heading_settle = False
            self._heading_settle_until = None
            if self._collision_pursuit_active:
                self._begin_collision_arrival_pause()
                return
            self._begin_phase_pause('reached')
            return

        self._publish_waypoint_status('going')

        if self._motion_phase is None:
            self._motion_phase = 'navigate'

        if self._motion_phase != 'navigate':
            return

        self._navigate_forward_toward(dx, dy)

    def _publish_waypoint_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._waypoint_status_pub.publish(msg)

        if status != self._last_waypoint_status:
            self._last_waypoint_status = status
            self.get_logger().info(f'[waypoint_status] {status}')

    def _publish_motion_status_now(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._motion_status_pub.publish(msg)
        if status != self._last_motion_status:
            self._last_motion_status = status
            self.get_logger().info(f'[motion_status] {status}')

    def _publish_motion_status(self) -> None:
        """Publish 'moving' if either motor is being driven, else 'stopped'."""
        moving = self._left.is_active or self._right.is_active
        status = 'moving' if moving else 'stopped'
        self._publish_motion_status_now(status)

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
        self._reset_motion_phase()

    def _on_run(self, msg: String) -> None:
        enabled = (msg.data or '').strip().lower() != 'off'
        if not enabled:
            self._run_enabled = False
            self._reset_motion_state()
            self._stop()
            self._publish_motion_status_now('stopped')
            return
        was_enabled = self._run_enabled
        self._run_enabled = True
        if not was_enabled:
            # Fresh match start: require a new fused pose before driving again.
            self._current_x = None
            self._current_y = None
            self._current_theta = None
            self._last_pose_update_mono = None
            self._reached_pause_until = None
            self._reset_motion_phase()
            self._sync_follow_target(immediate=True)
            self.get_logger().info('/run=on: match started, waiting for fused pose')

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
        self._nav_motion_mode = None
        self._rotate_started_at = None
        self._drive_started_at = None
        self._left.stop()
        self._right.stop()

    def shutdown_motors(self) -> None:
        self._stop()
        self._left.close()
        self._right.close()
        self._release_servo()
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
