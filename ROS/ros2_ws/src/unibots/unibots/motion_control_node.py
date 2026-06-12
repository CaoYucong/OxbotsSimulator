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

"""

import time

import rclpy
from rclpy.node import Node
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
ENCODER_DEBUG_HZ: float = 10.0  # how often to print encoder debug
ROTATE_SWITCH_INTERVAL_S: int = 10  # alternate rotate target every N seconds
ROTATE_PHASE_TARGET: int = 4000  # encoder count target per wheel in each phase


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
        time_topic = self.get_parameter('time_topic').get_parameter_value().string_value or '/time'
        self._counts_per_rev = float(self.get_parameter('counts_per_rev').get_parameter_value().double_value) or COUNTS_PER_REV
        encoder_debug_hz = float(self.get_parameter('encoder_debug_hz').get_parameter_value().double_value) or ENCODER_DEBUG_HZ

        self._run_enabled = False
        self._motion_started = False
        self._rotate_targets: tuple[int, int] | None = None
        self._start_left_steps = 0
        self._start_right_steps = 0

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
        self.create_timer(ROTATE_POLL_S, self._update_rotates)

        if self._left_enc is not None and self._right_enc is not None:
            period = 0.2 if encoder_debug_hz <= 0.0 else (1.0 / encoder_debug_hz)
            self.create_timer(period, self._print_encoders)

        self.get_logger().info(
            f'motion_control_node started (MDD3A dual-PWM): '
            f'left=({LEFT_MOTOR_A},{LEFT_MOTOR_B}) right=({RIGHT_MOTOR_A},{RIGHT_MOTOR_B}), '
            f'every {ROTATE_SWITCH_INTERVAL_S}s alternate left/right rotate to count={ROTATE_PHASE_TARGET}, '
            f'time_topic={time_topic}, counts_per_rev={self._counts_per_rev}'
        )

    def _update_rotates(self) -> None:
        if not self._run_enabled:
            return
        self._left.update_rotate()
        self._right.update_rotate()

    def _wheel_revolutions(self) -> tuple[float, float]:
        left_steps = self._left_enc.steps - self._start_left_steps
        right_steps = self._right_enc.steps - self._start_right_steps
        return (
            left_steps / self._counts_per_rev,
            right_steps / self._counts_per_rev,
        )

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
        self._rotate_targets = None
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
            self.get_logger().info(
                f'/time > 0: even phases left={ROTATE_PHASE_TARGET}/right=0, '
                f'odd phases left=0/right={ROTATE_PHASE_TARGET}, '
                f'each phase lasts {ROTATE_SWITCH_INTERVAL_S}s'
            )

        phase = int(t) // ROTATE_SWITCH_INTERVAL_S
        target_left = ROTATE_PHASE_TARGET if phase % 2 == 0 else 0
        target_right = -ROTATE_PHASE_TARGET if phase % 2 == 0 else 0
        targets = (target_left, target_right)
        if self._rotate_targets != targets:
            self._rotate_targets = targets
            self._left.rotate(target_left)
            self._right.rotate(target_right)

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
