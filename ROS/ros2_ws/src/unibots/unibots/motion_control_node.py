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
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from gpiozero import PWMOutputDevice, RotaryEncoder
except Exception:
    PWMOutputDevice = None
    RotaryEncoder = None


# --- Motor control pins (BCM numbering) ---
LEFT_M1A: int = 18   # left motor PWM input A
LEFT_M1B: int = 19   # left motor PWM input B
RIGHT_M2A: int = 12  # right motor PWM input A
RIGHT_M2B: int = 13  # right motor PWM input B

# --- Encoder pins (BCM numbering) ---
LEFT_ENC_A: int = 17
LEFT_ENC_B: int = 27
RIGHT_ENC_A: int = 22
RIGHT_ENC_B: int = 23

FORWARD_SPEED: float = 0.7   # PWM duty cycle (0.0 - 1.0)
TARGET_REVOLUTIONS: float = 10.0  # wheel revolutions to complete once /time > 0

COUNTS_PER_REV: float = 488.0  # quadrature counts per output-shaft revolution
ENCODER_DEBUG_HZ: float = 5.0  # how often to print encoder debug


class Motor:
    """A single MDD3A channel driven in dual-PWM mode."""

    def __init__(self, pin_a: int, pin_b: int) -> None:
        self._a = PWMOutputDevice(pin_a, initial_value=0.0)
        self._b = PWMOutputDevice(pin_b, initial_value=0.0)

    def forward(self, speed: float) -> None:
        self._a.value = max(0.0, min(1.0, speed))
        self._b.value = 0.0

    def reverse(self, speed: float) -> None:
        self._a.value = 0.0
        self._b.value = max(0.0, min(1.0, speed))

    def stop(self) -> None:
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

        self.declare_parameter('forward_speed', FORWARD_SPEED)
        self.declare_parameter('target_revolutions', TARGET_REVOLUTIONS)
        self.declare_parameter('time_topic', '/time')
        self.declare_parameter('counts_per_rev', COUNTS_PER_REV)
        self.declare_parameter('encoder_debug_hz', ENCODER_DEBUG_HZ)
        self._forward_speed = float(self.get_parameter('forward_speed').get_parameter_value().double_value) or FORWARD_SPEED
        self._target_revolutions = float(self.get_parameter('target_revolutions').get_parameter_value().double_value) or TARGET_REVOLUTIONS
        time_topic = self.get_parameter('time_topic').get_parameter_value().string_value or '/time'
        self._counts_per_rev = float(self.get_parameter('counts_per_rev').get_parameter_value().double_value) or COUNTS_PER_REV
        encoder_debug_hz = float(self.get_parameter('encoder_debug_hz').get_parameter_value().double_value) or ENCODER_DEBUG_HZ

        self._left = Motor(LEFT_M1A, LEFT_M1B)
        self._right = Motor(RIGHT_M2A, RIGHT_M2B)
        self._motion_started = False
        self._motion_done = False
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

        self.create_subscription(String, time_topic, self._on_time, 10)

        if self._left_enc is not None and self._right_enc is not None:
            period = 0.2 if encoder_debug_hz <= 0.0 else (1.0 / encoder_debug_hz)
            self.create_timer(period, self._print_encoders)

        self.get_logger().info(
            f'motion_control_node started (MDD3A dual-PWM): '
            f'left=({LEFT_M1A},{LEFT_M1B}) right=({RIGHT_M2A},{RIGHT_M2B}), '
            f'forward_speed={self._forward_speed}, target_revolutions={self._target_revolutions}, '
            f'time_topic={time_topic}, counts_per_rev={self._counts_per_rev}'
        )

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
                f'right={right_rev:+.3f} rev ({right_steps} cnt) '
                f'(target={self._target_revolutions:.1f})'
            )
        else:
            self.get_logger().info(
                f'[encoder] left={left_cnt} cnt, right={right_steps} cnt (idle)'
            )

    def _reset_motion_state(self) -> None:
        self._motion_started = False
        self._motion_done = False
        self._start_left_steps = 0
        self._start_right_steps = 0

    def _on_time(self, msg: String) -> None:
        try:
            t = float(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Ignoring non-numeric /time message: {msg.data!r}')
            return

        if t <= 0.0:
            self._reset_motion_state()
            self._stop()
            return

        if self._motion_done:
            self._stop()
            return

        if self._left_enc is None or self._right_enc is None:
            if not self._motion_started:
                self.get_logger().error('Encoders required for revolution control; motors will not run.')
                self._motion_started = True
            self._stop()
            return

        if not self._motion_started:
            self._start_left_steps = self._left_enc.steps
            self._start_right_steps = self._right_enc.steps
            self._motion_started = True
            self.get_logger().info(
                f'/time > 0: driving forward until either wheel reaches '
                f'{self._target_revolutions:.1f} rev'
            )

        left_rev, right_rev = self._wheel_revolutions()
        if left_rev >= self._target_revolutions or right_rev >= self._target_revolutions:
            self._motion_done = True
            self._stop()
            self.get_logger().info(
                f'Target reached: left={-left_rev:.3f} rev, right={right_rev:.3f} rev — stopped'
            )
            return

        self._move_forward()

    def _move_forward(self) -> None:
        self._left.forward(self._forward_speed)
        self._right.forward(self._forward_speed)

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
