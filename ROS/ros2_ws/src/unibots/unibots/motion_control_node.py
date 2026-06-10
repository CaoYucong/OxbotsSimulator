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
  - Servo signal: GPIO21

Keyboard control (stdin, run in a terminal):
  - W/S : forward / reverse
  - A/D : turn left / turn right
  - Space or Q : stop
"""

import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

try:
    from gpiozero import AngularServo, PWMOutputDevice, RotaryEncoder
except Exception:
    AngularServo = None
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

# --- Servo pin (BCM numbering) ---
SERVO_PIN: int = 21

FULL_SPEED: float = 1.0       # PWM duty cycle for full-speed forward
TURN_SPEED: float = 1.0       # PWM duty cycle for in-place turns

COUNTS_PER_REV: float = 488.0  # quadrature counts per output-shaft revolution
ENCODER_DEBUG_HZ: float = 5.0  # how often to print encoder debug
CONTROL_HZ: float = 20.0       # motor update rate while keys are held
KEY_STALE_SEC: float = 0.35    # stop if no key event within this window


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

        self.declare_parameter('full_speed', FULL_SPEED)
        self.declare_parameter('turn_speed', TURN_SPEED)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('key_stale_sec', KEY_STALE_SEC)
        self.declare_parameter('counts_per_rev', COUNTS_PER_REV)
        self.declare_parameter('encoder_debug_hz', ENCODER_DEBUG_HZ)
        self.declare_parameter('servo_pin', SERVO_PIN)
        self._full_speed = float(self.get_parameter('full_speed').get_parameter_value().double_value) or FULL_SPEED
        self._turn_speed = float(self.get_parameter('turn_speed').get_parameter_value().double_value) or TURN_SPEED
        control_hz = float(self.get_parameter('control_hz').get_parameter_value().double_value) or CONTROL_HZ
        self._key_stale_sec = float(self.get_parameter('key_stale_sec').get_parameter_value().double_value) or KEY_STALE_SEC
        self._counts_per_rev = float(self.get_parameter('counts_per_rev').get_parameter_value().double_value) or COUNTS_PER_REV
        encoder_debug_hz = float(self.get_parameter('encoder_debug_hz').get_parameter_value().double_value) or ENCODER_DEBUG_HZ
        servo_pin = int(self.get_parameter('servo_pin').get_parameter_value().integer_value) or SERVO_PIN

        self._left = Motor(LEFT_M1A, LEFT_M1B)
        self._right = Motor(RIGHT_M2A, RIGHT_M2B)
        self._driving = False
        self._direction: str | None = None
        self._servo = None
        if AngularServo is not None:
            try:
                self._servo = AngularServo(servo_pin, min_angle=0, max_angle=180)
                self._servo.detach()
            except Exception as exc:
                self.get_logger().warn(f'Servo init failed on GPIO{servo_pin}: {exc}')
        else:
            self.get_logger().warn('gpiozero AngularServo unavailable; servo control disabled.')

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

        self._keys_lock = threading.Lock()
        self._active_keys: set[str] = set()
        self._last_key_time = 0.0
        self._keyboard_thread = None
        self._keyboard_running = False
        self._stdin_attrs = None

        if sys.stdin.isatty():
            self._start_keyboard_thread()
        else:
            self.get_logger().error(
                'stdin is not a TTY; keyboard control disabled. '
                'Run this node in a terminal (e.g. ros2 run ... with emulate_tty:=true).'
            )

        period = 0.0 if control_hz <= 0.0 else (1.0 / control_hz)
        self.create_timer(period, self._apply_keyboard_drive)

        if self._left_enc is not None and self._right_enc is not None:
            enc_period = 0.2 if encoder_debug_hz <= 0.0 else (1.0 / encoder_debug_hz)
            self.create_timer(enc_period, self._print_encoders)

        self.get_logger().info(
            f'motion_control_node started (MDD3A dual-PWM, WASD keyboard): '
            f'left=({LEFT_M1A},{LEFT_M1B}) right=({RIGHT_M2A},{RIGHT_M2B}), '
            f'full_speed={self._full_speed}, turn_speed={self._turn_speed}, '
            f'keys=W/S forward-reverse, A/D turn, Space/Q stop, '
            f'counts_per_rev={self._counts_per_rev}'
        )

    def _start_keyboard_thread(self) -> None:
        self._stdin_attrs = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._keyboard_running = True
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()
        self.get_logger().info('Keyboard control active. Focus this terminal and use WASD.')

    def _keyboard_loop(self) -> None:
        while self._keyboard_running and rclpy.ok():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            except Exception:
                break
            if not ready:
                continue

            try:
                ch = sys.stdin.read(1)
            except Exception:
                break

            if not ch:
                continue

            key = ch.lower()
            now = self.get_clock().now().nanoseconds * 1e-9
            with self._keys_lock:
                self._last_key_time = now
                if key in ('w', 'a', 's', 'd'):
                    self._active_keys.add(key)
                elif key in (' ', 'q'):
                    self._active_keys.clear()

    def _drive_label(self, left_speed: float, right_speed: float) -> str:
        if abs(left_speed) < 1e-3 and abs(right_speed) < 1e-3:
            return 'idle'
        if left_speed > 0 and right_speed > 0:
            return 'forward'
        if left_speed < 0 and right_speed < 0:
            return 'reverse'
        if left_speed < 0 and right_speed > 0:
            return 'turn_left'
        if left_speed > 0 and right_speed < 0:
            return 'turn_right'
        return 'mixed'

    def _set_motor_speed(self, motor: Motor, speed: float) -> None:
        if speed > 0.0:
            motor.forward(speed)
        elif speed < 0.0:
            motor.reverse(-speed)
        else:
            motor.stop()

    def _apply_keyboard_drive(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._keys_lock:
            if self._active_keys and (now - self._last_key_time) > self._key_stale_sec:
                self._active_keys.clear()

            keys = set(self._active_keys)

        left_speed = 0.0
        right_speed = 0.0
        if 'w' in keys:
            left_speed += self._full_speed
            right_speed += self._full_speed
        if 's' in keys:
            left_speed -= self._full_speed
            right_speed -= self._full_speed
        if 'a' in keys:
            left_speed += self._turn_speed
            right_speed -= self._turn_speed
        if 'd' in keys:
            left_speed -= self._turn_speed
            right_speed += self._turn_speed

        peak = max(abs(left_speed), abs(right_speed))
        if peak > 1.0:
            left_speed /= peak
            right_speed /= peak

        direction = self._drive_label(left_speed, right_speed)
        driving = direction != 'idle'

        self._set_motor_speed(self._left, left_speed)
        self._set_motor_speed(self._right, right_speed)

        if driving != self._driving or (driving and direction != self._direction):
            self._driving = driving
            self._direction = direction if driving else None
            if driving:
                self.get_logger().info(
                    f'keyboard drive: keys={sorted(keys)} -> {direction} '
                    f'(left={left_speed:.2f}, right={right_speed:.2f})'
                )
            else:
                self.get_logger().info('keyboard drive: stopped')

    def _print_encoders(self) -> None:
        if self._left_enc is None or self._right_enc is None:
            return
        left_steps = self._left_enc.steps
        right_steps = self._right_enc.steps
        state = self._direction if self._driving else 'idle'
        self.get_logger().info(
            f'[encoder] left={-left_steps} cnt, right={right_steps} cnt ({state})'
        )

    def _restore_terminal(self) -> None:
        self._keyboard_running = False
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=0.5)
            self._keyboard_thread = None
        if self._stdin_attrs is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_attrs)
            except Exception:
                pass
            self._stdin_attrs = None

    def _stop(self) -> None:
        self._left.stop()
        self._right.stop()

    def shutdown_motors(self) -> None:
        self._restore_terminal()
        self._stop()
        self._left.close()
        self._right.close()
        if self._servo is not None:
            try:
                self._servo.detach()
                self._servo.close()
            except Exception:
                pass
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
