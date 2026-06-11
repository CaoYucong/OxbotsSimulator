"""Motion control node for the Cytron MDD3A dual DC motor driver.

Usage for awsd control:
first run on mac:
PI_USER=caoyucong
PI_IP=100.99.175.23

rsync -avz \
  ~/Desktop/OxbotsSimulator/ROS/ros2_ws/src/unibots/unibots/motion_control_node.py \
  $PI_USER@$PI_IP:~/OxbotsSimulator/ROS/ros2_ws/src/unibots/unibots/

then run on pi:
source /opt/ros/jazzy/setup.bash
cd ~/OxbotsSimulator/ROS/ros2_ws
colcon build --symlink-install
source install/setup.bash   

ros2 run unibots motion_control_node

Hardware:
  - Raspberry Pi 5
  - Cytron MDD3A dual DC motor driver (dual-PWM input logic, NOT PWM+DIR)
  - 2x JGA25-371 12V encoder gear motors
  - 3S LiPo battery -> MDD3A VB+/VB-
  - Separate 5V buck converter powers the Pi

Control wiring (BCM numbering, MDD3A silkscreen):
  - M1 channel: M1A = GPIO18, M1B = GPIO19
  - M2 channel: M2A = GPIO12, M2B = GPIO13
  On this robot's harness M2 drives the physical left wheel and M1 drives the right.
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
  - W   : forward 20 cm (encoder closed-loop, stops automatically)
  - S   : reverse 20 cm (encoder closed-loop, stops automatically)
  - A/D : turn left / turn right (hold)
  - Q   : left wheel forward 10 rev (encoder closed-loop)
  - P   : right wheel forward 10 rev (encoder closed-loop)
  - Space : stop
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

# --- Servo pin (BCM numbering) ---
SERVO_PIN: int = 21

FULL_SPEED: float = 1.0       # PWM duty cycle for full-speed forward
TURN_SPEED: float = 1.0       # PWM duty cycle for in-place turns

COUNTS_PER_REV: float = 488.0  # quadrature counts per output-shaft revolution
COUNTS_PER_METER: float = 3300  # encoder counts per meter of travel 
DEFAULT_MOVE_DIST: float = 0.2   # default straight move distance on W/S keys (meters)
DEFAULT_WHEEL_REVS: float = 10.0  # default single-wheel spin on Q/P keys (revolutions)
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
        # self.declare_parameter('counts_per_rev', COUNTS_PER_REV)
        self.declare_parameter('encoder_debug_hz', ENCODER_DEBUG_HZ)
        self.declare_parameter('servo_pin', SERVO_PIN)
        self._full_speed = float(self.get_parameter('full_speed').get_parameter_value().double_value) or FULL_SPEED
        self._turn_speed = float(self.get_parameter('turn_speed').get_parameter_value().double_value) or TURN_SPEED
        control_hz = float(self.get_parameter('control_hz').get_parameter_value().double_value) or CONTROL_HZ
        self._key_stale_sec = float(self.get_parameter('key_stale_sec').get_parameter_value().double_value) or KEY_STALE_SEC
        # self._counts_per_rev = float(self.get_parameter('counts_per_rev').get_parameter_value().double_value) or COUNTS_PER_REV
        encoder_debug_hz = float(self.get_parameter('encoder_debug_hz').get_parameter_value().double_value) or ENCODER_DEBUG_HZ
        servo_pin = int(self.get_parameter('servo_pin').get_parameter_value().integer_value) or SERVO_PIN

        self._left = Motor(LEFT_MOTOR_A, LEFT_MOTOR_B)
        self._right = Motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B)
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
        self._distance_move_requested: str | None = None
        self._distance_moving = False
        self._distance_direction: str | None = None
        self._distance_start_left = 0
        self._distance_start_right = 0
        self._distance_target_counts = 0.0
        self._wheel_spin_requested: str | None = None
        self._wheel_spinning = False
        self._wheel_spin_side: str | None = None
        self._wheel_spin_start_count = 0
        self._wheel_spin_target_counts = 0.0
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
            f'left=({LEFT_MOTOR_A},{LEFT_MOTOR_B}) right=({RIGHT_MOTOR_A},{RIGHT_MOTOR_B}), '
            f'full_speed={self._full_speed}, turn_speed={self._turn_speed}, '
            f'keys=W/S move {DEFAULT_MOVE_DIST:.2f}m, A/D turn, '
            f'Q left {DEFAULT_WHEEL_REVS:.0f}rev, P right {DEFAULT_WHEEL_REVS:.0f}rev, Space stop, '
            f'counts_per_meter={COUNTS_PER_METER}, counts_per_rev={COUNTS_PER_REV}'
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
                if key == 'w':
                    self._distance_move_requested = 'forward'
                elif key == 's':
                    self._distance_move_requested = 'reverse'
                elif key == 'q':
                    self._wheel_spin_requested = 'left'
                elif key == 'e':
                    self._wheel_spin_requested = 'right'
                elif key in ('a', 'd'):
                    self._active_keys.add(key)
                elif key == ' ':
                    self._active_keys.clear()
                    self._distance_move_requested = None
                    self._distance_moving = False
                    self._distance_direction = None
                    self._wheel_spin_requested = None
                    self._wheel_spinning = False
                    self._wheel_spin_side = None

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

    def _encoder_counts(self) -> tuple[int, int]:
        """Signed counts with left negated so both wheels increase when moving forward."""
        return (-self._left_enc.steps, self._right_enc.steps)

    def _wheel_encoder_count(self, side: str) -> int:
        if side == 'left':
            return -self._left_enc.steps
        return self._right_enc.steps

    def _apply_keyboard_drive(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._keys_lock:
            if self._active_keys and (now - self._last_key_time) > self._key_stale_sec:
                self._active_keys.clear()

            move_requested = self._distance_move_requested
            self._distance_move_requested = None
            wheel_spin_requested = self._wheel_spin_requested
            self._wheel_spin_requested = None
            keys = set(self._active_keys)

        motion_busy = self._distance_moving or self._wheel_spinning

        if move_requested and not motion_busy:
            if self._left_enc is None or self._right_enc is None:
                self.get_logger().warn(f'{move_requested} move ignored: encoders unavailable.')
            else:
                left_cnt, right_cnt = self._encoder_counts()
                self._distance_moving = True
                self._distance_direction = move_requested
                self._distance_start_left = left_cnt
                self._distance_start_right = right_cnt
                self._distance_target_counts = COUNTS_PER_METER * DEFAULT_MOVE_DIST
                self.get_logger().info(
                    f'{move_requested} {DEFAULT_MOVE_DIST:.2f} m started '
                    f'(target={self._distance_target_counts:.1f} counts)'
                )
            motion_busy = True

        if wheel_spin_requested and not motion_busy:
            if self._left_enc is None or self._right_enc is None:
                self.get_logger().warn(f'{wheel_spin_requested} wheel spin ignored: encoders unavailable.')
            else:
                self._wheel_spinning = True
                self._wheel_spin_side = wheel_spin_requested
                self._wheel_spin_start_count = self._wheel_encoder_count(wheel_spin_requested)
                self._wheel_spin_target_counts = DEFAULT_WHEEL_REVS * COUNTS_PER_REV
                self.get_logger().info(
                    f'{wheel_spin_requested} wheel {DEFAULT_WHEEL_REVS:.0f} rev started '
                    f'(target={self._wheel_spin_target_counts:.1f} counts)'
                )

        if self._distance_moving:
            left_cnt, right_cnt = self._encoder_counts()
            left_delta = left_cnt - self._distance_start_left
            right_delta = right_cnt - self._distance_start_right
            avg_delta = (left_delta + right_delta) / 2.0
            direction = self._distance_direction or 'forward'
            target = self._distance_target_counts

            if direction == 'forward':
                done = avg_delta >= target
                drive_speed = self._full_speed
            else:
                done = avg_delta <= -target
                drive_speed = -self._full_speed

            if done:
                self._distance_moving = False
                self._distance_direction = None
                self._set_motor_speed(self._left, 0.0)
                self._set_motor_speed(self._right, 0.0)
                if self._driving:
                    self._driving = False
                    self._direction = None
                    self.get_logger().info('keyboard drive: stopped')
                self.get_logger().info(
                    f'{direction} {DEFAULT_MOVE_DIST:.2f} m done (avg_delta={avg_delta:.1f} counts)'
                )
            else:
                self._set_motor_speed(self._left, drive_speed)
                self._set_motor_speed(self._right, drive_speed)
                if not self._driving or self._direction != direction:
                    self._driving = True
                    self._direction = direction
                    self.get_logger().info(
                        f'keyboard drive: {direction} {DEFAULT_MOVE_DIST:.2f} m '
                        f'(left={drive_speed:.2f}, right={drive_speed:.2f}, '
                        f'avg_delta={avg_delta:.1f}/{"+" if direction == "forward" else "-"}{target:.1f})'
                    )
            return

        if self._wheel_spinning:
            side = self._wheel_spin_side or 'left'
            count = self._wheel_encoder_count(side)
            delta = count - self._wheel_spin_start_count
            target = self._wheel_spin_target_counts

            if delta >= target:
                self._wheel_spinning = False
                self._wheel_spin_side = None
                self._set_motor_speed(self._left, 0.0)
                self._set_motor_speed(self._right, 0.0)
                if self._driving:
                    self._driving = False
                    self._direction = None
                    self.get_logger().info('keyboard drive: stopped')
                self.get_logger().info(
                    f'{side} wheel {DEFAULT_WHEEL_REVS:.0f} rev done (delta={delta:.1f} counts)'
                )
            else:
                if side == 'left':
                    self._set_motor_speed(self._left, self._full_speed)
                    self._set_motor_speed(self._right, 0.0)
                    spin_label = 'left_spin'
                else:
                    self._set_motor_speed(self._left, 0.0)
                    self._set_motor_speed(self._right, self._full_speed)
                    spin_label = 'right_spin'
                if not self._driving or self._direction != spin_label:
                    self._driving = True
                    self._direction = spin_label
                    self.get_logger().info(
                        f'keyboard drive: {side} wheel {DEFAULT_WHEEL_REVS:.0f} rev '
                        f'(delta={delta:.1f}/{target:.1f} counts)'
                    )
            return

        left_speed = 0.0
        right_speed = 0.0
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
