import json
import math
import threading
import time
import urllib.request

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5)


def _wrap_angle_rad(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class PoseEstimationSensorFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_estimation_sensor_fusion')

        self._sub = self.create_subscription(
            PoseStamped,
            '/current_position_camera',
            self._on_camera_pose,
            10,
        )
        self._pub = self.create_publisher(PoseStamped, '/current_position', 10)

        self.declare_parameter('use_real_sensor', True)
        self.declare_parameter('remote_host', '192.168.50.2')
        self.declare_parameter('remote_port', 5003)
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('request_timeout', 1.0)
        self.declare_parameter('time_topic', '/time')
        self.declare_parameter('tca9548a_addr', 0x70)
        self.declare_parameter('tof_front_channel', 2)
        self.declare_parameter('tof_left_channel', 3)
        self.declare_parameter('tof_right_channel', 1)
        self.declare_parameter('tof_rear_channel', 0)
        self.declare_parameter('tof_min_range_m', 0.02)
        self.declare_parameter('tof_max_range_m', 0.80)
        self.declare_parameter('tof_read_fail_value_m', 0.80)
        self.declare_parameter('tof_debug_enabled', True)
        self.declare_parameter('tof_debug_interval_sec', 1.0)
        self.declare_parameter('tof_init_retries', 5)
        self.declare_parameter('tof_reinit_after_fail_count', 8)
        self.declare_parameter('mpu_enabled', True)
        self.declare_parameter('mpu_addr', 0x68)
        self.declare_parameter('mpu_poll_hz', 200.0)
        self.declare_parameter('mpu_debug_interval_sec', 0.5)
        self.declare_parameter('mpu_reinit_after_fail_count', 20)
        self.declare_parameter('i2c_frequency', 100000)

        # IMU-aided pose fusion: predict /current_position at the MPU rate
        # (dead-reckoning) and correct it with the low-rate absolute camera pose.
        self.declare_parameter('imu_fusion_enabled', True)
        self.declare_parameter('fusion_publish_hz', 100.0)
        # Body-frame axis mapping for the MPU (x=0, y=1, z=2). Defaults assume the
        # MPU is mounted flat with x=forward, y=left, z=up. Use the *_sign params
        # to flip an axis if the readings move the wrong way.
        self.declare_parameter('imu_forward_axis', 0)
        self.declare_parameter('imu_left_axis', 1)
        self.declare_parameter('imu_yaw_rate_axis', 2)
        self.declare_parameter('imu_forward_sign', 1.0)
        self.declare_parameter('imu_left_sign', 1.0)
        self.declare_parameter('imu_yaw_rate_sign', 1.0)
        # Complementary-filter correction gains applied on each camera update.
        self.declare_parameter('fusion_pos_correction_gain', 0.5)
        self.declare_parameter('fusion_vel_correction_gain', 0.1)
        self.declare_parameter('fusion_heading_correction_gain', 0.4)
        self.declare_parameter('fusion_gyro_bias_gain', 0.02)
        # Velocity bleed-off time constant (sec) to limit accel-integration drift.
        self.declare_parameter('fusion_vel_damping_tau', 1.0)
        # Ignore horizontal accel below this magnitude (m/s^2) as noise.
        self.declare_parameter('fusion_accel_deadband', 0.08)
        self.declare_parameter('fusion_max_speed', 2.0)
        # Hard-reset the estimate to the camera pose if it disagrees by more than this (m).
        self.declare_parameter('fusion_reset_pos_threshold', 0.5)
        # Zero-velocity update: clamp velocity to 0 when the robot is detected still.
        self.declare_parameter('fusion_zupt_enabled', True)
        self.declare_parameter('fusion_zupt_accel_thresh', 0.15)
        self.declare_parameter('fusion_zupt_gyro_thresh', 0.05)
        self.declare_parameter('fusion_zupt_min_duration', 0.2)
        # Stationary samples collected at startup to estimate accel/gyro bias.
        self.declare_parameter('fusion_calib_samples', 200)

        self.use_real_sensor = self.get_parameter('use_real_sensor').get_parameter_value().bool_value
        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)
        self.time_topic = self.get_parameter('time_topic').get_parameter_value().string_value
        self.tof_min_range_m = float(self.get_parameter('tof_min_range_m').get_parameter_value().double_value)
        self.tof_max_range_m = float(self.get_parameter('tof_max_range_m').get_parameter_value().double_value)
        self.tof_read_fail_value_m = float(self.get_parameter('tof_read_fail_value_m').get_parameter_value().double_value)
        self.tof_debug_enabled = self.get_parameter('tof_debug_enabled').get_parameter_value().bool_value
        self.tof_debug_interval_sec = float(
            self.get_parameter('tof_debug_interval_sec').get_parameter_value().double_value
        )
        self.tof_init_retries = int(self.get_parameter('tof_init_retries').get_parameter_value().integer_value)
        self.tof_reinit_after_fail_count = int(
            self.get_parameter('tof_reinit_after_fail_count').get_parameter_value().integer_value
        )
        self.mpu_enabled = self.get_parameter('mpu_enabled').get_parameter_value().bool_value
        self.mpu_addr = int(self.get_parameter('mpu_addr').get_parameter_value().integer_value)
        mpu_poll_hz = float(self.get_parameter('mpu_poll_hz').get_parameter_value().double_value)
        self.mpu_debug_interval_sec = float(
            self.get_parameter('mpu_debug_interval_sec').get_parameter_value().double_value
        )
        self.mpu_reinit_after_fail_count = int(
            self.get_parameter('mpu_reinit_after_fail_count').get_parameter_value().integer_value
        )
        self.i2c_frequency = int(self.get_parameter('i2c_frequency').get_parameter_value().integer_value)

        self.imu_fusion_enabled = self.get_parameter('imu_fusion_enabled').get_parameter_value().bool_value
        fusion_publish_hz = float(self.get_parameter('fusion_publish_hz').get_parameter_value().double_value)
        self.imu_forward_axis = int(self.get_parameter('imu_forward_axis').get_parameter_value().integer_value)
        self.imu_left_axis = int(self.get_parameter('imu_left_axis').get_parameter_value().integer_value)
        self.imu_yaw_rate_axis = int(self.get_parameter('imu_yaw_rate_axis').get_parameter_value().integer_value)
        self.imu_forward_sign = float(self.get_parameter('imu_forward_sign').get_parameter_value().double_value)
        self.imu_left_sign = float(self.get_parameter('imu_left_sign').get_parameter_value().double_value)
        self.imu_yaw_rate_sign = float(self.get_parameter('imu_yaw_rate_sign').get_parameter_value().double_value)
        self.fusion_pos_gain = float(self.get_parameter('fusion_pos_correction_gain').get_parameter_value().double_value)
        self.fusion_vel_gain = float(self.get_parameter('fusion_vel_correction_gain').get_parameter_value().double_value)
        self.fusion_heading_gain = float(
            self.get_parameter('fusion_heading_correction_gain').get_parameter_value().double_value
        )
        self.fusion_gyro_bias_gain = float(
            self.get_parameter('fusion_gyro_bias_gain').get_parameter_value().double_value
        )
        self.fusion_vel_damping_tau = float(
            self.get_parameter('fusion_vel_damping_tau').get_parameter_value().double_value
        )
        self.fusion_accel_deadband = float(
            self.get_parameter('fusion_accel_deadband').get_parameter_value().double_value
        )
        self.fusion_max_speed = float(self.get_parameter('fusion_max_speed').get_parameter_value().double_value)
        self.fusion_reset_pos_threshold = float(
            self.get_parameter('fusion_reset_pos_threshold').get_parameter_value().double_value
        )
        self.fusion_zupt_enabled = self.get_parameter('fusion_zupt_enabled').get_parameter_value().bool_value
        self.fusion_zupt_accel_thresh = float(
            self.get_parameter('fusion_zupt_accel_thresh').get_parameter_value().double_value
        )
        self.fusion_zupt_gyro_thresh = float(
            self.get_parameter('fusion_zupt_gyro_thresh').get_parameter_value().double_value
        )
        self.fusion_zupt_min_duration = float(
            self.get_parameter('fusion_zupt_min_duration').get_parameter_value().double_value
        )
        self.fusion_calib_samples = int(self.get_parameter('fusion_calib_samples').get_parameter_value().integer_value)

        if fusion_publish_hz < 0.0:
            fusion_publish_hz = 0.0
        self._fusion_publish_min_dt = 0.0 if fusion_publish_hz <= 0.0 else (1.0 / fusion_publish_hz)
        if self.fusion_vel_damping_tau <= 0.0:
            self.fusion_vel_damping_tau = 1.0
        if self.fusion_calib_samples < 0:
            self.fusion_calib_samples = 0
        for axis_name in ('imu_forward_axis', 'imu_left_axis', 'imu_yaw_rate_axis'):
            axis_val = getattr(self, axis_name)
            if axis_val < 0 or axis_val > 2:
                self.get_logger().warn(f'{axis_name}={axis_val} out of range [0,2]; clamping')
                setattr(self, axis_name, min(max(axis_val, 0), 2))

        if self.request_timeout <= 0.0:
            self.request_timeout = 1.0
        if self.tof_debug_interval_sec <= 0.0:
            self.tof_debug_interval_sec = 1.0
        if self.tof_init_retries <= 0:
            self.tof_init_retries = 1
        if self.tof_reinit_after_fail_count <= 0:
            self.tof_reinit_after_fail_count = 8
        if self.mpu_debug_interval_sec <= 0.0:
            self.mpu_debug_interval_sec = 0.5
        if self.mpu_reinit_after_fail_count <= 0:
            self.mpu_reinit_after_fail_count = 20
        if mpu_poll_hz <= 0.0:
            mpu_poll_hz = 200.0
        if self.i2c_frequency <= 0:
            self.i2c_frequency = 100000

        self._latest_time_s: float | None = None
        self._last_tof_debug = self.get_clock().now()
        self._tof_backend = 'none'
        self._tof_readers = {}
        self._tof_consecutive_fail_count = 0
        self._i2c = None
        self._i2c_lock = threading.Lock()
        self._tof_mux = None
        self._mux_addr = None
        self._mpu = None
        self._mpu_backend = 'none'
        self._last_mpu_debug = self.get_clock().now()
        self._mpu_error_logged = False
        self._mpu_consecutive_fail_count = 0

        # --- IMU-aided pose fusion state (world frame) ---
        self._fusion_lock = threading.Lock()
        self._fusion_ready = False          # set once first camera pose arrives
        self._fusion_x = 0.0
        self._fusion_y = 0.0
        self._fusion_vx = 0.0
        self._fusion_vy = 0.0
        self._fusion_heading = 0.0          # rad, world frame
        self._gyro_bias = 0.0               # rad/s, estimated online + at startup
        self._accel_bias_fwd = 0.0          # m/s^2, body frame
        self._accel_bias_left = 0.0
        self._calibrating = self.fusion_calib_samples > 0
        self._calib_count = 0
        self._calib_fwd_sum = 0.0
        self._calib_left_sum = 0.0
        self._calib_gyro_sum = 0.0
        self._last_mpu_time: float | None = None
        self._last_cam_time: float | None = None
        self._last_cam_x: float | None = None
        self._last_cam_y: float | None = None
        self._last_fusion_pub_time = 0.0
        self._still_since: float | None = None
        self._camera_frame_id = 'map'

        if self.time_topic:
            self.create_subscription(String, self.time_topic, self._on_time, 10)

        self._urls = [
            f'http://{self.remote_host}:{self.remote_port}/data/simulation_data',
            f'http://{self.remote_host}:{self.remote_port}/simulation_data',
        ]
        self._url_index = 0
        self._pub_radar_sensor = self.create_publisher(String, '/radar_sensor', 10)
        self._last_radar_text = None
        self._error_logged = False
        self._tof_error_logged = False

        if self.use_real_sensor:
            self._tof_readers = self._build_tof_readers()
            self._tof_backend = 'vl53l0x' if self._tof_readers else 'none'
            if self.mpu_enabled:
                self._mpu = self._build_mpu()
                self._mpu_backend = 'mpu6050' if self._mpu is not None else 'none'

        # Fusion is active only when explicitly enabled AND an MPU is available.
        # Otherwise /current_position is republished 1:1 from the camera pose.
        self._fusion_active = self.imu_fusion_enabled and self._mpu is not None

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        # Separate callback groups so the high-rate MPU timer runs on its own
        # thread and is not blocked by the slower ToF poll under a MultiThreadedExecutor.
        self._tof_cb_group = MutuallyExclusiveCallbackGroup()
        self._mpu_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(period, self._poll_once, callback_group=self._tof_cb_group)

        if self._mpu is not None:
            self.create_timer(1.0 / mpu_poll_hz, self._poll_mpu, callback_group=self._mpu_cb_group)

        if self.use_real_sensor:
            self.get_logger().info(
                'pose_estimation_sensor_fusion started in real-sensor mode; '
                f'backend={self._tof_backend}, mpu={self._mpu_backend}, '
                f'imu_fusion={"on" if self._fusion_active else "off"}, '
                f'poll_hz={poll_hz}, mpu_poll_hz={mpu_poll_hz}, '
                f'i2c_hz={self.i2c_frequency}, time_topic={self.time_topic}'
            )
        else:
            self.get_logger().info(
                f'pose_estimation_sensor_fusion started in simulation mode; upstream={self._urls}, poll_hz={poll_hz}'
            )

    def _on_camera_pose(self, msg: PoseStamped) -> None:
        if not self._fusion_active:
            # No IMU fusion: pass the absolute camera pose straight through.
            self._pub.publish(msg)
            return

        meas_x = float(msg.pose.position.x)
        meas_y = float(msg.pose.position.y)
        meas_heading = _yaw_from_quaternion(
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        if msg.header.frame_id:
            self._camera_frame_id = msg.header.frame_id
        self._apply_camera_correction(meas_x, meas_y, meas_heading)

    def _apply_camera_correction(self, meas_x: float, meas_y: float, meas_heading: float) -> None:
        now = time.monotonic()
        with self._fusion_lock:
            if not self._fusion_ready:
                # First absolute fix: seed the estimate directly from the camera.
                self._fusion_x = meas_x
                self._fusion_y = meas_y
                self._fusion_heading = meas_heading
                self._fusion_vx = 0.0
                self._fusion_vy = 0.0
                self._fusion_ready = True
            else:
                innov_x = meas_x - self._fusion_x
                innov_y = meas_y - self._fusion_y
                dist = math.hypot(innov_x, innov_y)
                if dist > self.fusion_reset_pos_threshold:
                    # Estimate diverged (e.g. relocalization jump): hard reset.
                    self._fusion_x = meas_x
                    self._fusion_y = meas_y
                    self._fusion_vx = 0.0
                    self._fusion_vy = 0.0
                else:
                    self._fusion_x += self.fusion_pos_gain * innov_x
                    self._fusion_y += self.fusion_pos_gain * innov_y
                    if self._last_cam_time is not None:
                        dt_cam = now - self._last_cam_time
                        if dt_cam > 1e-3:
                            # Nudge velocity using the position innovation (alpha-beta filter).
                            self._fusion_vx += self.fusion_vel_gain * innov_x / dt_cam
                            self._fusion_vy += self.fusion_vel_gain * innov_y / dt_cam

                heading_err = _wrap_angle_rad(meas_heading - self._fusion_heading)
                self._fusion_heading = _wrap_angle_rad(
                    self._fusion_heading + self.fusion_heading_gain * heading_err
                )
                # Drive the gyro bias estimate so integrated heading tracks the camera.
                if self._last_cam_time is not None:
                    dt_cam = now - self._last_cam_time
                    if dt_cam > 1e-3:
                        self._gyro_bias -= self.fusion_gyro_bias_gain * heading_err / dt_cam

            self._last_cam_time = now
            self._last_cam_x = meas_x
            self._last_cam_y = meas_y

    def _on_time(self, msg: String) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            return
        try:
            self._latest_time_s = float(raw)
        except ValueError:
            self._latest_time_s = None

    def _resolve_time_s(self) -> float:
        if self._latest_time_s is not None:
            return self._latest_time_s
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _replace_radar_timestamp(self, radar_text: str) -> str:
        parts = [part.strip() for part in radar_text.split(',')]
        if len(parts) < 5:
            return radar_text
        parts[0] = f'{self._resolve_time_s():.6f}'
        return ','.join(parts[:5])

    def _poll_once(self) -> None:
        if self.use_real_sensor:
            radar_text = self._build_radar_text_from_tof()
            if not radar_text:
                return
            if radar_text == self._last_radar_text:
                return
            msg = String()
            msg.data = radar_text
            self._pub_radar_sensor.publish(msg)
            self._last_radar_text = radar_text
            return

        try:
            body = self._fetch_simulation_data_body()
            payload = json.loads(body.decode('utf-8', errors='ignore'))
            if not isinstance(payload, dict):
                raise ValueError('simulation_data payload must be a JSON object')
        except Exception as exc:
            if not self._error_logged:
                self.get_logger().warn(f'upstream fetch failed for {self._urls}: {exc}')
                self._error_logged = True
            return

        self._error_logged = False
        radar_text = str(payload.get('radar_sensor', '')).strip()
        if not radar_text:
            return
        radar_text = self._replace_radar_timestamp(radar_text)
        if radar_text == self._last_radar_text:
            return

        msg = String()
        msg.data = radar_text
        self._pub_radar_sensor.publish(msg)
        self._last_radar_text = radar_text

    def _get_i2c(self):
        if self._i2c is not None:
            return self._i2c
        import board
        import busio
        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=self.i2c_frequency)
        return self._i2c

    def _build_tof_readers(self):
        try:
            import adafruit_vl53l0x
            import adafruit_tca9548a
        except Exception as exc:
            self.get_logger().error(f'use_real_sensor=true but ToF dependencies unavailable: {exc}')
            return {}

        mux_addr = int(self.get_parameter('tca9548a_addr').get_parameter_value().integer_value)
        channels = {
            'front': int(self.get_parameter('tof_front_channel').get_parameter_value().integer_value),
            'left': int(self.get_parameter('tof_left_channel').get_parameter_value().integer_value),
            'right': int(self.get_parameter('tof_right_channel').get_parameter_value().integer_value),
            'rear': int(self.get_parameter('tof_rear_channel').get_parameter_value().integer_value),
        }

        try:
            i2c = self._get_i2c()
            tca = adafruit_tca9548a.TCA9548A(i2c, address=mux_addr)
        except Exception as exc:
            self.get_logger().error(f'failed to initialise TCA9548A at 0x{mux_addr:02X}: {exc}')
            return {}

        self._tof_mux = tca
        self._mux_addr = mux_addr

        readers = {}
        for name, ch in channels.items():
            sensor = None
            last_exc = None
            for _ in range(self.tof_init_retries):
                try:
                    sensor = adafruit_vl53l0x.VL53L0X(tca[ch])
                    break
                except Exception as exc:
                    last_exc = exc
                    time.sleep(0.05)

            if sensor is None:
                detected = None
                try:
                    detected = [hex(a) for a in tca[ch].scan()]
                except Exception as scan_exc:
                    detected = f'<scan failed: {scan_exc}>'
                self.get_logger().warn(
                    f'failed to init ToF {name} on TCA9548A channel {ch}: {last_exc}; '
                    f'i2c scan on channel {ch} = {detected} (expect 0x29 if sensor present)'
                )
                continue

            # Continuous ranging makes each .range a fast register read instead of a
            # ~30ms blocking single-shot, so ToF reads hold the shared I2C lock only
            # briefly and don't starve the high-rate MPU timer.
            try:
                sensor.start_continuous()
            except Exception as exc:
                self.get_logger().warn(f'ToF {name} start_continuous failed, using single-shot: {exc}')

            def _make_reader(dev):
                return lambda: float(dev.range) / 1000.0

            readers[name] = _make_reader(sensor)

        if not readers:
            self.get_logger().error(
                'no ToF sensors initialized; pose_estimation_sensor_fusion cannot publish real radar data'
            )
        return readers

    def _build_mpu(self):
        try:
            import adafruit_mpu6050
        except Exception as exc:
            self.get_logger().error(f'mpu_enabled=true but adafruit_mpu6050 unavailable: {exc}')
            return None

        try:
            i2c = self._get_i2c()
            mpu = adafruit_mpu6050.MPU6050(i2c, address=self.mpu_addr)
        except Exception as exc:
            self.get_logger().error(f'failed to init MPU6050 at 0x{self.mpu_addr:02X}: {exc}')
            return None

        self.get_logger().info(f'MPU6050 initialised at 0x{self.mpu_addr:02X}')
        return mpu

    def _deselect_mux_channels(self) -> None:
        # Disable all TCA9548A channels so no VL53L0X (0x29) is bridged onto the
        # bus while we talk to the directly-attached MPU6050 (0x68). Without this
        # the leftover open channel corrupts the next ToF transaction.
        if self._i2c is None or self._mux_addr is None:
            return
        locked = False
        try:
            while not self._i2c.try_lock():
                pass
            locked = True
            self._i2c.writeto(self._mux_addr, bytes([0x00]))
        except Exception:
            pass
        finally:
            if locked:
                self._i2c.unlock()

    def _poll_mpu(self) -> None:
        if self._mpu is None:
            return

        with self._i2c_lock:
            self._deselect_mux_channels()
            try:
                ax, ay, az = self._mpu.acceleration
                gx, gy, gz = self._mpu.gyro
            except Exception as exc:
                self._mpu_consecutive_fail_count += 1
                if not self._mpu_error_logged:
                    self.get_logger().warn(f'MPU6050 read failed: {exc}')
                    self._mpu_error_logged = True
                if self._mpu_consecutive_fail_count >= self.mpu_reinit_after_fail_count:
                    self.get_logger().warn(
                        'MPU6050 reads failed repeatedly; attempting reinitialization '
                        f'(count={self._mpu_consecutive_fail_count})'
                    )
                    self._mpu = self._build_mpu()
                    self._mpu_backend = 'mpu6050' if self._mpu is not None else 'none'
                    self._mpu_consecutive_fail_count = 0
                return

            self._mpu_consecutive_fail_count = 0
            self._mpu_error_logged = False

        if self._fusion_active:
            self._fusion_step(ax, ay, az, gx, gy, gz)

        self._maybe_log_mpu_debug(ax, ay, az, gx, gy, gz)

    def _fusion_step(self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float) -> None:
        # Map raw MPU axes to robot body frame (forward / left / yaw-rate).
        accel = (ax, ay, az)
        gyro = (gx, gy, gz)
        a_fwd = accel[self.imu_forward_axis] * self.imu_forward_sign
        a_left = accel[self.imu_left_axis] * self.imu_left_sign
        yaw_rate = gyro[self.imu_yaw_rate_axis] * self.imu_yaw_rate_sign

        now = time.monotonic()

        # Estimate accel/gyro bias from the first stationary samples at startup.
        if self._calibrating:
            self._calib_fwd_sum += a_fwd
            self._calib_left_sum += a_left
            self._calib_gyro_sum += yaw_rate
            self._calib_count += 1
            if self._calib_count >= self.fusion_calib_samples:
                n = float(self._calib_count)
                self._accel_bias_fwd = self._calib_fwd_sum / n
                self._accel_bias_left = self._calib_left_sum / n
                self._gyro_bias = self._calib_gyro_sum / n
                self._calibrating = False
                self.get_logger().info(
                    'IMU bias calibrated | '
                    f'accel_fwd={self._accel_bias_fwd:+.3f} accel_left={self._accel_bias_left:+.3f} '
                    f'gyro={self._gyro_bias:+.4f} (assumed stationary over {self._calib_count} samples)'
                )
            self._last_mpu_time = now
            return

        if self._last_mpu_time is None:
            self._last_mpu_time = now
            return
        dt = now - self._last_mpu_time
        self._last_mpu_time = now
        if dt <= 0.0 or dt > 0.05:
            # Skip integration on the first tick or after a stall (keeps dt sane).
            return

        a_fwd -= self._accel_bias_fwd
        a_left -= self._accel_bias_left
        yaw_rate_corr = yaw_rate - self._gyro_bias

        # Zero-velocity update: when both accel and gyro are quiet for long enough,
        # the robot is treated as stationary and velocity is clamped to kill drift.
        is_still = (
            self.fusion_zupt_enabled
            and math.hypot(a_fwd, a_left) < self.fusion_zupt_accel_thresh
            and abs(yaw_rate_corr) < self.fusion_zupt_gyro_thresh
        )

        with self._fusion_lock:
            if not self._fusion_ready:
                # No absolute fix yet; can't publish a meaningful world pose.
                return

            self._fusion_heading = _wrap_angle_rad(self._fusion_heading + yaw_rate_corr * dt)

            if is_still:
                if self._still_since is None:
                    self._still_since = now
                if (now - self._still_since) >= self.fusion_zupt_min_duration:
                    self._fusion_vx = 0.0
                    self._fusion_vy = 0.0
            else:
                self._still_since = None
                # Deadband then rotate body-frame accel into the world frame.
                if abs(a_fwd) < self.fusion_accel_deadband:
                    a_fwd = 0.0
                if abs(a_left) < self.fusion_accel_deadband:
                    a_left = 0.0
                cos_h = math.cos(self._fusion_heading)
                sin_h = math.sin(self._fusion_heading)
                ax_world = a_fwd * cos_h - a_left * sin_h
                ay_world = a_fwd * sin_h + a_left * cos_h
                self._fusion_vx += ax_world * dt
                self._fusion_vy += ay_world * dt
                # Bleed off velocity to bound accel-integration drift between fixes.
                decay = math.exp(-dt / self.fusion_vel_damping_tau)
                self._fusion_vx *= decay
                self._fusion_vy *= decay

            speed = math.hypot(self._fusion_vx, self._fusion_vy)
            if speed > self.fusion_max_speed and speed > 0.0:
                scale = self.fusion_max_speed / speed
                self._fusion_vx *= scale
                self._fusion_vy *= scale

            self._fusion_x += self._fusion_vx * dt
            self._fusion_y += self._fusion_vy * dt

            pub_x = self._fusion_x
            pub_y = self._fusion_y
            pub_heading = self._fusion_heading

        if self._fusion_publish_min_dt > 0.0 and (now - self._last_fusion_pub_time) < self._fusion_publish_min_dt:
            return
        self._last_fusion_pub_time = now
        self._publish_fused_pose(pub_x, pub_y, pub_heading)

    def _publish_fused_pose(self, x: float, y: float, heading: float) -> None:
        qx, qy, qz, qw = _quaternion_from_yaw(heading)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub.publish(msg)

    def _maybe_log_mpu_debug(
        self, ax: float, ay: float, az: float, gx: float, gy: float, gz: float
    ) -> None:
        now = self.get_clock().now()
        delta = (now - self._last_mpu_debug).nanoseconds / 1_000_000_000.0
        if delta < self.mpu_debug_interval_sec:
            return
        self._last_mpu_debug = now
        base = (
            f'mpu accel m/s^2 | ax={ax:+.3f} ay={ay:+.3f} az={az:+.3f} | '
            f'gyro rad/s | gx={gx:+.3f} gy={gy:+.3f} gz={gz:+.3f}'
        )
        if self._fusion_active:
            state = 'calibrating' if self._calibrating else ('ready' if self._fusion_ready else 'waiting-camera')
            base += (
                f' | fusion[{state}] x={self._fusion_x:+.3f} y={self._fusion_y:+.3f} '
                f'hdg={math.degrees(self._fusion_heading):+.1f}deg '
                f'v=({self._fusion_vx:+.3f},{self._fusion_vy:+.3f})'
            )
        self.get_logger().info(base)

    def _build_radar_text_from_tof(self) -> str | None:
        if not self._tof_readers:
            if not self._tof_error_logged:
                self.get_logger().error('real-sensor mode enabled but no ToF readers available')
                self._tof_error_logged = True
            return None

        values = {}
        debug_values = {}
        directions = ('front', 'right', 'left', 'rear')
        with self._i2c_lock:
            for name in directions:
                reader = self._tof_readers.get(name)
                if reader is None:
                    values[name] = self.tof_read_fail_value_m
                    debug_values[name] = (None, 'FAIL', values[name])
                    continue

                try:
                    raw = float(reader())
                    if raw < self.tof_min_range_m:
                        pub = self.tof_min_range_m
                        status = 'CLAMP_MIN'
                    elif raw > self.tof_max_range_m:
                        pub = self.tof_max_range_m
                        status = 'CLAMP_MAX'
                    else:
                        pub = raw
                        status = 'OK'
                    values[name] = pub
                    debug_values[name] = (raw, status, pub)
                except Exception:
                    values[name] = self.tof_read_fail_value_m
                    debug_values[name] = (None, 'FAIL', values[name])

            all_failed = all(debug_values[name][1] == 'FAIL' for name in directions)
            if all_failed:
                self._tof_consecutive_fail_count += 1
            else:
                self._tof_consecutive_fail_count = 0

            if self._tof_consecutive_fail_count >= self.tof_reinit_after_fail_count:
                self.get_logger().warn(
                    'all ToF reads failed repeatedly; attempting TCA9548A reinitialization '
                    f'(count={self._tof_consecutive_fail_count})'
                )
                self._tof_readers = self._build_tof_readers()
                self._tof_backend = 'vl53l0x' if self._tof_readers else 'none'
                self._tof_consecutive_fail_count = 0

        self._maybe_log_radar_debug(debug_values)

        timestamp_s = self._resolve_time_s()
        return (
            f'{timestamp_s:.6f},'
            f'{values["front"]:.3f},{values["right"]:.3f},{values["left"]:.3f},{values["rear"]:.3f}'
        )

    def _maybe_log_radar_debug(self, debug_values: dict[str, tuple[float | None, str, float]]) -> None:
        if not self.tof_debug_enabled:
            return
        now = self.get_clock().now()
        delta = (now - self._last_tof_debug).nanoseconds / 1_000_000_000.0
        if delta < self.tof_debug_interval_sec:
            return
        self._last_tof_debug = now

        summary = []
        for name in ('front', 'right', 'left', 'rear'):
            raw, status, pub = debug_values[name]
            raw_mm = 'None' if raw is None else f'{int(raw * 1000.0)}'
            summary.append(f'{name}:raw_mm={raw_mm},status={status},pub={pub:.3f}')
        self.get_logger().info('radar tof diag | ' + ' | '.join(summary))

    def _fetch_simulation_data_body(self) -> bytes:
        first_error: Exception | None = None
        total = len(self._urls)
        for offset in range(total):
            idx = (self._url_index + offset) % total
            url = self._urls[idx]
            try:
                req = urllib.request.Request(url, headers={'Connection': 'close'})
                with urllib.request.urlopen(req, timeout=self.request_timeout) as res:
                    body = res.read()
                self._url_index = idx
                return body
            except Exception as exc:
                if first_error is None:
                    first_error = exc
                continue

        if first_error is None:
            raise RuntimeError('no simulation_data urls configured')
        raise first_error

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseEstimationSensorFusionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
