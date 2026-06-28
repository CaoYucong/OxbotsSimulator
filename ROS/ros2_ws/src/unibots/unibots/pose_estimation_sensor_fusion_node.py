"""Sensor-fusion pose node: camera absolute pose + wheel odometry + ToF radar.

Inputs:
  - /current_position_camera (geometry_msgs/PoseStamped):
      Absolute robot-origin pose in the 'map' frame estimated from AprilTags.
      Low rate (~few Hz) and latent, but drift-free. Its header.stamp is the
      camera (system) timestamp the pose is valid for. Used as a correction
      anchor.
  - /wheel_joint_states (sensor_msgs/JointState):
      High-rate (~50 Hz) left/right wheel angle (rad) and angular velocity
      (rad/s) from the encoders. Used to dead-reckon between camera fixes.
  - /robot_motion_status (std_msgs/String):
      'moving' or 'stopped' from motion_control_node. Debug logs are emitted
      only while the robot is stopped.
  - /time (std_msgs/String, optional):
      Simulation clock used to stamp the first field of /radar_sensor payloads.

Outputs:
  - /current_position (geometry_msgs/PoseStamped):
      Fused pose published at the wheel-odometry rate. Computed as:
          current = camera_anchor  (+)  (odom_now (-) odom_at_camera_stamp)
      i.e. take the most recent camera pose and add the relative wheel motion
      accumulated since that camera frame's timestamp.
  - /radar_sensor (std_msgs/String):
      Comma-separated distances in metres:
          time,front,right,left,rear
      Real hardware: a single front VL53L0X/VL53L1X ToF (default I2C 0x29) on the
      shared bus (MPU6050 may also be present at 0x68). Other radar directions
      publish the configured max/fail range. Simulation: payload is fetched from
      upstream simulation_data.radar_sensor.

Geometry / calibration:
  - Differential drive, both driven wheels are at the REAR.
  - 488 encoder counts per wheel revolution. The wheel angles arrive in rad and
    are converted back to counts here so calibration is done in raw counts.
  - count_average_per_meter: mean of both wheels' count change per 1 m of
    straight travel (drive straight 1 m, read the average count delta).
  - count_diff_per_degree: (right_count_change - left_count_change) per 1 deg of
    in-place rotation (rotate a known angle, divide the count diff by degrees).
  - The robot coordinate origin is NOT on the wheel axle: it sits 40 mm
    forward of the midpoint of the two rear wheels. Odometry is integrated at
    the wheel-axle midpoint and converted to the robot origin on publish.
"""

import json
import math
import threading
import time
import urllib.request
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

COUNTS_PER_REV: float = 488.0  # encoder counts per wheel revolution
# Calibration constants (measured on the real robot):
#   count_average_per_meter: mean of both wheels' count change per 1 m of straight travel.
#   count_diff_per_degree:   (right_count_change - left_count_change) per 1 deg of in-place rotation.
# Defaults reproduce the previous geometry (radius 0.0235 m, track 0.19 m):
#   COUNT_AVERAGE_PER_METER = COUNTS_PER_REV / (2*pi*0.0235)
#   COUNT_DIFF_PER_DEGREE   = COUNT_AVERAGE_PER_METER * 0.19 * (pi/180)
COUNT_AVERAGE_PER_METER: float = 3387.390395
# COUNT_AVERAGE_PER_METER: float = 3337.390395
COUNT_DIFF_PER_DEGREE: float = 10.5
ORIGIN_FORWARD_OFFSET_M: float = 0.04  # robot origin is 40mm ahead of the axle midpoint
ODOM_HISTORY_SEC: float = 3.0
LEFT_WHEEL_JOINT_NAME: str = 'left_wheel_joint'
RIGHT_WHEEL_JOINT_NAME: str = 'right_wheel_joint'
FUSION_DEBUG_HZ: float = 1.0  # how often to log wheel counts + camera/fused pose (Hz)
ROBOT_MOTION_STATUS_TOPIC: str = '/robot_motion_status'
# While the robot is stopped its true pose is constant, so camera frames can be
# accumulated and averaged (after outlier rejection) for a far less noisy anchor.
STOPPED_AVG_OUTLIER_FACTOR: float = 3.0  # MAD multiplier for outlier rejection
STOPPED_AVG_POS_FLOOR_M: float = 0.01  # don't reject samples within this position spread
STOPPED_AVG_YAW_FLOOR_DEG: float = 1.0  # don't reject samples within this yaw spread
RADAR_SENSOR_TOPIC: str = '/radar_sensor'
RADAR_DIRECTIONS: tuple[str, ...] = ('front', 'right', 'left', 'rear')
VL53_I2C_ADDR: int = 0x29  # default address for both VL53L0X and VL53L1X
MPU6050_I2C_ADDR: int = 0x68  # may share the bus with the front ToF
DEFAULT_USE_TCA9548A: bool = True
DEFAULT_USE_REAL_SENSOR: bool = True
DEFAULT_REMOTE_HOST: str = '192.168.50.2'
DEFAULT_REMOTE_PORT: int = 5003
DEFAULT_RADAR_POLL_HZ: float = 10.0
DEFAULT_REQUEST_TIMEOUT: float = 3.0
DEFAULT_TIME_TOPIC: str = '/time'
DEFAULT_TCA9548A_ADDR: int = 0x70
DEFAULT_TOF_FRONT_CHANNEL: int = 1
DEFAULT_TOF_MIN_RANGE_M: float = 0.02
DEFAULT_TOF_MAX_RANGE_M: float = 0.80
DEFAULT_TOF_READ_FAIL_VALUE_M: float = 0.80
DEFAULT_TOF_DEBUG_ENABLED: bool = False
DEFAULT_TOF_DEBUG_INTERVAL_SEC: float = 0.1
DEFAULT_RADAR_SENSOR_DEBUG_ENABLED: bool = True
DEFAULT_RADAR_SENSOR_DEBUG_INTERVAL_SEC: float = 0.1
DEFAULT_TOF_INIT_RETRIES: int = 5
DEFAULT_TOF_STARTUP_DELAY_SEC: float = 1.0
DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT: int = 8
DEFAULT_TOF_READ_RETRIES: int = 3
DEFAULT_I2C_FREQUENCY: int = 100_000
VL53L0X_SOFT_RESET_REG: int = 0xBF
VL53L0X_INVALID_RANGE_MM: int = 8190  # 8190/8191 = timeout / no valid sample
DEFAULT_TOF_WARMUP_SEC: float = 0.5
DEFAULT_TOF_WARMUP_ATTEMPTS: int = 5
DEFAULT_TOF_RECOVERY_INTERVAL_SEC: float = 2.0


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _normalize_yaw_rad(yaw: float) -> float:
    """Wrap yaw (rad) to (-pi, pi], i.e. ±180 deg."""
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _median(values: list[float]) -> float:
    s = sorted(values)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def _circular_mean_rad(yaws: list[float]) -> float:
    sin_sum = sum(math.sin(y) for y in yaws)
    cos_sum = sum(math.cos(y) for y in yaws)
    return math.atan2(sin_sum, cos_sum)


def _robust_average_pose(
    samples: list[tuple[float, float, float]],
    pos_floor_m: float,
    yaw_floor_rad: float,
    outlier_factor: float,
) -> tuple[float, float, float, int]:
    """Outlier-rejected average of (x, y, yaw) camera poses.

    Returns (mean_x, mean_y, mean_yaw, kept_count). Outliers are detected with a
    robust MAD-style test on both position (euclidean distance from the median
    point) and yaw (angular distance from the circular mean); a sample must pass
    both tests to be kept.
    """
    n = len(samples)
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    yaws = [s[2] for s in samples]
    if n < 3:
        return sum(xs) / n, sum(ys) / n, _circular_mean_rad(yaws), n

    median_x = _median(xs)
    median_y = _median(ys)
    pos_dists = [math.hypot(x - median_x, y - median_y) for x, y in zip(xs, ys)]
    pos_thresh = max(_median(pos_dists) * outlier_factor, pos_floor_m)

    yaw_center = _circular_mean_rad(yaws)
    yaw_dists = [abs(_normalize_yaw_rad(y - yaw_center)) for y in yaws]
    yaw_thresh = max(_median(yaw_dists) * outlier_factor, yaw_floor_rad)

    kept = [
        s
        for s, pd, yd in zip(samples, pos_dists, yaw_dists)
        if pd <= pos_thresh and yd <= yaw_thresh
    ]
    if not kept:
        kept = samples

    k = len(kept)
    kx = sum(s[0] for s in kept) / k
    ky = sum(s[1] for s in kept) / k
    kyaw = _circular_mean_rad([s[2] for s in kept])
    return kx, ky, kyaw, k


class PoseEstimationSensorFusionNode(Node):
    """Fuses absolute camera pose with wheel odometry into /current_position."""

    def __init__(self) -> None:
        super().__init__('pose_estimation_sensor_fusion')

        self.declare_parameter('camera_pose_topic', '/current_position_camera')
        self.declare_parameter('wheel_state_topic', '/wheel_joint_states')
        self.declare_parameter('output_topic', '/current_position')
        self.declare_parameter('counts_per_rev', COUNTS_PER_REV)
        self.declare_parameter('count_average_per_meter', COUNT_AVERAGE_PER_METER)
        self.declare_parameter('count_diff_per_degree', COUNT_DIFF_PER_DEGREE)
        self.declare_parameter('origin_forward_offset_m', ORIGIN_FORWARD_OFFSET_M)
        self.declare_parameter('left_wheel_joint', LEFT_WHEEL_JOINT_NAME)
        self.declare_parameter('right_wheel_joint', RIGHT_WHEEL_JOINT_NAME)
        self.declare_parameter('odom_history_sec', ODOM_HISTORY_SEC)
        self.declare_parameter('fusion_debug_hz', FUSION_DEBUG_HZ)
        self.declare_parameter('robot_motion_status_topic', ROBOT_MOTION_STATUS_TOPIC)
        self.declare_parameter('stopped_avg_outlier_factor', STOPPED_AVG_OUTLIER_FACTOR)
        self.declare_parameter('stopped_avg_pos_floor_m', STOPPED_AVG_POS_FLOOR_M)
        self.declare_parameter('stopped_avg_yaw_floor_deg', STOPPED_AVG_YAW_FLOOR_DEG)
        self.declare_parameter('use_real_sensor', DEFAULT_USE_REAL_SENSOR)
        self.declare_parameter('remote_host', DEFAULT_REMOTE_HOST)
        self.declare_parameter('remote_port', DEFAULT_REMOTE_PORT)
        self.declare_parameter('poll_hz', DEFAULT_RADAR_POLL_HZ)
        self.declare_parameter('request_timeout', DEFAULT_REQUEST_TIMEOUT)
        self.declare_parameter('time_topic', DEFAULT_TIME_TOPIC)
        self.declare_parameter('radar_sensor_topic', RADAR_SENSOR_TOPIC)
        self.declare_parameter('use_tca9548a', DEFAULT_USE_TCA9548A)
        self.declare_parameter('tca9548a_addr', DEFAULT_TCA9548A_ADDR)
        self.declare_parameter('tof_front_channel', DEFAULT_TOF_FRONT_CHANNEL)
        self.declare_parameter('tof_min_range_m', DEFAULT_TOF_MIN_RANGE_M)
        self.declare_parameter('tof_max_range_m', DEFAULT_TOF_MAX_RANGE_M)
        self.declare_parameter('tof_read_fail_value_m', DEFAULT_TOF_READ_FAIL_VALUE_M)
        self.declare_parameter('tof_debug_enabled', DEFAULT_TOF_DEBUG_ENABLED)
        self.declare_parameter('tof_debug_interval_sec', DEFAULT_TOF_DEBUG_INTERVAL_SEC)
        self.declare_parameter('radar_sensor_debug_enabled', DEFAULT_RADAR_SENSOR_DEBUG_ENABLED)
        self.declare_parameter(
            'radar_sensor_debug_interval_sec', DEFAULT_RADAR_SENSOR_DEBUG_INTERVAL_SEC
        )
        self.declare_parameter('tof_init_retries', DEFAULT_TOF_INIT_RETRIES)
        self.declare_parameter('tof_startup_delay_sec', DEFAULT_TOF_STARTUP_DELAY_SEC)
        self.declare_parameter('tof_reinit_after_fail_count', DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT)
        self.declare_parameter('tof_read_retries', DEFAULT_TOF_READ_RETRIES)
        self.declare_parameter('i2c_frequency', DEFAULT_I2C_FREQUENCY)

        self._camera_topic = (
            self.get_parameter('camera_pose_topic').get_parameter_value().string_value
            or '/current_position_camera'
        )
        self._wheel_topic = (
            self.get_parameter('wheel_state_topic').get_parameter_value().string_value
            or '/wheel_joint_states'
        )
        self._output_topic = (
            self.get_parameter('output_topic').get_parameter_value().string_value
            or '/current_position'
        )
        self._counts_per_rev = (
            float(self.get_parameter('counts_per_rev').get_parameter_value().double_value)
            or COUNTS_PER_REV
        )
        self._count_avg_per_m = (
            float(self.get_parameter('count_average_per_meter').get_parameter_value().double_value)
            or COUNT_AVERAGE_PER_METER
        )
        self._count_diff_per_deg = (
            float(self.get_parameter('count_diff_per_degree').get_parameter_value().double_value)
            or COUNT_DIFF_PER_DEGREE
        )
        self._origin_offset = float(
            self.get_parameter('origin_forward_offset_m').get_parameter_value().double_value
        )
        self._left_joint = (
            self.get_parameter('left_wheel_joint').get_parameter_value().string_value
            or LEFT_WHEEL_JOINT_NAME
        )
        self._right_joint = (
            self.get_parameter('right_wheel_joint').get_parameter_value().string_value
            or RIGHT_WHEEL_JOINT_NAME
        )
        history_sec = (
            float(self.get_parameter('odom_history_sec').get_parameter_value().double_value)
            or ODOM_HISTORY_SEC
        )
        self._history_window_ns = int(history_sec * 1e9)
        fusion_debug_hz = (
            float(self.get_parameter('fusion_debug_hz').get_parameter_value().double_value)
            or FUSION_DEBUG_HZ
        )
        self._motion_status_topic = (
            self.get_parameter('robot_motion_status_topic').get_parameter_value().string_value
            or ROBOT_MOTION_STATUS_TOPIC
        )
        self._stopped_avg_outlier_factor = (
            float(self.get_parameter('stopped_avg_outlier_factor').get_parameter_value().double_value)
            or STOPPED_AVG_OUTLIER_FACTOR
        )
        self._stopped_avg_pos_floor_m = (
            float(self.get_parameter('stopped_avg_pos_floor_m').get_parameter_value().double_value)
            or STOPPED_AVG_POS_FLOOR_M
        )
        self._stopped_avg_yaw_floor_rad = math.radians(
            float(self.get_parameter('stopped_avg_yaw_floor_deg').get_parameter_value().double_value)
            or STOPPED_AVG_YAW_FLOOR_DEG
        )

        self._use_real_sensor = self.get_parameter('use_real_sensor').get_parameter_value().bool_value
        self._remote_host = (
            self.get_parameter('remote_host').get_parameter_value().string_value or DEFAULT_REMOTE_HOST
        )
        self._remote_port = int(
            self.get_parameter('remote_port').get_parameter_value().integer_value or DEFAULT_REMOTE_PORT
        )
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value) or DEFAULT_RADAR_POLL_HZ
        self._request_timeout = float(
            self.get_parameter('request_timeout').get_parameter_value().double_value or DEFAULT_REQUEST_TIMEOUT
        )
        self._time_topic = (
            self.get_parameter('time_topic').get_parameter_value().string_value or DEFAULT_TIME_TOPIC
        )
        self._radar_topic = (
            self.get_parameter('radar_sensor_topic').get_parameter_value().string_value or RADAR_SENSOR_TOPIC
        )
        self._tof_front_channel = int(
            self.get_parameter('tof_front_channel').get_parameter_value().integer_value
        )
        if self._tof_front_channel < 0 or self._tof_front_channel > 7:
            self._tof_front_channel = DEFAULT_TOF_FRONT_CHANNEL
        self._tof_min_range_m = float(
            self.get_parameter('tof_min_range_m').get_parameter_value().double_value or DEFAULT_TOF_MIN_RANGE_M
        )
        self._tof_max_range_m = float(
            self.get_parameter('tof_max_range_m').get_parameter_value().double_value or DEFAULT_TOF_MAX_RANGE_M
        )
        self._tof_read_fail_value_m = float(
            self.get_parameter('tof_read_fail_value_m').get_parameter_value().double_value
            or DEFAULT_TOF_READ_FAIL_VALUE_M
        )
        self._tof_debug_enabled = self.get_parameter('tof_debug_enabled').get_parameter_value().bool_value
        self._radar_sensor_debug_enabled = (
            self.get_parameter('radar_sensor_debug_enabled').get_parameter_value().bool_value
        )
        self._radar_sensor_debug_interval_sec = float(
            self.get_parameter('radar_sensor_debug_interval_sec').get_parameter_value().double_value
            or DEFAULT_RADAR_SENSOR_DEBUG_INTERVAL_SEC
        )
        self._tof_init_retries = int(
            self.get_parameter('tof_init_retries').get_parameter_value().integer_value or DEFAULT_TOF_INIT_RETRIES
        )
        self._tof_startup_delay_sec = float(
            self.get_parameter('tof_startup_delay_sec').get_parameter_value().double_value
            or DEFAULT_TOF_STARTUP_DELAY_SEC
        )
        self._tof_reinit_after_fail_count = int(
            self.get_parameter('tof_reinit_after_fail_count').get_parameter_value().integer_value
            or DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT
        )
        self._tof_read_retries = int(
            self.get_parameter('tof_read_retries').get_parameter_value().integer_value
            or DEFAULT_TOF_READ_RETRIES
        )
        self._i2c_frequency = int(
            self.get_parameter('i2c_frequency').get_parameter_value().integer_value or DEFAULT_I2C_FREQUENCY
        )
        self._use_tca9548a = self.get_parameter('use_tca9548a').get_parameter_value().bool_value
        self._mux_addr = int(self.get_parameter('tca9548a_addr').get_parameter_value().integer_value)
        if self._request_timeout <= 0.0:
            self._request_timeout = DEFAULT_REQUEST_TIMEOUT
        if self._radar_sensor_debug_interval_sec <= 0.0:
            self._radar_sensor_debug_interval_sec = DEFAULT_RADAR_SENSOR_DEBUG_INTERVAL_SEC
        if self._tof_init_retries <= 0:
            self._tof_init_retries = 1
        if self._tof_startup_delay_sec < 0.0:
            self._tof_startup_delay_sec = DEFAULT_TOF_STARTUP_DELAY_SEC
        if self._tof_reinit_after_fail_count <= 0:
            self._tof_reinit_after_fail_count = DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT
        if self._tof_read_retries <= 0:
            self._tof_read_retries = 1
        if self._i2c_frequency <= 0:
            self._i2c_frequency = DEFAULT_I2C_FREQUENCY

        self._latest_time_s: float | None = None
        self._last_radar_text: str | None = None
        self._radar_fetch_error_logged = False
        self._tof_error_logged = False
        self._tof_backend = 'none'
        self._tof_readers: dict[str, object] = {}
        self._tof_sensors: dict[str, tuple[object, str]] = {}
        self._tof_consecutive_fail_count = 0
        self._last_tof_recovery_attempt = 0.0
        self._last_radar_sensor_debug = self.get_clock().now()
        self._i2c = None
        self._i2c_lock = threading.Lock()
        self._tof_mux = None
        self._radar_urls = [
            f'http://{self._remote_host}:{self._remote_port}/data/simulation_data',
            f'http://{self._remote_host}:{self._remote_port}/simulation_data',
        ]
        self._radar_url_index = 0

        # Free-running wheel-axle-midpoint odometry pose (drifts; corrected by camera).
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0  # kept unwrapped/continuous for exact deltas
        self._prev_left_rad: float | None = None
        self._prev_right_rad: float | None = None
        self._left_rad: float | None = None
        self._right_rad: float | None = None
        self._camera_x: float | None = None
        self._camera_y: float | None = None
        self._camera_yaw_deg: float | None = None
        self._fused_yaw_deg: float | None = None
        self._robot_motion_status: str | None = None
        # Camera (robot-origin) poses collected while the robot is stopped; averaged
        # (with outlier rejection) into the anchor. Cleared when the robot moves
        # because the true pose then changes.
        self._stopped_camera_samples: list[tuple[float, float, float]] = []
        self._stopped_sample_count: int = 0
        self._stopped_kept_count: int = 0
        # History of (stamp_ns, x, y, yaw) for interpolating odom at camera time.
        self._odom_history: deque[tuple[int, float, float, float]] = deque()

        # Camera correction anchor, expressed at the wheel-axle midpoint.
        self._anchor_valid = False
        self._anchor_x = 0.0
        self._anchor_y = 0.0
        self._anchor_yaw = 0.0
        self._anchor_odom_x = 0.0
        self._anchor_odom_y = 0.0
        self._anchor_odom_yaw = 0.0

        self._pub = self.create_publisher(PoseStamped, self._output_topic, 10)
        self._pub_radar_sensor = self.create_publisher(String, self._radar_topic, 10)
        self.create_subscription(JointState, self._wheel_topic, self._on_wheel_state, 50)
        self.create_subscription(PoseStamped, self._camera_topic, self._on_camera_pose, 10)
        self.create_subscription(
            String, self._motion_status_topic, self._on_robot_motion_status, 10
        )
        if self._time_topic:
            self.create_subscription(String, self._time_topic, self._on_time, 10)
        self._radar_poll_stop = threading.Event()
        self._radar_poll_thread: threading.Thread | None = None
        if poll_hz > 0.0:
            self._radar_poll_period_s = 1.0 / poll_hz
            self._radar_poll_thread = threading.Thread(
                target=self._radar_poll_loop,
                name='radar_poll',
                daemon=True,
            )
            self._radar_poll_thread.start()
        else:
            self._radar_poll_period_s = 0.0
        if fusion_debug_hz > 0.0:
            self.create_timer(1.0 / fusion_debug_hz, self._log_fusion_debug)

        self._tof_init_thread: threading.Thread | None = None
        if self._use_real_sensor:
            self._tof_init_thread = threading.Thread(
                target=self._deferred_init_tof_readers,
                name='tof_init',
                daemon=True,
            )
            self._tof_init_thread.start()

        self.get_logger().info(
            'pose_estimation_sensor_fusion started; fusing '
            f'{self._camera_topic} (anchor) + {self._wheel_topic} (odom) -> {self._output_topic}; '
            f'counts_per_rev={self._counts_per_rev:.1f}, '
            f'count_average_per_meter={self._count_avg_per_m:.1f}, '
            f'count_diff_per_degree={self._count_diff_per_deg:.3f}, '
            f'origin_forward_offset={self._origin_offset:.3f}m, '
            f'debug_when={self._motion_status_topic}=stopped, '
            f'radar={"real-tof" if self._use_real_sensor else "upstream"} '
            f'backend={self._tof_backend} -> {self._radar_topic}, poll_hz={poll_hz}'
        )

    def _deferred_init_tof_readers(self) -> None:
        """Initialize ToF off the critical path so pose fusion can spin immediately."""
        if self._tof_startup_delay_sec > 0.0:
            time.sleep(self._tof_startup_delay_sec)
        for attempt in range(self._tof_init_retries):
            if attempt > 0:
                time.sleep(DEFAULT_TOF_RECOVERY_INTERVAL_SEC)
            try:
                with self._i2c_lock:
                    self._stop_tof_sensor('front')
                    self._tof_mux = None
                    readers = self._build_tof_readers()
                if readers:
                    self._tof_readers = readers
                    self._tof_backend = self._tof_backend if self._tof_readers else 'none'
                    self.get_logger().info(
                        f'front ToF ready after deferred init attempt {attempt + 1}/'
                        f'{self._tof_init_retries}'
                    )
                    return
            except Exception as exc:
                self.get_logger().error(
                    f'deferred ToF init attempt {attempt + 1}/{self._tof_init_retries} failed: {exc}'
                )
        self.get_logger().error(
            'deferred ToF init exhausted retries; recovery loop will keep retrying'
        )

    def _on_robot_motion_status(self, msg: String) -> None:
        status = (msg.data or '').strip().lower()
        # The robot moved, so the pose averaged while stopped is no longer valid.
        if status == 'moving' and self._stopped_camera_samples:
            self._stopped_camera_samples.clear()
            self._stopped_sample_count = 0
            self._stopped_kept_count = 0
        self._robot_motion_status = status

    # ----- wheel odometry -----------------------------------------------------

    def _on_wheel_state(self, msg: JointState) -> None:
        left_rad = right_rad = None
        for name, position in zip(msg.name, msg.position):
            if name == self._left_joint:
                left_rad = float(position)
            elif name == self._right_joint:
                right_rad = float(position)
        if left_rad is None or right_rad is None:
            return

        self._left_rad = left_rad
        self._right_rad = right_rad
        stamp_ns = self._stamp_to_ns(msg)

        if self._prev_left_rad is None or self._prev_right_rad is None:
            self._prev_left_rad = left_rad
            self._prev_right_rad = right_rad
            self._record_odom(stamp_ns)
            return

        # Incoming wheel angles are in rad; convert back to encoder counts so the
        # calibration constants are expressed directly in counts.
        rad_to_counts = self._counts_per_rev / (2.0 * math.pi)
        d_left_count = (left_rad - self._prev_left_rad) * rad_to_counts
        d_right_count = (right_rad - self._prev_right_rad) * rad_to_counts
        self._prev_left_rad = left_rad
        self._prev_right_rad = right_rad

        d_center = 0.5 * (d_left_count + d_right_count) / self._count_avg_per_m
        d_yaw_deg = (d_right_count - d_left_count) / self._count_diff_per_deg
        d_yaw = math.radians(d_yaw_deg)

        # Integrate at the heading midpoint (2nd-order exact for constant-curvature arc).
        yaw_mid = self._odom_yaw + 0.5 * d_yaw
        self._odom_x += d_center * math.cos(yaw_mid)
        self._odom_y += d_center * math.sin(yaw_mid)
        self._odom_yaw += d_yaw

        self._record_odom(stamp_ns)
        self._publish_fused(stamp_ns)

    def _record_odom(self, stamp_ns: int) -> None:
        self._odom_history.append((stamp_ns, self._odom_x, self._odom_y, self._odom_yaw))
        while (
            len(self._odom_history) > 1
            and (stamp_ns - self._odom_history[0][0]) > self._history_window_ns
        ):
            self._odom_history.popleft()

    def _odom_pose_at(self, stamp_ns: int) -> tuple[float, float, float]:
        """Interpolate the recorded odom pose at the given timestamp."""
        hist = self._odom_history
        if not hist:
            return self._odom_x, self._odom_y, self._odom_yaw
        if stamp_ns <= hist[0][0]:
            return hist[0][1], hist[0][2], hist[0][3]
        if stamp_ns >= hist[-1][0]:
            return hist[-1][1], hist[-1][2], hist[-1][3]

        for i in range(len(hist) - 1):
            t0, x0, y0, yaw0 = hist[i]
            t1, x1, y1, yaw1 = hist[i + 1]
            if t0 <= stamp_ns <= t1:
                span = t1 - t0
                a = 0.0 if span <= 0 else (stamp_ns - t0) / span
                return (
                    x0 + a * (x1 - x0),
                    y0 + a * (y1 - y0),
                    yaw0 + a * (yaw1 - yaw0),
                )
        return hist[-1][1], hist[-1][2], hist[-1][3]

    # ----- camera correction --------------------------------------------------

    def _on_camera_pose(self, msg: PoseStamped) -> None:
        ox = msg.pose.position.x
        oy = msg.pose.position.y
        yaw = _yaw_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if not all(math.isfinite(v) for v in (ox, oy, yaw)):
            return

        # While stopped the true pose is constant: accumulate camera frames and use
        # their outlier-rejected average as the anchor instead of a single noisy fix.
        if self._robot_motion_status == 'stopped':
            self._stopped_camera_samples.append((ox, oy, yaw))
            ox, oy, yaw, kept = _robust_average_pose(
                self._stopped_camera_samples,
                self._stopped_avg_pos_floor_m,
                self._stopped_avg_yaw_floor_rad,
                self._stopped_avg_outlier_factor,
            )
            self._stopped_sample_count = len(self._stopped_camera_samples)
            self._stopped_kept_count = kept

        # Camera gives the robot-origin pose; convert to the wheel-axle midpoint,
        # which is the point odometry integrates.
        axle_x = ox - self._origin_offset * math.cos(yaw)
        axle_y = oy - self._origin_offset * math.sin(yaw)

        stamp_ns = self._stamp_to_ns(msg)
        odom_x, odom_y, odom_yaw = self._odom_pose_at(stamp_ns)

        self._anchor_x = axle_x
        self._anchor_y = axle_y
        self._anchor_yaw = yaw
        self._anchor_odom_x = odom_x
        self._anchor_odom_y = odom_y
        self._anchor_odom_yaw = odom_yaw
        self._anchor_valid = True
        self._camera_x = ox
        self._camera_y = oy
        self._camera_yaw_deg = math.degrees(yaw)

        self._publish_fused(self._latest_stamp_ns())

    # ----- fusion + publish ---------------------------------------------------

    def _publish_fused(self, stamp_ns: int) -> None:
        if not self._anchor_valid:
            # Wheel odometry only until camera establishes an anchor.
            axle_yaw = self._odom_yaw
            axle_x = self._odom_x
            axle_y = self._odom_y
            origin_x = axle_x + self._origin_offset * math.cos(axle_yaw)
            origin_y = axle_y + self._origin_offset * math.sin(axle_yaw)
            self._fused_yaw_deg = math.degrees(axle_yaw)
            qx, qy, qz, qw = _quaternion_from_yaw(axle_yaw)
            out = PoseStamped()
            out.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
            out.header.frame_id = 'map'
            out.pose.position.x = origin_x
            out.pose.position.y = origin_y
            out.pose.position.z = 0.0
            out.pose.orientation.x = qx
            out.pose.orientation.y = qy
            out.pose.orientation.z = qz
            out.pose.orientation.w = qw
            self._pub.publish(out)
            return

        # Relative motion in odom frame since the camera anchor's timestamp.
        dx = self._odom_x - self._anchor_odom_x
        dy = self._odom_y - self._anchor_odom_y
        d_yaw = self._odom_yaw - self._anchor_odom_yaw

        # Rotate the odom-frame delta into the anchor (map) frame and compose.
        rot = self._anchor_yaw - self._anchor_odom_yaw
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)
        axle_x = self._anchor_x + cos_r * dx - sin_r * dy
        axle_y = self._anchor_y + sin_r * dx + cos_r * dy
        axle_yaw = _normalize_yaw_rad(self._anchor_yaw + d_yaw)

        # Convert the axle-midpoint pose back to the robot origin (40mm forward).
        origin_x = axle_x + self._origin_offset * math.cos(axle_yaw)
        origin_y = axle_y + self._origin_offset * math.sin(axle_yaw)
        self._fused_yaw_deg = math.degrees(axle_yaw)

        qx, qy, qz, qw = _quaternion_from_yaw(axle_yaw)
        out = PoseStamped()
        out.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
        out.header.frame_id = 'map'
        out.pose.position.x = origin_x
        out.pose.position.y = origin_y
        out.pose.position.z = 0.0
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self._pub.publish(out)

    # ----- radar / ToF --------------------------------------------------------

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

    def _radar_poll_loop(self) -> None:
        """Poll ToF/upstream radar off the ROS executor so I2C cannot block odometry."""
        while not self._radar_poll_stop.is_set():
            try:
                self._poll_radar()
            except Exception as exc:
                self.get_logger().warn(f'radar poll failed: {exc}')
            if self._radar_poll_stop.wait(self._radar_poll_period_s):
                break

    def stop_radar_poll_thread(self) -> None:
        self._radar_poll_stop.set()
        if self._radar_poll_thread is not None:
            self._radar_poll_thread.join(timeout=2.0)

    def _poll_radar(self) -> None:
        tof_debug_values: dict[str, tuple[float | None, str, float]] | None = None
        if self._use_real_sensor:
            radar_text, tof_debug_values = self._build_radar_text_from_tof()
            if not radar_text:
                return
        else:
            try:
                body = self._fetch_simulation_data_body()
                payload = json.loads(body.decode('utf-8', errors='ignore'))
                if not isinstance(payload, dict):
                    raise ValueError('simulation_data payload must be a JSON object')
            except Exception as exc:
                if not self._radar_fetch_error_logged:
                    self.get_logger().warn(
                        f'upstream fetch failed for radar from {self._radar_urls}: {exc}'
                    )
                    self._radar_fetch_error_logged = True
                return

            self._radar_fetch_error_logged = False
            radar_text = str(payload.get('radar_sensor', '')).strip()
            if not radar_text:
                return
            radar_text = self._replace_radar_timestamp(radar_text)

        self._maybe_log_radar_sensor_debug(radar_text, tof_debug_values)

        if radar_text == self._last_radar_text:
            return

        msg = String()
        msg.data = radar_text
        self._pub_radar_sensor.publish(msg)
        self._last_radar_text = radar_text

    def _replace_radar_timestamp(self, radar_text: str) -> str:
        parts = [part.strip() for part in radar_text.split(',')]
        if len(parts) < 5:
            return radar_text
        parts[0] = f'{self._resolve_time_s():.6f}'
        return ','.join(parts[:5])

    def _get_i2c(self):
        if self._i2c is not None:
            return self._i2c
        import board
        import busio

        self._i2c = busio.I2C(board.SCL, board.SDA, frequency=self._i2c_frequency)
        return self._i2c

    def _scan_main_i2c_addresses(self, i2c) -> list[int]:
        try:
            return list(i2c.scan())
        except Exception as exc:
            self.get_logger().error(f'I2C scan on main bus failed: {exc}')
            return []

    def _log_main_i2c_devices(self, addresses: list[int]) -> None:
        labels: list[str] = []
        for addr in sorted(addresses):
            if addr == VL53_I2C_ADDR:
                labels.append(f'0x{addr:02x}(VL53 ToF)')
            elif addr == MPU6050_I2C_ADDR:
                labels.append(f'0x{addr:02x}(MPU6050)')
            elif addr == DEFAULT_TCA9548A_ADDR:
                labels.append(f'0x{addr:02x}(TCA9548A MUX)')
            else:
                labels.append(f'0x{addr:02x}')
        self.get_logger().info(
            'main I2C scan: ' + (', '.join(labels) if labels else 'no devices detected')
        )

    def _soft_reset_vl53l0x(self, bus) -> None:
        """Hardware soft-reset the VL53L0X before init (needed after mux power-up)."""
        try:
            while not bus.try_lock():
                pass
            try:
                bus.writeto(VL53_I2C_ADDR, bytes([VL53L0X_SOFT_RESET_REG, 0x00]))
            finally:
                bus.unlock()
            time.sleep(0.05)
            while not bus.try_lock():
                pass
            try:
                bus.writeto(VL53_I2C_ADDR, bytes([VL53L0X_SOFT_RESET_REG, 0x01]))
            finally:
                bus.unlock()
            time.sleep(0.05)
        except Exception as exc:
            self.get_logger().debug(f'VL53L0X soft reset failed: {exc}')

    def _stop_tof_sensor(self, name: str) -> None:
        stored = self._tof_sensors.pop(name, None)
        if stored is None:
            return
        sensor, model = stored
        try:
            if model == 'VL53L0X':
                sensor.stop_continuous()
            elif model == 'VL53L1X':
                sensor.stop_ranging()
        except Exception:
            pass

    def _warmup_vl53l0x(self, sensor) -> int:
        """Return the first valid warmup sample (mm), or raise if all invalid."""
        time.sleep(DEFAULT_TOF_WARMUP_SEC)
        last_mm = VL53L0X_INVALID_RANGE_MM
        for _ in range(DEFAULT_TOF_WARMUP_ATTEMPTS):
            last_mm = int(sensor.range)
            if last_mm < VL53L0X_INVALID_RANGE_MM:
                return last_mm
            time.sleep(0.05)
        raise RuntimeError(f'VL53L0X warmup read invalid: {last_mm} mm')

    def _read_vl53l0x_mm(self, sensor) -> float:
        last_mm = VL53L0X_INVALID_RANGE_MM
        for _ in range(self._tof_read_retries):
            last_mm = int(sensor.range)
            if last_mm < VL53L0X_INVALID_RANGE_MM:
                return float(last_mm) / 1000.0
            time.sleep(0.02)
        raise RuntimeError(f'VL53L0X invalid range: {last_mm} mm')

    def _try_init_vl53l0x(self, bus):
        try:
            import adafruit_vl53l0x
        except ImportError:
            return None, None

        last_exc: Exception | None = None
        for _ in range(self._tof_init_retries):
            sensor = None
            try:
                self._soft_reset_vl53l0x(bus)
                time.sleep(0.1)
                sensor = adafruit_vl53l0x.VL53L0X(bus)
                sensor.start_continuous()
                self._warmup_vl53l0x(sensor)
                return sensor, 'VL53L0X'
            except Exception as exc:
                last_exc = exc
                if sensor is not None:
                    try:
                        sensor.stop_continuous()
                    except Exception:
                        pass
                time.sleep(0.05)
        if last_exc is not None:
            self.get_logger().debug(f'VL53L0X init failed: {last_exc}')
        return None, None

    def _try_init_vl53l1x(self, bus):
        try:
            import adafruit_vl53l1x
        except ImportError:
            return None, None

        last_exc: Exception | None = None
        for _ in range(self._tof_init_retries):
            try:
                sensor = adafruit_vl53l1x.VL53L1X(bus)
                sensor.start_ranging()
                return sensor, 'VL53L1X'
            except Exception as exc:
                last_exc = exc
                time.sleep(0.05)
        if last_exc is not None:
            self.get_logger().debug(f'VL53L1X init failed: {last_exc}')
        return None, None

    def _make_vl53_reader(self, sensor, model: str):
        if model == 'VL53L0X':
            return lambda: self._read_vl53l0x_mm(sensor)

        def _read_l1x() -> float:
            distance_cm = sensor.distance
            if distance_cm is None:
                raise RuntimeError('VL53L1X returned no distance')
            return float(distance_cm) / 100.0

        return _read_l1x

    def _init_vl53_reader(self, bus, name: str, channel: int | None):
        sensor, model = self._try_init_vl53l0x(bus)
        if sensor is None:
            sensor, model = self._try_init_vl53l1x(bus)

        if sensor is None or model is None:
            where = (
                f'TCA9548A channel {channel}'
                if channel is not None
                else f'main I2C bus (0x{VL53_I2C_ADDR:02x})'
            )
            detected = None
            try:
                detected = [hex(addr) for addr in bus.scan()]
            except Exception as scan_exc:
                detected = f'<scan failed: {scan_exc}>'
            self.get_logger().warn(
                f'failed to init ToF {name} on {where} as VL53L0X or VL53L1X; '
                f'i2c scan = {detected}'
            )
            return None, None

        self._tof_sensors[name] = (sensor, model)
        return self._make_vl53_reader(sensor, model), model

    def _init_front_tof_on_main_bus(self, i2c, addresses: list[int]) -> dict[str, object]:
        readers: dict[str, object] = {}
        if VL53_I2C_ADDR not in addresses:
            return readers

        reader, model = self._init_vl53_reader(i2c, 'front', None)
        if reader is not None and model is not None:
            readers['front'] = reader
            self._tof_backend = model
            self.get_logger().info(f'front {model} ready on main I2C bus at 0x{VL53_I2C_ADDR:02x}')
        return readers

    def _init_front_tof_via_mux(self, i2c) -> dict[str, object]:
        readers: dict[str, object] = {}
        try:
            import adafruit_tca9548a
        except ImportError:
            self.get_logger().warn('use_tca9548a=true but adafruit_tca9548a is not installed')
            return readers

        try:
            tca = adafruit_tca9548a.TCA9548A(i2c, address=self._mux_addr)
            self._tof_mux = tca
        except Exception as exc:
            self.get_logger().warn(f'TCA9548A at 0x{self._mux_addr:02x} unavailable: {exc}')
            self._tof_mux = None
            return readers

        front_channel = self._tof_front_channel
        mux_bus = tca[front_channel]
        try:
            channel_addrs = [hex(addr) for addr in mux_bus.scan()]
        except Exception as scan_exc:
            self.get_logger().warn(
                f'I2C scan on TCA9548A channel {front_channel} failed: {scan_exc}'
            )
            return readers
        vl53_hex = f'0x{VL53_I2C_ADDR:02x}'
        if vl53_hex not in channel_addrs:
            self.get_logger().warn(
                f'VL53 ToF ({vl53_hex}) not seen on TCA9548A channel {front_channel} scan '
                f'({channel_addrs}); attempting driver init anyway (scan can miss booting VL53)'
            )
        time.sleep(0.1)
        reader, model = self._init_vl53_reader(mux_bus, 'front', front_channel)
        if reader is not None and model is not None:
            readers['front'] = reader
            self._tof_backend = model
            self.get_logger().info(
                f'front {model} ready on TCA9548A channel {front_channel} '
                f'(mux locked to channel {front_channel:02d})'
            )
        return readers

    def _build_tof_readers(self) -> dict[str, object]:
        try:
            import adafruit_vl53l0x  # noqa: F401
        except ImportError:
            adafruit_vl53l0x = None

        try:
            import adafruit_vl53l1x  # noqa: F401
        except ImportError:
            adafruit_vl53l1x = None

        if adafruit_vl53l0x is None and adafruit_vl53l1x is None:
            self.get_logger().error(
                'use_real_sensor=true but no ToF driver available '
                '(install adafruit-circuitpython-vl53l0x and/or adafruit-circuitpython-vl53l1x)'
            )
            return {}

        try:
            i2c = self._get_i2c()
        except Exception as exc:
            self.get_logger().error(f'failed to open I2C bus: {exc}')
            return {}

        addresses = self._scan_main_i2c_addresses(i2c)
        self._log_main_i2c_devices(addresses)

        if self._use_tca9548a:
            readers = self._init_front_tof_via_mux(i2c)
        else:
            readers = self._init_front_tof_on_main_bus(i2c, addresses)

        if not readers:
            where = (
                f'TCA9548A channel {self._tof_front_channel:02d}'
                if self._use_tca9548a
                else f'main I2C bus (0x{VL53_I2C_ADDR:02x})'
            )
            self.get_logger().error(f'no ToF sensors initialized; expected VL53 at 0x29 on {where}')
        return readers

    def _reinit_front_tof_reader(self) -> dict[str, object]:
        """Re-init front ToF on the already-selected mux channel (no channel scan)."""
        self._stop_tof_sensor('front')
        if self._use_tca9548a and self._tof_mux is not None:
            front_channel = self._tof_front_channel
            reader, model = self._init_vl53_reader(
                self._tof_mux[front_channel], 'front', front_channel
            )
            if reader is not None and model is not None:
                self._tof_backend = model
                return {'front': reader}
            return {}

        try:
            i2c = self._get_i2c()
        except Exception:
            return {}
        addresses = self._scan_main_i2c_addresses(i2c)
        return self._init_front_tof_on_main_bus(i2c, addresses)

    def _maybe_recover_tof_readers(self) -> None:
        if self._tof_readers:
            return
        now = time.monotonic()
        if now - self._last_tof_recovery_attempt < DEFAULT_TOF_RECOVERY_INTERVAL_SEC:
            return
        self._last_tof_recovery_attempt = now
        recovered = self._reinit_front_tof_reader()
        if recovered:
            self._tof_readers = recovered
            self._tof_backend = self._tof_backend if self._tof_readers else 'none'
            self._tof_error_logged = False
            self.get_logger().info('front ToF recovered after reader loss')

    def _fail_radar_payload(
        self,
    ) -> tuple[str, dict[str, tuple[float | None, str, float]]]:
        debug_values: dict[str, tuple[float | None, str, float]] = {}
        values: dict[str, float] = {}
        for name in RADAR_DIRECTIONS:
            values[name] = self._tof_read_fail_value_m
            debug_values[name] = (None, 'MISSING', values[name])
        timestamp_s = self._resolve_time_s()
        radar_text = (
            f'{timestamp_s:.6f},'
            f'{values["front"]:.3f},{values["right"]:.3f},'
            f'{values["left"]:.3f},{values["rear"]:.3f}'
        )
        return radar_text, debug_values

    def _clamp_tof_distance(self, raw: float) -> tuple[float, str]:
        if raw < self._tof_min_range_m:
            return self._tof_min_range_m, 'CLAMP_MIN'
        if raw > self._tof_max_range_m:
            return self._tof_max_range_m, 'CLAMP_MAX'
        return raw, 'OK'

    def _build_radar_text_from_tof(
        self,
    ) -> tuple[str | None, dict[str, tuple[float | None, str, float]] | None]:
        if not self._tof_readers:
            if not self._tof_error_logged:
                self.get_logger().error(
                    'real-sensor mode enabled but no ToF readers available; '
                    'publishing fail values and retrying recovery'
                )
                self._tof_error_logged = True
            with self._i2c_lock:
                self._maybe_recover_tof_readers()
            if not self._tof_readers:
                return self._fail_radar_payload()

        values: dict[str, float] = {}
        debug_values: dict[str, tuple[float | None, str, float]] = {}
        with self._i2c_lock:
            for name in RADAR_DIRECTIONS:
                reader = self._tof_readers.get(name)
                if reader is None:
                    values[name] = self._tof_read_fail_value_m
                    debug_values[name] = (None, 'MISSING', values[name])
                    continue

                try:
                    raw = float(reader())
                    pub, status = self._clamp_tof_distance(raw)
                    values[name] = pub
                    debug_values[name] = (raw, status, pub)
                except Exception:
                    values[name] = self._tof_read_fail_value_m
                    debug_values[name] = (None, 'FAIL', values[name])

            front_status = debug_values['front'][1]
            if front_status in ('FAIL', 'MISSING'):
                self._tof_consecutive_fail_count += 1
            else:
                self._tof_consecutive_fail_count = 0

            if self._tof_consecutive_fail_count >= self._tof_reinit_after_fail_count:
                self.get_logger().warn(
                    'front ToF reads failed repeatedly; re-init on fixed mux channel '
                    f'{self._tof_front_channel:02d} (count={self._tof_consecutive_fail_count})'
                )
                recovered = self._reinit_front_tof_reader()
                if recovered:
                    self._tof_readers = recovered
                    self._tof_backend = self._tof_backend if self._tof_readers else 'none'
                else:
                    self.get_logger().warn(
                        'front ToF re-init failed; keeping fail-value publishes and '
                        'will retry recovery'
                    )
                    self._tof_readers = {}
                self._tof_consecutive_fail_count = 0

        timestamp_s = self._resolve_time_s()
        radar_text = (
            f'{timestamp_s:.6f},'
            f'{values["front"]:.3f},{values["right"]:.3f},'
            f'{values["left"]:.3f},{values["rear"]:.3f}'
        )
        return radar_text, debug_values

    def _format_radar_sensor_debug(
        self,
        radar_text: str,
        tof_debug_values: dict[str, tuple[float | None, str, float]] | None,
    ) -> str:
        parts = [part.strip() for part in radar_text.split(',')]
        if len(parts) >= 5:
            topic_summary = (
                f't={parts[0]} front={parts[1]}m right={parts[2]}m '
                f'left={parts[3]}m rear={parts[4]}m'
            )
        else:
            topic_summary = f'payload={radar_text!r}'

        lines = [f'[radar_sensor] {self._radar_topic} | {topic_summary}']
        if tof_debug_values and self._tof_debug_enabled:
            tof_summary = []
            for name in RADAR_DIRECTIONS:
                raw, status, pub = tof_debug_values[name]
                raw_mm = 'None' if raw is None else f'{int(raw * 1000.0)}'
                tof_summary.append(f'{name}:raw_mm={raw_mm},status={status},pub={pub:.3f}')
            lines.append('tof diag | ' + ' | '.join(tof_summary))
        return '\n'.join(lines)

    def _maybe_log_radar_sensor_debug(
        self,
        radar_text: str,
        tof_debug_values: dict[str, tuple[float | None, str, float]] | None = None,
    ) -> None:
        if not self._radar_sensor_debug_enabled:
            return
        now = self.get_clock().now()
        delta = (now - self._last_radar_sensor_debug).nanoseconds / 1_000_000_000.0
        if delta < self._radar_sensor_debug_interval_sec:
            return
        self._last_radar_sensor_debug = now
        self.get_logger().info(
            self._format_radar_sensor_debug(radar_text, tof_debug_values)
        )

    def _fetch_simulation_data_body(self) -> bytes:
        first_error: Exception | None = None
        total = len(self._radar_urls)
        for offset in range(total):
            idx = (self._radar_url_index + offset) % total
            url = self._radar_urls[idx]
            try:
                req = urllib.request.Request(url, headers={'Connection': 'close'})
                with urllib.request.urlopen(req, timeout=self._request_timeout) as res:
                    body = res.read()
                self._radar_url_index = idx
                return body
            except Exception as exc:
                if first_error is None:
                    first_error = exc
                continue

        if first_error is None:
            raise RuntimeError('no simulation_data urls configured')
        raise first_error

    # ----- helpers ------------------------------------------------------------

    def _rad_to_count(self, rad: float) -> int:
        return int(round(rad * self._counts_per_rev / (2.0 * math.pi)))

    def _log_fusion_debug(self) -> None:
        if self._robot_motion_status != 'stopped':
            return
        if self._left_rad is None or self._right_rad is None:
            return
        left_cnt = self._rad_to_count(self._left_rad)
        right_cnt = self._rad_to_count(self._right_rad)
        if self._camera_x is None or self._camera_y is None:
            camera_xy = 'N/A'
        else:
            camera_xy = f'({self._camera_x:.3f}, {self._camera_y:.3f})'
        cam_yaw = (
            'N/A'
            if self._camera_yaw_deg is None
            else f'{self._camera_yaw_deg:.1f}deg'
        )
        fused_yaw = (
            'N/A'
            if self._fused_yaw_deg is None
            else f'{self._fused_yaw_deg:.1f}deg'
        )
        self.get_logger().info(
            f'[fusion] left_count={left_cnt}, right_count={right_cnt}, '
            f'camera_xy={camera_xy}, camera_yaw={cam_yaw}, fused_yaw={fused_yaw}, '
            f'cam_samples={self._stopped_kept_count}/{self._stopped_sample_count}'
        )

    def _stamp_to_ns(self, msg) -> int:
        stamp = msg.header.stamp
        ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if ns <= 0:
            return self.get_clock().now().nanoseconds
        return ns

    def _latest_stamp_ns(self) -> int:
        if self._odom_history:
            return self._odom_history[-1][0]
        return self.get_clock().now().nanoseconds


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = PoseEstimationSensorFusionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_radar_poll_thread()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
