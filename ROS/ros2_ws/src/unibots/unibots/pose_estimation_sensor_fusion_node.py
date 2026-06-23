"""Sensor-fusion pose node: EKF fusion of camera + wheel encoders + MPU6050 + ToF.

Inputs:
  - /current_position_camera (geometry_msgs/PoseStamped):
      Absolute robot-origin pose in the 'map' frame estimated from AprilTags.
      Low rate (~few Hz) and latent, but drift-free. EKF measurement update.
  - /wheel_joint_states (sensor_msgs/JointState):
      High-rate (~50 Hz) left/right wheel angle (rad) and angular velocity
      (rad/s) from the encoders. EKF velocity measurement + short-term predict.
  - MPU6050 on I2C (default 0x68, real hardware only):
      200 Hz gyro + accelerometer for high-rate EKF prediction and yaw-rate
      measurement. Axis mapping is configurable (see imu_*_axis params).
  - /robot_motion_status (std_msgs/String):
      'moving' or 'stopped' from motion_control_node. Enables ZUPT and stopped
      camera averaging.
  - /time (std_msgs/String, optional):
      Simulation clock used to stamp the first field of /radar_sensor payloads.

Outputs:
  - /current_position (geometry_msgs/PoseStamped):
      EKF-fused robot-origin pose. State: [x, y, yaw, v, omega] in map frame.
      Published up to fusion_publish_hz. NaN pose when camera is stale.
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
import os
import threading
import time
import urllib.request

import numpy as np

# #region agent log
_DBG_LOG_PATH = os.environ.get(
    'UNIBOTS_DEBUG_LOG',
    os.path.expanduser('~/.unibots/debug-4da0ac.log'),
)
_DBG_SESSION = '4da0ac'
_DBG_RUN_ID = os.environ.get('UNIBOTS_DEBUG_RUN', 'post-fix-v3')


def _agent_dbg_fusion(hypothesis_id: str, location: str, message: str, data: dict) -> None:
    try:
        os.makedirs(os.path.dirname(_DBG_LOG_PATH), exist_ok=True)
        payload = {
            'sessionId': _DBG_SESSION,
            'runId': _DBG_RUN_ID,
            'hypothesisId': hypothesis_id,
            'location': location,
            'message': message,
            'data': data,
            'timestamp': int(time.time() * 1000),
        }
        with open(_DBG_LOG_PATH, 'a', encoding='utf-8') as _f:
            _f.write(json.dumps(payload, ensure_ascii=True) + '\n')
    except Exception:
        pass


# #endregion
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
COUNT_AVERAGE_PER_METER: float = 3337.390395
COUNT_DIFF_PER_DEGREE: float = 10.3038
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
DEFAULT_USE_TCA9548A: bool = False
DEFAULT_USE_REAL_SENSOR: bool = True
DEFAULT_REMOTE_HOST: str = '192.168.50.2'
DEFAULT_REMOTE_PORT: int = 5003
DEFAULT_RADAR_POLL_HZ: float = 10.0
DEFAULT_REQUEST_TIMEOUT: float = 3.0
DEFAULT_TIME_TOPIC: str = '/time'
DEFAULT_TCA9548A_ADDR: int = 0x70
DEFAULT_TOF_FRONT_CHANNEL: int = 4
DEFAULT_TOF_MIN_RANGE_M: float = 0.02
DEFAULT_TOF_MAX_RANGE_M: float = 0.80
DEFAULT_TOF_READ_FAIL_VALUE_M: float = 0.80
DEFAULT_TOF_DEBUG_ENABLED: bool = False
DEFAULT_TOF_DEBUG_INTERVAL_SEC: float = 0.1
DEFAULT_TOF_INIT_RETRIES: int = 5
DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT: int = 8
DEFAULT_I2C_FREQUENCY: int = 100_000
DEFAULT_MPU_ENABLED: bool = True
DEFAULT_MPU_ADDR: int = MPU6050_I2C_ADDR
DEFAULT_MPU_POLL_HZ: float = 200.0
DEFAULT_MPU_DEBUG_INTERVAL_SEC: float = 0.5
DEFAULT_MPU_REINIT_AFTER_FAIL_COUNT: int = 20
DEFAULT_IMU_FUSION_ENABLED: bool = True
DEFAULT_FUSION_PUBLISH_HZ: float = 100.0
DEFAULT_IMU_FORWARD_AXIS: int = 0
DEFAULT_IMU_LEFT_AXIS: int = 1
DEFAULT_IMU_YAW_RATE_AXIS: int = 2
DEFAULT_IMU_FORWARD_SIGN: float = -1.0
DEFAULT_IMU_LEFT_SIGN: float = -1.0
DEFAULT_IMU_YAW_RATE_SIGN: float = 1.0
DEFAULT_EKF_PROCESS_POS_STD: float = 0.02
DEFAULT_EKF_PROCESS_YAW_STD: float = 0.03
DEFAULT_EKF_PROCESS_VEL_STD: float = 0.15
DEFAULT_EKF_PROCESS_OMEGA_STD: float = 0.08
DEFAULT_EKF_CAMERA_POS_STD: float = 0.04
DEFAULT_EKF_CAMERA_YAW_STD: float = 0.06
DEFAULT_EKF_CAMERA_POS_STD_MOVING: float = 0.12
DEFAULT_EKF_CAMERA_YAW_STD_MOVING: float = 0.15
DEFAULT_EKF_WHEEL_VEL_STD: float = 0.05
DEFAULT_EKF_WHEEL_OMEGA_STD: float = 0.04
DEFAULT_EKF_GYRO_STD: float = 0.03
DEFAULT_EKF_ACCEL_STD: float = 0.8
DEFAULT_EKF_ZUPT_VEL_STD: float = 0.02
DEFAULT_EKF_ZUPT_OMEGA_STD: float = 0.02
DEFAULT_FUSION_RESET_POS_THRESHOLD: float = 0.5
DEFAULT_FUSION_ACCEL_DEADBAND: float = 0.08
DEFAULT_FUSION_MAX_SPEED: float = 2.0
DEFAULT_FUSION_MAX_OMEGA: float = 4.0
DEFAULT_WHEEL_GAP_RESET_SEC: float = 0.25
DEFAULT_WHEEL_CATCHUP_MAX_SEC: float = 2.0
DEFAULT_FUSION_CALIB_STATIONARY_OMEGA: float = 0.05
DEFAULT_FUSION_RESET_YAW_THRESHOLD: float = 0.8
EKF_PREDICT_DT_CAP: float = 0.25
DEFAULT_FUSION_ZUPT_ENABLED: bool = True
DEFAULT_FUSION_ZUPT_ACCEL_THRESH: float = 0.15
DEFAULT_FUSION_ZUPT_GYRO_THRESH: float = 0.05
DEFAULT_FUSION_ZUPT_MIN_DURATION: float = 0.2
DEFAULT_FUSION_CALIB_SAMPLES: int = 200
DEFAULT_FUSION_CAMERA_STALE_TIMEOUT_SEC: float = 5.0
GRAVITY_MPS2: float = 9.80665
MPU6050_ACCEL_LSB_PER_G: float = 16384.0
MPU6050_GYRO_LSB_PER_DPS: float = 131.0


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


class ImuAxisMapper:
    """Map MPU6050 body-frame vectors into robot forward/left/yaw-rate."""

    def __init__(
        self,
        forward_axis: int,
        left_axis: int,
        yaw_rate_axis: int,
        forward_sign: float,
        left_sign: float,
        yaw_rate_sign: float,
    ) -> None:
        self._forward_axis = forward_axis
        self._left_axis = left_axis
        self._yaw_rate_axis = yaw_rate_axis
        self._forward_sign = forward_sign
        self._left_sign = left_sign
        self._yaw_rate_sign = yaw_rate_sign

    def forward_accel(self, accel: tuple[float, float, float]) -> float:
        return self._forward_sign * accel[self._forward_axis]

    def left_accel(self, accel: tuple[float, float, float]) -> float:
        return self._left_sign * accel[self._left_axis]

    def yaw_rate(self, gyro: tuple[float, float, float]) -> float:
        return self._yaw_rate_sign * gyro[self._yaw_rate_axis]


class Mpu6050Driver:
    """Minimal MPU6050 reader over CircuitPython busio.I2C."""

    _PWR_MGMT_1 = 0x6B
    _ACCEL_XOUT_H = 0x3B
    _WHO_AM_I = 0x75

    def __init__(self, i2c, address: int) -> None:
        self._i2c = i2c
        self._address = address
        self._adafruit = None
        try:
            import adafruit_mpu6050

            self._adafruit = adafruit_mpu6050.MPU6050(i2c, address=address)
        except ImportError:
            self._wake_raw()

    def _wake_raw(self) -> None:
        while not self._i2c.try_lock():
            pass
        try:
            self._i2c.writeto(self._address, bytes([self._PWR_MGMT_1, 0x00]))
        finally:
            self._i2c.unlock()

    def _read_raw(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        buf = bytearray(14)
        while not self._i2c.try_lock():
            pass
        try:
            self._i2c.writeto(self._address, bytes([self._ACCEL_XOUT_H]))
            self._i2c.readfrom_into(self._address, buf)
        finally:
            self._i2c.unlock()

        def _i16(hi: int, lo: int) -> int:
            v = (hi << 8) | lo
            return v - 65536 if v >= 32768 else v

        ax = _i16(buf[0], buf[1])
        ay = _i16(buf[2], buf[3])
        az = _i16(buf[4], buf[5])
        gx = _i16(buf[8], buf[9])
        gy = _i16(buf[10], buf[11])
        gz = _i16(buf[12], buf[13])

        accel = (
            ax / MPU6050_ACCEL_LSB_PER_G * GRAVITY_MPS2,
            ay / MPU6050_ACCEL_LSB_PER_G * GRAVITY_MPS2,
            az / MPU6050_ACCEL_LSB_PER_G * GRAVITY_MPS2,
        )
        dps_to_rps = math.pi / 180.0
        gyro = (
            gx / MPU6050_GYRO_LSB_PER_DPS * dps_to_rps,
            gy / MPU6050_GYRO_LSB_PER_DPS * dps_to_rps,
            gz / MPU6050_GYRO_LSB_PER_DPS * dps_to_rps,
        )
        return accel, gyro

    def read(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        if self._adafruit is not None:
            return tuple(self._adafruit.acceleration), tuple(self._adafruit.gyro)
        return self._read_raw()


class PlanarPoseEKF:
    """Extended Kalman filter for planar differential-drive pose.

    State (robot origin, map frame):
      x = [px, py, yaw, v_forward, omega]^T
    """

    _N = 5

    def __init__(
        self,
        process_pos_std: float,
        process_yaw_std: float,
        process_vel_std: float,
        process_omega_std: float,
    ) -> None:
        self.x = np.zeros((self._N, 1), dtype=np.float64)
        self.P = np.diag(
            [0.25, 0.25, 0.5, 0.5, 0.5]
        ).astype(np.float64)
        self._q_pos = process_pos_std ** 2
        self._q_yaw = process_yaw_std ** 2
        self._q_vel = process_vel_std ** 2
        self._q_omega = process_omega_std ** 2
        self.initialized = False
        self._yaw_unwrapped = 0.0

    def _state(self) -> tuple[float, float, float, float, float]:
        return (
            float(self.x[0, 0]),
            float(self.x[1, 0]),
            float(self.x[2, 0]),
            float(self.x[3, 0]),
            float(self.x[4, 0]),
        )

    def initialize(self, px: float, py: float, yaw: float) -> None:
        self._yaw_unwrapped = yaw
        self.x[:] = [[px], [py], [yaw], [0.0], [0.0]]
        self.P = np.diag([0.09, 0.09, 0.1, 0.25, 0.25]).astype(np.float64)
        self.initialized = True

    def hard_reset(self, px: float, py: float, yaw: float) -> None:
        self.initialize(px, py, yaw)

    def predict(
        self,
        dt: float,
        *,
        a_forward: float = 0.0,
        omega_drive: float | None = None,
    ) -> None:
        if not self.initialized or dt <= 0.0:
            return
        dt = min(dt, EKF_PREDICT_DT_CAP)

        px, py, yaw, v, omega = self._state()
        if omega_drive is not None:
            omega = omega_drive
        v_new = v + a_forward * dt
        yaw_mid = yaw + 0.5 * omega * dt
        px_new = px + v * math.cos(yaw_mid) * dt
        py_new = py + v * math.sin(yaw_mid) * dt
        yaw_new = yaw + omega * dt
        self._yaw_unwrapped = yaw_new

        self.x[:] = [[px_new], [py_new], [yaw_new], [v_new], [omega]]

        c = math.cos(yaw_mid)
        s = math.sin(yaw_mid)
        F = np.array(
            [
                [1.0, 0.0, -v * s * dt, c * dt, 0.0],
                [0.0, 1.0, v * c * dt, s * dt, 0.0],
                [0.0, 0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        Q = np.diag(
            [
                self._q_pos * dt,
                self._q_pos * dt,
                self._q_yaw * dt,
                self._q_vel * dt,
                self._q_omega * dt,
            ]
        ).astype(np.float64)
        self.P = F @ self.P @ F.T + Q

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        innov = z - H @ self.x
        if innov.shape[0] >= 3:
            innov[2, 0] = _normalize_yaw_rad(float(innov[2, 0]))
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.x[2, 0] = self._yaw_unwrapped + _normalize_yaw_rad(
            float(self.x[2, 0] - self._yaw_unwrapped)
        )
        self._yaw_unwrapped = float(self.x[2, 0])
        I = np.eye(self._N, dtype=np.float64)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def update_camera(self, px: float, py: float, yaw: float, pos_std: float, yaw_std: float) -> None:
        if not self.initialized:
            self.initialize(px, py, yaw)
            return
        yaw_meas = self._yaw_unwrapped + _normalize_yaw_rad(yaw - self._yaw_unwrapped)
        z = np.array([[px], [py], [yaw_meas]], dtype=np.float64)
        H = np.zeros((3, self._N), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        R = np.diag([pos_std ** 2, pos_std ** 2, yaw_std ** 2]).astype(np.float64)
        self._update(z, H, R)

    def update_wheel_velocity(self, v: float, omega: float, v_std: float, omega_std: float) -> None:
        if not self.initialized:
            return
        z = np.array([[v], [omega]], dtype=np.float64)
        H = np.zeros((2, self._N), dtype=np.float64)
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        R = np.diag([v_std ** 2, omega_std ** 2]).astype(np.float64)
        self._update(z, H, R)

    def update_gyro(self, omega: float, omega_std: float) -> None:
        if not self.initialized:
            return
        z = np.array([[omega]], dtype=np.float64)
        H = np.zeros((1, self._N), dtype=np.float64)
        H[0, 4] = 1.0
        R = np.array([[omega_std ** 2]], dtype=np.float64)
        self._update(z, H, R)

    def update_zupt(self, vel_std: float, omega_std: float) -> None:
        if not self.initialized:
            return
        z = np.zeros((2, 1), dtype=np.float64)
        H = np.zeros((2, self._N), dtype=np.float64)
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        R = np.diag([vel_std ** 2, omega_std ** 2]).astype(np.float64)
        self._update(z, H, R)

    def pose(self) -> tuple[float, float, float]:
        px, py, yaw, _, _ = self._state()
        return px, py, yaw

    def velocity(self) -> tuple[float, float]:
        _, _, _, v, omega = self._state()
        return v, omega


class PoseEstimationSensorFusionNode(Node):
    """EKF fusion of camera pose, wheel encoders, and MPU6050 into /current_position."""

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
        self.declare_parameter('tof_init_retries', DEFAULT_TOF_INIT_RETRIES)
        self.declare_parameter('tof_reinit_after_fail_count', DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT)
        self.declare_parameter('i2c_frequency', DEFAULT_I2C_FREQUENCY)
        self.declare_parameter('mpu_enabled', DEFAULT_MPU_ENABLED)
        self.declare_parameter('mpu_addr', DEFAULT_MPU_ADDR)
        self.declare_parameter('mpu_poll_hz', DEFAULT_MPU_POLL_HZ)
        self.declare_parameter('mpu_debug_interval_sec', DEFAULT_MPU_DEBUG_INTERVAL_SEC)
        self.declare_parameter('mpu_reinit_after_fail_count', DEFAULT_MPU_REINIT_AFTER_FAIL_COUNT)
        self.declare_parameter('imu_fusion_enabled', DEFAULT_IMU_FUSION_ENABLED)
        self.declare_parameter('fusion_publish_hz', DEFAULT_FUSION_PUBLISH_HZ)
        self.declare_parameter('imu_forward_axis', DEFAULT_IMU_FORWARD_AXIS)
        self.declare_parameter('imu_left_axis', DEFAULT_IMU_LEFT_AXIS)
        self.declare_parameter('imu_yaw_rate_axis', DEFAULT_IMU_YAW_RATE_AXIS)
        self.declare_parameter('imu_forward_sign', DEFAULT_IMU_FORWARD_SIGN)
        self.declare_parameter('imu_left_sign', DEFAULT_IMU_LEFT_SIGN)
        self.declare_parameter('imu_yaw_rate_sign', DEFAULT_IMU_YAW_RATE_SIGN)
        self.declare_parameter('ekf_process_pos_std', DEFAULT_EKF_PROCESS_POS_STD)
        self.declare_parameter('ekf_process_yaw_std', DEFAULT_EKF_PROCESS_YAW_STD)
        self.declare_parameter('ekf_process_vel_std', DEFAULT_EKF_PROCESS_VEL_STD)
        self.declare_parameter('ekf_process_omega_std', DEFAULT_EKF_PROCESS_OMEGA_STD)
        self.declare_parameter('ekf_camera_pos_std', DEFAULT_EKF_CAMERA_POS_STD)
        self.declare_parameter('ekf_camera_yaw_std', DEFAULT_EKF_CAMERA_YAW_STD)
        self.declare_parameter('ekf_camera_pos_std_moving', DEFAULT_EKF_CAMERA_POS_STD_MOVING)
        self.declare_parameter('ekf_camera_yaw_std_moving', DEFAULT_EKF_CAMERA_YAW_STD_MOVING)
        self.declare_parameter('ekf_wheel_vel_std', DEFAULT_EKF_WHEEL_VEL_STD)
        self.declare_parameter('ekf_wheel_omega_std', DEFAULT_EKF_WHEEL_OMEGA_STD)
        self.declare_parameter('ekf_gyro_std', DEFAULT_EKF_GYRO_STD)
        self.declare_parameter('ekf_accel_std', DEFAULT_EKF_ACCEL_STD)
        self.declare_parameter('ekf_zupt_vel_std', DEFAULT_EKF_ZUPT_VEL_STD)
        self.declare_parameter('ekf_zupt_omega_std', DEFAULT_EKF_ZUPT_OMEGA_STD)
        self.declare_parameter('fusion_reset_pos_threshold', DEFAULT_FUSION_RESET_POS_THRESHOLD)
        self.declare_parameter('fusion_accel_deadband', DEFAULT_FUSION_ACCEL_DEADBAND)
        self.declare_parameter('fusion_max_speed', DEFAULT_FUSION_MAX_SPEED)
        self.declare_parameter('fusion_max_omega', DEFAULT_FUSION_MAX_OMEGA)
        self.declare_parameter('wheel_gap_reset_sec', DEFAULT_WHEEL_GAP_RESET_SEC)
        self.declare_parameter('wheel_catchup_max_sec', DEFAULT_WHEEL_CATCHUP_MAX_SEC)
        self.declare_parameter(
            'fusion_calib_stationary_omega', DEFAULT_FUSION_CALIB_STATIONARY_OMEGA
        )
        self.declare_parameter(
            'fusion_reset_yaw_threshold', DEFAULT_FUSION_RESET_YAW_THRESHOLD
        )
        self.declare_parameter('fusion_zupt_enabled', DEFAULT_FUSION_ZUPT_ENABLED)
        self.declare_parameter('fusion_zupt_accel_thresh', DEFAULT_FUSION_ZUPT_ACCEL_THRESH)
        self.declare_parameter('fusion_zupt_gyro_thresh', DEFAULT_FUSION_ZUPT_GYRO_THRESH)
        self.declare_parameter('fusion_zupt_min_duration', DEFAULT_FUSION_ZUPT_MIN_DURATION)
        self.declare_parameter('fusion_calib_samples', DEFAULT_FUSION_CALIB_SAMPLES)
        self.declare_parameter('fusion_camera_stale_timeout_sec', DEFAULT_FUSION_CAMERA_STALE_TIMEOUT_SEC)

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
        self._tof_debug_interval_sec = float(
            self.get_parameter('tof_debug_interval_sec').get_parameter_value().double_value
            or DEFAULT_TOF_DEBUG_INTERVAL_SEC
        )
        self._tof_init_retries = int(
            self.get_parameter('tof_init_retries').get_parameter_value().integer_value or DEFAULT_TOF_INIT_RETRIES
        )
        self._tof_reinit_after_fail_count = int(
            self.get_parameter('tof_reinit_after_fail_count').get_parameter_value().integer_value
            or DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT
        )
        self._i2c_frequency = int(
            self.get_parameter('i2c_frequency').get_parameter_value().integer_value or DEFAULT_I2C_FREQUENCY
        )
        self._use_tca9548a = self.get_parameter('use_tca9548a').get_parameter_value().bool_value
        self._mux_addr = int(self.get_parameter('tca9548a_addr').get_parameter_value().integer_value)
        self._mpu_enabled = self.get_parameter('mpu_enabled').get_parameter_value().bool_value
        self._mpu_addr = int(
            self.get_parameter('mpu_addr').get_parameter_value().integer_value or DEFAULT_MPU_ADDR
        )
        mpu_poll_hz = float(
            self.get_parameter('mpu_poll_hz').get_parameter_value().double_value or DEFAULT_MPU_POLL_HZ
        )
        self._mpu_debug_interval_sec = float(
            self.get_parameter('mpu_debug_interval_sec').get_parameter_value().double_value
            or DEFAULT_MPU_DEBUG_INTERVAL_SEC
        )
        self._mpu_reinit_after_fail_count = int(
            self.get_parameter('mpu_reinit_after_fail_count').get_parameter_value().integer_value
            or DEFAULT_MPU_REINIT_AFTER_FAIL_COUNT
        )
        self._imu_fusion_enabled = self.get_parameter('imu_fusion_enabled').get_parameter_value().bool_value
        fusion_publish_hz = float(
            self.get_parameter('fusion_publish_hz').get_parameter_value().double_value
            or DEFAULT_FUSION_PUBLISH_HZ
        )
        self._imu_axes = ImuAxisMapper(
            int(self.get_parameter('imu_forward_axis').get_parameter_value().integer_value),
            int(self.get_parameter('imu_left_axis').get_parameter_value().integer_value),
            int(self.get_parameter('imu_yaw_rate_axis').get_parameter_value().integer_value),
            float(self.get_parameter('imu_forward_sign').get_parameter_value().double_value),
            float(self.get_parameter('imu_left_sign').get_parameter_value().double_value),
            float(self.get_parameter('imu_yaw_rate_sign').get_parameter_value().double_value),
        )
        self._ekf = PlanarPoseEKF(
            float(self.get_parameter('ekf_process_pos_std').get_parameter_value().double_value),
            float(self.get_parameter('ekf_process_yaw_std').get_parameter_value().double_value),
            float(self.get_parameter('ekf_process_vel_std').get_parameter_value().double_value),
            float(self.get_parameter('ekf_process_omega_std').get_parameter_value().double_value),
        )
        self._ekf_camera_pos_std = float(
            self.get_parameter('ekf_camera_pos_std').get_parameter_value().double_value
        )
        self._ekf_camera_yaw_std = float(
            self.get_parameter('ekf_camera_yaw_std').get_parameter_value().double_value
        )
        self._ekf_camera_pos_std_moving = float(
            self.get_parameter('ekf_camera_pos_std_moving').get_parameter_value().double_value
        )
        self._ekf_camera_yaw_std_moving = float(
            self.get_parameter('ekf_camera_yaw_std_moving').get_parameter_value().double_value
        )
        self._ekf_wheel_vel_std = float(
            self.get_parameter('ekf_wheel_vel_std').get_parameter_value().double_value
        )
        self._ekf_wheel_omega_std = float(
            self.get_parameter('ekf_wheel_omega_std').get_parameter_value().double_value
        )
        self._ekf_gyro_std = float(self.get_parameter('ekf_gyro_std').get_parameter_value().double_value)
        self._ekf_accel_std = float(self.get_parameter('ekf_accel_std').get_parameter_value().double_value)
        self._ekf_zupt_vel_std = float(
            self.get_parameter('ekf_zupt_vel_std').get_parameter_value().double_value
        )
        self._ekf_zupt_omega_std = float(
            self.get_parameter('ekf_zupt_omega_std').get_parameter_value().double_value
        )
        self._fusion_reset_pos_threshold = float(
            self.get_parameter('fusion_reset_pos_threshold').get_parameter_value().double_value
        )
        self._fusion_accel_deadband = float(
            self.get_parameter('fusion_accel_deadband').get_parameter_value().double_value
        )
        self._fusion_max_speed = float(
            self.get_parameter('fusion_max_speed').get_parameter_value().double_value
        )
        self._fusion_max_omega = float(
            self.get_parameter('fusion_max_omega').get_parameter_value().double_value
        )
        self._wheel_gap_reset_sec = float(
            self.get_parameter('wheel_gap_reset_sec').get_parameter_value().double_value
        )
        self._wheel_catchup_max_sec = float(
            self.get_parameter('wheel_catchup_max_sec').get_parameter_value().double_value
        )
        self._fusion_calib_stationary_omega = float(
            self.get_parameter('fusion_calib_stationary_omega').get_parameter_value().double_value
        )
        self._fusion_reset_yaw_threshold = float(
            self.get_parameter('fusion_reset_yaw_threshold').get_parameter_value().double_value
        )
        self._fusion_zupt_enabled = self.get_parameter('fusion_zupt_enabled').get_parameter_value().bool_value
        self._fusion_zupt_accel_thresh = float(
            self.get_parameter('fusion_zupt_accel_thresh').get_parameter_value().double_value
        )
        self._fusion_zupt_gyro_thresh = float(
            self.get_parameter('fusion_zupt_gyro_thresh').get_parameter_value().double_value
        )
        self._fusion_zupt_min_duration = float(
            self.get_parameter('fusion_zupt_min_duration').get_parameter_value().double_value
        )
        self._fusion_calib_samples = int(
            self.get_parameter('fusion_calib_samples').get_parameter_value().integer_value
        )
        self._fusion_camera_stale_timeout_sec = float(
            self.get_parameter('fusion_camera_stale_timeout_sec').get_parameter_value().double_value
        )
        if self._request_timeout <= 0.0:
            self._request_timeout = DEFAULT_REQUEST_TIMEOUT
        if self._tof_debug_interval_sec <= 0.0:
            self._tof_debug_interval_sec = DEFAULT_TOF_DEBUG_INTERVAL_SEC
        if self._tof_init_retries <= 0:
            self._tof_init_retries = 1
        if self._tof_reinit_after_fail_count <= 0:
            self._tof_reinit_after_fail_count = DEFAULT_TOF_REINIT_AFTER_FAIL_COUNT
        if self._i2c_frequency <= 0:
            self._i2c_frequency = DEFAULT_I2C_FREQUENCY
        if fusion_publish_hz < 0.0:
            fusion_publish_hz = DEFAULT_FUSION_PUBLISH_HZ
        self._fusion_publish_period_ns = (
            0 if fusion_publish_hz <= 0.0 else int(1e9 / fusion_publish_hz)
        )

        self._latest_time_s: float | None = None
        self._last_radar_text: str | None = None
        self._radar_fetch_error_logged = False
        self._tof_error_logged = False
        self._tof_backend = 'none'
        self._tof_readers: dict[str, object] = {}
        self._tof_consecutive_fail_count = 0
        self._last_tof_debug = self.get_clock().now()
        self._i2c = None
        self._i2c_lock = threading.Lock()
        self._tof_mux = None
        self._radar_urls = [
            f'http://{self._remote_host}:{self._remote_port}/data/simulation_data',
            f'http://{self._remote_host}:{self._remote_port}/simulation_data',
        ]
        self._radar_url_index = 0

        self._ekf_lock = threading.Lock()
        self._prev_left_rad: float | None = None
        self._prev_right_rad: float | None = None
        self._prev_wheel_stamp_ns: int | None = None
        self._left_rad: float | None = None
        self._right_rad: float | None = None
        self._last_predict_ns: int | None = None
        self._last_publish_ns: int = 0
        self._last_camera_ns: int | None = None
        self._camera_valid = False
        self._camera_x: float | None = None
        self._camera_y: float | None = None
        self._camera_yaw_deg: float | None = None
        self._fused_yaw_deg: float | None = None
        self._robot_motion_status: str | None = None
        self._stopped_camera_samples: list[tuple[float, float, float]] = []
        self._stopped_sample_count: int = 0
        self._stopped_kept_count: int = 0
        self._latest_wheel_v = 0.0
        self._latest_wheel_omega = 0.0
        self._mpu: Mpu6050Driver | None = None
        self._mpu_available = False
        self._mpu_fail_count = 0
        self._last_mpu_debug = self.get_clock().now()
        self._gyro_bias = 0.0
        self._accel_bias_forward = 0.0
        self._calib_gyro: list[float] = []
        self._calib_accel_fwd: list[float] = []
        self._calib_done = self._fusion_calib_samples <= 0
        self._zupt_since_ns: int | None = None
        self._last_imu_a_forward = 0.0
        self._last_imu_omega = 0.0
        self._agent_dbg_last_ms = 0.0

        self._wheel_cb_group = ReentrantCallbackGroup()
        self._sensor_cb_group = MutuallyExclusiveCallbackGroup()

        self._pub = self.create_publisher(PoseStamped, self._output_topic, 10)
        self._pub_radar_sensor = self.create_publisher(String, self._radar_topic, 10)
        self.create_subscription(
            JointState,
            self._wheel_topic,
            self._on_wheel_state,
            50,
            callback_group=self._wheel_cb_group,
        )
        self.create_subscription(
            PoseStamped,
            self._camera_topic,
            self._on_camera_pose,
            10,
            callback_group=self._wheel_cb_group,
        )
        self.create_subscription(
            String,
            self._motion_status_topic,
            self._on_robot_motion_status,
            10,
            callback_group=self._wheel_cb_group,
        )
        if self._time_topic:
            self.create_subscription(
                String,
                self._time_topic,
                self._on_time,
                10,
                callback_group=self._wheel_cb_group,
            )
        if poll_hz > 0.0:
            self.create_timer(
                1.0 / poll_hz,
                self._poll_radar,
                callback_group=self._sensor_cb_group,
            )
        if fusion_debug_hz > 0.0:
            self.create_timer(
                1.0 / fusion_debug_hz,
                self._log_fusion_debug,
                callback_group=self._wheel_cb_group,
            )
        if fusion_publish_hz > 0.0:
            self.create_timer(
                1.0 / fusion_publish_hz,
                self._publish_on_timer,
                callback_group=self._wheel_cb_group,
            )

        if self._use_real_sensor:
            self._tof_readers = self._build_tof_readers()
            self._tof_backend = self._tof_backend if self._tof_readers else 'none'
            if self._mpu_enabled and self._imu_fusion_enabled:
                self._init_mpu()
            if mpu_poll_hz > 0.0 and self._mpu_available:
                self.create_timer(
                    1.0 / mpu_poll_hz,
                    self._poll_mpu,
                    callback_group=self._sensor_cb_group,
                )

        imu_mode = (
            'mpu+ekf'
            if self._mpu_available and self._imu_fusion_enabled
            else 'wheel+ekf'
        )
        self.get_logger().info(
            'pose_estimation_sensor_fusion started; EKF fusing '
            f'{self._camera_topic} + {self._wheel_topic}'
            f'{" + MPU6050" if self._mpu_available else ""} -> {self._output_topic}; '
            f'mode={imu_mode}, publish_hz={fusion_publish_hz}, '
            f'counts_per_rev={self._counts_per_rev:.1f}, '
            f'count_average_per_meter={self._count_avg_per_m:.1f}, '
            f'count_diff_per_degree={self._count_diff_per_deg:.3f}, '
            f'origin_forward_offset={self._origin_offset:.3f}m, '
            f'radar={"real-tof" if self._use_real_sensor else "upstream"} '
            f'backend={self._tof_backend} -> {self._radar_topic}, poll_hz={poll_hz}, '
            f'debug_log={_DBG_LOG_PATH}'
        )

    def _on_robot_motion_status(self, msg: String) -> None:
        status = (msg.data or '').strip().lower()
        # The robot moved, so the pose averaged while stopped is no longer valid.
        if status == 'moving' and self._stopped_camera_samples:
            self._stopped_camera_samples.clear()
            self._stopped_sample_count = 0
            self._stopped_kept_count = 0
        self._robot_motion_status = status

    # ----- EKF fusion core ----------------------------------------------------

    def _agent_dbg_throttled(
        self, hypothesis_id: str, location: str, message: str, data: dict
    ) -> None:
        now_ms = time.time() * 1000.0
        if now_ms - self._agent_dbg_last_ms < 250.0:
            return
        self._agent_dbg_last_ms = now_ms
        # #region agent log
        _agent_dbg_fusion(hypothesis_id, location, message, data)
        # #endregion

    def _predict_to(
        self,
        stamp_ns: int,
        *,
        omega_override: float | None = None,
        dt_override: float | None = None,
    ) -> None:
        with self._ekf_lock:
            if self._last_predict_ns is None:
                self._last_predict_ns = stamp_ns
                return
            if dt_override is not None:
                dt = dt_override
            else:
                dt = (stamp_ns - self._last_predict_ns) * 1e-9
            if dt <= 0.0:
                return
            a_forward = (
                self._last_imu_a_forward if self._mpu_available and self._imu_fusion_enabled else 0.0
            )
            # Wheel encoders are the authoritative yaw-rate source for prediction;
            # gyro is fused as a measurement update only (see _poll_mpu).
            if omega_override is not None:
                omega = omega_override
            else:
                omega = self._latest_wheel_omega
            yaw_before = float(self._ekf.x[2, 0]) if self._ekf.initialized else None
            self._ekf.predict(dt, a_forward=a_forward, omega_drive=omega)
            yaw_after = float(self._ekf.x[2, 0]) if self._ekf.initialized else None
            self._last_predict_ns = stamp_ns
        # #region agent log
        self._agent_dbg_throttled(
            'H1',
            'pose_estimation_sensor_fusion_node.py:_predict_to',
            'ekf predict',
            {
                'dt': round(dt, 4),
                'dt_override': dt_override is not None,
                'omega_used': round(omega, 4),
                'omega_override': omega_override is not None,
                'omega_enc': round(self._latest_wheel_omega, 4),
                'imu_omega': round(self._last_imu_omega, 4),
                'mpu_available': self._mpu_available,
                'imu_fusion': self._imu_fusion_enabled,
                'yaw_before_deg': None if yaw_before is None else round(math.degrees(yaw_before), 2),
                'yaw_after_deg': None if yaw_after is None else round(math.degrees(yaw_after), 2),
                'motion_status': self._robot_motion_status,
            },
        )
        # #endregion

    def _predict_wheel_interval(self, stamp_ns: int, omega: float, dt_total: float) -> None:
        remaining = max(0.0, dt_total)
        while remaining > 1e-6:
            step = min(remaining, EKF_PREDICT_DT_CAP)
            self._predict_to(stamp_ns, omega_override=omega, dt_override=step)
            remaining -= step

    def _wheel_delta_to_velocities(
        self, d_left_count: float, d_right_count: float, dt: float
    ) -> tuple[float, float]:
        d_center = 0.5 * (d_left_count + d_right_count) / self._count_avg_per_m
        d_yaw_deg = (d_right_count - d_left_count) / self._count_diff_per_deg
        d_yaw = math.radians(d_yaw_deg)
        if dt <= 0.0:
            return 0.0, 0.0
        return d_center / dt, d_yaw / dt

    def _clamp_speed(self, v: float) -> float:
        return max(-self._fusion_max_speed, min(self._fusion_max_speed, v))

    def _clamp_omega(self, omega: float) -> float:
        return max(-self._fusion_max_omega, min(self._fusion_max_omega, omega))

    def _robot_is_stationary(self) -> bool:
        return (
            self._robot_motion_status == 'stopped'
            and abs(self._latest_wheel_omega) < self._fusion_calib_stationary_omega
            and abs(self._latest_wheel_v) < 0.05
        )

    def _wheel_rates_from_joint(
        self, left_rad_s: float, right_rad_s: float
    ) -> tuple[float, float]:
        rad_to_counts = self._counts_per_rev / (2.0 * math.pi)
        d_left = left_rad_s * rad_to_counts
        d_right = right_rad_s * rad_to_counts
        return self._wheel_delta_to_velocities(d_left, d_right, 1.0)

    def _camera_is_stale(self, now_ns: int) -> bool:
        if self._last_camera_ns is None:
            return True
        age_s = (now_ns - self._last_camera_ns) * 1e-9
        return age_s > self._fusion_camera_stale_timeout_sec

    def _maybe_zupt(self, stamp_ns: int, accel_fwd: float, gyro_yaw: float) -> None:
        if not self._fusion_zupt_enabled:
            self._zupt_since_ns = None
            return
        still = (
            abs(accel_fwd) < self._fusion_zupt_accel_thresh
            and abs(gyro_yaw) < self._fusion_zupt_gyro_thresh
        )
        if still:
            if self._zupt_since_ns is None:
                self._zupt_since_ns = stamp_ns
            elif (stamp_ns - self._zupt_since_ns) * 1e-9 >= self._fusion_zupt_min_duration:
                with self._ekf_lock:
                    self._ekf.update_zupt(self._ekf_zupt_vel_std, self._ekf_zupt_omega_std)
        else:
            self._zupt_since_ns = None

    def _apply_camera_measurement(self, ox: float, oy: float, yaw: float, moving: bool) -> None:
        with self._ekf_lock:
            if self._ekf.initialized:
                px, py, yaw_est = self._ekf.pose()
                if math.hypot(ox - px, oy - py) > self._fusion_reset_pos_threshold:
                    self._ekf.hard_reset(ox, oy, yaw)
                    self.get_logger().warn(
                        f'EKF hard-reset to camera pose ({ox:.3f}, {oy:.3f}) '
                        f'after {self._fusion_reset_pos_threshold:.2f}m innovation'
                    )
                    return
                yaw_innov = abs(_normalize_yaw_rad(yaw - yaw_est))
                if yaw_innov > self._fusion_reset_yaw_threshold:
                    self._ekf.hard_reset(ox, oy, yaw)
                    self.get_logger().warn(
                        f'EKF hard-reset to camera yaw {math.degrees(yaw):.1f}deg '
                        f'after {math.degrees(yaw_innov):.1f}deg innovation'
                    )
                    return
            pos_std = self._ekf_camera_pos_std_moving if moving else self._ekf_camera_pos_std
            yaw_std = self._ekf_camera_yaw_std_moving if moving else self._ekf_camera_yaw_std
            self._ekf.update_camera(ox, oy, yaw, pos_std, yaw_std)

    def _publish_pose(self, stamp_ns: int, *, force: bool = False) -> None:
        if self._fusion_publish_period_ns > 0 and not force:
            if stamp_ns - self._last_publish_ns < self._fusion_publish_period_ns:
                return
        self._last_publish_ns = stamp_ns

        if self._camera_is_stale(stamp_ns):
            with self._ekf_lock:
                ekf_ready = self._ekf.initialized
            if not ekf_ready:
                # #region agent log
                self._agent_dbg_throttled(
                    'H3',
                    'pose_estimation_sensor_fusion_node.py:_publish_pose',
                    'camera stale nan publish',
                    {
                        'last_camera_age_s': None
                        if self._last_camera_ns is None
                        else round((stamp_ns - self._last_camera_ns) * 1e-9, 2),
                        'stale_timeout_s': self._fusion_camera_stale_timeout_sec,
                        'ekf_initialized': False,
                    },
                )
                # #endregion
                out = PoseStamped()
                out.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
                out.header.frame_id = 'map'
                out.pose.position.x = float('nan')
                out.pose.position.y = float('nan')
                out.pose.position.z = 0.0
                out.pose.orientation.w = 1.0
                self._pub.publish(out)
                return
            # #region agent log
            self._agent_dbg_throttled(
                'H3',
                'pose_estimation_sensor_fusion_node.py:_publish_pose',
                'camera stale odom-only publish',
                {
                    'last_camera_age_s': round((stamp_ns - self._last_camera_ns) * 1e-9, 2)
                    if self._last_camera_ns is not None
                    else None,
                    'stale_timeout_s': self._fusion_camera_stale_timeout_sec,
                    'ekf_initialized': True,
                },
            )
            # #endregion

        with self._ekf_lock:
            if not self._ekf.initialized:
                return
            px, py, yaw = self._ekf.pose()
        yaw_pub = _normalize_yaw_rad(yaw)
        self._fused_yaw_deg = math.degrees(yaw_pub)
        qx, qy, qz, qw = _quaternion_from_yaw(yaw_pub)
        out = PoseStamped()
        out.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
        out.header.frame_id = 'map'
        out.pose.position.x = px
        out.pose.position.y = py
        out.pose.position.z = 0.0
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self._pub.publish(out)
        # #region agent log
        with self._ekf_lock:
            ekf_v, ekf_omega = self._ekf.velocity() if self._ekf.initialized else (0.0, 0.0)
        self._agent_dbg_throttled(
            'H2',
            'pose_estimation_sensor_fusion_node.py:_publish_pose',
            'fused pose publish',
            {
                'x': round(px, 3),
                'y': round(py, 3),
                'yaw_deg': round(self._fused_yaw_deg, 2) if self._fused_yaw_deg is not None else None,
                'camera_yaw_deg': self._camera_yaw_deg,
                'ekf_v': round(ekf_v, 3),
                'ekf_omega': round(ekf_omega, 3),
                'wheel_omega': round(self._latest_wheel_omega, 3),
                'imu_omega': round(self._last_imu_omega, 3),
                'motion_status': self._robot_motion_status,
            },
        )
        # #endregion

    def _publish_on_timer(self) -> None:
        stamp_ns = self.get_clock().now().nanoseconds
        self._publish_pose(stamp_ns)

    # ----- wheel odometry -----------------------------------------------------

    def _on_wheel_state(self, msg: JointState) -> None:
        left_rad = right_rad = None
        left_vel_rad = right_vel_rad = None
        velocities = list(msg.velocity) if msg.velocity else []
        for idx, name in enumerate(msg.name):
            if idx >= len(msg.position):
                break
            if name == self._left_joint:
                left_rad = float(msg.position[idx])
                if idx < len(velocities):
                    left_vel_rad = float(velocities[idx])
            elif name == self._right_joint:
                right_rad = float(msg.position[idx])
                if idx < len(velocities):
                    right_vel_rad = float(velocities[idx])
        if left_rad is None or right_rad is None:
            return

        self._left_rad = left_rad
        self._right_rad = right_rad
        stamp_ns = self._stamp_to_ns(msg)

        if self._prev_left_rad is None or self._prev_right_rad is None:
            self._prev_left_rad = left_rad
            self._prev_right_rad = right_rad
            self._prev_wheel_stamp_ns = stamp_ns
            self._last_predict_ns = stamp_ns
            return

        dt = 0.02
        if self._prev_wheel_stamp_ns is not None:
            dt = max(1e-4, (stamp_ns - self._prev_wheel_stamp_ns) * 1e-9)

        rad_to_counts = self._counts_per_rev / (2.0 * math.pi)
        d_left_count = (left_rad - self._prev_left_rad) * rad_to_counts
        d_right_count = (right_rad - self._prev_right_rad) * rad_to_counts
        self._prev_left_rad = left_rad
        self._prev_right_rad = right_rad
        self._prev_wheel_stamp_ns = stamp_ns

        has_joint_vel = (
            left_vel_rad is not None
            and right_vel_rad is not None
            and math.isfinite(left_vel_rad)
            and math.isfinite(right_vel_rad)
        )
        gap = dt > self._wheel_gap_reset_sec
        if has_joint_vel:
            v_enc, omega_enc = self._wheel_rates_from_joint(left_vel_rad, right_vel_rad)
        else:
            v_enc, omega_enc = self._wheel_delta_to_velocities(d_left_count, d_right_count, dt)
        v_enc = self._clamp_speed(v_enc)
        omega_enc = self._clamp_omega(omega_enc)
        self._latest_wheel_v = v_enc
        self._latest_wheel_omega = omega_enc

        predict_dt = min(dt, self._wheel_catchup_max_sec)
        if gap:
            # #region agent log
            self._agent_dbg_throttled(
                'H2',
                'pose_estimation_sensor_fusion_node.py:_on_wheel_state',
                'wheel gap catchup',
                {
                    'dt': round(dt, 4),
                    'predict_dt': round(predict_dt, 4),
                    'omega_enc': round(omega_enc, 4),
                    'has_joint_vel': has_joint_vel,
                },
            )
            # #endregion

        self._predict_wheel_interval(stamp_ns, omega_enc, predict_dt)

        with self._ekf_lock:
            self._ekf.update_wheel_velocity(
                v_enc, omega_enc, self._ekf_wheel_vel_std, self._ekf_wheel_omega_std
            )

        self._publish_pose(stamp_ns)

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

        moving = self._robot_motion_status == 'moving'
        if not moving:
            self._stopped_camera_samples.append((ox, oy, yaw))
            ox, oy, yaw, kept = _robust_average_pose(
                self._stopped_camera_samples,
                self._stopped_avg_pos_floor_m,
                self._stopped_avg_yaw_floor_rad,
                self._stopped_avg_outlier_factor,
            )
            self._stopped_sample_count = len(self._stopped_camera_samples)
            self._stopped_kept_count = kept

        stamp_ns = self._stamp_to_ns(msg)
        self._predict_to(stamp_ns)
        self._last_camera_ns = stamp_ns
        self._camera_valid = True
        self._camera_x = ox
        self._camera_y = oy
        self._camera_yaw_deg = math.degrees(yaw)
        self._apply_camera_measurement(ox, oy, yaw, moving)
        self._publish_pose(stamp_ns, force=True)

    # ----- MPU6050 ------------------------------------------------------------

    def _init_mpu(self) -> None:
        try:
            i2c = self._get_i2c()
        except Exception as exc:
            self.get_logger().warn(f'MPU6050 skipped: failed to open I2C bus: {exc}')
            return
        addresses = self._scan_main_i2c_addresses(i2c)
        if self._mpu_addr not in addresses:
            self.get_logger().warn(
                f'MPU6050 not found at 0x{self._mpu_addr:02x}; EKF will use wheel+camera only'
            )
            return
        try:
            with self._i2c_lock:
                self._mpu = Mpu6050Driver(i2c, self._mpu_addr)
            self._mpu_available = True
            self.get_logger().info(f'MPU6050 ready at 0x{self._mpu_addr:02x}')
        except Exception as exc:
            self.get_logger().warn(f'MPU6050 init failed: {exc}')

    def _finish_imu_calibration(self) -> None:
        if self._calib_gyro:
            self._gyro_bias = sum(self._calib_gyro) / len(self._calib_gyro)
        if self._calib_accel_fwd:
            self._accel_bias_forward = sum(self._calib_accel_fwd) / len(self._calib_accel_fwd)
        self._calib_done = True
        self.get_logger().info(
            f'MPU6050 calibration done: gyro_bias={self._gyro_bias:.4f} rad/s, '
            f'accel_bias_fwd={self._accel_bias_forward:.3f} m/s^2'
        )

    def _poll_mpu(self) -> None:
        if not self._mpu_available or self._mpu is None:
            return
        stamp_ns = self.get_clock().now().nanoseconds
        try:
            with self._i2c_lock:
                accel, gyro = self._mpu.read()
            self._mpu_fail_count = 0
        except Exception as exc:
            self._mpu_fail_count += 1
            if self._mpu_fail_count >= self._mpu_reinit_after_fail_count:
                self.get_logger().warn(f'MPU6050 read failures; reinitializing ({exc})')
                self._mpu = None
                self._mpu_available = False
                self._init_mpu()
                self._mpu_fail_count = 0
            return

        a_forward = self._imu_axes.forward_accel(accel) - self._accel_bias_forward
        omega = self._imu_axes.yaw_rate(gyro) - self._gyro_bias
        self._last_imu_omega = omega

        if not self._calib_done:
            if not self._robot_is_stationary():
                return
            self._calib_gyro.append(omega)
            self._calib_accel_fwd.append(self._imu_axes.forward_accel(accel))
            # #region agent log
            self._agent_dbg_throttled(
                'H4',
                'pose_estimation_sensor_fusion_node.py:_poll_mpu',
                'imu calibrating',
                {
                    'calib_samples': len(self._calib_gyro),
                    'calib_target': self._fusion_calib_samples,
                    'raw_omega': round(omega, 4),
                    'wheel_omega': round(self._latest_wheel_omega, 4),
                    'motion_status': self._robot_motion_status,
                },
            )
            # #endregion
            if len(self._calib_gyro) >= self._fusion_calib_samples:
                self._finish_imu_calibration()
            return

        if abs(a_forward) < self._fusion_accel_deadband:
            a_forward = 0.0
        self._last_imu_a_forward = a_forward

        if self._imu_fusion_enabled:
            with self._ekf_lock:
                if self._ekf.initialized:
                    self._ekf.update_gyro(omega, self._ekf_gyro_std)
            self._maybe_zupt(stamp_ns, a_forward, omega)
            # #region agent log
            self._agent_dbg_throttled(
                'H5',
                'pose_estimation_sensor_fusion_node.py:_poll_mpu',
                'imu poll',
                {
                    'omega': round(omega, 4),
                    'gyro_bias': round(self._gyro_bias, 4),
                    'wheel_omega': round(self._latest_wheel_omega, 4),
                    'zupt_since': self._zupt_since_ns is not None,
                    'motion_status': self._robot_motion_status,
                },
            )
            # #endregion

        if self._fusion_publish_period_ns == 0:
            self._publish_pose(stamp_ns)

        now = self.get_clock().now()
        if (now - self._last_mpu_debug).nanoseconds * 1e-9 >= self._mpu_debug_interval_sec:
            self._last_mpu_debug = now
            v, w = self._latest_wheel_v, self._latest_wheel_omega
            self.get_logger().debug(
                f'[mpu] a_fwd={a_forward:+.3f} omega={omega:+.3f} '
                f'wheel_v={v:+.3f} wheel_omega={w:+.3f}'
            )

    # ----- fusion + publish (legacy section marker) ---------------------------

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

    def _poll_radar(self) -> None:
        if self._use_real_sensor:
            radar_text = self._build_radar_text_from_tof()
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
            else:
                labels.append(f'0x{addr:02x}')
        self.get_logger().info(
            'main I2C scan: ' + (', '.join(labels) if labels else 'no devices detected')
        )

    def _scan_mux_channels_for_vl53(self, tca) -> list[int]:
        found: list[int] = []
        for ch in range(8):
            try:
                addresses = tca[ch].scan()
            except Exception:
                continue
            if VL53_I2C_ADDR in addresses:
                found.append(ch)
        return found

    def _try_init_vl53l0x(self, bus):
        try:
            import adafruit_vl53l0x
        except ImportError:
            return None, None

        last_exc: Exception | None = None
        for _ in range(self._tof_init_retries):
            try:
                sensor = adafruit_vl53l0x.VL53L0X(bus)
                try:
                    sensor.start_continuous()
                except Exception:
                    pass
                return sensor, 'VL53L0X'
            except Exception as exc:
                last_exc = exc
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
            return lambda: float(sensor.range) / 1000.0

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

        mux_channels = self._scan_mux_channels_for_vl53(tca)
        if not mux_channels:
            return readers

        if self._tof_front_channel in mux_channels:
            front_channel = self._tof_front_channel
        else:
            front_channel = mux_channels[0]
            self.get_logger().info(
                'I2C scan found VL53 (0x29) on TCA9548A channel(s) '
                f'{mux_channels}; front not on configured channel '
                f'{self._tof_front_channel}, using channel {front_channel}'
            )

        reader, model = self._init_vl53_reader(tca[front_channel], 'front', front_channel)
        if reader is not None and model is not None:
            readers['front'] = reader
            self._tof_backend = model
            self.get_logger().info(
                f'front {model} ready on TCA9548A channel {front_channel} '
                f'(discovered channels={mux_channels})'
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

        readers = self._init_front_tof_on_main_bus(i2c, addresses)
        if not readers and self._use_tca9548a:
            readers = self._init_front_tof_via_mux(i2c)

        if not readers:
            self.get_logger().error(
                'no ToF sensors initialized; expected VL53 at 0x29 on the main I2C bus'
            )
        return readers

    def _clamp_tof_distance(self, raw: float) -> tuple[float, str]:
        if raw < self._tof_min_range_m:
            return self._tof_min_range_m, 'CLAMP_MIN'
        if raw > self._tof_max_range_m:
            return self._tof_max_range_m, 'CLAMP_MAX'
        return raw, 'OK'

    def _build_radar_text_from_tof(self) -> str | None:
        if not self._tof_readers:
            if not self._tof_error_logged:
                self.get_logger().error('real-sensor mode enabled but no ToF readers available')
                self._tof_error_logged = True
            return None

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
                    'front ToF reads failed repeatedly; attempting I2C rediscovery '
                    f'(count={self._tof_consecutive_fail_count})'
                )
                self._tof_readers = self._build_tof_readers()
                self._tof_backend = self._tof_backend if self._tof_readers else 'none'
                self._tof_consecutive_fail_count = 0

        self._maybe_log_radar_debug(debug_values)
        timestamp_s = self._resolve_time_s()
        return (
            f'{timestamp_s:.6f},'
            f'{values["front"]:.3f},{values["right"]:.3f},'
            f'{values["left"]:.3f},{values["rear"]:.3f}'
        )

    def _maybe_log_radar_debug(
        self, debug_values: dict[str, tuple[float | None, str, float]]
    ) -> None:
        if not self._tof_debug_enabled:
            return
        now = self.get_clock().now()
        delta = (now - self._last_tof_debug).nanoseconds / 1_000_000_000.0
        if delta < self._tof_debug_interval_sec:
            return
        self._last_tof_debug = now

        summary = []
        for name in RADAR_DIRECTIONS:
            raw, status, pub = debug_values[name]
            raw_mm = 'None' if raw is None else f'{int(raw * 1000.0)}'
            summary.append(f'{name}:raw_mm={raw_mm},status={status},pub={pub:.3f}')
        self.get_logger().info('radar tof diag | ' + ' | '.join(summary))

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
        with self._ekf_lock:
            ekf_v, ekf_omega = self._ekf.velocity() if self._ekf.initialized else (0.0, 0.0)
        self.get_logger().info(
            f'[ekf] left_count={left_cnt}, right_count={right_cnt}, '
            f'camera_xy={camera_xy}, camera_yaw={cam_yaw}, fused_yaw={fused_yaw}, '
            f'v={ekf_v:+.3f} omega={ekf_omega:+.3f}, '
            f'cam_samples={self._stopped_kept_count}/{self._stopped_sample_count}'
        )

    def _stamp_to_ns(self, msg) -> int:
        stamp = msg.header.stamp
        ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if ns <= 0:
            return self.get_clock().now().nanoseconds
        return ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseEstimationSensorFusionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
