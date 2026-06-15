"""Sensor-fusion pose node: camera absolute pose + wheel odometry.

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

Output:
  - /current_position (geometry_msgs/PoseStamped):
      Fused pose published at the wheel-odometry rate. Computed as:
          current = camera_anchor  (+)  (odom_now (-) odom_at_camera_stamp)
      i.e. take the most recent camera pose and add the relative wheel motion
      accumulated since that camera frame's timestamp.

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

import math
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
COUNT_AVERAGE_PER_METER: float = 3333.3333
COUNT_DIFF_PER_DEGREE: float = 26.9
ORIGIN_FORWARD_OFFSET_M: float = 0.04  # robot origin is 40mm ahead of the axle midpoint
ODOM_HISTORY_SEC: float = 3.0
LEFT_WHEEL_JOINT_NAME: str = 'left_wheel_joint'
RIGHT_WHEEL_JOINT_NAME: str = 'right_wheel_joint'
FUSION_DEBUG_HZ: float = 1.0  # how often to log wheel counts + camera/fused pose (Hz)
ROBOT_MOTION_STATUS_TOPIC: str = '/robot_motion_status'


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


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
        self.create_subscription(JointState, self._wheel_topic, self._on_wheel_state, 50)
        self.create_subscription(PoseStamped, self._camera_topic, self._on_camera_pose, 10)
        self.create_subscription(
            String, self._motion_status_topic, self._on_robot_motion_status, 10
        )
        if fusion_debug_hz > 0.0:
            self.create_timer(1.0 / fusion_debug_hz, self._log_fusion_debug)

        self.get_logger().info(
            'pose_estimation_sensor_fusion started; fusing '
            f'{self._camera_topic} (anchor) + {self._wheel_topic} (odom) -> {self._output_topic}; '
            f'counts_per_rev={self._counts_per_rev:.1f}, '
            f'count_average_per_meter={self._count_avg_per_m:.1f}, '
            f'count_diff_per_degree={self._count_diff_per_deg:.3f}, '
            f'origin_forward_offset={self._origin_offset:.3f}m, '
            f'debug_when={self._motion_status_topic}=stopped'
        )

    def _on_robot_motion_status(self, msg: String) -> None:
        self._robot_motion_status = (msg.data or '').strip().lower()

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
        axle_yaw = self._anchor_yaw + d_yaw

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
            f'camera_xy={camera_xy}, camera_yaw={cam_yaw}, fused_yaw={fused_yaw}'
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
    rclpy.init(args=args)
    node = PoseEstimationSensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
