import json
import math
import os
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

PERF_METRICS_ENABLED = True


def _load_intrinsics(intrinsic_path: Path) -> tuple[np.ndarray, np.ndarray]:
    with intrinsic_path.open('r', encoding='utf-8') as f:
        payload = json.load(f)

    intr = payload.get('intrinsics', {})
    fx = float(intr['fx'])
    fy = float(intr['fy'])
    cx = float(intr['cx'])
    cy = float(intr['cy'])
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    dist = payload.get('distortion', {}).get('coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])
    dist_coeffs = np.array(dist, dtype=np.float64).reshape(-1, 1)
    return K, dist_coeffs


def _load_tag_world_map(tag_map_path: Path) -> dict[int, np.ndarray]:
    with tag_map_path.open('r', encoding='utf-8') as f:
        payload = json.load(f)

    result: dict[int, np.ndarray] = {}
    for item in payload.get('tags', []):
        tag_id = int(item['id'])
        corners = np.array(item['world_corners'], dtype=np.float64)
        if corners.shape != (4, 3):
            continue
        result[tag_id] = corners
    return result


def _detect_apriltag_corners(image: np.ndarray) -> tuple[list[np.ndarray], Optional[np.ndarray]]:
    if not hasattr(cv2, 'aruco'):
        raise RuntimeError('OpenCV aruco module is unavailable.')
    if not hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
        raise RuntimeError('AprilTag dictionary DICT_APRILTAG_36h11 is unavailable.')

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dict_id = cv2.aruco.DICT_APRILTAG_36h11
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

    # OpenCV API differs across versions:
    # - Newer: DetectorParameters + ArucoDetector(...).detectMarkers(...)
    # - Older: DetectorParameters_create + detectMarkers(...)
    if hasattr(cv2.aruco, 'DetectorParameters'):
        params = cv2.aruco.DetectorParameters()
    elif hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        raise RuntimeError('OpenCV aruco DetectorParameters API is unavailable.')

    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return corners, ids


def _opencv_apriltag_runtime_supported() -> tuple[bool, str]:
    if not hasattr(cv2, 'aruco'):
        return False, 'OpenCV aruco module is unavailable'
    if not hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
        return False, 'OpenCV aruco APRILTAG dictionary is unavailable'

    # On some Raspberry Pi system OpenCV builds (notably 4.6.x), AprilTag detection
    # can crash in native code (SIGSEGV). Keep node alive by disabling pose estimation.
    version = str(getattr(cv2, '__version__', 'unknown'))
    try:
        parts = version.split('.')
        major = int(parts[0])
        minor = int(parts[1]) if len(parts) > 1 else 0
    except Exception:
        return False, f'Cannot parse OpenCV version: {version}'

    if (major, minor) < (4, 7):
        return False, f'OpenCV {version} is too old for stable AprilTag runtime on this target (need >= 4.7)'

    return True, f'OpenCV {version}'


def _rotation_matrix_to_euler_zyx_deg(R: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        yaw = math.atan2(R[1, 0], R[0, 0])
        pitch = math.atan2(-R[2, 0], sy)
        roll = math.atan2(R[2, 1], R[2, 2])
    else:
        yaw = math.atan2(-R[0, 1], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        roll = 0.0

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def estimate_camera_world_position(
    image: np.ndarray,
    K: np.ndarray,
    dist_coeffs: np.ndarray,
    tag_world: dict[int, np.ndarray],
    cam_offset_x_robot: float,
    detected: Optional[tuple[list[np.ndarray], Optional[np.ndarray]]] = None,
) -> Optional[dict]:
    timing_payload: Optional[dict] = None
    if PERF_METRICS_ENABLED:
        t_total_start = time.perf_counter()
        t_detect_start = time.perf_counter()
    if detected is None:
        corners_list, ids = _detect_apriltag_corners(image)
    else:
        corners_list, ids = detected
    if PERF_METRICS_ENABLED:
        detect_ms = (time.perf_counter() - t_detect_start) * 1000.0

    if ids is None or len(ids) == 0:
        return None

    obj_points = []
    img_points = []

    order = [2, 3, 0, 1]
    for marker_corners, marker_id_arr in zip(corners_list, ids):
        tag_id = int(marker_id_arr[0])
        if tag_id not in tag_world:
            continue

        img4 = marker_corners.reshape(4, 2)[order, :]
        world4_raw = tag_world[tag_id]
        world4 = world4_raw[[3, 2, 1, 0], :]
        for p_img, p_obj in zip(img4, world4):
            img_points.append(p_img)
            obj_points.append(p_obj)

    if len(obj_points) < 4:
        return None

    obj = np.asarray(obj_points, dtype=np.float64)
    img = np.asarray(img_points, dtype=np.float64)
    if PERF_METRICS_ENABLED:
        t_pnp_start = time.perf_counter()
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    if PERF_METRICS_ENABLED:
        pnp_ms = (time.perf_counter() - t_pnp_start) * 1000.0
        total_ms = (time.perf_counter() - t_total_start) * 1000.0
        timing_payload = {
            'detect': float(detect_ms),
            'pnp': float(pnp_ms),
            'total': float(total_ms),
        }
    cam_world = -R.T @ tvec
    cam_world = cam_world.reshape(3)
    R_wc = R.T
    roll_deg, pitch_deg, yaw_deg = _rotation_matrix_to_euler_zyx_deg(R_wc)

    forward_world = R_wc @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    heading_x0_deg = math.degrees(math.atan2(float(forward_world[1]), float(forward_world[0])))
    heading_x0_rad = math.radians(heading_x0_deg)

    robot_x = float(cam_world[0]) - cam_offset_x_robot * math.cos(heading_x0_rad)
    robot_y = float(cam_world[1]) - cam_offset_x_robot * math.sin(heading_x0_rad)

    return {
        'camera_position_world': {
            'x': float(cam_world[0]),
            'y': float(cam_world[1]),
            'z': float(cam_world[2]),
        },
        'camera_orientation_world_deg': {
            'roll': float(roll_deg),
            'pitch': float(pitch_deg),
            'yaw': float(yaw_deg),
            'heading_x0': float(heading_x0_deg),
        },
        'num_points': int(len(obj_points)),
        'num_tags_used': int(len(img_points) // 4),
        'timing_ms': timing_payload,
        'robot_pose_world': {
            'x': robot_x,
            'y': robot_y,
            'heading_x0': float(heading_x0_deg),
        },
        'tag_corners': corners_list,
        'tag_ids': ids,
    }


class PoseEstimationNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_estimation')

        self.declare_parameter('camera_topic', '/front_camera')
        self.declare_parameter('intrinsic_path', '')
        self.declare_parameter('tag_map_path', '')
        self.declare_parameter('camera_offset_x', 0.105)
        self.declare_parameter('tick_hz', 0.0)
        self.declare_parameter('log_every_sec', 1.0)
        self.declare_parameter('pose_estimation', False)
        self.declare_parameter('allow_legacy_opencv', False)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        intrinsic_path_raw = self.get_parameter('intrinsic_path').get_parameter_value().string_value
        tag_map_path_raw = self.get_parameter('tag_map_path').get_parameter_value().string_value
        self.camera_offset_x = float(self.get_parameter('camera_offset_x').get_parameter_value().double_value)
        self.tick_hz = float(self.get_parameter('tick_hz').get_parameter_value().double_value)
        self.log_every_sec = float(self.get_parameter('log_every_sec').get_parameter_value().double_value)
        self._enabled = bool(self.get_parameter('pose_estimation').get_parameter_value().bool_value)
        self._allow_legacy_opencv = bool(
            self.get_parameter('allow_legacy_opencv').get_parameter_value().bool_value
        )

        if self._enabled:
            ok_runtime, reason = _opencv_apriltag_runtime_supported()
            if not ok_runtime:
                if self._allow_legacy_opencv:
                    self.get_logger().warn(
                        f'pose_estimation running in legacy OpenCV mode (may crash): {reason}'
                    )
                else:
                    self.get_logger().warn(
                        f'pose_estimation requested but disabled to avoid native crash: {reason}'
                    )
                    self._enabled = False

        pkg_share = get_package_share_directory('unibots')
        default_intrinsic = os.path.join(pkg_share, 'config', 'camera_intrinsic.json')
        default_tag_map = os.path.join(pkg_share, 'config', 'tag_world_map.json')

        intrinsic_path = Path(intrinsic_path_raw) if intrinsic_path_raw else Path(default_intrinsic)
        tag_map_path = Path(tag_map_path_raw) if tag_map_path_raw else Path(default_tag_map)

        self.K, self.dist_coeffs = _load_intrinsics(intrinsic_path)
        self.tag_world = _load_tag_world_map(tag_map_path)

        if not self.tag_world:
            raise RuntimeError(f'No tag map entries loaded from {tag_map_path}')

        self._bridge = CvBridge()
        self._last_pose_log = 0.0
        self._last_warn_log = 0.0
        self._latest_image: Optional[np.ndarray] = None
        if PERF_METRICS_ENABLED:
            self._perf_window_start = time.monotonic()
            self._perf_frames = 0
            self._perf_success = 0
            self._perf_tags = 0
            self._perf_detect_ms = 0.0
            self._perf_pnp_ms = 0.0
            self._perf_total_ms = 0.0

        self._pub_current_position = self.create_publisher(PoseStamped, '/current_position', 10)
        self._pub_processed_image = self.create_publisher(Image, '/processed_image', 10)
        self.create_subscription(Image, camera_topic, self._on_image, 10)
        if self.tick_hz > 0.0:
            self.create_timer(1.0 / self.tick_hz, self._on_tick)

        state = 'enabled' if self._enabled else 'disabled'
        tick_hz_text = f'{self.tick_hz:.3f}Hz' if self.tick_hz > 0.0 else 'image-rate'
        self.get_logger().info(
            f'pose_estimation {state}; topic={camera_topic}, tick={tick_hz_text}, '
            f'intrinsics={intrinsic_path}, tag_map={tag_map_path}'
        )

    def _should_log(self, last_time: float) -> bool:
        now = time.monotonic()
        if self.log_every_sec <= 0.0:
            return True
        return (now - last_time) >= self.log_every_sec

    def _on_image(self, msg: Image) -> None:
        if not self._enabled:
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            if self._should_log(self._last_warn_log):
                self._last_warn_log = time.monotonic()
                self.get_logger().warn(f'failed to decode image: {exc}')
            return

        if self.tick_hz > 0.0:
            self._latest_image = image
            return

        self._process_image(image)

    def _on_tick(self) -> None:
        if not self._enabled or self.tick_hz <= 0.0:
            return
        if self._latest_image is None:
            return
        self._process_image(self._latest_image)

    def _process_image(self, image: np.ndarray) -> None:
        if PERF_METRICS_ENABLED:
            self._perf_frames += 1

        try:
            corners_list, ids = _detect_apriltag_corners(image)
            estimate = estimate_camera_world_position(
                image=image,
                K=self.K,
                dist_coeffs=self.dist_coeffs,
                tag_world=self.tag_world,
                cam_offset_x_robot=self.camera_offset_x,
                detected=(corners_list, ids),
            )
        except Exception as exc:
            if self._should_log(self._last_warn_log):
                self._last_warn_log = time.monotonic()
                self.get_logger().warn(f'pose estimation failed: {exc}')
            return

        processed = self._render_processed_image(image, corners_list, ids, estimate)
        if processed is not None:
            msg_image = self._bridge.cv2_to_imgmsg(processed, encoding='bgr8')
            msg_image.header.stamp = self.get_clock().now().to_msg()
            msg_image.header.frame_id = 'camera'
            self._pub_processed_image.publish(msg_image)

        if estimate is None:
            if self._should_log(self._last_warn_log):
                self._last_warn_log = time.monotonic()
                self.get_logger().warn('pose estimation: no valid AprilTag-world correspondences')
            return

        used_tags = int(estimate.get('num_tags_used', 0))
        if PERF_METRICS_ENABLED:
            timing = estimate.get('timing_ms', {}) or {}
            detect_ms = float(timing.get('detect', 0.0))
            pnp_ms = float(timing.get('pnp', 0.0))
            total_ms = float(timing.get('total', 0.0))
            self._perf_success += 1
            self._perf_tags += used_tags
            self._perf_detect_ms += detect_ms
            self._perf_pnp_ms += pnp_ms
            self._perf_total_ms += total_ms

        if self._should_log(self._last_pose_log):
            self._last_pose_log = time.monotonic()
            robot = estimate['robot_pose_world']
            cam = estimate['camera_position_world']
            used_points = int(estimate.get('num_points', 0))
            if PERF_METRICS_ENABLED:
                now = time.monotonic()
                window_sec = max(1e-6, now - self._perf_window_start)
                proc_hz = self._perf_frames / window_sec
                solve_hz = self._perf_success / window_sec
                tags_per_sec = self._perf_tags / window_sec
                avg_detect_ms = self._perf_detect_ms / max(1, self._perf_success)
                avg_pnp_ms = self._perf_pnp_ms / max(1, self._perf_success)
                avg_total_ms = self._perf_total_ms / max(1, self._perf_success)
                self.get_logger().info(
                    'Estimated pose '
                    f"robot=({robot['x']:.2f}, {robot['y']:.2f}, {robot['heading_x0']:.0f}deg), "
                    f"camera=({cam['x']:.2f}, {cam['y']:.2f}, {cam['z']:.2f}), "
                    f'tags={used_tags}, points={used_points}, '
                    f'proc_hz={proc_hz:.2f}, solve_hz={solve_hz:.2f}, tags_per_sec={tags_per_sec:.2f}, '
                    f'detect_ms={avg_detect_ms:.2f}, pnp_ms={avg_pnp_ms:.2f}, total_ms={avg_total_ms:.2f}'
                )
                self._perf_window_start = now
                self._perf_frames = 0
                self._perf_success = 0
                self._perf_tags = 0
                self._perf_detect_ms = 0.0
                self._perf_pnp_ms = 0.0
                self._perf_total_ms = 0.0
            else:
                self.get_logger().info(
                    'Estimated pose '
                    f"robot=({robot['x']:.2f}, {robot['y']:.2f}, {robot['heading_x0']:.2f}deg), "
                    f"camera=({cam['x']:.2f}, {cam['y']:.2f}, {cam['z']:.2f}), "
                    f'tags={used_tags}, points={used_points}'
                )

        robot = estimate['robot_pose_world']
        heading_rad = math.radians(float(robot['heading_x0']))
        msg_out = PoseStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'map'
        msg_out.pose.position.x = float(robot['x'])
        msg_out.pose.position.y = float(robot['y'])
        msg_out.pose.position.z = 0.0
        msg_out.pose.orientation.x = 0.0
        msg_out.pose.orientation.y = 0.0
        msg_out.pose.orientation.z = math.sin(heading_rad * 0.5)
        msg_out.pose.orientation.w = math.cos(heading_rad * 0.5)
        self._pub_current_position.publish(msg_out)

    def _render_processed_image(
        self,
        image: np.ndarray,
        corners_list: list[np.ndarray],
        ids: Optional[np.ndarray],
        estimate: Optional[dict],
    ) -> Optional[np.ndarray]:
        if image is None:
            return None
        overlay = image.copy()
        if hasattr(cv2, 'aruco') and ids is not None and len(ids) > 0:
            try:
                cv2.aruco.drawDetectedMarkers(overlay, corners_list, ids)
            except Exception:
                pass

            # Draw world (x, y) coordinates near each corner
            img_order = [2, 3, 0, 1]
            world_order = [3, 2, 1, 0]
            for marker_corners, marker_id_arr in zip(corners_list, ids):
                tag_id = int(marker_id_arr[0])
                if tag_id not in self.tag_world:
                    continue
                world_corners = self.tag_world[tag_id]  # shape (4, 3)
                img_corners = marker_corners.reshape(4, 2)
                for k in range(4):
                    img_pt = img_corners[img_order[k]]
                    world_pt = world_corners[world_order[k]]
                    wx, wy = float(world_pt[0]), float(world_pt[1])
                    px = int(img_pt[0]) + 4
                    py = int(img_pt[1]) - 4
                    cv2.putText(
                        overlay,
                        f'({wx:.2f},{wy:.2f})',
                        (px, py),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.35,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

        tag_count = int(len(ids)) if ids is not None else 0
        if estimate is not None:
            robot = estimate['robot_pose_world']
            text = (
                f"tags={tag_count} robot=({robot['x']:.2f}, {robot['y']:.2f}, "
                f"{robot['heading_x0']:.0f}deg)"
            )
        else:
            text = f"tags={tag_count} pose=unavailable"

        cv2.putText(
            overlay,
            text,
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return overlay


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
