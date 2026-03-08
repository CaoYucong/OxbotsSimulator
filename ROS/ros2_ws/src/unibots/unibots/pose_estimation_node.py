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
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    return corners, ids


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
) -> Optional[dict]:
    corners_list, ids = _detect_apriltag_corners(image)

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
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
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
        'robot_pose_world': {
            'x': robot_x,
            'y': robot_y,
            'heading_x0': float(heading_x0_deg),
        },
    }


class PoseEstimationNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_estimation')

        self.declare_parameter('camera_topic', '/front_camera')
        self.declare_parameter('intrinsic_path', '')
        self.declare_parameter('tag_map_path', '')
        self.declare_parameter('camera_offset_x', 0.105)
        self.declare_parameter('log_every_sec', 1.0)
        self.declare_parameter('pose_estimation', False)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        intrinsic_path_raw = self.get_parameter('intrinsic_path').get_parameter_value().string_value
        tag_map_path_raw = self.get_parameter('tag_map_path').get_parameter_value().string_value
        self.camera_offset_x = float(self.get_parameter('camera_offset_x').get_parameter_value().double_value)
        self.log_every_sec = float(self.get_parameter('log_every_sec').get_parameter_value().double_value)
        self._enabled = bool(self.get_parameter('pose_estimation').get_parameter_value().bool_value)

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

        self._pub_current_position = self.create_publisher(PoseStamped, '/current_position', 10)
        self.create_subscription(Image, camera_topic, self._on_image, 10)

        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().info(
            f'pose_estimation {state}; topic={camera_topic}, intrinsics={intrinsic_path}, tag_map={tag_map_path}'
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

        try:
            estimate = estimate_camera_world_position(
                image=image,
                K=self.K,
                dist_coeffs=self.dist_coeffs,
                tag_world=self.tag_world,
                cam_offset_x_robot=self.camera_offset_x,
            )
        except Exception as exc:
            if self._should_log(self._last_warn_log):
                self._last_warn_log = time.monotonic()
                self.get_logger().warn(f'pose estimation failed: {exc}')
            return

        if estimate is None:
            if self._should_log(self._last_warn_log):
                self._last_warn_log = time.monotonic()
                self.get_logger().warn('pose estimation: no valid AprilTag-world correspondences')
            return

        if self._should_log(self._last_pose_log):
            self._last_pose_log = time.monotonic()
            robot = estimate['robot_pose_world']
            self.get_logger().info(
                'Estimated robot world pose: '
                f"({robot['x']:.2f}, {robot['y']:.2f}, {robot['heading_x0']:.2f}deg)"
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
