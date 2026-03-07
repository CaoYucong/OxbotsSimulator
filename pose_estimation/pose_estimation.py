import argparse
import json
import math
from pathlib import Path
from typing import Optional
from urllib.request import urlopen

import cv2
import numpy as np


def load_front_camera_image_from_config(config_path: Path) -> np.ndarray:
	with config_path.open("r", encoding="utf-8") as f:
		config = json.load(f)

	data_flow = str(config.get("data_flow", "")).strip().lower()
	workspace_root = config_path.parent

	if data_flow == "file":
		image_path = workspace_root / "controllers" / "supervisor_controller" / "real_time_data" / "front_camera.jpg"
		image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
		if image is None:
			raise FileNotFoundError(f"Failed to read image from file source: {image_path}")
		return image

	if data_flow == "web":
		url = "http://localhost:5003/front_camera"
		with urlopen(url, timeout=3.0) as resp:
			data = resp.read()
		arr = np.frombuffer(data, dtype=np.uint8)
		image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
		if image is None:
			raise ValueError(f"Failed to decode image bytes from web source: {url}")
		return image

	raise ValueError(
		f"Unsupported data_flow='{data_flow}' in {config_path}. Expected 'file' or 'web'."
	)


def _load_intrinsics(intrinsic_path: Path) -> tuple[np.ndarray, np.ndarray]:
	with intrinsic_path.open("r", encoding="utf-8") as f:
		payload = json.load(f)

	intr = payload.get("intrinsics", {})
	fx = float(intr["fx"])
	fy = float(intr["fy"])
	cx = float(intr["cx"])
	cy = float(intr["cy"])
	K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

	dist = payload.get("distortion", {}).get("coefficients", [0.0, 0.0, 0.0, 0.0, 0.0])
	dist_coeffs = np.array(dist, dtype=np.float64).reshape(-1, 1)
	return K, dist_coeffs


def _load_tag_world_map(tag_map_path: Path) -> dict[int, np.ndarray]:
	with tag_map_path.open("r", encoding="utf-8") as f:
		payload = json.load(f)

	result: dict[int, np.ndarray] = {}
	for item in payload.get("tags", []):
		tag_id = int(item["id"])
		corners = np.array(item["world_corners"], dtype=np.float64)
		if corners.shape != (4, 3):
			continue
		result[tag_id] = corners
	return result


def _detect_apriltag_corners(image: np.ndarray) -> tuple[list[np.ndarray], Optional[np.ndarray]]:
	if not hasattr(cv2, "aruco"):
		raise RuntimeError("OpenCV aruco module is unavailable.")
	if not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
		raise RuntimeError("AprilTag dictionary DICT_APRILTAG_36h11 is unavailable.")

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	dict_id = cv2.aruco.DICT_APRILTAG_36h11
	dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
	params = cv2.aruco.DetectorParameters()
	detector = cv2.aruco.ArucoDetector(dictionary, params)
	corners, ids, _ = detector.detectMarkers(gray)
	return corners, ids


def _rotation_matrix_to_euler_zyx_deg(R: np.ndarray) -> tuple[float, float, float]:
	"""Convert rotation matrix to ZYX Euler angles in degrees: roll(X), pitch(Y), yaw(Z)."""
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
	intrinsic_path: Path,
	tag_map_path: Path,
) -> tuple[Optional[dict], np.ndarray]:
	K, dist_coeffs = _load_intrinsics(intrinsic_path)
	tag_world = _load_tag_world_map(tag_map_path)
	corners_list, ids = _detect_apriltag_corners(image)
	debug_img = image.copy()

	if ids is None or len(ids) == 0:
		cv2.putText(
			debug_img,
			"No AprilTag detected",
			(30, 50),
			cv2.FONT_HERSHEY_SIMPLEX,
			1.0,
			(0, 0, 255),
			2,
			cv2.LINE_AA,
		)
		return None, debug_img

	# Draw detected marker borders and ids for quick visual sanity check.
	cv2.aruco.drawDetectedMarkers(debug_img, corners_list, ids)

	obj_points = []
	img_points = []

	# OpenCV marker corner order is TL, TR, BR, BL.
	# tag_world_map stores BL, BR, TR, TL; here we swap Bottom/Top as requested.
	order = [2, 3, 0, 1]
	corner_names = ["TL", "TR", "BR", "BL"]

	for marker_corners, marker_id_arr in zip(corners_list, ids):
		tag_id = int(marker_id_arr[0])
		if tag_id not in tag_world:
			pt = marker_corners.reshape(4, 2).mean(axis=0)
			cv2.putText(
				debug_img,
				f"id={tag_id} not in map",
				(int(pt[0]), int(pt[1])),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.45,
				(0, 0, 255),
				1,
				cv2.LINE_AA,
			)
			continue

		img4 = marker_corners.reshape(4, 2)[order, :]
		world4_raw = tag_world[tag_id]
		world4 = world4_raw[[3, 2, 1, 0], :]
		for idx, (p_img, p_obj) in enumerate(zip(img4, world4)):
			img_points.append(p_img)
			obj_points.append(p_obj)
			x, y = int(p_img[0]), int(p_img[1])
			cv2.circle(debug_img, (x, y), 4, (0, 255, 0), -1)
			label = (
				f"id={tag_id} {corner_names[idx]}"
				f" -> ({p_obj[0]:.3f},{p_obj[1]:.3f},{p_obj[2]:.3f})"
			)
			cv2.putText(
				debug_img,
				label,
				(x + 6, y - 6),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.35,
				(0, 255, 0),
				1,
				cv2.LINE_AA,
			)

	if len(obj_points) < 4:
		cv2.putText(
			debug_img,
			"Not enough mapped points for solvePnP",
			(30, 90),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.8,
			(0, 0, 255),
			2,
			cv2.LINE_AA,
		)
		return None, debug_img

	obj = np.asarray(obj_points, dtype=np.float64)
	img = np.asarray(img_points, dtype=np.float64)
	ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
	if not ok:
		cv2.putText(
			debug_img,
			"solvePnP failed",
			(30, 90),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.8,
			(0, 0, 255),
			2,
			cv2.LINE_AA,
		)
		return None, debug_img

	R, _ = cv2.Rodrigues(rvec)
	cam_world = -R.T @ tvec
	cam_world = cam_world.reshape(3)
	R_wc = R.T
	roll_deg, pitch_deg, yaw_deg = _rotation_matrix_to_euler_zyx_deg(R_wc)

	# Heading with world +X as 0 deg, positive towards +Y.
	# OpenCV camera forward axis is +Z in camera coordinates.
	forward_world = R_wc @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
	heading_x0_deg = math.degrees(math.atan2(float(forward_world[1]), float(forward_world[0])))
	heading_x0_rad = math.radians(heading_x0_deg)

	# Camera extrinsic in robot frame (given): x=0.105, y=0, same heading as robot +X.
	# So robot position is camera position shifted backward along heading by 0.105 m.
	cam_offset_x_robot = 0.105
	robot_x = float(cam_world[0]) - cam_offset_x_robot * math.cos(heading_x0_rad)
	robot_y = float(cam_world[1]) - cam_offset_x_robot * math.sin(heading_x0_rad)

	estimate = {
		"camera_position_world": {
			"x": float(cam_world[0]),
			"y": float(cam_world[1]),
			"z": float(cam_world[2]),
		},
		"camera_orientation_world_deg": {
			"roll": float(roll_deg),
			"pitch": float(pitch_deg),
			"yaw": float(yaw_deg),
			"heading_x0": float(heading_x0_deg),
		},
		"num_points": int(len(obj_points)),
		"num_tags_used": int(len(img_points) // 4),
		"robot_pose_world": {
			"x": robot_x,
			"y": robot_y,
			"heading_x0": float(heading_x0_deg),
		},
	}

	cv2.putText(
		debug_img,
		(
			"cam_world="
			f"({estimate['camera_position_world']['x']:.3f},"
			f"{estimate['camera_position_world']['y']:.3f},"
			f"{estimate['camera_position_world']['z']:.3f})"
		),
		(30, 90),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.75,
		(255, 220, 0),
		2,
		cv2.LINE_AA,
	)

	cv2.putText(
		debug_img,
		(
			"rpy_deg="
			f"({estimate['camera_orientation_world_deg']['roll']:.2f},"
			f"{estimate['camera_orientation_world_deg']['pitch']:.2f},"
			f"{estimate['camera_orientation_world_deg']['yaw']:.2f})"
		),
		(30, 125),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.65,
		(255, 220, 0),
		2,
		cv2.LINE_AA,
	)

	cv2.putText(
		debug_img,
		f"heading_x0_deg={estimate['camera_orientation_world_deg']['heading_x0']:.2f}",
		(30, 155),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.65,
		(255, 220, 0),
		2,
		cv2.LINE_AA,
	)

	cv2.putText(
		debug_img,
		(
			"robot_world="
			f"({estimate['robot_pose_world']['x']:.3f},"
			f"{estimate['robot_pose_world']['y']:.3f},"
			f"{estimate['robot_pose_world']['heading_x0']:.2f}deg)"
		),
		(30, 185),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.62,
		(0, 255, 255),
		2,
		cv2.LINE_AA,
	)

	return estimate, debug_img


def main() -> None:
	parser = argparse.ArgumentParser(description="Pose estimation entrypoint")
	parser.add_argument(
		"--config",
		type=Path,
		default=Path(__file__).resolve().parent.parent / "config.json",
		help="Path to workspace config.json",
	)
	parser.add_argument(
		"--intrinsic",
		type=Path,
		default=Path(__file__).with_name("camera_intrinsic.json"),
		help="Path to camera intrinsic JSON",
	)
	parser.add_argument(
		"--tag-map",
		type=Path,
		default=Path(__file__).with_name("tag_world_map.json"),
		help="Path to tag world map JSON",
	)
	parser.add_argument(
		"--save-debug",
		type=Path,
		default=None,
		help="Optional path to save the loaded input image for debug",
	)
	args = parser.parse_args()

	image = load_front_camera_image_from_config(args.config)

	if args.save_debug is not None:
		args.save_debug.parent.mkdir(parents=True, exist_ok=True)
		ok = cv2.imwrite(str(args.save_debug), image)
		if not ok:
			raise IOError(f"Failed to save debug image to: {args.save_debug}")

	estimate, debug_img = estimate_camera_world_position(
		image=image,
		intrinsic_path=args.intrinsic,
		tag_map_path=args.tag_map,
	)

	recognised_path = (
		args.config.parent
		/ "controllers"
		/ "supervisor_controller"
		/ "real_time_data"
		/ "temp_img_recognised.jpg"
	)
	recognised_path.parent.mkdir(parents=True, exist_ok=True)
	ok = cv2.imwrite(str(recognised_path), debug_img)
	if not ok:
		raise IOError(f"Failed to save recognised image to: {recognised_path}")

	# print(f"Loaded image shape: {image.shape}")
	# print("Image source resolved from config.json successfully.")
	# print(f"Recognised image saved to: {recognised_path}")
	if estimate is None:
		print("Pose estimation failed: no valid AprilTag-world correspondences.")
		return

	# pos = estimate["camera_position_world"]
	# orient = estimate["camera_orientation_world_deg"]
	# print(
	# 	"Estimated camera world position: "
	# 	f"x={pos['x']:.4f}, y={pos['y']:.4f}, z={pos['z']:.4f}"
	# )
	# print(
	# 	"Estimated camera world orientation (deg): "
	# 	f"roll={orient['roll']:.2f}, pitch={orient['pitch']:.2f}, yaw={orient['yaw']:.2f}"
	# )
	# print(
	# 	"Estimated heading (world +X is 0 deg): "
	# 	f"heading_x0={orient['heading_x0']:.2f}"
	# )
	robot = estimate["robot_pose_world"]
	print(
		"Estimated robot world pose: "
		f"({robot['x']:.2f}, {robot['y']:.2f}, {robot['heading_x0']:.2f}deg)"
	)
	print("-----------------------------------")
	# print(
	# 	f"Used tags={estimate['num_tags_used']}, points={estimate['num_points']}"
	# )


if __name__ == "__main__":
	main()
