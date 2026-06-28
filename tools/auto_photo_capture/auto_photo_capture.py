"""Capture periodic 1920x1080 photos from a physical USB camera with live preview.

Auto-selects the first non-virtual camera (skips OBS Virtual Camera, etc.).
The script saves timestamped JPEG files into the local `photos/` directory.
Press `q` or `ESC` in the preview window to stop.
"""

from __future__ import annotations

import argparse
import os
import time
from datetime import datetime
from pathlib import Path

import cv2

try:
	from pygrabber.dshow_graph import FilterGraph
except ImportError:  # pragma: no cover - optional dependency
	FilterGraph = None


TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080
DEFAULT_PHOTO_INTERVAL = 2.0
DEFAULT_CAMERA_INDEX = 0
DEFAULT_CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "").strip()
DEFAULT_CAMERA_NAME = os.getenv("CAMERA_NAME", "").strip()
DEFAULT_OUTPUT_FORMAT = "jpg"
# Skip virtual cameras (OBS, etc.) when auto-selecting a physical USB device.
VIRTUAL_CAMERA_KEYWORDS = ("obs", "virtual", "snap camera", "manycam", "xsplit")


def build_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(
		description="Capture 1920x1080 photos every PHOTO_INTERVAL seconds."
	)
	parser.add_argument(
		"--camera-device",
		default=DEFAULT_CAMERA_DEVICE,
		help="USB camera device path (e.g. /dev/video0). Takes priority over index/name.",
	)
	parser.add_argument(
		"--camera-index",
		type=int,
		default=int(os.getenv("CAMERA_INDEX", DEFAULT_CAMERA_INDEX)),
		help="Fallback camera index when device path/name selection fails (default: 0).",
	)
	parser.add_argument(
		"--camera-name",
		default=DEFAULT_CAMERA_NAME,
		help="Preferred camera name substring; empty = first physical USB camera.",
	)
	parser.add_argument(
		"--photo-interval",
		type=float,
		default=float(os.getenv("PHOTO_INTERVAL", DEFAULT_PHOTO_INTERVAL)),
		help="Seconds between saved photos (default: PHOTO_INTERVAL env var or 5).",
	)
	parser.add_argument(
		"--output-dir",
		type=Path,
		default=Path(__file__).resolve().parent / "photos",
		help="Directory where captured photos will be stored.",
	)
	return parser


def configure_camera(camera: cv2.VideoCapture) -> None:
	camera.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
	camera.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
	camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
	print(
		"Requested capture size: "
		f"{TARGET_WIDTH}x{TARGET_HEIGHT} MJPG"
	)


def _is_virtual_camera(device_name: str) -> bool:
	lowered = device_name.casefold()
	return any(keyword in lowered for keyword in VIRTUAL_CAMERA_KEYWORDS)


def _list_input_devices() -> list[tuple[int, str]]:
	if FilterGraph is None:
		return []
	return list(enumerate(FilterGraph().get_input_devices()))


def resolve_camera_index(camera_name: str | None, fallback_index: int) -> tuple[int, str]:
	devices = _list_input_devices()
	if devices:
		if camera_name:
			name_lower = camera_name.casefold()
			for index, device_name in devices:
				if name_lower in device_name.casefold() and not _is_virtual_camera(device_name):
					return index, device_name
		else:
			for index, device_name in devices:
				if not _is_virtual_camera(device_name):
					return index, device_name

		print("Available cameras:")
		for index, device_name in devices:
			suffix = " (virtual, skipped)" if _is_virtual_camera(device_name) else ""
			print(f"  [{index}] {device_name}{suffix}")

	return fallback_index, f"camera index {fallback_index}"


def open_camera(camera_index: int, camera_device: str = "") -> cv2.VideoCapture:
	if camera_device:
		camera = cv2.VideoCapture(camera_device)
		if camera.isOpened():
			return camera
		camera.release()

	if os.name == "nt":
		camera = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
		if camera.isOpened():
			return camera
		camera.release()

	return cv2.VideoCapture(camera_index)


def ensure_target_resolution(frame):
	height, width = frame.shape[:2]
	if width != TARGET_WIDTH or height != TARGET_HEIGHT:
		raise RuntimeError(
			"Camera is not delivering 1920x1080 frames. "
			f"Got {width}x{height} instead of {TARGET_WIDTH}x{TARGET_HEIGHT}."
		)
	return frame


def rotate_frame_180(frame):
	return cv2.rotate(frame, cv2.ROTATE_180)


def save_frame(output_dir: Path, frame) -> Path:
	timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
	output_path = output_dir / f"photo_{timestamp}.{DEFAULT_OUTPUT_FORMAT}"
	ok = cv2.imwrite(str(output_path), frame)
	if not ok:
		raise RuntimeError(f"Failed to save image to {output_path}")
	return output_path


def main() -> int:
	parser = build_parser()
	args = parser.parse_args()

	if args.photo_interval <= 0:
		parser.error("--photo-interval must be greater than 0")

	args.output_dir.mkdir(parents=True, exist_ok=True)

	camera_device = (args.camera_device or "").strip()
	if camera_device:
		camera_label = camera_device
		camera_index = args.camera_index
	else:
		camera_index, camera_label = resolve_camera_index(args.camera_name, args.camera_index)

	camera = open_camera(camera_index, camera_device)
	if not camera.isOpened():
		target = camera_device or f"'{camera_label}' at index {camera_index}"
		raise RuntimeError(f"Unable to open USB camera {target}")

	if camera_device:
		print(f"Using USB camera device {camera_device}")
	else:
		print(f"Using USB camera {camera_label} (index {camera_index})")

	configure_camera(camera)

	window_name = "Auto Photo Capture"
	cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(window_name, 1280, 720)

	next_capture_time = time.monotonic()
	last_saved_path: Path | None = None

	try:
		while True:
			ok, frame = camera.read()
			if not ok:
				raise RuntimeError("Failed to read frame from camera")

			frame = rotate_frame_180(frame)
			frame = ensure_target_resolution(frame)

			now = time.monotonic()
			if now >= next_capture_time:
				last_saved_path = save_frame(args.output_dir, frame)
				print(f"Saved {last_saved_path}")
				next_capture_time = now + args.photo_interval

			preview = frame.copy()
			status_text = f"{TARGET_WIDTH}x{TARGET_HEIGHT} | interval {args.photo_interval:.2f}s"
			cv2.putText(
				preview,
				status_text,
				(30, 50),
				cv2.FONT_HERSHEY_SIMPLEX,
				1.0,
				(0, 255, 0),
				2,
				cv2.LINE_AA,
			)
			if last_saved_path is not None:
				cv2.putText(
					preview,
					f"Last saved: {last_saved_path.name}",
					(30, 95),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.8,
					(255, 255, 255),
					2,
					cv2.LINE_AA,
				)

			cv2.imshow(window_name, preview)

			key = cv2.waitKey(1) & 0xFF
			if key in (27, ord("q")):
				break
	finally:
		camera.release()
		cv2.destroyAllWindows()

	return 0


if __name__ == "__main__":
	raise SystemExit(main())
