import argparse
import json
import math
from pathlib import Path


def compute_theoretical_intrinsics(camera_info: dict) -> dict:
    camera = camera_info["camera"]
    width = int(camera["width"])
    height = int(camera["height"])
    fov_x = float(camera["fieldOfView"])

    if width <= 0 or height <= 0:
        raise ValueError("Camera width/height must be positive.")
    if not (0.0 < fov_x < math.pi):
        raise ValueError("fieldOfView must be in radians and in (0, pi).")

    # Webots fieldOfView is horizontal FOV.
    fx = (width / 2.0) / math.tan(fov_x / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0

    k = [
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ]

    distortion = {
        "model": "plumb_bob",
        "coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
    }

    return {
        "camera_name": camera_info.get("camera_name", "front_camera"),
        "image_size": {
            "width": width,
            "height": height,
        },
        "fov": {
            "horizontal_rad": fov_x,
            "horizontal_deg": math.degrees(fov_x),
        },
        "intrinsics": {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "skew": 0.0,
            "K": k,
        },
        "distortion": distortion,
        "camera_pose_on_robot": camera_info.get("transform", {}),
        "source": "theoretical_from_camera_info_json",
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate theoretical camera intrinsics from camera_info.json"
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=Path(__file__).with_name("camera_info.json"),
        help="Input camera info JSON file",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("camera_intrinsic.json"),
        help="Output camera intrinsic JSON file",
    )
    args = parser.parse_args()

    if not args.camera_info.exists():
        raise FileNotFoundError(f"camera info file not found: {args.camera_info}")

    with args.camera_info.open("r", encoding="utf-8") as f:
        camera_info = json.load(f)

    result = compute_theoretical_intrinsics(camera_info)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    print(f"Saved camera intrinsics to: {args.output}")
    print(
        "fx={:.6f}, fy={:.6f}, cx={:.3f}, cy={:.3f}".format(
            result["intrinsics"]["fx"],
            result["intrinsics"]["fy"],
            result["intrinsics"]["cx"],
            result["intrinsics"]["cy"],
        )
    )


if __name__ == "__main__":
    main()
