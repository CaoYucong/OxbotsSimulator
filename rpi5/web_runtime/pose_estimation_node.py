#!/usr/bin/env python3
"""Pose estimation runtime node for Raspberry Pi (non-ROS)."""

from __future__ import annotations

import json
import urllib.request
from pathlib import Path


class PoseEstimationNode:
    def __init__(
        self,
        project_root: Path,
        bridge_host: str,
        bridge_port: int,
        enabled: bool = True,
    ) -> None:
        self.enabled = enabled
        self.project_root = project_root
        self.bridge_url = f"http://{bridge_host}:{bridge_port}/data/decision_making_data"

        if not self.enabled:
            print("[pose_estimation] disabled")
            return

        from pose_estimation import pose_estimation as pe  # pylint: disable=import-outside-toplevel

        self._pe = pe
        self.config_path = project_root / "config.json"
        self.intrinsic_path = project_root / "pose_estimation" / "camera_intrinsic.json"
        self.tag_map_path = project_root / "pose_estimation" / "tag_world_map.json"

        self._K, self._dist_coeffs = self._pe._load_intrinsics(self.intrinsic_path)
        self._tag_world = self._pe._load_tag_world_map(self.tag_map_path)
        self._last_pose: dict | None = None
        print(f"[pose_estimation] ready with {self.intrinsic_path.name} and {self.tag_map_path.name}")

    def tick(self) -> None:
        if not self.enabled:
            return

        try:
            image = self._pe.load_front_camera_image_from_config(self.config_path)
            estimate, _ = self._pe.estimate_camera_world_position(
                image=image,
                intrinsic_path=self.intrinsic_path,
                tag_map_path=self.tag_map_path,
            )
            if estimate is None:
                return
            self._last_pose = estimate
            self._post_pose_to_decision_data(estimate)
        except Exception as exc:
            print(f"[pose_estimation] tick failed: {exc}")

    def _post_pose_to_decision_data(self, estimate: dict) -> None:
        payload = self._get_current_decision_payload()
        payload["pose_estimation_robot_pose"] = json.dumps(
            estimate.get("robot_pose_world", {}),
            ensure_ascii=True,
            separators=(",", ":"),
        )
        body = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        req = urllib.request.Request(
            self.bridge_url,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
        )
        with urllib.request.urlopen(req, timeout=0.3) as resp:
            resp.read()

    def _get_current_decision_payload(self) -> dict:
        try:
            with urllib.request.urlopen(self.bridge_url, timeout=0.3) as resp:
                raw = resp.read().decode("utf-8", errors="ignore")
            data = json.loads(raw)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
        return {}
