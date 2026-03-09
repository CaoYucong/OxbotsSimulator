#!/usr/bin/env python3
"""Web bridge runtime node for Raspberry Pi (non-ROS).

This mirrors upstream simulation and front-camera payloads to a local HTTP server.
It also accepts local POST updates for decisions/decision_making_data.
"""

from __future__ import annotations

import json
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional


class MirrorState:
    def __init__(self, default_speed: float = 3.0) -> None:
        default_speed_payload = json.dumps(
            {
                "dynamic_waypoints": "",
                "speed": f"{default_speed:.6f}",
            },
            ensure_ascii=True,
            separators=(",", ":"),
        ).encode("utf-8")

        self._lock = threading.Lock()
        self._items = {
            "/data/simulation_data": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
            },
            "/data/front_camera": {
                "body": b"",
                "content_type": "image/jpeg",
                "has_data": False,
            },
            "/data/decisions": {
                "body": default_speed_payload,
                "content_type": "application/json; charset=utf-8",
                "has_data": True,
            },
            "/data/decision_making_data": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": True,
            },
        }

    def set(self, path: str, body: bytes, content_type: str) -> bool:
        with self._lock:
            item = self._items.get(path)
            if item is None:
                return False
            changed = (
                item["body"] != body
                or item["content_type"] != content_type
                or (not item["has_data"])
            )
            item["body"] = body
            item["content_type"] = content_type
            item["has_data"] = True
            return changed

    def get(self, path: str) -> Optional[dict]:
        with self._lock:
            item = self._items.get(path)
            if item is None:
                return None
            return {
                "body": item["body"],
                "content_type": item["content_type"],
                "has_data": item["has_data"],
            }


def build_handler(state: MirrorState):
    class MirrorHandler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            item = state.get(self.path)
            if item is None:
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Not Found")
                return
            if not item["has_data"]:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Mirror has no upstream data yet")
                return
            body = item["body"]
            self.send_response(200)
            self.send_header("Content-Type", item["content_type"])
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):  # noqa: N802
            if self.path not in ("/data/decisions", "/data/decision_making_data"):
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Not Found")
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
            except Exception:
                length = 0

            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(body.decode("utf-8", errors="ignore"))
                if not isinstance(payload, dict):
                    raise ValueError("payload must be a JSON object")
            except Exception:
                self.send_response(400)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Invalid JSON payload")
                return

            canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
            state.set(self.path, canonical, "application/json; charset=utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"{}")

        def log_message(self, _fmt, *_args):
            return

    return MirrorHandler


class WebBridgeNode:
    def __init__(
        self,
        remote_host: str,
        remote_port: int,
        local_host: str,
        local_port: int,
        request_timeout: float,
        camera_remote_host: str,
        camera_remote_port: int,
        camera_path: str,
        camera_request_timeout: float,
        default_speed: float,
    ) -> None:
        self.remote_targets = (
            {
                "path": "/data/simulation_data",
                "url": f"http://{remote_host}:{remote_port}/data/simulation_data",
                "content_type": "application/json; charset=utf-8",
            },
        )
        self.camera_url = f"http://{camera_remote_host}:{camera_remote_port}{camera_path}"
        self.request_timeout = max(request_timeout, 0.1)
        self.camera_request_timeout = max(camera_request_timeout, 0.1)

        self.state = MirrorState(default_speed=default_speed)
        self.server = ThreadingHTTPServer((local_host, local_port), build_handler(self.state))
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self._error_logged = {item["path"]: False for item in self.remote_targets}
        self._camera_error_logged = False

        print(
            f"[web_bridge] started local mirror http://{local_host}:{local_port} "
            f"(upstream data: {remote_host}:{remote_port}, camera: {camera_remote_host}:{camera_remote_port}{camera_path})"
        )

    def tick(self) -> None:
        self._poll_simulation_data()
        self._poll_front_camera()

    def shutdown(self) -> None:
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass

    def _poll_simulation_data(self) -> None:
        for target in self.remote_targets:
            path = target["path"]
            url = target["url"]
            try:
                with urllib.request.urlopen(url, timeout=self.request_timeout) as res:
                    content_type = res.headers.get_content_type()
                    body = res.read()
                merged_content_type = (
                    f"{content_type}; charset=utf-8" if content_type else target["content_type"]
                )
                self.state.set(path, body, merged_content_type)
                self._error_logged[path] = False
            except Exception as exc:
                if not self._error_logged[path]:
                    print(f"[web_bridge] upstream fetch failed for {url}: {exc}")
                    self._error_logged[path] = True

    def _poll_front_camera(self) -> None:
        try:
            with urllib.request.urlopen(self.camera_url, timeout=self.camera_request_timeout) as res:
                content_type = (res.headers.get_content_type() or "").lower()
                body = res.read()
        except Exception as exc:
            if not self._camera_error_logged:
                print(f"[web_bridge] front camera fetch failed for {self.camera_url}: {exc}")
                self._camera_error_logged = True
            return

        self._camera_error_logged = False
        if not body:
            return
        if content_type and not content_type.startswith("image/"):
            print(
                "[web_bridge] front camera upstream returned non-image content-type: "
                f"{content_type}; check camera_path"
            )
            return

        merged_content_type = f"{content_type}; charset=utf-8" if content_type else "image/jpeg"
        self.state.set("/data/front_camera", body, merged_content_type)
