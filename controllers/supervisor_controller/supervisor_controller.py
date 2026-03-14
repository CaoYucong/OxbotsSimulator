# supervisor_controller.py
# Non-blocking Supervisor controller:
# - random ball placement
# - simplified absorption logic
# - dynamic waypoint handling
from controller import Supervisor, Robot, Camera
import json
import math
import numpy as np
import os
import random
import re
import signal
import socket
import subprocess
import sys
import time
import urllib.request

# optionally use OpenCV for external window
try:
    import cv2
except ImportError:
    cv2 = None


def _post_binary(url, data, content_type):
    try:
        req = urllib.request.Request(
            url,
            data=data,
            method="POST",
            headers={"Content-Type": content_type, "Content-Length": str(len(data))},
        )
        with urllib.request.urlopen(req, timeout=0.3) as res:
            res.read()
        return True
    except Exception:
        return False


def _load_early_data_flow(default="web"):
    try:
        config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config.json"))
        with open(config_path, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            flow = str(payload.get("data_flow", payload.get("data flow", default))).strip().lower()
            if flow in ("web", "file"):
                return flow
    except Exception:
        pass
    return default


def _load_early_run_on_pi(default=False):
    try:
        config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config.json"))
        with open(config_path, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            raw = payload.get("run_on_pi", default)
            if isinstance(raw, bool):
                return raw
            if isinstance(raw, (int, float)):
                return bool(raw)
            text = str(raw).strip().lower()
            if text in ("1", "true", "yes", "y", "on"):
                return True
            if text in ("0", "false", "no", "n", "off"):
                return False
    except Exception:
        pass
    return default


def _load_early_camera_on(default=True):
    try:
        config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config.json"))
        with open(config_path, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            raw = payload.get("camera", default)
            if isinstance(raw, bool):
                return raw
            if isinstance(raw, (int, float)):
                return bool(raw)
            text = str(raw).strip().lower()
            if text in ("on", "true", "1", "yes", "y"):
                return True
            if text in ("off", "false", "0", "no", "n"):
                return False
    except Exception:
        pass
    return default


def _load_early_pose_estimation_on(default=False):
    try:
        config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config.json"))
        with open(config_path, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            raw = payload.get("pose_estimation", payload.get("pose estimation", default))
            if isinstance(raw, bool):
                return raw
            if isinstance(raw, (int, float)):
                return bool(raw)
            text = str(raw).strip().lower()
            if text in ("on", "true", "1", "yes", "y"):
                return True
            if text in ("off", "false", "0", "no", "n"):
                return False
    except Exception:
        pass
    return default


def _post_front_camera_frame(camera_device, endpoint=None, image_path=None, write_local=False):
    if camera_device is None:
        return
    if write_local and image_path:
        try:
            os.makedirs(os.path.dirname(image_path), exist_ok=True)
            camera_device.saveImage(image_path, CAMERA_JPEG_QUALITY)
        except Exception:
            pass
        return
    if endpoint is None:
        return
    if cv2 is not None:
        try:
            raw = camera_device.getImage()
            if raw:
                cam_w = int(camera_device.getWidth())
                cam_h = int(camera_device.getHeight())
                img = np.frombuffer(raw, dtype=np.uint8).reshape((cam_h, cam_w, 4))
                bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            else:
                bgr = None
            ok, encoded = cv2.imencode(
                ".jpg",
                bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY],
            )
            if ok and _post_binary(endpoint, encoded.tobytes(), "image/jpeg"):
                return
        except Exception:
            pass

    try:
        tmp_dir = os.path.join(os.path.dirname(__file__), "real_time_data")
        os.makedirs(tmp_dir, exist_ok=True)
        fallback_path = os.path.join(tmp_dir, "front_camera_fallback.jpg")
        camera_device.saveImage(fallback_path, CAMERA_JPEG_QUALITY)
        with open(fallback_path, "rb") as f:
            img_bytes = f.read()
        _post_binary(endpoint, img_bytes, "image/jpeg")
    except Exception:
        pass


def _trigger_pose_estimation_if_ready():
    """Run pose_estimation periodically without creating overlapping processes."""
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    script_path = os.path.abspath(
        os.path.join(project_root, "pose_estimation", "pose_estimation.py")
    )
    config_path = os.path.abspath(os.path.join(project_root, "config.json"))
    if not os.path.isfile(script_path):
        return
    try:
        proc = getattr(_trigger_pose_estimation_if_ready, "_proc", None)
        if proc is not None and proc.poll() is None:
            return
    except Exception:
        pass
    try:
        proc = subprocess.Popen(
            [
                sys.executable,
                script_path,
                "--config",
                config_path,
            ],
            cwd=project_root,
        )
        _trigger_pose_estimation_if_ready._proc = proc
    except Exception:
        pass


def _print_ground_truth_current_position(node=None):
    """Print ground-truth robot position (x, y, heading_deg)."""
    try:
        target = node
        if target is None:
            target = globals().get("main_robot", None)
        if target is None:
            return
        trans_field = target.getField("translation")
        rot_field = target.getField("rotation")
        if trans_field is None or rot_field is None:
            return
        pos = trans_field.getSFVec3f()
        rot = rot_field.getSFRotation()
        x, y = float(pos[0]), float(pos[1])
        heading_deg = math.degrees(float(rot[3]))
        print(f"[GT] current_position=      ({x:.2f}, {y:.2f}, {heading_deg:.2f}deg)")
    except Exception:
        pass


# =============================================================================
# RUNTIME INITIALIZATION
# Instantiate controller for whichever node this script is attached to.
# It may run on the supervisor or on the main robot; we branch accordingly.
# =============================================================================

supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())
dt = TIME_STEP / 1000.0  # seconds
FIELD_OF_VIEW_DEGREES = 120.0  # degrees, for camera visibility checks (also used by cruise script)


def _has_supervisor_privileges(ctrl):
    """Return True when this controller is attached to a supervisor-capable node."""
    try:
        return ctrl.getRoot() is not None
    except Exception:
        return False

# early constant needed by camera branch before full globals are defined
# main robot name (also redefined later with full constant block)
MAIN_ROBOT_NAME = "MY_ROBOT"
CAMERA_ON = _load_early_camera_on(default=True)
POSE_ESTIMATION_ON = _load_early_pose_estimation_on(default=False)
IS_SUPERVISOR_NODE = _has_supervisor_privileges(supervisor)
EARLY_DATA_FLOW = _load_early_data_flow()
EARLY_RUN_ON_PI = _load_early_run_on_pi(default=False)

camera = None
front_camera_endpoint = None
camera_local_image_path = os.path.join(os.path.dirname(__file__), "real_time_data", "front_camera.jpg")
camera_frame_counter = 0
post_interval_frames = 1
CAMERA_JPEG_QUALITY = 65

if CAMERA_ON:
    # Determine whether we are running on the dedicated supervisor node,
    # main robot, or one of the obstacle robots.
    node_name = supervisor.getName()
    if node_name != "supervisor" and node_name != MAIN_ROBOT_NAME:
        # Obstacle robots do not need this controller.
        sys.exit(0)

    if node_name == MAIN_ROBOT_NAME:
        # Running as the main Cube-Robot: enable front camera streaming.
        robot = supervisor  # object is usable as Robot as well
        camera = None
        try:
            camera = robot.getDevice('front_camera')
            if camera:
                camera.enable(TIME_STEP)
                print(f"[Camera] enabled on {node_name} {camera.getWidth()}x{camera.getHeight()}")
            else:
                print(f"[Camera] front_camera device not found on {node_name}")
        except Exception as e:
            print(f"[Camera] error enabling camera: {e}")
        if camera is None:
            sys.exit(0)

        port = 5001
        try:
            html_port_path = os.path.join(os.path.dirname(__file__), "html_port.txt")
            with open(html_port_path, "r") as f:
                raw = f.read().strip()
            candidate = int(raw)
            if 1 <= candidate <= 65535:
                port = candidate
        except Exception:
            pass

        front_camera_endpoint = None if EARLY_DATA_FLOW == "file" else f"http://localhost:{port}/front_camera"

        # If this node is not a supervisor, keep camera-only behavior and exit.
        # If this node is also the supervisor, continue to full supervisor logic below.
        if not IS_SUPERVISOR_NODE:
            while robot.step(TIME_STEP) != -1:
                camera_frame_counter += 1
                if camera_frame_counter % post_interval_frames != 0:
                    continue
                _post_front_camera_frame(
                    camera,
                    endpoint=front_camera_endpoint,
                    image_path=camera_local_image_path,
                    write_local=(EARLY_DATA_FLOW == "file"),
                )
                if POSE_ESTIMATION_ON and (not EARLY_RUN_ON_PI):
                    _trigger_pose_estimation_if_ready()
                    try:
                        _print_ground_truth_current_position(robot.getSelf())
                    except Exception:
                        _print_ground_truth_current_position()

            sys.exit(0)

# if we reach here, we are the supervisor controller and will proceed with full logic
if not IS_SUPERVISOR_NODE:
    # Safety guard: only supervisor-capable nodes are allowed to execute supervisor APIs.
    sys.exit(0)



# =============================================================================
# GLOBAL CONSTANTS
# Tunable parameters for motion, balls, absorption, and scheduling.
# =============================================================================
DEFAULT_RANDOM_SEED = 1236
DEFAULT_LINEAR_VELOCITY_FALLBACK = 3.0
DEFAULT_ANGULAR_VELOCITY_FALLBACK = 40.0
DEFAULT_VELOCITY = DEFAULT_LINEAR_VELOCITY_FALLBACK  # m/s
DEFAULT_ANGULAR_VELOCITY_MAIN = DEFAULT_ANGULAR_VELOCITY_FALLBACK  # deg/s
DEFAULT_ANGULAR_VELOCITY_OBSTACLE = DEFAULT_ANGULAR_VELOCITY_FALLBACK  # deg/s

MAIN_ROBOT_NAME = "MY_ROBOT"
OBSTACLE_ROBOT_NAMES = ["OBSTACLE_ROBOT_1", "OBSTACLE_ROBOT_2", "OBSTACLE_ROBOT_3"]
OBSTACLE_START_INDICES = [1, 50, 99]

BALL_PREFIX = "BALL_"
BALL_COUNT = 40
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86
BALL_RADIUS = 0.02
MIN_SEPARATION = 2.0 * BALL_RADIUS + 0.001
MAX_TRIES_PER_BALL = 2000
SETTLE_STEPS_AFTER_PLACEMENT = max(1, int(0.2 / dt))
Z_EPS = 0.001
VISIBLE_RANGE_METERS = 2.0
RADAR_MAX_RANGE = 0.8
RADAR_CORRIDOR = 0.2

# Startup placement avoidance around the main robot
ROBOT_X = -0.8
ROBOT_Y = 0.0
ROBOT_HALF_SIZE = 0.1
CLEARANCE = 0.05

# Robot-local absorption area
ABSORB_BOX_HALF_X = 0.12
ABSORB_BOX_HALF_Y = 0.05
ABSORB_LOCATION = (-1.1, 0.0, 0.1)

# Waypoint orientation aliases (radians)
North = math.pi / 2
East = 0.0
South = -math.pi / 2
West = math.pi

# Cruise script execution config
CRUISE_INTERVAL_FRAMES = 15

# Field viewer config
def _load_html_port(path, default_port=5001):
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
        port = int(raw)
        if 1 <= port <= 65535:
            return port
    except Exception:
        pass
    return default_port

# =============================================================================
# PATHS AND FILE LOCATIONS
# Centralized paths used by supervisor, decision modules, and data exchange files.
# =============================================================================
THIS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
WHO_IS_DEV_FILE = os.path.join(PROJECT_ROOT, "config.json")
WHO_IS_DEV_FILE_LEGACY = os.path.join(PROJECT_ROOT, "who_is_developing.txt")
RANDOM_SEED_FILE = os.path.join(THIS_DIR, "random_seed.txt")
REAL_TIME_DATA_DIR = os.path.join(THIS_DIR, "real_time_data")
FIELD_VIEWER_PATH = os.path.abspath(
    os.path.join(THIS_DIR, "..", "..", "tools", "field_viewer", "server.py")
)
HTML_PORT_FILE = os.path.join(THIS_DIR, "html_port.txt")
FIELD_VIEWER_PORT = _load_html_port(HTML_PORT_FILE)
SIM_DATA_ENDPOINT = f"http://localhost:{FIELD_VIEWER_PORT}/data/simulation_data"
DEFAULT_PI_IP = "192.168.50.2"
DECISIONS_CACHE = {}
DECISIONS_SEQ = 0


def _load_runtime_config():
    """Load branch/data-flow config from JSON, fallback to legacy txt branch file."""
    branch = ""
    data_flow = "web"
    run_on_pi = False
    pi_ip = DEFAULT_PI_IP
    default_linear_velocity = DEFAULT_LINEAR_VELOCITY_FALLBACK
    default_angular_velocity = DEFAULT_ANGULAR_VELOCITY_FALLBACK

    try:
        with open(WHO_IS_DEV_FILE, "r") as f:
            raw = f.read().strip()
        payload = json.loads(raw)
        if isinstance(payload, dict):
            branch = str(payload.get("develop_branch", payload.get("develope_brancch", ""))).strip().lower()
            flow_raw = str(payload.get("data_flow", payload.get("data flow", "web"))).strip().lower()
            if flow_raw in ("web", "file"):
                data_flow = flow_raw
            run_on_pi_raw = payload.get("run_on_pi", False)
            if isinstance(run_on_pi_raw, bool):
                run_on_pi = run_on_pi_raw
            elif isinstance(run_on_pi_raw, (int, float)):
                run_on_pi = bool(run_on_pi_raw)
            else:
                run_on_pi = str(run_on_pi_raw).strip().lower() in ("1", "true", "yes", "y", "on")
            ip_raw = str(payload.get("pi_ip", DEFAULT_PI_IP)).strip()
            if ip_raw:
                pi_ip = ip_raw
            speed_raw = payload.get("default_linear_velocity", DEFAULT_LINEAR_VELOCITY_FALLBACK)
            try:
                parsed_speed = float(speed_raw)
                if parsed_speed > 0:
                    default_linear_velocity = parsed_speed
            except Exception:
                pass
            angular_speed_raw = payload.get("default_angular_velocity", DEFAULT_ANGULAR_VELOCITY_FALLBACK)
            try:
                parsed_angular_speed = float(angular_speed_raw)
                if parsed_angular_speed > 0:
                    default_angular_velocity = parsed_angular_speed
            except Exception:
                pass
    except Exception:
        pass

    if not branch:
        try:
            with open(WHO_IS_DEV_FILE_LEGACY, "r") as f:
                branch = f.read().strip().lower()
        except Exception:
            pass

    return branch, data_flow, run_on_pi, pi_ip, default_linear_velocity, default_angular_velocity


def _read_local_text(path):
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except Exception:
        return None


def _write_local_text(path, content):
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(content)
        return True
    except Exception:
        return False


def _load_random_seed(path, default_seed=DEFAULT_RANDOM_SEED):
    """Load random seed from file, fallback to default on any error."""
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
        return int(raw)
    except Exception:
        return default_seed


def _resolve_decision_making_dir(branch=None):
    """Select decision_making folder based on config (develop_branch)."""
    default_dir = os.path.join(PROJECT_ROOT, "decision_making")
    try:
        dev = (branch or "").strip().lower()
        if not dev:
            dev, _, _, _, _, _ = _load_runtime_config()
        if dev == "cyc":
            return os.path.join(PROJECT_ROOT, "decision_making_cyc")
        if dev == "wly":
            return os.path.join(PROJECT_ROOT, "decision_making_wly")
        if dev == "xjj":
            return os.path.join(PROJECT_ROOT, "decision_making_xjj")
        if dev == "ros":
            return os.path.join(PROJECT_ROOT, "decision_making_ros")
    except Exception:
        pass
    return default_dir


DEVELOP_BRANCH, DATA_FLOW, RUN_ON_PI, PI_IP, CONFIG_DEFAULT_LINEAR_VELOCITY, CONFIG_DEFAULT_ANGULAR_VELOCITY = _load_runtime_config()
DEFAULT_VELOCITY = CONFIG_DEFAULT_LINEAR_VELOCITY
DEFAULT_ANGULAR_VELOCITY_MAIN = CONFIG_DEFAULT_ANGULAR_VELOCITY
DEFAULT_ANGULAR_VELOCITY_OBSTACLE = CONFIG_DEFAULT_ANGULAR_VELOCITY
DECISIONS_HOST = PI_IP if RUN_ON_PI else "localhost"
DECISIONS_ENDPOINT = f"http://{DECISIONS_HOST}:{FIELD_VIEWER_PORT}/data/decisions"
DECISION_MAKING_DATA_ENDPOINT = f"http://{DECISIONS_HOST}:{FIELD_VIEWER_PORT}/data/decision_making_data"
DECISIONS_RETRY_SECONDS_ON_PI = 1.0
DECISIONS_RETRY_SLEEP_SECONDS_ON_PI = 0.05
DECISIONS_REFRESH_EVERY_FRAMES = 25
DECISION_MAKING_DIR = _resolve_decision_making_dir(DEVELOP_BRANCH)
DECISION_REAL_TIME_DIR = os.path.join(DECISION_MAKING_DIR, "real_time_data")
RANDOM_SEED = _load_random_seed(RANDOM_SEED_FILE)

WAYPOINTS_HISTORY_FILE = os.path.join(REAL_TIME_DATA_DIR, "waypoints_history.txt")
OBSTACLE_PLAN_FILE = os.path.join(REAL_TIME_DATA_DIR, "obstacle_plan.txt")
BALL_TAKEN_HISTORY_FILE = os.path.join(REAL_TIME_DATA_DIR, "ball_taken_history.txt")
RADAR_SENSOR_FILE = os.path.join(REAL_TIME_DATA_DIR, "radar_sensor.txt")
CRUISE_SCRIPT_PATH = os.path.join(DECISION_MAKING_DIR, "waypoints_cruise.py")
SUPERVISOR_STATUS_FILE = os.path.join(REAL_TIME_DATA_DIR, "supervisor_controller_status.txt")


# =============================================================================
# RUNTIME STATE
# Mutable counters and in-memory status caches.
# =============================================================================
SCORE = 0
STEEL_STORED = 0
PING_STORED = 0
STEEL_HIT = 0
PING_HIT = 0
MAIN_BALL_TAKEN = 0

# Tracks whether each ball has been absorbed to prevent double counting
ball_simple_state = {}  # name -> {"absorbed": bool}


# =============================================================================
# ROBOT REFERENCES
# Resolve main robot and obstacle robot nodes from Webots world definitions.
# =============================================================================
main_robot = supervisor.getFromDef(MAIN_ROBOT_NAME)
if main_robot is None:
    print(f"ERROR: DEF {MAIN_ROBOT_NAME} not found in world.")
    sys.exit(1)

obstacle_robots = []
for name in OBSTACLE_ROBOT_NAMES:
    robot = supervisor.getFromDef(name)
    if robot is not None:
        obstacle_robots.append(robot)
    else:
        print(f"Warning: DEF {name} not found in world.")

def _start_field_viewer():
    try:
        def _is_port_open(port):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.2)
            try:
                return sock.connect_ex(("127.0.0.1", int(port))) == 0
            finally:
                sock.close()

        def _pids_listening_on_port(port):
            pids = set()
            if sys.platform.startswith("win"):
                try:
                    out = subprocess.check_output(["netstat", "-ano", "-p", "tcp"], text=True)
                    token = f":{port}"
                    for raw in out.splitlines():
                        line = raw.strip()
                        if token not in line or "LISTENING" not in line.upper():
                            continue
                        parts = line.split()
                        if len(parts) >= 5 and parts[-1].isdigit():
                            pids.add(int(parts[-1]))
                except Exception:
                    pass
                return pids
            try:
                out = subprocess.check_output(["lsof", "-ti", f":{port}"], text=True)
                for pid in out.split():
                    if pid.strip().isdigit():
                        pids.add(int(pid))
            except Exception:
                pass
            return pids

        def _kill_pid(pid):
            try:
                if sys.platform.startswith("win"):
                    subprocess.run(["taskkill", "/PID", str(pid), "/F"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                else:
                    os.kill(pid, signal.SIGTERM)
            except Exception:
                pass

        try:
            for pid in _pids_listening_on_port(FIELD_VIEWER_PORT):
                _kill_pid(pid)
        except Exception:
            pass

        env = os.environ.copy()
        env.setdefault("PORT", str(FIELD_VIEWER_PORT))
        subprocess.Popen(
            [sys.executable, FIELD_VIEWER_PATH],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Wait briefly for server readiness to provide clear status in logs.
        started = False
        for _ in range(20):
            if _is_port_open(FIELD_VIEWER_PORT):
                started = True
                break
            time.sleep(0.1)

        if not started:
            print(f"可视化启动失败 / Failed to bind field viewer on port {FIELD_VIEWER_PORT}")
            return

        print(f"可视化已启动 / Field viewer running: http://localhost:{env['PORT']}")
        print(f"Front Camera: http://localhost:{env['PORT']}/front_camera")
        print(f"For Simulation Data: http://localhost:{env['PORT']}/simulation_data")
        print(f"For Decisions Data: http://localhost:{env['PORT']}/decisions")
        print(f"For Decision Making Data: http://localhost:{env['PORT']}/decision_making_data")
    except Exception as e:
        print(f"可视化启动失败 / Failed to start field viewer: {e}")

_start_field_viewer()

# =============================================================================
# SHARED HELPERS
# Small math helpers reused by controller logic.
# =============================================================================
def _normalize_angle(a):
    """Normalize angle to [-pi, pi]"""
    return (a + math.pi) % (2 * math.pi) - math.pi

def _deg_to_rad(deg):
    return deg * math.pi / 180.0

# =============================================================================
# MOTION CONTROLLER (NON-BLOCKING)
# Per-robot motion state machine with optional cyclic waypoint traversal.
# =============================================================================
class MotionController:
    """Controls robot motion with support for waypoint cycling"""
    def __init__(self, trans_field, rot_field, dt, cycle_mode=False):
        self.trans = trans_field
        self.rot = rot_field
        self.dt = dt
        self.active = False
        self.phase = None
        self.start_pos = None
        self.target_pos = None
        self.start_angle = 0.0
        self.target_angle = None
        self.velocity = DEFAULT_VELOCITY
        self.angular_speed = _deg_to_rad(
            DEFAULT_ANGULAR_VELOCITY_OBSTACLE if cycle_mode else DEFAULT_ANGULAR_VELOCITY_MAIN
        )
        self.direction = np.array([0.0, 0.0, 0.0])
        self.total_dist = 0.0
        self.traveled = 0.0
        self.total_time = 0.0
        self.elapsed = 0.0
        self.move_speed = 0.0
        self.angle_delta = 0.0
        
        # Cycle mode: automatically move to next waypoint in a list
        self.cycle_mode = cycle_mode
        self.waypoint_list = []
        self.current_waypoint_index = 0

    def start(self, x, y, velocity=None, angle=None):
        """Start a motion task (non-blocking)"""
        cur_pos = np.array(self.trans.getSFVec3f(), dtype=float)
        cur_angle = _normalize_angle(self.rot.getSFRotation()[3])

        target = np.array([x, y, cur_pos[2]], dtype=float)
        delta = target - cur_pos
        dist = np.linalg.norm(delta)

        self.start_pos = cur_pos
        self.target_pos = target
        self.start_angle = cur_angle
        self.target_angle = _normalize_angle(angle) if angle is not None else None
        self.velocity = DEFAULT_VELOCITY if velocity is None else velocity
        self.total_dist = dist
        self.traveled = 0.0
        self.total_time = 0.0
        self.elapsed = 0.0
        self.move_speed = self.velocity
        self.angle_delta = 0.0
        self.angular_speed = _deg_to_rad(
            DEFAULT_ANGULAR_VELOCITY_OBSTACLE if self.cycle_mode else DEFAULT_ANGULAR_VELOCITY_MAIN
        )
        self.direction = (delta / dist) if dist > 1e-9 else np.array([0.0, 0.0, 0.0])

        if dist <= 1e-6:
            if self.target_angle is None:
                self.phase = None
                self.active = False
                return
            else:
                self.phase = 'rotate_only'
                self.active = True
                return

        if self.target_angle is None:
            target_yaw = math.atan2((y - cur_pos[1]), (x - cur_pos[0]))
            self.target_angle = _normalize_angle(target_yaw)
            if self.cycle_mode:
                # Obstacle robots: move while rotating at constant angular speed.
                self.phase = 'move_and_rotate'
                self.angle_delta = _normalize_angle(self.target_angle - self.start_angle)
                time_linear = (self.total_dist / self.velocity) if self.velocity > 1e-9 else 0.0
                time_angular = (abs(self.angle_delta) / self.angular_speed) if self.angular_speed > 1e-9 else 0.0
                self.total_time = max(time_linear, time_angular, 1e-6)
                self.move_speed = self.total_dist / self.total_time if self.total_time > 1e-9 else 0.0
            else:
                # Main robot: rotate in place to face the target, then move in a straight line.
                self.phase = 'rotate_then_move'
        else:
            # Move and interpolate rotation to the target angle simultaneously
            self.phase = 'move_and_rotate'
            self.angle_delta = _normalize_angle(self.target_angle - self.start_angle)
            time_linear = (self.total_dist / self.velocity) if self.velocity > 1e-9 else 0.0
            time_angular = (abs(self.angle_delta) / self.angular_speed) if self.angular_speed > 1e-9 else 0.0
            self.total_time = max(time_linear, time_angular, 1e-6)
            self.move_speed = self.total_dist / self.total_time

        self.active = True

    def _complete_waypoint(self):
        """Mark current waypoint complete and auto-advance in cycle mode."""
        self.active = False
        self.phase = None
        if self.cycle_mode:
            self.start_next_waypoint()

    def cancel(self):
        self.active = False
        self.phase = None

    def load_waypoint_list(self, waypoint_list, start_index=0):
        """Load a list of waypoints for cycle mode
        start_index: The index of the first waypoint to start from
        """
        self.waypoint_list = waypoint_list
        self.current_waypoint_index = start_index % len(waypoint_list) if waypoint_list else 0
        if self.waypoint_list:
            self.start_next_waypoint()

    def start_next_waypoint(self):
        """Move to the next waypoint in the list (cycles if at end)"""
        if not self.waypoint_list:
            self.active = False
            return
        
        wp = self.waypoint_list[self.current_waypoint_index]
        x, y = wp[0], wp[1]
        angle = wp[2] if len(wp) > 2 else None
        
        self.start(x, y, velocity=None, angle=angle)
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoint_list)

    def update(self):
        """Called each frame to advance motion; returns True if the task is completed or idle"""
        if not self.active:
            return True

        cur_pos = np.array(self.trans.getSFVec3f(), dtype=float)
        step_dist = self.velocity * self.dt

        # helper: single-step rotate towards target_ang
        def step_rotate_towards(target_ang):
            cur = _normalize_angle(self.rot.getSFRotation()[3])
            rem = _normalize_angle(target_ang - cur)
            if abs(rem) <= 1e-3:
                self.rot.setSFRotation([0, 0, 1, target_ang])
                return True
            max_step = self.angular_speed * self.dt
            step_ang = max_step if abs(rem) > max_step else abs(rem)
            sign = 1.0 if rem > 0 else -1.0
            self.rot.setSFRotation([0, 0, 1, _normalize_angle(cur + sign * step_ang)])
            return False

        if self.phase == 'rotate_only':
            done = step_rotate_towards(self.target_angle)
            if done:
                self.active = False
                self.phase = None
                # In cycle mode, start next waypoint after reaching this one
                if self.cycle_mode:
                    self.start_next_waypoint()
                return True
            return False

        if self.phase == 'rotate_then_move':
            rotated = step_rotate_towards(self.target_angle)
            if not rotated:
                return False
            remaining = np.linalg.norm(self.target_pos - cur_pos)
            if remaining <= step_dist:
                self.trans.setSFVec3f(self.target_pos.tolist())
                self.active = False
                self.phase = None
                # In cycle mode, start next waypoint after reaching this one
                if self.cycle_mode:
                    self.start_next_waypoint()
                return True
            new_pos = cur_pos + self.direction * step_dist
            self.trans.setSFVec3f(new_pos.tolist())
            return False

        if self.phase == 'move_and_rotate':
            remaining = np.linalg.norm(self.target_pos - cur_pos)
            current_angle = _normalize_angle(self.rot.getSFRotation()[3])
            remaining_angle = abs(_normalize_angle(self.target_angle - current_angle))
            time_linear = (remaining / self.velocity) if self.velocity > 1e-9 else 0.0
            time_angular = (remaining_angle / self.angular_speed) if self.angular_speed > 1e-9 else 0.0
            self.total_time = max(time_linear, time_angular, 1e-6)
            self.move_speed = remaining / self.total_time if self.total_time > 1e-9 else 0.0
            step_dist = self.move_speed * self.dt
            step_rotate_towards(self.target_angle)
            if self.total_time <= self.dt or remaining <= step_dist:
                self.trans.setSFVec3f(self.target_pos.tolist())
                self.rot.setSFRotation([0, 0, 1, self.target_angle])
                self.active = False
                self.phase = None
                # In cycle mode, start next waypoint after reaching this one
                if self.cycle_mode:
                    self.start_next_waypoint()
                return True
            # Position update
            new_pos = cur_pos + self.direction * step_dist
            self.trans.setSFVec3f(new_pos.tolist())
            return False

        return True

# =============================================================================
# BALL PLACEMENT
# Randomized spawn with avoid-zones and overlap mitigation.
# =============================================================================

def _point_in_rect(px, py, rect):
    return (rect[0] <= px <= rect[1]) and (rect[2] <= py <= rect[3])

def randomize_balls(seed=None, ensure_no_overlap=True):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    avoid_zones = []
    margin = ROBOT_HALF_SIZE + CLEARANCE + BALL_RADIUS
    robot_rect = (ROBOT_X - margin, ROBOT_X + margin, ROBOT_Y - margin, ROBOT_Y + margin)
    avoid_zones.append(robot_rect)

    square_half = 0.15
    square_centers = [(-0.9, 0.0), (0.9, 0.0), (0.0, 0.9), (0.0, -0.9)]
    for cx, cy in square_centers:
        avoid_zones.append((cx - square_half, cx + square_half, cy - square_half, cy + square_half))

    placed_positions = []

    for i in range(BALL_COUNT):
        name = f"{BALL_PREFIX}{i}"
        node = supervisor.getFromDef(name)
        if node is None:
            print(f"Warning: {name} not found, skipping.")
            continue
        trans_field = node.getField("translation")

        if not ensure_no_overlap:
            for attempt in range(MAX_TRIES_PER_BALL):
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
                blocked = any(_point_in_rect(x,y,rect) for rect in avoid_zones)
                if not blocked:
                    break
            else:
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
            z = BALL_RADIUS + Z_EPS
            trans_field.setSFVec3f([x,y,z])
            node.resetPhysics()
            placed_positions.append((x,y))
            continue

        success = False
        for attempt in range(MAX_TRIES_PER_BALL):
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)
            if any(_point_in_rect(x,y,rect) for rect in avoid_zones):
                continue
            ok = True
            for (px,py) in placed_positions:
                if (x-px)**2 + (y-py)**2 < (MIN_SEPARATION**2):
                    ok = False
                    break
            if ok:
                z = BALL_RADIUS + Z_EPS
                trans_field.setSFVec3f([x,y,z])
                node.resetPhysics()
                placed_positions.append((x,y))
                success = True
                break
        if not success:
            for attempt in range(MAX_TRIES_PER_BALL):
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
                if any(_point_in_rect(x,y,rect) for rect in avoid_zones):
                    continue
                z = BALL_RADIUS + Z_EPS
                trans_field.setSFVec3f([x,y,z])
                node.resetPhysics()
                placed_positions.append((x,y))
                success = True
                break
        if not success:
            print(f"Warning: failed to place {name} without overlap/avoid zone. Forcing placement.")
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)
            z = BALL_RADIUS + Z_EPS
            trans_field.setSFVec3f([x,y,z])
            node.resetPhysics()
            placed_positions.append((x,y))

    # Let physics settle for a few steps (short blocking)
    for _ in range(SETTLE_STEPS_AFTER_PLACEMENT):
        supervisor.step(TIME_STEP)

    print(f"Randomized {len(placed_positions)} balls. seed={seed}, no_overlap={ensure_no_overlap}")

# =============================================================================
# ABSORPTION MONITORING
# Per-frame rectangular intake checks in robot-local coordinates.
# =============================================================================

def monitor_simple_init(ball_prefix="BALL_", ball_count=40):
    """Initialize ball_simple_state (call once)"""
    ball_simple_state.clear()
    for i in range(ball_count):
        name = f"{ball_prefix}{i}"
        node = supervisor.getFromDef(name)
        if node is None:
            continue
        ball_simple_state[name] = {"absorbed": False}
    # print(f"[monitor_simple] initialized for {len(ball_simple_state)} balls")

def monitor_simple_step(ball_prefix="BALL_", ball_count=40, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION):
    """
    Called each frame: if a ball enters the robot-local rectangular area it is immediately absorbed (teleport + resetPhysics).
    The region is based on the robot's current position (relative to the robot).

    Type detection: make a single attempt to read the proto/name (getProtoName),
    if the proto name contains "steel" it is considered 'steel', otherwise 'ping' by default.
    """
    _monitor_absorption_for_robot(main_robot, ball_prefix, ball_count, half_x, half_y, absorb_location)

def _monitor_absorption_for_robot(robot, ball_prefix="BALL_", ball_count=40, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION):
    """
    Generic absorption check for any robot.
    """
    global PING_STORED, STEEL_STORED, MAIN_BALL_TAKEN
    # Robot pose in world coordinates
    robot_pos = np.array(robot.getField("translation").getSFVec3f(), dtype=float)
    rx, ry = float(robot_pos[0]), float(robot_pos[1])
    robot_rot = np.array(robot.getField("rotation").getSFRotation())
    rangle = float(robot_rot[3])

    for i in range(ball_count):
        name = f"{ball_prefix}{i}"
        # Ensure the ball exists and is in the state table
        if name not in ball_simple_state:
            node_tmp = supervisor.getFromDef(name)
            if node_tmp is None:
                continue
            ball_simple_state[name] = {"absorbed": False}

        if ball_simple_state[name]["absorbed"]:
            continue

        node = supervisor.getFromDef(name)
        if node is None:
            continue

        # Read position
        pos = node.getField("translation").getSFVec3f()
        bx, by = float(pos[0]), float(pos[1])

        # World -> robot coordinate transform (2D, assuming rotation around z axis)
        x_rel = bx - rx
        y_rel = by - ry
        x_ball_robot = x_rel * math.cos(-rangle) - y_rel * math.sin(-rangle)
        y_ball_robot = x_rel * math.sin(-rangle) + y_rel * math.cos(-rangle)

        # Determine type from robotName; default to ping
        name_field = node.getField("robotName").getSFString()
        ball_type = "ping"
        if name_field is not None:
            node_name = name_field.lower()
            if "steel" in node_name:
                ball_type = "steel"
            else:
                ball_type = "ping"

        # Decide absorption based on type
        absorbed = False
        if ball_type == "ping":
            if (x_ball_robot > 0) and (x_ball_robot < half_x) and (abs(y_ball_robot) < half_y):
                absorbed = True
                PING_STORED += 1
        elif ball_type == "steel":
            if (x_ball_robot > 0) and (x_ball_robot < half_x - 0.01) and (abs(y_ball_robot) < half_y):
                absorbed = True
                STEEL_STORED += 1

        if absorbed:
            # Absorb by teleporting outside arena and resetting physics
            node.getField("translation").setSFVec3f([absorb_location[0], absorb_location[1], absorb_location[2]])
            try:
                node.resetPhysics()
            except Exception:
                pass
            ball_simple_state[name]["absorbed"] = True
            if robot == main_robot:
                MAIN_BALL_TAKEN += 1
                _append_ball_taken_history(BALL_TAKEN_HISTORY_FILE, supervisor.getTime(), MAIN_BALL_TAKEN)
            SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1
            # print(f"Score: {SCORE} | Ping Hit: {PING_HIT} | Steel Hit: {STEEL_HIT} | Steel Stored: {STEEL_STORED} | Ping Stored: {PING_STORED}")

# =============================================================================
# BOOTSTRAP
# Initial randomization and startup file reset.
# =============================================================================
randomize_balls(seed=RANDOM_SEED, ensure_no_overlap=True)
monitor_simple_init(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT)

# Clear all runtime txt files at startup for both decision-making variants.
for _dir in (
    os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data"),
    os.path.join(PROJECT_ROOT, "decision_making_wly", "real_time_data"),
    os.path.join(PROJECT_ROOT, "decision_making_ros", "real_time_data"),
):
    try:
        for name in os.listdir(_dir):
            if not name.endswith(".txt"):
                continue
            path = os.path.join(_dir, name)
            if os.path.isfile(path):
                with open(path, "w") as f:
                    f.write("")
    except FileNotFoundError:
        pass
    except Exception as e:
        try:
            print(f"[startup] failed to clear txt contents in {_dir}: {e}")
        except Exception:
            pass

# Reset dynamic waypoint/speed/taken-ball files at startup (disabled; web-only now).

# Initialize main robot ball-taken history file.
try:
    with open(BALL_TAKEN_HISTORY_FILE, "w") as f:
        f.write("")
except Exception as e:
    try:
        print(f"[startup] failed to set ball_taken_history.txt: {e}")
    except Exception:
        pass

def _load_dynamic_waypoint():
    # -------------------------------------------------------------------------
    # WAYPOINT PARSING HELPERS
    # Parse dynamic waypoint input from shared txt file.
    # -------------------------------------------------------------------------
    """Load a single waypoint from dynamic_waypoints.txt (read-only).
    Returns a single (x, y, orientation) tuple or None if file is empty/invalid.
    """
    namespace = {"North": 90.0, "East": 0.0, "South": -90.0, "West": 180.0, "None": None}
    raw_payload = _get_decision_value("dynamic_waypoints")
    if not raw_payload:
        pos = main_robot.getField("translation").getSFVec3f()
        x, y = float(pos[0]), float(pos[1])
        rot = main_robot.getField("rotation").getSFRotation()
        ang = float(rot[3])
        return (x, y, ang)
    for raw in raw_payload.splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        # remove trailing comma
        if line.endswith(','):
            line = line[:-1].strip()
        try:
            # evaluate in a restricted namespace
            wp = eval(line, {"__builtins__": None}, namespace)
        except Exception:
            # fallback: extract numbers
            nums = re.findall(r'[-+]?[0-9]*\.?[0-9]+', line)
            if len(nums) >= 2:
                x = float(nums[0]); y = float(nums[1])
                ang = float(nums[2]) if len(nums) >= 3 else None
                wp = (x, y, ang)
            else:
                continue
        # normalize
        if isinstance(wp, tuple) and len(wp) >= 2:
            if len(wp) == 2:
                wp = (float(wp[0]), float(wp[1]), None)
            else:
                ang = wp[2]
                if ang is None:
                    ang = None
                else:
                    ang = math.radians(float(ang))
                wp = (float(wp[0]), float(wp[1]), ang)
            return wp
    return None

def _append_to_history(path, waypoint, status, timestamp=None):
    """Append a waypoint record to waypoints_history.txt with status ('reached' or 'cut').
    status: 'reached' or 'cut'
    """
    if timestamp is None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    
    x, y, ang = waypoint
    ang_s = 'None' if ang is None else f"{math.degrees(ang):.2f}"
    record = f"({x}, {y}, {ang_s}),  {status}\n"
    
    # Always append to file
    with open(path, 'a') as f:
        f.write(record)
    _trim_history_file(path, max_lines=2000)

def _trim_history_file(path, max_lines=2000):
    """Keep only the last max_lines in the history file (rolling)."""
    try:
        with open(path, 'r') as f:
            lines = f.readlines()
        if len(lines) <= max_lines:
            return
        with open(path, 'w') as f:
            f.writelines(lines[-max_lines:])
    except Exception:
        pass

def _load_waypoint_list_from_file(path):
    """Load multiple waypoints from a file (for obstacle robot cycling).
    Returns a list of (x, y, angle) tuples, or empty list if file doesn't exist.
    """
    namespace = {"North": North, "East": East, "South": South, "West": West, "None": None}
    waypoints = []
    try:
        with open(path, 'r') as f:
            for raw in f:
                line = raw.split('#', 1)[0].strip()
                if not line:
                    continue
                if line.endswith(','):
                    line = line[:-1].strip()
                try:
                    wp = eval(line, {"__builtins__": None}, namespace)
                except Exception:
                    nums = re.findall(r'[-+]?[0-9]*\.?[0-9]+', line)
                    if len(nums) >= 2:
                        x = float(nums[0]); y = float(nums[1])
                        ang = float(nums[2]) if len(nums) >= 3 else None
                        wp = (x, y, ang)
                    else:
                        continue
                
                if isinstance(wp, tuple) and len(wp) >= 2:
                    if len(wp) == 2:
                        wp = (float(wp[0]), float(wp[1]), None)
                    else:
                        ang = wp[2]
                        ang = float(ang) if ang is not None else None
                        wp = (float(wp[0]), float(wp[1]), ang)
                    waypoints.append(wp)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"[load_waypoint_list] Error loading {path}: {e}")
    
    return waypoints

def _append_history_header(path):
    """Append a new session header to waypoints_history.txt with current timestamp.
    Called once at program startup.
    """
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    header = f"\n# Waypoint History - Started at {timestamp} - Random Seed {RANDOM_SEED}\n"
    with open(path, 'a') as f:
        f.write(header)
    _trim_history_file(path, max_lines=2000)

def _format_ball_positions():
    lines = []
    written_count = 0
    for i in range(BALL_COUNT):
        name = f"{BALL_PREFIX}{i}"
        node = supervisor.getFromDef(name)
        if node is None:
            continue
        try:
            pos = node.getField("translation").getSFVec3f()
            x, y = float(pos[0]), float(pos[1])
        except Exception:
            continue
        if x < -1.0 or x > 1.0 or y < -1.0 or y > 1.0:
            continue
        typ = "ping"
        try:
            name_field = node.getField("robotName").getSFString()
            if name_field is not None and "steel" in name_field.lower():
                typ = "metal"
        except Exception:
            pass
        lines.append(f"({x:.6f}, {y:.6f}, {typ.upper()})")
        written_count += 1
    return "\n".join(lines), written_count


def _format_current_position():
    pos = main_robot.getField("translation").getSFVec3f()
    x, y = float(pos[0]), float(pos[1])
    rot = main_robot.getField("rotation").getSFRotation()
    bearing_rad = float(rot[3])
    bearing_deg = math.degrees(bearing_rad)
    return f"({x:.6f}, {y:.6f}, {bearing_deg:.2f})"

def _format_obstacle_positions():
    lines = []
    for robot in obstacle_robots:
        pos = robot.getField("translation").getSFVec3f()
        x, y = float(pos[0]), float(pos[1])
        rot = robot.getField("rotation").getSFRotation()
        bearing_deg = math.degrees(float(rot[3]))
        lines.append(f"({x:.6f}, {y:.6f}, {bearing_deg:.2f})")
    return "\n".join(lines)

def _format_webots_time():
    return f"{supervisor.getTime():.6f}"

def _radar_sensor_distances(max_range=RADAR_MAX_RANGE, corridor=RADAR_CORRIDOR):
    """Return nearest radar distances by direction using world geometry samples."""
    pos = main_robot.getField("translation").getSFVec3f()
    rot = main_robot.getField("rotation").getSFRotation()
    cx, cy = float(pos[0]), float(pos[1])
    bearing_deg = math.degrees(float(rot[3]))

    half_band = corridor / 2.0
    theta = math.radians(bearing_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    robot_half = 0.1
    obstacle_half = 0.1

    sample_points_world = []

    sample_spacing = 0.1 * obstacle_half
    obstacle_edge_samples_local = []
    num_samples = int(2 * obstacle_half / sample_spacing) + 1
    for i in range(num_samples):
        offset = -obstacle_half + i * sample_spacing
        obstacle_edge_samples_local.append((offset, obstacle_half))
        obstacle_edge_samples_local.append((offset, -obstacle_half))
        obstacle_edge_samples_local.append((-obstacle_half, offset))
        obstacle_edge_samples_local.append((obstacle_half, offset))

    for robot in obstacle_robots:
        pos_o = robot.getField("translation").getSFVec3f()
        ox, oy = float(pos_o[0]), float(pos_o[1])
        rot_o = robot.getField("rotation").getSFRotation()
        otheta = float(rot_o[3])
        cos_o = math.cos(otheta)
        sin_o = math.sin(otheta)
        for lx, ly in obstacle_edge_samples_local:
            wx = ox + lx * cos_o - ly * sin_o
            wy = oy + lx * sin_o + ly * cos_o
            sample_points_world.append((wx, wy))

    edge_samples = [i * 0.05 for i in range(-20, 21)]
    wall_samples = (
        [(x, 1.0) for x in edge_samples]
        + [(x, -1.0) for x in edge_samples]
        + [(1.0, y) for y in edge_samples]
        + [(-1.0, y) for y in edge_samples]
    )
    sample_points_world.extend(wall_samples)

    hits = {}
    for wx, wy in sample_points_world:
        dx = wx - cx
        dy = wy - cy
        x_robot = dx * cos_t + dy * sin_t
        y_robot = -dx * sin_t + dy * cos_t

        if x_robot > 0 and abs(y_robot) <= half_band and x_robot <= max_range:
            dist = x_robot - robot_half
            direction = "front"
        elif x_robot < 0 and abs(y_robot) <= half_band and -x_robot <= max_range:
            dist = -x_robot - robot_half
            direction = "rear"
        elif y_robot > 0 and abs(x_robot) <= half_band and y_robot <= max_range:
            dist = y_robot - robot_half
            direction = "left"
        elif y_robot < 0 and abs(x_robot) <= half_band and -y_robot <= max_range:
            dist = -y_robot - robot_half
            direction = "right"
        else:
            continue

        if dist <= max_range:
            dist = max(0.0, dist)
            prev = hits.get(direction)
            if prev is None or dist < prev:
                hits[direction] = dist

    memory_values = {
        "front": max_range,
        "right": max_range,
        "left": max_range,
        "rear": max_range,
    }
    for direction, dist in hits.items():
        if direction in memory_values:
            memory_values[direction] = max(0.0, dist)
    return memory_values

def _read_speed_mps(default_value):
    """Read cruise speed in m/s from decisions data (web-only)."""
    raw = _get_decision_value("speed")
    if raw is None:
        return default_value
    try:
        value = float(raw)
    except Exception:
        return default_value
    if value <= 0:
        return default_value
    return value




def _post_sim_data(payload):
    if DATA_FLOW == "file":
        for key, value in payload.items():
            path = os.path.join(REAL_TIME_DATA_DIR, f"{key}.txt")
            _write_local_text(path, f"{value}\n")
        return
    try:
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            SIM_DATA_ENDPOINT,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
        )
        with urllib.request.urlopen(req, timeout=0.3) as res:
            res.read()
    except Exception:
        pass

def _refresh_decisions_data():
    global DECISIONS_CACHE, DECISIONS_SEQ
    if DATA_FLOW == "file":
        payload = {}
        for key in ("dynamic_waypoints", "speed"):
            path = os.path.join(DECISION_REAL_TIME_DIR, f"{key}.txt")
            value = _read_local_text(path)
            if value is not None:
                payload[key] = value
        DECISIONS_CACHE = payload
        DECISIONS_SEQ += 1
        return True
    if RUN_ON_PI:
        deadline = time.time() + DECISIONS_RETRY_SECONDS_ON_PI
        last_error = None
        while True:
            try:
                with urllib.request.urlopen(DECISIONS_ENDPOINT, timeout=0.3) as res:
                    payload = json.loads(res.read().decode("utf-8"))
                if isinstance(payload, dict):
                    DECISIONS_CACHE = payload
                    DECISIONS_SEQ += 1
                    return True
                last_error = RuntimeError(
                    f"Invalid decisions payload type from {DECISIONS_ENDPOINT}. "
                    f"Expected JSON object, got {type(payload).__name__}."
                )
            except Exception as e:
                last_error = e

            if time.time() >= deadline:
                break
            time.sleep(DECISIONS_RETRY_SLEEP_SECONDS_ON_PI)

        print(
            f"[Supervisor][warning] Failed to read decisions data from {DECISIONS_ENDPOINT} "
            f"after retrying for {DECISIONS_RETRY_SECONDS_ON_PI:.1f}s. "
            f"Last error: {last_error}. Continue without stopping supervisor."
        )
        return False

    try:
        with urllib.request.urlopen(DECISIONS_ENDPOINT, timeout=0.3) as res:
            payload = json.loads(res.read().decode("utf-8"))
        if isinstance(payload, dict):
            DECISIONS_CACHE = payload
            DECISIONS_SEQ += 1
            return True
    except Exception:
        pass
    return False

def _require_decision_value(key, allow_empty=False, retries=1, sleep_s=0.02):
    for attempt in range(retries + 1):
        _refresh_decisions_data()
        value = DECISIONS_CACHE.get(key)
        if value is not None:
            raw = str(value).strip()
            if allow_empty or raw:
                return raw
        if attempt < retries:
            time.sleep(sleep_s)
    raise RuntimeError(f"Missing decisions data for '{key}' from {DECISIONS_ENDPOINT}.")

def _get_decision_value(key):
    return DECISIONS_CACHE.get(key)

def _append_ball_taken_history(path, taken_time_s, taken_count):
    """Append main-robot ball taken history as: <time_seconds>,<cumulative_count>."""
    try:
        t = float(taken_time_s)
        c = int(taken_count)
        with open(path, 'a') as f:
            f.write(f"{t:.3f},{c}\n")
    except Exception as e:
        try:
            print(f"[ball_taken_history] failed to write: {e}")
        except Exception:
            pass

def _write_supervisor_status(path, status):
    """Write supervisor controller runtime status to file — overwrite atomically."""
    try:
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(f"{status}\n")
        os.replace(tmp, path)
    except Exception as e:
        try:
            print(f"[supervisor_status] failed to write: {e}")
        except Exception:
            pass

def _format_visible_balls(viewfield_deg=FIELD_OF_VIEW_DEGREES, visible_range_m=VISIBLE_RANGE_METERS):
    lines = []
    pos = main_robot.getField("translation").getSFVec3f()
    rx, ry = float(pos[0]), float(pos[1])
    rot = main_robot.getField("rotation").getSFRotation()
    rangle = float(rot[3])
    half_fov = math.radians(viewfield_deg) / 2.0
    range_sq = visible_range_m * visible_range_m

    def _segment_intersects_aabb(p0, p1, half):
        x0, y0 = p0
        x1, y1 = p1
        dx = x1 - x0
        dy = y1 - y0
        t0, t1 = 0.0, 1.0
        for p, q in ((-dx, x0 + half), (dx, half - x0), (-dy, y0 + half), (dy, half - y0)):
            if abs(p) < 1e-12:
                if q < 0:
                    return False
            else:
                t = q / p
                if p < 0:
                    if t > t1:
                        return False
                    if t > t0:
                        t0 = t
                else:
                    if t < t0:
                        return False
                    if t < t1:
                        t1 = t
        return True

    def _is_occluded_by_obstacles(ball_x, ball_y):
        for robot in obstacle_robots:
            opos = robot.getField("translation").getSFVec3f()
            ox, oy = float(opos[0]), float(opos[1])
            orot = robot.getField("rotation").getSFRotation()
            oangle = float(orot[3])

            def to_local(px, py):
                dx = px - ox
                dy = py - oy
                lx = dx * math.cos(-oangle) - dy * math.sin(-oangle)
                ly = dx * math.sin(-oangle) + dy * math.cos(-oangle)
                return lx, ly

            p0 = to_local(rx, ry)
            p1 = to_local(ball_x, ball_y)
            if _segment_intersects_aabb(p0, p1, 0.1):
                return True
        return False

    for i in range(BALL_COUNT):
        name = f"{BALL_PREFIX}{i}"
        node = supervisor.getFromDef(name)
        if node is None:
            continue
        try:
            bpos = node.getField("translation").getSFVec3f()
            bx, by = float(bpos[0]), float(bpos[1])
        except Exception:
            continue
        if bx < -1.0 or bx > 1.0 or by < -1.0 or by > 1.0:
            continue

        x_rel = bx - rx
        y_rel = by - ry
        x_robot = x_rel * math.cos(-rangle) - y_rel * math.sin(-rangle)
        y_robot = x_rel * math.sin(-rangle) + y_rel * math.cos(-rangle)
        dist_sq = x_robot * x_robot + y_robot * y_robot
        if dist_sq > range_sq:
            continue

        angle = math.atan2(y_robot, x_robot)
        if abs(angle) > half_fov:
            continue

        if _is_occluded_by_obstacles(bx, by):
            continue

        typ = "PING"
        try:
            name_field = node.getField("robotName").getSFString()
            if name_field is not None and "steel" in name_field.lower():
                typ = "METAL"
        except Exception:
            pass

        lines.append(f"({bx:.6f}, {by:.6f}, {typ})")

    return "\n".join(lines)

# =============================================================================
# MOTION CONTROLLER INITIALIZATION
# Build controller instances for main robot and obstacle robots.
# =============================================================================
# Main robot: single waypoint navigation from dynamic_waypoints.txt
main_trans = main_robot.getField("translation")
main_rot = main_robot.getField("rotation")
main_motion = MotionController(main_trans, main_rot, dt, cycle_mode=False)

# Obstacle robots: cyclic waypoint navigation from obstacle_plan.txt (optional)
obstacle_motions = []
obstacle_waypoints = _load_waypoint_list_from_file(OBSTACLE_PLAN_FILE)

if obstacle_waypoints:
    print(f"Loaded {len(obstacle_waypoints)} waypoints from {OBSTACLE_PLAN_FILE}")
    for i, robot in enumerate(obstacle_robots):
        obs_trans = robot.getField("translation")
        obs_rot = robot.getField("rotation")
        obs_motion = MotionController(obs_trans, obs_rot, dt, cycle_mode=True)
        
        # Load waypoints with different starting index for each robot
        start_idx = OBSTACLE_START_INDICES[i] if i < len(OBSTACLE_START_INDICES) else i * 2
        obs_motion.load_waypoint_list(obstacle_waypoints, start_index=start_idx)
        obstacle_motions.append(obs_motion)
        print(f"Obstacle robot {i+1} ({OBSTACLE_ROBOT_NAMES[i]}) starting at waypoint index {start_idx}")
else:
    print(f"WARNING: No waypoints loaded from {OBSTACLE_PLAN_FILE}")

# Initialize waypoints history with new session header
_append_history_header(WAYPOINTS_HISTORY_FILE)

# Prime decisions cache before first reads.
_refresh_decisions_data()

# Load initial dynamic waypoint for main robot
current_waypoint = _load_dynamic_waypoint()
last_dynamic_waypoint = current_waypoint
last_dynamic_waypoint_raw = _get_decision_value("dynamic_waypoints") or ""

if current_waypoint is not None:
    x, y, ang = current_waypoint
    main_motion.start(x, y, velocity=None, angle=ang)

# Note: Camera display is handled automatically by Webots
# To view the camera feed: right-click on 'front_camera' in Scene Tree and select 'View'

frame_counter = 0
decisions_frame_counter = 0
ball_taken_180_logged = False

# Mark supervisor status as running at initiation.
_write_supervisor_status(SUPERVISOR_STATUS_FILE, "runnung")

# =============================================================================
# MAIN NON-BLOCKING LOOP
# Drive motion, absorb balls, sync files, and run cruise script periodically.
# =============================================================================
while supervisor.step(TIME_STEP) != -1:
    sim_time = supervisor.getTime()

    decisions_frame_counter += 1
    if decisions_frame_counter % DECISIONS_REFRESH_EVERY_FRAMES == 0:
        _refresh_decisions_data()

    if (sim_time > 180.0) and (not ball_taken_180_logged):
        _append_ball_taken_history(BALL_TAKEN_HISTORY_FILE, 180.0, MAIN_BALL_TAKEN)
        ball_taken_180_logged = True

    if sim_time > 185.0:
        print("[Supervisor] Simulation time exceeded 185s. Stopping simulation.")
        _write_supervisor_status(SUPERVISOR_STATUS_FILE, "exited")
        break

    # ==== Update main robot (single waypoint navigation) ====
    # Check if dynamic_waypoints has changed (web-only)
    raw_dynamic_waypoint = _get_decision_value("dynamic_waypoints") or ""
    if raw_dynamic_waypoint != last_dynamic_waypoint_raw:
        new_waypoint = _load_dynamic_waypoint()
        if new_waypoint is not None and new_waypoint != last_dynamic_waypoint:
            # Waypoint changed: record the old one as 'cut' and start new one
            if last_dynamic_waypoint is not None and current_waypoint is not None:
                _append_to_history(WAYPOINTS_HISTORY_FILE, last_dynamic_waypoint, "cut")
            current_waypoint = new_waypoint
            last_dynamic_waypoint = new_waypoint
            x, y, ang = current_waypoint
            main_motion.start(x, y, velocity=None, angle=ang)
        last_dynamic_waypoint_raw = raw_dynamic_waypoint

    # 0.5) Update main robot cruise speed from speed.txt
    main_motion.velocity = _read_speed_mps(DEFAULT_VELOCITY)
    
    # 1) Advance main robot motion (non-blocking)
    main_motion.update()
    
    # 1.2) Advance all obstacle robots motion
    for obs_motion in obstacle_motions:
        obs_motion.update()
    
    # 1.5) Update real-time status (in-memory only for sim data)
    waypoint_status = "going" if main_motion.active else "reached"
    
    # 1.6) Execute cruise script at intervals
    frame_counter += 1
    if camera is not None and (front_camera_endpoint is not None or DATA_FLOW == "file"):
        if frame_counter % post_interval_frames == 0:
            _post_front_camera_frame(
                camera,
                endpoint=front_camera_endpoint,
                image_path=camera_local_image_path,
                write_local=(DATA_FLOW == "file"),
            )
            if POSE_ESTIMATION_ON and (not RUN_ON_PI):
                _trigger_pose_estimation_if_ready()
                _print_ground_truth_current_position(main_robot)
    if (not RUN_ON_PI) and frame_counter % CRUISE_INTERVAL_FRAMES == 0:
        try:
            subprocess.run([sys.executable, CRUISE_SCRIPT_PATH], check=False)
        except Exception as e:
            print(f"[Cruise] Error running waypoints_cruise.py: {e}")

    # 2) Call absorption check for main robot
    monitor_simple_step(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION)

    # 2.1) Call absorption check for all obstacle robots
    for i, obs_robot in enumerate(obstacle_robots):
        # Different absorb locations for each obstacle robot to avoid overlap
        absorb_x = 1.1 + i * 0.1
        _monitor_absorption_for_robot(obs_robot, ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=(absorb_x, 0.0, 0.1))

    # 2.5) Collect per-frame ball positions (in-memory only for sim data)
    ball_positions_text, written_ball_count = _format_ball_positions()
    if written_ball_count == 0:
        # print("[Supervisor] ball_position.txt is empty. Stopping simulation.")
        _write_supervisor_status(SUPERVISOR_STATUS_FILE, "exited")
        # break
    
    # 2.6) Collect sim data values (in-memory only for sim data)
    current_position_text = _format_current_position()
    obstacle_positions_text = _format_obstacle_positions()
    webots_time_text = _format_webots_time()
    visible_balls_text = _format_visible_balls()
    ball_taken_text = f"{int(MAIN_BALL_TAKEN)}"
    radar_values = _radar_sensor_distances()
    radar_sensor_text = (
        f"{sim_time:.3f},{radar_values['front']:.6f},{radar_values['right']:.6f},"
        f"{radar_values['left']:.6f},{radar_values['rear']:.6f}"
    )

    # 2.95) Push simulation data to field viewer server
    payload = {
        "ball_position": ball_positions_text,
        "current_position": current_position_text,
        "obstacle_robot": obstacle_positions_text,
        "time": webots_time_text,
        "visible_balls": visible_balls_text,
        "ball_taken_number": ball_taken_text,
        "waypoint_status": waypoint_status,
        "radar_sensor": radar_sensor_text,
    }
    _post_sim_data(payload)

    # 3) If main robot motion completed, record as 'reached' and wait for next dynamic waypoint
    if not main_motion.active and current_waypoint is not None:
        _append_to_history(WAYPOINTS_HISTORY_FILE, current_waypoint, "reached")
        current_waypoint = None

    # 4) Other per-frame logic can be added here (logging / keyboard / debug)
    SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1

# Exit
_write_supervisor_status(SUPERVISOR_STATUS_FILE, "exited")
print("Supervisor controller exiting.")