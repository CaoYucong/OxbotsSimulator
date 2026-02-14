# supervisor_controller.py
# Non-blocking Supervisor controller (includes random ball placement + simplified absorption logic)
from controller import Supervisor
import math
import numpy as np
import os
import random
import sys
import subprocess
import signal

# ==== Initialization ====
RANDOM_SEED = 1236
DEFAULT_VELOCITY = 0.3 # m/s
DEFAULT_ANGULAR_VELOCITY_MAIN = 90  # deg/s (equiv to 2 rad/s)
DEFAULT_ANGULAR_VELOCITY_OBSTACLE = 90  # deg/s (equiv to 2 rad/s)
supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())
dt = TIME_STEP / 1000.0  # seconds

SCORE = 0
STEEL_STORED = 0
PING_STORED = 0
STEEL_HIT = 0
PING_HIT = 0

# Get robot references
MAIN_ROBOT_NAME = "MY_ROBOT"
OBSTACLE_ROBOT_NAMES = ["OBSTACLE_ROBOT_1", "OBSTACLE_ROBOT_2", "OBSTACLE_ROBOT_3"]
OBSTACLE_START_INDICES = [1, 50, 99]  # Starting waypoint indices for each obstacle robot

main_robot = supervisor.getFromDef(MAIN_ROBOT_NAME)
if main_robot is None:
    print(f"ERROR: DEF {MAIN_ROBOT_NAME} not found in world.")
    sys.exit(1)

# Get all obstacle robots
obstacle_robots = []
for name in OBSTACLE_ROBOT_NAMES:
    robot = supervisor.getFromDef(name)
    if robot is not None:
        obstacle_robots.append(robot)
    else:
        print(f"Warning: DEF {name} not found in world.")

# ==== Field viewer (init) ====
FIELD_VIEWER_PORT = 5001
FIELD_VIEWER_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "tools", "field_viewer", "server.py")
)

def _start_field_viewer():
    try:
        try:
            out = subprocess.check_output(["lsof", "-ti", f":{FIELD_VIEWER_PORT}"], text=True)
            pids = [int(pid) for pid in out.split() if pid.strip()]
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                except Exception:
                    pass
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
        print(f"可视化已启动 / Field viewer running: http://localhost:{env['PORT']}")
    except Exception as e:
        print(f"可视化启动失败 / Failed to start field viewer: {e}")

_start_field_viewer()

# ==== Utility functions ====
def _normalize_angle(a):
    """Normalize angle to [-pi, pi]"""
    return (a + math.pi) % (2 * math.pi) - math.pi

def _deg_to_rad(deg):
    return deg * math.pi / 180.0

# ==== Non-blocking MotionController ====
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
        self.velocity = 0.3
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

    def _complete_waypoint(self):
        """Handle waypoint completion with cycle mode support"""
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

# ==== Randomize balls (start) ====
BALL_PREFIX = "BALL_"
BALL_COUNT = 40
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86
BALL_RADIUS = 0.02
MIN_SEPARATION = 2.0 * BALL_RADIUS + 0.001
MAX_TRIES_PER_BALL = 2000
SETTLE_STEPS_AFTER_PLACEMENT = max(1, int(0.2 / dt))
Z_EPS = 0.001

# Robot occupancy parameters (avoid when placing balls at start)
ROBOT_X = -0.8
ROBOT_Y = 0.0
ROBOT_HALF_SIZE = 0.1
CLEARANCE = 0.05

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

# ==== Simplified absorption monitoring (per-frame simple box check) ====
ABSORB_BOX_HALF_X = 0.12   # robot-local x half-width
ABSORB_BOX_HALF_Y = 0.05   # robot-local y half-width
ABSORB_LOCATION = (-1.1, 0.0, 0.4)

# Only record whether absorbed to avoid duplicate absorption
ball_simple_state = {}  # name -> {"absorbed": bool}

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
    global PING_STORED, STEEL_STORED
    # Read robot current position (world coordinates)
    robot_pos = np.array(robot.getField("translation").getSFVec3f(), dtype=float)
    rx, ry = float(robot_pos[0]), float(robot_pos[1])
    # Use the original rotation handling (assumes rotation around z axis based on angle)
    robor_rot = np.array(robot.getField("rotation").getSFRotation())
    rangle = float(robor_rot[3])

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

        # Single attempt: use Robot.name to determine type, default 'ping'
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
            # Per your logic: x_ball_robot > 0 and (x_ball_robot - 0.2 < half_x)
            if (x_ball_robot > 0) and (x_ball_robot < half_x - 0.01) and (abs(y_ball_robot) < half_y):
                absorbed = True
                STEEL_STORED += 1

        if absorbed:
            # Absorption action: teleport and resetPhysics
            node.getField("translation").setSFVec3f([absorb_location[0], absorb_location[1], absorb_location[2]])
            try:
                node.resetPhysics()
            except Exception:
                pass
            ball_simple_state[name]["absorbed"] = True
            SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1
            print(f"Score: {SCORE} | Ping Hit: {PING_HIT} | Steel Hit: {STEEL_HIT} | Steel Stored: {STEEL_STORED} | Ping Stored: {PING_STORED}")

import random, math, numpy as np

# ==== Main logic (randomize balls -> init monitoring -> waypoint queue -> non-blocking main loop) ====
randomize_balls(seed=RANDOM_SEED, ensure_no_overlap=True)
monitor_simple_init(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT)

# waypoints are loaded from the external file controllers/supervisor_controller/dynamic_waypoints.txt
import os, re, time

# waypoint orientation aliases (radians)
North = math.pi / 2
East = 0.0
South = -math.pi / 2
West = math.pi

REAL_TIME_DATA_DIR = os.path.join(os.path.dirname(__file__), "real_time_data")
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WHO_IS_DEV_FILE = os.path.join(PROJECT_ROOT, "who_is_developing.txt")

def _resolve_decision_making_dir():
    """Select decision_making folder based on who_is_developing.txt (cyc/wly)."""
    default_dir = os.path.join(PROJECT_ROOT, "decision_making")
    try:
        with open(WHO_IS_DEV_FILE, "r") as f:
            dev = f.read().strip().lower()
        if dev == "cyc":
            return os.path.join(PROJECT_ROOT, "decision_making_cyc")
        if dev == "wly":
            return os.path.join(PROJECT_ROOT, "decision_making_wly")
    except Exception:
        pass
    return default_dir

DECISION_MAKING_DIR = _resolve_decision_making_dir()
DECISION_REAL_TIME_DIR = os.path.join(DECISION_MAKING_DIR, "real_time_data")
DYNAMIC_WAYPOINTS_FILE = os.path.join(REAL_TIME_DATA_DIR, "dynamic_waypoints.txt")
WAYPOINTS_HISTORY_FILE = os.path.join(REAL_TIME_DATA_DIR, "waypoints_history.txt")
WAYPOINT_STATUS_FILE = os.path.join(REAL_TIME_DATA_DIR, "waypoint_status.txt")
BALL_POS_FILE = os.path.join(REAL_TIME_DATA_DIR, "ball_position.txt")
CURRENT_POSITION_FILE = os.path.join(REAL_TIME_DATA_DIR, "current_position.txt")
OBSTACLE_ROBOT_FILE = os.path.join(REAL_TIME_DATA_DIR, "obstacle_robot.txt")
TIME_FILE = os.path.join(REAL_TIME_DATA_DIR, "time.txt")
OBSTACLE_PLAN_FILE = os.path.join(REAL_TIME_DATA_DIR, "obstacle_plan.txt")
SPEED_FILE = os.path.join(REAL_TIME_DATA_DIR, "speed.txt")
VISIBLE_BALLS_FILE = os.path.join(REAL_TIME_DATA_DIR, "visible_balls.txt")

# Clear contents of all .txt files in decision_making_cyc/real_time_data
# and decision_making_wly/real_time_data at startup
for _dir in (
    os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data"),
    os.path.join(PROJECT_ROOT, "decision_making_wly", "real_time_data"),
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

# Clear dynamic waypoints at startup before any other processing
try:
    with open(DYNAMIC_WAYPOINTS_FILE, "w") as f:
        f.write("")
except Exception as e:
    try:
        print(f"[startup] failed to clear dynamic_waypoints.txt: {e}")
    except Exception:
        pass

# Initialize speed.txt with DEFAULT_VELOCITY at startup
try:
    with open(SPEED_FILE, "w") as f:
        f.write(f"{DEFAULT_VELOCITY}\n")
except Exception as e:
    try:
        print(f"[startup] failed to set speed.txt: {e}")
    except Exception:
        pass

def _load_dynamic_waypoint(path):
    """Load a single waypoint from dynamic_waypoints.txt (read-only).
    Returns a single (x, y, orientation) tuple or None if file is empty/invalid.
    """
    namespace = {"North": 90.0, "East": 0.0, "South": -90.0, "West": 180.0, "None": None}
    try:
        with open(path, 'r') as f:
            for raw in f:
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
    except Exception:
        pass
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

def _update_status_file(path, motion_active, has_waypoint):
    """Update real-time status file with current robot state.
    status: 'going' if motion is active, 'reached' otherwise
    """
    status = "going" if motion_active else "reached"
    with open(path, 'w') as f:
        f.write(status)


def _write_ball_positions(path):
    """Write current positions of all balls into `path`.
    Format: one line per ball: (x, y, ping/metal) — overwrite atomically.
    Only records balls with x >= -1.0
    """
    try:
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
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
                # Skip balls with x < -1.0
                if x < -1.0 or x > 1.0 or y < -1.0 or y > 1.0:
                    continue
                # Determine type: map 'steel' -> 'metal', else 'ping'
                typ = 'ping'
                try:
                    name_field = node.getField("robotName").getSFString()
                    if name_field is not None:
                        if 'steel' in name_field.lower():
                            typ = 'metal'
                except Exception:
                    pass
                f.write(f"({x:.6f}, {y:.6f}, {typ.upper()})\n")
        os.replace(tmp, path)
    except Exception as e:
        # non-fatal: print to stdout for debugging
        try:
            print(f"[ball_positions] failed to write positions: {e}")
        except Exception:
            pass


def _write_current_position(path):
    """Write main robot's current position (x, y, bearing_deg) to file — overwrite atomically."""
    try:
        tmp = path + '.tmp'
        pos = main_robot.getField("translation").getSFVec3f()
        x, y = float(pos[0]), float(pos[1])
        rot = main_robot.getField("rotation").getSFRotation()
        bearing_rad = float(rot[3])
        bearing_deg = math.degrees(bearing_rad)
        with open(tmp, 'w') as f:
            f.write(f"({x:.6f}, {y:.6f}, {bearing_deg:.2f})\n")
        os.replace(tmp, path)
    except Exception as e:
        # non-fatal
        try:
            print(f"[current_position] failed to write: {e}")
        except Exception:
            pass

def _write_obstacle_positions(path):
    """Write all obstacle robots' positions (x, y) to file — overwrite atomically."""
    try:
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            for robot in obstacle_robots:
                pos = robot.getField("translation").getSFVec3f()
                x, y = float(pos[0]), float(pos[1])
                rot = robot.getField("rotation").getSFRotation()
                bearing_deg = math.degrees(float(rot[3]))
                f.write(f"({x:.6f}, {y:.6f}, {bearing_deg:.2f})\n")
        os.replace(tmp, path)
    except Exception as e:
        # non-fatal
        try:
            print(f"[obstacle_positions] failed to write: {e}")
        except Exception:
            pass

def _write_webots_time(path):
    """Write current Webots simulator time to file — overwrite atomically."""
    try:
        webots_time = supervisor.getTime()
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(f"{webots_time:.6f}\n")
        os.replace(tmp, path)
    except Exception as e:
        # non-fatal
        try:
            print(f"[webots_time] failed to write: {e}")
        except Exception:
            pass

def _read_speed_mps(path, default_value):
    """Read cruise speed in m/s from speed.txt, or return default_value."""
    try:
        with open(path, 'r') as f:
            raw = f.read().strip()
            if not raw:
                return default_value
            value = float(raw)
            return value if value > 0 else default_value
    except Exception:
        return default_value

def _write_visible_balls(path, viewfield_deg=60.0, visible_range_m=0.8):
    """Write visible balls within viewfield and range to file — overwrite atomically."""
    try:
        tmp = path + '.tmp'
        pos = main_robot.getField("translation").getSFVec3f()
        rx, ry = float(pos[0]), float(pos[1])
        rot = main_robot.getField("rotation").getSFRotation()
        rangle = float(rot[3])
        half_fov = math.radians(viewfield_deg) / 2.0
        range_sq = visible_range_m * visible_range_m

        def _segment_intersects_aabb(p0, p1, half):
            """2D segment vs axis-aligned square centered at origin."""
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
            # Check line-of-sight segment from robot to ball against each obstacle square.
            for robot in obstacle_robots:
                opos = robot.getField("translation").getSFVec3f()
                ox, oy = float(opos[0]), float(opos[1])
                orot = robot.getField("rotation").getSFRotation()
                oangle = float(orot[3])

                # Transform segment endpoints into obstacle-local frame.
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

        with open(tmp, 'w') as f:
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
                
                if dist_sq < 0.15*0.15:
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

                f.write(f"({bx:.6f}, {by:.6f}, {typ})\n")

        os.replace(tmp, path)
    except Exception as e:
        try:
            print(f"[visible_balls] failed to write: {e}")
        except Exception:
            pass

# ==== Initialize robots ====
# Main robot: single waypoint navigation from dynamic_waypoints.txt
main_trans = main_robot.getField("translation")
main_rot = main_robot.getField("rotation")
main_motion = MotionController(main_trans, main_rot, dt, cycle_mode=False)

# Obstacle robots: cycle through waypoints from obstacle_plan.txt (optional)
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

# Load initial dynamic waypoint for main robot
current_waypoint = _load_dynamic_waypoint(DYNAMIC_WAYPOINTS_FILE)
last_dynamic_waypoint = current_waypoint
last_waypoint_mtime = None
try:
    last_waypoint_mtime = os.path.getmtime(DYNAMIC_WAYPOINTS_FILE)
except Exception:
    last_waypoint_mtime = None

if current_waypoint is not None:
    x, y, ang = current_waypoint
    main_motion.start(x, y, velocity=None, angle=ang)

# Cruise script execution parameters
CRUISE_SCRIPT_PATH = os.path.join(DECISION_MAKING_DIR, "waypoints_cruise.py")
CRUISE_INTERVAL_FRAMES = 15  # Execute cruise script every N frames
frame_counter = 0

# Non-blocking main loop
while supervisor.step(TIME_STEP) != -1:
    # ==== Update main robot (single waypoint navigation) ====
    # Check if dynamic_waypoints.txt has changed
    try:
        mtime = os.path.getmtime(DYNAMIC_WAYPOINTS_FILE)
    except Exception:
        mtime = None
    
    if mtime is not None and mtime != last_waypoint_mtime:
        new_waypoint = _load_dynamic_waypoint(DYNAMIC_WAYPOINTS_FILE)
        if new_waypoint is not None and new_waypoint != last_dynamic_waypoint:
            # Waypoint changed: record the old one as 'cut' and start new one
            if last_dynamic_waypoint is not None and current_waypoint is not None:
                _append_to_history(WAYPOINTS_HISTORY_FILE, last_dynamic_waypoint, "cut")
            current_waypoint = new_waypoint
            last_dynamic_waypoint = new_waypoint    
            x, y, ang = current_waypoint
            main_motion.start(x, y, velocity=None, angle=ang)
        last_waypoint_mtime = mtime

    # 0.5) Update main robot cruise speed from speed.txt
    main_motion.velocity = _read_speed_mps(SPEED_FILE, DEFAULT_VELOCITY)
    
    # 1) Advance main robot motion (non-blocking)
    main_motion.update()
    
    # 1.2) Advance all obstacle robots motion
    for obs_motion in obstacle_motions:
        obs_motion.update()
    
    # 1.5) Update real-time status file
    _update_status_file(WAYPOINT_STATUS_FILE, main_motion.active, current_waypoint is not None)
    
    # 1.6) Execute cruise script at intervals
    frame_counter += 1
    if frame_counter % CRUISE_INTERVAL_FRAMES == 0:
        try:
            subprocess.run([sys.executable, CRUISE_SCRIPT_PATH], check=False, timeout=1)
        except Exception as e:
            print(f"[Cruise] Error running waypoints_cruise.py: {e}")

    # 2) Call absorption check for main robot
    monitor_simple_step(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION)

    # 2.1) Call absorption check for all obstacle robots
    for i, obs_robot in enumerate(obstacle_robots):
        # Different absorb locations for each obstacle robot to avoid overlap
        absorb_x = 1.1 + i * 0.1
        _monitor_absorption_for_robot(obs_robot, ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=(absorb_x, 0.0, 0.4))

    # 2.5) Write per-frame ball positions to file
    _write_ball_positions(BALL_POS_FILE)
    
    # 2.6) Write robot current position to file
    _write_current_position(CURRENT_POSITION_FILE)
    # 2.65) Write obstacle robots positions to file
    _write_obstacle_positions(OBSTACLE_ROBOT_FILE)
    
    
    # 2.7) Write Webots simulator time to file
    _write_webots_time(TIME_FILE)

    # 2.8) Write visible balls for main robot
    _write_visible_balls(VISIBLE_BALLS_FILE)

    # 3) If main robot motion completed, record as 'reached' and wait for next dynamic waypoint
    if not main_motion.active and current_waypoint is not None:
        _append_to_history(WAYPOINTS_HISTORY_FILE, current_waypoint, "reached")
        current_waypoint = None

    # 4) Other per-frame logic can be added here (logging / keyboard / debug)
    SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1

# Exit
print("Supervisor controller exiting.")