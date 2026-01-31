# supervisor_controller.py
# Non-blocking Supervisor controller (includes random ball placement + simplified absorption logic)
from controller import Supervisor
import math
import numpy as np
import random
import sys
import subprocess

# ==== Initialization ====
DEFAULT_VELOCITY = 0.3 # m/s
DEFAULT_ANGULAR_VELOCITY = 3  # rad/s
supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())
dt = TIME_STEP / 1000.0  # seconds

SCORE = 0
STEEL_STORED = 0
PING_STORED = 0
STEEL_HIT = 0
PING_HIT = 0

# Ensure the world contains DEF MY_ROBOT
robot = supervisor.getFromDef("MY_ROBOT")
if robot is None:
    print("ERROR: DEF MY_ROBOT not found in world.")
    sys.exit(1)

trans = robot.getField("translation")
rot_field = robot.getField("rotation")

# ==== Utility functions ====
def _normalize_angle(a):
    """Normalize angle to [-pi, pi]"""
    return (a + math.pi) % (2 * math.pi) - math.pi

# ==== Non-blocking MotionController (replacement for blocking move_to) ====
class MotionController:
    def __init__(self, trans_field, rot_field, dt):
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
        self.angular_speed = DEFAULT_ANGULAR_VELOCITY  # rad/s (default 90 deg/s)
        self.direction = np.array([0.0, 0.0, 0.0])
        self.total_dist = 0.0
        self.traveled = 0.0

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
            # Rotate in place to face the target, then move in a straight line (do not change orientation)
            target_yaw = math.atan2((y - cur_pos[1]), (x - cur_pos[0]))
            self.target_angle = _normalize_angle(target_yaw)
            self.phase = 'rotate_then_move'
        else:
            # Move and interpolate rotation to the target angle simultaneously
            self.phase = 'move_and_rotate'

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

        if self.phase == 'rotate_only':
            done = step_rotate_towards(self.target_angle)
            if done:
                self.active = False
                self.phase = None
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
                return True
            new_pos = cur_pos + self.direction * step_dist
            self.trans.setSFVec3f(new_pos.tolist())
            return False

        if self.phase == 'move_and_rotate':
            remaining = np.linalg.norm(self.target_pos - cur_pos)
            if remaining <= step_dist:
                self.trans.setSFVec3f(self.target_pos.tolist())
                self.rot.setSFRotation([0, 0, 1, self.target_angle])
                self.active = False
                self.phase = None
                return True
            # Position update
            new_pos = cur_pos + self.direction * step_dist
            self.trans.setSFVec3f(new_pos.tolist())
            # Angle interpolated based on travelled/total_dist
            self.traveled += step_dist
            progress = min(self.traveled / (self.total_dist + 1e-9), 1.0)
            current_angle = _normalize_angle(self.start_angle + _normalize_angle(self.target_angle - self.start_angle) * progress)
            self.rot.setSFRotation([0,0,1,current_angle])
            return False

        return True

    def cancel(self):
        self.active = False
        self.phase = None

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
ABSORB_DISTANCE = 0  # robot-local absorption distance threshold
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
    global PING_STORED, STEEL_STORED
    # Read robot current position (world coordinates)
    robot_pos = np.array(robot.getField("translation").getSFVec3f(), dtype=float)
    rx, ry = float(robot_pos[0]), float(robot_pos[1])
    # Use the original rotation handling (assumes rotation around z axis based on angle)
    robor_rot = np.array(robot.getField("rotation").getSFRotation())
    rangle = float(robor_rot[3])

    min_distance = 2

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
        temp_distance = math.sqrt((bx - rx)**2 + (by - ry)**2)
        min_distance = min(min_distance, temp_distance)

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
            # print(f"[monitor_simple] absorbed {name} type={ball_type} at x={x_ball_robot:.3f}, y={y_ball_robot:.3f}, distance={temp_distance:.3f} -> moved to {absorb_location}")
            print(f"Score: {SCORE} | Ping Hit: {PING_HIT} | Steel Hit: {STEEL_HIT} | Steel Stored: {STEEL_STORED} | Ping Stored: {PING_STORED}")


    # print(f"[monitor_simple] min_distance to balls: {min_distance:.3f}")

import random, math, numpy as np

# ==== Main logic (randomize balls -> init monitoring -> waypoint queue -> non-blocking main loop) ====
randomize_balls(seed=1235, ensure_no_overlap=True)
monitor_simple_init(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT)

# waypoints are loaded from the external file controllers/supervisor_controller/dynamic_waypoints.txt
import os, re, time

# waypoint orientation aliases (will be used when parsing the file)
North = math.pi / 2
East = 0.0
South = -math.pi / 2
West = math.pi

DYNAMIC_WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), "dynamic_waypoints.txt")
WAYPOINTS_HISTORY_FILE = os.path.join(os.path.dirname(__file__), "waypoints_history.txt")
WAYPOINT_STATUS_FILE = os.path.join(os.path.dirname(__file__), "waypoint_status.txt")
BALL_POS_FILE = os.path.join(os.path.dirname(__file__), "ball_position.txt")
CURRENT_POSITION_FILE = os.path.join(os.path.dirname(__file__), "current_position.txt")

def _load_dynamic_waypoint(path):
    """Load a single waypoint from dynamic_waypoints.txt (read-only).
    Returns a single (x, y, orientation) tuple or None if file is empty/invalid.
    """
    namespace = {"North": North, "East": East, "South": South, "West": West, "None": None}
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
                            ang = float(ang)
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
    ang_s = 'None' if ang is None else f"{ang}"
    record = f"({x}, {y}, {ang_s}),  {status}\n"
    
    # Always append to file
    with open(path, 'a') as f:
        f.write(record)

def _append_history_header(path):
    """Append a new session header to waypoints_history.txt with current timestamp.
    Called once at program startup.
    """
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    header = f"\n# Waypoint History - Started at {timestamp}\n"
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
                if x < -1.0:
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
                f.write(f"({x:.6f}, {y:.6f}, {typ})\n")
        os.replace(tmp, path)
    except Exception as e:
        # non-fatal: print to stdout for debugging
        try:
            print(f"[ball_positions] failed to write positions: {e}")
        except Exception:
            pass


def _write_current_position(path):
    """Write robot's current position (x, y) to file — overwrite atomically."""
    try:
        tmp = path + '.tmp'
        pos = robot.getField("translation").getSFVec3f()
        x, y = float(pos[0]), float(pos[1])
        with open(tmp, 'w') as f:
            f.write(f"({x:.6f}, {y:.6f})\n")
        os.replace(tmp, path)
    except Exception as e:
        # non-fatal
        try:
            print(f"[current_position] failed to write: {e}")
        except Exception:
            pass

motion = MotionController(trans, rot_field, dt)

# Initialize waypoints history with new session header
_append_history_header(WAYPOINTS_HISTORY_FILE)

# Load initial dynamic waypoint
current_waypoint = _load_dynamic_waypoint(DYNAMIC_WAYPOINTS_FILE)
last_dynamic_waypoint = current_waypoint
last_waypoint_mtime = None
try:
    last_waypoint_mtime = os.path.getmtime(DYNAMIC_WAYPOINTS_FILE)
except Exception:
    last_waypoint_mtime = None

if current_waypoint is not None:
    x, y, ang = current_waypoint
    motion.start(x, y, velocity=None, angle=ang)

# Cruise script execution parameters
CRUISE_SCRIPT_PATH = os.path.join(os.path.dirname(__file__), "waypoints_cruise.py")
CRUISE_INTERVAL_FRAMES = 10  # Execute cruise script every N frames
frame_counter = 0

# Non-blocking main loop
while supervisor.step(TIME_STEP) != -1:
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
            motion.start(x, y, velocity=None, angle=ang)
        last_waypoint_mtime = mtime
    
    # 1) Advance motion (non-blocking)
    motion.update()
    
    # 1.5) Update real-time status file
    _update_status_file(WAYPOINT_STATUS_FILE, motion.active, current_waypoint is not None)
    
    # 1.6) Execute cruise script at intervals
    frame_counter += 1
    if frame_counter % CRUISE_INTERVAL_FRAMES == 0:
        try:
            subprocess.run([sys.executable, CRUISE_SCRIPT_PATH], check=False, timeout=1)
        except Exception as e:
            print(f"[Cruise] Error running waypoints_cruise.py: {e}")

    # 2) Call simplified absorption check each frame
    monitor_simple_step(ball_prefix=BALL_PREFIX, ball_count=BALL_COUNT, half_x=ABSORB_BOX_HALF_X, half_y=ABSORB_BOX_HALF_Y, absorb_location=ABSORB_LOCATION)

    # 2.5) Write per-frame ball positions to file
    _write_ball_positions(BALL_POS_FILE)
    
    # 2.6) Write robot current position to file
    _write_current_position(CURRENT_POSITION_FILE)

    # 3) If motion completed, record as 'reached' and wait for next dynamic waypoint
    if not motion.active and current_waypoint is not None:
        _append_to_history(WAYPOINTS_HISTORY_FILE, current_waypoint, "reached")
        current_waypoint = None

    # 4) Other per-frame logic can be added here (logging / keyboard / debug)
    SCORE = PING_HIT * 4 + STEEL_HIT * 2 + STEEL_STORED * 1

# Exit
print("Supervisor controller exiting.")