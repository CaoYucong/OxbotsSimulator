from controller import Supervisor
import numpy as np

supervisor = Supervisor()
TIME_STEP = int(supervisor.getBasicTimeStep())
dt = TIME_STEP / 1000.0  # seconds

robot = supervisor.getFromDef("MY_ROBOT")
assert robot is not None, "DEF MY_ROBOT not found"

trans = robot.getField("translation")

import math
import numpy as np

def _normalize_angle(a):
    """归一化到 [-pi, pi]"""
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return a

def move_to(x, y, velocity=None, angle=None):
    """
    移动到 (x, y)。
    - velocity: 线速度 m/s；None -> 默认 0.3 m/s
    - angle: 目标朝向（弧度），如果不传则先在原地朝向目标点再直行；
             若传入则为边走边转（在移动过程中按最短角差插值到该角度）。
    依赖全局：supervisor, TIME_STEP, dt, robot, trans
    """
    DEFAULT_MAX_VELOCITY = 0.3  # m/s
    ANGULAR_SPEED_RAD = 1.5708  # rad/s (≈90°/s)
    ANGLE_TOL = 1e-3
    POS_TOL = 1e-6

    if velocity is None:
        velocity = DEFAULT_MAX_VELOCITY

    # 当前位姿
    start_pos = np.array(trans.getSFVec3f(), dtype=float)
    target_pos = np.array([x, y, start_pos[2]], dtype=float)
    delta = target_pos - start_pos
    distance = np.linalg.norm(delta)

    rot_field = robot.getField("rotation")
    start_rot = rot_field.getSFRotation()  # [ax, ay, az, angle]
    start_angle = _normalize_angle(start_rot[3])

    # 情况 A: 距离几乎为零
    if distance < POS_TOL:
        if angle is None:
            # 无需移动也无需转向
            return
        # 只有旋转需求：在原地平滑旋转到目标角度
        target_angle = _normalize_angle(angle)
        diff = _normalize_angle(target_angle - start_angle)
        if abs(diff) <= ANGLE_TOL:
            rot_field.setSFRotation([0, 0, 1, target_angle])
            return
        # 以 ANGULAR_SPEED_RAD 逐帧旋转
        sign = 1.0 if diff > 0 else -1.0
        max_step = ANGULAR_SPEED_RAD * dt
        while supervisor.step(TIME_STEP) != -1:
            cur = _normalize_angle(rot_field.getSFRotation()[3])
            rem = _normalize_angle(target_angle - cur)
            if abs(rem) <= ANGLE_TOL:
                rot_field.setSFRotation([0, 0, 1, target_angle])
                break
            step_ang = max_step if abs(rem) > max_step else abs(rem)
            rot_field.setSFRotation([0, 0, 1, cur + sign * step_ang])
        return

    # 否则 distance > 0：
    # 当 angle 未提供时：先在原地转到面向目标点的朝向（target_yaw），再直线前进且不再转动。
    # 当 angle 提供时：边走边转至 angle（在移动过程中插值角度）。
    target_yaw = math.atan2((y - start_pos[1]), (x - start_pos[0]))

    if angle is None:
        # 先在原地旋转到面向目标
        desired_yaw = _normalize_angle(target_yaw)
        diff = _normalize_angle(desired_yaw - start_angle)
        if abs(diff) > ANGLE_TOL:
            sign = 1.0 if diff > 0 else -1.0
            max_step = ANGULAR_SPEED_RAD * dt
            while supervisor.step(TIME_STEP) != -1:
                cur = _normalize_angle(rot_field.getSFRotation()[3])
                rem = _normalize_angle(desired_yaw - cur)
                if abs(rem) <= ANGLE_TOL:
                    rot_field.setSFRotation([0, 0, 1, desired_yaw])
                    break
                step_ang = max_step if abs(rem) > max_step else abs(rem)
                rot_field.setSFRotation([0, 0, 1, cur + sign * step_ang])
        # 旋转完毕后开始平移（不改变朝向）
        direction = delta / distance
        step_dist = velocity * dt
        pos = start_pos.copy()
        while supervisor.step(TIME_STEP) != -1:
            remaining = np.linalg.norm(target_pos - pos)
            if remaining <= step_dist:
                trans.setSFVec3f(target_pos.tolist())
                break
            pos += direction * step_dist
            trans.setSFVec3f(pos.tolist())
        return

    # angle 提供：边走边转到 angle（同时最短角差插值）
    target_angle = _normalize_angle(angle)
    # 计算最短角差（用于插值）
    angle_diff = _normalize_angle(target_angle - start_angle)

    direction = delta / distance
    step_dist = velocity * dt

    pos = start_pos.copy()
    traveled = 0.0

    while supervisor.step(TIME_STEP) != -1:
        remaining = np.linalg.norm(target_pos - pos)
        # 当到达终点或能在一步内到达，直接写终点位姿并结束
        if remaining <= step_dist:
            trans.setSFVec3f(target_pos.tolist())
            rot_field.setSFRotation([0, 0, 1, target_angle])
            break

        # 位置更新
        pos += direction * step_dist
        traveled += step_dist
        trans.setSFVec3f(pos.tolist())

        # 角度按进度线性插值（沿最短角差）
        progress = min(traveled / distance, 1.0)
        current_angle = _normalize_angle(start_angle + angle_diff * progress)
        rot_field.setSFRotation([0, 0, 1, current_angle])

# ========== Randomize BALL placement script ==========
import random
import math
import numpy as np

# ========== Global / configuration (ensure consistency with the top of your script or modify as needed) ==========
BALL_PREFIX = "BALL_"     # DEF prefix
BALL_COUNT = 40
X_MIN, X_MAX = -0.96, 0.96
Y_MIN, Y_MAX = -0.96, 0.96
BALL_RADIUS = 0.02        # ball radius (m), should match world
MIN_SEPARATION = 2.0 * BALL_RADIUS + 0.001
MAX_TRIES_PER_BALL = 2000
SETTLE_STEPS_AFTER_PLACEMENT = int(0.2 / dt)
Z_EPS = 0.001

# ========== Robot occupancy parameters (as provided) ==========
ROBOT_X = -0.9
ROBOT_Y = 0.0
ROBOT_HALF_SIZE = 0.1    # robot half-size (0.2/2)
CLEARANCE = 0.05         # extra safety clearance around the robot (tunable)

def _point_in_rect(px, py, rect):
    """rect: (xmin, xmax, ymin, ymax)"""
    return (rect[0] <= px <= rect[1]) and (rect[2] <= py <= rect[3])

def randomize_balls(seed=None, ensure_no_overlap=True):
    """
    Randomly place BALL_0..BALL_{BALL_COUNT-1}, avoiding the robot's occupied rectangular area.

    seed: random seed
    ensure_no_overlap: whether to use rejection sampling to avoid ball overlap
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    # Build list of avoid zones (each is (xmin, xmax, ymin, ymax))
    avoid_zones = []

    # Add the robot's occupied rectangle (including ball radius and extra clearance)
    margin = ROBOT_HALF_SIZE + CLEARANCE + BALL_RADIUS
    robot_rect = (
        ROBOT_X - margin,
        ROBOT_X + margin,
        ROBOT_Y - margin,
        ROBOT_Y + margin
    )
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
            # Randomize directly but avoid avoid_zones
            for attempt in range(MAX_TRIES_PER_BALL):
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)

                blocked = False
                for rect in avoid_zones:
                    if _point_in_rect(x, y, rect):
                        blocked = True
                        break
                if not blocked:
                    break
            else:
                # On failure, force placement (may be inside an avoid zone)
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)

            z = BALL_RADIUS + Z_EPS
            trans_field.setSFVec3f([x, y, z])
            node.resetPhysics()
            placed_positions.append((x, y))
            continue

        # Use rejection sampling; avoid already placed balls and avoid_zones
        success = False
        for attempt in range(MAX_TRIES_PER_BALL):
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)

            # Check whether the point falls inside any avoid zone
            blocked = False
            for rect in avoid_zones:
                if _point_in_rect(x, y, rect):
                    blocked = True
                    break
            if blocked:
                continue

            # Then check distance to already placed balls (avoid overlap)
            ok = True
            for (px, py) in placed_positions:
                dx = x - px
                dy = y - py
                if (dx*dx + dy*dy) < (MIN_SEPARATION * MIN_SEPARATION):
                    ok = False
                    break

            if ok:
                z = BALL_RADIUS + Z_EPS
                trans_field.setSFVec3f([x, y, z])
                node.resetPhysics()
                placed_positions.append((x, y))
                success = True
                break

        if not success:
            # Fallback: try again without considering overlap, only avoid zones
            for attempt in range(MAX_TRIES_PER_BALL):
                x = random.uniform(X_MIN, X_MAX)
                y = random.uniform(Y_MIN, Y_MAX)
                blocked = False
                for rect in avoid_zones:
                    if _point_in_rect(x, y, rect):
                        blocked = True
                        break
                if not blocked:
                    z = BALL_RADIUS + Z_EPS
                    trans_field.setSFVec3f([x, y, z])
                    node.resetPhysics()
                    placed_positions.append((x, y))
                    success = True
                    break

        if not success:
            # If still failing, force place at a random location (and warn)
            print(f"Warning: failed to place {name} without overlap/avoid zone. Forcing placement.")
            x = random.uniform(X_MIN, X_MAX)
            y = random.uniform(Y_MIN, Y_MAX)
            z = BALL_RADIUS + Z_EPS
            trans_field.setSFVec3f([x, y, z])
            node.resetPhysics()
            placed_positions.append((x, y))

    # After placement, step the simulation for a few steps to let physics settle
    for _ in range(max(1, SETTLE_STEPS_AFTER_PLACEMENT)):
        supervisor.step(TIME_STEP)

    print(f"Randomized {len(placed_positions)} balls. seed={seed}, no_overlap={ensure_no_overlap}")

randomize_balls(seed=1234, ensure_no_overlap=True)

move_to(0.8, 0.8)
move_to(0.8, -0.8)
move_to(-0.8, -0.8)
move_to(-0.8, 0.8)
move_to(0.0, 0.0)
move_to(0.8, 0.8, angle=math.pi/2)
move_to(0.8, -0.8, angle=math.pi)
move_to(-0.8, -0.8, angle=math.pi* 3/2)
move_to(-0.8, 0.8, angle=math.pi)
move_to(0.0, 0.0, angle=math.pi/2)
