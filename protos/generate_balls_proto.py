#!/usr/bin/env python3
"""
generate_balls_proto.py

Generate a UnibotsBallsRandom.proto that depends on PingBall.proto and SteelBall.proto.
Each ball is instantiated as PingBall { translation ... rotation ... } or SteelBall { ... }.
Uses rejection sampling to avoid overlaps.

Writes to: ./protos/UnibotsBallsRandom.proto (backup previous file if present)
Usage: python3 tools/generate_balls_proto_random_dep.py
"""
import os
import sys
import math
import random
import time
from datetime import datetime

# ---------- Config ----------
PROJECT_ROOT = os.path.abspath('.')  # adjust if running from other dir
# default proto output dir (change if your protos live elsewhere)
PROTO_DIR = os.path.join(PROJECT_ROOT)
PROTO_FILENAME = 'UnibotsBallsRandom.proto'
PROTO_PATH = os.path.join(PROTO_DIR, PROTO_FILENAME)
BACKUP_SUFFIX = datetime.now().strftime('%Y%m%d-%H%M%S')

# names of the dependent proto files (expected to exist in same PROTO_DIR or resolvable by world)
PING_PROTO = 'PingBall.proto'
STEEL_PROTO = 'SteelBall.proto'

# Arena and placement params
ARENA_SIZE = 2.0      # 2m x 2m arena
MARGIN = 0.10        # keep objects this far from walls
XMIN = -ARENA_SIZE/2 + MARGIN
XMAX =  ARENA_SIZE/2 - MARGIN
YMIN = -ARENA_SIZE/2 + MARGIN
YMAX =  ARENA_SIZE/2 - MARGIN

PING_COUNT = 16
STEEL_COUNT = 24
PING_RADIUS = 0.02   # ping-pong
STEEL_RADIUS = 0.01  # steel ball
MAX_ATTEMPTS = 5000
SEED = None          # set to int for reproducible layouts, or None for random
INCLUDE_ROTATION = True  # if True, emit a small random rotation for each ball

# ---------- Helper functions ----------
def backup_existing_proto(path):
    if os.path.isfile(path):
        dirname = os.path.dirname(path)
        base = os.path.basename(path)
        bak_name = f"{base}.bak_{BACKUP_SUFFIX}"
        bak_path = os.path.join(dirname, bak_name)
        os.rename(path, bak_path)
        print(f"Backed up existing proto to: {bak_path}")
    else:
        print("No existing proto to back up.")

def random_rotation():
    # produce an SFRotation: axis_x axis_y axis_z angle
    # choose a random axis and small angle (0..2pi)
    ax = random.random() - 0.5
    ay = random.random() - 0.5
    az = random.random() - 0.5
    norm = math.hypot(ax, ay, az)
    if norm == 0:
        ax, ay, az = 0, 1, 0
        norm = 1.0
    ax /= norm; ay /= norm; az /= norm
    angle = random.uniform(0, 2*math.pi)
    return (ax, ay, az, angle)

def rejection_place(count, radius, xmin, xmax, Ymin, Ymax, placed, max_attempts=5000):
    """
    Place `count` items of `radius` avoiding existing `placed` list.
    `placed` is list of (x,z,radius); returns list of (x,z)
    """
    results = []
    for i in range(count):
        ok = False
        attempts = 0
        while not ok and attempts < max_attempts:
            attempts += 1
            x = random.uniform(xmin, xmax)
            z = random.uniform(Ymin, Ymax)
            too_close = False
            for (px, pz, pr) in placed:
                if math.hypot(px - x, pz - z) < 0.9 * (pr + radius):
                    too_close = True
                    break
            if not too_close:
                placed.append((x, z, radius))
                results.append((x, z))
                ok = True
        if not ok:
            # fallback: place at origin offset (rare)
            print(f"Warning: failed to place item radius={radius} after {max_attempts} attempts; placing at (0,0)")
            results.append((0.0, 0.0))
            placed.append((0.0, 0.0, radius))
    return results

# ---------- Build proto content ----------
def build_proto_content(ping_positions, steel_positions):
    header = '#VRML_SIM R2025a utf8\n'
    header += f'EXTERNPROTO "{PING_PROTO}"\n'
    header += f'EXTERNPROTO "{STEEL_PROTO}"\n\n'
    header += 'PROTO UnibotsBallsRandom [] {\n  Group {\n    children [\n\n'
    footer = '\n    ]\n  }\n}\n'

    lines = [header]

    # ping balls -> instantiate PingBall with translation and optional rotation
    for i, (x, z) in enumerate(ping_positions, start=1):
        tx = f'{x:.6f}'
        ty = f'{z:.6f}'
        tz = '0.15'  # height as in your previous proto
        if INCLUDE_ROTATION:
            ax, ay, az, angle = random_rotation()
            rot_str = f' rotation {ax:.6f} {ay:.6f} {az:.6f} {angle:.6f}'
        else:
            rot_str = ''
        lines.append(f'      PingBall {{ translation {tx} {ty} {tz}{rot_str} }}')

    # steel balls
    for i, (x, z) in enumerate(steel_positions, start=1):
        tx = f'{x:.6f}'
        ty = f'{z:.6f}'
        tz = '0.15'
        if INCLUDE_ROTATION:
            ax, ay, az, angle = random_rotation()
            rot_str = f' rotation {ax:.6f} {ay:.6f} {az:.6f} {angle:.6f}'
        else:
            rot_str = ''
        lines.append(f'      SteelBall {{ translation {tx} {ty} {tz}{rot_str} }}')

    lines.append(footer)
    return '\n'.join(lines)

# ---------- Main ----------
def main():
    if SEED is not None:
        random.seed(SEED)
    else:
        random.seed(time.time())

    os.makedirs(PROTO_DIR, exist_ok=True)
    backup_existing_proto(PROTO_PATH)

    placed = []
    ping_positions = rejection_place(PING_COUNT, PING_RADIUS, XMIN, XMAX, YMIN, YMAX, placed, MAX_ATTEMPTS)
    steel_positions = rejection_place(STEEL_COUNT, STEEL_RADIUS, XMIN, XMAX, YMIN, YMAX, placed, MAX_ATTEMPTS)

    content = build_proto_content(ping_positions, steel_positions)

    with open(PROTO_PATH, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Wrote new proto to: {PROTO_PATH}")
    print(f"Ping balls: {len(ping_positions)}, Steel balls: {len(steel_positions)}")
    print("Example first 3 ping positions:")
    for p in ping_positions[:3]:
        print(f"  x={p[0]:.3f}, y={p[1]:.3f}")
    print("Done. Please Reset webots world (Pause → Reset) and then Play to reload the new proto.")

if __name__ == '__main__':
    main()