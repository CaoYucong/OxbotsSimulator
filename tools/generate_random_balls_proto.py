#!/usr/bin/env python3
"""
generate_balls_proto_random_dep.py

Generate a UnibotsBallsRandom.proto that depends on PingBall.proto and SteelBall.proto.
Each ball instance has:
  - unique DEF name
  - unique solidName passed down to PingBall/SteelBall (with '_random' suffix)
  - translation x y z (y and z order adjusted per user request)
  - optional random rotation
Uses rejection sampling to avoid overlaps.

Writes to: ./protos/UnibotsBallsRandom.proto (backup previous file)
"""

import os
import sys
import math
import random
import time
from datetime import datetime

# ---------- Config ----------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(BASE_DIR)
PROJECT_ROOT = os.path.abspath('.')
PROTO_DIR = os.path.join(PARENT_DIR, "protos")
PROTO_PATH = os.path.join(PROTO_DIR, "UnibotsBallsRandom.proto")
BACKUP_SUFFIX = datetime.now().strftime("%Y%m%d-%H%M%S")

PING_PROTO = "PingBall.proto"
STEEL_PROTO = "SteelBall.proto"

# Arena layout
ARENA_SIZE = 2.0
MARGIN = 0.10
XMIN = -ARENA_SIZE/2 + MARGIN
XMAX =  ARENA_SIZE/2 - MARGIN
ZMIN = -ARENA_SIZE/2 + MARGIN
ZMAX =  ARENA_SIZE/2 - MARGIN

PING_COUNT = 16
STEEL_COUNT = 24
PING_RADIUS = 0.02
STEEL_RADIUS = 0.01
Y_HEIGHT = 0.15    # height from ground for placement (kept as third coord)
MAX_ATTEMPTS = 5000

SEED = None
INCLUDE_ROTATION = True


# ---------- Helpers ----------

def backup_existing_proto(path):
    if os.path.isfile(path):
        bak = f"{path}.bak_{BACKUP_SUFFIX}"
        os.rename(path, bak)
        print(f"[Backup] {bak}")
    else:
        print("[Info] No previous proto to back up.")


def random_rotation():
    ax = random.random() - 0.5
    ay = random.random() - 0.5
    az = random.random() - 0.5
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        ax, ay, az = 0,1,0
        norm = 1
    ax /= norm; ay /= norm; az /= norm
    angle = random.uniform(0, 2*math.pi)
    return (ax, ay, az, angle)


def rejection_place(count, radius, placed):
    """
    placed: list of (x,z,r)
    returns list of (x,z)
    """
    results = []
    for k in range(count):
        ok = False
        attempts = 0
        while attempts < MAX_ATTEMPTS and not ok:
            attempts += 1
            x = random.uniform(XMIN, XMAX)
            z = random.uniform(ZMIN, ZMAX)

            overlap = False
            for (px, pz, pr) in placed:
                if math.hypot(px - x, pz - z) < 0.9 * (pr + radius):
                    overlap = True
                    break

            if not overlap:
                results.append((x, z))
                placed.append((x, z, radius))
                ok = True

        if not ok:
            print(f"[WARN] Failed after {MAX_ATTEMPTS}, placing at origin")
            results.append((0,0))
            placed.append((0,0,radius))

    return results


# ---------- Build proto ----------

def build_proto(ping_positions, steel_positions):

    header = (
        '#VRML_SIM R2025a utf8\n'
        f'EXTERNPROTO "{PING_PROTO}"\n'
        f'EXTERNPROTO "{STEEL_PROTO}"\n\n'
        'PROTO UnibotsBallsRandom [] {\n'
        '  Group {\n'
        '    children [\n\n'
    )

    footer = (
        '\n    ]\n'
        '  }\n'
        '}\n'
    )

    lines = [header]

    # ---- Ping balls ----
    for i, (x, z) in enumerate(ping_positions, start=1):
        def_name = f"PING{i:02d}"
        solid_name = f"PING{i:02d}_solid_random"
        rot = ""
        if INCLUDE_ROTATION:
            ax, ay, az, ang = random_rotation()
            rot = f" rotation {ax:.6f} {ay:.6f} {az:.6f} {ang:.6f}"
        # NOTE: user requested swap y and z: we output translation as "x z Y_HEIGHT"
        # Webots expects translation X Y Z; here we set Y = z (the generated z) and Z = Y_HEIGHT height
        lines.append(
            f'      DEF {def_name} PingBall {{ '
            f'solidName "{solid_name}" '
            f'translation {x:.6f} {z:.6f} {Y_HEIGHT:.6f}{rot} }}'
        )

    # ---- Steel balls ----
    for i, (x, z) in enumerate(steel_positions, start=1):
        def_name = f"STEEL{i:02d}"
        solid_name = f"STEEL{i:02d}_solid_random"
        rot = ""
        if INCLUDE_ROTATION:
            ax, ay, az, ang = random_rotation()
            rot = f" rotation {ax:.6f} {ay:.6f} {az:.6f} {ang:.6f}"
        lines.append(
            f'      DEF {def_name} SteelBall {{ '
            f'solidName "{solid_name}" '
            f'translation {x:.6f} {z:.6f} {Y_HEIGHT:.6f}{rot} }}'
        )

    lines.append(footer)
    return "\n".join(lines)


# ---------- Main ----------
def main():

    if SEED is not None:
        random.seed(SEED)
    else:
        random.seed(time.time())

    os.makedirs(PROTO_DIR, exist_ok=True)
    backup_existing_proto(PROTO_PATH)

    placed = []
    ping_positions = rejection_place(PING_COUNT, PING_RADIUS, placed)
    steel_positions = rejection_place(STEEL_COUNT, STEEL_RADIUS, placed)

    content = build_proto(ping_positions, steel_positions)

    with open(PROTO_PATH, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"[OK] Wrote {PROTO_PATH}")
    print("[Summary] Sample ping positions (x, z used as X Y in translation):")
    for x, z in ping_positions[:3]:
        print(f"  x={x:.3f}, y={z:.3f}")
    print("Reset Webots (Pause → Reset) to load new proto.")


if __name__ == "__main__":
    main()