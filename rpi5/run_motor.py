#!/usr/bin/env python3
"""
Interactive mecanum controller with single-key per-wheel toggles.

Per-wheel single-key toggles (cycle Stop->Forward->Reverse->Stop):
  i -> wheel 1
  u -> wheel 2
  k -> wheel 3
  j -> wheel 4

Pattern keys:
  W - forward
  S - backward
  A - rotate left
  D - rotate right
  E - strafe right
  Q - strafe left
  X or Space - stop
  Z - quit

Note: wheels 5 and 6 remain controlled by patterns only (edit KEY_TO_WHEEL to add more).
"""

from gpiozero import DigitalOutputDevice
import sys, termios, tty, atexit
from time import sleep

# === CONFIG ===
WHEELS = {
    1: (5, 6),
    2: (13, 19),
    3: (17, 18),
    4: (22, 23),
    5: (24, 25),
    6: (27, 4),
}

# Map single-letter keys to wheel IDs you asked for:
# i -> 1, u -> 2, k -> 3, j -> 4
KEY_TO_WHEEL = {
    'i': 1,
    'u': 2,
    'k': 3,
    'j': 4,
}

LEFT_WHEELS = (1, 2, 3)
RIGHT_WHEELS = (4, 5, 6)

LOOP_SLEEP = 0.05
POLL = LOOP_SLEEP

# Hard-coded patterns (you can edit these later)
PATTERNS = {
    'stop':  {i: 's' for i in WHEELS},
    'forward': {i: 'f' for i in WHEELS},
    'reverse': {i: 'r' for i in WHEELS},
    'rotate_right': {1: 'r', 2: 'r', 3: 'r', 4: 'f'},
    'rotate_left':  {1: 'f', 2: 'f', 3: 'f', 4: 'r'},
    'strafe_right': {1: 'f', 2: 'r', 3: 'r', 4: 'f'},  # tweak if needed
    'strafe_left':  {1: 'r', 2: 'f', 3: 'f', 4: 'r'},  # tweak if needed
}
# === END CONFIG ===


def init_devices(wheels_map):
    devices = {}
    for wid, (p1, p2) in wheels_map.items():
        devices[wid] = (DigitalOutputDevice(p1, initial_value=False),
                        DigitalOutputDevice(p2, initial_value=False))
    return devices

def close_devices(devices):
    for a, b in devices.values():
        try: a.off()
        except Exception: pass
        try: b.off()
        except Exception: pass
        try: a.close()
        except Exception: pass
        try: b.close()
        except Exception: pass

def set_wheel_state(devices, wid, state):
    """state: 'f' forward, 'r' reverse, 's' stop"""
    a, b = devices[wid]
    if state == 'f':
        a.on(); b.off()
    elif state == 'r':
        a.off(); b.on()
    else:
        a.off(); b.off()

def apply_pattern(devices, pattern):
    for wid, s in pattern.items():
        set_wheel_state(devices, wid, s)

# keyboard helper
def getkey(timeout=None):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if timeout is None:
            return sys.stdin.read(1)
        else:
            import select
            r, _, _ = select.select([sys.stdin], [], [], timeout)
            if r:
                return sys.stdin.read(1)
            return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def print_instructions():
    print("Per-wheel single-key toggles (cycle Stop->Forward->Reverse):")
    for k, wid in KEY_TO_WHEEL.items():
        print(f"  {k.upper()} -> wheel {wid}")
    print("\nPattern keys:")
    print("  W-forward, S-back, A-rotate left, D-rotate right")
    print("  E-strafe right, Q-strafe left, X/Space-stop, Z-quit")
    print("\nPress a key (no Enter). Current wheel states printed after each change.\n")

def wheel_states_to_str(states):
    # produce a readable string "1:F 2:S 3:R ..."
    return " ".join(f"{wid}:{states[wid].upper()}" for wid in sorted(states))

def main():
    devices = init_devices(WHEELS)
    atexit.register(lambda: close_devices(devices))

    # track current desired state for each wheel (s/f/r)
    wheel_states = {i: 's' for i in WHEELS}

    def apply_states():
        apply_pattern(devices, wheel_states)

    try:
        apply_pattern(devices, PATTERNS['stop'])
        print_instructions()
        print("Initial:", wheel_states_to_str(wheel_states))

        while True:
            k = getkey(timeout=POLL)
            if not k:
                continue
            key = k.lower()

            # single-key per-wheel toggle (cycle s -> f -> r -> s ...)
            if key in KEY_TO_WHEEL:
                wid = KEY_TO_WHEEL[key]
                cur = wheel_states[wid]
                new = {'s':'f','f':'r','r':'s'}[cur]
                wheel_states[wid] = new
                apply_states()
                print(f"Toggle wheel {wid}: {cur} -> {new} | {wheel_states_to_str(wheel_states)}")
                continue

            # pattern keys
            if key == 'w':
                wheel_states.update(PATTERNS['forward'])
                apply_states()
                print("PATTERN -> FORWARD |", wheel_states_to_str(wheel_states))
            elif key == 's':
                wheel_states.update(PATTERNS['reverse'])
                apply_states()
                print("PATTERN -> REVERSE |", wheel_states_to_str(wheel_states))
            elif key == 'a':
                wheel_states.update(PATTERNS['rotate_left'])
                apply_states()
                print("PATTERN -> ROTATE LEFT |", wheel_states_to_str(wheel_states))
            elif key == 'd':
                wheel_states.update(PATTERNS['rotate_right'])
                apply_states()
                print("PATTERN -> ROTATE RIGHT |", wheel_states_to_str(wheel_states))
            elif key == 'e':
                wheel_states.update(PATTERNS['strafe_right'])
                apply_states()
                print("PATTERN -> STRAFE RIGHT |", wheel_states_to_str(wheel_states))
            elif key == 'q':
                wheel_states.update(PATTERNS['strafe_left'])
                apply_states()
                print("PATTERN -> STRAFE LEFT |", wheel_states_to_str(wheel_states))
            elif key in ('x', ' '):
                wheel_states.update(PATTERNS['stop'])
                apply_states()
                print("PATTERN -> STOP |", wheel_states_to_str(wheel_states))
            elif key == 'z':
                print("Quitting...")
                break
            else:
                print("Unknown key:", repr(key))

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        apply_pattern(devices, PATTERNS['stop'])
        close_devices(devices)
        print("Motors stopped, devices closed.")

if __name__ == '__main__':
    main()
