#!/usr/bin/env bash
# setup_reset_button.sh
# GPIO 24 is the dedicated match-control line (no extra host setup required).
#
#   LOW  (0 V)   → reset
#   HIGH (3.3 V) → start competition (stays HIGH for the whole match)
#
# Usage:
#   chmod +x setup_reset_button.sh
#   ./setup_reset_button.sh

set -euo pipefail

echo "=== Match control GPIO ==="
echo "GPIO 24: LOW=reset, HIGH=start competition"
echo ""
echo "Ensure python3-gpiozero is installed (usually via unibots package deps):"
echo "  sudo apt-get install -y python3-gpiozero"
echo ""
echo "Start the node manually or via unibots.launch.py:"
echo "  ros2 run unibots reset_button_node"
echo ""
echo "Quick read test:"
python3 - <<'EOF'
import time
try:
    from gpiozero import DigitalInputDevice
    gpio = DigitalInputDevice(24, pull_up=False)
    for _ in range(5):
        print(f"  GPIO 24 = {'HIGH (start)' if gpio.value else 'LOW (reset)'}", flush=True)
        time.sleep(0.2)
    gpio.close()
except ImportError:
    print("ERROR: gpiozero not installed.")
except Exception as exc:
    print(f"ERROR: {exc}")
    print("Hint: run as a user with GPIO access (e.g. gpio group on Raspberry Pi OS).")
EOF
