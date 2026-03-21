#!/usr/bin/env bash
# setup_reset_button.sh
# Run once on the Raspberry Pi (Ubuntu 24.04 Server) to configure the
# hardware power button as a ROS 2 reset trigger instead of a shutdown key.
#
# Usage:
#   chmod +x setup_reset_button.sh
#   ./setup_reset_button.sh

set -euo pipefail

echo "=== [1/3] Installing python3-evdev ==="
sudo apt-get install -y python3-evdev

echo ""
echo "=== [2/3] Disabling power-key shutdown in systemd-logind ==="
LOGIND_CONF=/etc/systemd/logind.conf

# Replace existing HandlePowerKey line, or append if absent
if grep -qE '^#?HandlePowerKey=' "$LOGIND_CONF"; then
    sudo sed -i 's/^#*HandlePowerKey=.*/HandlePowerKey=ignore/' "$LOGIND_CONF"
else
    echo 'HandlePowerKey=ignore' | sudo tee -a "$LOGIND_CONF" > /dev/null
fi

sudo systemctl restart systemd-logind
echo "HandlePowerKey=ignore applied."

echo ""
echo "=== [3/3] Verifying power button device ==="
python3 - <<'EOF'
try:
    import evdev
    from evdev import InputDevice, ecodes as ev
    found = []
    for path in evdev.list_devices():
        try:
            d = InputDevice(path)
            caps = d.capabilities()
            if ev.EV_KEY in caps and ev.KEY_POWER in caps[ev.EV_KEY]:
                found.append(f"  {d.path}  ({d.name})")
        except Exception:
            pass
    if found:
        print("Power button device(s) found:")
        for f in found:
            print(f)
    else:
        print("WARNING: No power button device found. The node will warn at runtime.")
except ImportError:
    print("ERROR: evdev import failed even after install.")
EOF

echo ""
echo "=== Setup complete ==="
echo ""
echo "Add reset_button_node to your launch file, or start it manually:"
echo "  ros2 run unibots reset_button_node"
echo ""
echo "The node needs access to /dev/input — if you see permission errors, add"
echo "the user to the 'input' group:"
echo "  sudo usermod -aG input \$USER   # then log out and back in"
