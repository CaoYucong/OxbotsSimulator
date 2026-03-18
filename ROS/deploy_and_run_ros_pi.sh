#!/usr/bin/env bash

set -euo pipefail
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_WS_DEFAULT="$SCRIPT_DIR/ros2_ws"

PI_USER="${PI_USER:-caoyucong}"
PI_IP="${PI_IP:-192.168.50.2}"
PI_PASSWORD="${PI_PASSWORD:-041003}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REMOTE_WS="${REMOTE_WS:-~/OxbotsSimulator/ROS/ros2_ws}"
LOCAL_WS="${LOCAL_WS:-$LOCAL_WS_DEFAULT}"
LOCAL_CONFIG="${LOCAL_CONFIG:-$ROOT_DIR/config.json}"
REMOTE_PROJECT_ROOT="${REMOTE_PROJECT_ROOT:-~/OxbotsSimulator}"
REMOTE_CONFIG="${REMOTE_CONFIG:-$REMOTE_PROJECT_ROOT/config.json}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-unibots}"
LAUNCH_FILE="${LAUNCH_FILE:-unibots.launch.py}"
PREFER_SOURCE_CV_BRIDGE="${PREFER_SOURCE_CV_BRIDGE:-1}"
REMOTE_OFFLINE_ROOT="${REMOTE_OFFLINE_ROOT:-~/offline_pkgs}"
REMOTE_VISION_OPENCV_ARCHIVE="${REMOTE_VISION_OPENCV_ARCHIVE:-$REMOTE_OFFLINE_ROOT/src/vision_opencv-rolling.tar.gz}"
REMOTE_VISION_OPENCV_SRC_DIR="${REMOTE_VISION_OPENCV_SRC_DIR:-$REMOTE_WS/src/vision_opencv-rolling}"
START_LOCAL_INFERENCE="${START_LOCAL_INFERENCE:-1}"
INFERENCE_HOST="${INFERENCE_HOST:-127.0.0.1}"
INFERENCE_PORT="${INFERENCE_PORT:-9001}"
INFERENCE_READY_TIMEOUT="${INFERENCE_READY_TIMEOUT:-30}"
INFERENCE_IMAGE_REF="${INFERENCE_IMAGE_REF:-roboflow/roboflow-inference-server-cpu:latest}"
REMOTE_INFERENCE_IMAGE_ARCHIVE="${REMOTE_INFERENCE_IMAGE_ARCHIVE:-/home/${PI_USER}/roboflow-inference-server-cpu-latest-arm64.tar.gz}"
REMOTE_INFERENCE_START_CMD="${REMOTE_INFERENCE_START_CMD:-docker run -d --restart unless-stopped --name unibots-roboflow-inference -p ${INFERENCE_PORT}:9001 ${INFERENCE_IMAGE_REF}}"
ROBOFLOW_API_KEY="${ROBOFLOW_API_KEY:-}"
INIT_TOF_ON_DEPLOY="${INIT_TOF_ON_DEPLOY:-1}"
TOF_FRONT_XSHUT_GPIO="${TOF_FRONT_XSHUT_GPIO:-21}"
TOF_LEFT_XSHUT_GPIO="${TOF_LEFT_XSHUT_GPIO:-26}"
TOF_RIGHT_XSHUT_GPIO="${TOF_RIGHT_XSHUT_GPIO:-16}"
TOF_REAR_XSHUT_GPIO="${TOF_REAR_XSHUT_GPIO:-20}"
TOF_FRONT_ADDR="${TOF_FRONT_ADDR:-0x33}"
TOF_LEFT_ADDR="${TOF_LEFT_ADDR:-0x30}"
TOF_RIGHT_ADDR="${TOF_RIGHT_ADDR:-0x31}"
TOF_REAR_ADDR="${TOF_REAR_ADDR:-0x32}"
TOF_BOOT_DELAY_MS="${TOF_BOOT_DELAY_MS:-80}"

CONTROL_PATH_DEFAULT="$HOME/.ssh/cm-%C"
SSH_CONTROL_PATH="${SSH_CONTROL_PATH:-$CONTROL_PATH_DEFAULT}"

MODE="foreground"
SKIP_BUILD="0"

usage() {
  cat <<'EOF'
Usage:
  ./ROS/deploy_and_run_ros_pi.sh [--detach] [--skip-build]

Options:
  --detach      Build then launch in background on Raspberry Pi (logs -> ~/unibots_ros.log)
  --skip-build  Skip colcon build and directly launch
  -h, --help    Show this help

Environment overrides:
  PI_USER, PI_IP, PI_PASSWORD, ROS_DISTRO, REMOTE_WS, LOCAL_WS, LOCAL_CONFIG,
  REMOTE_PROJECT_ROOT, REMOTE_CONFIG, LAUNCH_PACKAGE, LAUNCH_FILE,
  PREFER_SOURCE_CV_BRIDGE, REMOTE_OFFLINE_ROOT, REMOTE_VISION_OPENCV_ARCHIVE,
  REMOTE_VISION_OPENCV_SRC_DIR, START_LOCAL_INFERENCE, INFERENCE_HOST,
  INFERENCE_PORT, INFERENCE_READY_TIMEOUT, INFERENCE_IMAGE_REF,
  REMOTE_INFERENCE_IMAGE_ARCHIVE, REMOTE_INFERENCE_START_CMD, ROBOFLOW_API_KEY,
  INIT_TOF_ON_DEPLOY, TOF_FRONT_XSHUT_GPIO, TOF_LEFT_XSHUT_GPIO,
  TOF_RIGHT_XSHUT_GPIO, TOF_REAR_XSHUT_GPIO, TOF_FRONT_ADDR, TOF_LEFT_ADDR,
  TOF_RIGHT_ADDR, TOF_REAR_ADDR, TOF_BOOT_DELAY_MS

Examples:
  ./ROS/deploy_and_run_ros_pi.sh
  PI_IP=192.168.50.23 ./ROS/deploy_and_run_ros_pi.sh --detach
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --detach)
      MODE="detach"
      shift
      ;;
    --skip-build)
      SKIP_BUILD="1"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ ! -d "$LOCAL_WS" ]]; then
  echo "[ERROR] Local workspace not found: $LOCAL_WS"
  exit 1
fi

if [[ ! -d "$LOCAL_WS/src/$LAUNCH_PACKAGE" ]]; then
  echo "[ERROR] Expected package folder not found: $LOCAL_WS/src/$LAUNCH_PACKAGE"
  exit 1
fi

if [[ ! -f "$LOCAL_CONFIG" ]]; then
  echo "[ERROR] Local config file not found: $LOCAL_CONFIG"
  exit 1
fi

if [[ -z "$ROBOFLOW_API_KEY" ]] && command -v python3 >/dev/null 2>&1; then
  ROBOFLOW_API_KEY="$(python3 - "$LOCAL_CONFIG" <<'PY'
import json
import sys

config_path = sys.argv[1]
try:
    with open(config_path, 'r', encoding='utf-8') as f:
        payload = json.load(f)
except Exception:
    print('', end='')
    raise SystemExit(0)

key = payload.get('roboflow_api_key', '')
if isinstance(key, str):
    print(key.strip(), end='')
PY
)"
fi

if [[ -n "$ROBOFLOW_API_KEY" ]]; then
  echo "[INFO] Roboflow API key detected"
else
  echo "[WARN] ROBOFLOW_API_KEY is empty; Roboflow model endpoint may return 401."
fi

SSH_COMMON_OPTS=(
  -o ConnectTimeout=5
  -o ControlMaster=auto
  -o ControlPersist=10m
  -o ControlPath="$SSH_CONTROL_PATH"
)

if command -v sshpass >/dev/null 2>&1; then
  USE_SSHPASS="1"
  PASS_ESCAPED="$(printf '%q' "$PI_PASSWORD")"
  RSYNC_SSH_CMD="sshpass -p $PASS_ESCAPED ssh -o ControlMaster=auto -o ControlPersist=10m -o ControlPath=$SSH_CONTROL_PATH"
else
  USE_SSHPASS="0"
  RSYNC_SSH_CMD="ssh -o ControlMaster=auto -o ControlPersist=10m -o ControlPath=$SSH_CONTROL_PATH"
  echo "[WARN] sshpass not found, SSH may prompt for password interactively."
fi

run_ssh() {
  if [[ "$USE_SSHPASS" == "1" ]]; then
    sshpass -p "$PI_PASSWORD" ssh "${SSH_COMMON_OPTS[@]}" "$@"
  else
    ssh "${SSH_COMMON_OPTS[@]}" "$@"
  fi
}

run_ssh_tty() {
  if [[ "$USE_SSHPASS" == "1" ]]; then
    sshpass -p "$PI_PASSWORD" ssh -t "${SSH_COMMON_OPTS[@]}" "$@"
  else
    ssh -t "${SSH_COMMON_OPTS[@]}" "$@"
  fi
}

mkdir -p "$(dirname "$SSH_CONTROL_PATH")"

cleanup() {
  run_ssh -O exit "$PI_USER@$PI_IP" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[1/10] Test SSH connectivity: $PI_USER@$PI_IP"
run_ssh -MNf "$PI_USER@$PI_IP"
run_ssh "$PI_USER@$PI_IP" "echo '[OK] SSH connected to ' \"\$(hostname)\""

echo "[2/10] Ensure remote directories exist"
run_ssh "$PI_USER@$PI_IP" "mkdir -p $REMOTE_PROJECT_ROOT $REMOTE_WS"

echo "[3/10] Sync local workspace to Raspberry Pi"
rsync -avz --delete \
  --exclude '.git' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude 'src/vision_opencv-rolling' \
  -e "$RSYNC_SSH_CMD" \
  "$LOCAL_WS/" "$PI_USER@$PI_IP:$REMOTE_WS/"

echo "[4/10] Sync config.json to Raspberry Pi"
rsync -avz \
  -e "$RSYNC_SSH_CMD" \
  "$LOCAL_CONFIG" "$PI_USER@$PI_IP:$REMOTE_CONFIG"

echo "[5/10] Verify remote package path"
run_ssh "$PI_USER@$PI_IP" "test -d $REMOTE_WS/src/$LAUNCH_PACKAGE"

echo "[6/10] Ensure Python OpenCV runtime exists on Raspberry Pi"
run_ssh "$PI_USER@$PI_IP" "bash -lc '
  if python3 -c \"import cv2\" >/dev/null 2>&1; then
    echo \"[OK] python3-opencv already available\"
  else
    echo \"[ERROR] Missing Python module cv2.\"
    echo \"[ERROR] Use offline wheels copied from Mac (recommended for this project).\"
    exit 1
  fi
'"

echo "[7/10] Ensure ROS cv_bridge runtime exists on Raspberry Pi"
run_ssh "$PI_USER@$PI_IP" "bash -lc '
  source /opt/ros/$ROS_DISTRO/setup.bash
  if [[ \"$PREFER_SOURCE_CV_BRIDGE\" == \"1\" ]]; then
    if [[ ! -d $REMOTE_VISION_OPENCV_SRC_DIR ]]; then
      if [[ -f $REMOTE_VISION_OPENCV_ARCHIVE ]]; then
        echo \"[INFO] Restoring vision_opencv source from offline archive\"
        mkdir -p $REMOTE_WS/src
        tar -xzf $REMOTE_VISION_OPENCV_ARCHIVE -C $REMOTE_WS/src
      else
        echo \"[WARN] Source cv_bridge requested but archive not found: $REMOTE_VISION_OPENCV_ARCHIVE\"
      fi
    fi

    if [[ -d $REMOTE_VISION_OPENCV_SRC_DIR ]]; then
      echo \"[OK] Source cv_bridge available: $REMOTE_VISION_OPENCV_SRC_DIR\"
    elif python3 -c \"import cv_bridge\" >/dev/null 2>&1; then
      echo \"[OK] System cv_bridge importable\"
    else
      echo \"[ERROR] cv_bridge missing and source package unavailable.\"
      echo \"[ERROR] Upload archive to $REMOTE_VISION_OPENCV_ARCHIVE from Mac.\"
      exit 1
    fi
  elif python3 -c \"import cv_bridge\" >/dev/null 2>&1; then
    echo \"[OK] System cv_bridge importable\"
  else
    echo \"[ERROR] Missing Python module cv_bridge.\"
    echo \"[ERROR] Set PREFER_SOURCE_CV_BRIDGE=1 and provide offline vision_opencv archive.\"
    exit 1
  fi
'"

echo "[8/10] Ensure ToF runtime and initialize I2C addresses on Raspberry Pi"
set +e
TOF_STEP_OUTPUT="$(run_ssh "$PI_USER@$PI_IP" \
  INIT_TOF_ON_DEPLOY="$INIT_TOF_ON_DEPLOY" \
  TOF_FRONT_XSHUT_GPIO="$TOF_FRONT_XSHUT_GPIO" \
  TOF_LEFT_XSHUT_GPIO="$TOF_LEFT_XSHUT_GPIO" \
  TOF_RIGHT_XSHUT_GPIO="$TOF_RIGHT_XSHUT_GPIO" \
  TOF_REAR_XSHUT_GPIO="$TOF_REAR_XSHUT_GPIO" \
  TOF_FRONT_ADDR="$TOF_FRONT_ADDR" \
  TOF_LEFT_ADDR="$TOF_LEFT_ADDR" \
  TOF_RIGHT_ADDR="$TOF_RIGHT_ADDR" \
  TOF_REAR_ADDR="$TOF_REAR_ADDR" \
  TOF_BOOT_DELAY_MS="$TOF_BOOT_DELAY_MS" \
  "bash -s" <<'REMOTE_TOF_SCRIPT' 2>&1
if ! python3 -m pip install --break-system-packages --user --disable-pip-version-check -q smbus2 gpiozero; then
  echo "[ERROR] Failed to install ToF Python dependencies"
  exit 1
fi
if ! python3 -c "import smbus2, gpiozero" >/dev/null 2>&1; then
  echo "[ERROR] ToF runtime import check failed (smbus2/gpiozero)"
  exit 1
fi

if [[ "${INIT_TOF_ON_DEPLOY}" == "1" ]]; then
  timeout 30s python3 -u - <<'PY'
import os
import time
import smbus2
from gpiozero import OutputDevice

VL53L0X_DEFAULT_ADDR = 0x29
VL53L0X_ADDR_REG = 0x8A
I2C_BUS = 1

boot_delay = max(10, int(os.environ.get('TOF_BOOT_DELAY_MS', '80'))) / 1000.0
order = [
    ('front', int(os.environ.get('TOF_FRONT_XSHUT_GPIO', '26')), int(os.environ.get('TOF_FRONT_ADDR', '0x30'), 0)),
    ('left', int(os.environ.get('TOF_LEFT_XSHUT_GPIO', '16')), int(os.environ.get('TOF_LEFT_ADDR', '0x31'), 0)),
    ('right', int(os.environ.get('TOF_RIGHT_XSHUT_GPIO', '20')), int(os.environ.get('TOF_RIGHT_ADDR', '0x32'), 0)),
    ('rear', int(os.environ.get('TOF_REAR_XSHUT_GPIO', '21')), int(os.environ.get('TOF_REAR_ADDR', '0x33'), 0)),
]

pins = {}
for name, gpio, _ in order:
    try:
        pins[name] = OutputDevice(gpio, active_high=True, initial_value=False)
    except Exception as exc:
        raise SystemExit(
            f'[ERROR] failed to claim XSHUT GPIO{gpio} for {name}: {exc}. '
            'This GPIO may be busy/reserved. Kill stale python3 and ensure no GPIO conflict.'
        )
time.sleep(0.1)

bus = smbus2.SMBus(I2C_BUS)

def i2c_scan():
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    return sorted(found)

print('[INFO] ToF init: all XSHUT LOW, begin one-by-one address assignment')
print(f'[INFO] initial I2C scan: {[hex(x) for x in i2c_scan()]}', flush=True)

for name, gpio, target in order:
    pins[name].on()
    time.sleep(boot_delay)

    before = i2c_scan()
    print(f'[INFO] assigning {name} (GPIO{gpio}) -> 0x{target:02X}  |  scan before: {[hex(x) for x in before]}', flush=True)

    if VL53L0X_DEFAULT_ADDR not in before:
        raise SystemExit(
            f'[ERROR] {name}: sensor not found at 0x{VL53L0X_DEFAULT_ADDR:02X} after XSHUT HIGH. '
            'Check XSHUT wiring or increase TOF_BOOT_DELAY_MS.'
        )

    try:
        bus.write_byte_data(VL53L0X_DEFAULT_ADDR, VL53L0X_ADDR_REG, target & 0x7F)
    except OSError as exc:
        raise SystemExit(
            f'[ERROR] {name}: I2C write to 0x{VL53L0X_DEFAULT_ADDR:02X} reg 0x{VL53L0X_ADDR_REG:02X} failed: {exc}. '
            'Check XSHUT wiring/power.'
        )

    time.sleep(0.02)
    after = i2c_scan()
    print(f'[OK] {name}: 0x{VL53L0X_DEFAULT_ADDR:02X} -> 0x{target:02X}  |  scan after: {[hex(x) for x in after]}', flush=True)

addresses = i2c_scan()
target_set = {target for (_, _, target) in order}
print('[INFO] I2C scan after init:', ' '.join(f'0x{x:02X}' for x in addresses))
missing = [f'0x{x:02X}' for x in sorted(target_set) if x not in addresses]
if missing:
    raise SystemExit('[ERROR] Missing ToF addresses after init: ' + ', '.join(missing))

for pin in pins.values():
    pin.close()
bus.close()
print('[OK] ToF initialization complete')
PY
  rc=$?
  if [[ "$rc" -eq 124 ]]; then
    echo "[ERROR] ToF initialization timed out after 30s."
    exit 1
  elif [[ "$rc" -ne 0 ]]; then
    exit "$rc"
  fi
else
  echo "[INFO] Skip ToF initialization (INIT_TOF_ON_DEPLOY=0)"
fi
REMOTE_TOF_SCRIPT
)"
TOF_STEP_RC=$?
set -e

echo "$TOF_STEP_OUTPUT"

if [[ "$TOF_STEP_RC" -ne 0 ]]; then
  if printf '%s' "$TOF_STEP_OUTPUT" | grep -qiE 'failed to claim XSHUT GPIO|GPIO busy'; then
    echo "[WARN] ToF init skipped: XSHUT GPIO is busy (likely already held by running radar_sensor_node)."
    echo "[INFO] Hint: set INIT_TOF_ON_DEPLOY=0 to skip step 8 intentionally."
  else
    exit "$TOF_STEP_RC"
  fi
fi

REMOTE_CMD="source /opt/ros/$ROS_DISTRO/setup.bash && cd $REMOTE_WS"

if [[ -n "$ROBOFLOW_API_KEY" ]]; then
  REMOTE_CMD="export ROBOFLOW_API_KEY=$(printf '%q' "$ROBOFLOW_API_KEY") && $REMOTE_CMD"
fi

if [[ "$SKIP_BUILD" != "1" ]]; then
  REMOTE_CMD+=" && colcon build --symlink-install"
fi

REMOTE_CMD+=" && source install/setup.bash"

if [[ "$START_LOCAL_INFERENCE" == "1" ]]; then
  echo "[9/10] Ensure local AI inference service exists on Raspberry Pi"
  REMOTE_INFERENCE_START_CMD_B64="$(printf '%s' "$REMOTE_INFERENCE_START_CMD" | base64 | tr -d '\n')"
  run_ssh "$PI_USER@$PI_IP" \
    INFERENCE_HOST="$INFERENCE_HOST" \
    INFERENCE_PORT="$INFERENCE_PORT" \
    INFERENCE_READY_TIMEOUT="$INFERENCE_READY_TIMEOUT" \
    INFERENCE_IMAGE_REF="$INFERENCE_IMAGE_REF" \
    REMOTE_INFERENCE_IMAGE_ARCHIVE="$REMOTE_INFERENCE_IMAGE_ARCHIVE" \
    ROBOFLOW_API_KEY="$ROBOFLOW_API_KEY" \
    REMOTE_INFERENCE_START_CMD_B64="$REMOTE_INFERENCE_START_CMD_B64" \
    "bash -s" <<'REMOTE_INFERENCE_SCRIPT'
REMOTE_INFERENCE_START_CMD="$(printf '%s' "${REMOTE_INFERENCE_START_CMD_B64}" | base64 -d)"

check_inference_port() {
  python3 - <<'PY'
import os
import socket

host = os.environ.get('INFERENCE_HOST', '127.0.0.1')
port = int(os.environ.get('INFERENCE_PORT', '9001'))

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(0.5)
try:
    sock.connect((host, port))
except Exception:
    raise SystemExit(1)
finally:
    sock.close()
raise SystemExit(0)
PY
}

if check_inference_port; then
  echo "[OK] Local inference already available at ${INFERENCE_HOST}:${INFERENCE_PORT}"
  exit 0
fi

echo "[INFO] Local inference not detected, starting service"

if ! command -v docker >/dev/null 2>&1; then
  if echo "${REMOTE_INFERENCE_START_CMD}" | grep -qi "docker"; then
    echo "[ERROR] docker command not found on Raspberry Pi."
    echo "[ERROR] Install Docker or override REMOTE_INFERENCE_START_CMD with a non-docker start command."
    exit 1
  fi
fi

if [[ -f "${REMOTE_INFERENCE_IMAGE_ARCHIVE}" ]]; then
  echo "[INFO] Found offline inference image archive: ${REMOTE_INFERENCE_IMAGE_ARCHIVE}"
  if gzip -t "${REMOTE_INFERENCE_IMAGE_ARCHIVE}"; then
    echo "[OK] Offline image archive integrity check passed"
    if ! docker image inspect "${INFERENCE_IMAGE_REF}" >/dev/null 2>&1; then
      echo "[INFO] Loading inference image from offline archive"
      gunzip -c "${REMOTE_INFERENCE_IMAGE_ARCHIVE}" | docker load
    else
      echo "[OK] Inference image already present: ${INFERENCE_IMAGE_REF}"
    fi
  else
    echo "[ERROR] Offline image archive is corrupted or incomplete: ${REMOTE_INFERENCE_IMAGE_ARCHIVE}"
    echo "[ERROR] Please re-transfer the archive to Raspberry Pi."
    exit 1
  fi
else
  echo "[WARN] Offline image archive not found: ${REMOTE_INFERENCE_IMAGE_ARCHIVE}"
  echo "[WARN] Will rely on online pull when starting container."
fi

if docker ps -a --format '{{.Names}}' | grep -qx 'unibots-roboflow-inference'; then
  if ! docker ps --format '{{.Names}}' | grep -qx 'unibots-roboflow-inference'; then
    echo "[INFO] Starting existing container: unibots-roboflow-inference"
    docker start unibots-roboflow-inference >/dev/null
  fi
else
  eval "${REMOTE_INFERENCE_START_CMD}"
fi

if [[ -n "${ROBOFLOW_API_KEY:-}" ]] && echo "${REMOTE_INFERENCE_START_CMD}" | grep -q "docker run"; then
  if ! docker inspect unibots-roboflow-inference --format '{{range .Config.Env}}{{println .}}{{end}}' | grep -Fx "ROBOFLOW_API_KEY=${ROBOFLOW_API_KEY}" >/dev/null 2>&1; then
    echo "[INFO] Recreating inference container with ROBOFLOW_API_KEY"
    docker rm -f unibots-roboflow-inference >/dev/null 2>&1 || true
    REMOTE_INFERENCE_START_CMD_WITH_KEY="${REMOTE_INFERENCE_START_CMD/docker run/docker run -e ROBOFLOW_API_KEY=\"${ROBOFLOW_API_KEY}\"}"
    eval "${REMOTE_INFERENCE_START_CMD_WITH_KEY}"
  fi
fi

python3 - <<'PY'
import os
import socket
import time

host = os.environ.get('INFERENCE_HOST', '127.0.0.1')
port = int(os.environ.get('INFERENCE_PORT', '9001'))
timeout_sec = int(os.environ.get('INFERENCE_READY_TIMEOUT', '30'))

deadline = time.monotonic() + max(1, timeout_sec)
while time.monotonic() < deadline:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.5)
    try:
        sock.connect((host, port))
        print(f"[OK] Local inference ready at {host}:{port}")
        raise SystemExit(0)
    except Exception:
        time.sleep(1.0)
    finally:
        sock.close()

print(f"[ERROR] Local inference did not become ready within {timeout_sec}s")
raise SystemExit(1)
PY
REMOTE_INFERENCE_SCRIPT
else
  echo "[9/10] Skip local AI inference startup (START_LOCAL_INFERENCE=0)"
fi

if [[ "$MODE" == "detach" ]]; then
  LOG_FILE='~/unibots_ros.log'
  REMOTE_CMD+=" && nohup ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE > $LOG_FILE 2>&1 & echo [OK] launched in background, log: $LOG_FILE"
  echo "[10/10] Build and launch (background)"
  run_ssh "$PI_USER@$PI_IP" "bash -lc '$REMOTE_CMD'"
  echo "Done. To view logs: ssh $PI_USER@$PI_IP 'tail -f ~/unibots_ros.log'"
else
  REMOTE_CMD+=" && ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE"
  echo "[10/10] Build and launch (foreground)"
  echo "Press Ctrl+C to stop launch on Raspberry Pi."
  run_ssh_tty "$PI_USER@$PI_IP" "bash -lc '$REMOTE_CMD'"
fi
