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
  REMOTE_INFERENCE_IMAGE_ARCHIVE, REMOTE_INFERENCE_START_CMD, ROBOFLOW_API_KEY

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

echo "[1/9] Test SSH connectivity: $PI_USER@$PI_IP"
run_ssh -MNf "$PI_USER@$PI_IP"
run_ssh "$PI_USER@$PI_IP" "echo '[OK] SSH connected to ' \"\$(hostname)\""

echo "[2/9] Ensure remote directories exist"
run_ssh "$PI_USER@$PI_IP" "mkdir -p $REMOTE_PROJECT_ROOT $REMOTE_WS"

echo "[3/9] Sync local workspace to Raspberry Pi"
rsync -avz --delete \
  --exclude '.git' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude 'src/vision_opencv-rolling' \
  -e "$RSYNC_SSH_CMD" \
  "$LOCAL_WS/" "$PI_USER@$PI_IP:$REMOTE_WS/"

echo "[4/9] Sync config.json to Raspberry Pi"
rsync -avz \
  -e "$RSYNC_SSH_CMD" \
  "$LOCAL_CONFIG" "$PI_USER@$PI_IP:$REMOTE_CONFIG"

echo "[5/9] Verify remote package path"
run_ssh "$PI_USER@$PI_IP" "test -d $REMOTE_WS/src/$LAUNCH_PACKAGE"

echo "[6/9] Ensure Python OpenCV runtime exists on Raspberry Pi"
run_ssh "$PI_USER@$PI_IP" "bash -lc '
  if python3 -c \"import cv2\" >/dev/null 2>&1; then
    echo \"[OK] python3-opencv already available\"
  else
    echo \"[ERROR] Missing Python module cv2.\"
    echo \"[ERROR] Use offline wheels copied from Mac (recommended for this project).\"
    exit 1
  fi
'"

echo "[7/9] Ensure ROS cv_bridge runtime exists on Raspberry Pi"
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

REMOTE_CMD="source /opt/ros/$ROS_DISTRO/setup.bash && cd $REMOTE_WS"

if [[ -n "$ROBOFLOW_API_KEY" ]]; then
  REMOTE_CMD="export ROBOFLOW_API_KEY=$(printf '%q' "$ROBOFLOW_API_KEY") && $REMOTE_CMD"
fi

if [[ "$SKIP_BUILD" != "1" ]]; then
  REMOTE_CMD+=" && colcon build --symlink-install"
fi

REMOTE_CMD+=" && source install/setup.bash"

if [[ "$START_LOCAL_INFERENCE" == "1" ]]; then
  echo "[8/9] Ensure local AI inference service exists on Raspberry Pi"
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
  echo "[8/9] Skip local AI inference startup (START_LOCAL_INFERENCE=0)"
fi

if [[ "$MODE" == "detach" ]]; then
  LOG_FILE='~/unibots_ros.log'
  REMOTE_CMD+=" && nohup ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE > $LOG_FILE 2>&1 & echo [OK] launched in background, log: $LOG_FILE"
  echo "[9/9] Build and launch (background)"
  run_ssh "$PI_USER@$PI_IP" "bash -lc '$REMOTE_CMD'"
  echo "Done. To view logs: ssh $PI_USER@$PI_IP 'tail -f ~/unibots_ros.log'"
else
  REMOTE_CMD+=" && ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE"
  echo "[9/9] Build and launch (foreground)"
  echo "Press Ctrl+C to stop launch on Raspberry Pi."
  run_ssh_tty "$PI_USER@$PI_IP" "bash -lc '$REMOTE_CMD'"
fi
