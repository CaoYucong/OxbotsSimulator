#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

PI_USER="${PI_USER:-caoyucong}"
PI_IP="${PI_IP:-192.168.50.2}"

LOCAL_PROJECT_DIR="${LOCAL_PROJECT_DIR:-$ROOT_DIR}"
REMOTE_PROJECT_ROOT="${REMOTE_PROJECT_ROOT:-~/OxbotsSimulator}"

WEB_ENTRY="${WEB_ENTRY:-rpi5/web_runtime/run_web_runtime.py}"
WEB_HZ="${WEB_HZ:-10}"
WEB_REMOTE_HOST="${WEB_REMOTE_HOST:-192.168.50.1}"
WEB_REMOTE_PORT="${WEB_REMOTE_PORT:-5003}"
WEB_LOCAL_HOST="${WEB_LOCAL_HOST:-127.0.0.1}"
WEB_LOCAL_PORT="${WEB_LOCAL_PORT:-5003}"
WEB_DISABLE_POSE="${WEB_DISABLE_POSE:-0}"

CONTROL_PATH_DEFAULT="$HOME/.ssh/cm-%C"
SSH_CONTROL_PATH="${SSH_CONTROL_PATH:-$CONTROL_PATH_DEFAULT}"

MODE="foreground"
SKIP_COMPILE="0"

usage() {
  cat <<'EOF'
Usage:
  ./rpi5/deploy_and_run_web_pi.sh [--detach] [--skip-compile] [--disable-pose]

Options:
  --detach        Upload + compile check, then run in background on Raspberry Pi
                  (logs -> ~/unibots_web_runtime.log)
  --skip-compile  Skip python compileall step
  --disable-pose  Start runtime with --disable-pose
  -h, --help      Show this help

Environment overrides:
  PI_USER, PI_IP,
  LOCAL_PROJECT_DIR, REMOTE_PROJECT_ROOT,
  WEB_ENTRY, WEB_HZ,
  WEB_REMOTE_HOST, WEB_REMOTE_PORT,
  WEB_LOCAL_HOST, WEB_LOCAL_PORT,
  WEB_DISABLE_POSE

Examples:
  ./rpi5/deploy_and_run_web_pi.sh
  PI_IP=192.168.50.23 WEB_REMOTE_HOST=192.168.50.1 ./rpi5/deploy_and_run_web_pi.sh --detach
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --detach)
      MODE="detach"
      shift
      ;;
    --skip-compile)
      SKIP_COMPILE="1"
      shift
      ;;
    --disable-pose)
      WEB_DISABLE_POSE="1"
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

if [[ ! -d "$LOCAL_PROJECT_DIR" ]]; then
  echo "[ERROR] Local project dir not found: $LOCAL_PROJECT_DIR"
  exit 1
fi

if [[ ! -f "$LOCAL_PROJECT_DIR/$WEB_ENTRY" ]]; then
  echo "[ERROR] Web runtime entry not found: $LOCAL_PROJECT_DIR/$WEB_ENTRY"
  exit 1
fi

if [[ ! -f "$LOCAL_PROJECT_DIR/config.json" ]]; then
  echo "[ERROR] Local config file not found: $LOCAL_PROJECT_DIR/config.json"
  exit 1
fi

SSH_COMMON_OPTS=(
  -o ConnectTimeout=5
  -o ControlMaster=auto
  -o ControlPersist=10m
  -o ControlPath="$SSH_CONTROL_PATH"
)

mkdir -p "$(dirname "$SSH_CONTROL_PATH")"

cleanup() {
  ssh "${SSH_COMMON_OPTS[@]}" -O exit "$PI_USER@$PI_IP" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[1/7] Test SSH connectivity: $PI_USER@$PI_IP"
ssh "${SSH_COMMON_OPTS[@]}" -MNf "$PI_USER@$PI_IP"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "echo '[OK] SSH connected to ' \"\$(hostname)\""

echo "[2/7] Ensure remote project directory exists"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "mkdir -p $REMOTE_PROJECT_ROOT"

echo "[3/7] Sync project to Raspberry Pi"
rsync -avz --delete \
  --exclude '.git' \
  --exclude '.venv' \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude 'cache' \
  --exclude 'benchmarks' \
  -e "ssh -o ControlMaster=auto -o ControlPersist=10m -o ControlPath=$SSH_CONTROL_PATH" \
  "$LOCAL_PROJECT_DIR/" "$PI_USER@$PI_IP:$REMOTE_PROJECT_ROOT/"

echo "[4/7] Ensure Python runtime dependencies"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '
  if python3 -c \"import cv2, numpy\" >/dev/null 2>&1; then
    echo \"[OK] python3-opencv and numpy already available\"
  elif sudo -n true >/dev/null 2>&1; then
    echo \"[INFO] Installing python3-opencv\"
    sudo -n apt-get update
    sudo -n apt-get install -y python3-opencv
    python3 -c \"import cv2, numpy\" >/dev/null 2>&1
    echo \"[OK] python3-opencv ready\"
  else
    echo \"[ERROR] Missing cv2/numpy and passwordless sudo is unavailable.\"
    echo \"[ERROR] Run once on Raspberry Pi: sudo apt-get update && sudo apt-get install -y python3-opencv\"
    exit 1
  fi
'"

echo "[5/7] Verify runtime entry exists on Raspberry Pi"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "test -f $REMOTE_PROJECT_ROOT/$WEB_ENTRY"

echo "[6/7] Compile check (python -m compileall)"
if [[ "$SKIP_COMPILE" != "1" ]]; then
  ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '
    cd $REMOTE_PROJECT_ROOT
    python3 -m compileall -q \
      rpi5/web_runtime \
      decision_making_ros \
      pose_estimation
    echo \"[OK] compileall passed\"
  '"
else
  echo "[INFO] Skipping compile step"
fi

WEB_DISABLE_POSE_FLAG=""
if [[ "$WEB_DISABLE_POSE" == "1" ]]; then
  WEB_DISABLE_POSE_FLAG="--disable-pose"
fi

REMOTE_RUN_CMD="cd $REMOTE_PROJECT_ROOT && python3 $WEB_ENTRY \
  --hz $WEB_HZ \
  --remote-host $WEB_REMOTE_HOST \
  --remote-port $WEB_REMOTE_PORT \
  --local-host $WEB_LOCAL_HOST \
  --local-port $WEB_LOCAL_PORT \
  $WEB_DISABLE_POSE_FLAG"

if [[ "$MODE" == "detach" ]]; then
  LOG_FILE='~/unibots_web_runtime.log'
  echo "[7/7] Run web runtime (background)"
  ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '$REMOTE_RUN_CMD > $LOG_FILE 2>&1 & echo [OK] web runtime launched in background, log: $LOG_FILE'"
  echo "Done. To view logs: ssh $PI_USER@$PI_IP 'tail -f ~/unibots_web_runtime.log'"
else
  echo "[7/7] Run web runtime (foreground)"
  echo "Press Ctrl+C to stop runtime on Raspberry Pi."
  ssh -t "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '$REMOTE_RUN_CMD'"
fi
