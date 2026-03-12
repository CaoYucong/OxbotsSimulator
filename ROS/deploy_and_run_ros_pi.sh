#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_WS_DEFAULT="$SCRIPT_DIR/ros2_ws"

PI_USER="${PI_USER:-caoyucong}"
PI_IP="${PI_IP:-192.168.50.2}"
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
  PI_USER, PI_IP, ROS_DISTRO, REMOTE_WS, LOCAL_WS, LOCAL_CONFIG,
  REMOTE_PROJECT_ROOT, REMOTE_CONFIG, LAUNCH_PACKAGE, LAUNCH_FILE,
  PREFER_SOURCE_CV_BRIDGE, REMOTE_OFFLINE_ROOT, REMOTE_VISION_OPENCV_ARCHIVE,
  REMOTE_VISION_OPENCV_SRC_DIR

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

echo "[1/8] Test SSH connectivity: $PI_USER@$PI_IP"
ssh "${SSH_COMMON_OPTS[@]}" -MNf "$PI_USER@$PI_IP"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "echo '[OK] SSH connected to ' \"\$(hostname)\""

echo "[2/8] Ensure remote directories exist"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "mkdir -p $REMOTE_PROJECT_ROOT $REMOTE_WS"

echo "[3/8] Sync local workspace to Raspberry Pi"
rsync -avz --delete \
  --exclude '.git' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude 'src/vision_opencv-rolling' \
  -e "ssh -o ControlMaster=auto -o ControlPersist=10m -o ControlPath=$SSH_CONTROL_PATH" \
  "$LOCAL_WS/" "$PI_USER@$PI_IP:$REMOTE_WS/"

echo "[4/8] Sync config.json to Raspberry Pi"
rsync -avz \
  -e "ssh -o ControlMaster=auto -o ControlPersist=10m -o ControlPath=$SSH_CONTROL_PATH" \
  "$LOCAL_CONFIG" "$PI_USER@$PI_IP:$REMOTE_CONFIG"

echo "[5/8] Verify remote package path"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "test -d $REMOTE_WS/src/$LAUNCH_PACKAGE"

echo "[6/8] Ensure Python OpenCV runtime exists on Raspberry Pi"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '
  if python3 -c \"import cv2\" >/dev/null 2>&1; then
    echo \"[OK] python3-opencv already available\"
  else
    echo \"[ERROR] Missing Python module cv2.\"
    echo \"[ERROR] Use offline wheels copied from Mac (recommended for this project).\"
    exit 1
  fi
'"

echo "[7/8] Ensure ROS cv_bridge runtime exists on Raspberry Pi"
ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '
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

if [[ "$SKIP_BUILD" != "1" ]]; then
  REMOTE_CMD+=" && colcon build --symlink-install"
fi

REMOTE_CMD+=" && source install/setup.bash"

if [[ "$MODE" == "detach" ]]; then
  LOG_FILE='~/unibots_ros.log'
  REMOTE_CMD+=" && nohup ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE > $LOG_FILE 2>&1 & echo [OK] launched in background, log: $LOG_FILE"
  echo "[8/8] Build and launch (background)"
  ssh "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '$REMOTE_CMD'"
  echo "Done. To view logs: ssh $PI_USER@$PI_IP 'tail -f ~/unibots_ros.log'"
else
  REMOTE_CMD+=" && ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE"
  echo "[8/8] Build and launch (foreground)"
  echo "Press Ctrl+C to stop launch on Raspberry Pi."
  ssh -t "${SSH_COMMON_OPTS[@]}" "$PI_USER@$PI_IP" "bash -lc '$REMOTE_CMD'"
fi
