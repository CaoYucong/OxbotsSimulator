#!/usr/bin/env bash
# rpi_img.sh
# macOS: 备份与恢复 Raspberry Pi SD 卡镜像（含压缩与校验）
# 用法：
#   备份: sudo ./rpi_img.sh -b -d /dev/disk4 [-o /path/to/out.img.gz]
#   恢复: sudo ./rpi_img.sh -r -d /dev/disk4 -f /path/to/out.img.gz
#
# 注意：请务必确认 -d 指向 SD 卡（使用 `diskutil list` 先核对），错误的磁盘号会导致数据全部丢失。

set -euo pipefail

progname=$(basename "$0")
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
DEFAULT_OUT="$HOME/Desktop/RaspiBackup/raspberrypi_backup_${TIMESTAMP}.img.gz"

usage(){
  cat <<EOF
Usage:
  Backup SD -> compressed image:
    sudo $progname -b -d /dev/diskX [-o /path/to/output.img.gz]

  Restore image -> SD:
    sudo $progname -r -d /dev/diskX -f /path/to/input.img.gz

Options:
  -b            create backup (backup mode)
  -r            restore (restore mode)
  -d DEVICE     device (e.g. /dev/disk4)  <- REQUIRED
  -o OUTFILE    backup output file (for -b). Default: $DEFAULT_OUT
  -f INFILE     input image file to restore (for -r)
  -h            show this help

Important:
  * Always run 'diskutil list' BEFORE using this script to confirm the correct device.
  * This script WILL DESTROY data on the target device when restoring.
EOF
  exit 1
}

# parse args
MODE=""
DEVICE=""
OUTFILE=""
INFILE=""

while getopts "brd:o:f:h" opt; do
  case $opt in
    b) MODE="backup" ;;
    r) MODE="restore" ;;
    d) DEVICE="$OPTARG" ;;
    o) OUTFILE="$OPTARG" ;;
    f) INFILE="$OPTARG" ;;
    h|*) usage ;;
  esac
done

if [[ -z "$MODE" ]]; then
  echo "必须指定 -b (backup) 或 -r (restore)"
  usage
fi
if [[ -z "$DEVICE" ]]; then
  echo "必须指定目标设备 -d /dev/diskN"
  usage
fi

# normalize device: allow /dev/disk4 or disk4 etc.
if [[ "$DEVICE" =~ ^/dev/rdisk ]]; then
  RDEVICE="$DEVICE"
elif [[ "$DEVICE" =~ ^/dev/disk ]]; then
  RDEVICE="${DEVICE/disk/rdisk}"
elif [[ "$DEVICE" =~ ^rdisk ]]; then
  RDEVICE="/dev/$DEVICE"
elif [[ "$DEVICE" =~ ^disk ]]; then
  RDEVICE="/dev/${DEVICE/disk/rdisk}"
else
  RDEVICE="/dev/$DEVICE"
fi

# helper
confirm_type_yes(){
  echo
  echo "!!! 注意：这一步会影响设备： $RDEVICE"
  echo "如果你确定无误并继续，请输入 YES（区分大小写）然后回车："
  read -r ans
  if [[ "$ans" != "YES" ]]; then
    echo "已取消。没有对设备进行任何更改。"
    exit 2
  fi
}

has_cmd(){
  command -v "$1" >/dev/null 2>&1
}

# check running as root (dd needs sudo)
if [[ "$(id -u)" -ne 0 ]]; then
  echo "请以 root（sudo）运行此脚本，例如： sudo $progname ..."
  exit 3
fi

# check disk exists
if ! [ -b "$RDEVICE" ] && ! [ -e "$RDEVICE" ]; then
  echo "设备 $RDEVICE 不存在。请先用 'diskutil list' 确认。"
  exit 4
fi

# ensure diskutil unmount will work
echo "目标设备（raw）: $RDEVICE"

# choose pv or not
USE_PV=0
if has_cmd pv; then
  USE_PV=1
fi

if [[ "$MODE" == "backup" ]]; then
  OUTFILE=${OUTFILE:-$DEFAULT_OUT}
  echo "备份模式 -> 将 $RDEVICE 完整镜像并压缩为 $OUTFILE"
  echo
  echo "请先确认你想备份的设备是 SD 卡（用 'diskutil list' 检查）。"
  confirm_type_yes

  echo "卸载设备上的所有分区..."
  diskutil unmountDisk "$RDEVICE" || { echo "unmountDisk 失败，继续但请小心"; }

  # build dd -> gzip command
  if [[ $USE_PV -eq 1 ]]; then
    echo "使用 pv 显示实时进度（系统检测到 pv 已安装）"
    # dd to stdout then pipe to pv -> gzip
    dd if="$RDEVICE" bs=4m status=progress conv=sync,noerror | pv -s $(diskutil info "$RDEVICE" | awk '/Disk Size:/ {print $3$4}' | tr -d '()') 2>/dev/null | gzip -c > "$OUTFILE"
  else
    echo "pv 未安装，使用 dd status=progress -> gzip"
    dd if="$RDEVICE" bs=4m status=progress conv=sync,noerror | gzip -c > "$OUTFILE"
  fi

  sync
  echo "写入并压缩完成： $OUTFILE"

  echo "生成 SHA256 校验文件..."
  if has_cmd shasum; then
    shasum -a 256 "$OUTFILE" > "${OUTFILE}.sha256"
    echo "校验值保存在 ${OUTFILE}.sha256"
  elif has_cmd sha256sum; then
    sha256sum "$OUTFILE" > "${OUTFILE}.sha256"
    echo "校验值保存在 ${OUTFILE}.sha256"
  else
    echo "系统上没有 shasum/sha256sum，请手动生成校验值。"
  fi

  echo "备份完成。建议将镜像复制到其他存储（外置盘或 NAS）作为异地备份。"

elif [[ "$MODE" == "restore" ]]; then
  if [[ -z "${INFILE:-}" ]]; then
    echo "恢复模式需要 -f /path/to/image.img.gz 参数"
    usage
  fi
  if [[ ! -f "$INFILE" ]]; then
    echo "指定的镜像文件 $INFILE 不存在"
    exit 5
  fi

  echo "恢复模式 -> 将镜像 $INFILE 写回设备 $RDEVICE"
  echo "警告：此操作会清除 $RDEVICE 上的所有数据。"
  confirm_type_yes

  echo "卸载设备上的所有分区..."
  diskutil unmountDisk "$RDEVICE" || { echo "unmountDisk 失败，继续但请小心"; }

  # choose decompress tool
  if has_cmd pigz; then
    DECOMP="pigz -dc"
  else
    DECOMP="gzip -dc"
  fi

  if [[ $USE_PV -eq 1 ]]; then
    echo "使用 pv 管道显示写入进度（pv 已安装）"
    # estimate size for pv: use file size
    FILESIZE=$(stat -f%z "$INFILE" 2>/dev/null || stat -c%s "$INFILE")
    $DECOMP "$INFILE" | pv -s "$FILESIZE" | dd of="$RDEVICE" bs=4m status=progress conv=sync,noerror
  else
    echo "pv 未安装，使用 gzip -dc -> dd"
    $DECOMP "$INFILE" | dd of="$RDEVICE" bs=4m status=progress conv=sync,noerror
  fi

  sync
  echo "恢复写入完成。请安全弹出设备："
  diskutil eject "$RDEVICE" || echo "eject 失败，手动拔出设备"
  echo "完成。将 SD 卡插回 Raspberry Pi 并开机测试。"
fi

exit 0s