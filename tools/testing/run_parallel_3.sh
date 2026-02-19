#!/usr/bin/env zsh
set -euo pipefail

WEBOTS="/Applications/Webots.app/Contents/MacOS/webots"

ROOT1="$HOME/Desktop/OxbotsSimulator_run1"
ROOT2="$HOME/Desktop/OxbotsSimulator_run2"
ROOT3="$HOME/Desktop/OxbotsSimulator_run3"
TEMPLATE_ROOT="$HOME/Desktop/OxbotsSimulator"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_BASE="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROOT_PARENT="$(cd "$ROOT_BASE/.." && pwd)"

ROOT1="$ROOT_PARENT/OxbotsSimulator_run1"
ROOT2="$ROOT_PARENT/OxbotsSimulator_run2"
ROOT3="$ROOT_PARENT/OxbotsSimulator_run3"
TEMPLATE_ROOT="$ROOT_BASE"

SCRIPT_REL="tools/testing/webots_auto_loop_crossplatform.py"

MODES=(
  "improved_nearest_v2_5"
  "improved_nearest_v2"
)

AVOIDANCE=(
  "off" "2"
)

SEED1_START=1005
SEED1_END=1006
SEED2_START=1015
SEED2_END=1016
SEED3_START=1025
SEED3_END=1026

PID_FILE="$HOME/Desktop/OxbotsSimulator_parallel_3.pids"
MERGED_CSV_DEFAULT="$HOME/Desktop/benchmark_merged_all.csv"

CSV1="$ROOT1/tools/testing/benchmark_${SEED1_START}_${SEED1_END}.csv"
CSV2="$ROOT2/tools/testing/benchmark_${SEED2_START}_${SEED2_END}.csv"
CSV3="$ROOT3/tools/testing/benchmark_${SEED3_START}_${SEED3_END}.csv"


dir_has_files() {
  local dir="$1"
  [[ -d "$dir" ]] || return 1
  local first_entry
  first_entry=$(find "$dir" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null || true)
  [[ -n "$first_entry" ]]
}

ensure_root_copy() {
  local target="$1"
  local port=""

  case "$target" in
    "$ROOT1") port="5001" ;;
    "$ROOT2") port="5002" ;;
    "$ROOT3") port="5003" ;;
  esac

  if dir_has_files "$target"; then
    return 0
  fi

  if [[ ! -d "$TEMPLATE_ROOT" ]]; then
    echo "模板目录不存在：$TEMPLATE_ROOT"
    echo "请先准备模板工程，或修改脚本顶部 TEMPLATE_ROOT。"
    exit 1
  fi

  echo "检测到目录缺失或为空，开始复制模板：$target"
  mkdir -p "$target"
  cp -R "$TEMPLATE_ROOT"/. "$target"/
  echo "复制完成：$target"

  if [[ -n "$port" ]]; then
    printf '%s\n' "$port" > "$target/controllers/supervisor_controller/html_port.txt"
    echo "已设置 html_port: $port"
  fi
}

bootstrap_roots_if_needed() {
  ensure_root_copy "$ROOT1"
  ensure_root_copy "$ROOT2"
  ensure_root_copy "$ROOT3"
}


bootstrap_roots_if_needed
mkdir -p "$ROOT1/tools/testing" "$ROOT2/tools/testing" "$ROOT3/tools/testing"

start_all() {
  if [[ -f "$PID_FILE" ]]; then
    echo "PID 文件已存在：$PID_FILE"
    echo "可能已经在运行。先执行：$0 stop"
    exit 1
  fi

  echo "启动 3 路并发矩阵测试..."

  python3 "$ROOT1/$SCRIPT_REL" \
    --webots "$WEBOTS" \
    --world "$ROOT1/worlds/Decision_making.wbt" \
    --benchmark-matrix \
    --modes "${MODES[@]}" \
    --avoidance "${AVOIDANCE[@]}" \
    --seed-start "$SEED1_START" --seed-end "$SEED1_END" \
    --result-csv "$CSV1" \
    > "$ROOT1/tools/testing/stdout_${SEED1_START}_${SEED1_END}.log" 2>&1 &
  PID1=$!

  python3 "$ROOT2/$SCRIPT_REL" \
    --webots "$WEBOTS" \
    --world "$ROOT2/worlds/Decision_making.wbt" \
    --benchmark-matrix \
    --modes "${MODES[@]}" \
    --avoidance "${AVOIDANCE[@]}" \
    --seed-start "$SEED2_START" --seed-end "$SEED2_END" \
    --result-csv "$CSV2" \
    > "$ROOT2/tools/testing/stdout_${SEED2_START}_${SEED2_END}.log" 2>&1 &
  PID2=$!

  python3 "$ROOT3/$SCRIPT_REL" \
    --webots "$WEBOTS" \
    --world "$ROOT3/worlds/Decision_making.wbt" \
    --benchmark-matrix \
    --modes "${MODES[@]}" \
    --avoidance "${AVOIDANCE[@]}" \
    --seed-start "$SEED3_START" --seed-end "$SEED3_END" \
    --result-csv "$CSV3" \
    > "$ROOT3/tools/testing/stdout_${SEED3_START}_${SEED3_END}.log" 2>&1 &
  PID3=$!

  {
    echo "$PID1"
    echo "$PID2"
    echo "$PID3"
  } > "$PID_FILE"

  echo "已启动。PIDs: $PID1 $PID2 $PID3"
  echo "查看状态：$0 status"
  echo "查看日志：$0 logs 1|2|3"
  echo "单次合并：$0 merge [out]  单次合并 3 份 benchmark CSV（默认输出到 Desktop）"
  echo "动态合并：$0 merge-watch [out] [sec]  动态合并（默认每 5 秒刷新一次）"
}

stop_all() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "未找到 PID 文件：$PID_FILE"
    echo "可能没有通过本脚本启动任务。"
    exit 0
  fi

  echo "停止并发任务..."
  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      sleep 1
      if kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
      fi
      echo "已停止 PID: $pid"
    else
      echo "PID 不存在或已退出: $pid"
    fi
  done < "$PID_FILE"

  rm -f "$PID_FILE"
  echo "已清理 PID 文件。"
}

status_all() {
  if [[ ! -f "$PID_FILE" ]]; then
    echo "未在运行（找不到 PID 文件：$PID_FILE）"
    exit 0
  fi

  idx=0
  while IFS= read -r pid; do
    idx=$((idx + 1))
    [[ -z "$pid" ]] && continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "[$idx] RUNNING pid=$pid"
    else
      echo "[$idx] EXITED  pid=$pid"
    fi
  done < "$PID_FILE"
}

logs_one() {
  case "${1:-}" in
    1) tail -f "$ROOT1/tools/testing/stdout_${SEED1_START}_${SEED1_END}.log" ;;
    2) tail -f "$ROOT2/tools/testing/stdout_${SEED2_START}_${SEED2_END}.log" ;;
    3) tail -f "$ROOT3/tools/testing/stdout_${SEED3_START}_${SEED3_END}.log" ;;
    *)
      echo "用法: $0 logs 1|2|3"
      exit 1
      ;;
  esac
}

merge_once() {
  output_csv="${1:-$MERGED_CSV_DEFAULT}"
  mkdir -p "$(dirname "$output_csv")"

  python3 - "$output_csv" "$CSV1" "$CSV2" "$CSV3" <<'PY'
import csv
import os
import sys

output = os.path.expanduser(sys.argv[1])
inputs = [os.path.expanduser(p) for p in sys.argv[2:]]

existing = [p for p in inputs if os.path.exists(p)]
if not existing:
    print("未找到可合并的 CSV，跳过。")
    sys.exit(0)

header = None
rows = []

for path in existing:
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            continue
        if header is None:
            header = list(reader.fieldnames)
        elif list(reader.fieldnames) != header:
            print(f"警告：表头不一致，仍尝试按列名并集处理: {path}")
            union = []
            for col in header + list(reader.fieldnames):
                if col not in union:
                    union.append(col)
            header = union

        source = os.path.basename(os.path.dirname(os.path.dirname(path)))
        for row in reader:
            row["source_instance"] = source
            rows.append(row)

if header is None:
    print("CSV 文件存在但没有可用表头，跳过。")
    sys.exit(0)

if "source_instance" not in header:
    header.append("source_instance")

with open(output, "w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=header, lineterminator="\n", extrasaction="ignore")
    writer.writeheader()
    for row in rows:
        normalized = {k: row.get(k, "") for k in header}
        writer.writerow(normalized)

print(f"已合并 {len(rows)} 行 -> {output}")
PY
}

merge_watch() {
  output_csv="${1:-$MERGED_CSV_DEFAULT}"
  interval="${2:-5}"
  echo "动态合并已启动：每 ${interval}s 刷新一次 -> $output_csv"
  echo "按 Ctrl-C 停止。"
  while true; do
    merge_once "$output_csv"
    sleep "$interval"
  done
}

usage() {
  cat <<EOF
用法：
  $0 start        启动 3 路并发（${SEED1_START}-${SEED1_END} / ${SEED2_START}-${SEED2_END} / ${SEED3_START}-${SEED3_END}）
  $0 stop         停止本脚本启动的 3 路任务
  $0 status       查看 3 路任务状态
  $0 logs 1|2|3   跟踪某一路 stdout 日志
  $0 merge [out]  单次合并 3 份 benchmark CSV（默认输出到 Desktop）
  $0 merge-watch [out] [sec]
                  动态合并（默认每 5 秒刷新一次）
EOF
}

cmd="${1:-}"
case "$cmd" in
  start) start_all ;;
  stop) stop_all ;;
  status) status_all ;;
  logs) logs_one "${2:-}" ;;
  merge) merge_once "${2:-$MERGED_CSV_DEFAULT}" ;;
  merge-watch) merge_watch "${2:-$MERGED_CSV_DEFAULT}" "${3:-5}" ;;
  *) usage ;;
esac
