#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
webots_auto_loop_crossplatform.py

跨平台（macOS / Linux / Windows）自动循环脚本：
启动 Webots 加载 world，实时读取 stdout/stderr，等待目标文本（默认 "Supervisor controller exiting."）
出现后优雅终止/重启，重复执行。

用法示例：
  python3 webots_auto_loop_crossplatform.py
  python3 webots_auto_loop_crossplatform.py --webots "/Applications/Webots.app/Contents/MacOS/webots" \
    --world "~/Desktop/OxbotsSimulator/worlds/Decision_making.wbt" --max-runs 10 --delay-after 2

矩阵测试示例：
    python3 webots_auto_loop_crossplatform.py --benchmark-matrix \
        --world "~/Desktop/OxbotsSimulator/worlds/Decision_making.wbt" \
        --modes realistic_nearest improved_nearest nearest planned \
        --avoidance off 0 1 2 3 4 5 6 7 8 9 \
        --seed-start 1000 --seed-end 1029 \
        --result-csv "~/Desktop/OxbotsSimulator/tools/testing/benchmark_results.csv"

注意：
- 请确保 Supervisor 控制器会把目标文本输出到 stdout 或 stderr（区分大小写）。
- 在 Windows 上，若通过 PowerShell 运行，注意路径引号。
"""

import argparse
import csv
import subprocess
import sys
import time
import threading
import queue
import os
import signal
import atexit
from datetime import datetime
from typing import Optional, Tuple, TextIO

# 默认路径（你之前给出的 mac 路径 & world）
DEFAULT_WEBOTS_MAC = "/Applications/Webots.app/Contents/MacOS/webots"
DEFAULT_WEBOTS_LINUX = "/usr/local/webots/bin/webots"
DEFAULT_WEBOTS_WINDOWS = r"C:\Program Files\Webots\webots.exe"

WATCH_TEXT = "Supervisor controller exiting."
DEFAULT_SHARED_OUTPUT_FILE = os.path.expanduser("~/Desktop/webots_auto_loop_shared_output.txt")

_ORIGINAL_STDOUT = sys.stdout
_ORIGINAL_STDERR = sys.stderr
_SHARED_OUTPUT_HANDLE: Optional[TextIO] = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class TeeStream:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            try:
                stream.write(data)
            except Exception:
                pass
        return len(data)

    def flush(self):
        for stream in self.streams:
            try:
                stream.flush()
            except Exception:
                pass

    def isatty(self):
        try:
            return any(getattr(stream, "isatty", lambda: False)() for stream in self.streams)
        except Exception:
            return False


def setup_shared_output(shared_output_file: Optional[str]) -> None:
    global _SHARED_OUTPUT_HANDLE

    if not shared_output_file:
        return

    shared_output_file = os.path.expanduser(os.path.expandvars(shared_output_file))
    os.makedirs(os.path.dirname(shared_output_file), exist_ok=True)

    _SHARED_OUTPUT_HANDLE = open(shared_output_file, "a", buffering=1, encoding="utf-8")
    _SHARED_OUTPUT_HANDLE.write(f"\n===== session start {datetime.now().isoformat()} =====\n")
    _SHARED_OUTPUT_HANDLE.flush()

    sys.stdout = TeeStream(_ORIGINAL_STDOUT, _SHARED_OUTPUT_HANDLE)
    sys.stderr = TeeStream(_ORIGINAL_STDERR, _SHARED_OUTPUT_HANDLE)

    def _close_shared_output():
        try:
            if _SHARED_OUTPUT_HANDLE and not _SHARED_OUTPUT_HANDLE.closed:
                _SHARED_OUTPUT_HANDLE.write(f"\n===== session end {datetime.now().isoformat()} =====\n")
                _SHARED_OUTPUT_HANDLE.flush()
                _SHARED_OUTPUT_HANDLE.close()
        except Exception:
            pass

    atexit.register(_close_shared_output)
    print(f"[INFO] 已启用共享输出日志：{shared_output_file}")


def resolve_project_root() -> str:
    candidates = [
        SCRIPT_DIR,
        os.path.abspath(os.path.join(SCRIPT_DIR, "..")),
        os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..")),
    ]
    for candidate in candidates:
        if (
            os.path.isdir(os.path.join(candidate, "controllers"))
            and os.path.isdir(os.path.join(candidate, "worlds"))
        ):
            return candidate
    return os.path.abspath(os.path.join(SCRIPT_DIR, ".."))


PROJECT_ROOT = resolve_project_root()
DEFAULT_WORLD = os.path.join(PROJECT_ROOT, "worlds", "Decision_making.wbt")

DEFAULT_MODE_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "mode.txt")
DEFAULT_COLLISION_CONFIG_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "collision_avoiding.txt")
DEFAULT_PLANNED_INDEX_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data", "planned_waypoints_index.txt")
DEFAULT_BALL_HISTORY_FILE = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "real_time_data", "ball_taken_history.txt")
DEFAULT_COLLISION_COUNTER_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data", "collision_counter.txt")
DEFAULT_TOTAL_CONTACT_TIME_FILE = os.path.join(PROJECT_ROOT, "decision_making_cyc", "real_time_data", "total_contact_time.txt")
DEFAULT_SUPERVISOR_STATUS_FILE = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "real_time_data", "supervisor_controller_status.txt")
DEFAULT_RANDOM_SEED_FILE = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "random_seed.txt")

DEFAULT_BENCHMARK_MODES = ["realistic_nearest", "improved_nearest","nearest"]
DEFAULT_AVOIDANCE_SETTINGS = ["100"]


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def log_info(msg: str) -> None:
    print(f"[{_ts()}][INFO] {msg}")


def log_warn(msg: str) -> None:
    print(f"[{_ts()}][WARN] {msg}")


def log_error(msg: str) -> None:
    print(f"[{_ts()}][ERROR] {msg}")


def log_step(msg: str) -> None:
    print(f"[{_ts()}][STEP] {msg}")

# ---------- 辅助函数 ----------
def enqueue_stream(stream, q):
    """后台线程：把流逐行放入队列，避免阻塞主线程"""
    try:
        for line in iter(stream.readline, ''):
            if not line:
                break
            q.put(line)
    except Exception:
        pass
    finally:
        try:
            stream.close()
        except Exception:
            pass

def build_default_webots_path() -> str:
    """根据平台返回默认 webots 可执行路径（优先使用存在的路径）"""
    candidates = []
    if sys.platform.startswith("darwin"):
        candidates = [DEFAULT_WEBOTS_MAC, DEFAULT_WEBOTS_LINUX, DEFAULT_WEBOTS_WINDOWS]
    elif sys.platform.startswith("win"):
        candidates = [DEFAULT_WEBOTS_WINDOWS, DEFAULT_WEBOTS_MAC, DEFAULT_WEBOTS_LINUX]
    else:
        candidates = [DEFAULT_WEBOTS_LINUX, DEFAULT_WEBOTS_MAC, DEFAULT_WEBOTS_WINDOWS]

    for p in candidates:
        if os.path.isabs(p) and os.path.exists(p):
            return p
    # 若都不存在，返回第一个候选（让用户覆盖）
    return candidates[0]

def run_and_wait_for_text(webots_bin: str, world_file: str, extra_args: Optional[list]=None,
                          timeout: Optional[float]=None, log_file: Optional[str]=None,
                          status_file: Optional[str]=None,
                          status_running_values: Optional[list]=None,
                          status_done_values: Optional[list]=None
                         ) -> Tuple[bool, subprocess.Popen]:
    """
    启动 webots 进程并监听输出，直到状态文件判定结束（返回 True）或进程自然结束/超时（返回 False）。
    返回 (found_flag, proc) —— proc 是 subprocess.Popen 实例（可能已结束）。
    """
    cmd = [webots_bin, world_file]
    if extra_args:
        cmd += extra_args

    log_step(f"启动 Webots 进程，命令：{' '.join(cmd)}")

    nowstr = datetime.now().strftime("%Y%m%d_%H%M%S")
    logfile = None
    if log_file:
        logfile = open(log_file, "a", buffering=1, encoding="utf-8")
        logfile.write(f"\n===== run start {nowstr} cmd: {' '.join(cmd)} =====\n")

    # Windows: 为了能发送 CTRL_BREAK_EVENT，需要在创建进程时设置 CREATE_NEW_PROCESS_GROUP
    creationflags = 0
    start_new_session = False
    if sys.platform.startswith("win"):
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    else:
        # Unix 系统可以考虑 start_new_session=True 来隔离进程组（等同 setsid）
        start_new_session = True

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            creationflags=creationflags,
            start_new_session=start_new_session
        )
        log_info(f"Webots 已启动，pid={proc.pid}")
    except FileNotFoundError as e:
        raise RuntimeError(f"无法启动 Webots，可执行文件找不到：{webots_bin}") from e
    except Exception as e:
        raise RuntimeError(f"启动 Webots 失败：{e}") from e

    q = queue.Queue()
    t = threading.Thread(target=enqueue_stream, args=(proc.stdout, q), daemon=True)
    t.start()

    running_values = {v.strip().lower() for v in (status_running_values or ["runnung", "running"]) if str(v).strip()}
    done_values = {v.strip().lower() for v in (status_done_values or ["exited"]) if str(v).strip()}
    seen_running_status = False
    last_status_value = None

    if status_file:
        log_info(
            f"使用状态文件判断结束：status_file={status_file}, running={sorted(running_values)}, done={sorted(done_values)}"
        )

    def _read_status(path: Optional[str]) -> Optional[str]:
        if not path:
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                raw = f.read().strip()
            if not raw:
                return None
            return raw
        except Exception:
            return None

    start_time = time.time()
    found = False
    last_heartbeat = start_time

    try:
        while True:
            try:
                line = q.get(timeout=0.2)
            except queue.Empty:
                line = None

            if line is not None:
                # 输出到终端与日志
                print(line, end='')
                if logfile:
                    logfile.write(line)

            current_status = _read_status(status_file)
            if current_status is not None and current_status != last_status_value:
                log_info(f"状态文件变化：{last_status_value} -> {current_status}")
                last_status_value = current_status

            normalized_status = current_status.strip().lower() if current_status else None
            if normalized_status in running_values and not seen_running_status:
                seen_running_status = True
                log_info(f"检测到运行状态：{current_status}")

            if seen_running_status and normalized_status in done_values:
                log_info(f"检测到结束状态：{current_status}")
                found = True
                break

            # 兼容旧逻辑：若 stdout 出现目标文本也视为结束
            if line is not None and WATCH_TEXT in line:
                log_info(f"检测到目标文本：{WATCH_TEXT}")
                found = True
                break

            # 检查进程状态
            ret = proc.poll()
            if ret is not None:
                log_info(f"Webots 进程结束，returncode={ret}")
                # 进程已退出，继续吐尽队列
                while not q.empty():
                    extra = q.get_nowait()
                    print(extra, end='')
                    if logfile:
                        logfile.write(extra)
                break

            # 检查超时
            if timeout is not None and (time.time() - start_time) > timeout:
                log_warn(f"超时：超过 {timeout} 秒仍未检测到结束状态/目标文本。")
                break

            now = time.time()
            if now - last_heartbeat >= 60:
                elapsed = now - start_time
                status_msg = last_status_value if last_status_value is not None else "<unknown>"
                log_info(f"运行中... 已等待 {elapsed:.1f}s，当前状态={status_msg}，尚未结束")
                last_heartbeat = now

        return found, proc
    finally:
        if logfile:
            logfile.write(f"\n===== run end {datetime.now().strftime('%Y%m%d_%H%M%S')} =====\n")
            logfile.flush()
            logfile.close()

def terminate_process(proc: subprocess.Popen, grace: int = 5):
    """
    尝试优雅终止 proc：
    - Windows: 先发送 CTRL_BREAK_EVENT（需要子进程在新进程组），然后 terminate，再 kill。
    - Unix: 先发送 SIGINT，再 terminate/kll。
    """
    if proc is None:
        return

    pid = getattr(proc, "pid", None)
    if pid is None:
        return

    is_running = proc.poll() is None

    try:
        if sys.platform.startswith("win"):
            # Windows：优先让当前进程优雅退出，再用 taskkill /T 清理子进程树。
            if is_running:
                try:
                    proc.send_signal(signal.CTRL_BREAK_EVENT)
                    proc.wait(timeout=grace)
                    return
                except Exception:
                    pass

                try:
                    proc.terminate()
                except Exception:
                    pass

                try:
                    proc.wait(timeout=grace)
                    return
                except subprocess.TimeoutExpired:
                    pass

            try:
                subprocess.run(
                    ["taskkill", "/PID", str(pid), "/T", "/F"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass

            if is_running:
                try:
                    proc.wait(timeout=3)
                except Exception:
                    pass

        else:
            # Unix：按进程组终止，确保 GUI 子进程也被清理。
            try:
                pgid = os.getpgid(pid)
            except Exception:
                pgid = None

            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGINT)
                except Exception:
                    pass

                if is_running:
                    try:
                        proc.wait(timeout=grace)
                        return
                    except subprocess.TimeoutExpired:
                        pass

                try:
                    os.killpg(pgid, signal.SIGTERM)
                except Exception:
                    pass

                if is_running:
                    try:
                        proc.wait(timeout=3)
                        return
                    except subprocess.TimeoutExpired:
                        pass

                try:
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    pass

            if is_running:
                try:
                    proc.kill()
                except Exception:
                    pass
                try:
                    proc.wait(timeout=3)
                except Exception:
                    pass
    except Exception as e:
        # 忽略任何终止中出现的问题，但记录到 stdout
        print(f"[terminate_process] 终止过程中发生异常：{e}")


def read_text_file(path: str) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        log_warn(f"读取文件失败：{path} ({e})")
        return None


def write_text_file(path: str, content: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    log_info(f"写入文件：{path}")


def ensure_file_ends_with_newline(path: str) -> None:
    try:
        if not os.path.exists(path):
            return
        with open(path, "rb+") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            if size == 0:
                return
            f.seek(-1, os.SEEK_END)
            last = f.read(1)
            if last not in (b"\n", b"\r"):
                f.seek(0, os.SEEK_END)
                f.write(b"\n")
                f.flush()
                os.fsync(f.fileno())
                log_warn(f"检测到 CSV 末尾缺少换行，已自动补齐：{path}")
    except Exception as e:
        log_warn(f"检查/修复 CSV 末尾换行失败：{path} ({e})")


def append_csv_row(path: str, fieldnames: list, row: dict) -> None:
    ensure_file_ends_with_newline(path)
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writerow(row)
        f.flush()
        os.fsync(f.fileno())


def parse_ball_taken_history_last_line(path: str) -> Tuple[Optional[float], Optional[int], Optional[str]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]
        if not lines:
            log_warn(f"ball_taken_history 为空：{path}")
            return None, None, None

        last = lines[-1]
        parts = [p.strip() for p in last.split(",")]
        if len(parts) < 2:
            log_warn(f"ball_taken_history 最后一行格式异常：{last}")
            return None, None, last

        last_ball_time = float(parts[0])
        total_balls = int(float(parts[1]))
        return last_ball_time, total_balls, last
    except Exception as e:
        log_warn(f"解析 ball_taken_history 失败：{path} ({e})")
        return None, None, None


def parse_collision_counter(path: str) -> Optional[int]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]

        if not lines:
            log_warn(f"collision_counter 为空：{path}")
            return None

        count_value = None
        for line in lines:
            if line.lower().startswith("count="):
                raw = line.split("=", 1)[1].strip()
                count_value = int(float(raw))
                break

        if count_value is None:
            raw = lines[0]
            if "=" in raw:
                raw = raw.split("=", 1)[1].strip()
            count_value = int(float(raw))

        return count_value
    except Exception as e:
        log_warn(f"解析 collision_counter 失败：{path} ({e})")
        return None


def normalize_collision_counter_file(path: str, count_value: Optional[int]) -> None:
    if count_value is None:
        return
    try:
        write_text_file(path, f"count={int(count_value)}\n")
    except Exception as e:
        log_warn(f"标准化 collision_counter 失败：{path} ({e})")


def parse_total_contact_time(path: str) -> Optional[float]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = f.read().strip()

        if not raw:
            log_warn(f"total_contact_time 为空：{path}")
            return None

        value = float(raw)
        if value < 0:
            value = 0.0
        return value
    except Exception as e:
        log_warn(f"解析 total_contact_time 失败：{path} ({e})")
        return None


def reset_runtime_metric_files(ball_history_file: str,
                               collision_counter_file: str,
                               total_contact_time_file: str,
                               planned_index_file: Optional[str] = None) -> None:
    write_text_file(ball_history_file, "")
    write_text_file(collision_counter_file, "count=0\n")
    write_text_file(total_contact_time_file, "0.000000\n")
    if planned_index_file:
        write_text_file(planned_index_file, "")


def read_runtime_run_config(mode_file: str, collision_config_file: str, random_seed_file: str) -> Tuple[str, str, str]:
    mode_value = "<unknown>"
    avoidance_value = "<unknown>"
    seed_value = "<unknown>"

    try:
        with open(mode_file, "r", encoding="utf-8") as f:
            raw = f.read().strip()
            if raw:
                mode_value = raw
    except Exception:
        pass

    try:
        with open(collision_config_file, "r", encoding="utf-8") as f:
            raw = f.read().strip()
        if raw:
            if raw.lower() == "off":
                avoidance_value = "off"
            else:
                avoidance_value = raw.replace(" ", "")
    except Exception:
        pass

    try:
        with open(random_seed_file, "r", encoding="utf-8") as f:
            raw = f.read().strip()
            if raw:
                seed_value = raw
    except Exception:
        pass

    return mode_value, avoidance_value, seed_value


def run_benchmark_matrix(args) -> int:
    modes = [m.strip() for m in args.modes if str(m).strip()]
    avoidance_settings = [a.strip() for a in args.avoidance if str(a).strip()]

    if not modes:
        log_error("modes 为空，无法执行基准测试。")
        return 1
    if not avoidance_settings:
        log_error("avoidance 为空，无法执行基准测试。")
        return 1

    mode_file = os.path.expanduser(os.path.expandvars(args.mode_file))
    collision_config_file = os.path.expanduser(os.path.expandvars(args.collision_config_file))
    planned_index_file = os.path.expanduser(os.path.expandvars(args.planned_index_file))
    ball_history_file = os.path.expanduser(os.path.expandvars(args.ball_history_file))
    collision_counter_file = os.path.expanduser(os.path.expandvars(args.collision_counter_file))
    total_contact_time_file = os.path.expanduser(os.path.expandvars(args.total_contact_time_file))
    status_file = os.path.expanduser(os.path.expandvars(args.status_file)) if args.status_file else None
    random_seed_file = os.path.expanduser(os.path.expandvars(args.random_seed_file))

    seed_values = []
    if args.seeds and len(args.seeds) > 0:
        seed_values = [int(s) for s in args.seeds]
    else:
        step = 1 if args.seed_end >= args.seed_start else -1
        seed_values = list(range(args.seed_start, args.seed_end + step, step))

    if args.repeats_per_config <= 0:
        log_error("repeats_per_config 必须 > 0")
        return 1
    if not seed_values:
        log_error("未生成任何随机种子，请检查 --seed-start/--seed-end 或 --seeds")
        return 1

    if args.repeats_per_config != len(seed_values):
        log_warn(
            f"检测到 repeats_per_config={args.repeats_per_config} 与种子数量 {len(seed_values)} 不一致；"
            "当前版本按 seed 列表完整运行，不再截断种子。"
        )

    nowstr = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_csv = args.result_csv
    if not result_csv:
        result_csv = os.path.join(SCRIPT_DIR, f"benchmark_results_{nowstr}.csv")
    result_csv = os.path.expanduser(os.path.expandvars(result_csv))

    total_runs = len(modes) * len(avoidance_settings) * len(seed_values)
    log_step(f"开始矩阵测试，总计 {total_runs} 组")
    log_info(f"modes={modes}")
    log_info(f"avoidance={avoidance_settings}")
    log_info(f"repeats_per_config={args.repeats_per_config} (effective_by_seeds={len(seed_values)})")
    log_info(f"seed_values={seed_values}")
    log_info(f"结果 CSV：{result_csv}")
    log_info(f"mode_file={mode_file}")
    log_info(f"collision_config_file={collision_config_file}")
    log_info(f"ball_history_file={ball_history_file}")
    log_info(f"collision_counter_file={collision_counter_file}")
    log_info(f"total_contact_time_file={total_contact_time_file}")
    log_info(f"status_file={status_file}")
    log_info(f"random_seed_file={random_seed_file}")

    fieldnames = [
        "run_index",
        "mode",
        "avoidance",
        "repeat_index",
        "seed",
        "watch_text_found",
        "last_ball_time",
        "total_balls_taken",
        "collision_count",
        "total_contact_time",
        "ball_history_last_line",
        "process_return_code",
        "process_error",
        "started_at",
        "finished_at",
    ]

    csv_dir = os.path.dirname(result_csv) or "."
    os.makedirs(csv_dir, exist_ok=True)
    with open(result_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
    log_info(f"CSV 已初始化（可实时查看）：{result_csv}")

    original_mode_content = read_text_file(mode_file)
    original_collision_content = read_text_file(collision_config_file)
    original_random_seed_content = read_text_file(random_seed_file)

    results = []
    run_count = 0

    try:
        combos_per_seed = len(avoidance_settings) * len(modes)
        for repeat_idx, seed in enumerate(seed_values, start=1):
            log_step(f"开始种子批次 {repeat_idx}/{len(seed_values)}：seed={seed}（将运行 {combos_per_seed} 组）")
            for avoidance_idx, avoidance in enumerate(avoidance_settings, start=1):
                log_step(f"开始避障批次 {avoidance_idx}/{len(avoidance_settings)}：avoidance={avoidance}（mode 共 {len(modes)} 组）")
                for mode_idx, mode in enumerate(modes, start=1):
                    run_count += 1
                    if avoidance.lower() == "off":
                        collision_text = "off\n"
                        avoidance_label = "off"
                        avoidance_csv_value = "-1"
                    else:
                        collision_text = f"smart_factor = {avoidance}\n"
                        avoidance_label = f"smart_factor={avoidance}"
                        avoidance_csv_value = str(avoidance)

                    print(
                        f"\n=== [{run_count}/{total_runs}] mode={mode} collision={avoidance_label} seed={seed} "
                        f"mode_batch={mode_idx}/{len(modes)} ({datetime.now().isoformat()}) ==="
                    )
                    log_step(
                        f"准备测试组合 [{run_count}/{total_runs}]：mode={mode}, collision={avoidance_label}, seed={seed}, "
                        f"mode_batch={mode_idx}/{len(modes)}, avoidance_batch={avoidance_idx}/{len(avoidance_settings)}, "
                        f"seed_batch={repeat_idx}/{len(seed_values)}"
                    )
                    log_info(
                        f"[RUN_CONFIG] mode={mode} × avoidance={avoidance_label} × seed={seed} "
                        f"(mode_batch {mode_idx}/{len(modes)}, avoidance_batch {avoidance_idx}/{len(avoidance_settings)}, "
                        f"seed_batch {repeat_idx}/{len(seed_values)}, global {run_count}/{total_runs})"
                    )

                    write_text_file(mode_file, f"{mode}\n")
                    write_text_file(collision_config_file, collision_text)
                    write_text_file(random_seed_file, f"{seed}\n")
                    reset_runtime_metric_files(
                        ball_history_file=ball_history_file,
                        collision_counter_file=collision_counter_file,
                        total_contact_time_file=total_contact_time_file,
                        planned_index_file=planned_index_file,
                    )

                    started_at = datetime.now().isoformat()
                    found = False
                    return_code = None
                    process_error = ""

                    try:
                        log_step("启动仿真并等待结束信号")
                        found, proc = run_and_wait_for_text(
                            webots_bin=args.webots,
                            world_file=args.world,
                            extra_args=args.extra_args,
                            timeout=args.timeout,
                            log_file=args.log_file,
                            status_file=status_file,
                            status_running_values=args.status_running_values,
                            status_done_values=args.status_done_values,
                        )
                        if proc:
                            log_step("本轮结束后，关闭当前 Webots 进程")
                            terminate_process(proc, grace=5)
                        return_code = proc.returncode if proc else None
                    except RuntimeError as e:
                        process_error = str(e)
                        log_error(f"运行失败：{e}")
                        return_code = -1

                    log_step("采集本轮结果文件")
                    last_ball_time, total_balls, last_history_line = parse_ball_taken_history_last_line(ball_history_file)
                    collision_count = parse_collision_counter(collision_counter_file)
                    total_contact_time = parse_total_contact_time(total_contact_time_file)
                    normalize_collision_counter_file(collision_counter_file, collision_count)

                    row = {
                        "run_index": run_count,
                        "mode": mode,
                        "avoidance": avoidance_csv_value,
                        "repeat_index": repeat_idx,
                        "seed": seed,
                        "watch_text_found": found,
                        "last_ball_time": "" if last_ball_time is None else f"{last_ball_time}",
                        "total_balls_taken": "" if total_balls is None else str(total_balls),
                        "collision_count": "" if collision_count is None else str(collision_count),
                        "total_contact_time": "" if total_contact_time is None else f"{total_contact_time:.6f}",
                        "ball_history_last_line": "" if last_history_line is None else last_history_line,
                        "process_return_code": "" if return_code is None else str(return_code),
                        "process_error": process_error,
                        "started_at": started_at,
                        "finished_at": datetime.now().isoformat(),
                    }
                    results.append(row)

                    append_csv_row(result_csv, fieldnames, row)
                    log_info("CSV 已追加 1 行")

                    log_info(
                        f"本轮结果：seed={seed}, time={row['last_ball_time'] or 'N/A'}, "
                        f"balls={row['total_balls_taken'] or 'N/A'}, "
                        f"collision={row['collision_count'] or 'N/A'}, "
                        f"contact_time={row['total_contact_time'] or 'N/A'}, "
                        f"watch_text_found={found}, return_code={row['process_return_code'] or 'N/A'}"
                    )
                    if process_error:
                        log_error(f"本轮错误信息：{process_error}")

                    time.sleep(max(args.delay_after, args.run_interval))

    except KeyboardInterrupt:
        log_warn("收到 Ctrl-C，提前结束矩阵测试。")
    finally:
        log_step("恢复原始配置文件")
        if original_mode_content is not None:
            write_text_file(mode_file, original_mode_content)
        if original_collision_content is not None:
            write_text_file(collision_config_file, original_collision_content)
        if original_random_seed_content is not None:
            write_text_file(random_seed_file, original_random_seed_content)

    log_step(f"矩阵测试结束，共写入 {len(results)} 条结果")
    log_info(f"结果文件：{result_csv}")
    return 0

# ---------- 主流程 ----------
def main():
    global WATCH_TEXT

    parser = argparse.ArgumentParser(description="跨平台自动循环：启动 webots，等待 Supervisor 输出，然后重启循环。")
    default_webots = build_default_webots_path()
    parser.add_argument("--webots", default=default_webots, help=f"webots 可执行路径（默认尝试：{default_webots}）")
    parser.add_argument("--world", default=DEFAULT_WORLD, help=f"要加载的 .wbt 世界文件路径（默认已填）")
    parser.add_argument("--extra-args", nargs="*", default=[], help="传给 webots 的额外参数，例如 --batch --no-rendering")
    parser.add_argument("--delay-after", type=float, default=1.0, help="检测到文本后，在重启前等待的秒数（默认 1s）")
    parser.add_argument("--max-runs", type=int, default=0, help="最大循环次数，0 表示无限循环")
    parser.add_argument("--run-interval", type=float, default=0.5, help="每次启动间的最小间隔（秒）")
    parser.add_argument("--timeout", type=float, default=None, help="每次运行查找文本的超时时间（秒），空表示不超时")
    parser.add_argument("--log-file", default=None, help="追加日志到此文件（可选）")
    parser.add_argument("--watch-text", default=WATCH_TEXT, help="要匹配的目标文本（默认 'Supervisor controller exiting.'）")
    parser.add_argument("--shared-output-file", default=DEFAULT_SHARED_OUTPUT_FILE, help="将 stdout/stderr 追加写入共享文本文件（默认桌面同一文件）")
    parser.add_argument("--benchmark-matrix", action="store_true", help="按 mode × collision 配置矩阵批量测试并导出 CSV")
    parser.add_argument("--modes", nargs="*", default=DEFAULT_BENCHMARK_MODES, help="矩阵测试使用的 mode 列表")
    parser.add_argument("--avoidance", nargs="*", default=DEFAULT_AVOIDANCE_SETTINGS, help="矩阵测试避障设置列表，例如 off 1 2 3")
    parser.add_argument("--mode-file", default=DEFAULT_MODE_FILE, help="mode.txt 路径")
    parser.add_argument("--collision-config-file", default=DEFAULT_COLLISION_CONFIG_FILE, help="collision_avoiding.txt 路径")
    parser.add_argument("--planned-index-file", default=DEFAULT_PLANNED_INDEX_FILE, help="planned_waypoints_index.txt 路径")
    parser.add_argument("--ball-history-file", default=DEFAULT_BALL_HISTORY_FILE, help="ball_taken_history.txt 路径")
    parser.add_argument("--collision-counter-file", default=DEFAULT_COLLISION_COUNTER_FILE, help="collision_counter.txt 路径")
    parser.add_argument("--total-contact-time-file", default=DEFAULT_TOTAL_CONTACT_TIME_FILE, help="total_contact_time.txt 路径")
    parser.add_argument("--status-file", default=DEFAULT_SUPERVISOR_STATUS_FILE, help="supervisor_controller_status.txt 路径")
    parser.add_argument("--status-running-values", nargs="*", default=["runnung", "running"], help="状态文件中表示运行中的值列表")
    parser.add_argument("--status-done-values", nargs="*", default=["exited"], help="状态文件中表示结束的值列表")
    parser.add_argument("--random-seed-file", default=DEFAULT_RANDOM_SEED_FILE, help="随机种子文件路径（supervisor_controller/random_seed.txt）")
    parser.add_argument("--repeats-per-config", type=int, default=30, help="保留参数；实际重复次数由 seeds 数量决定")
    parser.add_argument("--seed-start", type=int, default=1000, help="随机种子起始值（含）")
    parser.add_argument("--seed-end", type=int, default=1029, help="随机种子结束值（含）")
    parser.add_argument("--seeds", nargs="*", type=int, default=None, help="显式指定种子列表；提供后优先于 seed-start/seed-end")
    parser.add_argument("--result-csv", default=None, help="矩阵测试结果 CSV 路径（默认 testing/benchmark_results_时间戳.csv）")
    args = parser.parse_args()

    setup_shared_output(args.shared_output_file)

    WATCH_TEXT = args.watch_text

    # expand ~ 和 env
    args.world = os.path.expanduser(os.path.expandvars(args.world))
    args.webots = os.path.expanduser(os.path.expandvars(args.webots))
    args.status_file = os.path.expanduser(os.path.expandvars(args.status_file)) if args.status_file else None
    args.mode_file = os.path.expanduser(os.path.expandvars(args.mode_file))
    args.collision_config_file = os.path.expanduser(os.path.expandvars(args.collision_config_file))
    args.random_seed_file = os.path.expanduser(os.path.expandvars(args.random_seed_file))

    if args.benchmark_matrix:
        log_step("进入矩阵测试模式")
        return run_benchmark_matrix(args)

    if not os.path.isabs(args.webots) or not os.path.exists(args.webots):
        print(f"[警告] 指定的 webots 可执行可能不存在：{args.webots}")
        # 仍继续尝试，让用户覆盖或修正

    if not os.path.exists(args.world):
        print(f"[警告] 指定的 world 文件不存在：{args.world}")
        # 仍继续尝试，让用户覆盖或修正

    if args.status_file and not os.path.exists(args.status_file):
        log_warn(f"状态文件当前不存在（启动后会重试读取）：{args.status_file}")

    run_count = 0
    print(f"自动化开始，webots={args.webots} world={args.world}\n状态文件：{args.status_file}\n目标文本（兼容）：{WATCH_TEXT}")

    try:
        while True:
            run_count += 1
            if args.max_runs > 0 and run_count > args.max_runs:
                print("达到最大运行次数，退出。")
                break

            print(f"\n=== 启动第 {run_count} 次运行（{datetime.now().isoformat()}） ===")
            mode_value, avoidance_value, seed_value = read_runtime_run_config(
                mode_file=args.mode_file,
                collision_config_file=args.collision_config_file,
                random_seed_file=args.random_seed_file,
            )
            log_info(
                f"[RUN_CONFIG] mode={mode_value} × avoidance={avoidance_value} × seed={seed_value} "
                f"(run {run_count})"
            )
            try:
                found, proc = run_and_wait_for_text(
                    webots_bin=args.webots,
                    world_file=args.world,
                    extra_args=args.extra_args,
                    timeout=args.timeout,
                    log_file=args.log_file,
                    status_file=args.status_file,
                    status_running_values=args.status_running_values,
                    status_done_values=args.status_done_values,
                )
            except RuntimeError as e:
                print(f"[错误] 无法启动 webots: {e}")
                return 1

            if found:
                print(f"[检测到] 在第 {run_count} 次运行中找到文本：{WATCH_TEXT}")
            else:
                print(f"[未检测到] 进程退出或超时（run_count={run_count}），将尝试终止进程并重启（若进程仍在运行）")

            # 尝试优雅终止仍在运行的进程
            if proc:
                print("[信息] 本轮结束，尝试关闭当前 Webots 进程...")
                terminate_process(proc, grace=5)
            if proc and proc.poll() is None:
                print("[警告] Webots 进程可能仍在运行，请检查系统进程。")
            else:
                if proc:
                    print(f"[信息] 进程已退出，返回码 {proc.returncode}")
                else:
                    print("[信息] 没有可操作的进程句柄。")

            # 等待一会儿再重新启动（避免连续重启过快）
            time.sleep(max(args.delay_after, args.run_interval))
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，正在退出...")
        return 0

    print("自动化结束。")
    return 0

if __name__ == "__main__":
    sys.exit(main())