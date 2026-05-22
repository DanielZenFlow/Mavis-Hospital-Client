import argparse
import glob
import os
import re
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_LEVEL_DIR = "complevels26"
DEFAULT_OUTPUT_FILE = "benchmark_results_complevels26.md"
DEFAULT_RAW_LOG_DIR = "target/benchmark-complevels26-logs"
DEFAULT_SERVER_JAR = "server.jar"
DEFAULT_TIMEOUT_SECONDS = 180
DEFAULT_HEARTBEAT_SECONDS = 10
DEFAULT_CLIENT_CMD = "java -Xmx4g -cp target/classes mapf.client.Client"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run MAvis benchmark levels with foreground progress."
    )
    parser.add_argument("--level-dir", default=DEFAULT_LEVEL_DIR)
    parser.add_argument("--output", default=DEFAULT_OUTPUT_FILE)
    parser.add_argument("--raw-log-dir", default=DEFAULT_RAW_LOG_DIR)
    parser.add_argument("--server-jar", default=DEFAULT_SERVER_JAR)
    parser.add_argument("--client-cmd", default=DEFAULT_CLIENT_CMD)
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--heartbeat", type=int, default=DEFAULT_HEARTBEAT_SECONDS)
    parser.add_argument("--limit", type=int, default=0, help="Run only the first N levels.")
    return parser.parse_args()


def parse_server_output(output):
    solved = "No"
    actions = "N/A"
    duration = "N/A"

    for line in output.splitlines():
        if "Level solved:" in line:
            solved = line.split("Level solved:", 1)[1].strip(" .")
        elif "Actions used:" in line:
            actions = line.split("Actions used:", 1)[1].strip(" .")
        elif "Time to solve:" in line:
            duration = line.split("Time to solve:", 1)[1].split("seconds", 1)[0].strip()

    return solved, actions, duration


def git_value(*args):
    result = subprocess.run(["git", *args], capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def safe_log_name(level_path):
    name = Path(level_path).name
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name) + ".log"


def format_duration(seconds):
    seconds = int(max(0, seconds))
    minutes, sec = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours}h{minutes:02d}m{sec:02d}s"
    if minutes:
        return f"{minutes}m{sec:02d}s"
    return f"{sec}s"


def format_eta(completed, total, elapsed):
    if completed <= 0:
        return "unknown"
    remaining = total - completed
    if remaining <= 0:
        return "0s"
    return format_duration((elapsed / completed) * remaining)


def read_log(log_path):
    if not log_path.exists():
        return ""
    return log_path.read_text(encoding="utf-8", errors="replace")


def run_level(level_path, index, total, success_count, args, raw_log_dir, benchmark_started):
    level_name = Path(level_path).name
    log_path = raw_log_dir / safe_log_name(level_path)
    command = [
        "java",
        "-jar",
        args.server_jar,
        "-l",
        level_path,
        "-c",
        args.client_cmd,
        "-t",
        str(args.timeout),
    ]

    level_started = time.monotonic()
    next_heartbeat = level_started

    print(f"\n[{index}/{total}] START {level_name}", flush=True)
    print(f"    log: {log_path}", flush=True)

    timed_out = False
    with log_path.open("w", encoding="utf-8", errors="replace") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )

        while True:
            return_code = process.poll()
            now = time.monotonic()
            level_elapsed = now - level_started
            total_elapsed = now - benchmark_started

            if return_code is not None:
                break

            if level_elapsed > args.timeout + 10:
                timed_out = True
                process.kill()
                return_code = process.wait()
                break

            if now >= next_heartbeat:
                eta = format_eta(index - 1, total, total_elapsed)
                print(
                    f"[{index}/{total}] RUNNING {level_name} "
                    f"level={format_duration(level_elapsed)} "
                    f"total={format_duration(total_elapsed)} "
                    f"score={success_count}/{index - 1} "
                    f"eta={eta}",
                    flush=True,
                )
                next_heartbeat = now + max(1, args.heartbeat)

            time.sleep(1)

    wall_seconds = time.monotonic() - level_started
    output = read_log(log_path)
    solved, actions, duration = parse_server_output(output)
    note = ""

    if timed_out:
        solved = "Timeout"
        actions = "N/A"
        duration = f">{args.timeout}"
        note = "process timeout"
    elif return_code != 0:
        note = f"exit {return_code}"

    print(
        f"[{index}/{total}] DONE {level_name} "
        f"solved={solved} actions={actions} "
        f"time={duration}s wall={wall_seconds:.1f}s {note}",
        flush=True,
    )

    return {
        "level": level_name,
        "solved": solved,
        "actions": actions,
        "duration": duration,
        "wall": f"{wall_seconds:.1f}",
        "note": note,
        "log": str(log_path).replace("\\", "/"),
    }


def validate_inputs(args):
    missing = [
        path
        for path in (args.server_jar, "target/classes", args.level_dir)
        if not os.path.exists(path)
    ]
    if missing:
        print(f"Missing prerequisite(s): {', '.join(missing)}")
        return False
    return True


def write_results(output_file, rows, args, branch, commit, started_at, raw_log_dir):
    success_count = sum(1 for row in rows if row["solved"] == "Yes")

    with open(output_file, "w", encoding="utf-8") as handle:
        handle.write("# Benchmark Results - complevels26\n\n")
        handle.write(f"**Date:** {started_at}\n")
        handle.write(f"**Branch:** `{branch}`\n")
        handle.write(f"**Commit:** `{commit}`\n")
        handle.write(f"**Level dir:** `{args.level_dir}`\n")
        handle.write(f"**Timeout:** {args.timeout}s per level\n")
        handle.write(f"**Raw logs:** `{raw_log_dir}`\n")
        handle.write(f"**Score:** {success_count}/{len(rows)}\n\n")
        handle.write("| Level | Solved | Actions | Time (s) | Wall (s) | Note | Log |\n")
        handle.write("| :--- | :---: | :---: | :---: | :---: | :--- | :--- |\n")
        for row in rows:
            handle.write(
                f"| {row['level']} | {row['solved']} | {row['actions']} | "
                f"{row['duration']} | {row['wall']} | {row['note']} | "
                f"`{row['log']}` |\n"
            )

    return success_count


def main():
    args = parse_args()
    print(f"Current working directory: {os.getcwd()}")
    print(f"Python version: {sys.version.split()[0]}")

    if not validate_inputs(args):
        return 1

    levels = sorted(glob.glob(os.path.join(args.level_dir, "*.lvl")))
    if args.limit > 0:
        levels = levels[: args.limit]

    if not levels:
        print(f"No .lvl files found in {args.level_dir}")
        return 1

    raw_log_dir = Path(args.raw_log_dir)
    raw_log_dir.mkdir(parents=True, exist_ok=True)

    branch = git_value("branch", "--show-current")
    commit = git_value("rev-parse", "--short", "HEAD")
    started_at = time.strftime("%Y-%m-%d %H:%M:%S")
    benchmark_started = time.monotonic()

    print(f"Branch: {branch}")
    print(f"Commit: {commit}")
    print(f"Level dir: {args.level_dir}")
    print(f"Levels: {len(levels)}")
    print(f"Timeout: {args.timeout}s per level")
    print(f"Heartbeat: {args.heartbeat}s")
    print(f"Output: {args.output}")

    rows = []
    success_count = 0
    for index, level_path in enumerate(levels, start=1):
        row = run_level(
            level_path,
            index,
            len(levels),
            success_count,
            args,
            raw_log_dir,
            benchmark_started,
        )
        rows.append(row)
        if row["solved"] == "Yes":
            success_count += 1

        elapsed = time.monotonic() - benchmark_started
        eta = format_eta(index, len(levels), elapsed)
        print(
            f"[{index}/{len(levels)}] PROGRESS score={success_count}/{index} "
            f"elapsed={format_duration(elapsed)} eta={eta}",
            flush=True,
        )

    score = write_results(args.output, rows, args, branch, commit, started_at, raw_log_dir)
    elapsed = time.monotonic() - benchmark_started

    print(f"\nBenchmark complete. Results saved to {args.output}")
    print(f"Score: {score}/{len(rows)}")
    print(f"Elapsed: {format_duration(elapsed)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
