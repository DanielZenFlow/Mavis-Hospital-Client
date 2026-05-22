import glob
import os
import re
import subprocess
import sys
import time


LEVEL_DIR = "complevels"
OUTPUT_FILE = "benchmark_results_current_branch.md"
SERVER_JAR = "server.jar"
TIMEOUT_SECONDS = 180
CLIENT_CMD = "java -Xmx4g -cp target/classes mapf.client.Client"


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
    result = subprocess.run(
        ["git", *args],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def run_level(level_path):
    command = [
        "java",
        "-jar",
        SERVER_JAR,
        "-l",
        level_path,
        "-c",
        CLIENT_CMD,
        "-t",
        str(TIMEOUT_SECONDS),
    ]

    started = time.time()
    print(f"Running {level_path}...", end="", flush=True)

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=TIMEOUT_SECONDS + 10,
        )
    except subprocess.TimeoutExpired:
        elapsed = time.time() - started
        print(f" Timeout ({elapsed:.1f}s)")
        return "Timeout", "N/A", f">{TIMEOUT_SECONDS}", "process timeout"

    output = result.stdout + result.stderr
    solved, actions, duration = parse_server_output(output)

    if result.returncode != 0:
        print(f" Error (exit {result.returncode})")
        return "Error", actions, duration, f"exit {result.returncode}"

    elapsed = time.time() - started
    print(f" {solved} ({duration}s, wall {elapsed:.1f}s)")
    return solved, actions, duration, ""


def main():
    print(f"Current working directory: {os.getcwd()}")
    print(f"Python version: {sys.version.split()[0]}")

    missing = [
        path for path in (SERVER_JAR, "target/classes", LEVEL_DIR)
        if not os.path.exists(path)
    ]
    if missing:
        print(f"Missing prerequisite(s): {', '.join(missing)}")
        return 1

    levels = sorted(glob.glob(os.path.join(LEVEL_DIR, "*.lvl")))
    if not levels:
        print(f"No .lvl files found in {LEVEL_DIR}")
        return 1

    branch = git_value("branch", "--show-current")
    commit = git_value("rev-parse", "--short", "HEAD")
    started_at = time.strftime("%Y-%m-%d %H:%M:%S")

    print(f"Branch: {branch}")
    print(f"Commit: {commit}")
    print(f"Found {len(levels)} levels. Starting benchmark...")

    rows = []
    success_count = 0

    for level in levels:
        level_name = os.path.basename(level)
        solved, actions, duration, note = run_level(level)
        rows.append((level_name, solved, actions, duration, note))
        if solved == "Yes":
            success_count += 1

    with open(OUTPUT_FILE, "w", encoding="utf-8") as handle:
        handle.write("# Benchmark Results - Current Branch\n\n")
        handle.write(f"**Date:** {started_at}\n")
        handle.write(f"**Branch:** `{branch}`\n")
        handle.write(f"**Commit:** `{commit}`\n")
        handle.write(f"**Timeout:** {TIMEOUT_SECONDS}s per level\n")
        handle.write(f"**Score:** {success_count}/{len(levels)}\n\n")
        handle.write("| Level | Solved | Actions | Time (s) | Note |\n")
        handle.write("| :--- | :---: | :---: | :---: | :--- |\n")
        for level_name, solved, actions, duration, note in rows:
            handle.write(f"| {level_name} | {solved} | {actions} | {duration} | {note} |\n")

    print(f"\nBenchmark complete. Results saved to {OUTPUT_FILE}")
    print(f"Score: {success_count}/{len(levels)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
