import os
import glob
import subprocess
import re
import sys
import time

# Configuration
LEVEL_DIR = "complevels"
OUTPUT_FILE = "benchmark_results.md"
SERVER_JAR = "server.jar"
TIMEOUT_SECONDS = 180
# Detect OS for classpath separator if needed (not needed for single path, but good practice)
# CP_SEP = ";" if os.name == 'nt' else ":"
CLIENT_CMD = 'java -Xmx4g -cp target/classes mapf.client.Client'

def parse_output(output):
    """
    Parses the server output to extract metrics.
    Expected output lines:
    [server][info] Level solved: Yes.
    [server][info] Actions used: 692.
    [server][info] Time to solve: 1.275 seconds.
    """
    solved = "No"
    actions = "N/A"
    duration = "N/A"
    
    for line in output.splitlines():
        if "Level solved:" in line:
            solved = line.split("Level solved:")[1].strip(" .")
        if "Actions used:" in line:
            actions = line.split("Actions used:")[1].strip(" .")
        if "Time to solve:" in line:
            duration = line.split("Time to solve:")[1].split("seconds")[0].strip()
            
    return solved, actions, duration

def run_level(level_path):
    cmd = [
        "java", "-jar", SERVER_JAR,
        "-l", level_path,
        "-c", CLIENT_CMD,
        "-t", str(TIMEOUT_SECONDS),
        # "-g"  # Disabled for batch/CI to ensure headless execution
    ]
    
    print(f"Running {level_path}...", end="", flush=True)
    
    try:
        # Capture strictly stdout/stderr
        result = subprocess.run(
            cmd, 
            capture_output=True, 
            text=True, 
            timeout=TIMEOUT_SECONDS + 5
        )
        
        # Combine stdout and stderr for parsing
        full_output = result.stdout + result.stderr
        
        # Basic check if it crashed
        if result.returncode != 0:
            print(" Crashed/Error!")
            return "Error", "N/A", "N/A"
            
        solved, actions, duration = parse_output(full_output)
        print(f" {solved} ({duration}s)")
        return solved, actions, duration
        
    except subprocess.TimeoutExpired:
        print(" Timeout!")
        return "Timeout", "N/A", ">" + str(TIMEOUT_SECONDS)
    except Exception as e:
        print(f" Exception: {e}")
        return "Exception", "N/A", "N/A"

def main():
    # Verify prerequisites
    print(f"Current working directory: {os.getcwd()}")
    print(f"Python version: {sys.version}")
    
    # Check server.jar exists
    if not os.path.exists(SERVER_JAR):
        print(f"ERROR: {SERVER_JAR} not found!")
        sys.exit(1)
    
    # Check target/classes exists (compiled Java code)
    if not os.path.exists("target/classes"):
        print("ERROR: target/classes not found! Did Maven compile succeed?")
        sys.exit(1)
    
    # Find all .lvl files
    levels = sorted(glob.glob(os.path.join(LEVEL_DIR, "*.lvl")))
    
    if not levels:
        print(f"No levels found in {LEVEL_DIR}")
        print(f"Directory exists: {os.path.exists(LEVEL_DIR)}")
        if os.path.exists(LEVEL_DIR):
            print(f"Contents: {os.listdir(LEVEL_DIR)}")
        sys.exit(1)

    print(f"Found {len(levels)} levels. Starting benchmark...")
    
    results = []
    
    # Header for MD file
    results.append("| Level | Solved | Actions | Time (s) |")
    results.append("| :--- | :---: | :---: | :---: |")
    
    success_count = 0
    
    for lvl in levels:
        level_name = os.path.basename(lvl)
        solved, actions, duration = run_level(lvl)
        
        # Markdown table row
        results.append(f"| {level_name} | {solved} | {actions} | {duration} |")
        
        if solved == "Yes":
            success_count += 1
            
    # Write to file
    try:
        with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
            f.write(f"# Benchmark Results\n")
            f.write(f"**Date:** {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"**Score:** {success_count}/{len(levels)}\n\n")
            f.write("\n".join(results))
        
        print(f"\nBenchmark complete. Results saved to {OUTPUT_FILE}")
        print(f"File exists: {os.path.exists(OUTPUT_FILE)}")
        print(f"File size: {os.path.getsize(OUTPUT_FILE)} bytes")
    except Exception as e:
        print(f"ERROR writing results file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
