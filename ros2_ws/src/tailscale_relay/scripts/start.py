"""
start.py — One-button startup for the AutoWarehouseBot
======================================================
Launches the full system: Pi hardware nodes + relay server, then
laptop autonomy nodes + relay client. No CycloneDDS, no firewall
holes, no IP injection. Just WebSocket over Tailscale.

Usage:
  python3 start.py             # normal mode (dashboard)
  python3 start.py --verbose   # stream all logs
"""

import os
import sys
import subprocess
import time
import signal
from datetime import datetime

# Configuration
PI_IP = "100.91.37.52"
PI_USER = "ece_441"
PI_PASS = "group4pi"
PI_ROS_SETUP = (
    "source /opt/ros/kilted/setup.bash && "
    "source /home/ece_441/AutoWarehouseBot/ros2_ws/install/setup.bash 2>/dev/null && "
    "source /home/ece_441/AutoWarehouseBot/install/setup.bash 2>/dev/null"
)

local_processes = []


def banner(msg):
    print("\n" + "=" * 50)
    print(f"  {msg}")
    print("=" * 50)


def tprint(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")


def ssh_cmd(cmd, stream=False):
    """Run a command on the Pi via SSH."""
    full = f"sshpass -p '{PI_PASS}' ssh -o StrictHostKeyChecking=no {PI_USER}@{PI_IP} \"{cmd}\""
    if stream:
        return subprocess.Popen(full, shell=True)
    return subprocess.run(full, shell=True, capture_output=True, text=True)


def cleanup(sig, frame):
    banner("Shutting Down")
    tprint("Stopping local autonomy nodes...")
    for p in local_processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass

    tprint("Stopping Pi hardware nodes...")
    ssh_cmd("pkill -f ros2 2>/dev/null || true")

    print("\n  All systems stopped. Goodbye!\n")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)


def start_pi(verbose=False):
    banner("Step 1/2 — Starting Pi Hardware + Relay Server")

    tprint(f"Connecting to {PI_IP}...")
    result = ssh_cmd("echo OK")
    if result.returncode != 0:
        print("  [!] ERROR: Cannot reach Pi. Is Tailscale running?")
        sys.exit(1)

    tprint("Checking hardware (/dev/lidar, /dev/arduino)...")
    result = ssh_cmd("ls -1 /dev/lidar /dev/arduino 2>/dev/null || true")
    if "/dev/lidar" not in result.stdout:
        print("  [!] WARNING: LiDAR missing")
    if "/dev/arduino" not in result.stdout:
        print("  [!] WARNING: Arduino missing")

    tprint("Killing old ROS2 nodes...")
    ssh_cmd("pkill -f ros2 2>/dev/null; sleep 0.5")

    launch_cmd = f"{PI_ROS_SETUP} && ros2 launch embedded robot_bringup.launch.py"

    if verbose:
        tprint("Streaming Pi logs (Verbose Mode)...")
        proc = ssh_cmd(f"bash -c '{launch_cmd}'", stream=True)
        local_processes.append(proc)
    else:
        tprint("Launching robot_bringup.launch.py (includes relay_server @ :8765)...")
        ssh_cmd(f"bash -c 'nohup bash -c \"{launch_cmd}\" > /tmp/robot_bringup.log 2>&1 &'")
        time.sleep(2)
        tprint("Pi nodes active. [Logs at /tmp/robot_bringup.log on Pi]")


def start_wsl_nodes(verbose=False):
    banner("Step 2/2 — Starting Laptop Nodes + Relay Client")
    LOG_DIR = "/tmp/warehousebot"
    os.makedirs(LOG_DIR, exist_ok=True)

    # Find the ros2_ws install directory relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # scripts/start.py -> tailscale_relay/ -> src/ -> ros2_ws/
    ws_root = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))

    setup = (
        f"source /opt/ros/kilted/setup.bash && "
        f"source \\\"{ws_root}/install/setup.bash\\\" 2>/dev/null || true"
    )

    redirect = "" if verbose else f" > {LOG_DIR}/startup.log 2>&1"

    tprint("Launching Navigation + Relay Client...")
    cmd = (
        f"bash -c '{setup} && "
        f"ros2 launch navigation hardware.launch.py "
        f"pi_address:={PI_IP} {redirect}'"
    )
    local_processes.append(
        subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
    )

    if not verbose:
        time.sleep(1)
        tprint("  (Tip: Run with --verbose to see full output)")


def spawn_dashboard():
    """Spawn the dashboard in a separate Windows Terminal window/tab."""
    tprint("Spawning Dashboard in separate terminal...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Simple wt command: wt.exe wsl -e bash -c "cd '/mnt/c/...'; python3 start.py --dashboard"
    # We use double quotes for the inner command and escape the path quotes
    inner_cmd = f"cd \\\"{script_dir}\\\"; python3 start.py --dashboard"
    cmd = f'wt.exe wsl -e bash -c "{inner_cmd}"'
    
    try:
        subprocess.Popen(cmd, shell=True)
    except Exception as e:
        tprint(f"  [!] Failed to spawn dashboard: {e}")


def dashboard():
    """Live dashboard showing topic flow and connection status."""
    # Grouped for cleaner UI
    pi_to_pc = {
        "/scan":            "LiDAR Scan",
        "/camera/image_raw": "Camera Feed",
        "/odom":            "Raw Odometry",
        "/imu/data":        "IMU Data",
        "/ble/target/rssi": "BLE Signal",
    }
    pc_to_pi = {
        "/cmd_vel":             "Motion Commands",
        "/summon/motion_cmd":   "Summon Actions",
        "/summon/ble_target":   "BLE Target Set",
    }
    system = {
        "/odom_filtered":   "EKF Filtered",
        "/tf":              "Transforms",
    }

    try:
        while True:
            res = subprocess.run(
                "ros2 topic list", shell=True, capture_output=True, text=True
            )
            live = res.stdout.splitlines()

            print("\033c", end="") # Clear screen
            print("+" + "=" * 54 + "+")
            print(f"|  AutoWarehouseBot Live Dashboard — {datetime.now().strftime('%H:%M:%S')}   |")
            print("+" + "=" * 54 + "+")
            print(f"|  Network: {PI_IP.ljust(15)} | Status: ONLINE (Relay) |")
            print("+" + "=" * 25 + "+" + "=" * 28 + "+")

            print(f"| [Pi → Laptop] (Sensors)   | Status                     |")
            print("+" + "-" * 25 + "+" + "-" * 28 + "+")
            for topic, desc in pi_to_pc.items():
                status = "* ACTIVE" if topic in live else "  WAITING..."
                print(f"| {desc.ljust(23)} | {status.ljust(26)} |")

            print("+" + "=" * 25 + "+" + "=" * 28 + "+")
            print(f"| [Laptop → Pi] (Commands)  | Status                     |")
            print("+" + "-" * 25 + "+" + "-" * 28 + "+")
            for topic, desc in pc_to_pi.items():
                status = "* READY " if topic in live else "  OFFLINE    "
                print(f"| {desc.ljust(23)} | {status.ljust(26)} |")

            print("+" + "=" * 25 + "+" + "=" * 28 + "+")
            print(f"| [System Transforms]       | Status                     |")
            print("+" + "-" * 25 + "+" + "-" * 28 + "+")
            for topic, desc in system.items():
                status = "* SYNCED" if topic in live else "  INITIALIZING"
                print(f"| {desc.ljust(23)} | {status.ljust(26)} |")

            print("+" + "=" * 54 + "+")
            print("| Press Ctrl+C in the main terminal to shutdown.         |")
            print("+" + "=" * 54 + "+")
            time.sleep(2)
    except KeyboardInterrupt:
        pass


def main():
    # Parse arguments
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    is_dashboard = "--dashboard" in sys.argv

    if is_dashboard:
        dashboard()
        return

    banner("AutoWarehouseBot Startup Bridge")
    tprint(f"Network: {PI_IP} (Pi via Tailscale)")
    
    start_pi(verbose)
    start_wsl_nodes(verbose)

    # In normal or verbose mode, we spawn the dashboard in a new window
    time.sleep(1)
    spawn_dashboard()

    if verbose:
        tprint("System running. Monitor logs here; use the new window for Dashboard.")
        # Keep main thread alive for the subprocesses
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            cleanup(None, None)
    else:
        tprint("System running. Initializing local dashboard...")
        dashboard()


if __name__ == "__main__":
    main()
