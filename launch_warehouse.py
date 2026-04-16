#!/usr/bin/env python3
"""
launch_warehouse.py
===================
Windows launcher — starts the full warehouse bot stack from one script.

What it does (in order):
  1. Starts zenohd.exe (Zenoh router) locally on Windows
  2. SSHs into the Pi and launches robot_bringup + zenoh relay
  3. Launches the WSL relay + Nav2 hardware stack via wsl.exe

All processes stream their output to this terminal, colour-coded by machine.
Press Ctrl+C to shut everything down cleanly.

Configuration
-------------
Edit the CONFIG block below before first run. At minimum set:
  ZENOHD_PATH   — path to zenohd.exe on this machine
  PI_HOST       — Pi's IP address on your WiFi
  PI_USER       — Pi SSH username
  WINDOWS_IP    — this machine's WiFi IP (run `ipconfig` to find it)

Usage:
  python launch_warehouse.py
  python launch_warehouse.py --skip-nav2     # relay only, no Nav2
  python launch_warehouse.py --router-only   # just start zenohd, nothing else
"""

import argparse
import subprocess
import sys
import threading
import time
import os

# ── CONFIG — edit these before running ───────────────────────────────────────

CONFIG = {
    # Path to zenohd.exe on this machine.
    "ZENOHD_PATH": r"C:\tools\zenoh\zenohd.exe",  # v1.9.0 msvc, extracted from Downloads

    # Raspberry Pi connection
    "PI_HOST": "104.194.126.139",  # Pi's WiFi IP — update if it changes (hostname -I on Pi)
    "PI_USER": "pi",               # SSH username on Pi
    "PI_SSH_KEY": "group4pi",      # SSH key name in ~/.ssh/ — uses id_ed25519 if blank

    # This Windows machine's WiFi IP — confirmed via ipconfig.
    "WINDOWS_IP": "104.194.115.245",

    # Zenoh router port (default 7447, rarely needs changing)
    "ROUTER_PORT": 7447,

    # ROS2 workspace paths
    "WSL_WS":    "~/ros2_ws",           # native WSL workspace (fast — not /mnt/c)
    "PI_WS":     "~/AutoWarehouseBot",  # workspace root on Pi
    "ROS_DISTRO": "kilted",
}

# ── Colour codes ──────────────────────────────────────────────────────────────

COLOURS = {
    "router": "\033[36m",   # cyan
    "pi":     "\033[32m",   # green
    "wsl":    "\033[33m",   # yellow
    "reset":  "\033[0m",
}


def cprint(machine: str, line: str):
    colour = COLOURS.get(machine, "")
    reset  = COLOURS["reset"]
    print(f"{colour}[{machine:6s}] {line}{reset}", flush=True)


# ── Process management ────────────────────────────────────────────────────────

_procs: list[subprocess.Popen] = []
_threads: list[threading.Thread] = []


def stream_output(proc: subprocess.Popen, label: str):
    """Read stdout/stderr from a process and print it with a machine label."""
    for line in proc.stdout:
        cprint(label, line.rstrip())
    # Process exited
    code = proc.wait()
    if code != 0:
        cprint(label, f"EXIT code={code}")
    else:
        cprint(label, "EXIT clean")


def launch(label: str, cmd: list[str], **kwargs) -> subprocess.Popen:
    """Start a subprocess, stream its output in a daemon thread."""
    cprint(label, f"Starting: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        **kwargs,
    )
    _procs.append(proc)
    t = threading.Thread(target=stream_output, args=(proc, label), daemon=True)
    t.start()
    _threads.append(t)
    return proc


def kill_all():
    """Terminate all managed processes."""
    print("\nShutting down all processes…")
    for p in _procs:
        if p.poll() is None:
            p.terminate()
    for p in _procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    print("Done.")


# ── Individual launchers ──────────────────────────────────────────────────────

def start_router(cfg: dict):
    zenohd = cfg["ZENOHD_PATH"]
    if not os.path.exists(zenohd):
        print(f"[ERROR] zenohd.exe not found at {zenohd}")
        print("  Download from: https://github.com/eclipse-zenoh/zenoh/releases")
        print("  Then update ZENOHD_PATH in this script.")
        sys.exit(1)
    launch("router", [zenohd])
    cprint("router", f"Zenoh router listening on tcp/0.0.0.0:{cfg['ROUTER_PORT']}")
    time.sleep(1.5)  # give router a moment before clients connect


def _wsl_cmd(bash_script: str) -> list[str]:
    """Wrap a bash script to run inside WSL, with the zenoh venv active."""
    venv_activate = "source ~/zenoh_venv/bin/activate"
    return ["wsl.exe", "--", "bash", "-lc", f"{venv_activate} && {bash_script}"]


def _pi_cmd(cfg: dict, bash_script: str) -> list[str]:
    """SSH into the Pi via WSL (keys live in WSL ~/.ssh/, not Windows)."""
    key_flag = f"-i ~/.ssh/{cfg['PI_SSH_KEY']}" if cfg["PI_SSH_KEY"] else ""
    ssh = (
        f"ssh -o StrictHostKeyChecking=no -o BatchMode=yes {key_flag} "
        f"{cfg['PI_USER']}@{cfg['PI_HOST']} {repr(bash_script)}"
    )
    return ["wsl.exe", "--", "bash", "-lc", ssh]


def ros_source(cfg: dict, ws: str) -> str:
    """Return the bash snippet that sources ROS2 + workspace."""
    distro = cfg["ROS_DISTRO"]
    return (
        f"source /opt/ros/{distro}/setup.bash && "
        f"source {ws}/install/setup.bash 2>/dev/null || true"
    )


def start_pi_stack(cfg: dict, skip_nav2: bool):
    src = (
        f"source ~/zenoh_venv/bin/activate && "
        + ros_source(cfg, cfg["PI_WS"])
    )

    # robot_bringup + relay as a compound command (same SSH session)
    script = (
        f"{src} && "
        f"ros2 launch embedded robot_bringup.launch.py &  "
        f"BRINGUP_PID=$! && "
        f"sleep 3 && "
        f"ros2 launch embedded zenoh_relay.launch.py role:=pi router_ip:={cfg['WINDOWS_IP']} & "
        f"RELAY_PID=$! && "
        f"wait $BRINGUP_PID $RELAY_PID"
    )
    launch("pi", _pi_cmd(cfg, script))
    cprint("pi", "Pi robot_bringup + relay starting…")


def start_wsl_stack(cfg: dict, skip_nav2: bool):
    src = ros_source(cfg, cfg["WSL_WS"])

    relay_cmd = (
        f"{src} && "
        f"ros2 launch embedded zenoh_relay.launch.py "
        f"role:=laptop router_ip:={cfg['WINDOWS_IP']}"
    )

    if skip_nav2:
        launch("wsl", _wsl_cmd(relay_cmd))
        cprint("wsl", "WSL relay starting (Nav2 skipped)…")
        return

    nav2_cmd = (
        f"{src} && "
        f"ros2 launch navigation hardware.launch.py"
    )

    # Run both in the same WSL session using a compound command
    script = (
        f"( {nav2_cmd} ) & NAV2_PID=$! && "
        f"sleep 4 && "
        f"( {relay_cmd} ) & RELAY_PID=$! && "
        f"wait $NAV2_PID $RELAY_PID"
    )
    launch("wsl", _wsl_cmd(script))
    cprint("wsl", "WSL Nav2 + relay starting…")


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Warehouse bot launcher")
    p.add_argument(
        "--router-only", action="store_true",
        help="Only start zenohd — useful when WSL/Pi are launched manually"
    )
    p.add_argument(
        "--skip-nav2", action="store_true",
        help="Start relays but skip Nav2 on WSL (e.g. for relay-only testing)"
    )
    return p.parse_args()


def main():
    args = parse_args()
    cfg  = CONFIG

    print("=" * 60)
    print("  Warehouse Bot Launcher")
    print(f"  Router IP : {cfg['WINDOWS_IP']}:{cfg['ROUTER_PORT']}")
    print(f"  Pi        : {cfg['PI_USER']}@{cfg['PI_HOST']}")
    print("  Press Ctrl+C to stop everything")
    print("=" * 60)

    try:
        # 1. Always start the router first
        start_router(cfg)

        if args.router_only:
            cprint("router", "Router-only mode — WSL/Pi not started by this script.")
        else:
            # 2. Pi stack (SSH)
            start_pi_stack(cfg, skip_nav2=True)  # Pi never runs Nav2

            # 3. WSL stack
            time.sleep(2)  # slight stagger so Pi relay is up first
            start_wsl_stack(cfg, skip_nav2=args.skip_nav2)

        # Block until Ctrl+C
        while True:
            time.sleep(1)
            # Check if any critical process died unexpectedly
            for p in _procs:
                if p.poll() is not None and p.returncode not in (0, -15):
                    cprint("launcher", f"A process exited unexpectedly (pid={p.pid}, code={p.returncode})")

    except KeyboardInterrupt:
        pass
    finally:
        kill_all()


if __name__ == "__main__":
    main()
