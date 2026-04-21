#!/usr/bin/env python3
"""
Interactive calibration runner for the warehouse robot.

This script is intentionally WSL-side: keep the robot bringup running in your
own persistent Pi terminal, then run this tool from the laptop to command short
trials, capture ROS topic data, and write tape-measure observations to CSV.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import re
import shlex
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_ROOT = REPO_ROOT / "calibration_logs"

DEFAULT_DISTANCES_MM = [300, 500, 1000]
DEFAULT_ANGLES_DEG = [90, 180, 360, -90]
DEFAULT_EKF_SEGMENTS = [
    ("forward", 500),
    ("rotate", 90),
    ("forward", 500),
    ("rotate", -90),
]


def parse_int_list(value: str) -> list[int]:
    items = []
    for raw in value.split(","):
        raw = raw.strip()
        if raw:
            items.append(int(raw))
    if not items:
        raise argparse.ArgumentTypeError("list must contain at least one integer")
    return items


def now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def iso_now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def prompt_text(label: str, default: str = "", skip: bool = False) -> str:
    if skip:
        return default
    suffix = f" [{default}]" if default else ""
    value = input(f"{label}{suffix}: ").strip()
    return value if value else default


def prompt_float(label: str, default: float | None = None, skip: bool = False) -> float | None:
    if skip:
        return default
    while True:
        suffix = f" [{default}]" if default is not None else ""
        value = input(f"{label}{suffix}: ").strip()
        if not value and default is not None:
            return default
        if not value:
            return None
        try:
            return float(value)
        except ValueError:
            print("Please enter a number, or leave blank if optional.")


def prompt_bool(label: str, default: bool = True, skip: bool = False) -> bool:
    if skip:
        return default
    marker = "Y/n" if default else "y/N"
    value = input(f"{label} [{marker}]: ").strip().lower()
    if not value:
        return default
    return value in {"y", "yes", "true", "1", "pass", "passed"}


def mean(values: Iterable[float]) -> float:
    values = list(values)
    return statistics.fmean(values) if values else 0.0


def stdev(values: Iterable[float]) -> float:
    values = list(values)
    if len(values) < 2:
        return 0.0
    return statistics.stdev(values)


def percent_error(error: float, target: float) -> float:
    if target == 0:
        return 0.0
    return 100.0 * error / abs(target)


def safe_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


class CalibrationRun:
    def __init__(self, test_name: str, args: argparse.Namespace):
        self.test_name = test_name
        self.args = args
        self.root = Path(args.log_root).expanduser()
        if not self.root.is_absolute():
            self.root = REPO_ROOT / self.root
        self.path = self.root / f"{now_stamp()}_{test_name}"
        self.topics_path = self.path / "topics"
        self.path.mkdir(parents=True, exist_ok=False)
        self.topics_path.mkdir(parents=True, exist_ok=True)
        self.write_metadata()

    def write_metadata(self) -> None:
        metadata = {
            "test_name": self.test_name,
            "created_at": iso_now(),
            "cwd": str(REPO_ROOT),
            "dry_run": self.args.dry_run,
            "stream_topics": self.args.stream_topics,
            "options": vars(self.args),
            "environment": {
                "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", ""),
                "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", ""),
                "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", ""),
                "FASTDDS_DEFAULT_PROFILES_FILE": os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE", ""),
            },
            "topics": {
                "move_distance_mm": "/move_distance_mm",
                "rotate_angle_deg": "/rotate_angle_deg",
                "left_ticks": "/left_ticks",
                "right_ticks": "/right_ticks",
                "odom": "/odom",
                "imu_data": "/imu/data",
                "imu_mag": "/imu/mag",
            },
        }
        (self.path / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    def csv_append(self, relative_path: str, row: dict[str, object], fieldnames: list[str]) -> None:
        path = self.path / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        exists = path.exists()
        with path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            if not exists:
                writer.writeheader()
            writer.writerow(row)

    def csv_write(self, relative_path: str, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
        path = self.path / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

    def text_write(self, relative_path: str, text: str) -> None:
        path = self.path / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")


class RosRunner:
    def __init__(self, run: CalibrationRun):
        self.calibration_run = run
        self.dry_run = run.args.dry_run

    def _bash_command(self, command: str) -> list[str]:
        workspace_setup = REPO_ROOT / "ros2_ws" / "install" / "setup.bash"
        prelude = [
            "set +u",
            "if [ -f \"$HOME/.ros_network_env\" ]; then source \"$HOME/.ros_network_env\"; "
            "else source /opt/ros/kilted/setup.bash; fi",
        ]
        if workspace_setup.exists():
            prelude.append(f"if [ -f {shlex.quote(str(workspace_setup))} ]; then source {shlex.quote(str(workspace_setup))}; fi")
        prelude.append(command)
        return ["bash", "-lc", "; ".join(prelude)]

    def run(self, command: str, timeout: float | None = None, check: bool = False) -> subprocess.CompletedProcess[str]:
        if self.dry_run:
            print(f"[dry-run] {command}")
            return subprocess.CompletedProcess(command, 0, "", "")
        proc = subprocess.run(
            self._bash_command(command),
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
        if check and proc.returncode != 0:
            raise RuntimeError(f"Command failed ({proc.returncode}): {command}\n{proc.stderr}")
        return proc

    def capture_stream(self, topic: str, seconds: float, output_path: Path, csv_mode: bool = True) -> None:
        if self.dry_run:
            output_path.write_text(f"dry_run,topic,seconds\ntrue,{topic},{seconds}\n", encoding="utf-8")
            print(f"[dry-run] capture {topic} for {seconds}s -> {output_path}")
            return
        mode = "--csv" if csv_mode else ""
        command = f"timeout {shlex.quote(str(seconds))} ros2 topic echo {mode} {shlex.quote(topic)}"
        proc = self.run(command, timeout=seconds + 8.0)
        output_path.write_text(proc.stdout, encoding="utf-8")
        if proc.stderr:
            output_path.with_suffix(output_path.suffix + ".stderr").write_text(proc.stderr, encoding="utf-8")

    def capture_once_text(self, topic: str, output_path: Path, timeout_s: float = 8.0) -> str:
        if self.dry_run:
            text = f"dry_run: true\ntopic: {topic}\n"
            output_path.write_text(text, encoding="utf-8")
            return text
        command = f"timeout {shlex.quote(str(timeout_s))} ros2 topic echo --once {shlex.quote(topic)}"
        proc = self.run(command, timeout=timeout_s + 3.0)
        text = proc.stdout
        output_path.write_text(text, encoding="utf-8")
        if proc.stderr:
            output_path.with_suffix(output_path.suffix + ".stderr").write_text(proc.stderr, encoding="utf-8")
        return text

    def publish_int_once(self, topic: str, value: int) -> None:
        command = f"ros2 topic pub --once {shlex.quote(topic)} std_msgs/msg/Int32 '{{data: {value}}}'"
        self.run(command, timeout=8.0, check=not self.dry_run)


def parse_data_int(text: str) -> int | None:
    match = re.search(r"data:\s*(-?\d+)", text)
    return int(match.group(1)) if match else None


def extract_first_float_after(label: str, text: str) -> float | None:
    match = re.search(rf"{re.escape(label)}:\s*(-?\d+(?:\.\d+)?(?:e[-+]?\d+)?)", text, re.IGNORECASE)
    return float(match.group(1)) if match else None


def parse_vector3(prefix: str, text: str) -> tuple[float | None, float | None, float | None]:
    section_match = re.search(rf"{re.escape(prefix)}:\n((?:\s+[xyz]:\s*[-+0-9.eE]+\n?){{1,3}})", text)
    section = section_match.group(1) if section_match else ""
    return (
        extract_first_float_after("x", section),
        extract_first_float_after("y", section),
        extract_first_float_after("z", section),
    )


def parse_quaternion(prefix: str, text: str) -> tuple[float | None, float | None, float | None, float | None]:
    section_match = re.search(rf"{re.escape(prefix)}:\n((?:\s+[xyzw]:\s*[-+0-9.eE]+\n?){{1,4}})", text)
    section = section_match.group(1) if section_match else ""
    return (
        extract_first_float_after("x", section),
        extract_first_float_after("y", section),
        extract_first_float_after("z", section),
        extract_first_float_after("w", section),
    )


def yaw_from_quaternion(x: float | None, y: float | None, z: float | None, w: float | None) -> float | None:
    if None in {x, y, z, w}:
        return None
    assert x is not None and y is not None and z is not None and w is not None
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def angle_delta_deg(start: float | None, end: float | None) -> float | None:
    if start is None or end is None:
        return None
    delta = end - start
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def parse_imu_sample(text: str) -> dict[str, float | None]:
    ox, oy, oz, ow = parse_quaternion("orientation", text)
    avx, avy, avz = parse_vector3("angular_velocity", text)
    lax, lay, laz = parse_vector3("linear_acceleration", text)
    return {
        "orientation_x": ox,
        "orientation_y": oy,
        "orientation_z": oz,
        "orientation_w": ow,
        "orientation_yaw_deg": yaw_from_quaternion(ox, oy, oz, ow),
        "angular_velocity_x": avx,
        "angular_velocity_y": avy,
        "angular_velocity_z": avz,
        "linear_acceleration_x": lax,
        "linear_acceleration_y": lay,
        "linear_acceleration_z": laz,
    }


def parse_odom_sample(text: str) -> dict[str, float | None]:
    px, py, pz = parse_vector3("position", text)
    ox, oy, oz, ow = parse_quaternion("orientation", text)
    linear_x = extract_first_float_after("x", text)
    return {
        "position_x": px,
        "position_y": py,
        "position_z": pz,
        "orientation_x": ox,
        "orientation_y": oy,
        "orientation_z": oz,
        "orientation_w": ow,
        "yaw_deg": yaw_from_quaternion(ox, oy, oz, ow),
        "linear_x_first_match": linear_x,
    }


def read_imu_csv(path: Path) -> list[dict[str, float]]:
    fieldnames = [
        "header.stamp.sec",
        "header.stamp.nanosec",
        "header.frame_id",
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
        "orientation_covariance.0",
        "orientation_covariance.1",
        "orientation_covariance.2",
        "orientation_covariance.3",
        "orientation_covariance.4",
        "orientation_covariance.5",
        "orientation_covariance.6",
        "orientation_covariance.7",
        "orientation_covariance.8",
        "angular_velocity.x",
        "angular_velocity.y",
        "angular_velocity.z",
        "angular_velocity_covariance.0",
        "angular_velocity_covariance.1",
        "angular_velocity_covariance.2",
        "angular_velocity_covariance.3",
        "angular_velocity_covariance.4",
        "angular_velocity_covariance.5",
        "angular_velocity_covariance.6",
        "angular_velocity_covariance.7",
        "angular_velocity_covariance.8",
        "linear_acceleration.x",
        "linear_acceleration.y",
        "linear_acceleration.z",
        "linear_acceleration_covariance.0",
        "linear_acceleration_covariance.1",
        "linear_acceleration_covariance.2",
        "linear_acceleration_covariance.3",
        "linear_acceleration_covariance.4",
        "linear_acceleration_covariance.5",
        "linear_acceleration_covariance.6",
        "linear_acceleration_covariance.7",
        "linear_acceleration_covariance.8",
    ]
    rows: list[dict[str, float]] = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    with path.open(newline="", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        first = next(reader, None)
        if first is None:
            return rows

        has_header = safe_float(first[0]) is None
        if has_header:
            header = first
        else:
            header = fieldnames
            reader = itertools.chain([first], reader)

        for row in reader:
            parsed = {}
            for key, value in zip(header, row):
                parsed[key] = safe_float(value)
            rows.append(parsed)
    return rows


def wait_for_operator(message: str, args: argparse.Namespace) -> None:
    if args.dry_run or args.yes:
        print(message)
        return
    input(f"{message}\nPress Enter when ready...")


def write_observation(run: CalibrationRun, row: dict[str, object]) -> None:
    fields = [
        "timestamp",
        "test",
        "trial_id",
        "prompt",
        "value",
        "units",
        "notes",
    ]
    run.csv_append("observations.csv", row, fields)


def append_tick_snapshot(run: CalibrationRun, ros: RosRunner, trial_id: str, phase: str) -> dict[str, int | None]:
    left_text = ros.capture_once_text("/left_ticks", run.topics_path / f"{trial_id}_{phase}_left_ticks.txt")
    right_text = ros.capture_once_text("/right_ticks", run.topics_path / f"{trial_id}_{phase}_right_ticks.txt")
    left = parse_data_int(left_text)
    right = parse_data_int(right_text)
    run.csv_append(
        "topics/ticks.csv",
        {
            "timestamp": iso_now(),
            "trial_id": trial_id,
            "phase": phase,
            "left_ticks": left,
            "right_ticks": right,
        },
        ["timestamp", "trial_id", "phase", "left_ticks", "right_ticks"],
    )
    return {"left_ticks": left, "right_ticks": right}


def append_odom_snapshot(run: CalibrationRun, ros: RosRunner, trial_id: str, phase: str, topic: str = "/odom", filename_prefix: str = "odom") -> dict[str, float | None]:
    text = ros.capture_once_text(topic, run.topics_path / f"{trial_id}_{phase}_{filename_prefix}.txt")
    sample = parse_odom_sample(text)
    row = {"timestamp": iso_now(), "trial_id": trial_id, "phase": phase, "topic": topic, **sample}
    run.csv_append(
        f"topics/{filename_prefix}.csv",
        row,
        [
            "timestamp",
            "trial_id",
            "phase",
            "topic",
            "position_x",
            "position_y",
            "position_z",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
            "yaw_deg",
            "linear_x_first_match",
        ],
    )
    return sample


def append_imu_snapshot(run: CalibrationRun, ros: RosRunner, trial_id: str, phase: str) -> dict[str, float | None]:
    text = ros.capture_once_text("/imu/data", run.topics_path / f"{trial_id}_{phase}_imu_data.txt")
    sample = parse_imu_sample(text)
    row = {"timestamp": iso_now(), "trial_id": trial_id, "phase": phase, **sample}
    run.csv_append(
        "topics/imu_data.csv",
        row,
        [
            "timestamp",
            "trial_id",
            "phase",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
            "orientation_yaw_deg",
            "angular_velocity_x",
            "angular_velocity_y",
            "angular_velocity_z",
            "linear_acceleration_x",
            "linear_acceleration_y",
            "linear_acceleration_z",
        ],
    )
    return sample


def start_optional_streams(run: CalibrationRun, ros: RosRunner, trial_id: str) -> list[subprocess.Popen[str]]:
    if not run.args.stream_topics or run.args.dry_run:
        return []
    commands = [
        ("/left_ticks", run.topics_path / f"{trial_id}_stream_left_ticks.csv"),
        ("/right_ticks", run.topics_path / f"{trial_id}_stream_right_ticks.csv"),
        ("/odom", run.topics_path / f"{trial_id}_stream_odom.csv"),
        ("/imu/data", run.topics_path / f"{trial_id}_stream_imu_data.csv"),
    ]
    procs: list[subprocess.Popen[str]] = []
    for topic, path in commands:
        cmd = ros._bash_command(f"ros2 topic echo --csv {shlex.quote(topic)}")
        f = path.open("w", encoding="utf-8")
        proc = subprocess.Popen(cmd, stdout=f, stderr=subprocess.DEVNULL, text=True)
        proc._calibration_file = f  # type: ignore[attr-defined]
        procs.append(proc)
    return procs


def stop_streams(procs: list[subprocess.Popen[str]]) -> None:
    for proc in procs:
        proc.terminate()
    for proc in procs:
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        file_obj = getattr(proc, "_calibration_file", None)
        if file_obj:
            file_obj.close()


def run_imu(args: argparse.Namespace) -> Path:
    run = CalibrationRun("imu", args)
    ros = RosRunner(run)
    print(f"IMU calibration log: {run.path}")
    wait_for_operator("Place the robot flat, still, and untouched.", args)

    if not args.dry_run and not args.yes:
        for label, units in [
            ("table_condition", ""),
            ("motors_powered", "yes/no"),
            ("nearby_metal_or_electronics", ""),
            ("notes", ""),
        ]:
            value = prompt_text(label)
            write_observation(
                run,
                {
                    "timestamp": iso_now(),
                    "test": "imu",
                    "trial_id": "stationary",
                    "prompt": label,
                    "value": value,
                    "units": units,
                    "notes": "",
                },
            )

    imu_csv = run.topics_path / "imu_data.csv"
    hz_txt = run.topics_path / "imu_hz.txt"
    mag_csv = run.topics_path / "imu_mag.csv"

    ros.capture_stream("/imu/data", args.imu_seconds, imu_csv, csv_mode=True)
    if args.dry_run:
        hz_txt.write_text("dry_run,average_rate\ntrue,0\n", encoding="utf-8")
    else:
        hz_proc = ros.run(f"timeout {args.hz_seconds} ros2 topic hz /imu/data", timeout=args.hz_seconds + 5.0)
        hz_txt.write_text(hz_proc.stdout, encoding="utf-8")
        if hz_proc.stderr:
            hz_txt.with_suffix(".txt.stderr").write_text(hz_proc.stderr, encoding="utf-8")
    ros.capture_stream("/imu/mag", 2.0, mag_csv, csv_mode=True)

    rows = read_imu_csv(imu_csv)
    accel_mags: list[float] = []
    gyro_x: list[float] = []
    gyro_y: list[float] = []
    gyro_z: list[float] = []
    accel_x: list[float] = []
    accel_y: list[float] = []
    accel_z: list[float] = []
    for row in rows:
        ax = row.get("linear_acceleration.x")
        ay = row.get("linear_acceleration.y")
        az = row.get("linear_acceleration.z")
        gx = row.get("angular_velocity.x")
        gy = row.get("angular_velocity.y")
        gz = row.get("angular_velocity.z")
        if ax is not None and ay is not None and az is not None:
            accel_x.append(ax)
            accel_y.append(ay)
            accel_z.append(az)
            accel_mags.append(math.sqrt(ax * ax + ay * ay + az * az))
        if gx is not None:
            gyro_x.append(gx)
        if gy is not None:
            gyro_y.append(gy)
        if gz is not None:
            gyro_z.append(gz)

    hz_text = hz_txt.read_text(encoding="utf-8", errors="ignore") if hz_txt.exists() else ""
    hz_rates = [float(v) for v in re.findall(r"average rate:\s*([0-9.]+)", hz_text)]
    summary = [
        {"metric": "samples", "value": len(rows), "units": "count"},
        {"metric": "average_rate", "value": mean(hz_rates), "units": "Hz"},
        {"metric": "accel_x_mean", "value": mean(accel_x), "units": "m/s^2"},
        {"metric": "accel_y_mean", "value": mean(accel_y), "units": "m/s^2"},
        {"metric": "accel_z_mean", "value": mean(accel_z), "units": "m/s^2"},
        {"metric": "accel_magnitude_mean", "value": mean(accel_mags), "units": "m/s^2"},
        {"metric": "accel_magnitude_stddev", "value": stdev(accel_mags), "units": "m/s^2"},
        {"metric": "gyro_x_mean", "value": mean(gyro_x), "units": "rad/s"},
        {"metric": "gyro_y_mean", "value": mean(gyro_y), "units": "rad/s"},
        {"metric": "gyro_z_mean", "value": mean(gyro_z), "units": "rad/s"},
        {"metric": "gyro_x_stddev", "value": stdev(gyro_x), "units": "rad/s"},
        {"metric": "gyro_y_stddev", "value": stdev(gyro_y), "units": "rad/s"},
        {"metric": "gyro_z_stddev", "value": stdev(gyro_z), "units": "rad/s"},
    ]
    run.csv_write("summary.csv", summary, ["metric", "value", "units"])
    print(f"IMU calibration complete: {run.path}")
    return run.path


def movement_wait_seconds(kind: str, value: int) -> float:
    if kind == "distance":
        return max(5.0, abs(value) / 90.0 + 3.0)
    return max(5.0, abs(value) / 45.0 + 3.0)


def run_distance(args: argparse.Namespace) -> Path:
    run = CalibrationRun("distance", args)
    ros = RosRunner(run)
    print(f"Distance calibration log: {run.path}")
    trial_rows: list[dict[str, object]] = []

    for distance in args.distances:
        for repeat in range(1, args.repeats + 1):
            trial_id = f"distance_{distance}mm_r{repeat}"
            wait_for_operator(f"Set robot at tape-measure zero for {trial_id}.", args)
            start_ticks = append_tick_snapshot(run, ros, trial_id, "start")
            start_odom = append_odom_snapshot(run, ros, trial_id, "start")
            start_imu = append_imu_snapshot(run, ros, trial_id, "start")

            streams = start_optional_streams(run, ros, trial_id)
            try:
                ros.publish_int_once("/move_distance_mm", distance)
                time.sleep(0.1 if args.dry_run else movement_wait_seconds("distance", distance))
            finally:
                stop_streams(streams)

            end_ticks = append_tick_snapshot(run, ros, trial_id, "end")
            end_odom = append_odom_snapshot(run, ros, trial_id, "end")
            end_imu = append_imu_snapshot(run, ros, trial_id, "end")

            measured = prompt_float("Measured actual distance in mm", default=float(distance) if args.dry_run else None, skip=args.dry_run)
            drift = prompt_float("Measured lateral drift in mm, optional", default=0.0, skip=args.dry_run)
            passed = prompt_bool("Pass this trial?", default=True, skip=args.dry_run)
            notes = prompt_text("Notes", default="", skip=args.dry_run)

            measured_value = measured if measured is not None else 0.0
            error_mm = measured_value - distance
            scale = (distance / measured_value) if measured_value else 0.0
            left_delta = None
            right_delta = None
            if start_ticks["left_ticks"] is not None and end_ticks["left_ticks"] is not None:
                left_delta = end_ticks["left_ticks"] - start_ticks["left_ticks"]
            if start_ticks["right_ticks"] is not None and end_ticks["right_ticks"] is not None:
                right_delta = end_ticks["right_ticks"] - start_ticks["right_ticks"]

            odom_distance = None
            if start_odom.get("position_x") is not None and end_odom.get("position_x") is not None:
                dx = (end_odom.get("position_x") or 0.0) - (start_odom.get("position_x") or 0.0)
                dy = (end_odom.get("position_y") or 0.0) - (start_odom.get("position_y") or 0.0)
                odom_distance = math.sqrt(dx * dx + dy * dy) * 1000.0

            row = {
                "timestamp": iso_now(),
                "trial_id": trial_id,
                "commanded_distance_mm": distance,
                "measured_distance_mm": measured,
                "error_mm": error_mm,
                "error_percent": percent_error(error_mm, distance),
                "suggested_distance_scale": scale,
                "drift_mm": drift,
                "left_tick_delta": left_delta,
                "right_tick_delta": right_delta,
                "odom_distance_mm": odom_distance,
                "imu_yaw_start_deg": start_imu.get("orientation_yaw_deg"),
                "imu_yaw_end_deg": end_imu.get("orientation_yaw_deg"),
                "passed": passed,
                "notes": notes,
            }
            fields = list(row.keys())
            run.csv_append("trials.csv", row, fields)
            trial_rows.append(row)
            write_observation(
                run,
                {
                    "timestamp": iso_now(),
                    "test": "distance",
                    "trial_id": trial_id,
                    "prompt": "measured_distance_mm",
                    "value": measured,
                    "units": "mm",
                    "notes": notes,
                },
            )

    write_distance_summary(run, trial_rows)
    print(f"Distance calibration complete: {run.path}")
    return run.path


def write_distance_summary(run: CalibrationRun, rows: list[dict[str, object]]) -> None:
    summary: list[dict[str, object]] = []
    for distance in sorted({int(row["commanded_distance_mm"]) for row in rows}):
        subset = [row for row in rows if int(row["commanded_distance_mm"]) == distance]
        errors = [float(row["error_mm"]) for row in subset if row["error_mm"] not in ("", None)]
        scales = [float(row["suggested_distance_scale"]) for row in subset if row["suggested_distance_scale"] not in ("", None, 0)]
        summary.append(
            {
                "group": f"{distance}mm",
                "trials": len(subset),
                "mean_error": mean(errors),
                "mean_error_percent": percent_error(mean(errors), distance),
                "suggested_scale": mean(scales),
                "units": "mm",
            }
        )
    all_scales = [float(row["suggested_distance_scale"]) for row in rows if row["suggested_distance_scale"] not in ("", None, 0)]
    summary.append(
        {
            "group": "overall",
            "trials": len(rows),
            "mean_error": mean(float(row["error_mm"]) for row in rows),
            "mean_error_percent": mean(float(row["error_percent"]) for row in rows),
            "suggested_scale": mean(all_scales),
            "units": "mm",
        }
    )
    run.csv_write("summary.csv", summary, ["group", "trials", "mean_error", "mean_error_percent", "suggested_scale", "units"])


def run_rotation(args: argparse.Namespace) -> Path:
    run = CalibrationRun("rotation", args)
    ros = RosRunner(run)
    print(f"Rotation calibration log: {run.path}")
    trial_rows: list[dict[str, object]] = []

    for angle in args.angles:
        repeat_count = 2 if abs(angle) == 360 and args.repeats == 3 else args.repeats
        for repeat in range(1, repeat_count + 1):
            trial_id = f"rotation_{angle}deg_r{repeat}".replace("-", "neg")
            wait_for_operator(f"Mark starting heading for {trial_id}.", args)
            start_ticks = append_tick_snapshot(run, ros, trial_id, "start")
            start_odom = append_odom_snapshot(run, ros, trial_id, "start")
            start_imu = append_imu_snapshot(run, ros, trial_id, "start")

            streams = start_optional_streams(run, ros, trial_id)
            try:
                ros.publish_int_once("/rotate_angle_deg", angle)
                time.sleep(0.1 if args.dry_run else movement_wait_seconds("rotation", angle))
            finally:
                stop_streams(streams)

            end_ticks = append_tick_snapshot(run, ros, trial_id, "end")
            end_odom = append_odom_snapshot(run, ros, trial_id, "end")
            end_imu = append_imu_snapshot(run, ros, trial_id, "end")

            measured = prompt_float("Measured actual angle in degrees", default=float(angle) if args.dry_run else None, skip=args.dry_run)
            direction_correct = prompt_bool("Direction correct?", default=True, skip=args.dry_run)
            skid_notes = prompt_text("Skid notes", default="", skip=args.dry_run)
            passed = prompt_bool("Pass this trial?", default=True, skip=args.dry_run)
            notes = prompt_text("Notes", default="", skip=args.dry_run)

            measured_value = measured if measured is not None else 0.0
            error_deg = measured_value - angle
            scale = (angle / measured_value) if measured_value else 0.0
            left_delta = None
            right_delta = None
            if start_ticks["left_ticks"] is not None and end_ticks["left_ticks"] is not None:
                left_delta = end_ticks["left_ticks"] - start_ticks["left_ticks"]
            if start_ticks["right_ticks"] is not None and end_ticks["right_ticks"] is not None:
                right_delta = end_ticks["right_ticks"] - start_ticks["right_ticks"]

            odom_yaw_delta = angle_delta_deg(start_odom.get("yaw_deg"), end_odom.get("yaw_deg"))
            imu_yaw_delta = angle_delta_deg(start_imu.get("orientation_yaw_deg"), end_imu.get("orientation_yaw_deg"))
            row = {
                "timestamp": iso_now(),
                "trial_id": trial_id,
                "commanded_angle_deg": angle,
                "measured_angle_deg": measured,
                "error_deg": error_deg,
                "error_percent": percent_error(error_deg, angle),
                "suggested_rotation_scale": scale,
                "direction_correct": direction_correct,
                "left_tick_delta": left_delta,
                "right_tick_delta": right_delta,
                "odom_yaw_delta_deg": odom_yaw_delta,
                "imu_yaw_delta_deg": imu_yaw_delta,
                "skid_notes": skid_notes,
                "passed": passed,
                "notes": notes,
            }
            run.csv_append("trials.csv", row, list(row.keys()))
            trial_rows.append(row)
            write_observation(
                run,
                {
                    "timestamp": iso_now(),
                    "test": "rotation",
                    "trial_id": trial_id,
                    "prompt": "measured_angle_deg",
                    "value": measured,
                    "units": "deg",
                    "notes": notes,
                },
            )

    write_rotation_summary(run, trial_rows)
    print(f"Rotation calibration complete: {run.path}")
    return run.path


def write_rotation_summary(run: CalibrationRun, rows: list[dict[str, object]]) -> None:
    summary: list[dict[str, object]] = []
    for angle in sorted({int(row["commanded_angle_deg"]) for row in rows}, key=lambda v: (abs(v), v)):
        subset = [row for row in rows if int(row["commanded_angle_deg"]) == angle]
        errors = [float(row["error_deg"]) for row in subset if row["error_deg"] not in ("", None)]
        scales = [float(row["suggested_rotation_scale"]) for row in subset if row["suggested_rotation_scale"] not in ("", None, 0)]
        summary.append(
            {
                "group": f"{angle}deg",
                "trials": len(subset),
                "mean_error": mean(errors),
                "mean_error_percent": percent_error(mean(errors), angle),
                "suggested_scale": mean(scales),
                "units": "deg",
            }
        )
    all_scales = [float(row["suggested_rotation_scale"]) for row in rows if row["suggested_rotation_scale"] not in ("", None, 0)]
    summary.append(
        {
            "group": "overall",
            "trials": len(rows),
            "mean_error": mean(float(row["error_deg"]) for row in rows),
            "mean_error_percent": mean(float(row["error_percent"]) for row in rows),
            "suggested_scale": mean(all_scales),
            "units": "deg",
        }
    )
    run.csv_write("summary.csv", summary, ["group", "trials", "mean_error", "mean_error_percent", "suggested_scale", "units"])


def run_ekf(args: argparse.Namespace) -> Path:
    run = CalibrationRun("ekf", args)
    ros = RosRunner(run)
    print(f"EKF verification log: {run.path}")
    trial_rows: list[dict[str, object]] = []
    wait_for_operator("Set robot at the start pose for the EKF verification path.", args)

    for index, (kind, value) in enumerate(DEFAULT_EKF_SEGMENTS, start=1):
        trial_id = f"ekf_segment_{index}_{kind}_{value}"
        start_raw = append_odom_snapshot(run, ros, trial_id, "start", "/odom", "raw_odom")
        start_imu = append_imu_snapshot(run, ros, trial_id, "start")
        start_filtered = append_odom_snapshot(run, ros, trial_id, "start", "/odometry/filtered", "filtered_odom")

        streams = start_optional_streams(run, ros, trial_id)
        try:
            if kind == "forward":
                ros.publish_int_once("/move_distance_mm", value)
                sleep_for = movement_wait_seconds("distance", value)
            else:
                ros.publish_int_once("/rotate_angle_deg", value)
                sleep_for = movement_wait_seconds("rotation", value)
            time.sleep(0.1 if args.dry_run else sleep_for)
        finally:
            stop_streams(streams)

        end_raw = append_odom_snapshot(run, ros, trial_id, "end", "/odom", "raw_odom")
        end_imu = append_imu_snapshot(run, ros, trial_id, "end")
        end_filtered = append_odom_snapshot(run, ros, trial_id, "end", "/odometry/filtered", "filtered_odom")

        observed_position_error = prompt_float("Observed position error after this segment in mm, optional", default=0.0, skip=args.dry_run)
        observed_heading_error = prompt_float("Observed heading error after this segment in deg, optional", default=0.0, skip=args.dry_run)
        passed = prompt_bool("Pass this segment?", default=True, skip=args.dry_run)
        notes = prompt_text("Notes", default="", skip=args.dry_run)

        raw_yaw_delta = angle_delta_deg(start_raw.get("yaw_deg"), end_raw.get("yaw_deg"))
        filtered_yaw_delta = angle_delta_deg(start_filtered.get("yaw_deg"), end_filtered.get("yaw_deg"))
        imu_yaw_delta = angle_delta_deg(start_imu.get("orientation_yaw_deg"), end_imu.get("orientation_yaw_deg"))

        row = {
            "timestamp": iso_now(),
            "trial_id": trial_id,
            "segment_index": index,
            "command_kind": kind,
            "command_value": value,
            "observed_position_error_mm": observed_position_error,
            "observed_heading_error_deg": observed_heading_error,
            "raw_odom_yaw_delta_deg": raw_yaw_delta,
            "filtered_odom_yaw_delta_deg": filtered_yaw_delta,
            "imu_yaw_delta_deg": imu_yaw_delta,
            "passed": passed,
            "notes": notes,
        }
        run.csv_append("trials.csv", row, list(row.keys()))
        trial_rows.append(row)

    final_x_error = prompt_float("Final measured X error in mm", default=0.0, skip=args.dry_run)
    final_y_error = prompt_float("Final measured Y error in mm", default=0.0, skip=args.dry_run)
    final_heading_error = prompt_float("Final measured heading error in deg", default=0.0, skip=args.dry_run)
    notes = prompt_text("Final notes", default="", skip=args.dry_run)
    for prompt, value, units in [
        ("final_x_error", final_x_error, "mm"),
        ("final_y_error", final_y_error, "mm"),
        ("final_heading_error", final_heading_error, "deg"),
    ]:
        write_observation(
            run,
            {
                "timestamp": iso_now(),
                "test": "ekf",
                "trial_id": "final",
                "prompt": prompt,
                "value": value,
                "units": units,
                "notes": notes,
            },
        )

    summary = [
        {"metric": "segments", "value": len(trial_rows), "units": "count"},
        {"metric": "mean_position_error", "value": mean(row.get("observed_position_error_mm") or 0.0 for row in trial_rows), "units": "mm"},
        {"metric": "mean_heading_error", "value": mean(row.get("observed_heading_error_deg") or 0.0 for row in trial_rows), "units": "deg"},
        {"metric": "final_x_error", "value": final_x_error, "units": "mm"},
        {"metric": "final_y_error", "value": final_y_error, "units": "mm"},
        {"metric": "final_heading_error", "value": final_heading_error, "units": "deg"},
    ]
    run.csv_write("summary.csv", summary, ["metric", "value", "units"])
    print(f"EKF verification complete: {run.path}")
    return run.path


def add_common_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--dry-run", action="store_true", help="Create logs and print commands without moving the robot.")
    parser.add_argument("--yes", action="store_true", help="Skip readiness prompts; measurement prompts still appear for real movement tests.")
    parser.add_argument("--stream-topics", action="store_true", help="Capture continuous topic streams during movement trials.")
    parser.add_argument("--log-root", default=str(DEFAULT_LOG_ROOT), help="Directory for calibration run folders.")
    parser.add_argument("--repeats", type=int, default=3, help="Repeats for distance/rotation tests.")
    parser.add_argument("--distances", type=parse_int_list, default=DEFAULT_DISTANCES_MM, help="Comma-separated distance list in mm.")
    parser.add_argument("--angles", type=parse_int_list, default=DEFAULT_ANGLES_DEG, help="Comma-separated angle list in deg.")
    parser.add_argument("--imu-seconds", type=float, default=60.0, help="Seconds to capture /imu/data.")
    parser.add_argument("--hz-seconds", type=float, default=30.0, help="Seconds to sample ros2 topic hz /imu/data.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="One-click CSV calibration pipeline for the warehouse robot.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ["imu", "distance", "rotation", "ekf", "all"]:
        sub = subparsers.add_parser(name)
        add_common_options(sub)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.repeats < 1:
        print("--repeats must be at least 1", file=sys.stderr)
        return 2

    if args.command == "imu":
        run_imu(args)
    elif args.command == "distance":
        run_distance(args)
    elif args.command == "rotation":
        run_rotation(args)
    elif args.command == "ekf":
        run_ekf(args)
    elif args.command == "all":
        run_imu(args)
        run_distance(args)
        run_rotation(args)
        run_ekf(args)
    else:
        raise AssertionError(args.command)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
