#!/usr/bin/env python3
"""
Python serial bridge for rapid iteration.

ROS API:
  /move_distance_mm (std_msgs/Int32)  signed mm (+forward, -backward)
  /rotate_angle_deg (std_msgs/Int32)  signed deg (+CCW, -CW)

Arduino legacy protocol:
  Pi -> Arduino: speed:<0-255>, w/s/q/e/x/r
  Arduino -> Pi: E:<left_ticks>,<right_ticks>
"""

from __future__ import annotations

import math
import os
import termios
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


@dataclass
class MotionState:
    active: bool = False
    mode: str = ""  # "linear" | "rotate"
    direction: int = 0  # +1 or -1
    target_ticks: int = 0
    start_left_ticks: int = 0
    start_right_ticks: int = 0
    start_time: float = 0.0
    speed_sent: bool = False
    last_drive_send_time: float = 0.0


class SerialBridgePy(Node):
    def __init__(self) -> None:
        super().__init__("serial_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("command_speed", 90)
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("wheel_separation", 0.21)
        self.declare_parameter("encoder_cpr", 2.0)
        self.declare_parameter("gear_ratio", 108.0)
        self.declare_parameter("distance_scale", 1.0158730158730158)
        self.declare_parameter("rotation_scale", 1.0)
        self.declare_parameter("rotation_speed", 60)
        self.declare_parameter("rotation_command_interval_s", 0.75)
        self.declare_parameter("use_drive_lr_linear", True)
        self.declare_parameter("linear_balance_kp", 0.4)
        self.declare_parameter("linear_steer_bias", -6.0)
        self.declare_parameter("command_timeout_s", 15.0)
        self.declare_parameter("command_rate_hz", 10.0)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.serial_baud = int(self.get_parameter("serial_baud").value)
        self.command_speed = int(self.get_parameter("command_speed").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_sep = float(self.get_parameter("wheel_separation").value)
        self.encoder_cpr = float(self.get_parameter("encoder_cpr").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.distance_scale = float(self.get_parameter("distance_scale").value)
        self.rotation_scale = float(self.get_parameter("rotation_scale").value)
        self.rotation_speed = int(self.get_parameter("rotation_speed").value)
        self.rotation_command_interval_s = float(self.get_parameter("rotation_command_interval_s").value)
        self.use_drive_lr_linear = bool(self.get_parameter("use_drive_lr_linear").value)
        self.linear_balance_kp = float(self.get_parameter("linear_balance_kp").value)
        self.linear_steer_bias = float(self.get_parameter("linear_steer_bias").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)

        self.command_speed = max(0, min(255, self.command_speed))
        self.rotation_speed = max(0, min(255, self.rotation_speed))
        self.rotation_command_interval_s = max(0.1, self.rotation_command_interval_s)
        effective_cpr = self.encoder_cpr * self.gear_ratio
        self.ticks_to_m = (2.0 * math.pi * self.wheel_radius) / effective_cpr

        self.left_pub = self.create_publisher(Int32, "/left_ticks", 10)
        self.right_pub = self.create_publisher(Int32, "/right_ticks", 10)

        self.create_subscription(Int32, "/move_distance_mm", self.on_move_mm, 10)
        self.create_subscription(Int32, "/rotate_angle_deg", self.on_rotate_deg, 10)

        self._fd = self._open_serial(self.serial_port, self.serial_baud)
        self.get_logger().info(f"Opened {self.serial_port} @ {self.serial_baud}")

        self._lock = threading.Lock()
        self._running = True
        self._current_left = 0
        self._current_right = 0
        self._motion = MotionState()

        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        period = 1.0 / self.command_rate_hz if self.command_rate_hz > 0 else 0.1
        self._timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("serial_bridge_py ready")

    def destroy_node(self) -> bool:
        self._running = False
        try:
            if self._fd is not None and self._fd >= 0:
                os.close(self._fd)
        except OSError:
            pass
        return super().destroy_node()

    def _open_serial(self, port: str, baud: int) -> int:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
        attrs = termios.tcgetattr(fd)

        speed_map = {
            9600: termios.B9600,
            57600: termios.B57600,
            115200: termios.B115200,
        }
        speed = speed_map.get(baud, termios.B115200)

        attrs[4] = speed  # ispeed
        attrs[5] = speed  # ospeed
        attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
        attrs[0] &= ~termios.IGNBRK
        attrs[3] = 0
        attrs[1] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~(termios.PARENB | termios.PARODD)
        attrs[2] &= ~termios.CSTOPB
        attrs[2] &= ~termios.CRTSCTS

        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd

    def _write_serial(self, msg: str) -> None:
        try:
            os.write(self._fd, msg.encode("utf-8"))
        except OSError as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")

    def _send_cmd(self, cmd: str) -> None:
        self._write_serial(f"{cmd}\n")

    def _send_speed(self) -> None:
        self._write_serial(f"speed:{self.command_speed}\n")

    def _send_drive_lr(self, left_pwm: int, right_pwm: int) -> None:
        left_pwm = max(-255, min(255, int(round(left_pwm))))
        right_pwm = max(-255, min(255, int(round(right_pwm))))
        self._write_serial(f"drive_lr:{left_pwm},{right_pwm}\n")

    def _stop_motion_outputs(self) -> None:
        self._send_drive_lr(0, 0)
        self._send_cmd("x")

    def on_move_mm(self, msg: Int32) -> None:
        if msg.data == 0:
            self._cancel_motion("zero move command")
            return
        self._start_motion("linear", msg.data)

    def on_rotate_deg(self, msg: Int32) -> None:
        if msg.data == 0:
            self._cancel_motion("zero rotate command")
            return
        self._start_motion("rotate", msg.data)

    def _start_motion(self, mode: str, signed_value: int) -> None:
        direction = 1 if signed_value > 0 else -1
        abs_value = abs(signed_value)

        if mode == "linear":
            meters = (abs_value * self.distance_scale) / 1000.0
            target_ticks = int(round(meters / self.ticks_to_m))
        else:
            theta = math.radians(abs_value * self.rotation_scale)
            wheel_arc = (self.wheel_sep / 2.0) * theta
            target_ticks = int(round(wheel_arc / self.ticks_to_m))

        target_ticks = max(1, target_ticks)

        with self._lock:
            self._motion = MotionState(
                active=True,
                mode=mode,
                direction=direction,
                target_ticks=target_ticks,
                start_left_ticks=self._current_left,
                start_right_ticks=self._current_right,
                start_time=time.monotonic(),
                speed_sent=False,
            )

        self.get_logger().info(
            f"Motion start mode={mode} value={signed_value} target_ticks={target_ticks} dir={direction}"
        )

    def _cancel_motion(self, reason: str) -> None:
        with self._lock:
            self._motion = MotionState()
        self._stop_motion_outputs()
        self.get_logger().info(f"Motion cancelled: {reason}")

    def control_loop(self) -> None:
        with self._lock:
            m = self._motion
            left = self._current_left
            right = self._current_right

        if not m.active:
            return

        elapsed = time.monotonic() - m.start_time
        if elapsed > self.command_timeout_s:
            self._stop_motion_outputs()
            self.get_logger().warn(f"Motion timeout after {elapsed:.2f}s")
            with self._lock:
                self._motion = MotionState()
            return

        dleft = abs(left - m.start_left_ticks)
        dright = abs(right - m.start_right_ticks)
        traveled = (dleft + dright) // 2
        if traveled >= m.target_ticks:
            self._stop_motion_outputs()
            self.get_logger().info(f"Motion complete traveled={traveled} target={m.target_ticks}")
            with self._lock:
                self._motion = MotionState()
            return

        if not m.speed_sent:
            self._send_speed()
            with self._lock:
                self._motion.speed_sent = True

        if m.mode == "linear":
            if self.use_drive_lr_linear:
                tick_error = dleft - dright
                correction = self.linear_balance_kp * tick_error
                left_pwm = (self.command_speed - correction + self.linear_steer_bias) * m.direction
                right_pwm = (self.command_speed + correction - self.linear_steer_bias) * m.direction
                self._send_drive_lr(left_pwm, right_pwm)
            else:
                self._send_cmd("w" if m.direction > 0 else "s")
        else:
            now = time.monotonic()
            if m.last_drive_send_time == 0.0 or now - m.last_drive_send_time >= self.rotation_command_interval_s:
                left_pwm = -self.rotation_speed * m.direction
                right_pwm = self.rotation_speed * m.direction
                self._send_drive_lr(left_pwm, right_pwm)
                with self._lock:
                    self._motion.last_drive_send_time = now

    def _read_loop(self) -> None:
        buf = ""
        while self._running:
            try:
                b = os.read(self._fd, 1)
            except OSError:
                time.sleep(0.01)
                continue

            if not b:
                continue

            ch = b.decode("utf-8", errors="ignore")
            if ch == "\n":
                line = buf.strip()
                buf = ""
                if line:
                    self._process_line(line)
                continue
            if ch != "\r":
                buf += ch
                if len(buf) > 128:
                    buf = ""

    def _process_line(self, line: str) -> None:
        if line.startswith("ACK:"):
            self.get_logger().info(f"Arduino: {line}")
            return
        if not line.startswith("E:"):
            self.get_logger().debug(f"Arduino: {line}")
            return

        payload = line[2:]
        if "," not in payload:
            self.get_logger().warn(f"Malformed encoder line: {line}")
            return
        # Firmware prints E:<front_right_ticks>,<rear_left_ticks>.
        # Publish them under robot-side names for odom and balancing.
        right_s, left_s = payload.split(",", 1)
        try:
            left = int(left_s)
            right = int(right_s)
        except ValueError:
            self.get_logger().warn(f"Parse error on encoder line: {line}")
            return

        with self._lock:
            self._current_left = left
            self._current_right = right

        msg_l = Int32()
        msg_l.data = left
        msg_r = Int32()
        msg_r.data = right
        self.left_pub.publish(msg_l)
        self.right_pub.publish(msg_r)


def main() -> None:
    rclpy.init()
    node: Optional[SerialBridgePy] = None
    try:
        node = SerialBridgePy()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
