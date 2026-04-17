#!/usr/bin/env python3
import queue
import os
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from PIL import Image, ImageTk
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image as RosImage
from std_msgs.msg import Int32


CAMERA_TOPIC = os.environ.get('OPERATOR_CAMERA_TOPIC', '/camera/image_raw/compressed')
OVERLAY_TOPIC = os.environ.get('OPERATOR_OVERLAY_TOPIC', '/perception/yolo/annotated_image')
APP_TITLE = 'Warehouse Bot Operator Console'
VIDEO_SIZE = (640, 480)
LOG_MAX_LINES = 200
OVERLAY_RECENCY_SEC = 1.0


class OperatorConsoleNode(Node):
    def __init__(self, ui_queue: queue.Queue):
        super().__init__('operator_console')
        self.ui_queue = ui_queue
        self.latest_camera_frame = None
        self.latest_overlay_frame = None
        self.latest_camera_time = 0.0
        self.latest_overlay_time = 0.0
        self.frame_lock = threading.Lock()

        self.move_pub = self.create_publisher(Int32, '/move_distance_mm', 10)
        self.rotate_pub = self.create_publisher(Int32, '/rotate_angle_deg', 10)

        self.create_subscription(
            CompressedImage,
            CAMERA_TOPIC,
            self._on_compressed_frame,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RosImage,
            OVERLAY_TOPIC,
            self._on_overlay_frame,
            qos_profile_sensor_data,
        )
        self.ui_queue.put(('log', f'Subscribed to {CAMERA_TOPIC}'))
        self.ui_queue.put(('log', f'Watching for YOLO overlay on {OVERLAY_TOPIC}'))

    def _on_compressed_frame(self, msg: CompressedImage) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2_decode(data)
        if frame is None:
            self.ui_queue.put(('log', 'Failed to decode compressed camera frame'))
            return
        with self.frame_lock:
            self.latest_camera_frame = frame
            self.latest_camera_time = time.time()

    def _on_overlay_frame(self, msg: RosImage) -> None:
        frame = decode_raw_image(msg)
        if frame is None:
            return
        with self.frame_lock:
            self.latest_overlay_frame = frame
            self.latest_overlay_time = time.time()

    def get_display_frame(self):
        with self.frame_lock:
            now = time.time()
            if (
                self.latest_overlay_frame is not None
                and now - self.latest_overlay_time <= OVERLAY_RECENCY_SEC
            ):
                return self.latest_overlay_frame.copy(), 'YOLO overlay'
            if self.latest_camera_frame is not None:
                return self.latest_camera_frame.copy(), 'camera'
        return None, 'waiting'

    def publish_move_mm(self, value: int) -> None:
        self.move_pub.publish(Int32(data=int(value)))
        direction = 'forward' if value >= 0 else 'backward'
        self.ui_queue.put(('log', f'Sent /move_distance_mm={value} ({direction})'))

    def publish_rotate_deg(self, value: int) -> None:
        self.rotate_pub.publish(Int32(data=int(value)))
        direction = 'CCW' if value >= 0 else 'CW'
        self.ui_queue.put(('log', f'Sent /rotate_angle_deg={value} ({direction})'))


def cv2_decode(data: np.ndarray):
    import cv2

    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def decode_raw_image(msg: RosImage):
    import cv2

    try:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'rgb8':
            frame = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return data.reshape((msg.height, msg.width, 3))
        if msg.encoding == 'mono8':
            frame = data.reshape((msg.height, msg.width))
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    except ValueError:
        return None
    return None


def bgr_to_tk_image(frame, size):
    import cv2

    resized = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb)
    return ImageTk.PhotoImage(pil_image)


def parse_distance_to_mm(value: float, unit: str) -> int:
    unit = unit.lower()
    if unit in ('mm', 'millimeter', 'millimeters'):
        return round(value)
    if unit in ('cm', 'centimeter', 'centimeters'):
        return round(value * 10.0)
    if unit in ('m', 'meter', 'meters'):
        return round(value * 1000.0)
    if unit in ('ft', 'foot', 'feet'):
        return round(value * 304.8)
    if unit in ('in', 'inch', 'inches'):
        return round(value * 25.4)
    raise ValueError(f'Unsupported distance unit: {unit}')


def build_help_text() -> str:
    return (
        'Examples:\n'
        '  forward 3 ft\n'
        '  back 250 mm\n'
        '  move 1.2 m\n'
        '  left 90 deg\n'
        '  right 45 deg\n'
        '  rotate -30 deg\n'
        '  move_mm 500\n'
        '  rotate_deg -90\n'
        '\n'
        'Notes:\n'
        '  forward/move use /move_distance_mm\n'
        '  left/right/rotate use /rotate_angle_deg\n'
        '  positive rotate = CCW, negative rotate = CW'
    )


def parse_operator_command(text: str):
    cmd = text.strip().lower()
    if not cmd:
        return None
    if cmd in ('help', '?'):
        return ('help', None)

    move_match = re.fullmatch(
        r'(forward|back|backward|move)\s+(-?\d+(?:\.\d+)?)\s*(mm|cm|m|ft|feet|foot|in|inch|inches)?',
        cmd,
    )
    if move_match:
        action, value_text, unit = move_match.groups()
        value = float(value_text)
        unit = unit or 'mm'
        mm = parse_distance_to_mm(value, unit)
        if action in ('back', 'backward'):
            mm = -abs(mm)
        return ('move_mm', mm)

    rotate_match = re.fullmatch(
        r'(left|right|rotate)\s+(-?\d+(?:\.\d+)?)\s*(deg|degree|degrees)?',
        cmd,
    )
    if rotate_match:
        action, value_text, _unit = rotate_match.groups()
        deg = round(float(value_text))
        if action == 'right':
            deg = -abs(deg)
        elif action == 'left':
            deg = abs(deg)
        return ('rotate_deg', deg)

    raw_move_match = re.fullmatch(r'move_mm\s+(-?\d+)', cmd)
    if raw_move_match:
        return ('move_mm', int(raw_move_match.group(1)))

    raw_rotate_match = re.fullmatch(r'rotate_deg\s+(-?\d+)', cmd)
    if raw_rotate_match:
        return ('rotate_deg', int(raw_rotate_match.group(1)))

    raise ValueError(f'Unsupported command: {text}')


class OperatorConsoleApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry('900x820')

        self.ui_queue: queue.Queue = queue.Queue()
        self.node = OperatorConsoleNode(self.ui_queue)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        self.last_frame_time = 0.0
        self.video_image = None
        self.log_lines = []
        self.video_status_var = tk.StringVar(value='Video source: waiting')

        self._build_ui()
        self._append_log('Operator console ready. Type "help" for command syntax.')
        self.root.after(50, self._drain_ui_queue)
        self.root.after(80, self._refresh_video)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=3)
        self.root.rowconfigure(1, weight=2)

        video_frame = ttk.LabelFrame(self.root, text='Live Camera')
        video_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=(10, 6))
        video_frame.columnconfigure(0, weight=1)
        video_frame.rowconfigure(0, weight=1)

        self.video_label = ttk.Label(video_frame, text='Waiting for camera frames...')
        self.video_label.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)
        ttk.Label(video_frame, textvariable=self.video_status_var).grid(
            row=1, column=0, sticky='w', padx=8, pady=(0, 8)
        )

        controls = ttk.LabelFrame(self.root, text='Movement Console')
        controls.grid(row=1, column=0, sticky='nsew', padx=10, pady=(6, 10))
        controls.columnconfigure(0, weight=1)
        controls.rowconfigure(2, weight=1)

        quick_frame = ttk.Frame(controls)
        quick_frame.grid(row=0, column=0, sticky='ew', padx=8, pady=(8, 4))
        for idx in range(4):
            quick_frame.columnconfigure(idx, weight=1)

        quick_buttons = [
            ('Forward 1 ft', lambda: self._send_move_feet(1.0)),
            ('Back 1 ft', lambda: self._send_move_feet(-1.0)),
            ('Left 90', lambda: self._send_rotate_deg(90)),
            ('Right 90', lambda: self._send_rotate_deg(-90)),
        ]
        for idx, (label, callback) in enumerate(quick_buttons):
            ttk.Button(quick_frame, text=label, command=callback).grid(
                row=0, column=idx, sticky='ew', padx=4, pady=4
            )

        entry_frame = ttk.Frame(controls)
        entry_frame.grid(row=1, column=0, sticky='ew', padx=8, pady=4)
        entry_frame.columnconfigure(0, weight=1)

        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(entry_frame, textvariable=self.command_var)
        self.command_entry.grid(row=0, column=0, sticky='ew', padx=(0, 6))
        self.command_entry.bind('<Return>', self._on_submit)

        ttk.Button(entry_frame, text='Send', command=self._on_submit).grid(
            row=0, column=1, sticky='e'
        )

        help_label = ttk.Label(
            controls,
            text='Try: forward 3 ft, back 250 mm, left 90 deg, rotate -45 deg',
        )
        help_label.grid(row=2, column=0, sticky='w', padx=8, pady=(0, 6))

        self.log_widget = tk.Text(controls, height=12, wrap='word', state='disabled')
        self.log_widget.grid(row=3, column=0, sticky='nsew', padx=8, pady=(0, 8))

        scrollbar = ttk.Scrollbar(controls, orient='vertical', command=self.log_widget.yview)
        scrollbar.grid(row=3, column=1, sticky='ns', pady=(0, 8))
        self.log_widget.configure(yscrollcommand=scrollbar.set)

        self.command_entry.focus_set()

    def _send_move_feet(self, feet: float) -> None:
        mm = parse_distance_to_mm(abs(feet), 'ft')
        if feet < 0:
            mm = -mm
        self.node.publish_move_mm(mm)

    def _send_rotate_deg(self, deg: int) -> None:
        self.node.publish_rotate_deg(deg)

    def _on_submit(self, _event=None) -> None:
        raw = self.command_var.get().strip()
        if not raw:
            return
        self.command_var.set('')
        self._append_log(f'> {raw}')
        try:
            parsed = parse_operator_command(raw)
            if parsed is None:
                return
            command, value = parsed
            if command == 'help':
                self._append_log(build_help_text())
            elif command == 'move_mm':
                self.node.publish_move_mm(int(value))
            elif command == 'rotate_deg':
                self.node.publish_rotate_deg(int(value))
        except Exception as exc:
            self._append_log(f'Error: {exc}')

    def _append_log(self, text: str) -> None:
        for line in text.splitlines():
            self.log_lines.append(line)
        self.log_lines = self.log_lines[-LOG_MAX_LINES:]
        self.log_widget.configure(state='normal')
        self.log_widget.delete('1.0', tk.END)
        self.log_widget.insert(tk.END, '\n'.join(self.log_lines) + '\n')
        self.log_widget.configure(state='disabled')
        self.log_widget.see(tk.END)

    def _drain_ui_queue(self) -> None:
        while True:
            try:
                kind, payload = self.ui_queue.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self._append_log(payload)
        self.root.after(50, self._drain_ui_queue)

    def _refresh_video(self) -> None:
        frame, source = self.node.get_display_frame()
        if frame is not None:
            self.video_image = bgr_to_tk_image(frame, VIDEO_SIZE)
            self.video_label.configure(image=self.video_image, text='')
            self.last_frame_time = time.time()
            self.video_status_var.set(f'Video source: {source}')
        elif time.time() - self.last_frame_time > 2.0:
            self.video_label.configure(text='Waiting for camera frames...', image='')
            self.video_status_var.set('Video source: waiting')
        self.root.after(80, self._refresh_video)

    def _on_close(self) -> None:
        try:
            self.node.destroy_node()
        finally:
            rclpy.shutdown()
            self.root.destroy()


def main() -> int:
    rclpy.init()
    root = tk.Tk()
    OperatorConsoleApp(root)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
