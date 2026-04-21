#!/usr/bin/env python3
import os
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from math import cos, isfinite, sin
from tkinter import ttk

import cv2
import numpy as np
import rclpy
from PIL import Image, ImageTk
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Int32


CAMERA_TOPIC = os.environ.get('OPERATOR_CAMERA_TOPIC', '/camera/image_raw/compressed')
SCAN_TOPIC = os.environ.get('OPERATOR_SCAN_TOPIC', '/scan')
MODEL_PATH = os.environ.get('OPERATOR_YOLO_MODEL', os.path.expanduser('~/warehouse_project/yolov8n.pt'))
USE_YOLO = os.environ.get('OPERATOR_USE_YOLO', '1').strip().lower() not in ('0', 'false', 'no')
CONFIDENCE_THRESHOLD = float(os.environ.get('OPERATOR_YOLO_CONF', '0.4'))
APP_TITLE = 'Warehouse Bot Operator Console'
VIDEO_SIZE = (640, 480)
SCAN_CANVAS_SIZE = 360
SCAN_RATE_WINDOW = 30
LOG_MAX_LINES = 200


class OperatorConsoleNode(Node):
    def __init__(self, ui_queue: queue.Queue):
        super().__init__('operator_console')
        self.ui_queue = ui_queue
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_frame_source = 'waiting'
        self.model = None
        self.class_names = {}
        self.scan_lock = threading.Lock()
        self.latest_scan_points = []
        self.latest_scan_stats = {
            'source': 'waiting',
            'point_count': 0,
            'min_range': None,
            'rate_hz': 0.0,
            'range_max': 0.0,
        }
        self.scan_arrival_times = deque(maxlen=SCAN_RATE_WINDOW)

        self.move_pub = self.create_publisher(Int32, '/move_distance_mm', 10)
        self.rotate_pub = self.create_publisher(Int32, '/rotate_angle_deg', 10)

        if USE_YOLO:
            self._load_model()

        self.create_subscription(
            CompressedImage,
            CAMERA_TOPIC,
            self._on_compressed_frame,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.ui_queue.put(('log', f'Subscribed to {CAMERA_TOPIC}'))
        self.ui_queue.put(('log', f'Subscribed to {SCAN_TOPIC}'))
        if self.model is not None:
            self.ui_queue.put(('log', f'YOLO overlay enabled: {MODEL_PATH}'))
        else:
            self.ui_queue.put(('log', 'YOLO overlay disabled; showing plain camera feed'))

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            self.ui_queue.put(('log', 'ultralytics not installed; operator console will stay on plain camera'))
            return

        try:
            self.model = YOLO(MODEL_PATH)
            self.class_names = getattr(self.model.model, 'names', {}) or {}
        except Exception as exc:
            self.ui_queue.put(('log', f'Failed to load YOLO model {MODEL_PATH}: {exc}'))
            self.model = None

    def _on_compressed_frame(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.ui_queue.put(('log', 'Failed to decode compressed camera frame'))
            return

        source = 'camera'
        annotated = frame
        if self.model is not None:
            try:
                results = self.model.predict(
                    source=frame,
                    verbose=False,
                    conf=CONFIDENCE_THRESHOLD,
                )
                detections = self._results_to_detections(results)
                annotated = self._draw_detections(frame.copy(), detections)
                source = 'camera + yolo'
            except Exception as exc:
                self.ui_queue.put(('log', f'YOLO inference failed, falling back to plain camera: {exc}'))
                self.model = None

        with self.frame_lock:
            self.latest_frame = annotated
            self.latest_frame_source = source

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.time()
        self.scan_arrival_times.append(now)
        if len(self.scan_arrival_times) >= 2:
            elapsed = self.scan_arrival_times[-1] - self.scan_arrival_times[0]
            rate_hz = (len(self.scan_arrival_times) - 1) / elapsed if elapsed > 0 else 0.0
        else:
            rate_hz = 0.0

        points = []
        min_range = None
        angle = msg.angle_min
        range_min = max(0.0, float(msg.range_min))
        range_max = float(msg.range_max)
        for distance in msg.ranges:
            if isfinite(distance) and range_min <= distance <= range_max:
                points.append((distance * cos(angle), distance * sin(angle)))
                if min_range is None or distance < min_range:
                    min_range = float(distance)
            angle += msg.angle_increment

        with self.scan_lock:
            self.latest_scan_points = points
            self.latest_scan_stats = {
                'source': SCAN_TOPIC,
                'point_count': len(points),
                'min_range': min_range,
                'rate_hz': rate_hz,
                'range_max': range_max,
            }

    def _results_to_detections(self, results):
        detections = []
        for result in results:
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                detections.append({
                    'class_name': self.class_names.get(cls_id, str(cls_id)),
                    'confidence': conf,
                    'bbox_xyxy': [x1, y1, x2, y2],
                })
        return detections

    def _draw_detections(self, frame, detections):
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det['bbox_xyxy']]
            label = f"{det['class_name']} {det['confidence']:.2f}"
            color = (0, 200, 0) if det['class_name'] == 'person' else (0, 140, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        return frame

    def get_display_frame(self):
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.latest_frame_source
        return None, 'waiting'

    def get_scan_snapshot(self):
        with self.scan_lock:
            return list(self.latest_scan_points), dict(self.latest_scan_stats)

    def publish_move_mm(self, value: int) -> None:
        self.move_pub.publish(Int32(data=int(value)))
        direction = 'forward' if value >= 0 else 'backward'
        self.ui_queue.put(('log', f'Sent /move_distance_mm={value} ({direction})'))

    def publish_rotate_deg(self, value: int) -> None:
        self.rotate_pub.publish(Int32(data=int(value)))
        direction = 'right' if value >= 0 else 'left'
        self.ui_queue.put(('log', f'Sent /rotate_angle_deg={value} ({direction})'))


def bgr_to_tk_image(frame, size):
    resized = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return ImageTk.PhotoImage(Image.fromarray(rgb))


def format_scan_status(stats) -> str:
    if stats.get('point_count', 0) == 0:
        return 'LiDAR: waiting'
    min_range = stats.get('min_range')
    min_text = f'{min_range:.2f} m' if min_range is not None else 'n/a'
    return (
        f"LiDAR: {stats.get('rate_hz', 0.0):.1f} Hz avg | "
        f"{stats.get('point_count', 0)} points | nearest {min_text}"
    )


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
        '  left/right commands are mapped to the calibrated robot directions\n'
        '  raw rotate_deg keeps the signed ROS command value'
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
        mm = parse_distance_to_mm(value, unit or 'mm')
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
            deg = abs(deg)
        elif action == 'left':
            deg = -abs(deg)
        return ('rotate_deg', deg)

    raw_move_match = re.fullmatch(r'move_mm\s+(-?\d+)', cmd)
    if raw_move_match:
        return ('move_mm', int(raw_move_match.group(1)))

    raw_rotate_match = re.fullmatch(r'rotate_deg\s+(-?\d+)', cmd)
    if raw_rotate_match:
        return ('rotate_deg', int(raw_rotate_match.group(1)))

    raise ValueError(f'Unsupported command: {text}')


def parse_optional_int_field(text: str, label: str) -> int:
    value = text.strip()
    if not value:
        return 0
    if not re.fullmatch(r'[+-]?\d+', value):
        raise ValueError(f'{label} must be a whole number')
    return int(value)


class OperatorConsoleApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry('1120x820')

        self.ui_queue = queue.Queue()
        self.node = OperatorConsoleNode(self.ui_queue)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        self.last_frame_time = 0.0
        self.video_image = None
        self.log_lines = []
        self.video_status_var = tk.StringVar(value='Video source: waiting')
        self.scan_status_var = tk.StringVar(value='LiDAR: waiting')
        self.distance_mm_var = tk.StringVar()
        self.rotate_deg_var = tk.StringVar()

        self._build_ui()
        self._append_log('Operator console ready. Enter one number and press Send.')
        self.root.after(50, self._drain_ui_queue)
        self.root.after(80, self._refresh_video)
        self.root.after(100, self._refresh_scan)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=3)
        self.root.rowconfigure(1, weight=2)

        sensor_frame = ttk.Frame(self.root)
        sensor_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=(10, 6))
        sensor_frame.columnconfigure(0, weight=3)
        sensor_frame.columnconfigure(1, weight=2)
        sensor_frame.rowconfigure(0, weight=1)

        video_frame = ttk.LabelFrame(sensor_frame, text='Live Camera')
        video_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 6), pady=0)
        video_frame.columnconfigure(0, weight=1)
        video_frame.rowconfigure(0, weight=1)

        self.video_label = ttk.Label(video_frame, text='Waiting for camera frames...')
        self.video_label.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)
        ttk.Label(video_frame, textvariable=self.video_status_var).grid(
            row=1, column=0, sticky='w', padx=8, pady=(0, 8)
        )

        scan_frame = ttk.LabelFrame(sensor_frame, text='LiDAR Scan')
        scan_frame.grid(row=0, column=1, sticky='nsew', padx=(6, 0), pady=0)
        scan_frame.columnconfigure(0, weight=1)
        scan_frame.rowconfigure(0, weight=1)

        self.scan_canvas = tk.Canvas(
            scan_frame,
            width=SCAN_CANVAS_SIZE,
            height=SCAN_CANVAS_SIZE,
            background='#111318',
            highlightthickness=0,
        )
        self.scan_canvas.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)
        ttk.Label(scan_frame, textvariable=self.scan_status_var).grid(
            row=1, column=0, sticky='w', padx=8, pady=(0, 8)
        )

        controls = ttk.LabelFrame(self.root, text='Movement Controls')
        controls.grid(row=1, column=0, sticky='nsew', padx=10, pady=(6, 10))
        controls.columnconfigure(0, weight=1)
        controls.rowconfigure(3, weight=1)

        command_frame = ttk.Frame(controls)
        command_frame.grid(row=0, column=0, sticky='ew', padx=8, pady=(8, 4))
        command_frame.columnconfigure(1, weight=1)
        command_frame.columnconfigure(4, weight=1)

        ttk.Label(command_frame, text='Distance (mm)').grid(
            row=0, column=0, sticky='w', padx=(0, 6), pady=4
        )
        self.distance_entry = ttk.Entry(command_frame, textvariable=self.distance_mm_var, width=12)
        self.distance_entry.grid(row=0, column=1, sticky='ew', padx=(0, 12), pady=4)
        ttk.Label(command_frame, text='forward + / back -').grid(
            row=0, column=2, sticky='w', padx=(0, 18), pady=4
        )

        ttk.Label(command_frame, text='Rotate (deg)').grid(
            row=0, column=3, sticky='w', padx=(0, 6), pady=4
        )
        self.rotate_entry = ttk.Entry(command_frame, textvariable=self.rotate_deg_var, width=12)
        self.rotate_entry.grid(row=0, column=4, sticky='ew', padx=(0, 12), pady=4)
        ttk.Label(command_frame, text='right + / left -').grid(
            row=0, column=5, sticky='w', padx=(0, 0), pady=4
        )

        button_frame = ttk.Frame(controls)
        button_frame.grid(row=1, column=0, sticky='ew', padx=8, pady=4)
        button_frame.columnconfigure(0, weight=0)
        button_frame.columnconfigure(1, weight=0)
        button_frame.columnconfigure(2, weight=1)

        ttk.Button(button_frame, text='Send', command=self._on_send_numeric).grid(
            row=0, column=0, sticky='w', padx=(0, 6), pady=4
        )
        ttk.Button(button_frame, text='Clear', command=self._clear_numeric_fields).grid(
            row=0, column=1, sticky='w', padx=(0, 12), pady=4
        )

        ttk.Label(
            controls,
            text='Enter only one nonzero value per send to avoid overlapping motion commands.',
        ).grid(row=2, column=0, sticky='w', padx=8, pady=(0, 6))

        self.log_widget = tk.Text(controls, height=12, wrap='word', state='disabled')
        self.log_widget.grid(row=3, column=0, sticky='nsew', padx=8, pady=(0, 8))

        scrollbar = ttk.Scrollbar(controls, orient='vertical', command=self.log_widget.yview)
        scrollbar.grid(row=3, column=1, sticky='ns', pady=(0, 8))
        self.log_widget.configure(yscrollcommand=scrollbar.set)

        self.distance_entry.bind('<Return>', self._on_send_numeric)
        self.rotate_entry.bind('<Return>', self._on_send_numeric)
        self.distance_entry.focus_set()

    def _send_rotate_deg(self, deg: int) -> None:
        self.node.publish_rotate_deg(deg)

    def _clear_numeric_fields(self) -> None:
        self.distance_mm_var.set('')
        self.rotate_deg_var.set('')
        self.distance_entry.focus_set()

    def _on_send_numeric(self, _event=None) -> None:
        try:
            distance_mm = parse_optional_int_field(self.distance_mm_var.get(), 'Distance')
            rotate_deg = parse_optional_int_field(self.rotate_deg_var.get(), 'Rotate')
            if distance_mm == 0 and rotate_deg == 0:
                raise ValueError('Enter a nonzero distance or rotation')
            if distance_mm != 0 and rotate_deg != 0:
                raise ValueError('Send distance or rotation separately, not both at once')

            if distance_mm != 0:
                self.node.publish_move_mm(distance_mm)
                self.distance_mm_var.set('')
            else:
                self.node.publish_rotate_deg(rotate_deg)
                self.rotate_deg_var.set('')
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

    def _refresh_scan(self) -> None:
        points, stats = self.node.get_scan_snapshot()
        self._draw_scan(points, stats)
        self.scan_status_var.set(format_scan_status(stats))
        self.root.after(100, self._refresh_scan)

    def _draw_scan(self, points, stats) -> None:
        canvas = self.scan_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), SCAN_CANVAS_SIZE)
        height = max(canvas.winfo_height(), SCAN_CANVAS_SIZE)
        cx = width / 2.0
        cy = height / 2.0
        radius = min(width, height) * 0.44
        range_max = min(max(float(stats.get('range_max') or 4.0), 1.0), 8.0)
        scale = radius / range_max

        for meters in (0.5, 1.0, 2.0, 3.0):
            if meters > range_max:
                continue
            r = meters * scale
            canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline='#2b3440')
            canvas.create_text(cx + 4, cy - r, text=f'{meters:g}m', fill='#7d8792', anchor='w')

        canvas.create_line(cx, cy - radius, cx, cy + radius, fill='#26303b')
        canvas.create_line(cx - radius, cy, cx + radius, cy, fill='#26303b')
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill='#e8eef7', outline='')
        canvas.create_text(cx, cy + 18, text='robot', fill='#c7d0dc')

        for x_m, y_m in points:
            px = cx + y_m * scale
            py = cy - x_m * scale
            dist = (x_m * x_m + y_m * y_m) ** 0.5
            color = '#ff5757' if dist < 0.5 else '#ffb347' if dist < 1.0 else '#5fd38d'
            canvas.create_oval(px - 2, py - 2, px + 2, py + 2, fill=color, outline='')

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
