#!/usr/bin/env python3
"""Display /camera/image_raw in a native window via WSLg."""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = '/camera/image_raw'
NO_DATA_TIMEOUT = 5.0
YUV422_ORDER = os.environ.get('CAMERA_YUV422_ORDER', 'uyvy').lower()


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self._last_frame = None
        self._first_frame_time = None
        self._lock = threading.Lock()

        self.create_subscription(Image, TOPIC, self._image_cb, qos_profile_sensor_data)
        print(f'Subscribed to {TOPIC} — waiting for frames...')

    def _image_cb(self, msg: Image) -> None:
        frame = self._decode_image(msg)
        if frame is None:
            return
        with self._lock:
            if self._first_frame_time is None:
                self._first_frame_time = time.monotonic()
                h, w = frame.shape[:2]
                print(f'First frame received: {w}x{h} ({msg.encoding})')
            self._last_frame = frame

    def _decode_image(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        encoding = msg.encoding.lower()

        if encoding in ('yuv422', 'yuv422_yuy2', 'yuyv'):
            try:
                frame = data.reshape((msg.height, msg.step))
            except ValueError:
                return None
            frame = frame[:, : msg.width * 2].reshape((msg.height, msg.width, 2))
            if encoding in ('yuv422_yuy2', 'yuyv'):
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            if YUV422_ORDER == 'yuy2':
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_UYVY)

        if encoding == 'rgb8':
            try:
                frame = data.reshape((msg.height, msg.width, 3))
            except ValueError:
                return None
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if encoding == 'bgr8':
            try:
                return data.reshape((msg.height, msg.width, 3))
            except ValueError:
                return None

        if encoding == 'mono8':
            try:
                frame = data.reshape((msg.height, msg.width))
            except ValueError:
                return None
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        print(f'Unsupported encoding: {msg.encoding}')
        return None

    def get_frame(self):
        with self._lock:
            return self._last_frame

    def seconds_since_first_frame(self):
        with self._lock:
            if self._first_frame_time is None:
                return None
            return time.monotonic() - self._first_frame_time


def main():
    rclpy.init()
    node = CameraViewer()
    print(f'Using YUV422 decode order: {YUV422_ORDER}')

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    start = time.monotonic()
    exit_code = 0
    while True:
        frame = node.get_frame()

        if frame is None:
            elapsed = time.monotonic() - start
            if elapsed > NO_DATA_TIMEOUT:
                print(
                    f'\n[ERROR] No frames received on {TOPIC} after {NO_DATA_TIMEOUT:.0f}s.\n'
                    '  -> Check that Pi bringup is running: ./scripts/start_pi_robot.sh\n'
                    '  -> Check that ROS network is up: ./scripts/check_topics.sh --snapshot'
                )
                exit_code = 1
                break
            cv2.waitKey(100)
            continue

        cv2.imshow('Camera Feed - /camera/image_raw  (q to quit)', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    node.destroy_node()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
