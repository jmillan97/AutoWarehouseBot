#!/usr/bin/env python3
"""Display /camera/image_raw/compressed in a native window via WSLg.

Subscribes to the compressed topic so the Pi forwards raw MJPEG bytes
without decoding; decoding happens here on the PC via cv2.imdecode.
"""

import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

TOPIC = '/camera/image_raw/compressed'
NO_DATA_TIMEOUT = 5.0


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self._last_frame = None
        self._first_frame_time = None
        self._lock = threading.Lock()

        self.create_subscription(CompressedImage, TOPIC, self._image_cb, 10)
        print(f'Subscribed to {TOPIC} — waiting for frames...')

    def _image_cb(self, msg: CompressedImage) -> None:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            if self._first_frame_time is None:
                self._first_frame_time = time.monotonic()
                h, w = frame.shape[:2]
                print(f'First frame received: {w}x{h} ({msg.format})')
            self._last_frame = frame

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

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    start = time.monotonic()
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
                break
            cv2.waitKey(100)
            continue

        cv2.imshow('Camera Feed — /camera/image_raw  (q to quit)', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
