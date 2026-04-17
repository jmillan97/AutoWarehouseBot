#!/usr/bin/env python3
import os
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


TOPIC = os.environ.get('CAMERA_TOPIC', '/camera/image_raw/compressed')
WINDOW = f'Camera Feed - {TOPIC}  (q to quit)'
TIMEOUT_SEC = 5.0


class CompressedCameraViewer(Node):
    def __init__(self):
        super().__init__('compressed_camera_viewer')
        self.frame = None
        self.frame_lock = threading.Lock()
        self.first_frame = threading.Event()
        self.create_subscription(
            CompressedImage,
            TOPIC,
            self._on_frame,
            qos_profile_sensor_data,
        )

    def _on_frame(self, msg: CompressedImage) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode compressed frame')
            return
        with self.frame_lock:
            self.frame = frame
        if not self.first_frame.is_set():
            h, w = frame.shape[:2]
            print(f'First frame received: {w}x{h} ({msg.format or "compressed"})')
            self.first_frame.set()


def main() -> int:
    rclpy.init()
    node = CompressedCameraViewer()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f'Subscribed to {TOPIC} — waiting for frames...')
    if not node.first_frame.wait(timeout=TIMEOUT_SEC):
        print(f'\n[ERROR] No frames received on {TOPIC} after {int(TIMEOUT_SEC)}s.')
        print('  -> Check that Pi bringup is running: ./scripts/start_pi_robot.sh')
        print('  -> Check that ROS network is up: ./scripts/check_topics.sh --snapshot')
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        return 1

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            with node.frame_lock:
                frame = None if node.frame is None else node.frame.copy()
            if frame is not None:
                cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
