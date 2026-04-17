#!/usr/bin/env python3
import json
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, String


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('annotated_image_topic', '/perception/yolo/annotated_image')
        self.declare_parameter('detections_topic', '/perception/yolo/detections')
        self.declare_parameter('people_topic', '/perception/yolo/people')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('person_only', False)
        self.declare_parameter('image_topic_type', 'auto')
        self.declare_parameter('publish_annotated_image', False)
        self.declare_parameter('display_overlay', True)
        self.declare_parameter('display_window_name', 'YOLO Overlay')

        self.image_topic = self.get_parameter('image_topic').value
        self.annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.people_topic = self.get_parameter('people_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.person_only = bool(self.get_parameter('person_only').value)
        self.image_topic_type = str(self.get_parameter('image_topic_type').value)
        self.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)
        self.display_overlay = bool(self.get_parameter('display_overlay').value)
        self.display_window_name = str(self.get_parameter('display_window_name').value)

        self.model = None
        self.class_names = {}
        self._load_model()
        self._subscription_mode = self._resolve_topic_type()
        if self._subscription_mode == 'compressed':
            self.create_subscription(CompressedImage, self.image_topic, self._on_compressed_image, 10)
        else:
            self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.people_pub = self.create_publisher(Int32, self.people_topic, 10)
        self.annotated_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(Image, self.annotated_image_topic, 10)

        self.get_logger().info(
            f'YOLO detector subscribed to {self.image_topic} ({self._subscription_mode})'
        )
        self.get_logger().info(f'Publishing detections on {self.detections_topic}')
        self.get_logger().info(f'Publishing people count on {self.people_topic}')
        if self.display_overlay:
            self.get_logger().info(f'Showing local overlay window: {self.display_window_name}')
        if self.publish_annotated_image:
            self.get_logger().info(f'Publishing annotated image on {self.annotated_image_topic}')

    def _resolve_topic_type(self) -> str:
        topic_type = self.image_topic_type.strip().lower()
        if topic_type in ('raw', 'compressed'):
            return topic_type
        if self.image_topic.endswith('/compressed'):
            return 'compressed'
        return 'raw'

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            self.get_logger().error(
                'ultralytics is not installed. Install it in WSL before enabling use_yolo.'
            )
            return

        try:
            self.model = YOLO(self.model_path)
            self.class_names = getattr(self.model.model, 'names', {}) or {}
            self.get_logger().info(f'Loaded YOLO model: {self.model_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model {self.model_path}: {exc}')
            self.model = None

    def _on_image(self, msg: Image) -> None:
        frame = self._decode_image(msg)
        if frame is None:
            return
        self._run_inference(frame, msg.header.stamp, msg.header.frame_id)

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        frame = self._decode_compressed_image(msg)
        if frame is None:
            return
        self._run_inference(frame, msg.header.stamp, '')

    def _run_inference(self, frame, stamp, frame_id: str) -> None:
        if self.model is None:
            return
        try:
            results = self.model.predict(
                source=frame,
                verbose=False,
                conf=self.confidence_threshold,
            )
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        detections = self._results_to_detections(results)
        if self.person_only:
            detections = [det for det in detections if det['class_name'] == 'person']

        people_count = sum(1 for det in detections if det['class_name'] == 'person')
        self.people_pub.publish(Int32(data=people_count))
        self.detections_pub.publish(String(data=json.dumps(detections)))

        annotated = None
        if self.display_overlay or self.publish_annotated_image:
            annotated = self._draw_detections(frame.copy(), detections)

        if self.display_overlay and annotated is not None:
            cv2.imshow(self.display_window_name, annotated)
            cv2.waitKey(1)

        if self.publish_annotated_image and annotated is not None and self.annotated_pub is not None:
            annotated_msg = self._encode_bgr_image(annotated, frame_id)
            annotated_msg.header.stamp = stamp
            self.annotated_pub.publish(annotated_msg)

    def _decode_image(self, msg: Image):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == 'rgb8':
                frame = frame.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if msg.encoding == 'bgr8':
                return frame.reshape((msg.height, msg.width, 3))
            if msg.encoding == 'mono8':
                gray = frame.reshape((msg.height, msg.width))
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except ValueError as exc:
            self.get_logger().warning(f'Bad image shape for {msg.encoding}: {exc}')
            return None

        self.get_logger().warning(f'Unsupported image encoding for YOLO node: {msg.encoding}')
        return None

    def _decode_compressed_image(self, msg: CompressedImage):
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        decoded = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        if decoded is None:
            self.get_logger().warning('Failed to decode compressed image frame for YOLO node')
        return decoded

    def _results_to_detections(self, results) -> List[dict]:
        detections: List[dict] = []
        for result in results:
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                class_name = self.class_names.get(cls_id, str(cls_id))
                detections.append({
                    'class_id': cls_id,
                    'class_name': class_name,
                    'confidence': round(conf, 4),
                    'bbox_xyxy': [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                })
        return detections

    def _draw_detections(self, frame, detections: List[dict]):
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

    def _encode_bgr_image(self, frame, frame_id: str) -> Image:
        msg = Image()
        msg.header.frame_id = frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg


def main() -> None:
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.display_overlay:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
