import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class ArUcoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        self.bridge = CvBridge()
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.params = cv2.aruco.DetectorParameters()
        
        # Load Landmarks
        self.config_path = os.path.join(
            get_package_share_directory('summon'), 'landmarks.yaml')
        self.landmarks = {}
        self.load_landmarks()

        # Camera calibration placeholders (will be filled by CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.img_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, 10)

        # Publisher (sends pose reset to EKF)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info("ArUco node started. Listening for /camera/image_raw/compressed")

    def load_landmarks(self):
        try:
            with open(self.config_path, 'r') as f:
                self.landmarks = yaml.safe_load(f)
            self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks from config.")
        except Exception as e:
            self.get_logger().error(f"Failed to load landmarks: {e}")

    def info_callback(self, msg: CameraInfo):
        # Read the K matrix and D coefficients
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: CompressedImage):
        if self.camera_matrix is None:
            return # Wait for calibration info
            
        # 1. Decode Compressed Image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 2. Detect Markers
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.dict, parameters=self.params)
        
        if ids is not None:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                if marker_id in self.landmarks:
                    # 3. Estimate Pose of Marker
                    # marker_size is 0.15m (plan §Phase 3.5)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i:i+1], 0.15, self.camera_matrix, self.dist_coeffs)
                    
                    # 4. Calculate Robot Global Pose
                    # For a simple implementation, we publish the landmark pose 
                    # whenever we see it as a "coarse reset"
                    self.reset_pose(marker_id)

    def reset_pose(self, marker_id):
        data = self.landmarks[marker_id]
        x, y, yaw_deg = data
        
        yaw = np.radians(yaw_deg)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        
        # Euler to Quaternion
        msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        # Very low covariance so EKF trusts it
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.01 # x
        msg.pose.covariance[7] = 0.01 # y
        msg.pose.covariance[35] = 0.01 # yaw
        
        self.pose_pub.publish(msg)
        self.get_logger().info(f"SEE MARKER {marker_id}: Resetting robot pose to ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
