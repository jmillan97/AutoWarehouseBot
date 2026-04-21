"""
summon_node.py
==============
AutoWarehouseBot — ros2 branch
Laptop / WSL2 side — runs alongside Nav2

ROS2 action client that receives summon goals from summon_server.py
and sends NavigateToPose goals to Nav2.

New in hardware version:
  - Publishes an initial pose to /initialpose on startup so AMCL
    can localize without a manual RViz "2D Pose Estimate" click.
    Set the initial_pose_* params to where the robot starts in your map.
  - Exposes a /summon/goal topic so summon_server can publish goals
    directly without the static/dynamic endpoint split.
  - State machine: IDLE → NAVIGATING → ARRIVED/FAILED → IDLE

Run:
  ros2 run wb_summon summon_node

  With custom start pose:
  ros2 run wb_summon summon_node --ros-args \
    -p initial_pose_x:=1.0 \
    -p initial_pose_y:=0.5 \
    -p initial_pose_theta:=0.0
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class SummonNode(Node):

    def __init__(self):
        super().__init__('summon_node')

        # ── Parameters ──────────────────────────────────────────
        # Set these to the robot's starting position in your map.
        # Run the SLAM mapping phase first, then note where the robot
        # starts (usually near the origin — 0, 0, 0).
        self.declare_parameter('initial_pose_x',     0.0)
        self.declare_parameter('initial_pose_y',     0.0)
        self.declare_parameter('initial_pose_theta', 0.0)   # radians
        self.declare_parameter('auto_set_initial_pose', True)
        self.declare_parameter('initial_pose_delay_sec', 3.0)  # wait for AMCL to start

        self.init_x     = self.get_parameter('initial_pose_x').value
        self.init_y     = self.get_parameter('initial_pose_y').value
        self.init_theta = self.get_parameter('initial_pose_theta').value
        self.auto_pose  = self.get_parameter('auto_set_initial_pose').value
        self.pose_delay = self.get_parameter('initial_pose_delay_sec').value

        # ── State ────────────────────────────────────────────────
        self.state        = 'IDLE'
        self.current_goal = None
        self._goal_handle = None
        self._lock        = threading.Lock()

        # ── Nav2 action client ───────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Subscribers ──────────────────────────────────────────
        # summon_server publishes goals here as "x,y,theta" strings
        self.create_subscription(
            String, '/summon/goal', self._goal_callback, 10)

        # ── Publishers ───────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/summon/status', 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Auto initial pose ────────────────────────────────────
        if self.auto_pose:
            threading.Thread(target=self._delayed_initial_pose, daemon=True).start()

        self.get_logger().info(
            f'Summon node initialized and IDLE. '
            f'Start pose will be set at ({self.init_x}, {self.init_y}, {self.init_theta:.2f}rad) '
            f'after {self.pose_delay}s delay.'
        )

    # ── Initial pose broadcast ───────────────────────────────────────────────
    def _delayed_initial_pose(self):
        """
        Wait for AMCL to start, then publish the initial pose.
        This replaces the manual RViz '2D Pose Estimate' click for hardware demos.
        """
        time.sleep(self.pose_delay)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.init_x
        msg.pose.pose.position.y = self.init_y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(self.init_theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.init_theta / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0

        # Covariance — moderate uncertainty on x/y, low on z/roll/pitch, moderate on yaw
        # [0]=x, [7]=y, [35]=yaw in the 6x6 flattened covariance matrix
        cov = [0.0] * 36
        cov[0]  = 0.5   # x uncertainty [m^2]
        cov[7]  = 0.5   # y uncertainty [m^2]
        cov[35] = 0.26  # yaw uncertainty [rad^2] ~30 degrees
        msg.pose.covariance = cov

        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'Published initial pose to AMCL: '
            f'({self.init_x:.2f}, {self.init_y:.2f}, {math.degrees(self.init_theta):.1f}°)'
        )

    # ── Goal subscriber callback ─────────────────────────────────────────────
    def _goal_callback(self, msg: String):
        """
        Parse "x,y,theta" from /summon/goal and send to Nav2.
        Published by summon_server.py when a REST call comes in.
        """
        try:
            parts = msg.data.strip().split(',')
            x     = float(parts[0])
            y     = float(parts[1])
            theta = float(parts[2]) if len(parts) > 2 else 0.0
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Bad goal message [{msg.data}]: {e}')
            return

        with self._lock:
            if self.state == 'NAVIGATING':
                self.get_logger().warn('Already navigating — cancelling previous goal')
                self._cancel_current()

        self._send_goal(x, y, theta)

    # ── Send NavigateToPose goal ─────────────────────────────────────────────
    def _send_goal(self, x: float, y: float, theta: float):
        self.get_logger().info(f'Waiting for Nav2 action server...')

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available after 10s')
            self._transition('FAILED')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, theta)
        goal_msg.behavior_tree = ''   # use default BT

        self._transition('NAVIGATING')
        self.current_goal = {'x': x, 'y': y, 'theta': theta}
        self.get_logger().info(f'Sending goal to Nav2: ({x:.2f}, {y:.2f})')

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self._transition('FAILED')
            return

        self.get_logger().info('Nav2 goal accepted')
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result   = future.result()
        status   = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
            self._transition('ARRIVED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal cancelled')
            self._transition('IDLE')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            self._transition('FAILED')

        self.current_goal = None
        self._goal_handle = None

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.2f}m', throttle_duration_sec=2.0)

    # ── Cancel ───────────────────────────────────────────────────────────────
    def _cancel_current(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def cancel_goal(self):
        with self._lock:
            self._cancel_current()

    # ── State machine ────────────────────────────────────────────────────────
    def _transition(self, new_state: str):
        self.get_logger().info(f'Transition: {self.state} -> {new_state}')
        self.state = new_state
        self._publish_status()

        # Auto-reset FAILED → IDLE after 5 seconds
        if new_state == 'FAILED':
            threading.Timer(5.0, lambda: self._transition('IDLE')).start()

    def _publish_status(self):
        msg = String()
        msg.data = self.state
        self._status_pub.publish(msg)

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _make_pose(self, x: float, y: float, theta: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        return pose


# ── Main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SummonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()