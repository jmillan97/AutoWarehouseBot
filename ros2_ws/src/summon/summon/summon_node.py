"""
summon_node.py
==============
AutoWarehouseBot — ros2 branch
Laptop / WSL2 side — runs alongside Nav2

Direct planner+controller version — bypasses bt_navigator entirely
to avoid the FastDDS discovery bug in WSL2 where bt_navigator's
internal rclcpp_node cannot discover action servers.

Instead of NavigateToPose (bt_navigator), this node:
  1. Calls /compute_path_to_pose (planner_server) to get a path
  2. Calls /follow_path (controller_server) to execute the path

State machine: IDLE → NAVIGATING → ARRIVED/FAILED → IDLE
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from summon_msgs.msg import SummonGoal


class SummonNode(Node):

    def __init__(self):
        super().__init__('summon_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('initial_pose_x',     0.0)
        self.declare_parameter('initial_pose_y',     0.0)
        self.declare_parameter('initial_pose_theta', 0.0)
        self.declare_parameter('auto_set_initial_pose', True)
        self.declare_parameter('initial_pose_delay_sec', 3.0)

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

        # ── Action clients (direct to planner + controller) ──────
        self._plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            SummonGoal, '/summon/goal', self._goal_callback, 10)

        # ── Publishers ───────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/summon/status', 10)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── Auto initial pose ────────────────────────────────────
        if self.auto_pose:
            threading.Thread(target=self._delayed_initial_pose, daemon=True).start()

        self.get_logger().info(
            f'Summon node initialized (direct planner+controller mode). '
            f'Start pose: ({self.init_x}, {self.init_y}, {self.init_theta:.2f}rad)'
        )

    # ── Initial pose broadcast ───────────────────────────────────
    def _delayed_initial_pose(self):
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

        cov = [0.0] * 36
        cov[0]  = 0.5
        cov[7]  = 0.5
        cov[35] = 0.26
        msg.pose.covariance = cov

        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'Published initial pose: '
            f'({self.init_x:.2f}, {self.init_y:.2f}, {math.degrees(self.init_theta):.1f}deg)'
        )

    # ── Goal subscriber callback ─────────────────────────────────
    def _goal_callback(self, msg: SummonGoal):
        try:
            x     = msg.x
            y     = msg.y
            theta = msg.theta if hasattr(msg, 'theta') else 0.0
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Bad goal message: {e}')
            return

        with self._lock:
            if self.state == 'NAVIGATING':
                self.get_logger().warn('Already navigating — cancelling previous goal')
                self._cancel_current()

        self._plan_and_follow(x, y, theta)

    # ── Plan then follow ─────────────────────────────────────────
    def _plan_and_follow(self, x: float, y: float, theta: float):
        self.get_logger().info('Waiting for planner action server...')
        if not self._plan_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Planner action server not available')
            self._transition('FAILED')
            return

        self._transition('NAVIGATING')
        self.current_goal = {'x': x, 'y': y, 'theta': theta}

        # Step 1: Compute path
        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = self._make_pose(x, y, theta)
        plan_goal.planner_id = 'GridBased'
        plan_goal.use_start = False  # use current robot pose as start

        self.get_logger().info(f'Computing path to ({x:.2f}, {y:.2f})...')
        plan_future = self._plan_client.send_goal_async(plan_goal)
        plan_future.add_done_callback(self._plan_response_callback)

    def _plan_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Plan goal rejected')
            self._transition('FAILED')
            return

        self.get_logger().info('Plan goal accepted, waiting for path...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result_callback)

    def _plan_result_callback(self, future):
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Planning failed with status: {result.status}')
            self._transition('FAILED')
            return

        path = result.result.path
        if len(path.poses) == 0:
            self.get_logger().error('Planner returned empty path')
            self._transition('FAILED')
            return

        self.get_logger().info(f'Path received with {len(path.poses)} poses. Following...')

        # Step 2: Follow the path
        if not self._follow_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Controller action server not available')
            self._transition('FAILED')
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'

        follow_future = self._follow_client.send_goal_async(
            follow_goal,
            feedback_callback=self._follow_feedback_callback
        )
        follow_future.add_done_callback(self._follow_response_callback)

    def _follow_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Follow goal rejected')
            self._transition('FAILED')
            return

        self.get_logger().info('Controller accepted path')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_result_callback)

    def _follow_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
            self._transition('ARRIVED')
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal cancelled')
            self._transition('IDLE')
        else:
            self.get_logger().warn(f'Follow failed with status: {result.status}')
            self._transition('FAILED')

        self.current_goal = None
        self._goal_handle = None

    def _follow_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(
            f'Distance remaining: {dist:.2f}m', throttle_duration_sec=2.0)

    # ── Cancel ───────────────────────────────────────────────────
    def _cancel_current(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def cancel_goal(self):
        with self._lock:
            self._cancel_current()

    # ── State machine ────────────────────────────────────────────
    def _transition(self, new_state: str):
        self.get_logger().info(f'Transition: {self.state} -> {new_state}')
        self.state = new_state
        self._publish_status()

        if new_state == 'FAILED':
            threading.Timer(5.0, lambda: self._transition('IDLE')).start()

    def _publish_status(self):
        msg = String()
        msg.data = self.state
        self._status_pub.publish(msg)

    # ── Helpers ──────────────────────────────────────────────────
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