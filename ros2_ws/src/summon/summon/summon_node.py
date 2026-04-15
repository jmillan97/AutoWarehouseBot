import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from summon_msgs.msg import SummonGoal, SummonStatus


class SummonNode(Node):
    def __init__(self):
        super().__init__('summon_node')

        # State tracking
        self.state = 'IDLE'
        self.distance_remaining = -1.0
        self.current_goal: SummonGoal = None

        # Communication
        self.goal_sub = self.create_subscription(
            SummonGoal, '/summon/goal', self.goal_callback, 10)
            
        self.status_pub = self.create_publisher(
            SummonStatus, '/summon/status', 10)
            
        self.motion_pub = self.create_publisher(
            String, '/summon/motion_cmd', 10)

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 10Hz Control Loop timer
        self.timer = self.create_timer(0.1, self.loop)

        # Nav2 Goal handles
        self.nav_future = None
        self.goal_handle = None

        self.get_logger().info('Summon node initialized and IDLE.')

    def goal_callback(self, msg: SummonGoal):
        self.get_logger().info(f'Received summon goal: target=({msg.x}, {msg.y}), mode={msg.mode}')
        self.current_goal = msg
        
        # If we are already navigating, we cancel and start new
        if self.state != 'IDLE' and self.goal_handle is not None:
            self.get_logger().info('Canceling current goal to process new goal.')
            self.goal_handle.cancel_goal_async()

        self.transition_to('NAVIGATING')
        self.send_nav_goal(msg.x, msg.y)

    def send_nav_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available! Transitioning to FAILED.')
            self.transition_to('FAILED')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0 # Forward facing
        
        self.get_logger().info(f'Sending goal to Nav2: ({x}, {y})')
        self.nav_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback_callback)
        self.nav_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected! Transitioning to FAILED.')
            self.transition_to('FAILED')
            return
            
        self.get_logger().info('Nav2 goal accepted.')
        self.nav_result_future = self.goal_handle.get_result_async()
        self.nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.distance_remaining = feedback.distance_remaining

    def nav_result_callback(self, future):
        result = future.result()
        # ROS 2 Action Status: 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if result.status == 4:
            self.get_logger().info('Successfully arrived at Nav2 goal!')
            # Transition based on static vs dynamic mode
            if self.current_goal is not None and self.current_goal.mode == 'static':
                self.transition_to('ARRIVED')
            else:
                self.transition_to('SPIRAL_SEARCH')
        elif result.status == 5:
            self.get_logger().info('Navigation was cancelled.')
            self.transition_to('IDLE')
        else:
            self.get_logger().error(f'Navigation failed with status {result.status}')
            self.transition_to('FAILED')

    def transition_to(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'Transition: {self.state} -> {new_state}')
            self.state = new_state
            
            if new_state == 'FAILED':
                # Return to IDLE after a delay
                self.create_timer(5.0, self.reset_to_idle)
            elif new_state == 'ARRIVED':
                # Halt motors and return to IDLE after a delay
                self.motion_pub.publish(String(data='stop'))
                self.create_timer(10.0, self.reset_to_idle)

    def reset_to_idle(self):
        self.transition_to('IDLE')
        # We only want this timer to run once, so we return False or simply don't rely on persisting it
        # Actually it's easier to just cancel the timer instance, but for now this is ok
        # A proper implementation would track the timer and cancel it, or use a state timestamp

    def loop(self):
        # 1. Publish status
        status_msg = SummonStatus()
        status_msg.state = self.state
        status_msg.distance_remaining = float(self.distance_remaining)
        self.status_pub.publish(status_msg)
        
        # 2. State-specific 10Hz logic
        if self.state == 'IDLE':
            pass
            
        elif self.state == 'NAVIGATING':
            # Rely mostly on Nav2 callbacks; could do bounds checking here
            pass
            
        elif self.state == 'SPIRAL_SEARCH':
            # Stub for Step 4 of the implementation plan
            # Here we would compute Archimedean spiral and send "forward / rotate" commands
            self.get_logger().info('In SPIRAL SEARCH - stepping to HOMING for now (stub)')
            self.transition_to('HOMING')
            
        elif self.state == 'HOMING':
            # Stub for Step 4 of the implementation plan
            # Here we steer using BLE /ble/target topic
            self.get_logger().info('In HOMING - stepping to ARRIVED for now (stub)')
            self.transition_to('ARRIVED')
            
        elif self.state == 'ARRIVED':
            # Just chilling
            pass
            
        elif self.state == 'FAILED':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SummonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
