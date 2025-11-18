#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TabletNavBridge(Node):
    def __init__(self):
        super().__init__('tablet_nav_bridge')

        # Listen for goals from the tablet
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'tablet_goal',
            self.goal_callback,
            10
        )

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('TabletNavBridge node started. Waiting for /tablet_goal...')

    def goal_callback(self, pose_msg: PoseStamped):
        # Called whenever the tablet publishes a goal
        self.get_logger().info(
            f'Received tablet goal: x={pose_msg.pose.position.x:.2f}, '
            f'y={pose_msg.pose.position.y:.2f}'
        )

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 navigate_to_pose action server not available.')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose_msg

        self.get_logger().info('Sending goal to Nav2...')
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal was rejected.')
            return

        self.get_logger().info('Nav2 goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Nav2 navigation finished with status: {result.status}')


def main(args=None):
    rclpy.init(args=args)
    node = TabletNavBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
