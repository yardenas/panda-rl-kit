#!/usr/bin/env python3
"""Dummy cube pose publisher for simulation (ROS2).

This node publishes a static cube pose to the 'pose' topic for testing the
pick-and-place pipeline without ArUco marker detection. The cube is placed
at a fixed position on the table.
"""

from typing import Optional, List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DummyCubePublisher(Node):
    """ROS2 node that publishes static cube pose at 10 Hz.

    The cube is placed at (0.66105, 0.0, 0.05) in the panda_link0 frame,
    representing a position on the table in front of the robot.

    Attributes:
        pub: Publisher for PoseStamped messages.
        timer: Timer for periodic publishing.
    """

    def __init__(self) -> None:
        """Initialize the dummy cube publisher node."""
        super().__init__('dummy_cube_publisher')
        self.pub = self.create_publisher(PoseStamped, 'pose', 10)
        # Create timer at 10 Hz
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def timer_callback(self) -> None:
        """Timer callback to generate and publish a static cube pose."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "panda_link0"
        # Fill in dummy pose values
        pose_msg.pose.position.x = 0.66105
        pose_msg.pose.position.y = 0.
        pose_msg.pose.position.z = 0.05

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.pub.publish(pose_msg)


def main(args: Optional[List[str]] = None) -> None:
    """Run the dummy cube publisher node.

    Args:
        args: Optional command line arguments for rclpy.init().
    """
    rclpy.init(args=args)
    node = DummyCubePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
