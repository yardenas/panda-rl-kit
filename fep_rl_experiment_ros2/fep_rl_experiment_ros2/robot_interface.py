#!/usr/bin/env python3
"""Test node for Franka robot interface (ROS2).

This node demonstrates basic robot control by executing a predefined sequence
of actions (move, open/close gripper) and then resetting to home position.
Useful for testing robot communication and action execution.
"""

from typing import Optional, List
import time
import rclpy
import numpy as np
import numpy.typing as npt
from fep_rl_experiment_ros2.robot import Robot


def main(args: Optional[List[str]] = None) -> None:
    """Execute test action sequence and reset robot.

    Runs through 4 test actions (forward, right, up, back) with gripper
    commands, then resets to starting position.

    Args:
        args: Optional command line arguments for rclpy.init().
    """
    rclpy.init(args=args)
    robot = Robot(node_name='robot_interface')
    robot.get_logger().info("Robot node is running.")

    # Wait for publishers to be ready
    time.sleep(1.0)

    # Define a series of test actions: [dx, dy, dz, gripper]
    # gripper >= 0 to open, < 0 to close
    test_actions: List[npt.NDArray[np.float64]] = [
        np.array([1., 0.0, 0.0, 1.0]),   # Move forward, open gripper
        np.array([0.0, 1., 0.0, -1.0]),  # Move right, close gripper
        np.array([0.0, 0.0, 1., 1.0]),   # Move up, open gripper
        np.array([-1., 0.0, 0.0, -1.0]), # Move back, close gripper
    ]

    rate = robot.create_rate(20)
    for i, action in enumerate(test_actions):
        robot.get_logger().info(f"Sending action {i+1}: {action}")
        for _ in range(100):
            new_pos = robot.act(action)
            rate.sleep()
        robot.get_logger().info(f"New position: {new_pos}")

    time.sleep(1.0)
    robot.reset_service_cb(None, None)
    robot.get_logger().info("Resetting to starting position")
    time.sleep(1.0)

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
