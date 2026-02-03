#!/usr/bin/env python
"""Test node for Franka robot interface.

This node demonstrates basic robot control by executing a predefined sequence
of actions (move, open/close gripper) and then resetting to home position.
Useful for testing robot communication and action execution.
"""

from typing import List
import rospy
import numpy as np
import numpy.typing as npt
from fep_rl_experiment.robot import Robot


def main() -> None:
    """Execute test action sequence and reset robot.

    Runs through 4 test actions (forward, right, up, back) with gripper
    commands, then resets to starting position.
    """
    robot = Robot(init_node=True)
    rospy.loginfo("Robot node is running.")
    # Wait for Gazebo and publishers to be ready
    rospy.sleep(1.0)

    # Define a series of test actions: [dx, dy, dz, gripper]
    # gripper >= 0 to open, < 0 to close
    test_actions: List[npt.NDArray[np.float64]] = [
        np.array([1., 0.0, 0.0, 1.0]),   # Move forward, open gripper
        np.array([0.0, 1., 0.0, -1.0]),  # Move right, close gripper
        np.array([0.0, 0.0, 1., 1.0]),   # Move up, open gripper
        np.array([-1., 0.0, 0.0, -1.0]), # Move back, close gripper
    ]
    rate = rospy.Rate(20)
    for i, action in enumerate(test_actions):
        rospy.loginfo(f"Sending action {i+1}: {action}")
        for _ in range(100): 
            new_pos = robot.act(action)
            rate.sleep()
        rospy.loginfo(f"New position: {new_pos}")
    rospy.sleep(1.0)
    robot.reset_service_cb(None)
    rospy.loginfo("Resetting to starting position")
    rospy.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass