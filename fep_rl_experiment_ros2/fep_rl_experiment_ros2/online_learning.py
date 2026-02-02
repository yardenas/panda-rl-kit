#!/usr/bin/env python3
"""Main entry point for online RL experiments (ROS2).

This node initializes the ExperimentDriver which coordinates robot control,
environment interaction, ZMQ server, and data logging for online learning.
"""

from typing import Optional, List
import rclpy
from fep_rl_experiment_ros2.experiment_driver import ExperimentDriver


def main(args: Optional[List[str]] = None) -> None:
    """Initialize experiment driver and spin until shutdown.

    Args:
        args: Optional command line arguments for rclpy.init().
    """
    rclpy.init(args=args)
    driver = ExperimentDriver()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
