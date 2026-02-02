#!/usr/bin/env python
"""Main entry point for online RL experiments.

This node initializes the ExperimentDriver which coordinates robot control,
environment interaction, ZMQ server, and data logging for online learning.
"""

import rospy
from fep_rl_experiment.experiment_driver import ExperimentDriver


def main() -> None:
    """Initialize experiment driver and spin until shutdown."""
    ExperimentDriver()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass