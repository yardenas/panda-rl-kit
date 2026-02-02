"""Experiment orchestration for online reinforcement learning.

This module coordinates the interaction between the robot, environment, policy,
and data collection for online RL experiments. It manages:
- Trajectory sampling and rollout execution
- Reward telemetry publishing
- Session logging and experiment tracking
- ZMQ server for policy communication
"""

from typing import Any, NamedTuple, Callable, Dict, List
from collections import defaultdict
import threading
import time

import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from fep_rl_experiment_ros2.transitions_server import TransitionsServer
from fep_rl_experiment_ros2.session import Session
from fep_rl_experiment_ros2.environment import PandaPickCube
from fep_rl_experiment_ros2.robot import Robot


class Transition(NamedTuple):
    """Container for a single environment transition.

    Attributes:
        observation: Observation dict with 'pixels/view_0' and 'state' keys.
        action: Action array [dy, dz, gripper] executed in this step.
        reward: Scalar reward received after taking the action.
        discount: Discount factor (0 if terminal, 1 otherwise).
        next_observation: Observation dict after taking the action.
        extras: Additional metadata including state_extras dict.
    """

    observation: Dict[str, npt.NDArray[np.float32]]
    action: npt.NDArray[np.float32]
    reward: float
    discount: float
    next_observation: Dict[str, npt.NDArray[np.float32]]
    extras: Dict[str, Any] = {}


class ExperimentDriver(Node):
    """Orchestrates online RL experiments on the Franka Panda robot.

    This class manages the full experiment lifecycle:
    - Initializes robot, environment, and ZMQ server
    - Collects trajectories using policies received over ZMQ
    - Publishes reward telemetry to ROS topics
    - Logs experiment data to CSV files

    The driver runs a ZMQ server in a background thread that receives ONNX
    policies from the training stack, executes rollouts, and returns transitions.

    Attributes:
        dt: Control loop timestep in seconds.
        trajectory_length: Maximum steps per trajectory.
        session: Session logger for experiment data.
        robot: Robot interface for hardware communication.
        env: Task environment (PandaPickCube).
        running: Whether the controller is actively running.
        run_id: Counter for trajectory numbers.
        transitions_server: ZMQ server for policy communication.
        server_thread: Background thread running the ZMQ server.
        reward_pub: Publisher for instantaneous reward values.
        episode_reward_pub: Publisher for cumulative episode rewards.
    """

    def __init__(self) -> None:
        """Initialize the experiment driver.

        Loads parameters, creates robot and environment instances, starts
        the ZMQ server thread, and sets up reward publishers.
        """
        super().__init__("franka_emika_robot_interface")

        # Declare and get ROS parameters
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("trajectory_length", 250)
        self.declare_parameter("session_id", "default_session")

        self.dt: float = self.get_parameter("dt").value
        self.trajectory_length: int = self.get_parameter("trajectory_length").value
        session_id: str = self.get_parameter("session_id").value

        # Initialize session logging
        self.session = Session(
            filename=session_id,
            directory="experiment_sessions",
            logger=self.get_logger()
        )
        num_steps = len(self.session.steps)

        # Initialize robot and environment
        self.robot = Robot()
        self.env = PandaPickCube(self.robot)
        self.running = False
        self.run_id = num_steps

        # Start ZMQ server for receiving policies
        self.transitions_server = TransitionsServer(self, safe_mode=True)
        self.server_thread = threading.Thread(
            target=self.transitions_server.loop, daemon=True
        )
        self.server_thread.start()

        # Set up reward telemetry publishers
        self.reward_pub = self.create_publisher(Float32, "instant_reward", 10)
        self.episode_reward_pub = self.create_publisher(
            Float32, "episode_reward", 10
        )

        self.get_logger().info("Experiment driver initialized.")

    def sample_trajectory(
        self, policy: Callable[[Dict[str, npt.NDArray]], npt.NDArray[np.float32]]
    ) -> List[Transition]:
        """Sample a full trajectory using the given policy.

        Args:
            policy: Callable that maps observations to actions.

        Returns:
            List of Transition objects representing the trajectory.
        """
        self.get_logger().info(f"Starting trajectory sampling... Run id: {self.run_id}")
        trajectory = self._collect_trajectory(policy)
        self.summarize_trial(trajectory)
        return trajectory

    def summarize_trial(self, transitions: List[Transition]) -> None:
        """Compute and log summary statistics for a completed trajectory.

        Aggregates reward components, success metrics, and episode statistics,
        then logs them to the session file and ROS logger.

        Args:
            transitions: List of transitions from the completed trajectory.
        """
        infos = [transition.extras["state_extras"] for transition in transitions]
        table_data: Dict[str, float] = defaultdict(float)

        # Aggregate reward components across all steps
        for info in infos:
            for key, value in info.items():
                table_data[key] += value

        # Compute episode-level metrics
        table_data["steps"] = len(infos)
        table_data["reward"] = float(
            sum(transition.reward for transition in transitions)
        )
        table_data["terminated"] = (
            1 - transitions[-1].discount and not infos[-1]["truncation"]
        )

        # Log summary
        self.get_logger().info(
            f"Total reward: {table_data['reward']}\n"
            f"Total cost: {table_data['cost']}\n"
            f"{_format_reward_summary(table_data)}"
        )

        # Save to session log
        self.session.update(table_data)
        self.run_id += 1

    @property
    def robot_ok(self) -> bool:
        """Check if robot is ready for operation.

        Returns:
            True if all robot sensors and state are available, False otherwise.
        """
        return self.robot.ok

    def _collect_trajectory(
        self, policy: Callable[[Dict[str, npt.NDArray]], npt.NDArray[np.float32]]
    ) -> List[Transition]:
        """Execute a full trajectory rollout with the given policy.

        Resets the environment, waits for robot readiness, then executes
        steps until termination or max length. Publishes reward telemetry
        and enforces the control loop timing.

        Args:
            policy: Callable that maps observations to actions.

        Returns:
            List of Transition objects for the trajectory.
        """
        transitions: List[Transition] = []
        done = False
        steps = 0

        # Reset environment and wait for robot
        obs = self.env.reset()
        while not self.robot_ok:
            self.get_logger().info("Waiting for robot to be ready...")
            time.sleep(2.5)

        ongoing_reward = 0.0

        # Main rollout loop
        while not done and steps < self.trajectory_length:
            start = time.time()

            # Get action from policy
            action = policy(obs)

            # Execute action in environment
            next_obs, reward, done, info = self.env.step(action)

            # Publish instantaneous reward
            reward_msg = Float32()
            reward_msg.data = float(reward)
            self.reward_pub.publish(reward_msg)

            # Publish cumulative episode reward
            episode_reward_msg = Float32()
            episode_reward_msg.data = ongoing_reward
            self.episode_reward_pub.publish(episode_reward_msg)

            ongoing_reward += reward
            end = time.time()
            elapsed = end - start
            steps += 1

            # Check if trajectory was truncated
            truncated = not done and steps == self.trajectory_length - 1

            # Create transition with metadata
            transition = Transition(
                obs,
                action,
                reward,
                1.0 - float(done),  # Discount is 0 if done, 1 otherwise
                next_obs,
                {"state_extras": {"truncation": truncated, **info}},
            )
            transition.extras["state_extras"]["time"] = elapsed
            transitions.append(transition)

            # Enforce control loop timing
            remaining = self.dt - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)
            else:
                self.get_logger().warn(
                    f"Iteration took too long: {elapsed:.4f}s > dt={self.dt:.4f}s"
                )

            obs = next_obs

        # Verify trajectory length for non-terminal episodes
        if not done:
            assert len(transitions) == self.trajectory_length

        # Publish final cumulative reward
        final_reward_msg = Float32()
        final_reward_msg.data = ongoing_reward
        self.episode_reward_pub.publish(final_reward_msg)

        return transitions


def _format_reward_summary(table_data: Dict[str, float]) -> str:
    """Format reward component summary as a table string.

    Args:
        table_data: Dictionary mapping reward component names to values.

    Returns:
        Formatted string with tabular layout of reward components.
    """
    lines = []
    header = f"{'Reward Component':<20} {'Total Value':>12}"
    lines.append(header)
    lines.append("-" * len(header))
    for key, value in table_data.items():
        lines.append(f"{key:<20} {value:>12.2f}")
    return "\n".join(lines)
