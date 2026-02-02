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
import rospy
import threading
import time
import numpy as np
import numpy.typing as npt
from std_msgs.msg import Float32
from fep_rl_experiment.transitions_server import TransitionsServer
from fep_rl_experiment.session import Session
from fep_rl_experiment.environment import PandaPickCube
from fep_rl_experiment.robot import Robot


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


class ExperimentDriver:
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
        rospy.init_node("franka_emika_robot_interface")
        self.dt = rospy.get_param("~dt")
        self.trajectory_length = rospy.get_param("~trajectory_length")
        session_id = rospy.get_param("~session_id")
        self.session = Session(filename=session_id, directory="experiment_sessions")
        num_steps = len(self.session.steps)
        self.robot = Robot()
        self.env = PandaPickCube(self.robot)
        self.running = False
        self.run_id = num_steps
        self.transitions_server = TransitionsServer(self, safe_mode=True)
        self.server_thread = threading.Thread(
            target=self.transitions_server.loop, daemon=True
        )
        self.server_thread.start()
        self.reward_pub = rospy.Publisher("instant_reward", Float32, queue_size=10)
        self.episode_reward_pub = rospy.Publisher(
            "episode_reward", Float32, queue_size=10
        )
        rospy.loginfo("Experiment driver initialized.")

    def sample_trajectory(
        self, policy: Callable[[Dict[str, npt.NDArray]], npt.NDArray[np.float32]]
    ) -> List[Transition]:
        """Sample a full trajectory using the given policy.

        Args:
            policy: Callable that maps observations to actions.

        Returns:
            List of Transition objects representing the trajectory.
        """
        rospy.loginfo(f"Starting trajectory sampling... Run id: {self.run_id}")
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
        for info in infos:
            for key, value in info.items():
                table_data[key] += value
        table_data["steps"] = len(infos)
        table_data["reward"] = float(
            sum(transition.reward for transition in transitions)
        )
        table_data["terminated"] = (
            1 - transitions[-1].discount and not infos[-1]["truncation"]
        )
        rospy.loginfo(
            f"Total reward: {table_data['reward']}\nTotal cost: {table_data['cost']}\n{_format_reward_summary(table_data)}"
        )
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
        obs = self.env.reset()
        while not self.robot_ok:
            rospy.loginfo("Waiting the robot to be ready...")
            time.sleep(2.5)
        ongoing_reward = 0.0
        while not done and steps < self.trajectory_length:
            start = time.time()
            action = policy(obs)
            next_obs, reward, done, info = self.env.step(action)
            self.reward_pub.publish(reward)
            self.episode_reward_pub.publish(ongoing_reward)
            ongoing_reward += reward
            end = time.time()
            elapsed = end - start
            steps += 1
            truncated = not done and steps == self.trajectory_length - 1
            transition = Transition(
                obs,
                action,
                reward,
                1 - done,
                next_obs,
                {"state_extras": {"truncation": truncated, **info}},
            )
            transition.extras["state_extras"]["time"] = elapsed
            transitions.append(transition)
            # Wait or warn if behind schedule
            remaining = self.dt - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)
            else:
                rospy.logwarn(
                    f"Iteration took too long: {elapsed:.4f}s > dt={self.dt:.4f}s"
                )
            obs = next_obs  # Advance observation for next step
        if not done:
            assert len(transitions) == self.trajectory_length
        self.episode_reward_pub.publish(ongoing_reward)
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
