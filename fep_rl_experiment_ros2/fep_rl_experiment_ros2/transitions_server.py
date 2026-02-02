"""ZMQ server for receiving ONNX policies and returning robot transitions.

This module implements the TransitionsServer which listens on a ZMQ REP socket,
receives serialized ONNX policies from remote trainers, executes rollouts on
the robot, and returns flattened transitions for training.
"""

import pickle
from typing import Mapping, List, Tuple, Dict, Any, Optional
import zmq
import time
import numpy as np
import numpy.typing as npt
import onnxruntime as ort


class TransitionsServer:
    """ZMQ server for policy evaluation and transition collection.

    This server binds to a ZMQ REP socket and waits for requests containing
    serialized ONNX policies. For each policy, it collects the requested number
    of transitions by executing rollouts on the robot, then returns the flattened
    trajectories.

    Attributes:
        experiment_driver: ExperimentDriver instance for trajectory sampling.
        address: ZMQ address to bind to (default: tcp://*:5559).
        safe_mode: If True, requires manual confirmation before each rollout.
    """

    def __init__(
        self,
        experiment_driver: Any,  # ExperimentDriver
        safe_mode: bool = False,
        address: str = "tcp://*:5559"
    ) -> None:
        """Initialize the transitions server.

        Args:
            experiment_driver: ExperimentDriver instance for sampling trajectories.
            safe_mode: If True, require manual confirmation before each trajectory.
            address: ZMQ bind address (default: tcp://*:5559).
        """
        self.experiment_driver = experiment_driver
        self.address = address
        self.safe_mode = safe_mode

    def loop(self) -> None:
        """Main server loop that processes incoming policy requests.

        Binds to the ZMQ socket and waits for messages containing (policy_bytes, num_steps).
        For each request, collects the requested transitions and sends back flattened data.
        """
        with zmq.Context() as ctx:
            with ctx.socket(zmq.REP) as socket:
                socket.bind(self.address)
                while True:
                    message = socket.recv()
                    policy, num_steps = pickle.loads(message)
                    if num_steps < self.experiment_driver.trajectory_length:
                        self.experiment_driver.get_logger().error("Invalid num_steps: {}".format(num_steps))
                    trials = self.run(policy, num_steps)
                    if trials is None:
                        continue
                    socket.send(pickle.dumps(trials))

    def run(self, policy: bytes, num_steps: int) -> Tuple:
        """Collect the requested number of transitions using the given policy.

        Executes multiple trajectories until num_steps transitions are collected.
        May truncate the final trajectory to reach exactly num_steps.

        Args:
            policy: Serialized ONNX policy bytes.
            num_steps: Total number of transitions to collect.

        Returns:
            Flattened transitions tuple: (observations, actions, rewards,
            next_observations, discounts, extras).
        """
        trials = []
        num_transitions = 0
        while num_transitions < num_steps:
            trial = self.do_trial(policy)
            new_num_transitions = len(trial)
            if num_transitions + new_num_transitions > num_steps:
                trial = trial[: num_steps - num_transitions]
                trial[-1].extras["state_extras"]["truncation"] = True
                self.experiment_driver.get_logger().info("Truncating trajectory")
            num_transitions += len(trial)
            trials.append(trial)
            self.experiment_driver.get_logger().info("Completed trial")
        transitions = flatten_trajectories(trials)
        assert len(transitions[2]) == num_steps, (
            f"Expected {num_steps} transitions, got {len(transitions)}"
        )
        return transitions

    def do_trial(self, policy_bytes: bytes) -> List:
        """Execute a single trajectory with the given policy.

        If safe_mode is enabled, waits for user confirmation before starting.
        Retries on RuntimeError (e.g., robot not ready).

        Args:
            policy_bytes: Serialized ONNX policy.

        Returns:
            List of Transition objects for the completed trajectory.
        """
        self.experiment_driver.get_logger().info("Starting sampling")
        if self.safe_mode:
            while True:
                answer = input("Press Y/y when ready to collect trajectory\n")
                if not (answer == "Y" or answer == "y"):
                    self.experiment_driver.get_logger().info("Skipping trajectory")
                    continue
                else:
                    break
        policy_fn = self.parse_policy(policy_bytes)
        while True:
            try:
                trajectory = self.experiment_driver.sample_trajectory(policy_fn)
                break
            except RuntimeError:
                self.experiment_driver.get_logger().warn("Could not sample trajectory")
        self.experiment_driver.get_logger().info("Sampling finished")
        return trajectory

    def parse_policy(self, policy_bytes: bytes):
        """Create an inference function from serialized ONNX policy.

        Args:
            policy_bytes: Serialized ONNX model bytes.

        Returns:
            Callable that maps observation dicts to action arrays.
        """
        session = ort.InferenceSession(
            policy_bytes,
            providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
        )
        # Get input and output names (assuming 1 input and 1 output)
        output_name = session.get_outputs()[0].name

        def infer(inputs: Mapping[str, np.ndarray]) -> np.ndarray:
            inputs = {k: v.astype(np.float32)[None] for k, v in inputs.items()}
            result = session.run([output_name], inputs)
            return result[0][0]

        return infer


def flatten_trajectories(trajectories: List[List]) -> Tuple[
    Dict[str, npt.NDArray[np.float32]],
    npt.NDArray[np.float32],
    npt.NDArray[np.float32],
    Dict[str, npt.NDArray[np.float32]],
    npt.NDArray[np.float32],
    Dict[str, Any]
]:
    """Flatten a list of trajectories into stacked arrays for training.

    Concatenates all transitions from multiple trajectories into single arrays
    for each field (observations, actions, rewards, etc.).

    Args:
        trajectories: List of trajectory lists, where each trajectory is a list
            of Transition objects.

    Returns:
        Tuple of (observations, actions, rewards, next_observations, discount, extras):
            - observations: Dict mapping keys to stacked arrays (N, ...)
            - actions: Stacked action array (N, action_dim)
            - rewards: Stacked reward array (N,)
            - next_observations: Dict mapping keys to stacked arrays (N, ...)
            - discount: Stacked discount array (N,)
            - extras: Dict with state_extras and policy_extras
    """
    observations = {
        key: np.array(
            [t.observation[key] for traj in trajectories for t in traj],
            dtype=np.float32,
        )
        for key in trajectories[0][0].observation
    }
    actions = np.array(
        [t.action for traj in trajectories for t in traj], dtype=np.float32
    )
    rewards = np.array(
        [t.reward for traj in trajectories for t in traj], dtype=np.float32
    )
    next_observations = {
        key: np.array(
            [t.next_observation[key] for traj in trajectories for t in traj],
            dtype=np.float32,
        )
        for key in trajectories[0][0].next_observation
    }
    discount = np.array(
        [t.discount for traj in trajectories for t in traj], dtype=np.float32
    )
    state_extras = {
        key: np.array(
            [t.extras["state_extras"][key] for traj in trajectories for t in traj],
            dtype=np.float32,
        )
        for key in trajectories[0][0].extras["state_extras"]
    }
    extras = {"state_extras": state_extras, "policy_extras": {}}
    return (
        observations,
        actions,
        rewards,
        next_observations,
        discount,
        extras,
    )
