"""Pick-and-place task environment for Franka Panda robot.

This module implements the PandaPickCube environment which defines the
pick-and-place task, reward shaping, episode termination logic, and
observation/action spaces for the robot.
"""

from collections import deque
from typing import Any, NamedTuple, Dict, Tuple
import numpy as np
import numpy.typing as npt
import time
import rospy
from fep_rl_experiment.robot import Robot


class Transition(NamedTuple):
    """Container for a transition (deprecated, use experiment_driver.Transition).

    Attributes:
        observation: Observation dict.
        action: Action array.
        reward: Scalar reward.
        discount: Discount factor.
        next_observation: Next observation dict.
        extras: Additional metadata.
    """

    observation: Any
    action: Any
    reward: Any
    discount: Any
    next_observation: Any
    extras: Any = ()


# Reward function configuration
_REWARD_CONFIG = {
    "reward_scales": {
        "gripper_box": 4.0,
        "box_target": 8.0,
        "no_floor_collision": 0.25,
        "no_box_collision": 0.05,
        "robot_target_qpos": 0.0,
    },
    "action_rate": -0.0005,
    "no_soln_reward": -0.01,
    "lifted_reward": 0.5,
    "success_reward": 2.0,
}

# Success threshold: box height must be within 5cm of target height
_SUCCESS_THRESHOLD = 0.05


class PandaPickCube:
    """Pick-and-place environment for Panda robot with cube object.

    This environment implements a pick-and-place task where the robot must
    grasp a cube (tracked via ArUco markers) and lift it to a target height.

    The observation space includes:
    - Camera image (64x64 grayscale)
    - End-effector position (3D)
    - Gripper finger positions (2D, normalized)
    - Action history (last 5 gripper actions)

    The action space is:
    - dy: Y-axis velocity command [-1, 1]
    - dz: Z-axis velocity command [-1, 1]
    - gripper: Open (>= 0) or close (< 0)

    Note: X-axis is fixed to prevent robot from moving forward/backward.

    Attributes:
        robot: Robot interface for hardware communication.
        prev_reward: Previous cumulative reward (for progress-based rewards).
        reached_box: Binary flag indicating if gripper has reached box.
        target_pos: Target 3D position for the cube.
        target_quat: Target orientation quaternion.
        init_joint_state: Initial joint configuration.
        gripper_act: Deque storing recent gripper actions for observation.
    """

    def __init__(self, robot: Robot) -> None:
        """Initialize the pick-and-place environment.

        Args:
            robot: Robot instance for hardware control.
        """
        self.robot = robot
        self.prev_reward = 0.0
        self.reached_box = 0.0
        x_plane = self.robot.start_pos[0]
        self.target_pos = np.array([x_plane, -0.05, 0.2])
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.init_joint_state = np.array(
            [
                [
                    -0.00002,
                    0.47804,
                    -0.00055,
                    -1.81309,
                    -0.00161,
                    2.34597,
                    0.78501,
                    0.04000,
                    0.04000,
                ]
            ]
        )
        self.gripper_act: Deque[float] = deque(maxlen=5)

    def reset(self) -> Dict[str, npt.NDArray[np.float32]]:
        """Reset the environment to start a new episode.

        Resets the robot to home position, opens gripper, waits for cube
        to be placed on table, and returns the initial observation.

        Returns:
            Dictionary with keys:
                - 'pixels/view_0': Camera image (64, 64, 1) in [0, 1]
                - 'state': Proprioceptive state (10,) = [ee_pos(3), fingers(2), actions(5)]
        """
        self.robot.reset_service_cb(None)
        time.sleep(0.5)
        while not self.robot.fingers_open:
            rospy.logwarn(f"Fingers are not open: {self.robot.joint_state[-2:].mean()}")
            self.robot.open_gripper()
            time.sleep(1.)
        box_pos = self.robot.get_cube_pos()
        while box_pos[2] > 0.05:
            rospy.logwarn(f"Cube is not on the table: {box_pos}")
            time.sleep(1.)
            box_pos = self.robot.get_cube_pos()
        self.prev_reward = 0.0
        self.reached_box = 0.0
        self.gripper_act.extend([0.0] * 5)
        time.sleep(2.0)
        img = self.robot.get_camera_image()
        ee = self.robot.get_end_effector_pos()
        action_history = np.array(self.gripper_act)
        fingers = self.robot.get_joint_state()[-2:] / 0.04
        propreiceptive = np.concatenate([ee, fingers, action_history])
        obs = {"pixels/view_0": img, "state": propreiceptive}
        return obs

    def step(
        self, action: npt.NDArray[np.float32]
    ) -> Tuple[
        Dict[str, npt.NDArray[np.float32]],
        float,
        bool,
        Dict[str, Any]
    ]:
        """Execute one step in the environment.

        Args:
            action: Action array [dy, dz, gripper]. dy and dz are velocity
                commands in [-1, 1], gripper >= 0 to open, < 0 to close.

        Returns:
            Tuple of (observation, reward, done, info):
                - observation: Dict with 'pixels/view_0' and 'state' keys
                - reward: Progress-based reward (always non-negative)
                - done: True if episode should terminate
                - info: Dict with reward components and task metrics
        """
        only_yz = np.concatenate(
            ([self.robot.start_pos[0] - self.robot.get_end_effector_pos()[0]], action)
        )
        self.robot.act(only_yz)
        raw_rewards = self._get_reward()
        rewards = {
            k: v * _REWARD_CONFIG["reward_scales"][k] for k, v in raw_rewards.items()
        }
        hand_box = False
        raw_rewards["no_box_collision"] = np.where(hand_box, 0.0, 1.0)
        total_reward = np.clip(sum(rewards.values()), -1e4, 1e4)
        box_pos = self.robot.get_cube_pos()
        total_reward += (box_pos[2] > 0.05) * _REWARD_CONFIG["lifted_reward"]
        success = np.linalg.norm(box_pos[2] - self.target_pos[2]) < _SUCCESS_THRESHOLD
        total_reward += success * _REWARD_CONFIG["success_reward"]
        # Progress reward
        reward = max(total_reward - self.prev_reward, 0.0)
        self.prev_reward = max(reward + self.prev_reward, self.prev_reward)
        # Observations
        img = self.robot.get_camera_image()
        ee = self.robot.get_end_effector_pos()
        self.gripper_act.append(action[-1])
        action_history = np.array(self.gripper_act)
        fingers = self.robot.get_joint_state()[-2:] / 0.04
        propreiceptive = np.concatenate([ee, fingers, action_history])
        obs = {"pixels/view_0": img, "state": propreiceptive}
        out_of_bounds = np.any(np.abs(box_pos) > 1.0)
        out_of_bounds |= box_pos[2] < 0.0
        done = not self.robot.safe or success
        info = {
            **raw_rewards,
            "reached_box": self.reached_box,
            "success": success,
            "lifted": box_pos[2] > 0.05,
        }
        return obs, reward, done, info

    def _get_reward(self) -> Dict[str, float]:
        """Compute raw reward components before scaling.

        Returns:
            Dictionary mapping reward component names to unscaled values in [0, 1]:
                - gripper_box: Proximity of gripper to box (1 = touching)
                - box_target: Proximity of box to target (1 = at target)
                - no_floor_collision: 1 if no collision, 0 if collision
                - robot_target_qpos: Joint configuration similarity to home
        """
        box_pos = self.robot.get_cube_pos()
        # FIXME (yarden): double check that end effector pos == gripper pos
        gripper_pos = self.robot.get_end_effector_pos()
        pos_err = np.linalg.norm(box_pos - self.target_pos)
        box_target = 1.0 - np.tanh(5 * (pos_err))
        gripper_box = 1 - np.tanh(5 * np.linalg.norm(box_pos - gripper_pos))
        qpos = self.robot.get_joint_state()
        robot_target_qpos = 1 - np.tanh(np.linalg.norm(qpos - self.init_joint_state))
        hand_floor_collision = gripper_pos[-1] < -0.001
        no_floor_collision = 1 - hand_floor_collision
        self.reached_box = np.maximum(
            self.reached_box, np.linalg.norm(box_pos - gripper_pos) < 0.025
        )
        rewards = {
            "gripper_box": gripper_box,
            "box_target": box_target * self.reached_box,
            "no_floor_collision": no_floor_collision,
            "robot_target_qpos": robot_target_qpos,
        }
        return rewards
