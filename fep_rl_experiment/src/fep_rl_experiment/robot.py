"""ROS1 interface for Franka Emika Panda robot control.

This module provides the Robot class which handles all ROS communication with
the Franka Panda manipulator and its peripherals (gripper, camera, sensors).
It manages publishers, subscribers, services, and TF2 transforms for the robot.
"""

from typing import Optional, Tuple
from collections import deque

import rospy
import numpy as np
import numpy.typing as npt
import cv2
import time
from franka_gripper.msg import HomingAction, HomingGoal, StopActionGoal, MoveActionGoal
from control_msgs.msg import GripperCommandActionGoal
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_srvs.srv import Empty, SetBool, SetBoolResponse
import tf2_ros


class Robot:
    """ROS1 interface for Franka Emika Panda robot.

    This class encapsulates all ROS communication with the Franka Panda robot,
    including cartesian impedance control, gripper control, camera input,
    joint states, and ArUco-based cube tracking via TF2.

    The robot operates in a constrained workspace with safety checks for
    position bounds and end-effector velocity.

    Attributes:
        current_tip_pos: Current end-effector position [x, y, z] in meters.
        current_tip_quat: Current end-effector orientation [x, y, z, w] quaternion.
        joint_state: Current joint positions (7 arm joints + 2 gripper fingers).
        latest_image: Most recent preprocessed camera image (64x64 grayscale, normalized).
        last_image_time: Timestamp of latest camera image.
        last_tip_pos_time: Timestamp of latest end-effector pose update.
        last_joint_state_time: Timestamp of latest joint state update.
        last_cube_time: Timestamp of latest cube TF transform lookup.
        goal_tip_quat: Target orientation quaternion for end-effector.
        start_pos: Home position [x, y, z] for reset operations.
        ee_velocity_estimator: Linear velocity estimator for safety monitoring.
        tf_buffer: TF2 buffer for transform lookups.
        listener: TF2 transform listener.
        bridge: OpenCV-ROS bridge for image conversion.
    """

    def __init__(self, init_node: bool = False) -> None:
        """Initialize the Robot interface.

        Sets up all ROS publishers, subscribers, services, and TF2 listeners
        for communication with the Franka Panda robot and peripherals.

        Args:
            init_node: If True, initializes a new ROS node. Set to False if
                the node is already initialized elsewhere.
        """
        if init_node:
            rospy.init_node("franka_emika_robot_interface")
        self._desired_ee_pose_pub = rospy.Publisher(
            "/cartesian_impedance_example_controller/equilibrium_pose",
            PoseStamped,
            queue_size=1,
        )

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback, queue_size=1
        )
        self.gripper_command_pub = rospy.Publisher(
            "/franka_gripper/gripper_action/goal",
            GripperCommandActionGoal,
            queue_size=1,
        )
        self.stop_action_pub = rospy.Publisher(
            "/franka_gripper/stop/goal", StopActionGoal, queue_size=1
        )
        self.move_action_pub = rospy.Publisher(
            "/franka_gripper/move/goal", MoveActionGoal, queue_size=1
        )
        self.image_pub = rospy.Publisher("processed_image", Image, queue_size=1)
        self.ee_pose_sub = rospy.Subscriber(
            "/cartesian_impedance_example_controller/measured_pose",
            PoseStamped,
            self.ee_pose_callback,
            queue_size=1,
        )
        self.qpos_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback, queue_size=1
        )
        # Setup reset service
        self.reset_service = rospy.Service(
            "reset_controller",
            Empty,
            self.reset_service_cb,
        )
        self.start_service = rospy.Service(
            "start_controller",
            SetBool,
            self.start_service_cb,
        )
        self._action_scale = 0.005
        self.current_tip_pos = None
        self.joint_state = None
        self.latest_image = None
        self.last_image_time = None
        self.last_tip_pos_time = None
        self.last_joint_state_time = None
        self.last_cube_time = None
        self.goal_tip_quat = np.array([-1.0, 0.0, 0.0, 0.0])
        self.start_pos = np.array([6.6105e-1, -0.05, 1.7906836e-01])
        self.ee_velocity_estimator = LinearVelocityEstimator()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def image_callback(self, msg: Image) -> None:
        """Process incoming camera images.

        Converts ROS Image messages to OpenCV format, preprocesses to 64x64
        grayscale normalized format, and publishes the processed image.

        Args:
            msg: ROS Image message from the RealSense camera.
        """
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            grayscale = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            image = _preprocess_image(grayscale)
            self.latest_image = image[..., None]
            self.last_image_time = msg.header.stamp
            output_msg = self.bridge.cv2_to_imgmsg(
                (image * 255).astype(np.uint8), encoding="mono8"
            )
            output_msg.header.stamp = rospy.Time.now()  # Optional: add timestamp
            self.image_pub.publish(output_msg)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def ee_pose_callback(self, msg: PoseStamped) -> None:
        """Update end-effector pose from measured pose topic.

        Extracts position and orientation, updates velocity estimator, and
        stores timestamp.

        Args:
            msg: PoseStamped message with current end-effector pose.
        """
        pos = msg.pose.position
        self.current_tip_pos = np.array([pos.x, pos.y, pos.z])
        # Update internal orientation too if needed
        ori = msg.pose.orientation
        self.current_tip_quat = np.array([ori.x, ori.y, ori.z, ori.w])
        self.ee_velocity_estimator.add_measurement(
            self.current_tip_pos, msg.header.stamp
        )
        self.last_tip_pos_time = msg.header.stamp

    def joint_state_callback(self, msg: JointState) -> None:
        """Update joint state from JointState topic.

        Args:
            msg: JointState message with current joint positions.
        """
        self.joint_state = np.array(msg.position)
        self.last_joint_state_time = msg.header.stamp

    def get_camera_image(self) -> npt.NDArray[np.float32]:
        """Get the latest preprocessed camera image.

        Returns:
            64x64 grayscale image normalized to [0, 1], shape (64, 64, 1).
        """
        return self.latest_image

    def get_joint_state(self) -> npt.NDArray[np.float64]:
        """Get current joint positions.

        Returns:
            Array of 9 joint positions: 7 arm joints + 2 gripper fingers.
        """
        return self.joint_state

    def get_cube_pos(self, frame: str = "panda_link0") -> Optional[npt.NDArray[np.float64]]:
        """Get cube position from ArUco marker via TF2 lookup.

        Args:
            frame: Target reference frame for the transform (default: panda_link0).

        Returns:
            3D position [x, y, z] of the cube, or None if transform fails.
        """
        try:
            transformed_pose = self.tf_buffer.lookup_transform(
                frame, "aruco_cube_frame", rospy.Time(0)
            )
            pos = transformed_pose.transform.translation
            self.last_cube_time = transformed_pose.header.stamp
            return np.array([pos.x, pos.y, pos.z])
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as e:
            rospy.logerr(f"Transform error in get_cube_pos: {e}")
            return None

    def get_cube_quat(self, frame: str = "panda_link0") -> Optional[npt.NDArray[np.float64]]:
        """Get cube orientation from ArUco marker via TF2 lookup.

        Args:
            frame: Target reference frame for the transform (default: panda_link0).

        Returns:
            Quaternion [x, y, z, w] of the cube orientation, or None if transform fails.
        """
        try:
            transformed_pose = self.tf_buffer.lookup_transform(
                frame, "aruco_cube_frame", rospy.Time(0)
            )
            quat = transformed_pose.transform.rotation
            return np.array([quat.x, quat.y, quat.z, quat.w])
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as e:
            rospy.logerr(f"Transform error in get_cube_quat: {e}")
            return None

    def start_service_cb(self, req: SetBool) -> Tuple[bool, str]:
        """Service callback to start/stop the controller.

        Args:
            req: SetBool request with data field (True to start, False to stop).

        Returns:
            Tuple of (success, message).
        """
        self._running = req.data
        return True, "Started controller."

    def reset_service_cb(self, req: Empty) -> list:
        """Service callback to reset the robot to home position.

        Opens gripper and commands end-effector to start_pos with goal orientation.

        Args:
            req: Empty service request (unused).

        Returns:
            Empty list (required by ROS1 service protocol).
        """
        rospy.loginfo("Resetting robot...")
        self.open_gripper()
        target_pose = PoseStamped()
        target_pose.header.frame_id = "panda_link0"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = float(self.start_pos[0])
        target_pose.pose.position.y = float(self.start_pos[1])
        target_pose.pose.position.z = float(self.start_pos[2])
        # Set orientation from quaternion
        target_pose.pose.orientation.x = float(self.goal_tip_quat[0])
        target_pose.pose.orientation.y = float(self.goal_tip_quat[1])
        target_pose.pose.orientation.z = float(self.goal_tip_quat[2])
        target_pose.pose.orientation.w = float(self.goal_tip_quat[3])
        self._desired_ee_pose_pub.publish(target_pose)
        self._running = False
        return []

    def act(self, action: npt.NDArray[np.float32]) -> Optional[npt.NDArray[np.float64]]:
        """Execute a cartesian velocity action and gripper command.

        Applies a scaled delta to the end-effector position (Y-Z plane only,
        X is fixed to start_pos[0]) and commands gripper to open or close.

        Args:
            action: Action array [dx, dy, dz, gripper]. dx, dy, dz are velocity
                commands in [-1, 1] (scaled by action_scale=0.005). Gripper < 0
                means close, >= 0 means open.

        Returns:
            New target tip position [x, y, z], or None if robot not ready.

        Note:
            X-axis is fixed to prevent forward/backward movement. Only Y-Z motion
            is executed. Workspace bounds are enforced via clipping.
        """
        if not self.ok:
            rospy.logwarn("Not ready yet. Cannot execute action.")
            return
        # Scale and apply limits
        delta_pos = action[:3] * self._action_scale
        new_tip_pos = self.current_tip_pos + delta_pos

        # Clip new tip position within safe workspace bounds
        new_tip_pos[0] = np.clip(new_tip_pos[0], 0.25, 0.77)
        new_tip_pos[1] = np.clip(new_tip_pos[1], -0.32, 0.32)
        new_tip_pos[2] = np.clip(new_tip_pos[2], 0.02, 0.5)
        # Publish EE pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = float(self.start_pos[0])
        pose_msg.pose.position.y = float(new_tip_pos[1])
        pose_msg.pose.position.z = float(new_tip_pos[2])
        pose_msg.pose.orientation.x = float(self.goal_tip_quat[0])
        pose_msg.pose.orientation.y = float(self.goal_tip_quat[1])
        pose_msg.pose.orientation.z = float(self.goal_tip_quat[2])
        pose_msg.pose.orientation.w = float(self.goal_tip_quat[3])
        self._desired_ee_pose_pub.publish(pose_msg)
        fingers = self.joint_state[-2:].mean()
        if action[3] >= 0.0:
            rospy.loginfo("Opening gripper")
            goal = GripperCommandActionGoal()
            goal.header.stamp = rospy.Time.now()
            if fingers < 0.015:
                stop = StopActionGoal()
                stop.header.stamp = rospy.Time.now()
                self.stop_action_pub.publish(stop)
                position = 0.04
            else:
                position = np.clip(fingers + 0.01, 0.0, 0.0402)
            goal.goal.command.position = position
            goal.goal.command.max_effort = 0.0
            self.gripper_command_pub.publish(goal)
        else:
            rospy.loginfo("Closing Gripper")
            goal = GripperCommandActionGoal()
            goal.header.stamp = rospy.Time.now()
            goal.goal.command.position = np.clip(fingers - 0.01, 0.0, 0.0402)
            goal.goal.command.max_effort = 50.0
            self.gripper_command_pub.publish(goal)
        return new_tip_pos

    def get_end_effector_pos(self) -> npt.NDArray[np.float64]:
        """Get current end-effector position.

        Returns:
            3D position [x, y, z] of the end-effector.
        """
        return self.current_tip_pos

    def open_gripper(self) -> None:
        """Command gripper to fully open (0.08m width)."""
        if not self.ok:
            rospy.logwarn("Not ready yet. Cannot open gripper.")
            return
        move = MoveActionGoal()
        move.header.stamp = rospy.Time.now()
        move.goal.width = 0.08
        move.goal.speed = 10.0
        self.move_action_pub.publish(move)

    @property
    def fingers_open(self) -> bool:
        """Check if gripper fingers are in open position.

        Returns:
            True if gripper width >= 2.5cm, False otherwise.
        """
        if self.joint_state is None:
            return False
        fingers = self.joint_state[-2:].mean()
        return fingers >= 0.025

    @property
    def ok(self) -> bool:
        """Check if all robot sensors and state are available.

        Returns:
            True if all required data (pose, image, cube position, joints) is
            available, False otherwise.
        """
        ready = True
        if self.current_tip_pos is None:
            rospy.logwarn("current_tip_pos is None")
            ready = False
        if self.latest_image is None:
            rospy.logwarn("latest_image is None")
            ready = False
        if self.get_cube_pos() is None:
            rospy.logwarn("get_cube_pos() returned None")
            ready = False
        if self.joint_state is None:
            rospy.logwarn("joint_state is None")
            ready = False
        return ready

    @property
    def safe(self) -> bool:
        """Check if robot is operating safely within bounds and velocity limits.

        Returns:
            True if end-effector is within 0.3m of start_pos and velocity is
            below 0.5 m/s in all axes, False otherwise.
        """
        pos = self.get_end_effector_pos()
        out_of_bounds = np.any(np.abs(pos - self.start_pos) > 0.3)
        if out_of_bounds:
            rospy.logwarn(
                f"Robot out of bounds. Position is: {self.get_end_effector_pos()}"
            )
        velocity = self.ee_velocity_estimator.estimate_velocity()
        high_velocity = np.any(np.abs(velocity) > 0.5)
        if high_velocity:
            rospy.logwarn(f"EE high velocity. Velocity is: {velocity}")
        return not out_of_bounds and not high_velocity


class LinearVelocityEstimator:
    """Estimates linear velocity from position measurements using least squares.

    Uses a sliding window of position measurements with timestamps to compute
    velocity via linear regression. Useful for safety monitoring.

    Attributes:
        window_size: Number of measurements to retain in the sliding window.
        positions: Deque of 3D position measurements.
        timestamps: Deque of corresponding timestamps in seconds.
    """

    def __init__(self, window_size: int = 10) -> None:
        """Initialize velocity estimator.

        Args:
            window_size: Maximum number of measurements to keep (default: 10).
        """
        self.window_size = window_size
        self.positions = deque(maxlen=window_size)  # List of 3D position vectors
        self.timestamps = deque(maxlen=window_size)  # Corresponding timestamps

    def add_measurement(self, position: npt.NDArray[np.float64], timestamp: rospy.Time) -> None:
        """Add a new position measurement to the sliding window.

        Args:
            position: 3D position vector [x, y, z].
            timestamp: ROS timestamp of the measurement.
        """
        self.positions.append(np.array(position))
        self.timestamps.append(float(timestamp.to_sec()))

    def estimate_velocity(self) -> Optional[npt.NDArray[np.float64]]:
        """Compute velocity estimate via least squares linear regression.

        Fits a linear model p(t) = v*t + b to the position measurements and
        returns the slope v as the velocity estimate.

        Returns:
            Estimated velocity [vx, vy, vz] in m/s, or None if insufficient data.
        """
        if len(self.positions) < 2 or np.array(self.timestamps).std() < 1e-6:
            return None  # Not enough data yet
        t = np.array(self.timestamps)
        p = np.vstack(self.positions)  # Shape: (N, 3)
        # Normalize time to improve numerical stability
        t_centered = t - t.mean()
        # Solve for slope a in p = a*t + b, using least squares
        A = t_centered[:, np.newaxis]  # Shape: (N, 1)
        v_est, _, _, _ = np.linalg.lstsq(A, p - p.mean(axis=0), rcond=None)
        return v_est.flatten()  # Shape: (3,)


def _crop_and_resize(grayscale: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
    """Crop image to square and resize to 64x64.

    Crops from the center to make the image square (using height as the target
    dimension), then resizes to 64x64 using bilinear interpolation.

    Args:
        grayscale: Input grayscale image.

    Returns:
        64x64 grayscale image.
    """
    height, width = grayscale.shape
    # Determine side crop to make the image square
    new_width = height  # because height is smaller dimension
    crop_amount = (width - new_width) // 2  # crop equally from both sides
    # Perform square crop
    cropped = grayscale[:, crop_amount : crop_amount + new_width]
    cropped = cv2.resize(cropped, (64, 64), interpolation=cv2.INTER_LINEAR)
    return cropped


def _preprocess_image(grayscale: npt.NDArray[np.uint8]) -> npt.NDArray[np.float32]:
    """Preprocess camera image to 64x64 normalized grayscale.

    Crops, resizes, and normalizes the image to [0, 1] float32 format.

    Args:
        grayscale: Input grayscale image (uint8).

    Returns:
        64x64 grayscale image normalized to [0, 1] as float32.
    """
    grayscale = _crop_and_resize(grayscale)
    # Normalize to [0, 1] and convert to float32
    normalized = grayscale.astype(np.float32) / 255.0
    return normalized
