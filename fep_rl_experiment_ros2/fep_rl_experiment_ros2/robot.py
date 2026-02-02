"""Robot interface for Franka Emika Panda manipulator.

This module provides the Robot class which handles all ROS2 communication with
the Franka Emika Panda robot, including:
- End-effector pose control
- Gripper commands
- Camera image acquisition
- Joint state monitoring
- ArUco marker tracking via TF2
"""

from collections import deque
from typing import Optional, Deque

import cv2
import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
import tf2_ros
from control_msgs.msg import GripperCommandActionGoal
from cv_bridge import CvBridge
from franka_gripper.msg import HomingAction, HomingGoal, StopActionGoal, MoveActionGoal
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Empty, SetBool


class Robot(Node):
    """ROS2 interface for Franka Emika Panda robot.

    This class manages all communication with the robot hardware, including
    publishers for control commands, subscribers for sensor data, and services
    for robot state management.

    Attributes:
        _desired_ee_pose_pub: Publisher for desired end-effector pose.
        bridge: CV bridge for image conversion.
        image_sub: Subscriber for camera images.
        gripper_command_pub: Publisher for gripper position commands.
        stop_action_pub: Publisher for gripper stop commands.
        move_action_pub: Publisher for gripper move commands.
        image_pub: Publisher for processed images.
        ee_pose_sub: Subscriber for end-effector pose feedback.
        qpos_sub: Subscriber for joint states.
        reset_service: Service for resetting robot to start position.
        start_service: Service for starting/stopping controller.
        current_tip_pos: Current end-effector position [x, y, z].
        joint_state: Current joint positions (7 arm + 2 gripper).
        latest_image: Most recent preprocessed camera image.
        goal_tip_quat: Desired end-effector orientation [x, y, z, w].
        start_pos: Default start position for resets.
        ee_velocity_estimator: Velocity estimator for safety monitoring.
        tf_buffer: TF2 buffer for coordinate transforms.
        listener: TF2 transform listener.
    """

    def __init__(self, node_name: str = 'franka_emika_robot_interface') -> None:
        """Initialize the Robot node.

        Args:
            node_name: Name for the ROS2 node.
        """
        super().__init__(node_name)

        # Publishers
        self._desired_ee_pose_pub = self.create_publisher(
            PoseStamped,
            "/cartesian_impedance_example_controller/equilibrium_pose",
            1,
        )
        self.gripper_command_pub = self.create_publisher(
            GripperCommandActionGoal,
            "/franka_gripper/gripper_action/goal",
            1,
        )
        self.stop_action_pub = self.create_publisher(
            StopActionGoal,
            "/franka_gripper/stop/goal",
            1,
        )
        self.move_action_pub = self.create_publisher(
            MoveActionGoal,
            "/franka_gripper/move/goal",
            1,
        )
        self.image_pub = self.create_publisher(Image, "processed_image", 1)

        # Subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            1,
        )
        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            "/cartesian_impedance_example_controller/measured_pose",
            self.ee_pose_callback,
            1,
        )
        self.qpos_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            1,
        )

        # Services
        self.reset_service = self.create_service(
            Empty,
            "reset_controller",
            self.reset_service_cb,
        )
        self.start_service = self.create_service(
            SetBool,
            "start_controller",
            self.start_service_cb,
        )

        # Robot state
        self._action_scale = 0.005  # Scaling factor for actions
        self.current_tip_pos: Optional[npt.NDArray[np.float64]] = None
        self.joint_state: Optional[npt.NDArray[np.float64]] = None
        self.latest_image: Optional[npt.NDArray[np.float32]] = None
        self.last_image_time: Optional[rclpy.time.Time] = None
        self.last_tip_pos_time: Optional[rclpy.time.Time] = None
        self.last_joint_state_time: Optional[rclpy.time.Time] = None
        self.last_cube_time: Optional[rclpy.time.Time] = None

        # Robot configuration
        self.goal_tip_quat = np.array([-1.0, 0.0, 0.0, 0.0])  # Downward facing
        self.start_pos = np.array([6.6105e-1, -0.05, 1.7906836e-01])

        # Velocity estimation for safety
        self.ee_velocity_estimator = LinearVelocityEstimator()

        # TF2 for ArUco tracking
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._running = False

    def image_callback(self, msg: Image) -> None:
        """Process incoming camera images.

        Converts BGR images to grayscale, preprocesses (crop, resize, normalize),
        and publishes the processed image.

        Args:
            msg: ROS Image message containing the camera frame.
        """
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            grayscale = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            image = _preprocess_image(grayscale)
            self.latest_image = image[..., None]  # Add channel dimension
            self.last_image_time = msg.header.stamp

            # Publish processed image for visualization
            output_msg = self.bridge.cv2_to_imgmsg(
                (image * 255).astype(np.uint8), encoding="mono8"
            )
            output_msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def ee_pose_callback(self, msg: PoseStamped) -> None:
        """Update end-effector pose from controller feedback.

        Args:
            msg: PoseStamped message with current end-effector pose.
        """
        pos = msg.pose.position
        self.current_tip_pos = np.array([pos.x, pos.y, pos.z])

        # Update internal orientation
        ori = msg.pose.orientation
        self.current_tip_quat = np.array([ori.x, ori.y, ori.z, ori.w])

        # Update velocity estimator for safety monitoring
        self.ee_velocity_estimator.add_measurement(
            self.current_tip_pos, msg.header.stamp
        )
        self.last_tip_pos_time = msg.header.stamp

    def joint_state_callback(self, msg: JointState) -> None:
        """Update joint positions from robot state.

        Args:
            msg: JointState message with joint positions.
        """
        self.joint_state = np.array(msg.position)
        self.last_joint_state_time = msg.header.stamp

    def get_camera_image(self) -> Optional[npt.NDArray[np.float32]]:
        """Get the most recent preprocessed camera image.

        Returns:
            Preprocessed image array of shape (64, 64, 1) or None if no image received.
        """
        return self.latest_image

    def get_joint_state(self) -> Optional[npt.NDArray[np.float64]]:
        """Get current joint positions.

        Returns:
            Array of 9 joint positions (7 arm + 2 gripper) or None if unavailable.
        """
        return self.joint_state

    def get_cube_pos(self, frame: str = "panda_link0") -> Optional[npt.NDArray[np.float64]]:
        """Get cube position from ArUco marker tracking.

        Looks up the TF transform from the specified frame to the ArUco marker frame.

        Args:
            frame: Reference frame for the position (default: panda_link0).

        Returns:
            Cube position [x, y, z] in the reference frame, or None if transform fails.
        """
        try:
            transformed_pose = self.tf_buffer.lookup_transform(
                frame, "aruco_cube_frame", rclpy.time.Time()
            )
            pos = transformed_pose.transform.translation
            self.last_cube_time = transformed_pose.header.stamp
            return np.array([pos.x, pos.y, pos.z])
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as e:
            self.get_logger().error(f"Transform error in get_cube_pos: {e}")
            return None

    def get_cube_quat(self, frame: str = "panda_link0") -> Optional[npt.NDArray[np.float64]]:
        """Get cube orientation from ArUco marker tracking.

        Args:
            frame: Reference frame for the orientation (default: panda_link0).

        Returns:
            Cube orientation quaternion [x, y, z, w] or None if transform fails.
        """
        try:
            transformed_pose = self.tf_buffer.lookup_transform(
                frame, "aruco_cube_frame", rclpy.time.Time()
            )
            quat = transformed_pose.transform.rotation
            return np.array([quat.x, quat.y, quat.z, quat.w])
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as e:
            self.get_logger().error(f"Transform error in get_cube_quat: {e}")
            return None

    def start_service_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Service callback to start/stop the controller.

        Args:
            request: Service request with boolean data field.
            response: Service response to populate.

        Returns:
            Response with success flag and message.
        """
        self._running = request.data
        response.success = True
        response.message = "Started controller." if request.data else "Stopped controller."
        return response

    def reset_service_cb(
        self, request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        """Service callback to reset robot to start position.

        Opens the gripper and moves end-effector to the default start pose.

        Args:
            request: Empty service request.
            response: Empty service response.

        Returns:
            Empty response.
        """
        self.get_logger().info("Resetting robot...")
        self.open_gripper()

        # Create target pose message
        target_pose = PoseStamped()
        target_pose.header.frame_id = "panda_link0"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = float(self.start_pos[0])
        target_pose.pose.position.y = float(self.start_pos[1])
        target_pose.pose.position.z = float(self.start_pos[2])
        target_pose.pose.orientation.x = float(self.goal_tip_quat[0])
        target_pose.pose.orientation.y = float(self.goal_tip_quat[1])
        target_pose.pose.orientation.z = float(self.goal_tip_quat[2])
        target_pose.pose.orientation.w = float(self.goal_tip_quat[3])

        self._desired_ee_pose_pub.publish(target_pose)
        self._running = False
        return response

    def act(self, action: npt.NDArray[np.float64]) -> Optional[npt.NDArray[np.float64]]:
        """Execute an action on the robot.

        The action is a 4D vector: [dx, dy, dz, gripper]
        - dx, dy, dz: Velocity commands in range [-1, 1], scaled by action_scale
        - gripper: >= 0 to open, < 0 to close

        Args:
            action: Action vector of shape (4,).

        Returns:
            New end-effector position after applying the action, or None if robot not ready.
        """
        if not self.ok:
            self.get_logger().warn("Not ready yet. Cannot execute action.")
            return None

        # Apply position delta with scaling
        delta_pos = action[:3] * self._action_scale
        new_tip_pos = self.current_tip_pos + delta_pos

        # Clip to safe workspace bounds
        new_tip_pos[0] = np.clip(new_tip_pos[0], 0.25, 0.77)
        new_tip_pos[1] = np.clip(new_tip_pos[1], -0.32, 0.32)
        new_tip_pos[2] = np.clip(new_tip_pos[2], 0.02, 0.5)

        # Publish end-effector pose command
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(self.start_pos[0])  # Fixed X
        pose_msg.pose.position.y = float(new_tip_pos[1])
        pose_msg.pose.position.z = float(new_tip_pos[2])
        pose_msg.pose.orientation.x = float(self.goal_tip_quat[0])
        pose_msg.pose.orientation.y = float(self.goal_tip_quat[1])
        pose_msg.pose.orientation.z = float(self.goal_tip_quat[2])
        pose_msg.pose.orientation.w = float(self.goal_tip_quat[3])
        self._desired_ee_pose_pub.publish(pose_msg)

        # Handle gripper command
        fingers = self.joint_state[-2:].mean()  # Average of two finger joints
        if action[3] >= 0.0:
            # Open gripper
            self.get_logger().info("Opening gripper")
            goal = GripperCommandActionGoal()
            goal.header.stamp = self.get_clock().now().to_msg()

            # If gripper is nearly closed, stop it first before opening
            if fingers < 0.015:
                stop = StopActionGoal()
                stop.header.stamp = self.get_clock().now().to_msg()
                self.stop_action_pub.publish(stop)
                position = 0.04  # Open position
            else:
                position = np.clip(fingers + 0.01, 0.0, 0.0402)

            goal.goal.command.position = position
            goal.goal.command.max_effort = 0.0
            self.gripper_command_pub.publish(goal)
        else:
            # Close gripper
            self.get_logger().info("Closing Gripper")
            goal = GripperCommandActionGoal()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.goal.command.position = np.clip(fingers - 0.01, 0.0, 0.0402)
            goal.goal.command.max_effort = 50.0  # High effort for grasping
            self.gripper_command_pub.publish(goal)

        return new_tip_pos

    def get_end_effector_pos(self) -> Optional[npt.NDArray[np.float64]]:
        """Get current end-effector position.

        Returns:
            Position [x, y, z] or None if unavailable.
        """
        return self.current_tip_pos

    def open_gripper(self) -> None:
        """Fully open the gripper."""
        if not self.ok:
            self.get_logger().warn("Not ready yet. Cannot open gripper.")
            return

        move = MoveActionGoal()
        move.header.stamp = self.get_clock().now().to_msg()
        move.goal.width = 0.08  # Maximum width
        move.goal.speed = 10.0
        self.move_action_pub.publish(move)

    @property
    def fingers_open(self) -> bool:
        """Check if gripper fingers are open.

        Returns:
            True if fingers are open (> 25mm average width), False otherwise.
        """
        if self.joint_state is None:
            return False
        fingers = self.joint_state[-2:].mean()
        return fingers >= 0.025

    @property
    def ok(self) -> bool:
        """Check if robot is ready for operation.

        Verifies that all necessary sensor data is available.

        Returns:
            True if robot is ready, False otherwise.
        """
        ready = True
        if self.current_tip_pos is None:
            self.get_logger().warn("current_tip_pos is None")
            ready = False
        if self.latest_image is None:
            self.get_logger().warn("latest_image is None")
            ready = False
        if self.get_cube_pos() is None:
            self.get_logger().warn("get_cube_pos() returned None")
            ready = False
        if self.joint_state is None:
            self.get_logger().warn("joint_state is None")
            ready = False
        return ready

    @property
    def safe(self) -> bool:
        """Check if robot is in a safe state.

        Monitors position and velocity for safety violations.

        Returns:
            True if robot is safe, False if out of bounds or moving too fast.
        """
        pos = self.get_end_effector_pos()
        out_of_bounds = np.any(np.abs(pos - self.start_pos) > 0.3)
        if out_of_bounds:
            self.get_logger().warn(
                f"Robot out of bounds. Position is: {self.get_end_effector_pos()}"
            )

        velocity = self.ee_velocity_estimator.estimate_velocity()
        high_velocity = np.any(np.abs(velocity) > 0.5)
        if high_velocity:
            self.get_logger().warn(f"EE high velocity. Velocity is: {velocity}")

        return not out_of_bounds and not high_velocity


class LinearVelocityEstimator:
    """Estimates linear velocity using a sliding window of position measurements.

    Uses least-squares linear regression over recent position history to estimate
    velocity. This is more robust than simple finite differences.

    Attributes:
        window_size: Number of measurements to keep in the sliding window.
        positions: Deque of recent position vectors.
        timestamps: Deque of corresponding timestamps in seconds.
    """

    def __init__(self, window_size: int = 10) -> None:
        """Initialize the velocity estimator.

        Args:
            window_size: Number of samples to use for velocity estimation.
        """
        self.window_size = window_size
        self.positions: Deque[npt.NDArray[np.float64]] = deque(maxlen=window_size)
        self.timestamps: Deque[float] = deque(maxlen=window_size)

    def add_measurement(
        self, position: npt.NDArray[np.float64], timestamp: rclpy.time.Time
    ) -> None:
        """Add a new position measurement.

        Args:
            position: 3D position vector [x, y, z].
            timestamp: ROS2 timestamp of the measurement.
        """
        self.positions.append(np.array(position))
        # Convert ROS2 timestamp to seconds
        self.timestamps.append(float(timestamp.sec + timestamp.nanosec / 1e9))

    def estimate_velocity(self) -> Optional[npt.NDArray[np.float64]]:
        """Estimate current velocity using least-squares regression.

        Returns:
            Estimated velocity vector [vx, vy, vz] in m/s, or None if insufficient data.
        """
        if len(self.positions) < 2 or np.array(self.timestamps).std() < 1e-6:
            return None  # Not enough data or no time variation

        t = np.array(self.timestamps)
        p = np.vstack(self.positions)  # Shape: (N, 3)

        # Center time for numerical stability
        t_centered = t - t.mean()

        # Solve for velocity: p = v*t + b
        A = t_centered[:, np.newaxis]  # Shape: (N, 1)
        v_est, _, _, _ = np.linalg.lstsq(A, p - p.mean(axis=0), rcond=None)

        return v_est.flatten()  # Shape: (3,)


def _crop_and_resize(grayscale: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
    """Crop image to square and resize to 64x64.

    Args:
        grayscale: Input grayscale image.

    Returns:
        Square cropped and resized image of shape (64, 64).
    """
    height, width = grayscale.shape
    # Crop to square (assuming height < width)
    new_width = height
    crop_amount = (width - new_width) // 2
    cropped = grayscale[:, crop_amount : crop_amount + new_width]
    cropped = cv2.resize(cropped, (64, 64), interpolation=cv2.INTER_LINEAR)
    return cropped


def _preprocess_image(grayscale: npt.NDArray[np.uint8]) -> npt.NDArray[np.float32]:
    """Preprocess camera image for neural network input.

    Crops to square, resizes to 64x64, and normalizes to [0, 1].

    Args:
        grayscale: Input grayscale image (height, width).

    Returns:
        Preprocessed image of shape (64, 64) with values in [0, 1].
    """
    grayscale = _crop_and_resize(grayscale)
    # Normalize to [0, 1]
    normalized = grayscale.astype(np.float32) / 255.0
    return normalized
