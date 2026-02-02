#!/usr/bin/env python3
"""Dummy camera image publisher for simulation (ROS2).

This node publishes synthetic camera images to /camera/color/image_raw for
testing the pipeline without a physical RealSense camera. Useful for Gazebo
simulation or offline testing.
"""

from typing import Optional, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import numpy.typing as npt
import cv2


def generate_dummy_image() -> npt.NDArray[np.uint8]:
    """Generate a synthetic camera image with geometric shapes.

    Creates a 64x64 BGR image with a white rectangle and red square for
    testing vision processing pipelines.

    Returns:
        64x64x3 BGR image as uint8 array.
    """
    # Create a black background
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    # Draw a simple white rectangle to simulate a character or object
    cv2.rectangle(img, (0, 60), (320, 120), (255, 255, 255), -1)
    # Add a small red square below it
    cv2.rectangle(img, (150, 140), (170, 160), (0, 0, 255), -1)
    img_resized = cv2.resize(img, (64, 64), interpolation=cv2.INTER_AREA)
    return img_resized


class DummyImagePublisher(Node):
    """ROS2 node that publishes synthetic camera images at 15 Hz.

    Attributes:
        pub: Publisher for Image messages.
        bridge: CvBridge for OpenCV-ROS conversion.
        timer: Timer for periodic publishing.
    """

    def __init__(self) -> None:
        """Initialize the dummy image publisher node."""
        super().__init__('dummy_image_publisher')
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()
        # Create timer at 15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)

    def timer_callback(self) -> None:
        """Timer callback to generate and publish a dummy image."""
        try:
            cv_image = generate_dummy_image()
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Exception in loop: {e}")


def main(args: Optional[List[str]] = None) -> None:
    """Run the dummy image publisher node.

    Args:
        args: Optional command line arguments for rclpy.init().
    """
    rclpy.init(args=args)
    node = DummyImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
