#!/usr/bin/env python3
"""Dummy camera image publisher for simulation.

This node publishes synthetic camera images to /camera/color/image_raw for
testing the pipeline without a physical RealSense camera. Useful for Gazebo
simulation or offline testing.
"""

import rospy
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


def main() -> None:
    """Publish dummy camera images at 15 Hz."""
    rospy.init_node("dummy_image_publisher")
    pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
    rate = rospy.Rate(15)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        try:
            cv_image = generate_dummy_image()
            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Exception in loop: {e}")
        rate.sleep()


if __name__ == "__main__":
    main()
