#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import argparse
import sys

class ImageViewer(Node):
    """
    A simple ROS 2 node that subscribes to an image topic,
    converts it to an OpenCV image, and displays it in a window.
    """
    def __init__(self, topic_name):
        super().__init__('image_viewer')
        self.get_logger().info(f"Initializing image viewer for topic: {topic_name}")
        
        # --- Configuration ---
        self.bridge = CvBridge()
        self.topic_name = topic_name
        self.window_name = f"Image Viewer: {self.topic_name}"

        # --- Subscription ---
        # The subscription is created, and the callback function 'listener_callback' is set to be executed upon message arrival.
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10) # QoS profile depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        Callback function for the image subscriber.
        Converts the ROS Image message to an OpenCV image and displays it.
        """
        self.get_logger().debug(f'Received image on {self.topic_name}')
        try:
            # The 'passthrough' encoding is used to handle different image types,
            # especially for depth images which might be 16-bit.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert ROS Image message to OpenCV image: {e}")
            return

        # --- Image Processing and Display ---
        # If the image is single-channel (like a depth image), convert it for display.
        if len(cv_image.shape) == 2 or cv_image.shape[2] == 1:
            # For depth images (often 16UC1), we normalize and apply a colormap to make depth visible.
            if msg.encoding in ['16UC1', '32FC1']:
                 # Normalize the depth image to a 0-255 range to apply a colormap
                cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                cv_image_display = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
            else:
                # For other single-channel images, just convert to BGR.
                cv_image_display = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        else:
            # Assume it's a BGR or RGB image already.
            # The bridge usually converts to BGR by default if 'bgr8' is specified.
            # With 'passthrough', we might need to handle RGB->BGR conversion.
            if msg.encoding.lower() == 'rgb8':
                cv_image_display = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image_display = cv_image

        # Display the image
        cv2.imshow(self.window_name, cv_image_display)
        cv2.waitKey(1)

def main(args=None):
    # --- Argument Parsing ---
    # We use sys.argv directly because rclpy.init() can strip ROS arguments.
    parser = argparse.ArgumentParser(description="A simple ROS 2 image topic viewer.")
    parser.add_argument(
        'topic', 
        type=str, 
        help='The name of the image topic to subscribe to (e.g., /camera1/color/image_raw).'
    )
    
    # Isolate the topic argument from other ROS arguments
    parsed_args, remaining_args = parser.parse_known_args()

    # --- ROS 2 Initialization ---
    rclpy.init(args=remaining_args)
    
    image_viewer = ImageViewer(topic_name=parsed_args.topic)
    
    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        # --- Cleanup ---
        image_viewer.get_logger().info("Shutting down image viewer.")
        cv2.destroyAllWindows()
        image_viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

