#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import argparse
import time
from datetime import datetime

class MultiCameraSubscriber(Node):
    """
    This node subscribes to RGB and Depth topics from specified cameras,
    displays them, and saves them as video files.
    """
    def __init__(self, recording_length_sec, cameras_to_use, front_cam_num, back_cam_num):
        super().__init__('multi_camera_recorder')
        self.get_logger().info(f"Initializing Multi-Camera Recorder for: {cameras_to_use}")

        # --- Configuration ---
        self.recording_length_sec = recording_length_sec
        self.cameras_to_use = cameras_to_use
        self.bridge = CvBridge()

        # --- Dynamic Topic and Data Structure Setup ---
        self.subscriptions_info = {}
        if 'front' in self.cameras_to_use:
            self.get_logger().info(f"Mapping 'front' to /camera{front_cam_num}")
            self.subscriptions_info['front_rgb'] = f'/camera{front_cam_num}/color/image_raw'
            self.subscriptions_info['front_depth'] = f'/camera{front_cam_num}/aligned_depth_to_color/image_raw'
        if 'back' in self.cameras_to_use:
            self.get_logger().info(f"Mapping 'back' to /camera{back_cam_num}")
            self.subscriptions_info['back_rgb'] = f'/camera{back_cam_num}/color/image_raw'
            self.subscriptions_info['back_depth'] = f'/camera{back_cam_num}/aligned_depth_to_color/image_raw'

        self.video_writers = {key: None for key in self.subscriptions_info}
        self.latest_frames = {key: None for key in self.subscriptions_info}
        self.first_frame_received = {key: False for key in self.subscriptions_info}
        self.frame_sizes = {}
        self.log_counter = 0

        # --- Directory Setup ---
        self.episode_path = self._setup_directories()
        if self.episode_path is None:
            self.get_logger().error("Failed to create directories. Shutting down.")
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info(f"Recording data to: {self.episode_path}")

        # --- Subscriptions ---
        self._subscription_handlers = {}
        for key, topic in self.subscriptions_info.items():
            # Corrected the f-string to remove the leading underscore
            callback_func = getattr(self, f"{key.split('_')[0]}_{key.split('_')[1]}_callback")
            self._subscription_handlers[key] = self.create_subscription(
                Image, topic, callback_func, 10)
            self.get_logger().info(f"Subscribed to {topic}")

        # --- Timers ---
        self.display_timer = self.create_timer(1.0 / 30.0, self.display_streams)
        self.stop_timer = self.create_timer(self.recording_length_sec, self.stop_recording)
        self.start_time = time.time()

        self.get_logger().info(f"Node initialized. Recording for {self.recording_length_sec} seconds.")

    def _setup_directories(self):
        """Creates the directory structure for saving videos."""
        try:
            base_dir = "Recorded_Videos"
            os.makedirs(base_dir, exist_ok=True)
            existing_episodes = [d for d in os.listdir(base_dir) if d.startswith("Episode_")]
            next_episode_num = 1
            if existing_episodes:
                episode_numbers = [int(e.split('_')[1]) for e in existing_episodes if e.split('_')[1].isdigit()]
                if episode_numbers:
                    last_episode_num = max(episode_numbers)
                    next_episode_num = last_episode_num + 1
            episode_path = os.path.join(base_dir, f"Episode_{next_episode_num}")

            # Only create directories for the cameras being used
            self.paths = {}
            if 'front' in self.cameras_to_use:
                self.paths['front_rgb'] = os.path.join(episode_path, "front_cam", "RGB")
                self.paths['front_depth'] = os.path.join(episode_path, "front_cam", "Depth")
            if 'back' in self.cameras_to_use:
                self.paths['back_rgb'] = os.path.join(episode_path, "back_cam", "RGB")
                self.paths['back_depth'] = os.path.join(episode_path, "back_cam", "Depth")

            for path in self.paths.values():
                os.makedirs(path, exist_ok=True)
            return episode_path
        except OSError as e:
            self.get_logger().error(f"Error creating directories: {e}")
            return None

    def _initialize_video_writer(self, key, frame):
        """Initializes a VideoWriter for a given stream."""
        if self.video_writers.get(key) is None:
            height, width, *_ = frame.shape
            self.frame_sizes[key] = (width, height)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.paths[key], f"{key}_{timestamp}.mp4")
            self.video_writers[key] = cv2.VideoWriter(filename, fourcc, 30.0, (width, height))
            self.get_logger().info(f"Initialized video writer for {key} at {width}x{height}. Saving to {filename}")

    def _process_frame(self, key, msg):
        """Generic frame processing for all callbacks."""
        try:
            if not self.first_frame_received.get(key, True):
                self.get_logger().info(f"First frame received for '{key}'.")
                self.first_frame_received[key] = True

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_frames[key] = cv_image
            if self.video_writers.get(key) is None:
                self._initialize_video_writer(key, cv_image)
            if self.video_writers.get(key) is not None:
                self.video_writers[key].write(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error processing {key} frame: {e}")

    def _process_depth_frame(self, key, msg):
        """Specific frame processing for depth images."""
        try:
            if not self.first_frame_received.get(key, True):
                self.get_logger().info(f"First frame received for '{key}'.")
                self.first_frame_received[key] = True

            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            self.latest_frames[key] = depth_colormap
            if self.video_writers.get(key) is None:
                self._initialize_video_writer(key, depth_colormap)
            if self.video_writers.get(key) is not None:
                self.video_writers[key].write(depth_colormap)
        except Exception as e:
            self.get_logger().error(f"Error processing {key} (depth) frame: {e}")

    # --- Callbacks ---
    def front_rgb_callback(self, msg): self._process_frame('front_rgb', msg)
    def front_depth_callback(self, msg): self._process_depth_frame('front_depth', msg)
    def back_rgb_callback(self, msg): self._process_frame('back_rgb', msg)
    def back_depth_callback(self, msg): self._process_depth_frame('back_depth', msg)

    def display_streams(self):
        """Combines and displays all camera streams in a single window."""
        if not all(self.first_frame_received.values()):
            self.log_counter += 1
            if self.log_counter % 90 == 0:
                missing_streams = [key for key, received in self.first_frame_received.items() if not received]
                self.get_logger().warn(f"Window not open. Waiting for first frame from: {missing_streams}")
            return

        display_size = (640, 480)
        
        # --- Dynamically build the display grid based on active cameras ---
        displays = {}
        for key, frame in self.latest_frames.items():
            try:
                if frame is not None:
                    displays[key] = cv2.resize(frame, display_size)
                    label = key.replace('_', ' ').title()
                    cv2.putText(displays[key], label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            except (cv2.error, TypeError) as e:
                self.get_logger().warn(f"Could not resize {key} frame for display: {e}")
                return

        combined_view = None
        if 'front' in self.cameras_to_use and 'back' in self.cameras_to_use:
            # 2x2 grid for both cameras
            top_row = np.hstack((displays.get('front_rgb'), displays.get('front_depth')))
            bottom_row = np.hstack((displays.get('back_rgb'), displays.get('back_depth')))
            combined_view = np.vstack((top_row, bottom_row))
        elif 'front' in self.cameras_to_use:
            # 1x2 grid for front camera only
            front_rgb = displays.get('front_rgb')
            front_depth = displays.get('front_depth')
            if front_rgb is not None and front_depth is not None:
                combined_view = np.hstack((front_rgb, front_depth))
        elif 'back' in self.cameras_to_use:
            # 1x2 grid for back camera only
            back_rgb = displays.get('back_rgb')
            back_depth = displays.get('back_depth')
            if back_rgb is not None and back_depth is not None:
                combined_view = np.hstack((back_rgb, back_depth))

        if combined_view is not None:
            elapsed_time = time.time() - self.start_time
            remaining_time = max(0, self.recording_length_sec - elapsed_time)
            status_text = f"RECORDING... Time left: {remaining_time:.1f}s"
            cv2.putText(combined_view, status_text, (10, combined_view.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow("Multi-Camera Feeds", combined_view)
            cv2.waitKey(1)

    def stop_recording(self):
        """Stops the recording, releases resources, and shuts down the node."""
        self.get_logger().info("Recording time elapsed or stopped manually. Stopping...")
        for key, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
                self.get_logger().info(f"Released video writer for {key}.")
        cv2.destroyAllWindows()
        self.get_logger().info("Shutting down ROS 2 node.")
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    parser = argparse.ArgumentParser(description="ROS 2 Multi-Camera Recorder")
    parser.add_argument(
        '-l', '--length', type=int, default=20,
        help="Maximum recording length in seconds (default: 20)")
    parser.add_argument(
        '-c', '--cameras', nargs='+', choices=['front', 'back'],
        default=['front', 'back'],
        help="Which cameras to use (default: front back)")
    parser.add_argument(
        '--front-cam-num', type=int, default=2,
        help="The camera number/namespace for the 'front' camera (default: 1)")
    parser.add_argument(
        '--back-cam-num', type=int, default=1,
        help="The camera number/namespace for the 'back' camera (default: 2)")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    multi_camera_subscriber = None
    try:
        multi_camera_subscriber = MultiCameraSubscriber(
            recording_length_sec=cli_args.length,
            cameras_to_use=cli_args.cameras,
            front_cam_num=cli_args.front_cam_num,
            back_cam_num=cli_args.back_cam_num
        )
        rclpy.spin(multi_camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if multi_camera_subscriber is not None and multi_camera_subscriber.context.ok():
            multi_camera_subscriber.stop_recording()

if __name__ == '__main__':
    main()

