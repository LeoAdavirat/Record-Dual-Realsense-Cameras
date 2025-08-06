# Dual RealSense D435 Data Capture Toolkit for ROS 2

This toolkit provides a set of scripts to launch, view, and record data from one or two Intel RealSense D435 cameras within a ROS 2 environment.

## Overview

The system is composed of four main components:
1.  A hardware configuration file to identify the cameras.
2.  A ROS 2 launch file to start the camera nodes.
3.  A flexible, multi-camera recording script to save synchronized video streams.
4.  A simple viewing script to debug and inspect individual camera topics.

The intended workflow is to configure the cameras, launch them via ROS 2, and then use the standalone Python scripts to view or record the data streams.

---

## File Descriptions

Here is a breakdown of each file in this toolkit:

### `d435_rs_enumerate.txt`
* **Purpose**: This is a hardware configuration file that contains the unique serial numbers and other hardware information for your RealSense cameras.
* **Usage**: You should copy the serial number for each camera from this file and paste it into the `launch_both_cams_with_pointcloud.py` file. This ensures that ROS assigns `camera1` and `camera2` to the correct physical device every time, regardless of which USB port they are plugged into.

### `launch_both_cams_with_pointcloud.py`
* **Purpose**: This is the main ROS 2 launch file. It starts the `realsense2_camera_node` for each camera, placing them in unique namespaces (`/camera1` and `/camera2`) to prevent topic name collisions. It is pre-configured to enable the color, depth, and infrared streams, and to publish an aligned depth-to-color point cloud.
* **How to Run**: This is the only script that is run using `ros2 launch`.
    ```bash
    ros2 launch launch_both_cams_with_pointcloud.py
    ```

### `multicam_recorder.py`
* **Purpose**: A powerful Python script that acts as a ROS 2 node to subscribe to the camera topics and record them into organized video files. It can record from one or both cameras simultaneously and provides a combined live view of the active streams.
* **How to Run**: This script is run directly with `python3`. It must be run *after* the launch file is active.
    ```bash
    python3 multicam_recorder.py [OPTIONS]
    ```
* **Command-Line Options**:
    * `-l, --length SECONDS`: Sets the recording duration in seconds. The script will automatically stop after this time.
        * *Default*: `20`
        * *Example*: `python3 multicam_recorder.py --length 60`
    * `-c, --cameras [front] [back]`: Specifies which camera(s) to record.
        * *Default*: `front back` (records both)
        * *Example (front only)*: `python3 multicam_recorder.py --cameras front`
    * `--front-cam-num NUMBER`: Maps the logical "front" camera to a specific ROS namespace number.
        * *Default*: `1` (maps "front" to `/camera1`)
        * *Example*: `python3 multicam_recorder.py --front-cam-num 2`
    * `--back-cam-num NUMBER`: Maps the logical "back" camera to a specific ROS namespace number.
        * *Default*: `2` (maps "back" to `/camera2`)
        * *Example (swap cameras)*: `python3 multicam_recorder.py --front-cam-num 2 --back-cam-num 1`

### `show_stream.py` (ros_image_viewer.py)
* **Purpose**: A simple diagnostic tool for viewing a single ROS 2 image topic in an OpenCV window. This is extremely useful for verifying that a specific camera stream is being published correctly before you start a recording session.
* **How to Run**: This script is run directly with `python3` and requires a topic name as an argument.
    ```bash
    python3 show_stream.py <topic_name>
    ```
* **Example**:
    ```bash
    # View the color stream from the first camera
    python3 show_stream.py /camera1/color/image_raw

    # View the depth stream from the second camera
    python3 show_stream.py /camera2/aligned_depth_to_color/image_raw
    ```

---

## Standard Workflow

1.  **Hardware Setup**: Connect your RealSense camera(s) to the computer.
2.  **(One-Time) Configuration**: Run the `rs-enumerate-devices` tool to get your camera serial numbers and save them in `d435_rs_enumerate.txt`. Copy these serials into the `launch_both_cams_with_pointcloud.py` launch file.
3.  **Launch Cameras**: Open a terminal, source your ROS 2 workspace, and run the launch file:
    ```bash
    ros2 launch launch_both_cams_with_pointcloud.py
    ```
4.  **(Optional) Verify Streams**: Open a second terminal and use `show_stream.py` to check that the topics are active and publishing valid data.
    ```bash
    python3 show_stream.py /camera2/realsense2_camera2/color/image_raw
    ```
5.  **Record Data**: In the second terminal, run the recorder script with your desired options.
    ```bash
    # Record both cameras for 30 seconds
    python3 multicam_recorder.py --length 30

    # Record only the front camera indefinitely until you press Ctrl+C
    python3 multicam_recorder.py --cameras front --length 9999
    ```
6.  **Find Output**: Your videos will be saved in a new directory: `Recorded_Videos/Episode_X/`, where `X` increments with each new recording.


