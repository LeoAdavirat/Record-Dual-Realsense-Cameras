# In your ROS 2 launch file (e.g., my_dual_camera_launch.py)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            namespace='camera1', # Namespace for first camera
            executable='realsense2_camera_node',
            name='realsense2_camera1',
            parameters=[{
                'serial_no': '_336222070126',
                'enable_color': True,
                'enable_depth': True,
                'enable_infra': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'align_depth.enable': True,
                'enable_pointcloud': True, 
                'rgb_camera.color_profile': '640x480x60',
                # 960x540x60 or 1920x1080x30
                'depth_module.depth_profile': '848x480x60',
                # 848x480x90 or 1280x720x30
                'json_file_path': '',
                'initial_reset': True,
                'depth_module.exposure': 2858.0,
                'depth_module.laser_power': 90.0,
            }],
            output='screen'
        ),
        Node(
            package='realsense2_camera',
            namespace='camera2', # Namespace for first camera
            executable='realsense2_camera_node',
            name='realsense2_camera2',
            parameters=[{
                'serial_no': '_832112070255',
                'enable_color': True,
                'enable_depth': True,
                'enable_infra': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'align_depth.enable': True,
                'enable_pointcloud': True, 
                'rgb_camera.color_profile': '640x480x60',
                # 960x540x60 or 1920x1080x30
                'depth_module.depth_profile': '848x480x60',
                # 848x480x90 or 1280x720x30
                'json_file_path': '',
                'initial_reset': True,
                'depth_module.exposure': 2858.0,
                'depth_module.laser_power': 90.0,
            }],
            output='screen'
        )
    ])
