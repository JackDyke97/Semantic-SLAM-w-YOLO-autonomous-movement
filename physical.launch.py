from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

"""
Physical robot launch file.
Launches YOLO detector, fusion node.

Run separately:
  - TurtleBot 4 bringup (on robot via SSH)
  - RealSense camera (if using external camera)
  - RTABMap
  - RViz
  
"""


def generate_launch_description():

    yolo_realsense_node = Node(
        package='semantic_mapping',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw'
        }],
        output='screen',
        condition=__import__(
            'launch.conditions', fromlist=['UnlessCondition']
        ).UnlessCondition(use_tb4_camera)
    )

    # Real fusion node
    fusion_node = Node(
        package='semantic_mapping',
        executable='fusion_node',
        name='fusion_node',
        output='screen'
    )

    return LaunchDescription([
        use_tb4_camera_arg,
        yolo_realsense_node,
        # Delay fusion by 5s to let camera and TF settle
        TimerAction(period=5.0, actions=[fusion_node]),
    ])
