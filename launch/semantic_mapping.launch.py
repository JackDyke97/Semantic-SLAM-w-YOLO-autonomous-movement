from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='sim or real'
    )

    mode = LaunchConfiguration('mode')

    yolo_node = Node(
        package='semantic_mapping',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw'
        }],
        output='screen'
    )

    depth_node = Node(
        package='semantic_mapping',
        executable='depth_from_lidar',
        name='depth_from_lidar',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    sim_fusion_node = Node(
        package='semantic_mapping',
        executable='sim_fusion_node',
        name='sim_fusion_node',
        output='screen'
    )

    explorer_node = Node(
        package='semantic_mapping',
        executable='random_explorer',
        name='random_explorer',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        depth_node,
        # Delay YOLO and fusion by 5 seconds to let depth initialise
        TimerAction(period=5.0, actions=[yolo_node]),
        TimerAction(period=5.0, actions=[sim_fusion_node]),
        TimerAction(period=8.0, actions=[explorer_node]),
    ])
