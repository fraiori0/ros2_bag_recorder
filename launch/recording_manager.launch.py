#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for the rosbag recording manager.
    """
    
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_bag_recorder'),
            'config',
            'default_config.yaml' # default config file, also to provide an example
        ]),
        description='Path to YAML config file (defaults to package default_config.yaml)'
    )
    
    # Create the recording manager node
    recording_node = Node(
        package='ros2_bag_recorder',
        executable='recording_node',
        name='recording_manager',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }]
    )
    
    return LaunchDescription([
        config_path_arg,
        recording_node,
    ])