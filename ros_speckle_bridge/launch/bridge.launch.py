#!/usr/bin/env python3
"""
Launch file for Speckle Bridge Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for bridge node"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('ros_speckle_bridge')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'params.yaml'),
        description='Path to configuration YAML file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    
    # Create bridge node
    bridge_node = Node(
        package='ros_speckle_bridge',
        executable='bridge_node',
        name='speckle_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'default.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        config_file_arg,
        rviz_arg,
        bridge_node,
        rviz_node,
    ])
