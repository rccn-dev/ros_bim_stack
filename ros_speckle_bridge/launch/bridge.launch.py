#!/usr/bin/env python3
"""
Launch file for Speckle Bridge Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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
    
    # Create bridge node
    bridge_node = Node(
        package='ros_speckle_bridge',
        executable='bridge_node',
        name='speckle_bridge',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        bridge_node,
    ])
