#!/usr/bin/env python3
"""
ROS2 Launch file for TOTA MAVROS
Launches MAVROS with custom configuration for TOTA robot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    tota_mavros_pkg = get_package_share_directory('tota_mavros')
    default_mavros_config = os.path.join(tota_mavros_pkg, 'config', 'mavros_config.yaml')
    
    # Load default namespace from config
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tota1',
        description='Namespace for MAVROS node'
    )

    log_output_arg = DeclareLaunchArgument(
        'log_output',
        default_value='screen',
        description='Log output'
    )

    # Create MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=LaunchConfiguration('namespace'),
        output=LaunchConfiguration('log_output'),
        respawn=False,
        parameters=[default_mavros_config],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
        on_exit=Shutdown(),
    )
    
    return LaunchDescription([
        namespace_arg,
        log_output_arg,
        mavros_node
    ])