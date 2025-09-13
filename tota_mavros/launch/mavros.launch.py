#!/usr/bin/env python3
"""
ROS2 Launch file for TOTA MAVROS
Launches MAVROS with custom configuration for TOTA robot
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function to configure and launch MAVROS node"""
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # Load configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    mavlink = config.get('mavlink', {})
    mavros_config = config.get('mavros', {})
    
    # Build FCU URL from config
    fcu_url = f"udp://{mavlink.get('host', '127.0.0.1')}:{mavlink.get('input_port', 14551)}@:{mavlink.get('output_port', 14555)}"
    
    # Load plugin allowlist
    pkg_share = get_package_share_directory('tota_mavros')
    plugin_file = os.path.join(pkg_share, 'config', mavros_config.get('plugin_config', 'mavros_plugins.yaml'))
    
    plugin_allowlist = []
    if os.path.exists(plugin_file):
        with open(plugin_file, 'r') as f:
            plugin_config = yaml.safe_load(f)
            if not plugin_config.get('load_all_plugins', False):
                plugin_allowlist = plugin_config.get('plugin_allowlist', [])
    
    # Build MAVROS parameters
    mavros_params = {
        'fcu_url': fcu_url,
        'gcs_url': mavlink.get('gcs_url', ''),
        'target_system_id': mavlink.get('target_system_id', 20),
        'target_component_id': mavlink.get('target_component_id', 200),
        'fcu_protocol': mavlink.get('fcu_protocol', 'v2.0'),
    }
    
    if plugin_allowlist:
        mavros_params['plugin_allowlist'] = plugin_allowlist
    
    # Create MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output=mavros_config.get('log_output', 'screen'),
        respawn=False,
        parameters=[mavros_params],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
        on_exit=Shutdown(),
    )
    
    return [
        LogInfo(msg=f"Launching MAVROS with namespace: /{namespace}"),
        LogInfo(msg=f"FCU URL: {fcu_url}"),
        mavros_node
    ]


def generate_launch_description():
    """Generate launch description"""
    
    pkg_share = get_package_share_directory('tota_mavros')
    default_config = os.path.join(pkg_share, 'config', 'config.yaml')
    
    # Load default namespace from config
    default_namespace = 'tota1'
    if os.path.exists(default_config):
        with open(default_config, 'r') as f:
            default_namespace = yaml.safe_load(f).get('robot_name', 'tota1')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=default_namespace,
            description='Namespace for MAVROS node'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to configuration file'
        ),
        OpaqueFunction(function=launch_setup)
    ])