#!/usr/bin/env python3
"""
ROS2 Launch file for TOTA MAVROS
Launches MAVROS with custom configuration for TOTA robot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(config_path):
    """Load YAML configuration file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    """Setup function to configure and launch MAVROS node"""
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    plugin_config_file = LaunchConfiguration('plugin_config_file').perform(context)
    fcu_url = LaunchConfiguration('fcu_url').perform(context)
    gcs_url = LaunchConfiguration('gcs_url').perform(context)
    tgt_system = LaunchConfiguration('tgt_system').perform(context)
    tgt_component = LaunchConfiguration('tgt_component').perform(context)
    log_output = LaunchConfiguration('log_output').perform(context)
    fcu_protocol = LaunchConfiguration('fcu_protocol').perform(context)
    respawn_mavros = LaunchConfiguration('respawn_mavros').perform(context)
    
    # Get package share directory
    pkg_share = get_package_share_directory('tota_mavros')
    mavros_share = get_package_share_directory('mavros')
    
    # Load main configuration
    config = {}
    if config_file and os.path.exists(config_file):
        config = load_yaml(config_file)
    
    # Load plugin configuration
    plugin_allowlist = []
    if plugin_config_file and os.path.exists(plugin_config_file):
        plugin_config = load_yaml(plugin_config_file)
        if not plugin_config.get('load_all_plugins', False):
            plugin_allowlist = plugin_config.get('plugin_allowlist', [])
    
    # Build parameters for MAVROS node
    mavros_params = {
        'fcu_url': fcu_url,
        'gcs_url': gcs_url,
        'target_system_id': int(tgt_system),
        'target_component_id': int(tgt_component),
        'fcu_protocol': fcu_protocol,
    }
    
    # Add plugin allowlist if specified
    if plugin_allowlist:
        mavros_params['plugin_allowlist'] = plugin_allowlist
        LogInfo(msg=f"Loading plugins: {', '.join(plugin_allowlist)}")
    
    # Create MAVROS node
    # Note: We don't set a node name to preserve the plugin-based topic structure
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output=log_output,
        respawn=respawn_mavros.lower() == 'true',
        respawn_delay=1,
        parameters=[mavros_params],
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    return [
        LogInfo(msg=f"Launching MAVROS with namespace: /{namespace}"),
        LogInfo(msg=f"FCU URL: {fcu_url}"),
        LogInfo(msg=f"GCS URL: {gcs_url}"),
        LogInfo(msg=f"Target: {tgt_system}.{tgt_component}"),
        LogInfo(msg=f"FCU Protocol: {fcu_protocol}"),
        mavros_node
    ]


def generate_launch_description():
    """Generate launch description"""
    
    # Get package directories
    pkg_share = get_package_share_directory('tota_mavros')
    default_config = os.path.join(pkg_share, 'config', 'config.yaml')
    default_plugin_config = os.path.join(pkg_share, 'config', 'mavros_plugins.yaml')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tota1',
        description='Namespace for MAVROS node'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to main configuration file'
    )
    
    plugin_config_file_arg = DeclareLaunchArgument(
        'plugin_config_file',
        default_value=default_plugin_config,
        description='Path to plugin configuration file'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14551@:14555',
        description='FCU connection URL'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='GCS connection URL (optional)'
    )
    
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='20',
        description='MAVLink target system ID'
    )
    
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='200',
        description='MAVLink target component ID'
    )
    
    log_output_arg = DeclareLaunchArgument(
        'log_output',
        default_value='screen',
        description='Log output destination (screen or log)'
    )
    
    fcu_protocol_arg = DeclareLaunchArgument(
        'fcu_protocol',
        default_value='v2.0',
        description='MAVLink protocol version'
    )
    
    respawn_mavros_arg = DeclareLaunchArgument(
        'respawn_mavros',
        default_value='false',
        description='Automatically respawn MAVROS if it crashes'
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        config_file_arg,
        plugin_config_file_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        log_output_arg,
        fcu_protocol_arg,
        respawn_mavros_arg,
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])