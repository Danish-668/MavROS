#!/usr/bin/env python3
"""
Launch script for MAVROS with custom namespace from config
"""
import os
import sys
import yaml
import subprocess
import argparse

def load_config(config_path="config.yaml"):
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def launch_mavros(config):
    """Launch MAVROS with configuration"""
    robot_name = config.get('robot_name', 'tota1')
    mavlink_config = config.get('mavlink', {})
    input_port = mavlink_config.get('input_port', 14550)
    output_port = mavlink_config.get('output_port', 14555)
    mavros_config = config.get('mavros', {})
    fcu_protocol = mavros_config.get('fcu_protocol', 'v2.0')
    
    # Build the command - use local custom MAVROS binary
    cmd = [
        './install/mavros/lib/mavros/mavros_node',
        '--ros-args',
        '-r', f'__ns:=/{robot_name}',
        '-p', f'fcu_url:=udp://127.0.0.1:{input_port}@:{output_port}',
        '-p', f'fcu_protocol:={fcu_protocol}',
        '-p', 'tgt_system:=20',
        '-p', 'tgt_component:=200'
    ]
    
    # Load plugin configuration from separate file if specified
    plugin_config_file = mavros_config.get('plugin_config', None)
    plugin_allowlist = []
    
    if plugin_config_file:
        try:
            with open(plugin_config_file, 'r') as f:
                plugin_config = yaml.safe_load(f)
                # Check if we should load all plugins
                if plugin_config.get('load_all_plugins', False):
                    print("Loading all available plugins (load_all_plugins: true)")
                else:
                    plugin_allowlist = plugin_config.get('plugin_allowlist', [])
        except FileNotFoundError:
            print(f"Warning: Plugin config file '{plugin_config_file}' not found, loading all plugins")
    else:
        # Fallback to inline config if no separate file specified
        plugin_allowlist = mavros_config.get('plugin_allowlist', [])
    
    # Add plugin allowlist if specified
    if plugin_allowlist:
        plugins_str = ','.join(plugin_allowlist)
        cmd.extend(['-p', f'plugin_allowlist:=[{plugins_str}]'])
        print(f"Loading plugins from {plugin_config_file or 'config'}: {', '.join(plugin_allowlist)}")
    elif not plugin_config_file:
        print("Loading all available plugins (no allowlist specified)")
    
    print(f"Launching MAVROS with namespace: /{robot_name}")
    print(f"FCU URL: udp://127.0.0.1:{input_port}@:{output_port}")
    print(f"Protocol: {fcu_protocol}")
    print("-" * 50)
    
    # Launch MAVROS
    try:
        # Source the workspace setup and run
        source_cmd = f"source ./install/setup.bash && {' '.join(cmd)}"
        subprocess.run(source_cmd, shell=True, executable='/bin/bash')
    except KeyboardInterrupt:
        print("\nShutting down MAVROS...")

def main():
    parser = argparse.ArgumentParser(description='Launch MAVROS with custom configuration')
    parser.add_argument('--config', default='config.yaml', help='Path to config file')
    args = parser.parse_args()
    
    # Check if config file exists
    if not os.path.exists(args.config):
        print(f"Error: Config file '{args.config}' not found!")
        sys.exit(1)
    
    # Load config and launch
    config = load_config(args.config)
    launch_mavros(config)

if __name__ == "__main__":
    main()