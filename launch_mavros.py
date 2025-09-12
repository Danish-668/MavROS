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
    fcu_protocol = config.get('mavros', {}).get('fcu_protocol', 'v2.0')
    
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
    
    print(f"Launching MAVROS with namespace: /{robot_name}")
    print(f"FCU URL: udp://127.0.0.1:{input_port}@:{output_port}")
    print(f"Protocol: {fcu_protocol}")
    print("-" * 50)
    
    # Launch MAVROS with sourced environment
    try:
        env = os.environ.copy()
        # Source the local workspace setup
        source_cmd = f'source install/setup.bash && {" ".join(cmd)}'
        subprocess.run(['bash', '-c', source_cmd], env=env)
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