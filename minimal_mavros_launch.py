#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_node',
            namespace='/uas1',
            parameters=[{
                'fcu_url': 'udp://@:14555',
                'gcs_url': '',
                'system_id': 1,
                'component_id': 191,
                'target_system_id': 1,
                'target_component_id': 1,
            }],
            output='screen'
        )
    ])