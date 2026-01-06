#!/usr/bin/env python3
"""
Launch file for sensor fusion debugging
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('state_estimator')
    
    # Declare launch arguments
    state1_duration_arg = DeclareLaunchArgument(
        'state1_duration',
        default_value='10.0',
        description='Duration of state 1 in seconds'
    )
    
    transition_duration_arg = DeclareLaunchArgument(
        'transition_duration',
        default_value='5.0',
        description='Duration of transition in seconds'
    )
    
    loop_enabled_arg = DeclareLaunchArgument(
        'loop_enabled',
        default_value='false',
        description='Whether to loop the sequence'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'sensor_fusion_debug_config.yaml'),
        description='Path to config file'
    )
    
    # Debug publisher node
    debug_publisher = Node(
        package='state_estimator',
        executable='sensor_fusion_debug_publisher.py',
        name='sensor_fusion_debug_publisher',
        parameters=[
            LaunchConfiguration('config_file'),
            {'state1_duration': LaunchConfiguration('state1_duration')},
            {'transition_duration': LaunchConfiguration('transition_duration')},
            {'loop_enabled': LaunchConfiguration('loop_enabled')},
        ],
        output='screen'
    )
    
    return LaunchDescription([
        state1_duration_arg,
        transition_duration_arg,
        loop_enabled_arg,
        config_file_arg,
        debug_publisher,
    ])
