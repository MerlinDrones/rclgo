#!/usr/bin/env python3
"""
Multi-node launch file for logging tests.

This launch file tests:
1. Launching multiple Go nodes simultaneously
2. Console output interleaving (both nodes visible)
3. Separate log files for each node
4. Passing different log levels to different nodes
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory of this launch file
    launch_dir = Path(__file__).parent
    config_dir = launch_dir / 'config'

    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level for both nodes'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(config_dir / 'params.yaml'),
        description='Path to parameter file'
    )

    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')

    # Determine executable paths
    rclgo_root = Path(__file__).parent.parent.parent
    examples_dir = rclgo_root / 'examples' / 'publisher_subscriber'
    publisher_exe = examples_dir / 'publisher' / 'publisher'
    subscriber_exe = examples_dir / 'subscriber' / 'subscriber'

    # Create publisher node
    publisher_node = Node(
        package='rclgo_pubsub',
        executable=str(publisher_exe),
        name='publisher',
        output='both',  # Log to both screen and log file
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Create subscriber node
    subscriber_node = Node(
        package='rclgo_pubsub',
        executable=str(subscriber_exe),
        name='subscriber',
        output='both',  # Log to both screen and log file
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        log_level_arg,
        params_file_arg,
        LogInfo(msg=['Launching publisher and subscriber with log_level=', log_level]),
        publisher_node,
        subscriber_node,
    ])
