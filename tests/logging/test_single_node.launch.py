#!/usr/bin/env python3
"""
Single node launch file for logging tests.

This launch file tests:
1. Launching a single Go node with ROS 2 launch
2. Passing log level via launch arguments
3. Loading parameters from YAML file
4. Verifying logs go to correct location
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the directory of this launch file (ensure absolute path)
    launch_file = Path(__file__).resolve()
    launch_dir = launch_file.parent
    config_dir = launch_dir / 'config'

    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR, FATAL)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str((config_dir / 'params.yaml').resolve()),
        description='Path to parameter file'
    )

    use_realtime_logging_arg = DeclareLaunchArgument(
        'use_realtime_logging',
        default_value='false',
        description='Enable realtime (unbuffered) logging'
    )

    # Get launch configurations
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    use_realtime_logging = LaunchConfiguration('use_realtime_logging')

    # Determine the executable path (absolute path to rclgo root)
    rclgo_root = launch_file.parent.parent.parent
    param_demo_exe = rclgo_root / 'examples' / 'param_demo' / 'param_demo'

    if not param_demo_exe.exists():
        raise RuntimeError(f"Executable not found: {param_demo_exe}\n"
                         f"Please build it first:\n"
                         f"  cd {rclgo_root}/examples/param_demo\n"
                         f"  source /opt/ros/humble/setup.bash\n"
                         f"  source ../../cgo-flags.env\n"
                         f"  go build -o param_demo")

    # Build the command with arguments
    cmd = [
        str(param_demo_exe),
        '--ros-args',
        '--params-file', params_file,
        '--log-level', log_level,
    ]

    # Create the process
    param_demo_process = ExecuteProcess(
        cmd=cmd,
        name='param_demo',
        output='both',  # Log to both screen and log file
        shell=False,
    )

    return LaunchDescription([
        log_level_arg,
        params_file_arg,
        use_realtime_logging_arg,
        LogInfo(msg=['Launching param_demo with log_level=', log_level]),
        LogInfo(msg=['Using params_file=', params_file]),
        LogInfo(msg=['Executable: ', str(param_demo_exe)]),
        param_demo_process,
    ])
