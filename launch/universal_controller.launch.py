#!/usr/bin/env python3
"""
Universal Controller Framework Launch File
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('universal_controller')
    config_file = os.path.join(pkg_dir, 'config', 'controller_params.yaml')

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the configuration file'
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='infantry',
            description='Robot type: infantry, sentry'
        ),

        # RC Interpreter Node (独立节点)
        Node(
            package='universal_controller',
            executable='rc_interpreter_node',
            name='rc_interpreter',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),

        # Main Hub Node
        Node(
            package='universal_controller',
            executable='universal_controller_node',
            name='universal_controller_hub',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        ),
    ])
