#!/usr/bin/env python3
"""
Universal Controller Framework Launch File

启动单一节点：
1. universal_controller_node - 包含解释器、RC Hub、控制 Hub 的单进程入口

参数:
  robot_type: 机器人类型，infantry 或 sentry（默认 sentry）
  config_file: 配置文件路径（默认根据 robot_type 自动选择）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """根据 robot_type 参数选择对应的配置文件"""
    robot_type = context.launch_configurations['robot_type']
    pkg_dir = get_package_share_directory('universal_controller')

    # 允许用户通过 config_file 参数覆盖默认配置
    user_config = context.launch_configurations.get('config_file', '')
    if user_config and os.path.isfile(user_config):
        config_file = user_config
    elif robot_type == 'infantry':
        config_file = os.path.join(
            pkg_dir, 'config', 'controller_params.infantry.yaml')
    else:
        config_file = os.path.join(
            pkg_dir, 'config', 'controller_params.sentry.yaml')

    return [
        Node(
            package='universal_controller',
            executable='universal_controller_node',
            parameters=[config_file],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type',
            default_value='sentry',
            description='Robot type: infantry or sentry'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to the configuration file (overrides robot_type default)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
