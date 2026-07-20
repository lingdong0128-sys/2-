#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _load_start_pose(config_path):
    """读取整场任务起点配置。"""
    with open(config_path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return data.get('start_pose', {})


def generate_launch_description():
    package_dir = get_package_share_directory('racing_navigating')
    start_pose_file = os.path.join(package_dir, 'config', 'start_pose.yaml')
    start_pose = _load_start_pose(start_pose_file)

    reset_count = LaunchConfiguration('reset_count')
    reset_interval_sec = LaunchConfiguration('reset_interval_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'reset_count',
            default_value='5',
            description='向 /set_pose 重复发布零位姿的次数',
        ),
        DeclareLaunchArgument(
            'reset_interval_sec',
            default_value='0.3',
            description='两次归零消息之间的时间间隔',
        ),
        Node(
            package='racing_navigating',
            executable='fixed_start_manager',
            name='fixed_start_manager',
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'odom_frame': 'odom_combined'},
                {'start_x': float(start_pose.get('x', 0.0))},
                {'start_y': float(start_pose.get('y', 0.0))},
                {'start_yaw': float(start_pose.get('yaw', 0.0))},
                {'perform_initial_reset': True},
                {'reset_count': reset_count},
                {'reset_interval_sec': reset_interval_sec},
                {'publish_frequency': 20.0},
            ],
        ),
    ])
