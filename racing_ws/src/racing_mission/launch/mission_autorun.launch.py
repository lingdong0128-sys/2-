#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigating_pkg_dir = get_package_share_directory('racing_navigating')
    control_pkg_dir = get_package_share_directory('racing_control')
    track_pkg_dir = get_package_share_directory('racing_track_detection')
    obstacle_pkg_dir = get_package_share_directory('racing_obstacle_detection')

    bringup_real_launch = os.path.join(navigating_pkg_dir, 'launch', 'bringup_real.launch.py')
    control_launch = os.path.join(control_pkg_dir, 'launch', 'racing_control.launch.py')
    track_launch = os.path.join(track_pkg_dir, 'launch', 'racing_track_detection.launch.py')
    obstacle_launch = os.path.join(obstacle_pkg_dir, 'launch', 'obstacle_detection.launch.py')

    launch_obstacle_detection = LaunchConfiguration('launch_obstacle_detection')
    parking_y_threshold = LaunchConfiguration('parking_y_threshold')
    track_record_file = LaunchConfiguration('track_record_file')
    camera_topic = LaunchConfiguration('camera_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parking_y_threshold',
            default_value='420',
            description='停车牌触发阈值，需与 racing_control 默认值保持一致',
        ),
        DeclareLaunchArgument(
            'launch_obstacle_detection',
            default_value='true',
            description='是否同时启动障碍物检测节点',
        ),
        DeclareLaunchArgument(
            'track_record_file',
            default_value='/userdata/racing_ws/recordings/recorded_path.yaml',
            description='巡线路径导出文件',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/aurora/rgb/image_raw',
            description='场景守卫使用的相机图像话题',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_real_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'log_level': 'info',
                'launch_obstacle_detection': 'false',
                'enable_fixed_start_reset': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(track_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'parking_y_threshold': parking_y_threshold,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obstacle_launch),
            condition=IfCondition(launch_obstacle_detection),
        ),

        Node(
            package='racing_mission',
            executable='scene_state_guard',
            name='scene_state_guard',
            output='screen',
            parameters=[
                {'image_topic': camera_topic},
            ],
        ),
        Node(
            package='racing_mission',
            executable='mission_orchestrator',
            name='mission_orchestrator',
            output='screen',
        ),
        Node(
            package='racing_mission',
            executable='track_map_recorder',
            name='track_map_recorder',
            output='screen',
            parameters=[
                {'output_file': track_record_file},
            ],
        ),
    ])
