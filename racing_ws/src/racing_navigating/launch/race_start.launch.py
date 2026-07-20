#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mission_pkg_dir = get_package_share_directory('racing_mission')
    mission_launch = os.path.join(mission_pkg_dir, 'launch', 'mission_autorun.launch.py')

    launch_obstacle_detection = LaunchConfiguration('launch_obstacle_detection')
    parking_y_threshold = LaunchConfiguration('parking_y_threshold')
    track_record_file = LaunchConfiguration('track_record_file')
    camera_topic = LaunchConfiguration('camera_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_obstacle_detection',
            default_value='true',
            description='是否同时启动障碍物检测节点',
        ),
        DeclareLaunchArgument(
            'parking_y_threshold',
            default_value='420',
            description='停车牌触发阈值，需与 racing_control 默认值保持一致',
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

        # 比赛总入口统一代理到完整编排链路，避免与 mission_autorun 出现行为漂移。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission_launch),
            launch_arguments={
                'launch_obstacle_detection': launch_obstacle_detection,
                'parking_y_threshold': parking_y_threshold,
                'track_record_file': track_record_file,
                'camera_topic': camera_topic,
            }.items(),
        ),
    ])
