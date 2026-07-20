#!/usr/bin/env python3
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_start_pose(config_path):
    """读取固定起点配置，用于 map -> odom_combined 的初始对齐。"""
    with open(config_path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    return data.get('start_pose', {})


def generate_launch_description():
    package_dir = get_package_share_directory('racing_navigating')
    obstacle_pkg_dir = get_package_share_directory('racing_obstacle_detection')

    map_yaml_file = os.path.join(package_dir, 'map', 'map.yaml')
    params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')
    routes_file = os.path.join(package_dir, 'config', 'routes.yaml')
    landmarks_file = os.path.join(package_dir, 'config', 'landmarks.yaml')
    start_pose_file = os.path.join(package_dir, 'config', 'start_pose.yaml')
    navigation_launch_file = os.path.join(
        package_dir, 'launch', 'params', 'bringup', 'navigation_launch.py'
    )
    obstacle_launch_file = os.path.join(
        obstacle_pkg_dir, 'launch', 'obstacle_detection.launch.py'
    )

    start_pose = _load_start_pose(start_pose_file)

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    launch_obstacle_detection = LaunchConfiguration('launch_obstacle_detection')
    enable_fixed_start_reset = LaunchConfiguration('enable_fixed_start_reset')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='实体车运行时应保持 false',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='是否自动激活 Nav2 生命周期节点',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 与任务节点的日志级别',
        ),
        DeclareLaunchArgument(
            'launch_obstacle_detection',
            default_value='false',
            description='是否同时拉起现有的障碍物检测节点',
        ),
        DeclareLaunchArgument(
            'enable_fixed_start_reset',
            default_value='false',
            description='是否在本次启动时执行固定起点归零，只建议在整场任务最开始时使用',
        ),

        # 仅加载地图服务器，不再强依赖 AMCL。
        # 地图服务的包名为nav2_map_server
        # 可执行文件名为map_server
        # 传递了地图配置文件位置，配置文件位置，以及使用什么时间的配置
        # 为可执行文件传递了日志级别参数
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='log',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml_file},
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        # 启动生命周期节点的管理节点，但是这里只开了一个节点的管理
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='log',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': ['map_server']},
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),

        # 坐标对齐管理节点始终在线：任务起点时可做归零，扫码时可做地标矫正。
        # 包是racing_navigating的大包，下的可执行文件是fixed_start_manager
        # 传参是地图，odom_frame里程碑坐标系，启动的三个坐标系
        # 可能有问题的地方之一就是reset_count的传参问题，这个参数如果在小车开始跑了之后还在发归零会出大问题

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
                {'perform_initial_reset': enable_fixed_start_reset},
                {'reset_count': 3},
                {'reset_interval_sec': 0.3},
                {'publish_frequency': 20.0},
            ],
        ),

        # 复用示例中的 navigation_launch，只保留导航栈本身。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
                'log_level': log_level,
            }.items(),
        ),

        # 任务执行节点负责接二维码结果并读取 routes.yaml 跑固定路线。
        Node(
            package='racing_navigating',
            executable='route_executor',
            name='route_executor',
            output='screen',
            parameters=[
                {'routes_file': routes_file},
                {'landmarks_file': landmarks_file},
                {'start_route_sign_value': 5},
                {'finish_route_sign_value': 6},
                {'obstacle_confidence_threshold': 0.5},
                {'obstacle_bottom_threshold': 320},
                {'obstacle_clear_hold_sec': 1.0},
                {'local_avoidance_enabled': True},
                {'local_avoid_linear_speed': 0.10},
                {'local_avoid_angular_ratio': 0.20},
                {'local_recover_linear_speed': 0.12},
                {'local_recover_duration_sec': 0.8},
                {'local_avoid_timeout_sec': 5.0},
                {'image_center_x': 320.0},
                {'local_avoid_min_offset_px': 5.0},
            ],
        ),

        # 如果外部流程没提前启动障碍检测，也可以在这里一并拉起。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obstacle_launch_file),
            condition=IfCondition(launch_obstacle_detection),
        ),
    ])
