#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('racing_navigating')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    dev_base_dir = get_package_share_directory('origincar_base')
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    aurora_driver_dir = get_package_share_directory('deptrum-ros-driver-aurora930')

    slam_params_file = os.path.join(package_dir, 'config', 'slam_toolbox_depth.yaml')

    launch_base = LaunchConfiguration('launch_base')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_rosbridge = LaunchConfiguration('launch_rosbridge')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('launch_base', default_value='false'),
        DeclareLaunchArgument('launch_camera', default_value='false'),
        DeclareLaunchArgument('launch_rosbridge', default_value='false'),
        # Use the existing URDF camera joint pose as the initial estimate.
        DeclareLaunchArgument('camera_x', default_value='0.1205'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.11'),
        DeclareLaunchArgument('camera_roll', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(dev_base_dir, 'launch', 'origincar_bringup.launch.py')),
            condition=IfCondition(launch_base),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(aurora_driver_dir, 'launch', 'aurora930_launch.py')),
            condition=IfCondition(launch_camera),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
            ),
            condition=IfCondition(launch_rosbridge),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_depth_camera',
            arguments=[
                camera_x,
                camera_y,
                camera_z,
                camera_roll,
                camera_pitch,
                camera_yaw,
                'base_link',
                'depth_camera_link',
            ],
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='aurora_points_to_scan',
            remappings=[
                ('cloud_in', '/aurora/points2'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.2,
                'min_height': 0.02,
                'max_height': 0.40,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.12,
                'range_max': 4.0,
                'use_inf': True,
                'queue_size': 8,
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
                'log_level': 'info',
            }.items(),
        ),
    ])
