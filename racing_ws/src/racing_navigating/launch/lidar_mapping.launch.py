#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    package_dir = get_package_share_directory('racing_navigating')
    dev_base_dir = get_package_share_directory('origincar_base')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    foxglove_dir = get_package_share_directory('foxglove_bridge')

    launch_base = LaunchConfiguration('launch_base')
    launch_foxglove = LaunchConfiguration('launch_foxglove')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_rear_lidar = LaunchConfiguration('use_rear_lidar')
    launch_rear_driver = LaunchConfiguration('launch_rear_driver')
    front_scan_topic = LaunchConfiguration('front_scan_topic')
    rear_scan_topic = LaunchConfiguration('rear_scan_topic')
    merged_scan_topic = LaunchConfiguration('merged_scan_topic')
    rear_laser_x = LaunchConfiguration('rear_laser_x')
    rear_laser_y = LaunchConfiguration('rear_laser_y')
    rear_laser_z = LaunchConfiguration('rear_laser_z')
    rear_laser_roll = LaunchConfiguration('rear_laser_roll')
    rear_laser_pitch = LaunchConfiguration('rear_laser_pitch')
    rear_laser_yaw = LaunchConfiguration('rear_laser_yaw')

    slam_params_file = os.path.join(package_dir, 'config', 'slam_toolbox_lidar.yaml')
    nav2_params_file = os.path.join(package_dir, 'config', 'nav2_params_lidar.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('launch_base', default_value='false'),
        DeclareLaunchArgument('launch_foxglove', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('use_rear_lidar', default_value='false'),
        DeclareLaunchArgument('launch_rear_driver', default_value='false'),
        DeclareLaunchArgument('front_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('rear_scan_topic', default_value='/scan_rear'),
        DeclareLaunchArgument('merged_scan_topic', default_value='/scan_merged'),
        DeclareLaunchArgument('rear_laser_x', default_value='-0.138'),
        DeclareLaunchArgument('rear_laser_y', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_z', default_value='0.11'),
        DeclareLaunchArgument('rear_laser_roll', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('rear_laser_yaw', default_value='3.1415926'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(dev_base_dir, 'launch', 'origincar_bringup.launch.py')
            ),
            condition=IfCondition(launch_base),
        ),

        LifecycleNode(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            namespace='x10_rear',
            parameters=[{
                'lidar_type': 'X10',
                'lidar_model': 'N10',
                'serial_port': '/dev/wheeltec_lidar_rear',
                'device_ip': '192.168.1.200',
                'msop_port': 2368,
                'difop_port': 2369,
                'packet_rate': 188.0,
                'frame_id': 'rear_laser',
                'pointcloud_topic': 'lslidar_point_cloud_rear',
                'laserscan_topic': rear_scan_topic,
                'use_time_service': False,
                'use_first_point_time': False,
                'publish_scan': True,
                'use_high_precision': False,
                'publish_multiecholaserscan': False,
                'enable_noise_filter': False,
                'N10Plus_hz': 10,
                'min_range': 0.15,
                'max_range': 50.0,
                'angle_disable_min': [0],
                'angle_disable_max': [0],
                'is_pretreatment': False,
                'x_offset': 0.0,
                'y_offset': 0.0,
                'z_offset': 0.0,
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0,
                'is_MatrixTransformation': False,
            }],
            output='screen',
            condition=IfCondition(launch_rear_driver),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_laser_tf',
            arguments=[
                rear_laser_x,
                rear_laser_y,
                rear_laser_z,
                rear_laser_roll,
                rear_laser_pitch,
                rear_laser_yaw,
                'base_link',
                'rear_laser',
            ],
            condition=IfCondition(use_rear_lidar),
        ),

        Node(
            package='racing_navigating',
            executable='scan_merger',
            name='scan_merger',
            parameters=[{
                'front_scan_topic': front_scan_topic,
                'rear_scan_topic': rear_scan_topic,
                'output_scan_topic': merged_scan_topic,
                'target_frame': 'base_footprint',
                'source_timeout_sec': 0.6,
                'publish_rate_hz': 8.0,
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': nav2_params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
                'log_level': log_level,
            }.items(),
        ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(foxglove_dir, 'launch', 'foxglove_bridge_launch.xml')
            ),
            condition=IfCondition(launch_foxglove),
        ),
    ])
