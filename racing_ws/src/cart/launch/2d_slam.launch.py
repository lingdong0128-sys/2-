from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    dev_base_dir = FindPackageShare('origincar_base').find('origincar_base')

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    with_rviz_arg = DeclareLaunchArgument('with_rviz', default_value='false')
    launch_base_arg = DeclareLaunchArgument('launch_base', default_value='true')
    use_rear_lidar_arg = DeclareLaunchArgument('use_rear_lidar', default_value='true')
    launch_rear_driver_arg = DeclareLaunchArgument('launch_rear_driver', default_value='false')
    front_scan_topic_arg = DeclareLaunchArgument('front_scan_topic', default_value='/scan')
    rear_scan_topic_arg = DeclareLaunchArgument('rear_scan_topic', default_value='/scan_rear')
    rear_laser_x_arg = DeclareLaunchArgument('rear_laser_x', default_value='-0.138')
    rear_laser_y_arg = DeclareLaunchArgument('rear_laser_y', default_value='0.0')
    rear_laser_z_arg = DeclareLaunchArgument('rear_laser_z', default_value='0.11')
    rear_laser_roll_arg = DeclareLaunchArgument('rear_laser_roll', default_value='0.0')
    rear_laser_pitch_arg = DeclareLaunchArgument('rear_laser_pitch', default_value='0.0')
    rear_laser_yaw_arg = DeclareLaunchArgument('rear_laser_yaw', default_value='3.1415926')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dev_base_dir + '/launch/origincar_bringup.launch.py'),
        launch_arguments={
            'carto_slam': 'false',
            'base_odom_frame': 'odom',
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_base')),
    )

    rear_lidar_node = LifecycleNode(
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
            'laserscan_topic': LaunchConfiguration('rear_scan_topic'),
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
        condition=IfCondition(LaunchConfiguration('launch_rear_driver')),
    )

    rear_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rear_laser_tf',
        arguments=[
            LaunchConfiguration('rear_laser_x'),
            LaunchConfiguration('rear_laser_y'),
            LaunchConfiguration('rear_laser_z'),
            LaunchConfiguration('rear_laser_roll'),
            LaunchConfiguration('rear_laser_pitch'),
            LaunchConfiguration('rear_laser_yaw'),
            'base_link',
            'rear_laser',
        ],
        condition=IfCondition(LaunchConfiguration('use_rear_lidar')),
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('scan', LaunchConfiguration('front_scan_topic'))],
        arguments=[
            '-configuration_directory', FindPackageShare('cart').find('cart') + '/config',
            '-configuration_basename', 'backpack_2d.lua'],
        output='screen'
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05}],
        output='screen'
    )

    # 小车无显示屏，默认不在机器人端启动 RViz。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', FindPackageShare('cart').find('cart') + '/rviz/slam.rviz'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        with_rviz_arg,
        launch_base_arg,
        use_rear_lidar_arg,
        launch_rear_driver_arg,
        front_scan_topic_arg,
        rear_scan_topic_arg,
        rear_laser_x_arg,
        rear_laser_y_arg,
        rear_laser_z_arg,
        rear_laser_roll_arg,
        rear_laser_pitch_arg,
        rear_laser_yaw_arg,
        base_bringup,
        rear_lidar_node,
        rear_laser_tf,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])
