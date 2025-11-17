
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_mapping_project')

    imu_filter_params_file = os.path.join(pkg_share, 'config', 'imu_filter_params.yaml')

    #
    # LAUNCH ARGUMENTS
    #

    lidar_tf_x = LaunchConfiguration('lidar_tf_x', default='0.0')
    lidar_tf_y = LaunchConfiguration('lidar_tf_y', default='0.0')
    lidar_tf_z = LaunchConfiguration('lidar_tf_z', default='0.10')

    #
    # NODES
    #

    # Static TF transform from base_link to the laser scanner frame
    static_transform_publisher_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=[lidar_tf_x, lidar_tf_y, lidar_tf_z, '0', '0', '0', 'base_link', 'laser']
    )

    # Static TF transform from base_link to the IMU frame
    static_transform_publisher_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_imu',
        output='screen',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True,
        }]
    )

    # IMU node (raw data)
    imu_node = Node(
        package='ros2_mapping_project',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'bus': 7,
            'address': 0x28
        }]
    )

    # IMU filter (fuses raw data)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[imu_filter_params_file],
        remappings=[('/imu/data_raw', '/imu/raw')]
    )

    # Encoders node
    encoders_node = Node(
        package='ros2_mapping_project',
        executable='encoders_node',
        name='encoders_node',
        output='screen',
        parameters=[{
            'left_A': 7,
            'left_B': 11,
            'right_A': 12,
            'right_B': 16,
            'publish_rate_hz': 20.0,
            'wheel_radius': 0.05,
            'wheel_base': 0.24,
            'ticks_per_rev': 333.3333
        }]
    )

    #
    # LAUNCH DESCRIPTION
    #

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('lidar_tf_x', default_value='0.0', description='Lidar TF X offset'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_y', default_value='0.0', description='Lidar TF Y offset'))
    ld.add_action(DeclareLaunchArgument('lidar_tf_z', default_value='0.10', description='Lidar TF Z offset'))

    ld.add_action(static_transform_publisher_laser)
    ld.add_action(static_transform_publisher_imu)
    ld.add_action(rplidar_node)
    ld.add_action(imu_node)
    ld.add_action(imu_filter_node)
    ld.add_action(encoders_node)

    return ld
