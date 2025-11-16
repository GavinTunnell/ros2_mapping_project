
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the package share directory
    # bringup_dir = get_package_share_directory('ros2_mapping_project')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # odom_params_file = os.path.join(bringup_dir, 'config', 'odom_params.yaml')
    # slam_params_file = os.path.join(bringup_dir, 'config', 'slam_params.yaml')

    # For this project, the params files are in the root directory
    odom_params_file = 'odom_params.yaml'
    slam_params_file = 'slam_params.yaml'


    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    # Nodes

    static_transform_publisher_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']
    )

    static_transform_publisher_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_imu',
        output='screen',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

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

    imu_node = Node(
        package='ros2_mapping_project',
        executable='imu_bno055_smbus.py',
        name='imu_node',
        output='screen',
        parameters=[{
            'bus': 7,
            'address': 0x28
        }]
    )

    encoders_node = Node(
        package='ros2_mapping_project',
        executable='encoders_node.py',
        name='encoders_node',
        output='screen',
        parameters=[{
            'left_A': 7,
            'left_B': 11,
            'right_A': 12,
            'right_B': 16,
            'left_active_low': True,
            'right_active_low': True,
            'left_only_A_edges': False,
            'right_only_A_edges': False,
            'edge_min_us': 200,
            'debounce_ms': 0,
            'publish_rate_hz': 20.0,
            'wheel_radius': 0.05,
            'wheel_base': 0.24,
            'ticks_per_rev': 333.3333
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[odom_params_file],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', 'slam.rviz']
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)

    ld.add_action(static_transform_publisher_laser)
    ld.add_action(static_transform_publisher_imu)
    ld.add_action(rplidar_node)
    ld.add_action(imu_node)
    ld.add_action(encoders_node)
    ld.add_action(ekf_node)

    # Delay start of slam_toolbox until after ekf_node is running
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ekf_node,
            on_start=[slam_toolbox_node],
        )
    ))

    ld.add_action(rviz_node)

    return ld
