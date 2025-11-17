
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Get the package share directory
    pkg_share = get_package_share_directory('ros2_mapping_project')

    #
    # LAUNCH CONFIGURATIONS
    #

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #
    # PARAMETER FILES
    #

    odom_params_file = os.path.join(pkg_share, 'config', 'odom_params.yaml')

    #
    # LAUNCH FILES
    #

    # Launch file for hardware drivers
    bringup_gpio_launch_file = os.path.join(pkg_share, 'launch', 'bringup_gpio.launch.py')
    bringup_gpio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_gpio_launch_file)
    )

    # Launch file for SLAM
    slam_launch_file = os.path.join(pkg_share, 'launch', 'slam.launch.py')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file)
    )

    #
    # NODES
    #

    # EKF node - defined here so we can target it with the event handler
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[odom_params_file],
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    #
    # EVENT HANDLERS
    #

    # Delay start of SLAM until after the EKF is running
    delay_slam_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ekf_node,
            on_start=[slam],
        )
    )

    #
    # LAUNCH DESCRIPTION
    #

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation/Gazebo clock'))

    # Start the robot's hardware drivers first
    ld.add_action(bringup_gpio)

    # Then start the EKF
    ld.add_action(ekf_node)

    # Once the EKF is confirmed to be running, start SLAM
    ld.add_action(delay_slam_start)

    # Finally, start RViz
    ld.add_action(rviz_node)

    return ld
