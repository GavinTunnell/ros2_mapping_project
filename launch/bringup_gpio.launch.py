#!/usr/bin/env python3
"""Composite launch for Nav2, sensor stack, and GPIO drivetrain."""

import os, math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "t", "yes", "on"}

def _declare_argument(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)

def _launch_setup(context, *args, **kwargs):
    actions = []
    package_name = 'ros2_mapping_project'
    share_dir = get_package_share_directory(package_name)

    # ----- Feature toggles -----
    use_internal_lidar      = _as_bool(LaunchConfiguration('use_internal_lidar').perform(context))
    use_imu_velocity        = _as_bool(LaunchConfiguration('use_imu_velocity').perform(context))
    use_diff_drive_odometry = _as_bool(LaunchConfiguration('use_diff_drive_odometry').perform(context))
    use_motor_driver        = _as_bool(LaunchConfiguration('use_motor_driver').perform(context))
    use_nav2                = _as_bool(LaunchConfiguration('use_nav2').perform(context))
    use_teleop              = _as_bool(LaunchConfiguration('use_teleop').perform(context))

    # ----- File paths / configs -----
    nav2_params         = LaunchConfiguration('nav2_params').perform(context)
    ekf_config          = LaunchConfiguration('ekf_config').perform(context)
    twist_mux_config    = LaunchConfiguration('twist_mux_config').perform(context)
    robot_description_file = LaunchConfiguration('robot_description_file').perform(context)

    # ----- Kinematics / encoders -----
    wheel_separation = LaunchConfiguration('wheel_separation').perform(context)
    wheel_radius     = LaunchConfiguration('wheel_radius').perform(context)
    enc_tpr          = LaunchConfiguration('encoder_ticks_per_rev').perform(context)

    def _as_float(s):
        try:
            return float(s) if s not in ("", None) else None
        except ValueError:
            return None

    L  = _as_float(wheel_separation) or 0.24
    r  = _as_float(wheel_radius)     or 0.05
    tpr= _as_float(enc_tpr)          or 333.3333

    # Encoder pins
    try:    left_A_pin  = int(LaunchConfiguration('left_A_pin').perform(context))
    except: left_A_pin  = 12
    try:    left_B_pin  = int(LaunchConfiguration('left_B_pin').perform(context))
    except: left_B_pin  = 16
    try:    right_A_pin = int(LaunchConfiguration('right_A_pin').perform(context))
    except: right_A_pin = 7
    try:    right_B_pin = int(LaunchConfiguration('right_B_pin').perform(context))
    except: right_B_pin = 11

    # Motor driver params
    cmd_topic        = LaunchConfiguration('motor_cmd_topic').perform(context)
    max_speed_mps    = LaunchConfiguration('max_speed_mps').perform(context)
    deadband_mps     = LaunchConfiguration('deadband_mps').perform(context)
    pwm_freq_hz      = LaunchConfiguration('pwm_freq_hz').perform(context)
    duty_min         = LaunchConfiguration('duty_min').perform(context)
    left_invert      = LaunchConfiguration('left_invert').perform(context)
    right_invert     = LaunchConfiguration('right_invert').perform(context)
    brake_on_stop    = LaunchConfiguration('brake_on_stop').perform(context)

    slam_params       = os.path.join(share_dir, 'slam_params.yaml')
    imu_filter_params = os.path.join(share_dir, 'imu_filter_params.yaml')

    # ----- Environment (FastDDS SHM off) -----
    actions.append(SetEnvironmentVariable(name='FASTDDS_TRANSPORT_SHARED_MEM', value='off'))

    # ----- Robot description -----
    if robot_description_file:
        if os.path.exists(robot_description_file):
            with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
                robot_description = urdf_file.read()
            actions.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                name='robot_state_publisher', output='screen'
            ))
        else:
            actions.append(LogInfo(msg=f'robot_description_file not found: {robot_description_file}'))

    # ----- LiDAR -----
    if use_internal_lidar:
        actions.append(Node(
            package=package_name, executable='lidar_node',
            name='lidar_node', output='screen'
        ))
    else:
        actions.append(Node(
            package='sllidar_ros2', executable='sllidar_node',
            name='sllidar_node', output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ))

    # ----- IMU + filter -----
    actions.extend([
        Node(package=package_name, executable='imu_node', name='imu_node', output='screen'),
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='imu_filter', output='screen',
            parameters=[imu_filter_params, {'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
            remappings=[('/imu/data_raw', '/imu/raw'), ('/imu/data', '/imu/data')]
        ),
    ])
    if use_imu_velocity:
        actions.append(Node(
            package=package_name, executable='imu_twist_odom_node',
            name='imu_twist_odom_node', output='screen'
        ))

    # ----- Encoders -----
    actions.append(Node(
        package=package_name, executable='encoders_node',
        name='encoders_node', output='screen',
        parameters=[{
            'left_A': left_A_pin, 'left_B': left_B_pin,
            'right_A': right_A_pin, 'right_B': right_B_pin,
            'wheel_radius': r, 'wheel_base': L, 'ticks_per_rev': tpr,
        }]
    ))

    # ----- Optional external diff-drive odom from ticks -----
    if use_diff_drive_odometry:
        ticks_per_meter = tpr / (2.0 * math.pi * r)
        actions.append(Node(
            package=LaunchConfiguration('diff_drive_package').perform(context),
            executable=LaunchConfiguration('diff_drive_executable').perform(context),
            name='diff_drive_odometry', output='screen',
            parameters=[{
                'ticks_per_meter': ticks_per_meter,
                'wheel_separation': L, 'base_frame_id': 'base_link',
                'odom_frame_id': 'odom', 'rate': 50.0,
            }],
            remappings=[
                ('/diff_drive/odom', '/wheel_odom'),
                ('/diff_drive/lwheel_ticks', '/left_wheel/ticks'),
                ('/diff_drive/rwheel_ticks', '/right_wheel/ticks'),
            ]
        ))

    # ----- EKF (robot_localization) -----
    if ekf_config and os.path.exists(ekf_config):
        ekf_params = [ekf_config]
        if use_imu_velocity:
            ekf_params.append({'twist0': '/imu_odom'})
        actions.append(Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen', parameters=ekf_params
        ))
    else:
        actions.append(LogInfo(msg=f'ekf_config not found; expected at {ekf_config}'))

    # ----- Static TFs -----
    actions.extend([
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='laser_base_tf', arguments=['0','0','0.10','0','0','0','base_link','laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='imu_base_tf',   arguments=['0','0','0.05','0','0','0','base_link','imu_link']),
    ])

    # ----- SLAM toolbox -----
    if os.path.exists(slam_params):
        actions.append(Node(
            package='slam_toolbox', executable='sync_slam_toolbox_node',
            name='slam_toolbox', output='screen', parameters=[slam_params]
        ))
    else:
        actions.append(LogInfo(msg=f'slam_params.yaml not found at {slam_params}'))

    # ----- Nav2 velocity smoother -----
    actions.append(Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        remappings=[('cmd_vel', '/cmd_vel'), ('cmd_vel_smoothed', '/cmd_vel_smoothed')]
    ))

    # ----- twist_mux -----
    if twist_mux_config and os.path.exists(twist_mux_config):
        actions.append(Node(
            package='twist_mux', executable='twist_mux',
            name='twist_mux', output='screen',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', '/cmd_vel_out')]
        ))
    else:
        actions.append(LogInfo(msg=f'twist_mux_config not found; expected at {twist_mux_config}'))

    # ----- Motor driver (GPIO + PWM) -----
    if use_motor_driver:
        actions.append(Node(
            package=LaunchConfiguration('motor_driver_package').perform(context),
            executable=LaunchConfiguration('motor_driver_executable').perform(context),
            name='motor_driver_node', output='screen',
            parameters=[{
                'cmd_topic':     cmd_topic or '/cmd_vel_out',
                'wheel_base':    L,
                'max_speed_mps': float(max_speed_mps or 0.6),
                'deadband_mps':  float(deadband_mps or 0.03),
                'pwm_freq_hz':   int(pwm_freq_hz or 500),
                'duty_min':      float(duty_min or 15.0),
                'left_invert':   _as_bool(left_invert or 'false'),
                'right_invert':  _as_bool(right_invert or 'false'),
                'brake_on_stop': _as_bool(brake_on_stop or 'true'),
            }]
        ))

    # ----- Optional teleop for bench testing -----
    if use_teleop:
        actions.append(Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop', output='screen',
            remappings=[('cmd_vel', '/cmd_vel')],
            prefix=['xterm -e ']  # opens in a terminal so you can press keys
        ))

    # ----- Nav2 bringup -----
    if use_nav2:
        if nav2_params and os.path.exists(nav2_params):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', 'nav2_minimal.launch.py')),
                launch_arguments={'use_sim_time': 'false', 'params_file': nav2_params}.items()
            ))
        else:
            actions.append(LogInfo(msg=f'Nav2 params file not found; expected at {nav2_params}'))

    return actions


def generate_launch_description():
    package_name = 'ros2_mapping_project'
    share_dir = get_package_share_directory(package_name)

    default_ekf       = os.path.join(share_dir, 'config', 'ekf.yaml')
    default_twist_mux = os.path.join(share_dir, 'config', 'twist_mux.yaml')

    ld = LaunchDescription()

    # Feature toggles
    ld.add_action(_declare_argument('use_internal_lidar', 'false', 'Use the custom lidar_node instead of sllidar_ros2.'))
    ld.add_action(_declare_argument('use_imu_velocity', 'true',  'Fuse IMU-derived velocity (/imu_odom) into the EKF.'))
    ld.add_action(_declare_argument('use_diff_drive_odometry', 'false', 'Enable external diff-drive odometry node.'))
    ld.add_action(_declare_argument('use_motor_driver', 'true',  'Launch the GPIO motor driver node.'))
    ld.add_action(_declare_argument('use_nav2', 'true', 'Start the Nav2 bringup launch file.'))
    ld.add_action(_declare_argument('use_teleop', 'false', 'Start teleop_twist_keyboard for manual control.'))

    # Files
    ld.add_action(_declare_argument('nav2_params', default=os.path.join(share_dir, 'nav2_params.yaml'),
                                    description='Nav2 params YAML file.'))
    ld.add_action(_declare_argument('ekf_config', default_ekf, 'robot_localization EKF config file.'))
    ld.add_action(_declare_argument('twist_mux_config', default_twist_mux, 'twist_mux configuration file.'))
    ld.add_action(_declare_argument('robot_description_file', os.path.join(share_dir, 'description', 'robot.urdf'),
                                    'Path to the robot URDF file.'))

    # Geometry / encoders
    ld.add_action(_declare_argument('wheel_separation', '0.24', 'Distance between the drive wheels (meters).'))
    ld.add_action(_declare_argument('wheel_radius',     '0.05', 'Drive wheel radius (meters).'))
    ld.add_action(_declare_argument('encoder_ticks_per_rev', '333.3333', 'Encoder ticks per wheel revolution.'))
    ld.add_action(_declare_argument('left_A_pin',  '12', 'BOARD pin number for the left encoder phase A.'))
    ld.add_action(_declare_argument('left_B_pin',  '16', 'BOARD pin number for the left encoder phase B.'))
    ld.add_action(_declare_argument('right_A_pin', '7',  'BOARD pin number for the right encoder phase A.'))
    ld.add_action(_declare_argument('right_B_pin', '11', 'BOARD pin number for the right encoder phase B.'))

    # External diff drive package/exe (only used if toggle above is true)
    ld.add_action(_declare_argument('diff_drive_package',   'diff_drive',          'Package providing the diff_drive_odometry executable.'))
    ld.add_action(_declare_argument('diff_drive_executable','diff_drive_odometry', 'Executable that publishes odometry from encoder ticks.'))

    # Motor driver parameters
    ld.add_action(_declare_argument('motor_driver_package',    package_name, 'Package name for the GPIO motor driver node.'))
    ld.add_action(_declare_argument('motor_driver_executable', 'motor_driver_node', 'Executable for the GPIO motor driver.'))
    ld.add_action(_declare_argument('motor_cmd_topic', '/cmd_vel_out', 'Which Twist topic to listen to (e.g., /cmd_vel_out or /cmd_vel).'))
    ld.add_action(_declare_argument('max_speed_mps',  '0.6',  'Speed corresponding to 100% duty.'))
    ld.add_action(_declare_argument('deadband_mps',   '0.03', 'Below this speed, output 0 duty.'))
    ld.add_action(_declare_argument('pwm_freq_hz',    '500',  'PWM frequency (Hz).'))
    ld.add_action(_declare_argument('duty_min',       '15.0', 'Minimum duty to overcome stiction.'))
    ld.add_action(_declare_argument('left_invert',    'false','Invert left direction.'))
    ld.add_action(_declare_argument('right_invert',   'false','Invert right direction.'))
    ld.add_action(_declare_argument('brake_on_stop',  'true', 'Active brake (both highs) instead of coast on stop.'))

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
