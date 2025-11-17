
from setuptools import setup
from glob import glob
import os

package_name = 'ros2_mapping_project'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=[
        'encoders_node',
        'imu_node',
        'lidar_node',
        'motor_driver_pca_reg_dual',
    ],
    data_files=[
        # Standard ROS2 package files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install all launch files from the launch/ directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install all YAML parameter files from the config/ directory
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Install all RViz configuration files from the rviz/ directory
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Install the URDF file from the description/ directory (if it exists)
        (os.path.join('share', package_name, 'description'), glob('description/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team4',
    maintainer_email='team4@example.com',
    description='Custom mapping',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_node:main',
            'imu_node = imu_node:main',
            'encoders_node = encoders_node:main',
            'motor_driver_node = motor_driver_pca_reg_dual:main',
        ],
    },
)
