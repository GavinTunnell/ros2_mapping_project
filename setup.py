from setuptools import setup
from glob import glob
import os

package_name = 'ros2_mapping_project'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=[
        'lidar_node',
        'imu_node',
        'encoders_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'description'), glob('description/*.urdf')),
        (os.path.join('share', package_name), glob('*.yaml')),
        (os.path.join('share', package_name), glob('*.rviz')),
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
            'motor_driver_node = motor_driver_node:main',
            'yolo_autofocus_servo = ros2_mapping_project.yolo_autofocus_servo_node:main',
        ],
    },
)
