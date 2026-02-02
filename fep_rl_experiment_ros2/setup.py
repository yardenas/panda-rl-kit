from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fep_rl_experiment_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zrene',
    maintainer_email='zrene@example.com',
    description='Online reinforcement learning experiment for Franka Emika Panda (ROS2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_interface = fep_rl_experiment_ros2.robot_interface:main',
            'online_learning = fep_rl_experiment_ros2.online_learning:main',
            'dummy_image_publisher = fep_rl_experiment_ros2.dummy_image_publisher:main',
            'dummy_cube_publisher = fep_rl_experiment_ros2.dummy_cube_publisher:main',
        ],
    },
)
