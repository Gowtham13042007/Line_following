from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_following_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include URDF and Gazebo files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gowtham',
    maintainer_email='papanigowtham@gmail.com',
    description='A line following robot using OpenCV and ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'line_follower = line_following_robot.line_follower:main',
        ],
    },
)