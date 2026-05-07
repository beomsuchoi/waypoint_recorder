from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'waypoint_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robit',
    maintainer_email='robit@kwangwoon.ac.kr',
    description='Records /goal_pose topics and saves them as Nav2-compatible YAML waypoints',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_recorder = waypoint_recorder.waypoint_recorder_node:main',
            'waypoint_follower = waypoint_recorder.waypoint_follower_node:main',
            'waypoint_recorder_ui = waypoint_recorder.waypoint_recorder_ui:main',
        ],
    },
)