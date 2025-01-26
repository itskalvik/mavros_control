import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mavros_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kalvik',
    maintainer_email='itskalvik@gmail.com',
    description='A ROS2 package for controlling ArduPilot-based robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_path_follower = mavros_control.waypoint_path_follower:main',
        ],
    },
)
