# setup.py for the search_rescue_robot package (ament_python build type)
# installs the python nodes, launch files, URDF, world, and config files

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'search_rescue_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    # data_files tells colcon where to install non-python files
    # (launch files, URDF, world, config) into the share directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.ie',
    description='Search and rescue robot for ROS 2 assignment',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    # these entry points create the ros2 run executables
    # e.g. ros2 run search_rescue_robot twist_relay
    entry_points={
        'console_scripts': [
            'twist_relay = search_rescue_robot.twist_relay:main',
            'battery_simulator = search_rescue_robot.battery_simulator:main',
        ],
    },
)
