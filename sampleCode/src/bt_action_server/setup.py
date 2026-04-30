from setuptools import find_packages, setup

package_name = 'bt_action_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='dorian.tom@ul.ie',
    description='This package contains action servers for the behaviour tree tutorial.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_x_forward_server = bt_action_server.drive_x_forward:main',
            'rotate_x_server = bt_action_server.rotate_x_server:main',
        ],
    },
)
