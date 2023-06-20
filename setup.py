import os
from glob import glob
from setuptools import setup

package_name = 'dynamixel_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erwan MARTIN',
    maintainer_email='emartin@cesi.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = dynamixel_arm_control.control_node:main',
            'positions_node = dynamixel_arm_control.positions_node:main',
            'hardware_manager_node = dynamixel_arm_control.hardware_manager_node:main',
            'moveit_controller_bridge = dynamixel_arm_control.moveit_controller_bridge:main',
            'path_execution_moveit_from_csv = dynamixel_arm_control.path_execution_moveit_from_csv:main',
            'path_test = dynamixel_arm_control.path_test:main',

        ],
    },
)
