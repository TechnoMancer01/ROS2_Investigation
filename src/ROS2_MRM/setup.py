from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ROS2_MRM'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files in urdf directory (both .urdf and .xacro)
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Include rviz files if they exist
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # Include config files
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarmstrong',
    maintainer_email='aarmstrong@example.com',
    description='7-DOF Mill Relining Robot with Xacro configuration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mill_relining_controller = ROS2_MRM.mill_relining_controller:main',
        ],
    },
)