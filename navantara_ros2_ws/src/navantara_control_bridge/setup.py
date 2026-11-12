from setuptools import setup
import os
from glob import glob

package_name = 'navantara_control_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Navantara UMRAH Robotics Team',
    maintainer_email='navantara@umrah.ac.id',
    description='ROS2 Serial Control Bridge for ASV Navantara',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = navantara_control_bridge.serial_bridge_node:main',
        ],
    },
)
