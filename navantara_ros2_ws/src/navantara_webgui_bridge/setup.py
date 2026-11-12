from setuptools import setup
import os
from glob import glob

package_name = 'navantara_webgui_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navantara UMRAH Robotics Team',
    maintainer_email='navantara@umrah.ac.id',
    description='Telemetry logging and WebGUI bridge package for ASV Navantara',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webgui_bridge_node = navantara_webgui_bridge.webgui_bridge_node:main',
        ],
    },
)
