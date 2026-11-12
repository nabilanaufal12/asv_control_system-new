from setuptools import setup
import os
from glob import glob

package_name = 'navantara_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navantara UMRAH Robotics Team',
    maintainer_email='navantara@umrah.ac.id',
    description='Nav2 autonomous navigation package for ASV Navantara',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'buoy_navigator_node = navantara_navigation.buoy_navigator_node:main',
        ],
    },
)
