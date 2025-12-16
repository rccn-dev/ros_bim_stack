from setuptools import setup
from glob import glob
import os

package_name = 'ros_speckle_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Production-ready ROS 2 bridge for Speckle BIM data with offline caching',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ros_speckle_bridge.bridge_node:main',
            'list_streams = ros_speckle_bridge.list_streams:main',
        ],
    },
)
