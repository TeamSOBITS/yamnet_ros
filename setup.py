import os
from glob import glob
from setuptools import setup

package_name = 'yamnet_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.yamnet_src'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        # CSV is data, not Python — must go in data_files to be installed reliably
        (os.path.join('share', package_name, 'yamnet_src'),
         ['yamnet_ros/yamnet_src/yamnet_class_map.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='sobits@todo.todo',
    description='YAMNet-based real-time doorbell/bell sound detection for ROS 2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'yamnet_node = yamnet_ros.yamnet_node:main',
        ],
    },
)
