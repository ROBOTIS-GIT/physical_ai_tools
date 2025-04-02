from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_data_subscriber'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seongwoo Kim',
    maintainer_email='kimsw@robotis.com',
    description='Open Platform AI ROS 2 subscriber',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'robot_data_subscriber = robot_data_subscriber.subscriber_node:main',
        ],
    },
)

