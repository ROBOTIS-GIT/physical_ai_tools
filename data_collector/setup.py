from setuptools import setup
from glob import glob
import os

package_name = 'data_collector'

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
            'data_collector = data_collector.subscriber_node:main',
        ],
    },
)

