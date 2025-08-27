from setuptools import find_packages, setup

package_name = 'rosbag_to_lerobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/converter.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'torch',
        'cv-bridge',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS2 package to convert rosbag data to LeRobot dataset format',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag_to_lerobot_converter = rosbag_to_lerobot.rosbag_converter_node:main',
        ],
    },
)
