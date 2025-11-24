from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'physical_ai_bt'
authors_info = [
    ('Seongwoo Kim', 'kimsw@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name=package_name,
    version='0.7.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/trees', glob('trees/*.xml')),
        ('share/' + package_name + '/bt_bringup/launch', glob('bt_bringup/launch/*.launch.py')),
        ('share/' + package_name + '/bt_bringup/params', glob('bt_bringup/params/*.yaml')),
    ],
    install_requires=['setuptools', 'physical_ai_interfaces', 'numpy', 'lxml'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS 2 package for Behavior Tree based robot control',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'physical_ai_bt = physical_ai_bt.bt_node:main',
            'bt_node = physical_ai_bt.bt_node:main',
        ],
    },
)
