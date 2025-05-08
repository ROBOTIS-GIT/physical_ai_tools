from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'data_collector'
authors_info = [
    ('Seongwoo Kim', 'kimsw@robotis.com'),
    ('Hyungyu Kim', 'kimhg@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name=package_name,
    version='0.3.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors_info,
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
    description='ROS 2 package for Open Platform AI Kit integration',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = data_collector.topic_to_data:main',
            'data_collector_omx = data_collector.topic_to_data_omx:main',
            'data_collector_inspire = data_collector.topic_to_data_inspire:main',
            'trajectory_stamper = data_collector.trajectory_stamper:main',
            'trajectory_stamper_inspire = data_collector.trajectory_stamper_inspire:main',
        ],
    },
)
