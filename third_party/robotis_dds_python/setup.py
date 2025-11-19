from setuptools import setup, find_packages

setup(
    name='robotis_dds_python',
    version='0.1.0',
    author='Taehyeong Kim',
    author_email='kth@robotis.com',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    long_description=open('README.md').read(),
    long_description_content_type="text/markdown",
    license="Apache-2.0",
    packages=find_packages(include=['robotis_dds_python', 'robotis_dds_python.*']),
    description='Python SDK for DDS-based communication using Cyclone DDS',
    project_urls={
        "Source Code": "https://github.com/robotis-git/robotis_dds_python",
    },
    python_requires='>=3.8',
    install_requires=[
        "cyclonedds==0.10.2",
    ],
)
