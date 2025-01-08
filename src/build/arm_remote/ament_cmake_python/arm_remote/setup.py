from setuptools import find_packages
from setuptools import setup

setup(
    name='arm_remote',
    version='0.0.0',
    packages=find_packages(
        include=('arm_remote', 'arm_remote.*')),
)
