from setuptools import find_packages
from setuptools import setup

setup(
    name='mg400_msgs',
    version='1.3.3',
    packages=find_packages(
        include=('mg400_msgs', 'mg400_msgs.*')),
)
