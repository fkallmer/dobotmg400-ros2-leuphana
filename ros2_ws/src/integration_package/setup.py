from setuptools import find_packages, setup
import os
from glob import glob

# Define the package name
package_name = 'integration_package'

# Setup function to define the package metadata and configuration
setup(
    name=package_name,  # Package name
    version='0.0.0',  # Package version
    packages=find_packages(exclude=['test']),  # Find all packages, excluding 'test'
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # ROS 2 resource index
        ('share/' + package_name, ['package.xml']),  # Package metadata
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),  # URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),  # XACRO files
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),  # RVIZ config files
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Launch files
    ],
    install_requires=['setuptools'],  # Installation dependencies
    zip_safe=True,  # Safe for zip usage
    maintainer='fkallmer',  # Maintainer
    maintainer_email='falk-richard.kallmer@stud.leuphana.de', 
    description='Integration of Dobot MG400 and RPLIDAR A1M8', 
    license='MIT', 
    tests_require=['pytest'],  # Test dependencies
    entry_points={
        'console_scripts': [],  # Executable scripts
    },
)
