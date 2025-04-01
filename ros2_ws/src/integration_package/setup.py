from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'integration_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),  # <- Dies fügt alle URDF-Dateien hinzu
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),  # <- Dies fügt alle RVIZ-Dateien hinzu
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # <- Dies fügt alle Launch-Dateien hinzu
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fkallmer',
    maintainer_email='falk-richard.kallmer@stud.leuphana.de',
    description='Integration of Dobot MG400 and RPLIDAR A1M8',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
