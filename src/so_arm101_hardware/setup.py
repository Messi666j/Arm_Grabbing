import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'so_arm101_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='freebird',
    maintainer_email='freebird@todo.todo',
    description='Hardware Serial Driver and Real-Time State Publisher for SO-ARM101',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_driver_node = so_arm101_hardware.serial_driver_node:main',
        ],
    },
)
