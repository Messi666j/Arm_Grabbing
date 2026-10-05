import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'so_arm101_model_based'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='freebird',
    maintainer_email='freebird@todo.todo',
    description='Traditional Model-Based Control Package for SO-ARM101',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_node = so_arm101_model_based.simulation_node:main',
        ],
    },
)
