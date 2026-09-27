import os
from glob import glob
from setuptools import setup

package_name = 'aquaguard_hw'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zerun',
    maintainer_email='zerunwang@gatech.edu',
    description='ROS2 package for AquaGuard hardware control',
    license='MIT',
    entry_points={'console_scripts': ['m3508_speed_node = aquaguard_hw.speed_node:main']},
)
