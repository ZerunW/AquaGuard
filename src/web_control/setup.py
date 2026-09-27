import os
from glob import glob
from setuptools import setup

package_name = 'web_control'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer_email='user@example.com',
    description='Small web UI (two buttons) publishing /motor/trigger, for use behind a Cloudflare Tunnel',
    license='MIT',
    entry_points={'console_scripts': ['web_control_node = web_control.web_control_node:main']},
)
