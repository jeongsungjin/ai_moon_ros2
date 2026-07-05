import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'car_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seongjin',
    maintainer_email='smoony0226ai@kookmin.ac.kr',
    description='Mission arbiter / main planner (migrated from ROS1 car_planner)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main_planner_node = car_planner.main_planner_node:main',
            'web_viewer_node = car_planner.web_viewer_node:main',
        ],
    },
)
