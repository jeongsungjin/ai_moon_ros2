import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'yolo_detect'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seongjin',
    maintainer_email='smoony0226ai@kookmin.ac.kr',
    description='YOLO26 traffic light / sign detection node (green/left/red/right)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_detect_node = yolo_detect.yolo_detect_node:main',
        ],
    },
)
