from setuptools import find_packages, setup

package_name = 'lane_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seongjin',
    maintainer_email='smoony0226ai@kookmin.ac.kr',
    description='HSV + sliding window lane detection (camera-only, migrated from ROS1)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_detection_node = lane_detection.lane_detection_node:main',
        ],
    },
)
