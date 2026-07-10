from setuptools import find_packages, setup

package_name = 'missions'

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
    description='SEA:ME hackathon mission nodes (traffic light, roundabout, dynamic obstacle)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traffic_light_mission_node = missions.traffic_light_mission_node:main',
            'roundabout_mission_node = missions.roundabout_mission_node:main',
            'dynamic_obs_mission_node = missions.dynamic_obs_mission_node:main',
        ],
    },
)
