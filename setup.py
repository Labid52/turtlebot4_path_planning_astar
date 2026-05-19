import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tour_guide'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),  glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),  glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),  glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='labid.bashar@ou.edu',
    description='Autonomous tour-guide simulation for TurtleBot4',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node  = tour_guide.mission_manager_node:main',
            'stop_manager_node     = tour_guide.stop_manager_node:main',
            'route_planner_node    = tour_guide.route_planner_node:main',
            'route_executor_node   = tour_guide.route_executor_node:main',
            'obstacle_mapper_node  = tour_guide.obstacle_mapper_node:main',
        ],
    },
)
