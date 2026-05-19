#!/usr/bin/env python3
"""
tour_guide_launch.py  (REAL ROBOT version)
==========================================

Launches:
1. TurtleBot4 localization stack:
   map_server + AMCL + lifecycle manager

2. Custom tour-guide nodes:
   obstacle_mapper_node
   stop_manager_node
   route_planner_node
   route_executor_node
   mission_manager_node

Important:
- Custom nodes are delayed so map_server and AMCL can become active first.
- This avoids the intermittent startup issue where planner reports NO_MAP
  and mission_manager cannot find map -> base_footprint.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


# Change this to wherever you saved the map.
MAP_YAML = os.path.expanduser('/home/bash0030/map/my_map.yaml')


def generate_launch_description():

    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    # TurtleBot4 localization:
    # Starts map_server, AMCL, and lifecycle manager.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_nav, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'map': MAP_YAML,
        }.items(),
    )

    def tour_node(executable, name):
        return Node(
            package='tour_guide',
            executable=executable,
            name=name,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
            }],
        )

    obstacle_mapper = tour_node('obstacle_mapper_node', 'obstacle_mapper_node')
    stop_manager = tour_node('stop_manager_node', 'stop_manager_node')
    route_planner = tour_node('route_planner_node', 'route_planner_node')
    route_executor = tour_node('route_executor_node', 'route_executor_node')
    mission_manager = tour_node('mission_manager_node', 'mission_manager_node')

    delayed_tour_nodes = TimerAction(
        period=8.0,
        actions=[
            obstacle_mapper,
            stop_manager,
            route_planner,
            route_executor,
            mission_manager,
        ],
    )

    return LaunchDescription([
        localization,
        delayed_tour_nodes,
    ])
