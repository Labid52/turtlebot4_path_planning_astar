

https://github.com/user-attachments/assets/b83ed885-5c1a-417d-afca-50bc58720953

# TurtleBot4 Tour Guide Mission

This repository contains a ROS 2 package named `tour_guide` for running an adaptive TurtleBot4 tour-guide mission on a saved indoor map. The robot visits a set of predefined landmarks, plans paths using a custom A* planner, follows the planned path with a custom route executor, detects front obstacles using LiDAR, and requests replanning when the current route becomes blocked.

## Project Overview

The system is designed for a real TurtleBot4 operating with a saved occupancy-grid map and AMCL localization. The mission starts after the localization stack becomes active and the custom tour-guide nodes receive the required map and transform data.

The main mission behavior is:

1. Load the saved map and start AMCL localization.
2. Select the next unvisited landmark using A* path cost.
3. Plan a global path to the selected landmark using a custom A* planner.
4. Execute the path using a proportional heading controller.
5. Monitor the front LiDAR sector for blocked-path conditions.
6. Replan around temporary obstacles when required.
7. Dwell at each landmark after arrival.
8. Return to the configured home pose after all landmarks are visited.

## Main Features

- ROS 2 package for TurtleBot4 real-robot navigation experiments.
- Custom A* global path planner.
- A*-based next-stop selection among unvisited landmarks.
- Finite-state mission manager for tour execution.
- LiDAR-based front obstacle detection.
- Temporary obstacle injection for replanning around sudden blockages.
- Custom route executor using proportional heading control.
- Final yaw alignment at landmarks and home pose.
- RViz visualization support through TurtleBot4 navigation visualization.

## Package Nodes

The package is organized around five custom ROS 2 nodes.

| Node | Purpose |
|---|---|
| `mission_manager_node` | Coordinates the full mission using a finite-state machine. It requests stops, sends planning goals, forwards paths to the executor, handles dwell time, and triggers replanning or recovery. |
| `stop_manager_node` | Stores landmark states and selects the next unvisited stop using A* path cost. |
| `route_planner_node` | Generates the global A* path from the current robot pose to the requested goal. It also supports temporary obstacle injection during replanning. |
| `route_executor_node` | Follows the planned path using a proportional heading controller and publishes velocity commands to the robot. |
| `obstacle_mapper_node` | Reads the LiDAR scan, checks the front sector, and publishes whether the robot path is blocked. |

The launch file also starts the TurtleBot4 localization stack, including map server, AMCL, and lifecycle manager.

## Repository Structure

A typical package structure is shown below.

```text
tour_guide/
├── launch/
│   └── tour_guide_launch.py
├── tour_guide/
│   ├── config.py
│   ├── mission_manager_node.py
│   ├── obstacle_mapper_node.py
│   ├── route_executor_node.py
│   ├── route_planner_node.py
│   └── stop_manager_node.py
├── package.xml
├── setup.py
└── README.md
```

## System Requirements

This project was run with:

- ROS 2
- TurtleBot4 robot
- TurtleBot4 navigation packages
- TurtleBot4 visualization packages
- A saved occupancy-grid map
- AMCL localization
- LiDAR scan topic available as `/scan`
- TF transform available from `map` to `base_footprint`

The launch file uses the TurtleBot4 localization launch file from the `turtlebot4_navigation` package.

## Map Configuration

The real-robot launch file loads the saved map from:

```python
MAP_YAML = os.path.expanduser('/home/bash0030/map/my_map.yaml')
```

Before running the project, update this path in `tour_guide_launch.py` if your map is stored in a different location.

Example:

```python
MAP_YAML = os.path.expanduser('~/map/my_map.yaml')
```

## Mission Configuration

The main mission parameters are defined in `config.py`.

Configured landmarks:

| Landmark | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| Library | -1.20 | 0.50 | 1.57 |
| Cafeteria | -2.20 | -0.3100 | -1.57 |
| Lab | -3.60 | -0.0844 | 1.57 |
| Office | -4.1507 | 1.125 | 1.57 |
| Reception | -4.80 | -0.3100 | -1.57 |

Configured home pose:

| Target | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| Home | -0.1325 | -0.0257 | 0.1521 |

Important parameters:

| Parameter | Value | Description |
|---|---:|---|
| `DWELL_TIME_S` | 5.0 | Time spent at each landmark after arrival. |
| `MAX_REPLAN_ATTEMPTS` | 3 | Maximum replanning attempts before recovery or skip behavior. |
| `PLAN_TIMEOUT_S` | 15.0 | Planning timeout. |
| `EXEC_TIMEOUT_S` | 120.0 | Execution timeout. |
| `SLAM_WARMUP_S` | 5.0 | Startup wait time before mission execution. |
| `INFLATION_RADIUS_M` | 0.40 | Obstacle inflation radius used by the A* planner. |
| `PATH_STEP_M` | 0.10 | Path interpolation step size. |
| `ARRIVAL_DIST_M` | 0.35 | Position threshold for goal arrival. |
| `FRONT_BLOCKED_DIST` | 0.30 | LiDAR distance threshold for detecting a blocked front path. |
| `FRONT_HALF_DEG` | 90.0 | Half-angle of the front LiDAR sector. |
| `BLOCKED_PERSIST_S` | 5.0 | Required blockage persistence before reporting a blocked route. |

## Build Instructions

Open a terminal and build the package from the ROS 2 workspace.

```bash
cd ~/ros2_ws
colcon build --packages-select tour_guide
source install/setup.bash
```

## Running the Project

The project was run using two terminals.

### Terminal 1: Launch the Tour Guide System

```bash
cd ~/ros2_ws
colcon build --packages-select tour_guide
source install/setup.bash
ros2 launch tour_guide tour_guide_launch.py
```

This command launches the TurtleBot4 localization stack and the custom tour-guide nodes.

### Terminal 2: Launch RViz

```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

Use RViz to monitor the map, robot pose, planned path, LiDAR scan, and mission behavior.

## Main ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/mission/state` | `std_msgs/String` | Current mission state and status information. |
| `/stop_manager/cmd` | `std_msgs/String` | Commands sent to the stop manager. |
| `/stop_manager/next_stop` | `geometry_msgs/PoseStamped` | Selected next landmark pose. |
| `/stop_manager/next_stop_name` | `std_msgs/String` | Name of the selected next landmark. |
| `/stop_manager/markers` | `visualization_msgs/MarkerArray` | Landmark visualization markers. |
| `/planner/goal` | `geometry_msgs/PoseStamped` | Goal pose sent to the route planner. |
| `/planner/path` | `nav_msgs/Path` | Planned A* path. |
| `/planner/status` | `std_msgs/String` | Planner status. |
| `/executor/path` | `nav_msgs/Path` | Path sent to the route executor. |
| `/executor/cancel` | `std_msgs/Bool` | Cancel signal for the executor. |
| `/executor/status` | `std_msgs/String` | Executor status. |
| `/executor/progress` | `std_msgs/Float32` | Path execution progress. |
| `/obstacle_mapper/front_blocked` | `std_msgs/Bool` | Boolean front-blocked signal. |
| `/obstacle_mapper/min_front_dist` | `std_msgs/Float32` | Minimum front LiDAR distance. |
| `/obstacle_mapper/status` | `std_msgs/String` | JSON obstacle status output. |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity command sent by the route executor. |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan input. |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy-grid map. |

## Mission Flow

The mission is coordinated by `mission_manager_node` using a finite-state machine. The typical mission sequence is:

```text
INIT
GETTING_STOP
PLANNING
EXECUTING
DWELLING
GETTING_STOP
...
RETURNING_HOME
COMPLETE
```

If the path becomes blocked, the executor reports a blocked status. The mission manager then cancels the current execution and requests replanning. During replanning, the route planner may inject a temporary obstacle patch in front of the robot so that the new A* path avoids the blocked region.

## Replanning Behavior

The replanning logic is designed for temporary obstacles that appear during execution.

1. `obstacle_mapper_node` checks the front LiDAR sector.
2. If the minimum front distance is below the threshold, it publishes `front_blocked = True`.
3. `route_executor_node` determines whether the blockage affects the current path.
4. If the blockage persists long enough, the executor publishes `BLOCKED`.
5. `mission_manager_node` cancels the current path execution.
6. `route_planner_node` injects a temporary obstacle region in front of the robot.
7. A new A* path is generated to the same target.
8. The executor follows the new path.

The temporary obstacle is used only for the current planning cycle and is not saved permanently into the map.

## Notes for Real-Robot Operation

Before running the mission, check the following:

- The TurtleBot4 is powered on and connected.
- The saved map path in `tour_guide_launch.py` is correct.
- The robot has a valid initial pose estimate for AMCL.
- The `map -> base_footprint` transform is available.
- The `/scan` topic is publishing LiDAR data.
- The robot has enough free space around the initial pose.
- RViz is showing the map, robot pose, and scan correctly.

## Troubleshooting

### Planner reports `NO_MAP`

Make sure the map server is running and the map path in `tour_guide_launch.py` is correct. The custom nodes are delayed at startup to give map server and AMCL time to become active, but an incorrect map path can still prevent planning.

### Mission does not start

Check whether AMCL has a valid initial pose and whether TF can provide the transform from `map` to `base_footprint`.

Useful command:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

### Robot does not move

Check the executor status and velocity command topic.

```bash
ros2 topic echo /executor/status
ros2 topic echo /cmd_vel
```

Also confirm that the TurtleBot4 base expects `geometry_msgs/TwistStamped` on `/cmd_vel`.

### Obstacle detection does not trigger

Check the LiDAR input and obstacle mapper outputs.

```bash
ros2 topic echo /scan
ros2 topic echo /obstacle_mapper/front_blocked
ros2 topic echo /obstacle_mapper/min_front_dist
```

If the threshold is too small or too large, tune `FRONT_BLOCKED_DIST` in `config.py`.

### Robot stops too far from or too close to a landmark

Tune the arrival and execution parameters in `config.py` and `route_executor_node.py`, especially:

- `ARRIVAL_DIST_M`
- `ARRIVAL_LIDAR_DIST`
- `WAYPOINT_ACCEPT_DIST`
- `LIN_SPEED`
- `ANG_SPEED`
- `K_HEADING`

## License

Add the project license here.

## Authors

Labid Bin Bashar and Matthew Tran.


