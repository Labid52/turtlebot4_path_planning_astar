# tour_guide — TurtleBot4 Autonomous Tour-Guide Simulation

ROS 2 Jazzy + Gazebo Harmonic

## Quick Start

```bash
# 1. Place this package in your workspace
cd ~/ros2_ws/src
# (copy or clone tour_guide/ here)

# 2. Build
cd ~/ros2_ws
colcon build --packages-select tour_guide
source install/setup.bash

# 3. Launch everything
ros2 launch tour_guide tour_guide_launch.py
```

---

## What Happens

1. Gazebo opens with a 10 m × 8 m room containing **5 coloured landmark boxes**.
2. SLAM Toolbox builds the map as the robot boots.
3. After a 10-second warm-up the **mission** begins automatically:
   - Library (blue) → Cafeteria (red) → Lab (green) → Office (yellow) → Reception (purple)
   - 5-second dwell at each landmark
   - Returns to start when all stops are done
4. RViz shows the live map, costmaps, planned path, and landmark markers.

If a path is blocked the robot replans up to **3 times**, then skips that stop.

---

## Package Architecture

```
tour_guide/
├── tour_guide/
│   ├── config.py               ← ALL tunable parameters + landmark definitions
│   ├── mission_manager_node.py ← top-level FSM
│   ├── stop_manager_node.py    ← landmark list + visited state
│   ├── route_planner_node.py   ← custom A* on SLAM occupancy grid
│   ├── route_executor_node.py  ← Nav2 FollowPath client (swappable)
│   └── obstacle_mapper_node.py ← LiDAR front-obstacle monitor
├── worlds/tour_world.sdf
├── launch/tour_guide_launch.py
└── config/nav2_params.yaml
```

### Node communication (standard message types only)

| From               | Topic                         | To                  | Type              |
|--------------------|-------------------------------|---------------------|-------------------|
| MissionManager     | /stop_manager/cmd             | StopManager         | String            |
| StopManager        | /stop_manager/next_stop       | MissionManager      | PoseStamped       |
| StopManager        | /stop_manager/next_stop_name  | MissionManager      | String            |
| MissionManager     | /planner/goal                 | RoutePlanner        | PoseStamped       |
| RoutePlanner       | /planner/path                 | MissionManager      | Path              |
| RoutePlanner       | /planner/status               | MissionManager      | String            |
| MissionManager     | /executor/path                | RouteExecutor       | Path              |
| MissionManager     | /executor/cancel              | RouteExecutor       | Bool              |
| RouteExecutor      | /executor/status              | MissionManager      | String            |
| ObstacleMapper     | /obstacle_mapper/front_blocked| RouteExecutor       | Bool              |
| ObstacleMapper     | /obstacle_mapper/min_front_dist| RouteExecutor      | Float32           |

---

## Tuning Landmark Positions

Edit **`tour_guide/config.py`** — the `LANDMARKS` list.

Each entry:
```python
{
    'name':    'Library',
    'x':       1.0,     # navigation goal in MAP frame (metres)
    'y':       4.8,
    'yaw':     math.pi / 2,   # approach heading
    'world_x': 2.0,     # Gazebo world reference (not used at runtime)
    'world_y': 6.5,
    'color':   (0.2, 0.4, 1.0),   # RViz marker colour (R, G, B)
}
```

**How to verify coordinates:**
1. Launch the simulation and wait for the map to appear in RViz.
2. Open `/stop_manager/markers` in RViz → **MarkerArray**.
3. Cylinders should appear just in front of each coloured box.
4. If they are offset, adjust `x` / `y` in `config.py`, rebuild, and relaunch.

---

## Swapping the Executor (Custom Controller)

In `route_executor_node.py`, find the section marked:

```
── NAV2 FOLLOW-PATH SECTION ──
```

Delete the `_send_follow_path()`, `_fp_goal_response()`, `_fp_feedback()`,
`_fp_result()`, and `_cancel_follow_path()` methods, and replace them with
your own motion controller that publishes to `/cmd_vel` (Twist for simulation,
TwistStamped for the physical robot).

The rest of the node (arrival detection, block detection, status publishing)
stays exactly the same.

---

## Useful Debug Topics

```bash
# Mission FSM state (JSON)
ros2 topic echo /mission/state

# Planner status: IDLE / PLANNING / SUCCESS / FAILED / NO_MAP
ros2 topic echo /planner/status

# Executor status
ros2 topic echo /executor/status

# Obstacle mapper
ros2 topic echo /obstacle_mapper/front_blocked
ros2 topic echo /obstacle_mapper/min_front_dist

# Stop manager state (JSON)
ros2 topic echo /stop_manager/state

# A* inflated grid (add as OccupancyGrid in RViz)
/planner/debug_grid
```

---

## Common Issues

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Robot stuck in INIT | SLAM slow to start | Increase `SLAM_WARMUP_S` in config.py |
| Planner always FAILED | Landmarks inside inflated obstacles | Adjust `x`/`y` in config.py; decrease `INFLATION_RADIUS_M` |
| `follow_path` server not found | Nav2 not started | Check tb4_simulation_launch.py is running; wait longer |
| Markers far from boxes in RViz | Map/world offset | Fine-tune landmark coords after SLAM builds the map |
| `params_file` launch error | tb4_simulation_launch version mismatch | Comment out the `params_file` line in tour_guide_launch.py |
