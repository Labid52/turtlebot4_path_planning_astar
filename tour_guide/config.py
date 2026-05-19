#!/usr/bin/env python3
"""
config.py — Real robot version.
Landmark coordinates recorded from AMCL pose on 2026-04-29.
Map file: ~/map/my_map.yaml
These coordinates are stable for every run that uses this same map file.
"""
import math

# ---------------------------------------------------------------------------
# Landmark stops in MAP frame.
# x, y  : where the robot stands when visiting the landmark (recorded live).
# yaw   : robot heading at that position (converted from quaternion).
# color : RViz marker color (for stop_manager visualization).
#
# Names kept identical to simulation so all log messages, FSM transitions,
# and MARK_VISITED/SKIP commands remain consistent across the codebase.
# ---------------------------------------------------------------------------
LANDMARKS = [
    {
        'name':  'Library',
        'x':     -1.20,
        'y':      0.5,
        'yaw':    1.57,          # 150.6 deg
        'color':  (0.20, 0.40, 1.00),   # blue
    },
    {
        'name':  'Cafeteria',
        'x':     -2.2,
        'y':     -0.3100,
        'yaw':    -1.57,          # 158.6 deg
        'color':  (1.00, 0.20, 0.20),   # red
    },
    {
        'name':  'Lab',
        'x':     -3.6,
        'y':     -0.0844,
        'yaw':   1.57,          # -164.4 deg
        'color':  (0.20, 0.90, 0.20),   # green
    },
    {
        'name':  'Office',
        'x':     -4.1507,
        'y':      1.125,
        'yaw':   1.57,          # -161.4 deg
        'color':  (1.00, 1.00, 0.10),   # yellow
    },
    {
        'name':  'Reception',
        'x':     -4.8,
        'y':      -0.3100,
        'yaw':   -1.57,          # -86.2 deg
        'color':  (0.80, 0.20, 0.80),   # purple
    },
]

# Home pose — where the robot returns after completing all stops.
HOME_POSE = {
    'x':   -0.1325,
    'y':   -0.0257,
    'yaw':  0.1521,                # 8.7 deg
}

# ---------------------------------------------------------------------------
# Mission FSM parameters
# ---------------------------------------------------------------------------
DWELL_TIME_S        = 5.0
MAX_REPLAN_ATTEMPTS = 3
PLAN_TIMEOUT_S      = 15.0
EXEC_TIMEOUT_S      = 120.0
SLAM_WARMUP_S       = 5.0
STOP_TIMEOUT_S      = 5.0

# ---------------------------------------------------------------------------
# Route planner (A*) parameters
# ---------------------------------------------------------------------------
INFLATION_RADIUS_M  = 0.40
PATH_STEP_M         = 0.10

# ---------------------------------------------------------------------------
# Route executor parameters
# ---------------------------------------------------------------------------
ARRIVAL_DIST_M      = 0.35
ARRIVAL_LIDAR_DIST  = 0.32
BLOCKED_PERSIST_S   = 5.0

# ---------------------------------------------------------------------------
# Obstacle mapper parameters
# ---------------------------------------------------------------------------
FRONT_BLOCKED_DIST  = 0.30
LIDAR_MOUNT_OFFSET  = math.pi / 2
FRONT_HALF_DEG      = 90.0
