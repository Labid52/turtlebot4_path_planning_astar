#!/usr/bin/env python3
"""
obstacle_mapper_node.py
=======================
Monitors the LiDAR and publishes whether the front path is blocked.

The LiDAR on TurtleBot4 mounts with its 0-degree beam pointing to the
robot's RIGHT.  Adding pi/2 to every beam angle rotates the frame so
that 0 deg = robot forward, matching the ROS convention.

Publishes (10 Hz):
  /obstacle_mapper/front_blocked  (std_msgs/Bool)     True when min dist < FRONT_BLOCKED_DIST
  /obstacle_mapper/min_front_dist (std_msgs/Float32)  minimum range in front cone (metres)
  /obstacle_mapper/status         (std_msgs/String)   JSON debug dump (throttled to 2 Hz)
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from tour_guide.config import (
    FRONT_BLOCKED_DIST,
    FRONT_HALF_DEG,
    LIDAR_MOUNT_OFFSET,
)


class ObstacleMapperNode(Node):

    def __init__(self):
        super().__init__('obstacle_mapper_node')

        # Publishers
        self._pub_blocked = self.create_publisher(Bool,    '/obstacle_mapper/front_blocked',   10)
        self._pub_dist    = self.create_publisher(Float32, '/obstacle_mapper/min_front_dist',  10)
        self._pub_status  = self.create_publisher(String,  '/obstacle_mapper/status',          10)

        # Subscriber
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        # State
        self._first_scan       = True
        self._last_status_time = 0.0     # wall-clock time of last JSON publish

        self.get_logger().info(
            '[ObstacleMapper] Node started. '
            f'front_half={FRONT_HALF_DEG} deg  '
            f'blocked_dist={FRONT_BLOCKED_DIST} m  '
            f'mount_offset=+{math.degrees(LIDAR_MOUNT_OFFSET):.0f} deg'
        )

    # ------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        if self._first_scan:
            self._first_scan = False
            self.get_logger().info(
                f'[ObstacleMapper] First scan: '
                f'beams={len(msg.ranges)}  '
                f'angle=[{math.degrees(msg.angle_min):.1f}, '
                f'{math.degrees(msg.angle_max):.1f}] deg  '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}] m'
            )

        front_half_rad = math.radians(FRONT_HALF_DEG)
        front_ranges = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if not (msg.range_min <= r <= msg.range_max):
                continue
            # Rotate +90 deg to align LiDAR frame with robot forward
            a = msg.angle_min + i * msg.angle_increment + LIDAR_MOUNT_OFFSET
            # Normalise to (-pi, pi]
            while a >  math.pi: a -= 2.0 * math.pi
            while a < -math.pi: a += 2.0 * math.pi
            if abs(a) <= front_half_rad:
                front_ranges.append(r)

        min_dist = min(front_ranges) if front_ranges else float('inf')
        blocked  = min_dist < FRONT_BLOCKED_DIST

        # Publish blocked flag
        b = Bool()
        b.data = blocked
        self._pub_blocked.publish(b)

        # Publish distance
        d = Float32()
        d.data = float(min_dist) if not math.isinf(min_dist) else 999.0
        self._pub_dist.publish(d)

        # Throttled JSON status (2 Hz)
        now = time.monotonic()
        if now - self._last_status_time >= 0.5:
            self._last_status_time = now
            status = {
                'blocked':        blocked,
                'min_front_dist': round(min_dist, 3) if not math.isinf(min_dist) else None,
                'front_beams':    len(front_ranges),
            }
            s = String()
            s.data = json.dumps(status)
            self._pub_status.publish(s)
            if blocked:
                self.get_logger().warn(
                    f'[ObstacleMapper] BLOCKED  min_front={min_dist:.3f} m '
                    f'(threshold={FRONT_BLOCKED_DIST} m)  beams={len(front_ranges)}'
                )


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
