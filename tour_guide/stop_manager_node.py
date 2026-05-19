#!/usr/bin/env python3
"""
stop_manager_node.py
====================

Fancy stop manager for TurtleBot4 tour-guide mission.

Instead of blindly returning the next stop in the LANDMARKS list,
this version chooses the next unvisited/unskipped stop using A* path cost.

Important:
- This node does NOT replace route_planner_node.
- This node only chooses WHICH stop should be visited next.
- route_planner_node still computes the actual path to that stop.
- This node reuses the same AStarPlanner class from route_planner_node.py
  so the stop-selection cost matches the real planner logic.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import tf2_ros
from tf2_ros import TransformException

from tour_guide.config import LANDMARKS, INFLATION_RADIUS_M
from tour_guide.route_planner_node import AStarPlanner


def _yaw_to_quat(yaw: float):
    """Yaw (rad) → quaternion tuple (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class StopManagerNode(Node):

    def __init__(self):
        super().__init__('stop_manager_node')

        # Internal stop list — copy of LANDMARKS with visit/skip state added
        self._stops = [dict(lm, visited=False, skipped=False) for lm in LANDMARKS]

        # Map state for A* route-cost selection
        self._map: OccupancyGrid = None
        self._last_astar_costs = {}

        names = ', '.join(s['name'] for s in self._stops)
        self.get_logger().info(
            f'[StopManager] Loaded {len(self._stops)} stops: {names}'
        )

        # Publishers
        self._pub_next      = self.create_publisher(PoseStamped,  '/stop_manager/next_stop',      10)
        self._pub_next_name = self.create_publisher(String,        '/stop_manager/next_stop_name', 10)
        self._pub_state     = self.create_publisher(String,        '/stop_manager/state',          10)
        self._pub_markers   = self.create_publisher(MarkerArray,   '/stop_manager/markers',        10)

        # Subscribe to command topic
        self.create_subscription(String, '/stop_manager/cmd', self._cmd_cb, 10)

        # Subscribe to map with same style as route_planner_node
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        # TF for current robot pose
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # Timers
        self.create_timer(1.0, self._publish_state)
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            '[StopManager] Ready. A*-cost stop selection enabled.'
        )

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        first = self._map is None
        self._map = msg

        if first:
            self.get_logger().info(
                f'[StopManager] Map received for stop selection: '
                f'{msg.info.width}×{msg.info.height} cells, '
                f'res={msg.info.resolution:.3f} m/cell'
            )

    # ------------------------------------------------------------------
    # Command handler
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f'[StopManager] CMD received: "{cmd}"')

        if cmd == 'GET_NEXT':
            self._send_next_astar()
        elif cmd.startswith('MARK_VISITED:'):
            self._mark(cmd[len('MARK_VISITED:'):], visited=True)
        elif cmd.startswith('SKIP:'):
            self._mark(cmd[len('SKIP:'):], skipped=True)
        elif cmd == 'RESET':
            for s in self._stops:
                s['visited'] = False
                s['skipped'] = False
            self._last_astar_costs = {}
            self.get_logger().info('[StopManager] All stops RESET.')
        else:
            self.get_logger().warn(f'[StopManager] Unknown command: "{cmd}"')

    # ------------------------------------------------------------------
    # New fancy A*-based stop selection
    # ------------------------------------------------------------------

    def _send_next_astar(self):
        """
        Select next stop using A* path cost.

        Logic:
        1. Find all unvisited and non-skipped stops.
        2. If no stops remain, publish MAP_DONE.
        3. Get current robot pose from TF.
        4. Run A* from robot pose to each remaining stop.
        5. Select stop with shortest valid A* path length.
        6. Publish selected stop.
        """

        remaining = [
            s for s in self._stops
            if not s['visited'] and not s['skipped']
        ]

        if not remaining:
            self.get_logger().info(
                '[StopManager] No more stops — publishing MAP_DONE sentinel.'
            )
            sentinel = PoseStamped()
            sentinel.header.stamp    = self.get_clock().now().to_msg()
            sentinel.header.frame_id = 'MAP_DONE'
            self._pub_next.publish(sentinel)

            n = String()
            n.data = 'MAP_DONE'
            self._pub_next_name.publish(n)
            return

        # Fallback if map is not ready
        if self._map is None:
            self.get_logger().warn(
                '[StopManager] Map not ready. Falling back to first remaining stop.'
            )
            self._publish_stop(remaining[0], reason='fallback_no_map')
            return

        robot_xy = self._get_robot_xy()

        # Fallback if TF is not ready
        if robot_xy is None:
            self.get_logger().warn(
                '[StopManager] Robot pose unavailable. Falling back to first remaining stop.'
            )
            self._publish_stop(remaining[0], reason='fallback_no_tf')
            return

        planner = AStarPlanner(
            grid_data=list(self._map.data),
            width=self._map.info.width,
            height=self._map.info.height,
            resolution=self._map.info.resolution,
            origin_x=self._map.info.origin.position.x,
            origin_y=self._map.info.origin.position.y,
            inflation_radius_m=INFLATION_RADIUS_M,
        )

        best_stop = None
        best_cost = float('inf')
        debug_costs = {}

        for stop in remaining:
            goal_xy = (stop['x'], stop['y'])

            world_path, reason = planner.plan(robot_xy, goal_xy)

            if world_path is None:
                debug_costs[stop['name']] = {
                    'reachable': False,
                    'reason': reason,
                    'cost_m': None,
                }
                self.get_logger().warn(
                    f'[StopManager] Stop {stop["name"]}: unreachable by A* ({reason})'
                )
                continue

            cost = self._path_length(world_path)

            debug_costs[stop['name']] = {
                'reachable': True,
                'reason': reason,
                'cost_m': round(cost, 3),
            }

            self.get_logger().info(
                f'[StopManager] Stop {stop["name"]}: A* cost = {cost:.2f} m'
            )

            if cost < best_cost:
                best_cost = cost
                best_stop = stop

        self._last_astar_costs = debug_costs

        # If no stop was reachable, use old behavior as fallback
        if best_stop is None:
            self.get_logger().error(
                '[StopManager] No reachable stop found by A*. '
                'Falling back to first remaining stop.'
            )
            self._publish_stop(remaining[0], reason='fallback_no_reachable_stop')
            return

        self.get_logger().info(
            f'[StopManager] A* selected next stop → {best_stop["name"]}, '
            f'cost={best_cost:.2f} m'
        )

        self._publish_stop(best_stop, reason='astar_shortest_path')

    def _get_robot_xy(self):
        """Return robot x, y in map frame using TF."""
        try:
            tf = self._tf_buf.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except TransformException as e:
            self.get_logger().warn(f'[StopManager] TF lookup failed: {e}')
            return None

    @staticmethod
    def _path_length(points):
        """Compute length of a world-coordinate polyline."""
        if points is None or len(points) < 2:
            return 0.0

        total = 0.0
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            total += math.hypot(x1 - x0, y1 - y0)

        return total

    # ------------------------------------------------------------------
    # Stop publishing and marking
    # ------------------------------------------------------------------

    def _publish_stop(self, stop, reason='unknown'):
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = stop['x']
        pose.pose.position.y = stop['y']
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = _yaw_to_quat(stop['yaw'])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self._pub_next.publish(pose)

        n = String()
        n.data = stop['name']
        self._pub_next_name.publish(n)

        self.get_logger().info(
            f'[StopManager] Next stop → {stop["name"]} '
            f'map({stop["x"]:.2f}, {stop["y"]:.2f}) '
            f'yaw={math.degrees(stop["yaw"]):.0f} deg '
            f'reason={reason}'
        )

    def _mark(self, name: str, visited=False, skipped=False):
        for s in self._stops:
            if s['name'] == name:
                if visited:
                    s['visited'] = True
                    self.get_logger().info(f'[StopManager] {name} → VISITED')
                if skipped:
                    s['skipped'] = True
                    self.get_logger().warn(f'[StopManager] {name} → SKIPPED')
                return

        self.get_logger().warn(
            f'[StopManager] Stop "{name}" not found for marking.'
        )

    # ------------------------------------------------------------------
    # Periodic publishers
    # ------------------------------------------------------------------

    def _publish_state(self):
        state = {
            'map_ready': self._map is not None,
            'selection_mode': 'astar_shortest_reachable_stop',
            'stops': [
                {
                    'name': s['name'],
                    'visited': s['visited'],
                    'skipped': s['skipped'],
                    'x': s['x'],
                    'y': s['y'],
                    'yaw': s['yaw'],
                }
                for s in self._stops
            ],
            'last_astar_costs': self._last_astar_costs,
        }

        m = String()
        m.data = json.dumps(state)
        self._pub_state.publish(m)

    def _publish_markers(self):
        arr  = MarkerArray()
        now  = self.get_clock().now().to_msg()
        life = Duration(sec=2)

        for i, s in enumerate(self._stops):
            r, g, b = s['color']

            # Cylinder marker at navigation goal
            box = Marker()
            box.header.frame_id = 'map'
            box.header.stamp    = now
            box.ns              = 'landmark_goals'
            box.id              = i
            box.type            = Marker.CYLINDER
            box.action          = Marker.ADD
            box.pose.position.x = s['x']
            box.pose.position.y = s['y']
            box.pose.position.z = 0.5
            box.pose.orientation.w = 1.0
            box.scale.x = 0.3
            box.scale.y = 0.3
            box.scale.z = 1.0
            box.color.r = float(r)
            box.color.g = float(g)
            box.color.b = float(b)
            box.color.a = 0.35 if s['visited'] else 0.9
            box.lifetime = life
            arr.markers.append(box)

            # Arrow showing approach direction
            arr_m = Marker()
            arr_m.header.frame_id = 'map'
            arr_m.header.stamp    = now
            arr_m.ns              = 'landmark_arrows'
            arr_m.id              = i + 50
            arr_m.type            = Marker.ARROW
            arr_m.action          = Marker.ADD
            arr_m.pose.position.x = s['x']
            arr_m.pose.position.y = s['y']
            arr_m.pose.position.z = 0.1

            qx, qy, qz, qw = _yaw_to_quat(s['yaw'])
            arr_m.pose.orientation.x = qx
            arr_m.pose.orientation.y = qy
            arr_m.pose.orientation.z = qz
            arr_m.pose.orientation.w = qw

            arr_m.scale.x = 0.5
            arr_m.scale.y = 0.08
            arr_m.scale.z = 0.08
            arr_m.color.r = float(r)
            arr_m.color.g = float(g)
            arr_m.color.b = float(b)
            arr_m.color.a = 0.9
            arr_m.lifetime = life
            arr.markers.append(arr_m)

            # Text label
            lbl = Marker()
            lbl.header.frame_id = 'map'
            lbl.header.stamp    = now
            lbl.ns              = 'landmark_labels'
            lbl.id              = i + 100
            lbl.type            = Marker.TEXT_VIEW_FACING
            lbl.action          = Marker.ADD
            lbl.pose.position.x = s['x']
            lbl.pose.position.y = s['y']
            lbl.pose.position.z = 1.3
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.28
            lbl.color.r = 1.0
            lbl.color.g = 1.0
            lbl.color.b = 1.0
            lbl.color.a = 1.0

            icon = '✓' if s['visited'] else ('✗' if s['skipped'] else '○')
            lbl.text = f'{s["name"]} {icon}'
            lbl.lifetime = life
            arr.markers.append(lbl)

        self._pub_markers.publish(arr)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = StopManagerNode()

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
