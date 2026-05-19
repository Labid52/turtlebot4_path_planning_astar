#!/usr/bin/env python3
"""
route_planner_node.py
=====================

Custom A* path planner with temporary obstacle injection.

Original behavior:
- Subscribe to /map
- Subscribe to /planner/goal
- Get robot pose from TF: map -> base_footprint
- Run A* on inflated occupancy grid
- Publish /planner/path

New robust behavior:
- Also subscribe to obstacle mapper:
    /obstacle_mapper/front_blocked
    /obstacle_mapper/min_front_dist
- If the robot is currently blocked, or was blocked very recently,
  a temporary obstacle patch is injected in front of the robot before A*.
- This makes replanning avoid the newly detected obstacle without using SLAM.
- The saved map is not modified permanently.

If A* fails WITH the temp obstacle (narrow corridor), it retries on the
clean map so the robot always has a path to follow. The executor will stop
if the obstacle is still physically present.

This planner still performs GLOBAL A* from current robot pose to goal.
The temporary obstacle patch only changes the planning grid for the current
planning cycle.
"""

import heapq
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, Float32, String

import tf2_ros
from tf2_ros import TransformException

from tour_guide.config import (
    INFLATION_RADIUS_M,
    PATH_STEP_M,
    FRONT_BLOCKED_DIST,
)


# ---------------------------------------------------------------------------
# A* planner
# ---------------------------------------------------------------------------

class AStarPlanner:
    """8-directional A* on a 2-D occupancy grid with circular obstacle inflation."""

    OCCUPIED_THRESH = 65

    def __init__(
        self,
        grid_data,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        inflation_radius_m: float = 0.35,
    ):
        self._W = width
        self._H = height
        self._res = resolution
        self._ox = origin_x
        self._oy = origin_y

        raw = np.array(grid_data, dtype=np.int16).reshape((height, width))

        # Occupied cells become obstacles.
        # Unknown cells (-1) are treated as free, consistent with the older planner.
        obstacle = raw >= self.OCCUPIED_THRESH

        radius_cells = max(1, int(math.ceil(inflation_radius_m / resolution)))
        self._inflated = self._inflate(obstacle, radius_cells)

    # ------------------------------------------------------------------

    def _inflate(self, obstacle: np.ndarray, radius: int) -> np.ndarray:
        """
        Circular obstacle inflation using numpy roll.
        No scipy dependency.
        """
        result = obstacle.copy()

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if dr * dr + dc * dc > radius * radius:
                    continue

                shifted = np.roll(np.roll(obstacle, dr, axis=0), dc, axis=1)

                # Remove wraparound caused by np.roll.
                if dr > 0:
                    shifted[:dr, :] = False
                elif dr < 0:
                    shifted[dr:, :] = False

                if dc > 0:
                    shifted[:, :dc] = False
                elif dc < 0:
                    shifted[:, dc:] = False

                result |= shifted

        return result

    # ------------------------------------------------------------------

    def world_to_cell(self, wx: float, wy: float):
        col = int((wx - self._ox) / self._res)
        row = int((wy - self._oy) / self._res)

        if 0 <= row < self._H and 0 <= col < self._W:
            return row, col

        return None

    def cell_to_world(self, row: int, col: int):
        wx = self._ox + (col + 0.5) * self._res
        wy = self._oy + (row + 0.5) * self._res
        return wx, wy

    # ------------------------------------------------------------------

    def plan(self, start_world, goal_world):
        """
        Plan from start_world=(x,y) to goal_world=(x,y).
        Returns:
            world_path, reason
        where world_path is None on failure.
        """

        sc = self.world_to_cell(*start_world)
        gc = self.world_to_cell(*goal_world)

        if sc is None:
            return None, 'start_out_of_bounds'

        if gc is None:
            return None, 'goal_out_of_bounds'

        if self._inflated[sc]:
            sc = self._nearest_free(sc)
            if sc is None:
                return None, 'start_in_obstacle_no_free_nearby'

        if self._inflated[gc]:
            gc = self._nearest_free(gc)
            if gc is None:
                return None, 'goal_in_obstacle_no_free_nearby'

        cells = self._astar(sc, gc)

        if cells is None:
            return None, 'no_path_found'

        cells = self._prune(cells)

        world = [self.cell_to_world(r, c) for r, c in cells]
        return world, 'success'

    # ------------------------------------------------------------------

    def _nearest_free(self, cell, max_radius=25):
        """BFS outward from a blocked cell to find nearest free cell."""
        from collections import deque

        q = deque([cell])
        seen = {cell}
        start_r, start_c = cell

        while q:
            r, c = q.popleft()

            if not self._inflated[r, c]:
                return r, c

            for dr, dc in (
                (-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1),
            ):
                nr = r + dr
                nc = c + dc

                if not (0 <= nr < self._H and 0 <= nc < self._W):
                    continue

                if (nr, nc) in seen:
                    continue

                if abs(nr - start_r) + abs(nc - start_c) > max_radius:
                    continue

                seen.add((nr, nc))
                q.append((nr, nc))

        return None

    # ------------------------------------------------------------------

    def _astar(self, start, goal):
        """8-connected A* search."""

        moves = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]

        sqrt2 = math.sqrt(2.0)
        gr, gc = goal

        def h(r, c):
            return math.hypot(r - gr, c - gc)

        heap = [(h(*start), 0.0, start, None)]
        came = {}
        g_best = {start: 0.0}

        while heap:
            f, g, cur, parent = heapq.heappop(heap)

            if cur in came:
                continue

            came[cur] = parent

            if cur == goal:
                path = []
                node = cur

                while node is not None:
                    path.append(node)
                    node = came[node]

                path.reverse()
                return path

            r, c = cur

            for dr, dc in moves:
                nr = r + dr
                nc = c + dc

                if not (0 <= nr < self._H and 0 <= nc < self._W):
                    continue

                if self._inflated[nr, nc]:
                    continue

                neighbor = (nr, nc)

                if neighbor in came:
                    continue

                step = sqrt2 if dr != 0 and dc != 0 else 1.0
                ng = g + step

                if ng < g_best.get(neighbor, float('inf')):
                    g_best[neighbor] = ng
                    heapq.heappush(
                        heap,
                        (ng + h(nr, nc), ng, neighbor, cur),
                    )

        return None

    # ------------------------------------------------------------------

    def _prune(self, cells):
        """
        Remove unnecessary intermediate waypoints if direct line-of-sight exists.
        """
        if len(cells) <= 2:
            return cells

        result = [cells[0]]
        i = 0

        while i < len(cells) - 1:
            j = i + 2

            while j < len(cells) and self._los_clear(cells[i], cells[j]):
                j += 1

            result.append(cells[j - 1])
            i = j - 1

        return result

    def _los_clear(self, a, b) -> bool:
        """Bresenham line-of-sight check on inflated obstacle grid."""

        r0, c0 = a
        r1, c1 = b

        dr = abs(r1 - r0)
        dc = abs(c1 - c0)

        sr = 1 if r1 >= r0 else -1
        sc = 1 if c1 >= c0 else -1

        r = r0
        c = c0

        if dc > dr:
            err = dc // 2

            while c != c1:
                if self._inflated[r, c]:
                    return False

                err -= dr

                if err < 0:
                    r += sr
                    err += dc

                c += sc

        else:
            err = dr // 2

            while r != r1:
                if self._inflated[r, c]:
                    return False

                err -= dc

                if err < 0:
                    c += sc
                    err += dr

                r += sr

        return not self._inflated[r1, c1]

    # ------------------------------------------------------------------

    def get_debug_grid_msg(self, frame_id: str, stamp):
        """Return inflated obstacle grid for RViz."""
        msg = OccupancyGrid()

        msg.header.frame_id = frame_id
        msg.header.stamp = stamp

        msg.info.resolution = self._res
        msg.info.width = self._W
        msg.info.height = self._H
        msg.info.origin.position.x = self._ox
        msg.info.origin.position.y = self._oy
        msg.info.origin.orientation.w = 1.0

        msg.data = (self._inflated.flatten().astype(np.int8) * 100).tolist()

        return msg


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class RoutePlannerNode(Node):

    # Temporary obstacle settings.
    # These are not permanent map changes.
    TEMP_OBSTACLE_RADIUS_M = 0.15   # small enough to not fill narrow corridors

    # Keep using the temporary obstacle for a short time after the blocked flag
    # disappears to avoid race conditions during replan.
    TEMP_OBSTACLE_MEMORY_S = 3.0

    # Clamp the estimated obstacle distance.
    TEMP_OBSTACLE_MIN_DIST_M = 0.35
    TEMP_OBSTACLE_MAX_DIST_M = 1.20

    def __init__(self):
        super().__init__('route_planner_node')

        # ------------------------------------------------------------------
        # Map/planner state
        # ------------------------------------------------------------------
        self._map: OccupancyGrid = None
        self._status = 'IDLE'

        # ------------------------------------------------------------------
        # Obstacle mapper state
        # ------------------------------------------------------------------
        self._front_blocked = False
        self._front_dist = 999.0
        self._last_front_blocked_time = None

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_path = self.create_publisher(
            Path,
            '/planner/path',
            10,
        )

        self._pub_status = self.create_publisher(
            String,
            '/planner/status',
            10,
        )

        self._pub_debug = self.create_publisher(
            OccupancyGrid,
            '/planner/debug_grid',
            10,
        )

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_cb,
            map_qos,
        )

        self.create_subscription(
            PoseStamped,
            '/planner/goal',
            self._goal_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/obstacle_mapper/front_blocked',
            self._front_blocked_cb,
            10,
        )

        self.create_subscription(
            Float32,
            '/obstacle_mapper/min_front_dist',
            self._front_dist_cb,
            10,
        )

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ------------------------------------------------------------------
        # Status timer
        # ------------------------------------------------------------------
        self.create_timer(0.2, self._pub_status_tick)

        self.get_logger().info(
            f'[RoutePlanner] Node started with temporary obstacle injection. '
            f'inflation={INFLATION_RADIUS_M} m, '
            f'path_step={PATH_STEP_M} m, '
            f'temp_obs_radius={self.TEMP_OBSTACLE_RADIUS_M} m'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        first = self._map is None
        self._map = msg

        if first:
            self.get_logger().info(
                f'[RoutePlanner] Map received: '
                f'{msg.info.width}×{msg.info.height} cells, '
                f'res={msg.info.resolution:.3f} m/cell, '
                f'origin=({msg.info.origin.position.x:.2f}, '
                f'{msg.info.origin.position.y:.2f})'
            )

            if self._status == 'NO_MAP':
                self._status = 'IDLE'

    def _goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        self.get_logger().info(
            f'[RoutePlanner] Goal received: '
            f'map({gx:.2f}, {gy:.2f}), '
            f'yaw={math.degrees(self._quat_to_yaw(msg.pose.orientation)):.0f} deg'
        )

        self._plan(msg)

    def _front_blocked_cb(self, msg: Bool):
        self._front_blocked = msg.data

        if msg.data:
            self._last_front_blocked_time = time.monotonic()

    def _front_dist_cb(self, msg: Float32):
        self._front_dist = msg.data

    # ------------------------------------------------------------------
    # Pose and planning
    # ------------------------------------------------------------------

    def _get_robot_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = self._quat_to_yaw(q)

            return t.x, t.y, yaw

        except TransformException as e:
            self.get_logger().warn(f'[RoutePlanner] TF lookup failed: {e}')
            return None

    # ------------------------------------------------------------------

    def _plan(self, goal_msg: PoseStamped):
        if self._map is None:
            self.get_logger().error(
                '[RoutePlanner] Cannot plan: no map received.'
            )
            self._status = 'NO_MAP'
            return

        robot_pose = self._get_robot_pose()

        if robot_pose is None:
            self.get_logger().error(
                '[RoutePlanner] Cannot plan: robot pose unavailable.'
            )
            self._status = 'NO_POSE'
            return

        robot_x, robot_y, robot_yaw = robot_pose
        robot_xy = (robot_x, robot_y)

        goal_xy = (
            goal_msg.pose.position.x,
            goal_msg.pose.position.y,
        )

        self._status = 'PLANNING'

        self.get_logger().info(
            f'[RoutePlanner] A* request: '
            f'start=({robot_x:.2f}, {robot_y:.2f}, '
            f'{math.degrees(robot_yaw):.1f} deg), '
            f'goal=({goal_xy[0]:.2f}, {goal_xy[1]:.2f})'
        )

        t0 = time.monotonic()

        # ------------------------------------------------------------------
        # First attempt: plan WITH temporary obstacle injection
        # ------------------------------------------------------------------
        planning_data = list(self._map.data)

        injected = self._inject_temporary_obstacle_if_needed(
            planning_data=planning_data,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            goal_xy=goal_xy,
        )

        planner = AStarPlanner(
            grid_data=planning_data,
            width=self._map.info.width,
            height=self._map.info.height,
            resolution=self._map.info.resolution,
            origin_x=self._map.info.origin.position.x,
            origin_y=self._map.info.origin.position.y,
            inflation_radius_m=INFLATION_RADIUS_M,
        )

        self._pub_debug.publish(
            planner.get_debug_grid_msg('map', self.get_clock().now().to_msg())
        )

        world_path, reason = planner.plan(robot_xy, goal_xy)

        # ------------------------------------------------------------------
        # If A* failed WITH temp obstacle, retry on the clean map.
        #
        # Why: in a narrow corridor the temp obstacle bubble can fill the
        # entire corridor width and leave no path. Rather than looping in
        # PLANNING→FAILED→RECOVERY forever, give the executor a clean path.
        # The executor will stop itself when it physically hits the obstacle,
        # and the robot continues as soon as the person steps aside.
        # ------------------------------------------------------------------
        if world_path is None and injected:
            self.get_logger().warn(
                f'[RoutePlanner] A* failed with temp obstacle ({reason}). '
                'Retrying with reduced inflation to route around obstacle.'
            )
            planner_squeeze = AStarPlanner(
                grid_data=planning_data,
                width=self._map.info.width,
                height=self._map.info.height,
                resolution=self._map.info.resolution,
                origin_x=self._map.info.origin.position.x,
                origin_y=self._map.info.origin.position.y,
                inflation_radius_m=INFLATION_RADIUS_M * 0.5,
            )
            world_path, reason = planner_squeeze.plan(robot_xy, goal_xy)

            if world_path is not None:
                self.get_logger().info(
                    '[RoutePlanner] Reduced-inflation retry SUCCESS — '
                    'path routes around obstacle.'
                )
            else:
                self.get_logger().warn(
                    '[RoutePlanner] Reduced-inflation retry also failed. '
                    'Falling back to clean map.'
                )
                planner_clean = AStarPlanner(
                    grid_data=list(self._map.data),
                    width=self._map.info.width,
                    height=self._map.info.height,
                    resolution=self._map.info.resolution,
                    origin_x=self._map.info.origin.position.x,
                    origin_y=self._map.info.origin.position.y,
                    inflation_radius_m=INFLATION_RADIUS_M,
                )
                world_path, reason = planner_clean.plan(robot_xy, goal_xy)
                if world_path is not None:
                    self.get_logger().info('[RoutePlanner] Clean-map fallback SUCCESS.')
                else:
                    self.get_logger().error(
                        f'[RoutePlanner] Clean-map fallback also FAILED: {reason}'
                    )
        dt = time.monotonic() - t0

        # ------------------------------------------------------------------
        # Both attempts failed
        # ------------------------------------------------------------------
        if world_path is None:
            self.get_logger().error(
                f'[RoutePlanner] A* FAILED: {reason}, '
                f'temp_obstacle_injected={injected}, '
                f'time={dt*1000:.0f} ms'
            )
            self._status = 'FAILED'
            return

        self.get_logger().info(
            f'[RoutePlanner] A* SUCCESS: '
            f'{len(world_path)} pruned waypoints, '
            f'temp_obstacle_injected={injected}, '
            f'time={dt*1000:.0f} ms'
        )

        world_path = self._interpolate(world_path, PATH_STEP_M)

        path_msg = self._build_path_msg(world_path, goal_msg)

        self._pub_path.publish(path_msg)

        total_len = self._path_length(path_msg)

        self.get_logger().info(
            f'[RoutePlanner] Path published: '
            f'{len(path_msg.poses)} waypoints, '
            f'length≈{total_len:.2f} m'
        )

        self._status = 'SUCCESS'

    # ------------------------------------------------------------------
    # Temporary obstacle injection
    # ------------------------------------------------------------------

    def _should_inject_temporary_obstacle(self) -> bool:
        if self._front_blocked:
            return True

        if self._last_front_blocked_time is None:
            return False

        age = time.monotonic() - self._last_front_blocked_time

        return age <= self.TEMP_OBSTACLE_MEMORY_S

    def _inject_temporary_obstacle_if_needed(
        self,
        planning_data,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        goal_xy,
    ) -> bool:
        if not self._should_inject_temporary_obstacle():
            return False

        info = self._map.info
        width = info.width
        height = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        obs_dist = self._front_dist

        if obs_dist <= 0.01 or obs_dist > 50.0 or math.isinf(obs_dist) or math.isnan(obs_dist):
            obs_dist = FRONT_BLOCKED_DIST

        obs_dist = max(
            self.TEMP_OBSTACLE_MIN_DIST_M,
            min(self.TEMP_OBSTACLE_MAX_DIST_M, obs_dist),
        )

        cx = robot_x + obs_dist * math.cos(robot_yaw)
        cy = robot_y + obs_dist * math.sin(robot_yaw)

        dist_patch_to_goal = math.hypot(cx - goal_xy[0], cy - goal_xy[1])
        if dist_patch_to_goal < self.TEMP_OBSTACLE_RADIUS_M + 0.25:
            self.get_logger().warn(
                '[RoutePlanner] Temporary obstacle near goal. '
                'Skipping injection to avoid blocking the target.'
            )
            return False

        center_col = int((cx - ox) / res)
        center_row = int((cy - oy) / res)
        radius_cells = max(1, int(math.ceil(self.TEMP_OBSTACLE_RADIUS_M / res)))

        if not (0 <= center_row < height and 0 <= center_col < width):
            self.get_logger().warn(
                '[RoutePlanner] Temporary obstacle center out of map bounds. '
                'Skipping injection.'
            )
            return False

        robot_col = int((robot_x - ox) / res)
        robot_row = int((robot_y - oy) / res)
        robot_clearance_cells = max(1, int(math.ceil(0.25 / res)))

        marked = 0

        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc > radius_cells * radius_cells:
                    continue

                rr = center_row + dr
                cc = center_col + dc

                if not (0 <= rr < height and 0 <= cc < width):
                    continue

                if (
                    abs(rr - robot_row) <= robot_clearance_cells
                    and abs(cc - robot_col) <= robot_clearance_cells
                ):
                    continue

                idx = rr * width + cc
                planning_data[idx] = 100
                marked += 1

        self.get_logger().warn(
            f'[RoutePlanner] Temporary obstacle injected: '
            f'center=({cx:.2f}, {cy:.2f}), '
            f'dist={obs_dist:.2f} m, '
            f'radius={self.TEMP_OBSTACLE_RADIUS_M:.2f} m, '
            f'cells_marked={marked}'
        )

        return marked > 0

    # ------------------------------------------------------------------
    # Path message helpers
    # ------------------------------------------------------------------

    def _build_path_msg(self, world_path, goal_msg: PoseStamped) -> Path:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for wx, wy in world_path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        n = len(path_msg.poses)

        for i in range(n - 1):
            p0 = path_msg.poses[i].pose.position
            p1 = path_msg.poses[i + 1].pose.position

            yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)

            qx, qy, qz, qw = self._yaw_to_quat(yaw)

            path_msg.poses[i].pose.orientation.x = qx
            path_msg.poses[i].pose.orientation.y = qy
            path_msg.poses[i].pose.orientation.z = qz
            path_msg.poses[i].pose.orientation.w = qw

        if path_msg.poses:
            path_msg.poses[-1].pose.orientation = goal_msg.pose.orientation

        return path_msg

    @staticmethod
    def _interpolate(points, step_m):
        if len(points) < 2:
            return list(points)

        result = [points[0]]
        dist_to_next = step_m

        for i in range(1, len(points)):
            x0, y0 = points[i - 1]
            x1, y1 = points[i]

            seg = math.hypot(x1 - x0, y1 - y0)

            if seg < 1e-9:
                continue

            walked = 0.0

            while walked + dist_to_next <= seg:
                walked += dist_to_next
                frac = walked / seg

                result.append(
                    (
                        x0 + frac * (x1 - x0),
                        y0 + frac * (y1 - y0),
                    )
                )

                dist_to_next = step_m

            dist_to_next -= (seg - walked)
            dist_to_next = max(dist_to_next, 1e-9)

        if result[-1] != points[-1]:
            result.append(points[-1])

        return result

    @staticmethod
    def _path_length(path_msg: Path) -> float:
        total = 0.0

        poses = path_msg.poses

        for i in range(len(poses) - 1):
            total += math.hypot(
                poses[i + 1].pose.position.x - poses[i].pose.position.x,
                poses[i + 1].pose.position.y - poses[i].pose.position.y,
            )

        return total

    # ------------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _yaw_to_quat(yaw: float):
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    @staticmethod
    def _quat_to_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _pub_status_tick(self):
        if self._map is None and self._status not in ('NO_MAP', 'NO_POSE'):
            self._status = 'NO_MAP'

        msg = String()
        msg.data = self._status
        self._pub_status.publish(msg)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RoutePlannerNode()

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
