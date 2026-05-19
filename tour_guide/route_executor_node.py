#!/usr/bin/env python3
"""
route_executor_node.py  (REAL ROBOT version — TwistStamped, NO APF)
===================================================================

This node follows the A* path published by the route planner.

Important design:
- APF has been removed completely.
- The executor does not bend or disturb the A* path.
- It follows waypoints using a proportional heading controller.
- If the front path is blocked, it does NOT blindly stop all motion.
- If the replanned path requires turning away from the obstacle, the robot
  is allowed to rotate in place.
- If the robot is already facing the next waypoint and would drive forward
  into the obstacle, it stops.
- If that true path blockage remains for BLOCKED_PERSIST_S, it reports BLOCKED.
- After reaching the goal position, it rotates to the final goal yaw before
  declaring ARRIVED_POSE.
- MissionManager then decides whether to dwell, replan, recover, skip, or finish.

Subscribes:
  /executor/path                       nav_msgs/Path
  /executor/cancel                     std_msgs/Bool
  /obstacle_mapper/front_blocked       std_msgs/Bool
  /obstacle_mapper/min_front_dist      std_msgs/Float32
  TF: map -> base_footprint

Publishes:
  /cmd_vel                             geometry_msgs/TwistStamped
  /executor/status                     std_msgs/String
  /executor/progress                   std_msgs/Float32
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, String

import tf2_ros
from tf2_ros import TransformException

from tour_guide.config import (
    ARRIVAL_DIST_M,
    ARRIVAL_LIDAR_DIST,
    BLOCKED_PERSIST_S,
)


class RouteExecutorNode(Node):

    # ------------------------------------------------------------------
    # Motion parameters
    # ------------------------------------------------------------------
    LIN_SPEED = 0.10       # m/s, safe demo speed
    ANG_SPEED = 0.80       # rad/s
    K_HEADING = 1.5        # heading error gain

    # ------------------------------------------------------------------
    # Waypoint tracking parameters
    # ------------------------------------------------------------------
    WAYPOINT_ACCEPT_DIST = 0.25
    CTRL_HZ = 20.0

    # If heading error is larger than this, rotate in place first.
    ROTATE_IN_PLACE_ERR = 0.65     # rad, about 37 deg

    # If heading error is small, allow normal forward motion.
    FULL_SPEED_HEADING_ERR = 0.20  # rad, about 11 deg

    # ------------------------------------------------------------------
    # Final orientation alignment
    # ------------------------------------------------------------------
    USE_FINAL_YAW_ALIGNMENT = True

    # Robot must be within this yaw error before ARRIVED_POSE is declared.
    FINAL_YAW_TOL = 0.20           # rad, about 11 deg

    # If yaw error is very tiny, stop instead of oscillating.
    FINAL_YAW_DEADBAND = 0.04      # rad, about 2.3 deg

    # ------------------------------------------------------------------
    # Debug log period
    # ------------------------------------------------------------------
    DEBUG_PERIOD_S = 1.0

    def __init__(self):
        super().__init__('route_executor_node')

        # ------------------------------------------------------------------
        # Path state
        # ------------------------------------------------------------------
        self._status = 'IDLE'
        self._current_path = None
        self._wp_index = 0
        self._goal_xy = None
        self._goal_yaw = None
        self._cancel_req = False

        # True after robot reaches goal position and starts final yaw alignment.
        self._aligning_final_yaw = False

        # ------------------------------------------------------------------
        # Robot pose from TF
        # ------------------------------------------------------------------
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._pose_ok = False

        # ------------------------------------------------------------------
        # Obstacle state from obstacle_mapper_node
        # ------------------------------------------------------------------
        self._front_blocked = False
        self._front_dist = 999.0
        self._blocked_since = None

        # ------------------------------------------------------------------
        # Progress/debug
        # ------------------------------------------------------------------
        self._path_total_len = 0.0
        self._last_debug_t = 0.0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_cmd = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10,
        )

        self._pub_status = self.create_publisher(
            String,
            '/executor/status',
            10,
        )

        self._pub_progress = self.create_publisher(
            Float32,
            '/executor/progress',
            10,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Path,
            '/executor/path',
            self._path_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/executor/cancel',
            self._cancel_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/obstacle_mapper/front_blocked',
            self._blocked_cb,
            10,
        )

        self.create_subscription(
            Float32,
            '/obstacle_mapper/min_front_dist',
            self._dist_cb,
            10,
        )

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.create_timer(1.0 / self.CTRL_HZ, self._control_loop)
        self.create_timer(0.2, self._status_tick)

        self.get_logger().info(
            '[RouteExecutor] REAL ROBOT executor started: NO APF. '
            f'LIN={self.LIN_SPEED} m/s, '
            f'ANG={self.ANG_SPEED} rad/s, '
            f'K_HEADING={self.K_HEADING}, '
            f'WP_ACCEPT={self.WAYPOINT_ACCEPT_DIST} m, '
            f'ARRIVAL={ARRIVAL_DIST_M} m, '
            f'FINAL_YAW_ALIGN={self.USE_FINAL_YAW_ALIGNMENT}, '
            f'FINAL_YAW_TOL={self.FINAL_YAW_TOL:.2f} rad, '
            f'BLOCKED_PERSIST={BLOCKED_PERSIST_S} s'
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _path_cb(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn('[RouteExecutor] Empty path received. Ignoring.')
            return

        self._current_path = msg
        self._wp_index = 0

        last_pose = msg.poses[-1].pose

        self._goal_xy = (
            last_pose.position.x,
            last_pose.position.y,
        )

        # Final yaw comes from the orientation of the final path pose.
        # IMPORTANT:
        # For this to face the landmark exactly, route_planner_node should set
        # path_msg.poses[-1].pose.orientation = goal_msg.pose.orientation.
        self._goal_yaw = self._quat_to_yaw(last_pose.orientation)

        self._cancel_req = False
        self._blocked_since = None
        self._aligning_final_yaw = False
        self._last_debug_t = 0.0
        self._path_total_len = self._compute_path_length(msg)

        self.get_logger().info(
            f'[RouteExecutor] New A* path received: '
            f'{len(msg.poses)} waypoints, '
            f'length={self._path_total_len:.2f} m, '
            f'goal=({self._goal_xy[0]:.2f}, {self._goal_xy[1]:.2f}), '
            f'goal_yaw={math.degrees(self._goal_yaw):.1f} deg'
        )

        self._set_status('EXECUTING')

    def _cancel_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('[RouteExecutor] Cancel requested.')
            self._cancel_req = True
            self._stop_robot()
            self._current_path = None
            self._goal_xy = None
            self._goal_yaw = None
            self._blocked_since = None
            self._aligning_final_yaw = False
            self._set_status('IDLE')

    def _blocked_cb(self, msg: Bool):
        self._front_blocked = msg.data

    def _dist_cb(self, msg: Float32):
        self._front_dist = msg.data

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if self._status != 'EXECUTING' or self._current_path is None:
            return

        if self._cancel_req:
            self._stop_robot()
            self._set_status('IDLE')
            return

        if not self._update_pose():
            self.get_logger().warn(
                '[RouteExecutor] TF unavailable. Holding position.',
                throttle_duration_sec=2.0,
            )
            self._stop_robot()
            return

        if self._goal_xy is None:
            self.get_logger().error('[RouteExecutor] No goal_xy while EXECUTING.')
            self._stop_robot()
            self._set_status('FAILED')
            return

        dist_to_goal = math.hypot(
            self._robot_x - self._goal_xy[0],
            self._robot_y - self._goal_xy[1],
        )

        # --------------------------------------------------------------
        # 1. Pose-based arrival with final yaw alignment
        # --------------------------------------------------------------
        if dist_to_goal < ARRIVAL_DIST_M:
            if self.USE_FINAL_YAW_ALIGNMENT and self._goal_yaw is not None:
                yaw_error = self._norm(self._goal_yaw - self._robot_yaw)

                if abs(yaw_error) > self.FINAL_YAW_TOL:
                    self._aligning_final_yaw = True

                    omega = self.K_HEADING * yaw_error
                    omega = max(-self.ANG_SPEED, min(self.ANG_SPEED, omega))

                    # Avoid tiny oscillatory commands.
                    if abs(yaw_error) < self.FINAL_YAW_DEADBAND:
                        omega = 0.0

                    cmd = TwistStamped()
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    cmd.header.frame_id = 'base_link'
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = omega
                    self._pub_cmd.publish(cmd)

                    self.get_logger().info(
                        f'[RouteExecutor] At goal position. Aligning final yaw: '
                        f'goal_yaw={math.degrees(self._goal_yaw):.1f} deg, '
                        f'robot_yaw={math.degrees(self._robot_yaw):.1f} deg, '
                        f'error={math.degrees(yaw_error):.1f} deg, '
                        f'omega={omega:.3f}',
                        throttle_duration_sec=1.0,
                    )
                    return

            self.get_logger().info(
                f'[RouteExecutor] ARRIVED_POSE: '
                f'dist_to_goal={dist_to_goal:.3f} m'
            )
            self._stop_robot()
            self._aligning_final_yaw = False
            self._set_status('ARRIVED_POSE')
            return

        self._aligning_final_yaw = False

        # --------------------------------------------------------------
        # 2. Advance waypoint if close enough
        # --------------------------------------------------------------
        self._advance_waypoint()

        poses = self._current_path.poses
        if self._wp_index >= len(poses):
            self._wp_index = len(poses) - 1

        target = poses[self._wp_index].pose.position
        dx = target.x - self._robot_x
        dy = target.y - self._robot_y

        target_yaw = math.atan2(dy, dx)
        heading_error = self._norm(target_yaw - self._robot_yaw)

        # --------------------------------------------------------------
        # 3. LiDAR-based arrival
        # Use this only when already close to goal and roughly facing target.
        # This avoids false arrival when an obstacle/person is in front.
        # --------------------------------------------------------------
        if (
            self._front_dist < ARRIVAL_LIDAR_DIST
            and dist_to_goal < ARRIVAL_DIST_M * 1.5
            and abs(heading_error) < 0.6
        ):
            if self.USE_FINAL_YAW_ALIGNMENT and self._goal_yaw is not None:
                yaw_error = self._norm(self._goal_yaw - self._robot_yaw)

                if abs(yaw_error) > self.FINAL_YAW_TOL:
                    omega = self.K_HEADING * yaw_error
                    omega = max(-self.ANG_SPEED, min(self.ANG_SPEED, omega))

                    cmd = TwistStamped()
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    cmd.header.frame_id = 'base_link'
                    cmd.twist.linear.x = 0.0
                    cmd.twist.angular.z = omega
                    self._pub_cmd.publish(cmd)

                    self.get_logger().info(
                        f'[RouteExecutor] LiDAR-arrival zone reached. '
                        f'Aligning final yaw before arrival: '
                        f'error={math.degrees(yaw_error):.1f} deg',
                        throttle_duration_sec=1.0,
                    )
                    return

            self.get_logger().info(
                f'[RouteExecutor] ARRIVED_LIDAR: '
                f'front={self._front_dist:.3f} m, '
                f'dist_to_goal={dist_to_goal:.3f} m, '
                f'heading_error={heading_error:.2f} rad'
            )
            self._stop_robot()
            self._set_status('ARRIVED_LIDAR')
            return

        # --------------------------------------------------------------
        # 4. Path-aware obstacle handling
        # --------------------------------------------------------------
        if self._front_blocked:
            # Case A:
            # The replanned path asks the robot to turn away.
            # Allow rotation in place.
            if abs(heading_error) > self.FULL_SPEED_HEADING_ERR:
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = 'base_link'

                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = max(
                    -self.ANG_SPEED,
                    min(self.ANG_SPEED, self.K_HEADING * heading_error)
                )

                self._pub_cmd.publish(cmd)

                # Do not count this as blocked, because the robot is actively
                # turning away into the replanned path.
                self._blocked_since = None

                self.get_logger().warn(
                    f'[RouteExecutor] Front blocked, but path requires turn. '
                    f'Allowing rotate-in-place. heading_error={heading_error:.2f} rad',
                    throttle_duration_sec=1.0,
                )
                return

            # Case B:
            # Robot is facing the next waypoint and would move forward.
            # Now it is truly blocked.
            self._stop_robot()

            if self._blocked_since is None:
                self._blocked_since = self.get_clock().now()
                self.get_logger().warn(
                    f'[RouteExecutor] Front blocked on current path. '
                    f'front_dist={self._front_dist:.2f} m'
                )
                return

            blocked_time = (
                self.get_clock().now() - self._blocked_since
            ).nanoseconds / 1e9

            if blocked_time >= BLOCKED_PERSIST_S:
                self.get_logger().error(
                    f'[RouteExecutor] Path BLOCKED for {blocked_time:.1f} s. '
                    f'front_dist={self._front_dist:.2f} m'
                )
                self._stop_robot()
                self._set_status('BLOCKED')
                return

            return

        else:
            self._blocked_since = None

        # --------------------------------------------------------------
        # 5. Compute pure path-following control
        # --------------------------------------------------------------
        wp = self._current_path.poses[self._wp_index]
        wx = wp.pose.position.x
        wy = wp.pose.position.y

        v, omega, heading_err = self._compute_control_basic(wx, wy)

        v = max(-self.LIN_SPEED, min(self.LIN_SPEED, v))
        omega = max(-self.ANG_SPEED, min(self.ANG_SPEED, omega))

        # --------------------------------------------------------------
        # 6. Publish TwistStamped
        # --------------------------------------------------------------
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega
        self._pub_cmd.publish(cmd)

        # --------------------------------------------------------------
        # 7. Publish progress
        # --------------------------------------------------------------
        progress = self._compute_progress()
        pm = Float32()
        pm.data = float(progress)
        self._pub_progress.publish(pm)

        # --------------------------------------------------------------
        # 8. Debug log
        # --------------------------------------------------------------
        now = time.monotonic()
        if now - self._last_debug_t >= self.DEBUG_PERIOD_S:
            self._last_debug_t = now

            self.get_logger().info(
                f'[CTRL] '
                f'pos=({self._robot_x:.2f},{self._robot_y:.2f}) '
                f'yaw={math.degrees(self._robot_yaw):.1f}deg | '
                f'wp[{self._wp_index}/{len(self._current_path.poses)-1}]= '
                f'({wx:.2f},{wy:.2f}) | '
                f'goal=({self._goal_xy[0]:.2f},{self._goal_xy[1]:.2f}) '
                f'goal_yaw={math.degrees(self._goal_yaw):.1f}deg | '
                f'dist_goal={dist_to_goal:.2f} m | '
                f'heading_err={math.degrees(heading_err):.1f}deg | '
                f'v={v:.3f}, omega={omega:.3f} | '
                f'front={self._front_dist:.2f} m, blocked={self._front_blocked}'
            )

    # ------------------------------------------------------------------
    # Pure waypoint controller
    # ------------------------------------------------------------------

    def _compute_control_basic(self, wx: float, wy: float):
        """
        Pure A* path-following controller.

        No APF.
        No repulsive angular correction.
        No obstacle steering.

        Behavior:
        - If heading error is large, rotate in place.
        - If heading error is moderate, move slowly.
        - If heading error is small, move forward normally.
        """

        desired_yaw = math.atan2(
            wy - self._robot_y,
            wx - self._robot_x,
        )

        heading_err = self._norm(desired_yaw - self._robot_yaw)

        omega = self.K_HEADING * heading_err
        abs_err = abs(heading_err)

        if abs_err > self.ROTATE_IN_PLACE_ERR:
            v = 0.0
        elif abs_err < self.FULL_SPEED_HEADING_ERR:
            v = self.LIN_SPEED
        else:
            scale = (
                self.ROTATE_IN_PLACE_ERR - abs_err
            ) / (
                self.ROTATE_IN_PLACE_ERR - self.FULL_SPEED_HEADING_ERR
            )
            scale = max(0.0, min(1.0, scale))
            v = self.LIN_SPEED * scale

        return v, omega, heading_err

    # ------------------------------------------------------------------
    # Waypoint handling
    # ------------------------------------------------------------------

    def _advance_waypoint(self):
        if self._current_path is None:
            return

        max_idx = len(self._current_path.poses) - 1

        while self._wp_index < max_idx:
            wp = self._current_path.poses[self._wp_index]
            wx = wp.pose.position.x
            wy = wp.pose.position.y

            d = math.hypot(
                self._robot_x - wx,
                self._robot_y - wy,
            )

            if d >= self.WAYPOINT_ACCEPT_DIST:
                break

            self._wp_index += 1
            nwp = self._current_path.poses[self._wp_index]

            self.get_logger().info(
                f'[WP] Advanced to waypoint '
                f'[{self._wp_index}/{max_idx}] '
                f'({nwp.pose.position.x:.2f},{nwp.pose.position.y:.2f}); '
                f'previous distance={d:.2f} m'
            )

    # ------------------------------------------------------------------
    # TF pose update
    # ------------------------------------------------------------------

    def _update_pose(self) -> bool:
        try:
            tf = self._tf_buf.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            t = tf.transform.translation
            q = tf.transform.rotation

            self._robot_x = t.x
            self._robot_y = t.y
            self._robot_yaw = self._quat_to_yaw(q)
            self._pose_ok = True

            return True

        except TransformException as e:
            self.get_logger().debug(f'[RouteExecutor] TF lookup failed: {e}')
            return False

    # ------------------------------------------------------------------
    # Status / progress helpers
    # ------------------------------------------------------------------

    def _stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self._pub_cmd.publish(msg)

    def _set_status(self, status: str):
        if status != self._status:
            self.get_logger().warn(
                f'[RouteExecutor][STATUS] {self._status} → {status}'
            )
        self._status = status

    def _status_tick(self):
        msg = String()
        msg.data = self._status
        self._pub_status.publish(msg)

    def _compute_progress(self) -> float:
        if self._current_path is None or self._path_total_len < 1e-6:
            return 0.0

        if self._goal_xy is None:
            return 0.0

        dist_to_goal = math.hypot(
            self._robot_x - self._goal_xy[0],
            self._robot_y - self._goal_xy[1],
        )

        return max(
            0.0,
            min(1.0, 1.0 - dist_to_goal / self._path_total_len),
        )

    @staticmethod
    def _compute_path_length(path: Path) -> float:
        total = 0.0
        poses = path.poses

        for i in range(len(poses) - 1):
            total += math.hypot(
                poses[i + 1].pose.position.x - poses[i].pose.position.x,
                poses[i + 1].pose.position.y - poses[i].pose.position.y,
            )

        return total

    @staticmethod
    def _norm(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _quat_to_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
