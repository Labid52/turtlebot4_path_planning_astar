#!/usr/bin/env python3
"""
mission_manager_node.py
=======================

Robust top-level FSM for the TurtleBot4 tour-guide mission.

Important design choice:
- MissionManager does NOT publish /cmd_vel directly.
- Only RouteExecutor publishes /cmd_vel.
- MissionManager controls the robot indirectly by:
    1. requesting stops,
    2. requesting A* plans,
    3. sending paths to the executor,
    4. cancelling executor when needed,
    5. triggering replanning/recovery/skip decisions.

Why:
- The real TurtleBot4 Create3 base expects TwistStamped on /cmd_vel.
- The real executor already publishes TwistStamped.
- So MissionManager should not publish plain Twist on /cmd_vel.

Important replan-count rule:
- _replan_count counts failed navigation/planning attempts.
- Recovery itself does NOT increment _replan_count.
- This prevents one failure from consuming multiple attempts.

Executor timing rule:
- Start timeout clock begins when path is sent.
- Stale status watchdog begins only after executor acknowledges EXECUTING.

Important obstacle rule:
- MissionManager does NOT force replanning directly from raw front_blocked.
- RouteExecutor is path-aware and decides whether the current path is truly BLOCKED.
- MissionManager reacts to executor status == BLOCKED.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, String

import tf2_ros
from tf2_ros import TransformException

from tour_guide.config import (
    HOME_POSE,
    DWELL_TIME_S,
    MAX_REPLAN_ATTEMPTS,
    PLAN_TIMEOUT_S,
    EXEC_TIMEOUT_S,
    SLAM_WARMUP_S,
    STOP_TIMEOUT_S,
    LANDMARKS,
)


def _yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MissionManagerNode(Node):

    FSM_HZ = 5.0

    EXECUTOR_START_TIMEOUT_S = 6.0
    EXECUTOR_STATUS_STALE_S = 5.0
    RECOVERY_WAIT_S = 2.0

    # Return-home should not loop forever.
    MAX_HOME_ATTEMPTS = 3

    def __init__(self):
        super().__init__('mission_manager_node')

        # ------------------------------------------------------------------
        # FSM state
        # ------------------------------------------------------------------
        self._state = 'INIT'
        self._state_entry_t = time.monotonic()

        # ------------------------------------------------------------------
        # Mission tracking
        # ------------------------------------------------------------------
        self._current_stop_name = None
        self._current_goal = None
        self._is_returning_home = False
        self._replan_count = 0
        self._visited = []
        self._skipped = []
        self._home_attempts = 0
        self._return_home_prepared = False

        # ------------------------------------------------------------------
        # Stop manager state
        # ------------------------------------------------------------------
        self._stop_next = None
        self._stop_next_name = None
        self._stop_received = False
        self._stop_request_sent = False

        # ------------------------------------------------------------------
        # Planner state
        # ------------------------------------------------------------------
        self._planner_status = ''
        self._planner_status_recv = False
        self._planner_path = None
        self._plan_request_sent = False

        # ------------------------------------------------------------------
        # Executor state
        # ------------------------------------------------------------------
        self._executor_status = 'IDLE'
        self._last_executor_status_time = time.monotonic()
        self._exec_path_sent = False
        self._exec_waiting_for_start = False
        self._exec_start_wall_t = 0.0

        # ------------------------------------------------------------------
        # Obstacle monitor state
        # MissionManager only reports this in /mission/state.
        # It does NOT force replan from raw front_blocked anymore.
        # ------------------------------------------------------------------
        self._front_blocked = False
        self._front_dist = 999.0

        # ------------------------------------------------------------------
        # Pose readiness
        # ------------------------------------------------------------------
        self._pose_ready = False

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_stop_cmd = self.create_publisher(
            String,
            '/stop_manager/cmd',
            10,
        )

        self._pub_plan_goal = self.create_publisher(
            PoseStamped,
            '/planner/goal',
            10,
        )

        self._pub_exec_path = self.create_publisher(
            Path,
            '/executor/path',
            10,
        )

        self._pub_exec_cancel = self.create_publisher(
            Bool,
            '/executor/cancel',
            10,
        )

        self._pub_state = self.create_publisher(
            String,
            '/mission/state',
            10,
        )

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            PoseStamped,
            '/stop_manager/next_stop',
            self._next_stop_cb,
            10,
        )

        self.create_subscription(
            String,
            '/stop_manager/next_stop_name',
            self._next_stop_name_cb,
            10,
        )

        self.create_subscription(
            String,
            '/planner/status',
            self._planner_status_cb,
            10,
        )

        self.create_subscription(
            Path,
            '/planner/path',
            self._planner_path_cb,
            10,
        )

        self.create_subscription(
            String,
            '/executor/status',
            self._executor_status_cb,
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
        # Timers
        # ------------------------------------------------------------------
        self.create_timer(1.0 / self.FSM_HZ, self._fsm_tick)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            '[MissionManager] Robust FSM started. '
            'MissionManager does not publish /cmd_vel directly. '
            'Raw front_blocked is monitored but does not force replanning. '
            f'SLAM/map warmup={SLAM_WARMUP_S} s'
        )

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------

    def _next_stop_cb(self, msg: PoseStamped):
        self._stop_next = msg
        self._stop_received = True

        if msg.header.frame_id != 'MAP_DONE':
            self.get_logger().info(
                f'[MissionManager] Received next stop pose: '
                f'map({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )

    def _next_stop_name_cb(self, msg: String):
        self._stop_next_name = msg.data

    def _planner_status_cb(self, msg: String):
        prev = self._planner_status
        self._planner_status = msg.data
        self._planner_status_recv = True

        if msg.data != prev and msg.data not in ('IDLE', 'NO_MAP'):
            self.get_logger().info(
                f'[MissionManager] Planner status → {msg.data}'
            )

    def _planner_path_cb(self, msg: Path):
        self._planner_path = msg

    def _executor_status_cb(self, msg: String):
        prev = self._executor_status
        self._executor_status = msg.data
        self._last_executor_status_time = time.monotonic()

        if msg.data != prev:
            self.get_logger().info(
                f'[MissionManager] Executor status → {msg.data}'
            )

    def _front_blocked_cb(self, msg: Bool):
        self._front_blocked = msg.data

    def _front_dist_cb(self, msg: Float32):
        self._front_dist = msg.data

    # ------------------------------------------------------------------
    # FSM utility
    # ------------------------------------------------------------------

    def _fsm_tick(self):
        if self._state == 'INIT':
            self._s_init()
        elif self._state == 'GETTING_STOP':
            self._s_getting_stop()
        elif self._state == 'PLANNING':
            self._s_planning()
        elif self._state == 'EXECUTING':
            self._s_executing()
        elif self._state == 'DWELLING':
            self._s_dwelling()
        elif self._state == 'REPLANNING':
            self._s_replanning()
        elif self._state == 'RECOVERY':
            self._s_recovery()
        elif self._state == 'RETURNING_HOME':
            self._s_returning_home()
        elif self._state == 'COMPLETE':
            self._s_complete()
        elif self._state == 'FAILED':
            self._s_failed()
        else:
            self.get_logger().error(
                f'[MissionManager] Unknown FSM state: {self._state}'
            )
            self._go('FAILED')

    def _go(self, new_state: str):
        self.get_logger().warn(
            f'[MissionManager] FSM {self._state} → {new_state}'
        )

        self._state = new_state
        self._state_entry_t = time.monotonic()

        if new_state == 'GETTING_STOP':
            self._stop_request_sent = False
            self._stop_received = False
            self._stop_next = None
            self._stop_next_name = None

            # Normal stop sequence. Home attempts are only meaningful
            # after MAP_DONE triggers RETURNING_HOME.
            if not self._is_returning_home:
                self._home_attempts = 0

        elif new_state == 'PLANNING':
            self._plan_request_sent = False
            self._planner_path = None
            self._planner_status = 'IDLE'

        elif new_state == 'EXECUTING':
            self._exec_path_sent = False
            self._exec_waiting_for_start = False
            self._exec_start_wall_t = 0.0
            self._executor_status = 'IDLE'

        elif new_state == 'REPLANNING':
            pass

        elif new_state == 'RECOVERY':
            pass

        elif new_state == 'RETURNING_HOME':
            self._return_home_prepared = False

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_entry_t

    def _publish_cancel(self):
        msg = Bool()
        msg.data = True
        self._pub_exec_cancel.publish(msg)

    # ------------------------------------------------------------------
    # FSM states
    # ------------------------------------------------------------------

    def _s_init(self):
        if not self._pose_ready:
            try:
                self._tf_buf.lookup_transform(
                    'map',
                    'base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                self._pose_ready = True
                self.get_logger().info(
                    '[MissionManager] INIT: TF map → base_footprint available.'
                )
            except TransformException:
                pass

        elapsed = self._elapsed()

        if int(elapsed) % 5 == 0 and int(elapsed) != getattr(self, '_init_last_log', -1):
            self._init_last_log = int(elapsed)
            self.get_logger().info(
                f'[MissionManager] INIT: '
                f'pose_ready={self._pose_ready}, '
                f'planner_recv={self._planner_status_recv}, '
                f'planner_status={self._planner_status!r}, '
                f'elapsed={elapsed:.0f}/{SLAM_WARMUP_S} s'
            )

        planner_has_map = (
            self._planner_status_recv
            and self._planner_status not in ('', 'NO_MAP')
        )

        if self._pose_ready and planner_has_map and elapsed >= SLAM_WARMUP_S:
            self.get_logger().info(
                '[MissionManager] INIT complete. Starting mission.'
            )
            self._go('GETTING_STOP')
            return

        if elapsed > max(SLAM_WARMUP_S + 60.0, 90.0):
            self.get_logger().error(
                f'[MissionManager] INIT TIMEOUT: '
                f'pose_ready={self._pose_ready}, '
                f'planner_status={self._planner_status!r}'
            )
            self._go('FAILED')

    # ------------------------------------------------------------------

    def _s_getting_stop(self):
        if not self._stop_request_sent:
            self._stop_request_sent = True
            self._stop_received = False
            self._stop_next = None
            self._stop_next_name = None

            cmd = String()
            cmd.data = 'GET_NEXT'
            self._pub_stop_cmd.publish(cmd)

            self.get_logger().info(
                '[MissionManager] Sent GET_NEXT to StopManager.'
            )
            return

        if self._stop_received:
            nxt = self._stop_next

            if nxt.header.frame_id == 'MAP_DONE':
                self.get_logger().info(
                    '[MissionManager] All stops done or skipped. Returning home.'
                )
                self._go('RETURNING_HOME')
                return

            self._current_goal = nxt
            self._current_stop_name = (
                self._stop_next_name or self._lookup_name(nxt)
            )
            self._is_returning_home = False
            self._replan_count = 0

            self.get_logger().info(
                f'[MissionManager] New target: {self._current_stop_name}, '
                f'map({nxt.pose.position.x:.2f}, {nxt.pose.position.y:.2f})'
            )

            self._go('PLANNING')
            return

        if self._elapsed() > STOP_TIMEOUT_S:
            self.get_logger().error(
                '[MissionManager] GETTING_STOP timeout.'
            )
            self._go('FAILED')

    # ------------------------------------------------------------------

    def _s_planning(self):
        if self._current_goal is None:
            self.get_logger().error(
                '[MissionManager] Cannot plan: current goal is None.'
            )
            self._go('FAILED')
            return

        if not self._plan_request_sent:
            self._plan_request_sent = True
            self._planner_path = None
            self._planner_status = 'IDLE'

            self.get_logger().info(
                f'[MissionManager] Requesting A* plan to '
                f'{self._current_stop_name}: '
                f'map({self._current_goal.pose.position.x:.2f}, '
                f'{self._current_goal.pose.position.y:.2f})'
            )

            self._pub_plan_goal.publish(self._current_goal)
            return

        if self._planner_status == 'SUCCESS' and self._planner_path is not None:
            self.get_logger().info(
                f'[MissionManager] Plan received: '
                f'{len(self._planner_path.poses)} waypoints.'
            )
            self._go('EXECUTING')
            return

        if self._planner_status == 'FAILED':
            self._replan_count += 1

            self.get_logger().error(
                f'[MissionManager] Planner FAILED for {self._current_stop_name}. '
                f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
            )

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('RECOVERY')
            else:
                self._handle_max_attempts_exceeded('planner failed')
            return

        if self._planner_status == 'NO_POSE':
            self.get_logger().warn(
                '[MissionManager] Planner has no robot pose. Waiting...',
                throttle_duration_sec=2.0,
            )

        if self._elapsed() > PLAN_TIMEOUT_S:
            self._replan_count += 1

            self.get_logger().error(
                f'[MissionManager] PLANNING timeout after {PLAN_TIMEOUT_S} s. '
                f'planner_status={self._planner_status!r}. '
                f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
            )

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('RECOVERY')
            else:
                self._handle_max_attempts_exceeded('planning timeout')

    # ------------------------------------------------------------------

    def _s_executing(self):
        elapsed = self._elapsed()

        if self._planner_path is None:
            self.get_logger().error(
                '[MissionManager] EXECUTING entered with no planner path.'
            )
            self._replan_count += 1

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('RECOVERY')
            else:
                self._handle_max_attempts_exceeded('entered EXECUTING with no planner path')
            return

        # --------------------------------------------------------------
        # Send path once.
        # Start timeout clock starts here.
        # Stale executor watchdog does NOT start here.
        # --------------------------------------------------------------
        if not self._exec_path_sent:
            self._exec_path_sent = True
            self._exec_waiting_for_start = True
            self._exec_start_wall_t = time.monotonic()
            self._executor_status = 'IDLE'

            self.get_logger().info(
                f'[MissionManager] Sending path to executor: '
                f'{len(self._planner_path.poses)} waypoints → '
                f'{self._current_stop_name}'
            )

            self._pub_exec_path.publish(self._planner_path)
            return

        # --------------------------------------------------------------
        # Wait for executor acknowledgement.
        # --------------------------------------------------------------
        if self._exec_waiting_for_start:
            if self._executor_status == 'EXECUTING':
                self._exec_waiting_for_start = False

                # Stale status watchdog starts only after acknowledgement.
                self._last_executor_status_time = time.monotonic()

                self.get_logger().info(
                    '[MissionManager] Executor acknowledged path.'
                )

            elif time.monotonic() - self._exec_start_wall_t > self.EXECUTOR_START_TIMEOUT_S:
                self._replan_count += 1

                self.get_logger().error(
                    f'[MissionManager] Executor did not start. '
                    f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
                )

                self._publish_cancel()

                if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                    self._go('RECOVERY')
                else:
                    self._handle_max_attempts_exceeded('executor did not start')
            return

        # --------------------------------------------------------------
        # Stale executor status watchdog.
        # This only runs after EXECUTING acknowledgement.
        # --------------------------------------------------------------
        if time.monotonic() - self._last_executor_status_time > self.EXECUTOR_STATUS_STALE_S:
            self._replan_count += 1

            self.get_logger().error(
                f'[MissionManager] Executor status stale. '
                f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
            )

            self._publish_cancel()

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('RECOVERY')
            else:
                self._handle_max_attempts_exceeded('executor status stale')
            return

        # --------------------------------------------------------------
        # Important:
        # No direct front_blocked forced replanning here.
        # RouteExecutor is path-aware and will report BLOCKED only when needed.
        # --------------------------------------------------------------

        status = self._executor_status

        if status in ('ARRIVED_POSE', 'ARRIVED_LIDAR'):
            self.get_logger().info(
                f'[MissionManager] Arrived at {self._current_stop_name} '
                f'with status={status}.'
            )
            self._go('DWELLING')
            return

        if status == 'BLOCKED':
            self._replan_count += 1

            self.get_logger().warn(
                f'[MissionManager] Executor reports BLOCKED. '
                f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
            )

            self._publish_cancel()

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('REPLANNING')
            else:
                self._handle_max_attempts_exceeded('executor blocked')
            return

        if status == 'FAILED':
            self._replan_count += 1

            self.get_logger().error(
                f'[MissionManager] Executor FAILED. '
                f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
            )

            self._publish_cancel()

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('RECOVERY')
            else:
                self._handle_max_attempts_exceeded('executor failed')
            return

        if elapsed > EXEC_TIMEOUT_S:
            self._replan_count += 1

            self.get_logger().error(
                f'[MissionManager] EXECUTION timeout after {EXEC_TIMEOUT_S} s. '
                f'executor_status={status!r}. '
                f'Attempt {self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
            )

            self._publish_cancel()

            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self._go('RECOVERY')
            else:
                self._handle_max_attempts_exceeded('execution timeout')
            return

    # ------------------------------------------------------------------

    def _s_dwelling(self):
        """
        Pause at landmark.

        No /cmd_vel is published here.
        Executor already stopped when it declared ARRIVED.
        Cancel is published as extra safety for the first 0.5 s.
        """

        if self._elapsed() < 0.5:
            self._publish_cancel()
            self.get_logger().info(
                f'[MissionManager] Dwelling at {self._current_stop_name} '
                f'for {DWELL_TIME_S} s.'
            )

        if self._elapsed() >= DWELL_TIME_S:
            self.get_logger().info(
                f'[MissionManager] Dwell complete at {self._current_stop_name}.'
            )

            if self._current_stop_name and self._current_stop_name != 'Home':
                cmd = String()
                cmd.data = f'MARK_VISITED:{self._current_stop_name}'
                self._pub_stop_cmd.publish(cmd)

                if self._current_stop_name not in self._visited:
                    self._visited.append(self._current_stop_name)

            if self._is_returning_home:
                self._go('COMPLETE')
            else:
                self._go('GETTING_STOP')

    # ------------------------------------------------------------------

    def _s_replanning(self):
        if self._elapsed() < 0.2:
            self._publish_cancel()
            self.get_logger().info(
                '[MissionManager] REPLANNING: executor cancelled.'
            )

        if self._elapsed() >= 1.0:
            self.get_logger().info(
                '[MissionManager] REPLANNING: requesting fresh plan.'
            )
            self._go('PLANNING')

    # ------------------------------------------------------------------

    def _s_recovery(self):
        """
        Conservative recovery without direct /cmd_vel.

        Recovery does NOT increment _replan_count.
        The failure event that led to recovery already incremented it.
        """

        if self._elapsed() < 0.2:
            self._publish_cancel()
            self.get_logger().warn(
                '[MissionManager] RECOVERY: executor cancelled. Waiting before replanning.'
            )

        if self._elapsed() >= self.RECOVERY_WAIT_S:
            if self._replan_count <= MAX_REPLAN_ATTEMPTS:
                self.get_logger().info(
                    f'[MissionManager] RECOVERY complete. '
                    f'Retrying plan with attempt count '
                    f'{self._replan_count}/{MAX_REPLAN_ATTEMPTS}.'
                )
                self._go('PLANNING')
            else:
                self._handle_max_attempts_exceeded('recovery max attempts exceeded')

    # ------------------------------------------------------------------

    def _s_returning_home(self):
        """
        Prepare home as current goal once, then go through normal planning/execution.

        _return_home_prepared guards against repeated preparation during the
        same RETURNING_HOME entry.

        _home_attempts counts how many times we have entered return-home mode.
        If it exceeds MAX_HOME_ATTEMPTS, we declare COMPLETE rather than
        looping forever.
        """

        if getattr(self, '_return_home_prepared', False):
            return

        self._home_attempts = getattr(self, '_home_attempts', 0) + 1

        if self._home_attempts > self.MAX_HOME_ATTEMPTS:
            self.get_logger().error(
                f'[MissionManager] Cannot reach home after '
                f'{self._home_attempts - 1} attempts. '
                f'Declaring COMPLETE anyway.'
            )
            self._go('COMPLETE')
            return

        self.get_logger().info(
            f'[MissionManager] Returning home attempt '
            f'{self._home_attempts}/{self.MAX_HOME_ATTEMPTS}: '
            f'map({HOME_POSE["x"]:.2f}, {HOME_POSE["y"]:.2f})'
        )

        self._return_home_prepared = True
        self._is_returning_home = True
        self._current_stop_name = 'Home'
        self._replan_count = 0

        home = PoseStamped()
        home.header.stamp = self.get_clock().now().to_msg()
        home.header.frame_id = 'map'
        home.pose.position.x = HOME_POSE['x']
        home.pose.position.y = HOME_POSE['y']
        home.pose.position.z = 0.0

        qx, qy, qz, qw = _yaw_to_quat(HOME_POSE['yaw'])
        home.pose.orientation.x = qx
        home.pose.orientation.y = qy
        home.pose.orientation.z = qz
        home.pose.orientation.w = qw

        self._current_goal = home
        self._go('PLANNING')

    # ------------------------------------------------------------------

    def _s_complete(self):
        if self._elapsed() < 0.5:
            self._publish_cancel()

            sep = '=' * 60
            self.get_logger().info(
                f'\n{sep}\n'
                f'[MissionManager] *** MISSION COMPLETE ***\n'
                f'Visited: {self._visited}\n'
                f'Skipped: {self._skipped}\n'
                f'{sep}'
            )

    # ------------------------------------------------------------------

    def _s_failed(self):
        if self._elapsed() < 0.5:
            self._publish_cancel()
            self.get_logger().error(
                '[MissionManager] *** MISSION FAILED ***'
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _handle_max_attempts_exceeded(self, reason: str):
        """
        Centralized failure policy.

        For normal landmarks:
            skip current stop and continue mission.

        For Home:
            do not go back to StopManager and do not loop forever.
            declare COMPLETE.
        """

        self.get_logger().error(
            f'[MissionManager] Max attempts exceeded for {self._current_stop_name}. '
            f'Reason: {reason}'
        )

        self._publish_cancel()

        if self._is_returning_home or self._current_stop_name == 'Home':
            self.get_logger().error(
                '[MissionManager] Home could not be reached. '
                'Declaring COMPLETE instead of looping or stopping forever.'
            )
            self._go('COMPLETE')
            return

        self._skip_current()
        self._go('GETTING_STOP')

    def _skip_current(self):
        if self._current_stop_name and self._current_stop_name != 'Home':
            cmd = String()
            cmd.data = f'SKIP:{self._current_stop_name}'
            self._pub_stop_cmd.publish(cmd)

            if self._current_stop_name not in self._skipped:
                self._skipped.append(self._current_stop_name)

            self.get_logger().warn(
                f'[MissionManager] Skipped: {self._current_stop_name}'
            )

    def _lookup_name(self, pose: PoseStamped) -> str:
        for lm in LANDMARKS:
            if (
                abs(lm['x'] - pose.pose.position.x) < 0.15
                and abs(lm['y'] - pose.pose.position.y) < 0.15
            ):
                return lm['name']

        return f'Stop({pose.pose.position.x:.1f},{pose.pose.position.y:.1f})'

    def _publish_state(self):
        state = {
            'fsm': self._state,
            'stop': self._current_stop_name,
            'returning_home': self._is_returning_home,
            'home_attempts': self._home_attempts,
            'max_home_attempts': self.MAX_HOME_ATTEMPTS,
            'replan_count': self._replan_count,
            'max_replan_attempts': MAX_REPLAN_ATTEMPTS,
            'visited': self._visited,
            'skipped': self._skipped,
            'planner_status': self._planner_status,
            'planner_path_ready': self._planner_path is not None,
            'executor_status': self._executor_status,
            'front_blocked': self._front_blocked,
            'front_dist': round(self._front_dist, 3),
            'pose_ready': self._pose_ready,
        }

        msg = String()
        msg.data = json.dumps(state)
        self._pub_state.publish(msg)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cancel()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
