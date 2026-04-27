#!/usr/bin/env python3
"""
Coverage Planner Node  (fixed)
──────────────────────────────
Waits for:
  1. /exploration_done  (Bool True)  — published by PipelineManager
  2. AMCL pose          (/amcl_pose) — confirms localisation before sweep

Uses Fields2Cover for coverage path generation when available, with a
legacy grid-based fallback.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import Bool

try:
    import numpy as np
except ImportError:
    np = None

try:
    import cv2
except ImportError:
    cv2 = None


class CoveragePlanner(Node):

    def __init__(self):
        super().__init__('coverage_planner')

        self.declare_parameter('step_size',              5)
        self.declare_parameter('coverage_algorithm',     'fields2cover')
        self.declare_parameter('robot_width',            0.34)
        self.declare_parameter('robot_length',           0.40)
        self.declare_parameter('robot_min_radius',       0.30)
        self.declare_parameter('min_contour_area',       0.25)
        self.declare_parameter('amcl_timeout',           30.0)

        self._step_size            = self.get_parameter('step_size').get_parameter_value().integer_value
        self._coverage_algorithm   = self.get_parameter('coverage_algorithm').get_parameter_value().string_value.lower()
        self._robot_width          = self.get_parameter('robot_width').get_parameter_value().double_value
        self._robot_length         = self.get_parameter('robot_length').get_parameter_value().double_value
        self._robot_min_radius     = self.get_parameter('robot_min_radius').get_parameter_value().double_value
        self._min_contour_area     = self.get_parameter('min_contour_area').get_parameter_value().double_value
        self._amcl_timeout         = self.get_parameter('amcl_timeout').get_parameter_value().double_value

        self._exploration_done      = False
        self._map_received          = False
        self._map_msg               = None
        self._amcl_pose             = None
        self._amcl_timer            = None
        self._amcl_wait_ticks       = 0
        self._start_signal_received = False
        self._coverage_started      = False

        self.create_subscription(Bool,         '/exploration_done', self._done_cb,   1)
        self.create_subscription(Bool,         '/start_coverage',   self._start_cb,  1)
        self.create_subscription(OccupancyGrid,'/map',              self._map_cb,   10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_cb,
            10,
        )

        self._nav_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

        self.get_logger().info(
            'Coverage Planner ready — idle until /exploration_done signal.'
        )

    # ── Callbacks ──────────────────────────────────────────────────────── #

    def _done_cb(self, msg: Bool):
        if not msg.data or self._exploration_done:
            return
        self._exploration_done = True
        self.get_logger().info(
            f'✅ /exploration_done received. '
            f'Waiting up to {self._amcl_timeout:.0f}s for AMCL pose and /start_coverage signal...'
        )
        # Poll for AMCL using a repeating timer — no blocking spin
        self._amcl_timer = self.create_timer(1.0, self._amcl_poll_cb)

    def _start_cb(self, msg: Bool):
        if not msg.data or self._start_signal_received:
            return
        self._start_signal_received = True
        self.get_logger().info('✅ /start_coverage signal received.')
        self._try_generate_waypoints()

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        if self._amcl_pose is None:
            p = msg.pose.pose.position
            self.get_logger().info(
                f'📍 AMCL pose received — robot at ({p.x:.2f}, {p.y:.2f}).'
            )
        self._amcl_pose = msg

    def _map_cb(self, msg: OccupancyGrid):
        """Triggered by the next /map message after exploration is done."""
        if not self._exploration_done or self._map_received:
            return
        self._map_received = True
        self._map_msg = msg
        self.get_logger().info('🗺️  Final map received.')
        self._try_generate_waypoints()

    def _try_generate_waypoints(self):
        if self._coverage_started:
            return
        if not self._exploration_done:
            return
        if not self._map_received:
            self.get_logger().info('Waiting for final map before generating waypoints...')
            return
        if not self._start_signal_received:
            self.get_logger().info('Waiting for /start_coverage signal before generating waypoints...')
            return

        self._coverage_started = True
        self._stop_amcl_timer()
        self.get_logger().info('🔄 Starting coverage waypoint generation now.')
        waypoints = self._generate_path(self._map_msg)
        self._send_waypoints(waypoints)

    # ── AMCL poll timer ────────────────────────────────────────────────── #

    def _amcl_poll_cb(self):
        """Fires every 1 s. Cancels itself when AMCL arrives or timeout hits."""
        if self._amcl_pose is not None:
            self._stop_amcl_timer()
            self.get_logger().info('🤖 Localisation confirmed. Ready for sweep.')
            return

        self._amcl_wait_ticks += 1
        if self._amcl_wait_ticks >= int(self._amcl_timeout):
            self._stop_amcl_timer()
            self.get_logger().warn(
                '⚠️  AMCL timeout — proceeding without confirmed localisation.'
            )

    def _stop_amcl_timer(self):
        if self._amcl_timer is not None:
            self._amcl_timer.cancel()
            self._amcl_timer = None

    # ── Path generation ────────────────────────────────────────────────── #

    def _generate_path(self, map_msg: OccupancyGrid):
        if self._coverage_algorithm == 'fields2cover':
            try:
                self.get_logger().info('🧭 Using Fields2Cover to generate coverage path...')
                return self._generate_path_fields2cover(map_msg)
            except Exception as exc:
                self.get_logger().warn(
                    '⚠️  Fields2Cover generation failed; falling back to legacy grid coverage. '
                    f'Reason: {exc}'
                )

        self.get_logger().info('🧱 Using legacy grid-based coverage generator.')
        return self._generate_path_grid(map_msg)

    def _generate_path_grid(self, map_msg: OccupancyGrid):
        width  = map_msg.info.width
        height = map_msg.info.height
        res    = map_msg.info.resolution
        origin = map_msg.info.origin
        data   = np.asarray(map_msg.data, dtype=np.int8).reshape((height, width))

        waypoints = []
        row_index = 0
        total_segments = 0
        max_y = max(0, height - 1)

        for y in range(0, max_y + 1, self._step_size):
            if y >= height:
                break

            row_segments = []
            segment_start = None
            for x in range(0, width):
                if data[y, x] == 0:
                    if segment_start is None:
                        segment_start = x
                    segment_end = x
                else:
                    if segment_start is not None:
                        row_segments.append((segment_start, segment_end))
                        segment_start = None
            if segment_start is not None:
                row_segments.append((segment_start, segment_end))

            if not row_segments:
                continue

            total_segments += len(row_segments)
            if row_index % 2 != 0:
                row_segments = row_segments[::-1]

            for seg_start, seg_end in row_segments:
                if row_index % 2 == 0:
                    points = [(seg_start, y), (seg_end, y)]
                else:
                    points = [(seg_end, y), (seg_start, y)]

                for x_cell, y_cell in points:
                    wx = origin.position.x + (x_cell + 0.5) * res
                    wy = origin.position.y + (y_cell + 0.5) * res
                    if not self._is_within_map_bounds(wx, wy, origin, width, height, res):
                        continue
                    waypoints.append(self._make_pose(wx, wy))

            row_index += 1

        self.get_logger().info(
            f'Generated {len(waypoints)} waypoints across {row_index} sweep rows '
            f'and {total_segments} free segments.'
        )
        return waypoints

    def _generate_path_fields2cover(self, map_msg: OccupancyGrid):
        if np is None or cv2 is None:
            raise RuntimeError('Fields2Cover requires numpy and OpenCV; install python3-numpy and python3-opencv.')

        try:
            import fields2cover as f2c
        except ImportError as exc:
            raise RuntimeError('The fields2cover Python module is not available.') from exc

        self.get_logger().info('  → Extracting field polygons from map...')
        field = self._make_fields2cover_field(map_msg, f2c)
        self.get_logger().info('  → Creating robot configuration...')
        robot = self._make_fields2cover_robot(f2c)

        self.get_logger().info('  → Computing coverage path (this may take a moment)...')
        try:
            path = f2c.planCovPath(robot, field, False)
        except Exception as exc:
            raise RuntimeError(f'Fields2Cover planCovPath failed: {exc}') from exc
        
        if path is None or len(path) == 0:
            raise RuntimeError('Fields2Cover produced an empty path.')

        self.get_logger().info(f'  → Converting {len(path)} path points to waypoints...')
        waypoints = self._convert_f2c_path_to_waypoints(path)
        self.get_logger().info(f'Generated {len(waypoints)} waypoints from Fields2Cover path.')
        return waypoints

    def _make_fields2cover_robot(self, f2c):
        robot = f2c.Robot(self._robot_width, self._robot_length)
        # Note: setMinRadius not available in Fields2Cover Python API; would be set via planning algorithm
        # robot.setMinRadius(self._robot_min_radius)
        return robot

    def _make_fields2cover_field(self, map_msg: OccupancyGrid, f2c):
        width  = map_msg.info.width
        height = map_msg.info.height
        res    = map_msg.info.resolution
        origin = map_msg.info.origin
        data   = np.asarray(map_msg.data, dtype=np.int8).reshape((height, width))

        self.get_logger().info(f'    Map: {width}×{height}, res={res:.3f}m')
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[data == 0] = 255

        if mask.sum() == 0:
            raise RuntimeError('No free-space cells found on the final map.')
        
        self.get_logger().info(f'    Free cells: {mask.sum()}, extracting contours...')

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if not contours or hierarchy is None:
            raise RuntimeError('Failed to extract free-space contours from the map.')

        self.get_logger().info(f'    Found {len(contours)} contour(s), filtering by area...')
        hierarchy = hierarchy[0]
        field_cells = []
        parent_to_cell = {}

        for idx, contour in enumerate(contours):
            area_px = cv2.contourArea(contour)
            area_m2 = area_px * (res ** 2)
            if area_m2 < self._min_contour_area:
                continue

            ring = self._contour_to_linear_ring(contour, width, height, origin, res, f2c)
            if ring is None or ring.size() < 4:
                continue

            parent = int(hierarchy[idx][3])
            if parent == -1:
                cell = f2c.Cell(ring)
                parent_to_cell[idx] = cell
                field_cells.append(cell)

        for idx, contour in enumerate(contours):
            parent = int(hierarchy[idx][3])
            if parent == -1 or parent not in parent_to_cell:
                continue

            area_px = cv2.contourArea(contour)
            area_m2 = area_px * (res ** 2)
            if area_m2 < self._min_contour_area:
                continue

            hole_ring = self._contour_to_linear_ring(contour, width, height, origin, res, f2c)
            if hole_ring is None or hole_ring.size() < 4:
                continue
            parent_to_cell[parent].addRing(hole_ring)

        if not field_cells:
            raise RuntimeError('Fields2Cover found no valid field polygons in the map.')

        self.get_logger().info(f'    Creating Fields2Cover Field with {len(field_cells)} cell(s)...')
        cells = f2c.Cells()
        for cell in field_cells:
            cells.addGeometry(cell)

        return f2c.Field(cells)

    def _contour_to_linear_ring(self, contour, width, height, origin, res, f2c):
        ring = f2c.LinearRing()
        for point in contour.squeeze(axis=1):
            px = int(point[0])
            py = int(point[1])
            world_row = height - 1 - py
            wx = origin.position.x + (px + 0.5) * res
            wy = origin.position.y + (world_row + 0.5) * res
            ring.addPoint(wx, wy)

        if not ring.isClosed():
            ring.closeRing()

        return ring

    def _convert_f2c_path_to_waypoints(self, path):
        waypoints = []
        for i in range(len(path)):
            state = path[i]
            point = state.point
            x = point.getX()
            y = point.getY()
            angle = getattr(state, 'angle', 0.0)
            waypoints.append(self._make_pose(x, y, angle))
        return waypoints

    def _make_pose(self, x, y, yaw=0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _is_within_map_bounds(self, wx, wy, origin, width, height, res):
        max_x = origin.position.x + width * res
        max_y = origin.position.y + height * res
        return (
            wx >= origin.position.x + 0.5 * res and
            wy >= origin.position.y + 0.5 * res and
            wx < max_x - 0.5 * res and
            wy < max_y - 0.5 * res
        )

    # ── Nav2 dispatch ──────────────────────────────────────────────────── #

    def _send_waypoints(self, waypoints):
        if not waypoints:
            self.get_logger().warn('No valid waypoints — nothing to do.')
            return

        self.get_logger().info('⏳ Waiting for /navigate_through_poses action server...')
        self._nav_client.wait_for_server()

        goal = NavigateThroughPoses.Goal()
        goal.poses = waypoints
        goal.behavior_tree = ''

        self.get_logger().info(f'🚀 Sending {len(waypoints)} waypoints to Nav2...')
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('❌ FollowWaypoints goal REJECTED.')
            return
        self.get_logger().info('✅ Goal ACCEPTED — coverage sweep underway!')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb):
        self.get_logger().info(
            f'  → poses remaining {fb.feedback.number_of_poses_remaining}, '
            f'distance remaining {fb.feedback.distance_remaining:.2f}m',
            throttle_duration_sec=5.0,
        )

    def _result_cb(self, future):
        result = future.result().result
        if result.error_code != NavigateThroughPoses.Result.NONE:
            self.get_logger().warn(
                f'⚠️  Coverage sweep ended with error_code={result.error_code}, '
                f'msg="{result.error_msg}"'
            )
        else:
            self.get_logger().info('🎉 Coverage sweep COMPLETE!')


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()