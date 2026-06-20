"""Coverage planner node for AetherScan.

Generates systematic boustrophedon (lawnmower) or spiral coverage patterns
that adapt to discovered geometry, ensuring overlap for complete scanning.
"""

from __future__ import annotations

import enum
import math
import threading
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_group import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CoveragePattern(enum.Enum):
    BOUSTROPHEDON = 'boustrophedon'
    SPIRAL = 'spiral'


class CoveragePlanner(Node):
    """Generates systematic coverage paths for complete indoor scanning."""

    def __init__(self) -> None:
        super().__init__('coverage_planner')
        self._declare_parameters()

        self._overlap = self.get_parameter('coverage.overlap_ratio').value
        self._spacing = self.get_parameter('coverage.sweep_spacing').value
        self._altitudes = self.get_parameter('coverage.altitude_layers').value
        self._margin = self.get_parameter('coverage.boundary_margin').value
        self._pattern = CoveragePattern(self.get_parameter('coverage.pattern').value)
        self._speed = self.get_parameter('dynamics.exploration_speed').value

        self._lock = threading.Lock()
        self._occupancy_grid: Optional[OccupancyGrid] = None
        self._current_path: Optional[Path] = None
        self._plan_generated = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_cb = ReentrantCallbackGroup()
        timer_cb = MutuallyExclusiveCallbackGroup()

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback,
            map_qos, callback_group=sensor_cb,
        )

        self._path_pub = self.create_publisher(Path, '/coverage/planned_path', 10)
        self._waypoint_pub = self.create_publisher(PoseStamped, '/coverage/current_waypoint', 10)

        self._plan_timer = self.create_timer(
            5.0, self._plan_tick, callback_group=timer_cb,
        )

        self.get_logger().info(
            f'Coverage planner initialized (pattern={self._pattern.value}, '
            f'spacing={self._spacing}m, altitudes={self._altitudes})'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('coverage.overlap_ratio', 0.2)
        self.declare_parameter('coverage.sweep_spacing', 1.5)
        self.declare_parameter('coverage.altitude_layers', [1.0, 2.0, 3.0])
        self.declare_parameter('coverage.boundary_margin', 0.5)
        self.declare_parameter('coverage.pattern', 'boustrophedon')
        self.declare_parameter('dynamics.exploration_speed', 0.5)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _map_callback(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._occupancy_grid = msg
            self._plan_generated = False

    def _plan_tick(self) -> None:
        with self._lock:
            grid = self._occupancy_grid
            if grid is None or self._plan_generated:
                return

        bounds = self._compute_free_bounds(grid)
        if bounds is None:
            return

        if self._pattern == CoveragePattern.BOUSTROPHEDON:
            waypoints = self._generate_boustrophedon(bounds)
        else:
            waypoints = self._generate_spiral(bounds)

        path = self._waypoints_to_path(waypoints)

        with self._lock:
            self._current_path = path
            self._plan_generated = True

        self._path_pub.publish(path)
        self.get_logger().info(
            f'Coverage path generated: {len(path.poses)} waypoints'
        )

    # ------------------------------------------------------------------
    # Bounds extraction
    # ------------------------------------------------------------------

    def _compute_free_bounds(
        self, grid: OccupancyGrid,
    ) -> Optional[tuple[float, float, float, float]]:
        """Compute the axis-aligned bounding box of free space."""
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((h, w))
        free = np.argwhere((data >= 0) & (data < 35))

        if len(free) == 0:
            return None

        r_min, c_min = free.min(axis=0)
        r_max, c_max = free.max(axis=0)

        x_min = c_min * res + ox + self._margin
        x_max = c_max * res + ox - self._margin
        y_min = r_min * res + oy + self._margin
        y_max = r_max * res + oy - self._margin

        if x_max <= x_min or y_max <= y_min:
            return None

        return (x_min, y_min, x_max, y_max)

    # ------------------------------------------------------------------
    # Pattern generators
    # ------------------------------------------------------------------

    def _generate_boustrophedon(
        self, bounds: tuple[float, float, float, float],
    ) -> list[np.ndarray]:
        """Generate a lawnmower (boustrophedon) coverage pattern."""
        x_min, y_min, x_max, y_max = bounds
        effective_spacing = self._spacing * (1.0 - self._overlap)
        waypoints: list[np.ndarray] = []

        for altitude in self._altitudes:
            y = y_min
            sweep_idx = 0
            while y <= y_max:
                if sweep_idx % 2 == 0:
                    waypoints.append(np.array([x_min, y, altitude]))
                    waypoints.append(np.array([x_max, y, altitude]))
                else:
                    waypoints.append(np.array([x_max, y, altitude]))
                    waypoints.append(np.array([x_min, y, altitude]))
                y += effective_spacing
                sweep_idx += 1

        return waypoints

    def _generate_spiral(
        self, bounds: tuple[float, float, float, float],
    ) -> list[np.ndarray]:
        """Generate an inward spiral coverage pattern."""
        x_min, y_min, x_max, y_max = bounds
        effective_spacing = self._spacing * (1.0 - self._overlap)
        waypoints: list[np.ndarray] = []

        for altitude in self._altitudes:
            left, bottom, right, top = x_min, y_min, x_max, y_max

            while left < right and bottom < top:
                for x in np.arange(left, right, effective_spacing):
                    waypoints.append(np.array([x, bottom, altitude]))
                waypoints.append(np.array([right, bottom, altitude]))

                for y in np.arange(bottom + effective_spacing, top, effective_spacing):
                    waypoints.append(np.array([right, y, altitude]))
                waypoints.append(np.array([right, top, altitude]))

                for x in np.arange(right - effective_spacing, left, -effective_spacing):
                    waypoints.append(np.array([x, top, altitude]))
                waypoints.append(np.array([left, top, altitude]))

                for y in np.arange(top - effective_spacing, bottom + effective_spacing,
                                   -effective_spacing):
                    waypoints.append(np.array([left, y, altitude]))

                left += effective_spacing
                right -= effective_spacing
                bottom += effective_spacing
                top -= effective_spacing

            if left <= right and bottom <= top:
                cx = (left + right) / 2
                cy = (bottom + top) / 2
                waypoints.append(np.array([cx, cy, altitude]))

        return waypoints

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _waypoints_to_path(self, waypoints: list[np.ndarray]) -> Path:
        path = Path()
        path.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())

        for wp in waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position = Point(
                x=float(wp[0]), y=float(wp[1]), z=float(wp[2]),
            )
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down coverage planner')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
