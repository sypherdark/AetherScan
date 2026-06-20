"""Frontier-based Exploration for AetherScan.

Implements frontier detection in 3D occupancy space to guide autonomous
exploration. Selects next-best-viewpoint based on information gain,
distance cost, and safety constraints.
"""

import math
from enum import IntEnum
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Float32, String, Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


class ExplorationState(IntEnum):
    IDLE = 0
    EXPLORING = 1
    NAVIGATING_TO_FRONTIER = 2
    SCANNING = 3
    RETURNING_HOME = 4
    COMPLETE = 5


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('scan_altitude', 1.5)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_radius', 15.0)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('information_gain_weight', 1.0)
        self.declare_parameter('distance_weight', 0.3)
        self.declare_parameter('coverage_threshold', 0.95)
        self.declare_parameter('frontier_update_rate', 1.0)
        self.declare_parameter('home_position', [0.0, 0.0, 0.2])

        self.scan_altitude = self.get_parameter('scan_altitude').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.info_gain_weight = self.get_parameter('information_gain_weight').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value
        self.home_position = self.get_parameter('home_position').value

        self.state = ExplorationState.IDLE
        self.current_pose = None
        self.current_target = None
        self.frontiers: List[np.ndarray] = []
        self.visited_positions: List[np.ndarray] = []
        self.total_explored_cells = 0
        self.total_cells = 0
        self.start_time = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/aetherscan/odom', self.odom_callback, sensor_qos
        )
        self.grid_sub = self.create_subscription(
            OccupancyGrid, '/rtabmap/grid_map', self.grid_callback, 10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped, '/aetherscan/navigation/goal', 10
        )
        self.path_pub = self.create_publisher(
            Path, '/aetherscan/exploration/path', 10
        )
        self.frontier_pub = self.create_publisher(
            MarkerArray, '/aetherscan/exploration/frontiers', 10
        )
        self.state_pub = self.create_publisher(
            String, '/aetherscan/exploration/state', 10
        )
        self.coverage_pub = self.create_publisher(
            Float32, '/aetherscan/exploration/coverage', 10
        )
        self.enable_sub = self.create_subscription(
            Bool, '/aetherscan/exploration/enable', self.enable_callback, 10
        )

        self.exploration_timer = self.create_timer(
            1.0 / self.get_parameter('frontier_update_rate').value,
            self.exploration_step
        )

        self.get_logger().info('Frontier Explorer initialized')

    def enable_callback(self, msg: Bool):
        """Enable/disable autonomous exploration."""
        if msg.data and self.state == ExplorationState.IDLE:
            self.state = ExplorationState.EXPLORING
            self.start_time = self.get_clock().now()
            self.get_logger().info('Exploration ENABLED')
        elif not msg.data:
            self.state = ExplorationState.IDLE
            self.get_logger().info('Exploration DISABLED')

    def odom_callback(self, msg: Odometry):
        """Update current drone position."""
        pos = msg.pose.pose.position
        self.current_pose = np.array([pos.x, pos.y, pos.z])

    def grid_callback(self, msg: OccupancyGrid):
        """Process occupancy grid to detect frontiers."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid = np.array(msg.data).reshape((height, width))

        self.total_cells = np.sum(grid >= 0)
        self.total_explored_cells = np.sum(grid != -1)

        self.frontiers = self._detect_frontiers(
            grid, width, height, resolution, origin_x, origin_y
        )

    def _detect_frontiers(self, grid: np.ndarray, width: int, height: int,
                          resolution: float, origin_x: float,
                          origin_y: float) -> List[np.ndarray]:
        """Detect frontier cells (boundary between free and unknown)."""
        frontiers = []
        frontier_cells = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if grid[y, x] != 0:
                    continue

                neighbors = grid[y-1:y+2, x-1:x+2]
                if np.any(neighbors == -1):
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    frontier_cells.append([world_x, world_y])

        if not frontier_cells:
            return []

        frontier_array = np.array(frontier_cells)
        clusters = self._cluster_frontiers(frontier_array, resolution * 3)

        for cluster in clusters:
            if len(cluster) >= self.min_frontier_size:
                centroid = np.mean(cluster, axis=0)
                frontiers.append(centroid)

        return frontiers

    def _cluster_frontiers(self, points: np.ndarray,
                           threshold: float) -> List[np.ndarray]:
        """Simple distance-based clustering of frontier points."""
        if len(points) == 0:
            return []

        clusters = []
        visited = set()

        for i in range(len(points)):
            if i in visited:
                continue

            cluster = [points[i]]
            visited.add(i)
            queue = [i]

            while queue:
                current = queue.pop(0)
                distances = np.linalg.norm(points - points[current], axis=1)

                for j in range(len(points)):
                    if j not in visited and distances[j] < threshold:
                        visited.add(j)
                        cluster.append(points[j])
                        queue.append(j)

                        if len(cluster) > 100:
                            break
                if len(cluster) > 100:
                    break

            clusters.append(np.array(cluster))

        return clusters

    def exploration_step(self):
        """Main exploration logic executed periodically."""
        state_msg = String()
        state_msg.data = ExplorationState(self.state).name
        self.state_pub.publish(state_msg)

        coverage = self._calculate_coverage()
        coverage_msg = Float32()
        coverage_msg.data = coverage
        self.coverage_pub.publish(coverage_msg)

        self._publish_frontier_markers()

        if self.state == ExplorationState.IDLE:
            return

        if self.current_pose is None:
            return

        if coverage >= self.coverage_threshold:
            self.state = ExplorationState.RETURNING_HOME
            self._navigate_to_home()
            return

        if self.state == ExplorationState.EXPLORING:
            target = self._select_next_frontier()
            if target is not None:
                self.current_target = target
                self.state = ExplorationState.NAVIGATING_TO_FRONTIER
                self._publish_goal(target)
            else:
                if len(self.frontiers) == 0:
                    self.state = ExplorationState.RETURNING_HOME
                    self._navigate_to_home()

        elif self.state == ExplorationState.NAVIGATING_TO_FRONTIER:
            if self.current_target is not None and self.current_pose is not None:
                dist = np.linalg.norm(self.current_pose[:2] - self.current_target)
                if dist < 0.5:
                    self.visited_positions.append(self.current_target.copy())
                    self.state = ExplorationState.EXPLORING

        elif self.state == ExplorationState.RETURNING_HOME:
            if self.current_pose is not None:
                home = np.array(self.home_position[:2])
                dist = np.linalg.norm(self.current_pose[:2] - home)
                if dist < 0.5:
                    self.state = ExplorationState.COMPLETE
                    self.get_logger().info('Exploration COMPLETE!')

    def _select_next_frontier(self) -> Optional[np.ndarray]:
        """Select the best frontier based on information gain and distance."""
        if not self.frontiers or self.current_pose is None:
            return None

        best_score = -float('inf')
        best_frontier = None

        for frontier in self.frontiers:
            distance = np.linalg.norm(self.current_pose[:2] - frontier)

            if distance < 0.3 or distance > self.exploration_radius:
                continue

            too_close_to_visited = False
            for visited in self.visited_positions:
                if np.linalg.norm(visited - frontier) < 1.0:
                    too_close_to_visited = True
                    break
            if too_close_to_visited:
                continue

            info_gain = 1.0
            distance_cost = distance / self.exploration_radius
            score = (self.info_gain_weight * info_gain -
                     self.distance_weight * distance_cost)

            if score > best_score:
                best_score = score
                best_frontier = frontier

        return best_frontier

    def _publish_goal(self, target: np.ndarray):
        """Publish navigation goal."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])
        goal.pose.position.z = self.scan_altitude
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def _navigate_to_home(self):
        """Send drone back to home position."""
        home = np.array(self.home_position[:2])
        self._publish_goal(home)
        self.get_logger().info('Returning to home position')

    def _calculate_coverage(self) -> float:
        """Calculate exploration coverage percentage."""
        if self.total_cells == 0:
            return 0.0
        return min(1.0, self.total_explored_cells / max(1, self.total_cells))

    def _publish_frontier_markers(self):
        """Publish frontier visualization markers."""
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, frontier in enumerate(self.frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(frontier[0])
            marker.pose.position.y = float(frontier[1])
            marker.pose.position.z = self.scan_altitude
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.8
            marker.color.a = 0.8
            marker.lifetime.sec = 2
            marker_array.markers.append(marker)

        self.frontier_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
