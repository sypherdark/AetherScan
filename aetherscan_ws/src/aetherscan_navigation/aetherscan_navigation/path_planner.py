"""3D Path Planner for AetherScan.

Implements RRT*-based planning in 3D space with path smoothing
for collision-free navigation between waypoints.
"""

import math
import random
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool


class RRTNode:
    __slots__ = ['position', 'parent', 'cost']

    def __init__(self, position: np.ndarray, parent=None, cost: float = 0.0):
        self.position = position
        self.parent = parent
        self.cost = cost


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('max_iterations', 2000)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('bounds_min', [-1.0, -1.0, 0.3])
        self.declare_parameter('bounds_max', [25.0, 20.0, 3.5])
        self.declare_parameter('smoothing_iterations', 50)

        self.step_size = self.get_parameter('step_size').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.bounds_min = np.array(self.get_parameter('bounds_min').value)
        self.bounds_max = np.array(self.get_parameter('bounds_max').value)
        self.smoothing_iterations = self.get_parameter('smoothing_iterations').value

        self.current_pose = None
        self.current_path: Optional[List[np.ndarray]] = None

        self.goal_sub = self.create_subscription(
            PoseStamped, '/aetherscan/navigation/goal', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/aetherscan/odom', self.odom_callback, 10
        )

        self.path_pub = self.create_publisher(Path, '/aetherscan/navigation/path', 10)
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '/aetherscan/navigation/current_waypoint', 10
        )
        self.path_marker_pub = self.create_publisher(
            Marker, '/aetherscan/navigation/path_marker', 10
        )

        self.waypoint_timer = self.create_timer(0.2, self.publish_next_waypoint)
        self.current_waypoint_idx = 0

        self.get_logger().info('Path Planner (RRT*) initialized')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_pose = np.array([pos.x, pos.y, pos.z])

    def goal_callback(self, msg: PoseStamped):
        """Plan a path to the received goal."""
        if self.current_pose is None:
            self.get_logger().warn('No current pose available, cannot plan')
            return

        goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        self.get_logger().info(
            f'Planning path to ({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})'
        )

        path = self._plan_rrt_star(self.current_pose, goal)

        if path is not None:
            smoothed = self._smooth_path(path)
            self.current_path = smoothed
            self.current_waypoint_idx = 0
            self._publish_path(smoothed)
            self.get_logger().info(f'Path found: {len(smoothed)} waypoints')
        else:
            self.get_logger().warn('No path found!')
            self.current_path = None

    def _plan_rrt_star(self, start: np.ndarray,
                       goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """Plan a path using RRT* algorithm."""
        start_node = RRTNode(start)
        nodes = [start_node]
        rewire_radius = self.step_size * 2.0

        for _ in range(self.max_iterations):
            if random.random() < 0.1:
                sample = goal
            else:
                sample = np.array([
                    random.uniform(self.bounds_min[0], self.bounds_max[0]),
                    random.uniform(self.bounds_min[1], self.bounds_max[1]),
                    random.uniform(self.bounds_min[2], self.bounds_max[2]),
                ])

            nearest = min(nodes, key=lambda n: np.linalg.norm(n.position - sample))
            direction = sample - nearest.position
            distance = np.linalg.norm(direction)

            if distance < 0.01:
                continue

            direction = direction / distance
            new_pos = nearest.position + direction * min(self.step_size, distance)

            if not self._is_valid(new_pos):
                continue

            new_cost = nearest.cost + np.linalg.norm(new_pos - nearest.position)
            new_node = RRTNode(new_pos, nearest, new_cost)

            nearby = [n for n in nodes
                      if np.linalg.norm(n.position - new_pos) < rewire_radius]

            for near_node in nearby:
                alt_cost = near_node.cost + np.linalg.norm(near_node.position - new_pos)
                if alt_cost < new_node.cost:
                    new_node.parent = near_node
                    new_node.cost = alt_cost

            nodes.append(new_node)

            for near_node in nearby:
                alt_cost = new_node.cost + np.linalg.norm(new_node.position - near_node.position)
                if alt_cost < near_node.cost:
                    near_node.parent = new_node
                    near_node.cost = alt_cost

            if np.linalg.norm(new_pos - goal) < self.goal_tolerance:
                path = []
                current = new_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return list(reversed(path))

        return None

    def _is_valid(self, position: np.ndarray) -> bool:
        """Check if position is within bounds (obstacle check simplified)."""
        if np.any(position < self.bounds_min) or np.any(position > self.bounds_max):
            return False
        return True

    def _smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Smooth path using shortcutting."""
        if len(path) <= 2:
            return path

        smoothed = list(path)

        for _ in range(self.smoothing_iterations):
            if len(smoothed) <= 2:
                break

            i = random.randint(0, len(smoothed) - 2)
            j = random.randint(i + 1, min(i + 5, len(smoothed) - 1))

            if self._is_valid_segment(smoothed[i], smoothed[j]):
                smoothed = smoothed[:i+1] + smoothed[j:]

        return smoothed

    def _is_valid_segment(self, start: np.ndarray, end: np.ndarray) -> bool:
        """Check if a straight segment is collision-free."""
        distance = np.linalg.norm(end - start)
        steps = max(2, int(distance / 0.1))

        for t in np.linspace(0, 1, steps):
            point = start + t * (end - start)
            if not self._is_valid(point):
                return False
        return True

    def _publish_path(self, path: List[np.ndarray]):
        """Publish path as nav_msgs/Path."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for point in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self._publish_path_marker(path)

    def _publish_path_marker(self, path: List[np.ndarray]):
        """Publish path as visual marker."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'planned_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 0.9

        for point in path:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def publish_next_waypoint(self):
        """Publish the current waypoint for the trajectory tracker."""
        if self.current_path is None or self.current_pose is None:
            return

        if self.current_waypoint_idx >= len(self.current_path):
            return

        waypoint = self.current_path[self.current_waypoint_idx]
        dist = np.linalg.norm(self.current_pose - waypoint)

        if dist < 0.4 and self.current_waypoint_idx < len(self.current_path) - 1:
            self.current_waypoint_idx += 1
            waypoint = self.current_path[self.current_waypoint_idx]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(waypoint[0])
        msg.pose.position.y = float(waypoint[1])
        msg.pose.position.z = float(waypoint[2])
        msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
