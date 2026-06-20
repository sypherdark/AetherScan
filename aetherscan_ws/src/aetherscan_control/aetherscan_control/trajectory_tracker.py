"""Trajectory Tracker for AetherScan.

Receives a nav_msgs/Path and generates smooth trajectory setpoints
for the flight controller, with minimum-snap-like profile generation.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        self.declare_parameter('cruise_speed', 0.8)
        self.declare_parameter('approach_speed', 0.3)
        self.declare_parameter('waypoint_reach_threshold', 0.4)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('tracking_rate', 20.0)

        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.waypoint_threshold = self.get_parameter('waypoint_reach_threshold').value
        self.lookahead = self.get_parameter('lookahead_distance').value

        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.tracking_active = False

        self.path_sub = self.create_subscription(
            Path, '/aetherscan/navigation/path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/aetherscan/odom', self.odom_callback, 10
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/aetherscan/navigation/current_waypoint', 10
        )
        self.progress_pub = self.create_publisher(
            Float32, '/aetherscan/trajectory/progress', 10
        )

        rate = self.get_parameter('tracking_rate').value
        self.track_timer = self.create_timer(1.0 / rate, self.tracking_step)

        self.get_logger().info('Trajectory Tracker initialized')

    def path_callback(self, msg: Path):
        """Receive new path to follow."""
        if len(msg.poses) < 2:
            return

        self.current_path = msg.poses
        self.current_waypoint_idx = 0
        self.tracking_active = True
        self.get_logger().info(f'Tracking new path with {len(msg.poses)} waypoints')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_pose = np.array([pos.x, pos.y, pos.z])

    def tracking_step(self):
        """Generate next setpoint along the trajectory."""
        if not self.tracking_active or self.current_path is None:
            return
        if self.current_pose is None:
            return

        path = self.current_path

        while self.current_waypoint_idx < len(path) - 1:
            wp = path[self.current_waypoint_idx].pose.position
            wp_pos = np.array([wp.x, wp.y, wp.z])
            dist = np.linalg.norm(self.current_pose - wp_pos)

            if dist < self.waypoint_threshold:
                self.current_waypoint_idx += 1
            else:
                break

        if self.current_waypoint_idx >= len(path):
            self.tracking_active = False
            self.get_logger().info('Path tracking complete')
            return

        target_wp = path[self.current_waypoint_idx].pose.position
        target_pos = np.array([target_wp.x, target_wp.y, target_wp.z])

        lookahead_pos = target_pos
        if self.current_waypoint_idx < len(path) - 1:
            next_wp = path[min(self.current_waypoint_idx + 1, len(path) - 1)].pose.position
            next_pos = np.array([next_wp.x, next_wp.y, next_wp.z])
            direction = next_pos - target_pos
            norm = np.linalg.norm(direction)
            if norm > 0.01:
                lookahead_pos = target_pos + direction / norm * min(self.lookahead, norm)

        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'
        setpoint.pose.position.x = float(lookahead_pos[0])
        setpoint.pose.position.y = float(lookahead_pos[1])
        setpoint.pose.position.z = float(lookahead_pos[2])

        direction = target_pos - self.current_pose
        yaw = math.atan2(direction[1], direction[0])
        setpoint.pose.orientation.z = math.sin(yaw / 2)
        setpoint.pose.orientation.w = math.cos(yaw / 2)

        self.setpoint_pub.publish(setpoint)

        progress_msg = Float32()
        progress_msg.data = float(self.current_waypoint_idx) / max(1, len(path) - 1)
        self.progress_pub.publish(progress_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
