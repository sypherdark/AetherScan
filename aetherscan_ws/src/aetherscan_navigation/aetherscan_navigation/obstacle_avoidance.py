"""Real-time Obstacle Avoidance for AetherScan.

Uses depth camera and LiDAR data to detect obstacles and modify
velocity commands to ensure safe flight using a Vector Field Histogram approach.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2, Range
from std_msgs.msg import Bool
import struct
import math


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('safety_distance', 0.8)
        self.declare_parameter('critical_distance', 0.3)
        self.declare_parameter('max_velocity', 1.5)
        self.declare_parameter('avoidance_gain', 2.0)
        self.declare_parameter('num_sectors', 36)
        self.declare_parameter('min_altitude', 0.4)
        self.declare_parameter('max_altitude', 3.5)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.avoidance_gain = self.get_parameter('avoidance_gain').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.min_altitude = self.get_parameter('min_altitude').value
        self.max_altitude = self.get_parameter('max_altitude').value

        self.histogram = np.zeros(self.num_sectors)
        self.current_altitude = 0.0
        self.emergency_stop = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/aetherscan/lidar', self.lidar_callback, sensor_qos
        )
        self.range_sub = self.create_subscription(
            Range, '/aetherscan/rangefinder', self.rangefinder_callback, sensor_qos
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/aetherscan/cmd_vel_raw', self.cmd_vel_callback, 10
        )

        self.safe_cmd_pub = self.create_publisher(
            Twist, '/aetherscan/cmd_vel', 10
        )
        self.emergency_pub = self.create_publisher(
            Bool, '/aetherscan/emergency_stop', 10
        )

        self.get_logger().info(
            f'Obstacle Avoidance active (safety={self.safety_distance}m, '
            f'critical={self.critical_distance}m)'
        )

    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR data to update obstacle histogram."""
        self.histogram = np.zeros(self.num_sectors)

        if msg.width == 0:
            return

        x_offset = y_offset = z_offset = 0
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        min_distance = float('inf')

        for i in range(min(msg.width, 5000)):
            offset = i * msg.point_step
            try:
                x = struct.unpack_from('f', msg.data, offset + x_offset)[0]
                y = struct.unpack_from('f', msg.data, offset + y_offset)[0]
                z = struct.unpack_from('f', msg.data, offset + z_offset)[0]
            except struct.error:
                continue

            if np.isnan(x) or np.isnan(y):
                continue

            if abs(z) > 1.5:
                continue

            distance = math.sqrt(x*x + y*y)
            if distance < 0.1 or distance > self.safety_distance * 2:
                continue

            min_distance = min(min_distance, distance)
            angle = math.atan2(y, x)
            sector = int((angle + math.pi) / (2 * math.pi) * self.num_sectors)
            sector = sector % self.num_sectors

            certainty = max(0, 1.0 - distance / (self.safety_distance * 2))
            self.histogram[sector] = max(self.histogram[sector], certainty)

        self.emergency_stop = min_distance < self.critical_distance

        emergency_msg = Bool()
        emergency_msg.data = self.emergency_stop
        self.emergency_pub.publish(emergency_msg)

    def rangefinder_callback(self, msg: Range):
        """Update current altitude from downward rangefinder."""
        self.current_altitude = msg.range

    def cmd_vel_callback(self, msg: Twist):
        """Process incoming velocity command with obstacle avoidance."""
        safe_cmd = Twist()

        if self.emergency_stop:
            self.safe_cmd_pub.publish(safe_cmd)
            return

        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z
        wz = msg.angular.z

        if abs(vx) > 0.01 or abs(vy) > 0.01:
            cmd_angle = math.atan2(vy, vx)
            cmd_speed = math.sqrt(vx*vx + vy*vy)

            cmd_sector = int((cmd_angle + math.pi) / (2 * math.pi) * self.num_sectors)
            cmd_sector = cmd_sector % self.num_sectors

            window = 3
            max_obstacle = 0
            for offset in range(-window, window + 1):
                idx = (cmd_sector + offset) % self.num_sectors
                max_obstacle = max(max_obstacle, self.histogram[idx])

            if max_obstacle > 0.5:
                best_sector = self._find_best_gap(cmd_sector)
                if best_sector is not None:
                    new_angle = (best_sector / self.num_sectors) * 2 * math.pi - math.pi
                    blend = max_obstacle
                    final_angle = cmd_angle * (1 - blend) + new_angle * blend
                    scale_factor = 1.0 - max_obstacle * 0.7
                    cmd_speed *= scale_factor

                    vx = cmd_speed * math.cos(final_angle)
                    vy = cmd_speed * math.sin(final_angle)
                else:
                    vx = 0.0
                    vy = 0.0

        if self.current_altitude < self.min_altitude and vz < 0:
            vz = 0.0
        elif self.current_altitude > self.max_altitude and vz > 0:
            vz = 0.0

        speed = math.sqrt(vx*vx + vy*vy)
        if speed > self.max_velocity:
            scale = self.max_velocity / speed
            vx *= scale
            vy *= scale

        safe_cmd.linear.x = vx
        safe_cmd.linear.y = vy
        safe_cmd.linear.z = vz
        safe_cmd.angular.z = wz
        self.safe_cmd_pub.publish(safe_cmd)

    def _find_best_gap(self, desired_sector: int) -> int:
        """Find the nearest open gap sector to the desired direction."""
        threshold = 0.3

        for offset in range(1, self.num_sectors // 2):
            for direction in [1, -1]:
                sector = (desired_sector + offset * direction) % self.num_sectors
                window_clear = True
                for w in range(-2, 3):
                    idx = (sector + w) % self.num_sectors
                    if self.histogram[idx] > threshold:
                        window_clear = False
                        break
                if window_clear:
                    return sector

        return None


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
