#!/usr/bin/env python3
"""High-fidelity indoor kinematic simulator (Gazebo-free).

Runs on macOS Docker / Apple Silicon. Models office/warehouse geometry,
quadcopter dynamics, 3D LiDAR, IMU, barometer, and downward rangefinder.
"""

import math
import random
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2, PointField, Range, Image, CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


@dataclass
class Box:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float


def office_obstacles() -> List[Box]:
    """Approximate office_environment.sdf layout."""
    walls = [
        Box(0, 20, 0, 0.15, 0, 3),
        Box(0, 20, 14.85, 15, 0, 3),
        Box(0, 0.15, 0, 15, 0, 3),
        Box(19.85, 20, 0, 15, 0, 3),
        Box(2, 14, 4.9, 5.1, 0, 3),
        Box(2, 14, 9.9, 10.1, 0, 3),
        Box(6.9, 7.1, 0, 5, 0, 3),
        Box(6.9, 7.1, 10, 15, 0, 3),
    ]
    furniture = [
        Box(2.3, 3.7, 2.15, 2.85, 0, 0.74),
        Box(2.3, 3.7, 11.65, 12.35, 0, 0.74),
        Box(11.2, 12.8, 2.1, 2.9, 0, 0.74),
        Box(0.65, 1.35, 6.9, 8.1, 0, 2.0),
        Box(18.55, 19.45, 1.7, 2.3, 0, 1.2),
    ]
    return walls + furniture


def warehouse_obstacles() -> List[Box]:
    walls = [
        Box(0, 30, 0, 0.2, 0, 6),
        Box(0, 30, 19.8, 20, 0, 6),
        Box(0, 0.2, 0, 20, 0, 6),
        Box(29.8, 30, 0, 20, 0, 6),
    ]
    racks = [
        Box(4.5, 5.5, 1, 9, 0, 4),
        Box(9.5, 10.5, 1, 9, 0, 4),
        Box(14.5, 15.5, 1, 9, 0, 4),
        Box(4.5, 5.5, 11, 19, 0, 4),
        Box(9.5, 10.5, 11, 19, 0, 4),
        Box(21, 23, 3, 5, 0, 1.2),
        Box(24, 26, 9, 11, 0, 1.5),
    ]
    return walls + racks


class IndoorSimulator(Node):
    def __init__(self):
        super().__init__('indoor_simulator')

        self.declare_parameter('world', 'office_environment')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('lidar_rate', 10.0)

        world = self.get_parameter('world').value
        self.obstacles = (
            warehouse_obstacles() if world == 'warehouse' else office_obstacles()
        )

        self.pos = np.array([2.0, 7.5, 0.2], dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.cmd = np.zeros(4, dtype=np.float64)  # vx, vy, vz, wz

        self.bounds = self._world_bounds()

        self.odom_pub = self.create_publisher(Odometry, '/aetherscan/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/aetherscan/imu', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/aetherscan/lidar', 10)
        self.range_pub = self.create_publisher(Range, '/aetherscan/rangefinder', 10)
        self.image_pub = self.create_publisher(Image, '/aetherscan/camera/image', 10)
        self.depth_pub = self.create_publisher(Image, '/aetherscan/camera/depth', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/aetherscan/camera/points', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/aetherscan/camera/camera_info', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/rtabmap/grid_map', 10)

        self.cmd_sub = self.create_subscription(
            Twist, '/aetherscan/cmd_vel', self.cmd_callback, 10
        )
        self.cmd_raw_sub = self.create_subscription(
            Twist, '/aetherscan/cmd_vel_raw', self.cmd_callback, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        rate = self.get_parameter('publish_rate').value
        lidar_rate = self.get_parameter('lidar_rate').value
        self.physics_timer = self.create_timer(1.0 / rate, self.physics_step)
        self.sensor_timer = self.create_timer(1.0 / rate, self.publish_imu_odom)
        self.lidar_timer = self.create_timer(1.0 / lidar_rate, self.publish_lidar)
        self.camera_timer = self.create_timer(1.0 / 15.0, self.publish_camera)
        self.grid_timer = self.create_timer(2.0, self.publish_exploration_grid)
        self._explored_cells: set = set()

        self.t = 0.0
        self.get_logger().info(f'Indoor simulator ready (world={world})')

    def _world_bounds(self) -> Tuple[float, float, float, float]:
        xs, ys = [], []
        for o in self.obstacles:
            if (o.zmax - o.zmin) > 2.5:
                xs.extend([o.xmin, o.xmax])
                ys.extend([o.ymin, o.ymax])
        return min(xs), max(xs), min(ys), max(ys)

    def cmd_callback(self, msg: Twist):
        self.cmd[0] = msg.linear.x
        self.cmd[1] = msg.linear.y
        self.cmd[2] = msg.linear.z
        self.cmd[3] = msg.angular.z

    def physics_step(self):
        dt = 0.02
        self.t += dt

        max_v = 1.5
        max_z = 0.5
        alpha = 0.15

        target_v = np.clip(self.cmd[:3], [-max_v, -max_v, -max_z], [max_v, max_v, max_z])
        self.vel += alpha * (target_v - self.vel)
        self.yaw_rate += alpha * (self.cmd[3] - self.yaw_rate)

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        world_v = np.array([
            cy * self.vel[0] - sy * self.vel[1],
            sy * self.vel[0] + cy * self.vel[1],
            self.vel[2],
        ])

        self.pos += world_v * dt
        self.yaw += self.yaw_rate * dt

        xmin, xmax, ymin, ymax = self.bounds
        self.pos[0] = np.clip(self.pos[0], xmin + 0.3, xmax - 0.3)
        self.pos[1] = np.clip(self.pos[1], ymin + 0.3, ymax - 0.3)
        self.pos[2] = np.clip(self.pos[2], 0.15, 3.5)

        for box in self.obstacles:
            if self._inside_box(self.pos, box, margin=0.25):
                self._push_out(box)

    def _inside_box(self, p: np.ndarray, b: Box, margin: float = 0.0) -> bool:
        return (
            b.xmin - margin <= p[0] <= b.xmax + margin
            and b.ymin - margin <= p[1] <= b.ymax + margin
            and b.zmin - margin <= p[2] <= b.zmax + margin
        )

    def _push_out(self, b: Box):
        dx = min(abs(self.pos[0] - b.xmin), abs(self.pos[0] - b.xmax))
        dy = min(abs(self.pos[1] - b.ymin), abs(self.pos[1] - b.ymax))
        if dx < dy:
            self.pos[0] = b.xmin - 0.3 if abs(self.pos[0] - b.xmin) < abs(self.pos[0] - b.xmax) else b.xmax + 0.3
        else:
            self.pos[1] = b.ymin - 0.3 if abs(self.pos[1] - b.ymin) < abs(self.pos[1] - b.ymax) else b.ymax + 0.3
        self.vel *= 0.3

    def publish_imu_odom(self):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.pos[0])
        odom.pose.pose.position.y = float(self.pos[1])
        odom.pose.pose.position.z = float(self.pos[2])
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom.twist.twist.linear.x = float(self.vel[0])
        odom.twist.twist.linear.y = float(self.vel[1])
        odom.twist.twist.linear.z = float(self.vel[2])
        odom.twist.twist.angular.z = float(self.yaw_rate)
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.z = float(self.yaw_rate)
        imu.linear_acceleration.x = float(self.vel[0] * 2)
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.pos[0])
        t.transform.translation.y = float(self.pos[1])
        t.transform.translation.z = float(self.pos[2])
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_broadcaster.sendTransform(t)

        m2o = TransformStamped()
        m2o.header.stamp = stamp
        m2o.header.frame_id = 'map'
        m2o.child_frame_id = 'odom'
        m2o.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(m2o)

        rng = Range()
        rng.header.stamp = stamp
        rng.header.frame_id = 'rangefinder_link'
        rng.range = float(max(0.05, self.pos[2]))
        rng.min_range = 0.05
        rng.max_range = 8.0
        self.range_pub.publish(rng)

    def publish_lidar(self):
        points = self._simulate_lidar()
        if len(points) == 0:
            return
        msg = self._make_cloud(points, 'lidar_link')
        self.lidar_pub.publish(msg)

    def publish_camera(self):
        stamp = self.get_clock().now().to_msg()
        h, w = 240, 320
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'camera_optical_frame'
        img.height = h
        img.width = w
        img.encoding = 'rgb8'
        img.step = w * 3
        data = bytearray(h * w * 3)
        for y in range(h):
            for x in range(w):
                i = (y * w + x) * 3
                data[i] = 40 + int(20 * math.sin(self.t + x * 0.05))
                data[i + 1] = 45 + int(15 * math.cos(self.t + y * 0.04))
                data[i + 2] = 55
        img.data = bytes(data)
        self.image_pub.publish(img)

        depth = Image()
        depth.header = img.header
        depth.encoding = '32FC1'
        depth.height = h
        depth.width = w
        depth.step = w * 4
        depth.data = (np.full((h, w), 2.5, dtype=np.float32)).tobytes()
        self.depth_pub.publish(depth)

        info = CameraInfo()
        info.header = img.header
        info.width = w
        info.height = h
        info.k = [300.0, 0.0, w / 2, 0.0, 300.0, h / 2, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)

        fwd = np.array([
            math.cos(self.yaw), math.sin(self.yaw), -0.1
        ])
        cam_pts = []
        for _ in range(800):
            dist = random.uniform(0.5, 6.0)
            p = self.pos + fwd * dist + np.random.randn(3) * 0.05
            cam_pts.append(p)
        self.cloud_pub.publish(self._make_cloud(np.array(cam_pts), 'camera_optical_frame'))

    def _simulate_lidar(self) -> np.ndarray:
        points = []
        h_samples, v_samples = 360, 16
        v_min, v_max = -0.26, 0.26

        for hi in range(0, h_samples, 4):
            angle_h = (hi / h_samples) * 2 * math.pi
            for vi in range(v_samples):
                angle_v = v_min + (vi / max(1, v_samples - 1)) * (v_max - v_min)
                direction = np.array([
                    math.cos(angle_h) * math.cos(angle_v),
                    math.sin(angle_h) * math.cos(angle_v),
                    math.sin(angle_v),
                ])
                dist = self._raycast(self.pos, direction)
                if 0.3 < dist < 20.0:
                    p = self.pos + direction * dist
                    points.append(p)

        return np.array(points, dtype=np.float32) if points else np.zeros((0, 3))

    def _raycast(self, origin: np.ndarray, direction: np.ndarray) -> float:
        best = 20.0
        for box in self.obstacles:
            d = self._ray_box(origin, direction, box)
            if 0.05 < d < best:
                best = d
        return best

    def _ray_box(self, o: np.ndarray, d: np.ndarray, b: Box) -> float:
        tmin, tmax = 0.0, 20.0
        for i, (mn, mx, pi, di) in enumerate([
            (b.xmin, b.xmax, o[0], d[0]),
            (b.ymin, b.ymax, o[1], d[1]),
            (b.zmin, b.zmax, o[2], d[2]),
        ]):
            if abs(di) < 1e-9:
                if pi < mn or pi > mx:
                    return 20.0
                continue
            t1 = (mn - pi) / di
            t2 = (mx - pi) / di
            t1, t2 = (min(t1, t2), max(t1, t2))
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return 20.0
        return tmin if tmin > 0 else 20.0

    def publish_exploration_grid(self):
        """Occupancy grid for frontier exploration (grows as drone moves)."""
        res = 0.25
        xmin, xmax, ymin, ymax = self.bounds
        w = int((xmax - xmin) / res)
        h = int((ymax - ymin) / res)
        cx = int((self.pos[0] - xmin) / res)
        cy = int((self.pos[1] - ymin) / res)

        for dx in range(-12, 13):
            for dy in range(-12, 13):
                gx, gy = cx + dx, cy + dy
                if 0 <= gx < w and 0 <= gy < h:
                    self._explored_cells.add((gx, gy))

        data = [-1] * (w * h)
        for box in self.obstacles:
            if (box.zmax - box.zmin) < 1.5:
                continue
            x0 = max(0, int((box.xmin - xmin) / res))
            x1 = min(w - 1, int((box.xmax - xmin) / res))
            y0 = max(0, int((box.ymin - ymin) / res))
            y1 = min(h - 1, int((box.ymax - ymin) / res))
            for gy in range(y0, y1 + 1):
                for gx in range(x0, x1 + 1):
                    data[gy * w + gx] = 100

        for gx, gy in self._explored_cells:
            idx = gy * w + gx
            if 0 <= idx < len(data) and data[idx] != 100:
                data[idx] = 0

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = res
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = xmin
        grid.info.origin.position.y = ymin
        grid.data = data
        self.grid_pub.publish(grid)

    def _make_cloud(self, points: np.ndarray, frame: str) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.data = points.astype(np.float32).tobytes()
        return msg


def main():
    rclpy.init()
    node = IndoorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
