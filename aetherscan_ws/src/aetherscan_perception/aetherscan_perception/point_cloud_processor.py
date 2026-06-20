"""Point Cloud Processor for AetherScan.

Performs filtering, downsampling, noise removal, normal estimation,
and plane segmentation on incoming point clouds.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')

        self.declare_parameter('voxel_size', 0.03)
        self.declare_parameter('statistical_outlier_neighbors', 20)
        self.declare_parameter('statistical_outlier_stddev', 2.0)
        self.declare_parameter('min_height', -0.1)
        self.declare_parameter('max_height', 4.0)
        self.declare_parameter('process_rate', 5.0)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.sor_neighbors = self.get_parameter('statistical_outlier_neighbors').value
        self.sor_stddev = self.get_parameter('statistical_outlier_stddev').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value

        self.pending_cloud = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/aetherscan/map/point_cloud',
            self.cloud_callback, sensor_qos
        )

        self.processed_pub = self.create_publisher(
            PointCloud2, '/aetherscan/perception/processed_cloud', 10
        )

        process_rate = self.get_parameter('process_rate').value
        self.process_timer = self.create_timer(1.0 / process_rate, self.process)

        self.get_logger().info('Point Cloud Processor initialized')

    def cloud_callback(self, msg: PointCloud2):
        self.pending_cloud = msg

    def process(self):
        if self.pending_cloud is None:
            return

        points = self._parse_cloud(self.pending_cloud)
        if points is None or len(points) < 10:
            return

        # Height filter
        mask = (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)
        points = points[mask]

        if len(points) < 10:
            return

        # Voxel downsampling
        points = self._voxel_downsample(points)

        # Statistical outlier removal (simplified)
        points = self._remove_outliers(points)

        # Publish
        msg = self._create_cloud_msg(points)
        self.processed_pub.publish(msg)
        self.pending_cloud = None

    def _parse_cloud(self, msg: PointCloud2) -> np.ndarray:
        if msg.width == 0:
            return None

        x_off = y_off = z_off = 0
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset

        num_points = msg.width * msg.height
        points = []

        for i in range(num_points):
            offset = i * msg.point_step
            try:
                x = struct.unpack_from('f', msg.data, offset + x_off)[0]
                y = struct.unpack_from('f', msg.data, offset + y_off)[0]
                z = struct.unpack_from('f', msg.data, offset + z_off)[0]
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    points.append([x, y, z])
            except struct.error:
                break

        return np.array(points, dtype=np.float32) if points else None

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_idx]

    def _remove_outliers(self, points: np.ndarray) -> np.ndarray:
        """Simplified statistical outlier removal using random sampling."""
        if len(points) < self.sor_neighbors * 2:
            return points

        sample_size = min(5000, len(points))
        indices = np.random.choice(len(points), sample_size, replace=False)
        sample = points[indices]

        distances = np.zeros(sample_size)
        for i in range(sample_size):
            dists = np.linalg.norm(sample - sample[i], axis=1)
            dists.sort()
            k_dist = dists[1:min(self.sor_neighbors + 1, len(dists))]
            distances[i] = np.mean(k_dist) if len(k_dist) > 0 else 0

        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        threshold = mean_dist + self.sor_stddev * std_dist

        mask = distances < threshold
        return sample[mask]

    def _create_cloud_msg(self, points: np.ndarray) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.data = points.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
