"""Point Cloud Assembler node for AetherScan SLAM pipeline.

Subscribes to depth camera and LiDAR point clouds, transforms them into the
map frame, and assembles a coherent 3D representation with voxel filtering.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
import struct


class PointCloudAssembler(Node):
    def __init__(self):
        super().__init__('point_cloud_assembler')

        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_points', 5000000)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 2.0)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.accumulated_points = np.empty((0, 4), dtype=np.float32)
        self.point_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.depth_sub = self.create_subscription(
            PointCloud2,
            '/aetherscan/camera/points',
            self.depth_cloud_callback,
            sensor_qos
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/aetherscan/lidar',
            self.lidar_cloud_callback,
            sensor_qos
        )

        self.map_pub = self.create_publisher(
            PointCloud2,
            '/aetherscan/map/point_cloud',
            10
        )

        self.stats_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_map
        )

        self.get_logger().info(
            f'Point Cloud Assembler initialized (voxel={self.voxel_size}m, '
            f'max_range={self.max_range}m)'
        )

    def depth_cloud_callback(self, msg: PointCloud2):
        self._process_cloud(msg, 'camera_optical_frame')

    def lidar_cloud_callback(self, msg: PointCloud2):
        self._process_cloud(msg, 'lidar_link')

    def _process_cloud(self, msg: PointCloud2, source_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        points = self._parse_pointcloud2(msg)
        if points is None or len(points) == 0:
            return

        distances = np.linalg.norm(points[:, :3], axis=1)
        mask = (distances >= self.min_range) & (distances <= self.max_range)
        points = points[mask]

        if len(points) == 0:
            return

        transformed = self._transform_points(points, transform)
        self._add_points(transformed)

    def _parse_pointcloud2(self, msg: PointCloud2) -> np.ndarray:
        """Parse PointCloud2 message into numpy array (x, y, z, intensity)."""
        if msg.width * msg.height == 0:
            return None

        points = []
        point_step = msg.point_step
        data = msg.data

        x_offset = y_offset = z_offset = 0
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        num_points = msg.width * msg.height
        for i in range(min(num_points, 10000)):
            offset = i * point_step
            x = struct.unpack_from('f', data, offset + x_offset)[0]
            y = struct.unpack_from('f', data, offset + y_offset)[0]
            z = struct.unpack_from('f', data, offset + z_offset)[0]

            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z, 1.0])

        return np.array(points, dtype=np.float32) if points else None

    def _transform_points(self, points: np.ndarray,
                          transform: TransformStamped) -> np.ndarray:
        """Apply TF transform to point array."""
        t = transform.transform.translation
        q = transform.transform.rotation

        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        rot_matrix = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float32)

        translated = points[:, :3] @ rot_matrix.T
        translated[:, 0] += t.x
        translated[:, 1] += t.y
        translated[:, 2] += t.z

        result = np.column_stack([translated, points[:, 3]])
        return result

    def _add_points(self, new_points: np.ndarray):
        """Add new points with voxel grid downsampling."""
        self.accumulated_points = np.vstack([self.accumulated_points, new_points])

        if len(self.accumulated_points) > self.max_points:
            self._voxel_downsample()

        self.point_count = len(self.accumulated_points)

    def _voxel_downsample(self):
        """Downsample accumulated points using voxel grid filter."""
        if len(self.accumulated_points) == 0:
            return

        voxel_indices = np.floor(
            self.accumulated_points[:, :3] / self.voxel_size
        ).astype(np.int32)

        _, unique_idx = np.unique(
            voxel_indices, axis=0, return_index=True
        )

        self.accumulated_points = self.accumulated_points[unique_idx]

    def publish_map(self):
        """Publish the accumulated point cloud map."""
        if len(self.accumulated_points) == 0:
            return

        msg = self._create_pointcloud2(self.accumulated_points)
        self.map_pub.publish(msg)

    def _create_pointcloud2(self, points: np.ndarray) -> PointCloud2:
        """Create PointCloud2 message from numpy array."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAssembler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
