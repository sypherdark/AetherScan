"""Map Manager node for AetherScan.

Manages the 3D map lifecycle: saving, loading, statistics, and optimization.
"""

import os
import json
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32
from sensor_msgs.msg import PointCloud2
import struct


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')

        self.declare_parameter('save_directory', '/tmp/aetherscan_maps')
        self.declare_parameter('auto_save_interval', 60.0)
        self.declare_parameter('map_frame', 'map')

        self.save_dir = self.get_parameter('save_directory').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value
        os.makedirs(self.save_dir, exist_ok=True)

        self.total_points = 0
        self.map_bounds_min = np.array([float('inf')] * 3)
        self.map_bounds_max = np.array([float('-inf')] * 3)
        self.last_update_time = self.get_clock().now()

        self.map_sub = self.create_subscription(
            PointCloud2,
            '/aetherscan/map/point_cloud',
            self.map_callback,
            10
        )

        self.stats_pub = self.create_publisher(String, '/aetherscan/map/stats', 10)
        self.coverage_pub = self.create_publisher(Float32, '/aetherscan/map/coverage_area', 10)

        self.save_srv = self.create_service(
            Trigger, '/aetherscan/map/save', self.save_map_callback
        )
        self.reset_srv = self.create_service(
            Trigger, '/aetherscan/map/reset', self.reset_map_callback
        )

        self.stats_timer = self.create_timer(2.0, self.publish_stats)
        if self.auto_save_interval > 0:
            self.auto_save_timer = self.create_timer(
                self.auto_save_interval, self.auto_save
            )

        self.current_cloud_data = None
        self.get_logger().info(f'Map Manager initialized. Save dir: {self.save_dir}')

    def map_callback(self, msg: PointCloud2):
        """Update map statistics from incoming point cloud."""
        self.total_points = msg.width * msg.height
        self.current_cloud_data = msg.data
        self.last_update_time = self.get_clock().now()

        self._update_bounds(msg)

    def _update_bounds(self, msg: PointCloud2):
        """Update map bounding box from point cloud."""
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

        sample_size = min(1000, msg.width)
        step = max(1, msg.width // sample_size)

        for i in range(0, msg.width, step):
            offset = i * msg.point_step
            try:
                x = struct.unpack_from('f', msg.data, offset + x_offset)[0]
                y = struct.unpack_from('f', msg.data, offset + y_offset)[0]
                z = struct.unpack_from('f', msg.data, offset + z_offset)[0]

                self.map_bounds_min = np.minimum(
                    self.map_bounds_min, [x, y, z]
                )
                self.map_bounds_max = np.maximum(
                    self.map_bounds_max, [x, y, z]
                )
            except struct.error:
                break

    def publish_stats(self):
        """Publish map statistics."""
        if np.any(np.isinf(self.map_bounds_min)):
            area = 0.0
            volume = 0.0
        else:
            dims = self.map_bounds_max - self.map_bounds_min
            area = float(dims[0] * dims[1])
            volume = float(dims[0] * dims[1] * dims[2])

        stats = {
            'total_points': self.total_points,
            'area_m2': round(area, 2),
            'volume_m3': round(volume, 2),
            'bounds_min': self.map_bounds_min.tolist() if not np.any(np.isinf(self.map_bounds_min)) else [0, 0, 0],
            'bounds_max': self.map_bounds_max.tolist() if not np.any(np.isinf(self.map_bounds_max)) else [0, 0, 0],
            'timestamp': datetime.now().isoformat()
        }

        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)

        coverage_msg = Float32()
        coverage_msg.data = float(area)
        self.coverage_pub.publish(coverage_msg)

    def save_map_callback(self, request, response):
        """Service callback to save current map."""
        success = self._save_map()
        response.success = success
        response.message = 'Map saved successfully' if success else 'No map data to save'
        return response

    def reset_map_callback(self, request, response):
        """Service callback to reset the map."""
        self.total_points = 0
        self.map_bounds_min = np.array([float('inf')] * 3)
        self.map_bounds_max = np.array([float('-inf')] * 3)
        self.current_cloud_data = None
        response.success = True
        response.message = 'Map reset successfully'
        self.get_logger().info('Map has been reset')
        return response

    def _save_map(self) -> bool:
        """Save current map data to disk."""
        if self.current_cloud_data is None:
            return False

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.save_dir, f'aetherscan_map_{timestamp}.bin')

        with open(filepath, 'wb') as f:
            f.write(self.current_cloud_data)

        meta_path = os.path.join(self.save_dir, f'aetherscan_map_{timestamp}.json')
        meta = {
            'timestamp': timestamp,
            'total_points': self.total_points,
            'bounds_min': self.map_bounds_min.tolist() if not np.any(np.isinf(self.map_bounds_min)) else None,
            'bounds_max': self.map_bounds_max.tolist() if not np.any(np.isinf(self.map_bounds_max)) else None,
        }
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(f'Map saved: {filepath} ({self.total_points} points)')
        return True

    def auto_save(self):
        """Periodic auto-save of the map."""
        if self.current_cloud_data is not None:
            self._save_map()


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
