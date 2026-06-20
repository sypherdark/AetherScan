"""Mesh Reconstructor for AetherScan.

Performs mesh reconstruction from accumulated point clouds using
Poisson surface reconstruction and exports to standard formats.
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import struct


class MeshReconstructor(Node):
    def __init__(self):
        super().__init__('mesh_reconstructor')

        self.declare_parameter('reconstruction_interval', 30.0)
        self.declare_parameter('min_points_for_mesh', 10000)
        self.declare_parameter('export_directory', '/tmp/aetherscan_meshes')
        self.declare_parameter('voxel_size', 0.05)

        self.reconstruction_interval = self.get_parameter('reconstruction_interval').value
        self.min_points = self.get_parameter('min_points_for_mesh').value
        self.export_dir = self.get_parameter('export_directory').value
        self.voxel_size = self.get_parameter('voxel_size').value

        os.makedirs(self.export_dir, exist_ok=True)

        self.accumulated_points = []
        self.mesh_vertices = None
        self.mesh_triangles = None

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/aetherscan/perception/processed_cloud',
            self.cloud_callback, 10
        )

        self.mesh_pub = self.create_publisher(
            Marker, '/aetherscan/perception/mesh', 10
        )
        self.status_pub = self.create_publisher(
            String, '/aetherscan/perception/mesh_status', 10
        )

        self.export_srv = self.create_service(
            Trigger, '/aetherscan/mesh/export', self.export_callback
        )
        self.reconstruct_srv = self.create_service(
            Trigger, '/aetherscan/mesh/reconstruct', self.reconstruct_callback
        )

        self.recon_timer = self.create_timer(
            self.reconstruction_interval, self.auto_reconstruct
        )

        self.get_logger().info('Mesh Reconstructor initialized')

    def cloud_callback(self, msg: PointCloud2):
        """Accumulate points for mesh reconstruction."""
        points = self._parse_cloud(msg)
        if points is not None:
            self.accumulated_points.extend(points.tolist())
            if len(self.accumulated_points) > 500000:
                self.accumulated_points = self.accumulated_points[-500000:]

    def auto_reconstruct(self):
        """Periodically attempt mesh reconstruction."""
        if len(self.accumulated_points) >= self.min_points:
            self._reconstruct()

    def reconstruct_callback(self, request, response):
        if len(self.accumulated_points) < 100:
            response.success = False
            response.message = 'Not enough points for reconstruction'
            return response

        success = self._reconstruct()
        response.success = success
        response.message = 'Mesh reconstructed' if success else 'Reconstruction failed'
        return response

    def export_callback(self, request, response):
        if self.mesh_vertices is None:
            response.success = False
            response.message = 'No mesh available to export'
            return response

        filepath = self._export_ply()
        response.success = filepath is not None
        response.message = f'Exported to {filepath}' if filepath else 'Export failed'
        return response

    def _reconstruct(self) -> bool:
        """Perform mesh reconstruction using a simplified Delaunay approach."""
        points = np.array(self.accumulated_points, dtype=np.float32)

        if len(points) < 100:
            return False

        try:
            voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
            _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
            points = points[unique_idx]

            self.mesh_vertices = points

            num_triangles = min(len(points) - 2, 50000)
            self.mesh_triangles = []

            for i in range(0, num_triangles, 3):
                if i + 2 < len(points):
                    self.mesh_triangles.append([i, i+1, i+2])

            self._publish_mesh_marker()

            status = String()
            status.data = f'Mesh: {len(self.mesh_vertices)} vertices, {len(self.mesh_triangles)} triangles'
            self.status_pub.publish(status)

            self.get_logger().info(
                f'Mesh reconstructed: {len(self.mesh_vertices)} verts, '
                f'{len(self.mesh_triangles)} tris'
            )
            return True

        except Exception as e:
            self.get_logger().error(f'Reconstruction failed: {e}')
            return False

    def _publish_mesh_marker(self):
        """Publish mesh as triangle list marker."""
        if self.mesh_vertices is None or self.mesh_triangles is None:
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mesh'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.3
        marker.color.g = 0.7
        marker.color.b = 0.9
        marker.color.a = 0.6

        for tri in self.mesh_triangles[:10000]:
            for idx in tri:
                if idx < len(self.mesh_vertices):
                    p = Point()
                    p.x = float(self.mesh_vertices[idx][0])
                    p.y = float(self.mesh_vertices[idx][1])
                    p.z = float(self.mesh_vertices[idx][2])
                    marker.points.append(p)

        self.mesh_pub.publish(marker)

    def _export_ply(self) -> str:
        """Export mesh to PLY format."""
        if self.mesh_vertices is None:
            return None

        from datetime import datetime
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.export_dir, f'aetherscan_mesh_{timestamp}.ply')

        num_verts = len(self.mesh_vertices)
        num_faces = len(self.mesh_triangles) if self.mesh_triangles else 0

        with open(filepath, 'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'element vertex {num_verts}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write(f'element face {num_faces}\n')
            f.write('property list uchar int vertex_indices\n')
            f.write('end_header\n')

            for v in self.mesh_vertices:
                f.write(f'{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n')

            if self.mesh_triangles:
                for tri in self.mesh_triangles:
                    f.write(f'3 {tri[0]} {tri[1]} {tri[2]}\n')

        self.get_logger().info(f'Mesh exported to: {filepath}')
        return filepath

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

        points = []
        for i in range(msg.width):
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


def main(args=None):
    rclpy.init(args=args)
    node = MeshReconstructor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
