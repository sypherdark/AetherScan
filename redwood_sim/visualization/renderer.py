"""
Cinematic Open3D rendering — PBR materials, dynamic lighting, soft shadows (when supported).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
import open3d as o3d

try:
    import open3d.visualization.gui as gui
    import open3d.visualization.rendering as rendering

    HAS_FILAMENT = True
except ImportError:
    HAS_FILAMENT = False

from core.math3d import quat_to_rotation_matrix
from core.physics import RigidBodyState
from scene_loader import RedwoodScene


def _lineset_from_segments(segments: np.ndarray, rgb: Tuple[float, float, float]) -> o3d.geometry.LineSet:
    segments = np.asarray(segments, dtype=np.float64)
    points = segments.reshape(-1, 3)
    n_seg = len(segments)
    lines = np.array([[2 * i, 2 * i + 1] for i in range(n_seg)], dtype=np.int32)
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines)
    colors = np.tile(np.asarray(rgb, dtype=np.float64), (len(points), 1))
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


class SimulationRenderer:
    """Filament PBR renderer with legacy Visualizer fallback."""

    def __init__(self, scene: RedwoodScene, width: int = 1600, height: int = 1000):
        self.scene = scene
        self.width = width
        self.height = height
        self._use_filament = HAS_FILAMENT
        self._running = True
        self._path_points: List[np.ndarray] = []
        self._show_lidar = True
        self._show_patrol = True

        if self._use_filament:
            try:
                self._init_filament()
            except Exception as exc:
                print(f"Filament PBR unavailable ({exc}); using legacy Open3D visualizer.")
                self._use_filament = False
                self._init_legacy()
        else:
            self._init_legacy()

    def _pbr_material(self) -> "rendering.MaterialRecord":
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        mat.base_color = [0.78, 0.76, 0.72, 1.0]
        mat.base_metallic = 0.05
        mat.base_roughness = 0.62
        mat.base_reflectance = 0.35
        if hasattr(mat, "clear_coat"):
            mat.clear_coat = 0.15
            mat.clear_coat_roughness = 0.08
        return mat

    def _init_filament(self) -> None:
        self.app = gui.Application.instance
        self.app.initialize()
        self.window = self.app.create_window(
            "AetherScan — Indoor Autonomous Quadcopter", self.width, self.height
        )
        self.widget = gui.SceneWidget()
        self.widget.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.widget)
        self.window.set_on_close(self._on_close)

        self.widget.scene.set_background([0.04, 0.05, 0.08, 1.0])
        scene = self.widget.scene.scene
        if hasattr(self.widget.scene, "set_ambient_occlusion"):
            self.widget.scene.set_ambient_occlusion(True)
        if hasattr(self.widget.scene, "set_antialiasing"):
            self.widget.scene.set_antialiasing(True, rendering.AntiAliasingMode.FXAA)
        if hasattr(scene, "set_indirect_light_intensity"):
            scene.set_indirect_light_intensity(25000.0)
        if hasattr(scene, "set_sun_light"):
            scene.set_sun_light([0.4, -0.6, -0.75], [1.0, 0.96, 0.9], 75000)
            scene.enable_sun_light(True)
        if hasattr(scene, "enable_soft_shadows"):
            scene.enable_soft_shadows(True)

        self._mesh = o3d.geometry.TriangleMesh(self.scene.mesh)
        self._mesh.compute_vertex_normals()
        self.widget.scene.add_geometry("environment", self._mesh, self._pbr_material())

        self._drone_axes = _lineset_from_segments(
            np.array([[[0, 0, 0], [0.28, 0, 0]]], dtype=np.float64), (1.0, 0.25, 0.2)
        )
        self.widget.scene.add_geometry("drone", self._drone_axes)

        bounds = self.scene.bounds
        center = bounds.center
        self.widget.setup_camera(55.0, center, center + np.array([0, 0, 1.0]), [0, 0, 1])

    def _init_legacy(self) -> None:
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("AetherScan", self.width, self.height)
        self._mesh = o3d.geometry.TriangleMesh(self.scene.mesh)
        n = len(self._mesh.vertices)
        if n:
            self._mesh.vertex_colors = o3d.utility.Vector3dVector(
                np.tile([0.78, 0.76, 0.72], (n, 1))
            )
        self.vis.add_geometry(self._mesh)
        self._drone_axes = o3d.geometry.LineSet()
        self.vis.add_geometry(self._drone_axes)
        self._lidar_lines = o3d.geometry.LineSet()
        self._patrol_lines = o3d.geometry.LineSet()
        self._path_lines = o3d.geometry.LineSet()
        self.vis.add_geometry(self._lidar_lines)
        self.vis.add_geometry(self._patrol_lines)
        self.vis.add_geometry(self._path_lines)
        opt = self.vis.get_render_option()
        opt.background_color = np.array([0.04, 0.05, 0.08])
        opt.mesh_show_back_face = True
        opt.light_on = True

    def _on_close(self) -> bool:
        self._running = False
        return True

    def _update_drone_axes(self, state: RigidBodyState) -> None:
        R = quat_to_rotation_matrix(state.quaternion)
        local = np.array(
            [
                [0, 0, 0], [0.28, 0, 0],
                [0, 0, 0], [0, 0.28, 0],
                [0, 0, 0], [0, 0, 0.28],
            ],
            dtype=np.float64,
        )
        world = (R @ local.T).T + state.position
        segs = world.reshape(3, 2, 3)
        colors = [(1, 0.2, 0.2), (0.2, 1, 0.2), (0.2, 0.5, 1)]
        all_pts, all_lines, all_cols = [], [], []
        off = 0
        for seg, rgb in zip(segs, colors):
            all_pts.extend(seg.tolist())
            all_lines.append([off, off + 1])
            all_cols.extend([rgb, rgb])
            off += 2
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(np.asarray(all_pts))
        ls.lines = o3d.utility.Vector2iVector(np.asarray(all_lines))
        ls.colors = o3d.utility.Vector3dVector(np.asarray(all_cols))
        self._drone_axes = ls

    def update(
        self,
        state: RigidBodyState,
        patrol_waypoints: Optional[np.ndarray] = None,
        active_target: Optional[np.ndarray] = None,
    ) -> None:
        self._path_points.append(state.position.copy())
        if len(self._path_points) > 500:
            self._path_points.pop(0)

        self._update_drone_axes(state)

        if self._use_filament:
            self.widget.scene.remove_geometry("drone")
            self.widget.scene.add_geometry("drone", self._drone_axes)
            if patrol_waypoints is not None and self._show_patrol and len(patrol_waypoints) > 1:
                segs = np.stack([patrol_waypoints[:-1], patrol_waypoints[1:]], axis=1)
                pls = _lineset_from_segments(segs, (0.2, 0.85, 1.0))
                self.widget.scene.remove_geometry("patrol")
                self.widget.scene.add_geometry("patrol", pls)
            return

        self.vis.update_geometry(self._drone_axes)

        if self._show_lidar:
            origin = state.position
            hits = self.scene.lidar_scan_2d(origin, height=origin[2], num_rays=120, max_range=14.0)
            valid = [h for h in hits if np.linalg.norm(h - origin) < 13.9]
            if valid:
                segs = np.array([[origin, h] for h in valid])
                self._lidar_lines = _lineset_from_segments(segs, (0.1, 0.95, 0.75))
            else:
                self._lidar_lines = o3d.geometry.LineSet()
            self.vis.update_geometry(self._lidar_lines)

        if patrol_waypoints is not None and self._show_patrol and len(patrol_waypoints) > 1:
            segs = np.stack([patrol_waypoints[:-1], patrol_waypoints[1:]], axis=1)
            self._patrol_lines = _lineset_from_segments(segs, (0.2, 0.85, 1.0))
            self.vis.update_geometry(self._patrol_lines)

        if len(self._path_points) >= 2:
            pts = np.asarray(self._path_points[-300:])
            segs = np.stack([pts[:-1], pts[1:]], axis=1)
            self._path_lines = _lineset_from_segments(segs, (0.95, 0.55, 0.15))
            self.vis.update_geometry(self._path_lines)

        if active_target is not None:
            marker = o3d.geometry.TriangleMesh.create_box(0.2, 0.2, 0.2)
            marker.translate(active_target - np.array([0.1, 0.1, 0.1]))
            n = len(marker.vertices)
            marker.vertex_colors = o3d.utility.Vector3dVector(np.tile([0.1, 1.0, 0.4], (n, 1)))
            if hasattr(self, "_target_marker"):
                self.vis.remove_geometry(self._target_marker, reset_bounding_box=False)
            self._target_marker = marker
            self.vis.add_geometry(self._target_marker, reset_bounding_box=False)

        ctr = self.vis.get_view_control()
        eye = state.position + np.array([-2.0, -1.2, 0.8])
        ctr.set_lookat(state.position)
        ctr.set_front((eye - state.position) / (np.linalg.norm(eye - state.position) + 1e-9))
        ctr.set_up([0, 0, 1])

    def poll(self) -> bool:
        if not self._running:
            return False
        if self._use_filament:
            self.app.run_one_tick()
            return self._running
        return self.vis.poll_events()

    def render(self) -> None:
        if not self._use_filament:
            self.vis.update_renderer()

    def close(self) -> None:
        if self._use_filament:
            self.window.close()
        else:
            self.vis.destroy_window()

    def register_key(self, key: int, callback) -> None:
        if not self._use_filament:
            self.vis.register_key_callback(key, callback)
