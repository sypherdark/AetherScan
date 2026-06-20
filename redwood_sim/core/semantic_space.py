"""
Offline semantic analysis of the indoor 3D mesh — walls, objects, free space.

Produces a real spatial model (grid + labeled elements) used to classify sensor
hits consistently. The drone does not get this map for free; it must discover
cells via ``DiscoveryMap``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

import numpy as np

from scene_loader import RedwoodScene


class SemanticClass(IntEnum):
    UNKNOWN = 0
    FREE = 1
    WALL = 2
    OBJECT = 3
    FLOOR = 4
    CEILING = 5


SEMANTIC_NAMES = {
    SemanticClass.UNKNOWN: "unknown",
    SemanticClass.FREE: "free",
    SemanticClass.WALL: "wall",
    SemanticClass.OBJECT: "object",
    SemanticClass.FLOOR: "floor",
    SemanticClass.CEILING: "ceiling",
}


@dataclass
class SpatialElement:
    """One analyzed structure in the scene (wall plane or object cluster)."""

    id: int
    kind: str
    centroid: np.ndarray
    extent: np.ndarray
    normal: np.ndarray
    point_count: int
    confidence: float
    bounds_min: np.ndarray
    bounds_max: np.ndarray


@dataclass
class AnalyzedSpaceConfig:
    grid_resolution: float = 0.18
    flight_height: float = 1.45
    clearance_free: float = 0.95
    wall_normal_z_max: float = 0.32
    object_cluster_eps: float = 0.42
    object_min_points: int = 12


@dataclass
class AnalyzedIndoorSpace:
    """
    Complete semantic model of the mesh at navigation height.
    """

    scene: RedwoodScene
    config: AnalyzedSpaceConfig
    origin_xy: np.ndarray
    grid_shape: Tuple[int, int]
    nav_grid: np.ndarray
    elements: List[SpatialElement] = field(default_factory=list)
    floor_z: float = 0.05
    _points_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))

    @classmethod
    def build(
        cls,
        scene: RedwoodScene,
        flight_height: float = 1.45,
        config: AnalyzedSpaceConfig | None = None,
    ) -> AnalyzedIndoorSpace:
        cfg = config or AnalyzedSpaceConfig(flight_height=flight_height)
        cfg.flight_height = flight_height
        b = scene.bounds
        margin = 0.4
        origin = np.array(
            [b.min_corner[0] - margin, b.min_corner[1] - margin],
            dtype=np.float64,
        )
        size_x = b.max_corner[0] - b.min_corner[0] + 2 * margin
        size_y = b.max_corner[1] - b.min_corner[1] + 2 * margin
        nx = int(size_x / cfg.grid_resolution) + 1
        ny = int(size_y / cfg.grid_resolution) + 1
        grid = np.full((nx, ny), SemanticClass.UNKNOWN, dtype=np.uint8)

        pts = np.asarray(scene.point_cloud.points, dtype=np.float64)
        nrms = np.asarray(scene.point_cloud.normals, dtype=np.float64)
        if len(nrms) != len(pts):
            scene.point_cloud.estimate_normals()
            nrms = np.asarray(scene.point_cloud.normals, dtype=np.float64)

        z_lo = flight_height - 0.55
        z_hi = flight_height + 0.55
        band = (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
        pts_band = pts[band]
        nrms_band = nrms[band]

        inst = cls(
            scene=scene,
            config=cfg,
            origin_xy=origin,
            grid_shape=(nx, ny),
            nav_grid=grid,
            floor_z=float(b.min_corner[2]),
            _points_xy=pts_band[:, :2].copy(),
        )
        if scene.has_triangle_semantics:
            inst._fill_nav_grid_from_semantics()
        else:
            inst._fill_nav_grid(pts_band, nrms_band)
        inst._extract_elements(pts_band, nrms_band)
        return inst

    def _xy_to_ij(self, x: float, y: float) -> Tuple[int, int]:
        res = self.config.grid_resolution
        i = int((x - self.origin_xy[0]) / res)
        j = int((y - self.origin_xy[1]) / res)
        return i, j

    def _ij_to_xy(self, i: int, j: int) -> np.ndarray:
        res = self.config.grid_resolution
        return np.array(
            [
                self.origin_xy[0] + (i + 0.5) * res,
                self.origin_xy[1] + (j + 0.5) * res,
            ],
            dtype=np.float64,
        )

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.grid_shape[0] and 0 <= j < self.grid_shape[1]

    def _fill_nav_grid_from_semantics(self) -> None:
        """Build 2D nav grid from per-triangle Replica / semantic raycast labels."""
        res = self.config.grid_resolution
        h = self.config.flight_height
        nx, ny = self.grid_shape
        step = max(1, int(0.25 / res))

        for i in range(0, nx, step):
            for j in range(0, ny, step):
                center = self._ij_to_xy(i, j)
                center3 = np.array([center[0], center[1], h], dtype=np.float64)
                min_d = 999.0
                best_sem = SemanticClass.UNKNOWN

                for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
                    d = np.array([np.cos(ang), np.sin(ang), 0.0], dtype=np.float64)
                    dists, _, _, prim_ids = self.scene.cast_rays(
                        center3[None, :], d[None, :], max_distance=4.0
                    )
                    dist = float(dists[0])
                    if dist >= min_d:
                        continue
                    min_d = dist
                    pid = int(prim_ids[0])
                    if pid >= 0 and self.scene.has_triangle_semantics:
                        best_sem = SemanticClass(
                            int(self.scene._triangle_semantics[pid])
                        )
                    elif dist < self.config.clearance_free:
                        best_sem = SemanticClass.WALL

                if min_d >= self.config.clearance_free:
                    self.nav_grid[i, j] = SemanticClass.FREE
                elif best_sem in (
                    SemanticClass.WALL,
                    SemanticClass.OBJECT,
                    SemanticClass.FLOOR,
                    SemanticClass.CEILING,
                ):
                    self.nav_grid[i, j] = best_sem
                else:
                    self.nav_grid[i, j] = SemanticClass.WALL

    def _fill_nav_grid(self, pts: np.ndarray, nrms: np.ndarray) -> None:
        res = self.config.grid_resolution
        h = self.config.flight_height
        nx, ny = self.grid_shape

        step = max(1, int(0.25 / res))
        for i in range(0, nx, step):
            for j in range(0, ny, step):
                center = self._ij_to_xy(i, j)
                center3 = np.array([center[0], center[1], h])
                min_d = 999.0
                best_n = np.array([0.0, 0.0, 1.0])
                for ang in np.linspace(0, 2 * np.pi, 12, endpoint=False):
                    d = np.array([np.cos(ang), np.sin(ang), 0.0])
                    dists, _, norms, _ = self.scene.cast_rays(
                        center3[None, :], d[None, :], max_distance=4.0
                    )
                    if dists[0] < min_d:
                        min_d = float(dists[0])
                        best_n = norms[0]

                if min_d >= self.config.clearance_free:
                    self.nav_grid[i, j] = SemanticClass.FREE
                    continue

                nz = abs(float(best_n[2]))
                if nz <= self.config.wall_normal_z_max:
                    self.nav_grid[i, j] = SemanticClass.WALL
                else:
                    self.nav_grid[i, j] = SemanticClass.OBJECT

        # Refine with local point density
        for k in range(len(pts)):
            i, j = self._xy_to_ij(pts[k, 0], pts[k, 1])
            if not self.in_bounds(i, j):
                continue
            nz = abs(float(nrms[k, 2]))
            if nz <= self.config.wall_normal_z_max:
                self.nav_grid[i, j] = SemanticClass.WALL
            elif self.nav_grid[i, j] != SemanticClass.WALL:
                self.nav_grid[i, j] = SemanticClass.OBJECT

    def _extract_elements(self, pts: np.ndarray, nrms: np.ndarray) -> None:
        try:
            from sklearn.cluster import DBSCAN
        except ImportError:
            self._extract_elements_simple(pts, nrms)
            return

        wall_pts = pts[np.abs(nrms[:, 2]) <= self.config.wall_normal_z_max]
        obj_pts = pts[
            (np.abs(nrms[:, 2]) > self.config.wall_normal_z_max)
            & (np.abs(nrms[:, 2]) < 0.75)
        ]

        eid = 0
        if len(wall_pts) > 30:
            db = DBSCAN(eps=0.55, min_samples=20).fit(wall_pts)
            for label in set(db.labels_):
                if label < 0:
                    continue
                mask = db.labels_ == label
                cluster = wall_pts[mask]
                eid += 1
                self.elements.append(self._element_from_cluster(eid, "wall", cluster, nrms))

        if len(obj_pts) > 20:
            db = DBSCAN(eps=self.config.object_cluster_eps, min_samples=self.config.object_min_points).fit(
                obj_pts
            )
            for label in set(db.labels_):
                if label < 0:
                    continue
                mask = db.labels_ == label
                cluster = obj_pts[mask]
                eid += 1
                self.elements.append(self._element_from_cluster(eid, "object", cluster, nrms))

    def _extract_elements_simple(self, pts: np.ndarray, nrms: np.ndarray) -> None:
        wall = pts[np.abs(nrms[:, 2]) <= self.config.wall_normal_z_max]
        obj = pts[np.abs(nrms[:, 2]) > self.config.wall_normal_z_max]
        if len(wall) > 50:
            self.elements.append(self._element_from_cluster(1, "wall", wall, nrms))
        if len(obj) > 30:
            self.elements.append(self._element_from_cluster(2, "object", obj, nrms))

    def _element_from_cluster(
        self, eid: int, kind: str, cluster: np.ndarray, nrms: np.ndarray
    ) -> SpatialElement:
        c = cluster.mean(axis=0)
        mn = cluster.min(axis=0)
        mx = cluster.max(axis=0)
        n = nrms[: len(cluster)].mean(axis=0) if len(nrms) >= len(cluster) else np.array([0, 0, 1])
        n = n / (np.linalg.norm(n) + 1e-9)
        return SpatialElement(
            id=eid,
            kind=kind,
            centroid=c,
            extent=mx - mn,
            normal=n,
            point_count=len(cluster),
            confidence=min(1.0, len(cluster) / 200.0),
            bounds_min=mn,
            bounds_max=mx,
        )

    def classify_hit(
        self,
        hit_point: np.ndarray,
        surface_normal: np.ndarray,
        origin: np.ndarray,
        primitive_id: int = -1,
    ) -> Tuple[SemanticClass, int, float]:
        """Classify a sensor hit using mesh analysis + geometry."""
        if (
            primitive_id >= 0
            and self.scene.has_triangle_semantics
            and primitive_id < len(self.scene._triangle_semantics)
        ):
            sem, eid, conf = self.scene.classify_primitive(
                primitive_id, hit_point, surface_normal, origin
            )
            if sem == SemanticClass.WALL:
                return sem, eid, 1.0
            return sem, eid, conf

        n = surface_normal / (np.linalg.norm(surface_normal) + 1e-9)
        nz = abs(float(n[2]))

        if nz >= 0.72:
            if hit_point[2] < origin[2] - 0.05:
                return SemanticClass.FLOOR, -1, 0.95
            return SemanticClass.CEILING, -1, 0.9

        i, j = self._xy_to_ij(hit_point[0], hit_point[1])
        grid_class = SemanticClass.UNKNOWN
        element_id = -1
        conf = 0.7

        if self.in_bounds(i, j):
            grid_class = SemanticClass(int(self.nav_grid[i, j]))

        if nz <= self.config.wall_normal_z_max:
            geom = SemanticClass.WALL
        else:
            geom = SemanticClass.OBJECT

        final = grid_class if grid_class != SemanticClass.UNKNOWN else geom
        if grid_class != SemanticClass.UNKNOWN and grid_class != geom:
            final = geom
            conf = 0.75

        best_d = 999.0
        for el in self.elements:
            if SEMANTIC_NAMES.get(final, "") != el.kind and final != SemanticClass.UNKNOWN:
                continue
            d = float(np.linalg.norm(hit_point - el.centroid))
            if d < best_d and d < 1.2:
                best_d = d
                element_id = el.id
                conf = max(conf, el.confidence)

        return final, element_id, conf

    def query_navigable(self, x: float, y: float) -> SemanticClass:
        i, j = self._xy_to_ij(x, y)
        if not self.in_bounds(i, j):
            return SemanticClass.UNKNOWN
        return SemanticClass(int(self.nav_grid[i, j]))

    def nearby_elements(self, position: np.ndarray, radius: float = 3.0) -> List[SpatialElement]:
        out = []
        for el in self.elements:
            if float(np.linalg.norm(el.centroid[:2] - position[:2])) <= radius:
                out.append(el)
        return out

    def summary(self) -> Dict[str, object]:
        free = int(np.sum(self.nav_grid == SemanticClass.FREE))
        wall = int(np.sum(self.nav_grid == SemanticClass.WALL))
        obj = int(np.sum(self.nav_grid == SemanticClass.OBJECT))
        total = self.nav_grid.size
        walls_n = sum(1 for e in self.elements if e.kind == "wall")
        objs_n = sum(1 for e in self.elements if e.kind == "object")
        return {
            "grid_cells": total,
            "free_cells": free,
            "wall_cells": wall,
            "object_cells": obj,
            "free_percent": round(100.0 * free / max(total, 1), 1),
            "wall_elements": walls_n,
            "object_elements": objs_n,
            "total_elements": len(self.elements),
            "flight_height_m": self.config.flight_height,
            "elements": [
                {
                    "id": e.id,
                    "kind": e.kind,
                    "centroid": e.centroid.tolist(),
                    "extent": e.extent.tolist(),
                    "points": e.point_count,
                    "confidence": round(e.confidence, 2),
                }
                for e in self.elements[:24]
            ],
        }
