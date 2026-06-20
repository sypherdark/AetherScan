"""
Simulated onboard sensors — raycasts against 3D mesh + semantic classification.

Each hit is analyzed against the precomputed ``AnalyzedIndoorSpace`` model
(walls, objects, free space) so labels reflect real structure semantics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Optional, Tuple

import numpy as np

from core.math3d import quat_to_euler
from core.semantic_space import SEMANTIC_NAMES, SemanticClass
from scene_loader import RedwoodScene

if TYPE_CHECKING:
    from core.semantic_space import AnalyzedIndoorSpace


class ObstacleLabel:
    WALL = "wall"
    FLOOR = "floor"
    CEILING = "ceiling"
    OBJECT = "object"
    FREE = "free"
    UNKNOWN = "unknown"


_LABEL_FROM_SEM = {
    SemanticClass.WALL: ObstacleLabel.WALL,
    SemanticClass.OBJECT: ObstacleLabel.OBJECT,
    SemanticClass.FLOOR: ObstacleLabel.FLOOR,
    SemanticClass.CEILING: ObstacleLabel.CEILING,
    SemanticClass.FREE: ObstacleLabel.FREE,
    SemanticClass.UNKNOWN: ObstacleLabel.UNKNOWN,
}


@dataclass(frozen=True)
class SensorConfig:
    lidar_rays_per_ring: int = 24
    lidar_rings: int = 7
    # Vertical elevation rings.  A near-horizontal 2D LiDAR samples only a thin
    # band at flight height; with just ±15° a wall is captured in a narrow Z
    # slice and the reconstruction reads as a flat smear.  Spreading the rings to
    # ±40° lets each scan of a wall capture a near floor-to-ceiling COLUMN at
    # once — which both fills the reconstruction vertically AND gives the
    # semantic classifier the column-height signal it needs to tell a tall wall
    # from a short piece of furniture (both are vertical planes by normal alone).
    lidar_pitch_deg: Tuple[float, ...] = (0.0, 12.0, -12.0, 25.0, -25.0, 40.0, -40.0)
    # Horizontal field of view, centred on body-forward (+X).  360 = spinning
    # 2D LiDAR (RPLIDAR-class, the default hardware assumption).  Set ~87 to
    # model a fixed depth camera (RealSense D435-class): the same ray budget is
    # then concentrated inside the wedge and the drone is BLIND outside it —
    # exploration and avoidance must cope (REALWORLD_READINESS.md item 4).
    lidar_fov_deg: float = 360.0
    lidar_max_range: float = 8.0
    lidar_min_range: float = 0.1
    proximity_rays: int = 20
    proximity_max_range: float = 1.8
    sensor_height_offset: float = 0.07

    # ── Realistic error model ───────────────────────────────────────────────
    # Calibrated to bracket a 360° 2D LiDAR (RPLIDAR-class, ±2 cm) blended with a
    # forward depth camera (RealSense-class, ~1% of range).  When `realistic` is
    # True the suite emits only data a real drone could actually produce: noisy
    # ranges, dropouts, noisy normals, and GEOMETRY-ONLY classification (no peeking
    # at the mesh's ground-truth labels or a precomputed structural map).  Set
    # False to recover the old idealised behaviour for debugging.
    realistic: bool = True
    # Range error standard deviation:  σ = base + pct · distance.
    range_noise_base_m: float = 0.02
    range_noise_pct: float = 0.01
    # Depth quantisation step ≈ range_quant_k · d² (depth cameras); 0 disables.
    range_quant_k: float = 0.001
    # Dropout probability (a return is lost): base, plus extra at long range and
    # at grazing incidence (real depth fails on oblique / specular surfaces).
    dropout_base: float = 0.004
    dropout_range_pct: float = 0.03
    dropout_grazing: float = 0.5
    grazing_cos_thresh: float = 0.26      # |ray·normal| below this ≈ >75° incidence
    # Surface-normal estimate error (real normals are differentiated from noisy
    # depth, so they are far less clean than mesh face normals).
    normal_noise_deg: float = 5.0
    # Classify purely from geometry (surface normal + height) the way an onboard
    # perception stack must — never from the simulator's ground-truth semantics.
    geometry_only_classification: bool = True
    # RNG seed for reproducible noise; None = nondeterministic.
    rng_seed: Optional[int] = 0
    # Exponential temporal filter on the 1-D altitude rangefinders (downward/upward
    # ToF).  Real flight stacks never feed a raw range straight into altitude
    # control — the sensor driver / EKF low-passes it.  alpha∈(0,1]: 1 = no filter,
    # smaller = smoother.  Only smooths the altitude ToF, not the planar LiDAR.
    range_filter_alpha: float = 0.35

    @property
    def lidar_rays(self) -> int:
        return self.lidar_rings * self.lidar_rays_per_ring


@dataclass
class LidarReturn:
    bearing_world: float
    bearing_body: float
    range_m: float
    hit_point: np.ndarray
    surface_normal: np.ndarray
    label: str
    semantic_class: int = 0
    element_id: int = -1
    confidence: float = 0.0


@dataclass
class SensorFrame:
    position: np.ndarray
    yaw: float
    returns: List[LidarReturn] = field(default_factory=list)
    proximity_min: float = 999.0
    proximity_body_angle: float = 0.0
    floor_range_m: float = 999.0
    ceiling_range_m: float = 999.0
    detected_structures: List[dict] = field(default_factory=list)

    @property
    def min_range(self) -> float:
        obs = [
            r.range_m
            for r in self.returns
            if r.label not in (ObstacleLabel.FREE, ObstacleLabel.UNKNOWN)
        ]
        return float(min(obs)) if obs else 8.0

    @property
    def front_range(self) -> float:
        vals = [
            r.range_m
            for r in self.returns
            if r.label in (ObstacleLabel.WALL, ObstacleLabel.OBJECT)
            and abs(r.bearing_body) < np.deg2rad(35)
        ]
        return float(min(vals)) if vals else 8.0

    def walls_and_objects(self) -> List[LidarReturn]:
        return [
            r
            for r in self.returns
            if r.label in (ObstacleLabel.WALL, ObstacleLabel.OBJECT)
        ]

    def hit_points_obstacles(self) -> np.ndarray:
        """Wall + object hits only (used for VFH repulsion checks)."""
        pts = [r.hit_point for r in self.walls_and_objects() if r.range_m < 7.9]
        return np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 3))

    def hit_points_all_surfaces(self) -> np.ndarray:
        """
        All surface hits — walls, objects, floor AND ceiling.

        This is the correct source for 3D reconstruction map_points.
        The original hit_points_obstacles() omitted floor and ceiling
        entirely, causing those surfaces to be invisible in the scan output.
        """
        pts = [
            r.hit_point
            for r in self.returns
            if r.label not in (ObstacleLabel.FREE, ObstacleLabel.UNKNOWN)
            and r.range_m < 7.9
        ]
        return np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 3))

    def hit_points_labeled(self) -> np.ndarray:
        """
        All surface hits with semantic class — returns (N, 4) array where
        columns are [x, y, z, semantic_class].  Used for coloured point-cloud
        reconstruction in the dashboard.
        """
        rows = [
            [*r.hit_point, float(r.semantic_class)]
            for r in self.returns
            if r.label not in (ObstacleLabel.FREE, ObstacleLabel.UNKNOWN)
            and r.range_m < 7.9
        ]
        return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 4))


class MeshSensorSuite:
    def __init__(
        self,
        scene: RedwoodScene,
        analyzed: Optional[AnalyzedIndoorSpace] = None,
        config: SensorConfig | None = None,
    ):
        self.scene = scene
        self.analyzed = analyzed
        self.cfg = config or SensorConfig()
        self._rng = np.random.default_rng(self.cfg.rng_seed)
        self._floor_ema: Optional[float] = None  # filtered downward-ToF AGL
        self._ceil_ema: Optional[float] = None    # filtered upward-ToF clearance
        self._body_angles, self._pitch_angles, self._body_directions = (
            self._build_multiring_lidar_layout(self.cfg)
        )

    def _filter_floor(self, d: float) -> float:
        a = self.cfg.range_filter_alpha if self.cfg.realistic else 1.0
        self._floor_ema = d if self._floor_ema is None else a * d + (1 - a) * self._floor_ema
        return float(self._floor_ema)

    def _filter_ceil(self, d: float) -> float:
        a = self.cfg.range_filter_alpha if self.cfg.realistic else 1.0
        self._ceil_ema = d if self._ceil_ema is None else a * d + (1 - a) * self._ceil_ema
        return float(self._ceil_ema)

    def _apply_range_error(
        self,
        dists: np.ndarray,
        points: np.ndarray,
        normals: np.ndarray,
        directions: np.ndarray,
        origin: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Corrupt a batch of ideal raycast returns into something a real range
        sensor would output: additive range noise (σ grows with distance and
        incidence angle), depth quantisation, stochastic dropout, and noisy
        surface normals.  Dropped/lost returns are pushed to max-range so the
        rest of the pipeline treats them as free space — exactly what happens
        when a real beam gets no echo.
        """
        cfg = self.cfg
        max_r = cfg.lidar_max_range
        d = dists.astype(np.float64).copy()
        hit = d < max_r - 0.02
        if not np.any(hit):
            return d, points, normals

        n = normals.astype(np.float64)
        nlen = np.linalg.norm(n, axis=1, keepdims=True)
        n_unit = n / np.maximum(nlen, 1e-9)
        # |ray · normal| → 1 at normal incidence, → 0 at grazing.
        cos_inc = np.abs(np.sum(directions * n_unit, axis=1))
        grazing = hit & (cos_inc < cfg.grazing_cos_thresh)

        # ── Additive range noise (σ scales with distance; worse when grazing) ──
        sigma = cfg.range_noise_base_m + cfg.range_noise_pct * d
        sigma = np.where(grazing, sigma * 3.0, sigma)
        d_noisy = d + self._rng.normal(0.0, 1.0, d.shape) * sigma

        # ── Depth quantisation (≈ k·d²) ───────────────────────────────────────
        if cfg.range_quant_k > 0.0:
            step = np.maximum(cfg.range_quant_k * d * d, 1e-4)
            d_noisy = np.round(d_noisy / step) * step

        d_noisy = np.clip(d_noisy, cfg.lidar_min_range, max_r)

        # ── Dropout (lost echoes) ─────────────────────────────────────────────
        p_drop = (
            cfg.dropout_base
            + cfg.dropout_range_pct * (d / max_r)
            + cfg.dropout_grazing * grazing.astype(np.float64)
        )
        dropped = hit & (self._rng.random(d.shape) < p_drop)

        d_out = np.where(hit, d_noisy, d)
        d_out = np.where(dropped, max_r, d_out)

        # ── Noisy normals (real normals come from differentiating noisy depth) ─
        if cfg.normal_noise_deg > 0.0:
            sig_n = np.deg2rad(cfg.normal_noise_deg)
            perturb = self._rng.normal(0.0, sig_n, n_unit.shape)
            n_noisy = n_unit + perturb
            n_noisy /= np.maximum(np.linalg.norm(n_noisy, axis=1, keepdims=True), 1e-9)
            normals_out = np.where(hit[:, None], n_noisy, normals)
        else:
            normals_out = normals

        # Recompute hit points from the noisy range so the reconstruction point
        # cloud carries the same error the planner sees (no free ground truth).
        points_out = origin[None, :] + directions * d_out[:, None]
        return d_out, points_out, normals_out

    @staticmethod
    def _build_multiring_lidar_layout(
        cfg: SensorConfig,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Three elevation rings × 24 azimuth rays (72 total).
        Ring 1: 0° pitch (horizontal). Ring 2: +15°. Ring 3: -15°.
        """
        fov = float(np.deg2rad(min(max(cfg.lidar_fov_deg, 1.0), 360.0)))
        if fov >= 2.0 * np.pi - 1e-9:
            azimuth = np.linspace(-np.pi, np.pi, cfg.lidar_rays_per_ring, endpoint=False)
        else:
            # Restricted FOV (depth-camera-class): same ray budget inside the wedge
            azimuth = np.linspace(-fov / 2.0, fov / 2.0, cfg.lidar_rays_per_ring)
        body_angles: List[float] = []
        pitch_angles: List[float] = []
        body_dirs: List[np.ndarray] = []

        for pitch_deg in cfg.lidar_pitch_deg:
            pitch = float(np.deg2rad(pitch_deg))
            cp = float(np.cos(pitch))
            sp = float(np.sin(pitch))
            for az in azimuth:
                d = np.array([cp * np.cos(az), cp * np.sin(az), sp], dtype=np.float64)
                nrm = float(np.linalg.norm(d))
                if nrm > 1e-9:
                    d /= nrm
                body_dirs.append(d)
                body_angles.append(float(az))
                pitch_angles.append(pitch)

        return (
            np.asarray(body_angles, dtype=np.float64),
            np.asarray(pitch_angles, dtype=np.float64),
            np.asarray(body_dirs, dtype=np.float64),
        )

    def _noisy_scalar_range(self, d: float) -> float:
        """Apply the configured range-noise model to a single 1-D rangefinder
        reading (downward/upward ToF).  No dropout — these point at large flat
        surfaces (floor/ceiling) that almost always return."""
        if not self.cfg.realistic:
            return d
        sigma = self.cfg.range_noise_base_m + self.cfg.range_noise_pct * d
        return float(np.clip(d + self._rng.normal(0.0, sigma),
                             self.cfg.lidar_min_range, self.cfg.lidar_max_range))

    def _world_directions(self, yaw: float) -> np.ndarray:
        """Rotate body-frame unit rays into world Z-up frame."""
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        dirs = (rot @ self._body_directions.T).T
        norms = np.linalg.norm(dirs, axis=1, keepdims=True)
        return dirs / np.maximum(norms, 1e-9)

    def scan(self, position: np.ndarray, quaternion: np.ndarray) -> SensorFrame:
        pos = np.asarray(position, dtype=np.float64)
        quat = np.asarray(quaternion, dtype=np.float64)
        _, _, yaw = quat_to_euler(quat)

        origin = pos.copy()
        origin[2] += self.cfg.sensor_height_offset

        n = self.cfg.lidar_rays
        body_angles = self._body_angles
        world_angles = body_angles + yaw
        directions = self._world_directions(yaw)
        origins = np.repeat(origin[None, :], n, axis=0)

        dists, points, normals, primitive_ids = self.scene.cast_rays(
            origins, directions, max_distance=self.cfg.lidar_max_range
        )
        if self.cfg.realistic:
            dists, points, normals = self._apply_range_error(
                dists, points, normals, directions, origin
            )

        returns: List[LidarReturn] = []
        prox_min = self.cfg.proximity_max_range
        prox_angle = 0.0
        seen_elements: dict[int, dict] = {}

        for i in range(n):
            d = float(dists[i])
            hit = points[i]
            nrm = normals[i]
            if not np.isfinite(d) or d >= self.cfg.lidar_max_range - 0.02:
                sem = SemanticClass.FREE
                label = ObstacleLabel.FREE
                d = self.cfg.lidar_max_range
                hit = origin + directions[i] * d
                nrm = np.array([0.0, 0.0, 1.0])
                eid, conf = -1, 1.0
            else:
                sem, eid, conf = self._classify(
                    hit, nrm, origin, int(primitive_ids[i])
                )
                label = _LABEL_FROM_SEM.get(sem, ObstacleLabel.UNKNOWN)

            if label in (ObstacleLabel.WALL, ObstacleLabel.OBJECT) and eid >= 0:
                if eid not in seen_elements:
                    seen_elements[eid] = {
                        "id": eid,
                        "kind": label,
                        "range_m": round(d, 3),
                        "confidence": round(conf, 2),
                    }
                else:
                    seen_elements[eid]["range_m"] = min(seen_elements[eid]["range_m"], round(d, 3))

            returns.append(
                LidarReturn(
                    bearing_world=float(world_angles[i]),
                    bearing_body=float(body_angles[i]),
                    range_m=max(d, self.cfg.lidar_min_range),
                    hit_point=hit.copy(),
                    surface_normal=nrm.copy(),
                    label=label,
                    semantic_class=int(sem),
                    element_id=eid,
                    confidence=conf,
                )
            )

            if label in (ObstacleLabel.WALL, ObstacleLabel.OBJECT) and d < prox_min:
                prox_min = d
                prox_angle = float(body_angles[i])

        prox_dirs = self._proximity_directions()
        p_origins = np.repeat(origin[None, :], len(prox_dirs), axis=0) + prox_dirs * 0.03
        pd, ph, pn, _ = self.scene.cast_rays(
            p_origins, prox_dirs, max_distance=self.cfg.proximity_max_range
        )
        for j, d in enumerate(pd):
            if np.isfinite(d) and d < prox_min:
                prox_min = float(d)
                prox_angle = float(np.arctan2(prox_dirs[j, 1], prox_dirs[j, 0]))

        floor_range = self.cfg.lidar_max_range
        ceil_range = self.cfg.lidar_max_range
        fd, fp, fn, fprim = self.scene.cast_rays(
            origin[None, :], np.array([[0, 0, -1]]), 6.0
        )
        cd, cp, cn, cprim = self.scene.cast_rays(origin[None, :], np.array([[0, 0, 1]]), 6.0)
        if np.isfinite(fd[0]) and fd[0] < 5.9:
            floor_range = self._filter_floor(self._noisy_scalar_range(float(fd[0])))
            sem, eid, conf = self._classify(fp[0], fn[0], origin, int(fprim[0]))
            returns.append(
                LidarReturn(
                    bearing_world=float(yaw),
                    bearing_body=0.0,
                    range_m=floor_range,
                    hit_point=fp[0],
                    surface_normal=fn[0],
                    label=ObstacleLabel.FLOOR,
                    semantic_class=int(sem),
                    element_id=eid,
                    confidence=conf,
                )
            )
        if np.isfinite(cd[0]) and cd[0] < 5.9:
            ceil_range = self._filter_ceil(self._noisy_scalar_range(float(cd[0])))
            sem_c, eid_c, conf_c = self._classify(cp[0], cn[0], origin, int(cprim[0]))
            returns.append(
                LidarReturn(
                    bearing_world=float(yaw + np.pi),
                    bearing_body=float(np.pi),
                    range_m=ceil_range,
                    hit_point=cp[0],
                    surface_normal=cn[0],
                    label=ObstacleLabel.CEILING,
                    semantic_class=int(sem_c),
                    element_id=eid_c,
                    confidence=conf_c,
                )
            )

        structures = list(seen_elements.values())
        # In realistic mode the drone may only report structures it actually saw
        # via raycasts (seen_elements).  The precomputed-map injection below is a
        # ground-truth cheat — a real drone has no a-priori structural map — so it
        # is disabled whenever the realistic perception model is active.
        if self.analyzed is not None and not self.cfg.realistic:
            for el in self.analyzed.nearby_elements(pos, radius=2.5):
                if el.id not in seen_elements:
                    dist = float(np.linalg.norm(el.centroid - pos))
                    if dist < 3.5:
                        structures.append(
                            {
                                "id": el.id,
                                "kind": el.kind,
                                "range_m": round(dist, 3),
                                "confidence": round(el.confidence, 2),
                                "source": "map_nearby",
                            }
                        )

        return SensorFrame(
            position=pos,
            yaw=float(yaw),
            returns=returns,
            proximity_min=float(prox_min),
            proximity_body_angle=prox_angle,
            floor_range_m=floor_range,
            ceiling_range_m=ceil_range,
            detected_structures=structures[:16],
        )

    def _classify(
        self,
        hit: np.ndarray,
        normal: np.ndarray,
        origin: np.ndarray,
        primitive_id: int = -1,
    ) -> tuple[SemanticClass, int, float]:
        # Realistic mode: classify from geometry alone (surface normal + height),
        # exactly as an onboard perception stack must.  Never consult the mesh's
        # ground-truth triangle semantics or the precomputed structural analysis —
        # those are simulator-only oracles a real drone does not have.
        if self.cfg.geometry_only_classification:
            n = normal / (np.linalg.norm(normal) + 1e-9)
            nz = abs(float(n[2]))
            if nz >= 0.72:
                if hit[2] < origin[2] - 0.05:
                    return SemanticClass.FLOOR, -1, 0.85
                return SemanticClass.CEILING, -1, 0.85
            if nz <= 0.35:
                return SemanticClass.WALL, -1, 0.75
            return SemanticClass.OBJECT, -1, 0.7

        # Get raw classification from mesh semantics or analysis
        if primitive_id >= 0 and getattr(self.scene, "_triangle_semantics", None) is not None:
            sem, eid, conf = self.scene.classify_primitive(primitive_id, hit, normal, origin)
        elif self.analyzed is not None:
            sem, eid, conf = self.analyzed.classify_hit(hit, normal, origin, primitive_id)
        else:
            # Pure normal-based fallback (no mesh semantics available)
            n = normal / (np.linalg.norm(normal) + 1e-9)
            nz = abs(float(n[2]))
            if nz >= 0.72:
                if hit[2] < origin[2] - 0.05:
                    return SemanticClass.FLOOR, -1, 0.85
                return SemanticClass.CEILING, -1, 0.85
            if nz <= 0.35:
                return SemanticClass.WALL, -1, 0.75
            return SemanticClass.OBJECT, -1, 0.7

        # Post-classification override: the Replica collision mesh only has labels
        # 0-3 (UNKNOWN/FREE/WALL/OBJECT) — FLOOR and CEILING do not exist in the
        # triangle semantics file.  Horizontal surfaces therefore come back as WALL
        # or UNKNOWN.  Correct them here using the surface normal + absolute Z:
        #   Z < 0.40 m → floor level    (actual floor is at Z ≈ 0)
        #   Z > 2.30 m → ceiling level  (actual ceiling is at Z ≈ 2.85)
        # Mid-height horizontal faces (table tops, shelves) are left as-is.
        n = normal / (np.linalg.norm(normal) + 1e-9)
        if abs(float(n[2])) >= 0.72:
            hit_z = float(hit[2])
            if hit_z < 0.40:
                return SemanticClass.FLOOR, eid, 0.85
            if hit_z > 2.30:
                return SemanticClass.CEILING, eid, 0.85

        return sem, eid, conf

    @staticmethod
    def _proximity_directions() -> np.ndarray:
        phi = np.linspace(0, np.pi, 4)
        theta = np.linspace(0, 2 * np.pi, 5, endpoint=False)
        dirs = []
        for p in phi:
            for t in theta:
                dirs.append(
                    [np.sin(p) * np.cos(t), np.sin(p) * np.sin(t), np.cos(p)]
                )
        return np.asarray(dirs, dtype=np.float64)

    def to_telemetry_dict(self, frame: SensorFrame, discovery: Optional[dict] = None) -> dict:
        step = max(1, len(frame.returns) // 36)
        sectors = [
            {
                "bearing_deg": round(float(np.rad2deg(r.bearing_body)), 1),
                "range_m": round(r.range_m, 3),
                "label": r.label,
                "element_id": r.element_id,
                "confidence": round(r.confidence, 2),
            }
            for r in frame.returns[::step]
        ]
        return {
            "min_range_m": round(frame.min_range, 3),
            "front_range_m": round(frame.front_range, 3),
            "proximity_m": round(frame.proximity_min, 3),
            "open_direction_deg": round(float(self._open_direction_deg(frame)), 1),
            "wall_hits": sum(1 for r in frame.returns if r.label == ObstacleLabel.WALL),
            "object_hits": sum(1 for r in frame.returns if r.label == ObstacleLabel.OBJECT),
            "floor_range_m": round(frame.floor_range_m, 3),
            "ceiling_range_m": round(frame.ceiling_range_m, 3),
            "structures": frame.detected_structures,
            "sectors": sectors,
            "discovery": discovery or {},
        }

    @staticmethod
    def _open_direction_deg(frame: SensorFrame) -> float:
        best_range = -1.0
        best_angle = 0.0
        for r in frame.returns:
            if r.label in (ObstacleLabel.FLOOR, ObstacleLabel.CEILING, ObstacleLabel.FREE):
                continue
            if r.range_m > best_range:
                best_range = r.range_m
                best_angle = r.bearing_body
        return float(np.rad2deg(best_angle))
