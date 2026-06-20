"""
Continuous waypoint patrol with cubic-spline trajectory tracking.

Fixes early-stop behaviour: patrol time monotonically increases, waypoints cycle
forever, and the spline is rebuilt smoothly on each capture event.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

from core.collision import MeshCollisionSolver
from scene_loader import RedwoodScene


@dataclass
class TrajectorySample:
    position: np.ndarray
    velocity: np.ndarray
    yaw: float


@dataclass
class WaypointPatrolManager:
    """
    Infinite cyclic patrol through indoor space with smooth CubicSpline segments.
    """

    scene: RedwoodScene
    cruise_altitude: float = 1.55
    capture_radius: float = 0.65
    cruise_speed: float = 0.85
    num_waypoints: int = 32
    lookahead_waypoints: int = 8

    _waypoints: List[np.ndarray] = field(default_factory=list, init=False)
    _active_index: int = field(default=0, init=False)
    _patrol_time: float = field(default=0.0, init=False)
    _spline: Optional[CubicSpline] = field(default=None, init=False)
    _spline_t0: float = field(default=0.0, init=False)
    _spline_duration: float = field(default=1.0, init=False)
    _segment_start: np.ndarray = field(default_factory=lambda: np.zeros(3), init=False)

    def __post_init__(self) -> None:
        self._collision = MeshCollisionSolver(self.scene)
        self._waypoints = self._generate_patrol_waypoints()
        if len(self._waypoints) < 4:
            raise RuntimeError("Insufficient patrol waypoints for spline navigation")
        self._segment_start = self._waypoints[0].copy()
        self._rebuild_spline(self._segment_start)

    def _generate_patrol_waypoints(self) -> List[np.ndarray]:
        b = self.scene.bounds
        mn, mx = b.min_corner, b.max_corner
        margin = 0.85
        z = float(np.clip(self.cruise_altitude, mn[2] + 0.8, mx[2] - 0.5))

        xs = np.linspace(mn[0] + margin, mx[0] - margin, 6)
        ys = np.linspace(mn[1] + margin, mx[1] - margin, 6)
        candidates: List[np.ndarray] = []
        for x in xs:
            for y in ys:
                p = np.array([x, y, z], dtype=np.float64)
                if self.scene.is_inside_bounds(p, margin=0.35):
                    p = self._collision.push_to_free_space(p)
                    if self._collision.is_position_free(p):
                        candidates.append(p)

        rng = np.random.default_rng(42)
        if len(candidates) < self.num_waypoints:
            for _ in range(self.num_waypoints * 4):
                p = np.array(
                    [
                        rng.uniform(mn[0] + margin, mx[0] - margin),
                        rng.uniform(mn[1] + margin, mx[1] - margin),
                        z,
                    ],
                    dtype=np.float64,
                )
                if self.scene.is_inside_bounds(p, margin=0.35):
                    p = self._collision.push_to_free_space(p)
                    if self._collision.is_position_free(p):
                        candidates.append(p)

        if not candidates:
            candidates = [
                np.array([b.center[0], b.center[1], z]),
                np.array([mn[0] + margin, mn[1] + margin, z]),
                np.array([mx[0] - margin, mn[1] + margin, z]),
                np.array([mx[0] - margin, mx[1] - margin, z]),
                np.array([mn[0] + margin, mx[1] - margin, z]),
            ]

        tree = cKDTree(np.asarray(candidates))
        ordered: List[np.ndarray] = [candidates[0]]
        remaining = set(range(1, len(candidates)))
        while remaining and len(ordered) < self.num_waypoints:
            dists, idxs = tree.query(ordered[-1], k=min(len(candidates), 12))
            picked = False
            for idx in np.atleast_1d(idxs).tolist():
                if idx in remaining:
                    ordered.append(candidates[idx])
                    remaining.remove(idx)
                    picked = True
                    break
            if not picked:
                idx = remaining.pop()
                ordered.append(candidates[idx])

        while len(ordered) < self.num_waypoints:
            ordered.append(ordered[len(ordered) % len(candidates)])

        return ordered[: self.num_waypoints]

    def _window_indices(self) -> List[int]:
        n = len(self._waypoints)
        count = min(self.lookahead_waypoints, n)
        return [(self._active_index + k) % n for k in range(count + 1)]

    def _rebuild_spline(self, start_position: np.ndarray) -> None:
        indices = self._window_indices()
        points = [start_position.copy()]
        for idx in indices[1:]:
            points.append(self._waypoints[idx].copy())
        points.append(self._waypoints[indices[-1]].copy())

        pts = np.asarray(points, dtype=np.float64)
        filtered = [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - filtered[-1]) <= 0.08:
                continue
            p_safe = self._collision.push_to_free_space(p)
            if not self._collision.segment_is_clear(filtered[-1], p_safe):
                mid = 0.5 * (filtered[-1] + p_safe)
                p_safe = self._collision.push_to_free_space(mid)
                if not self._collision.segment_is_clear(filtered[-1], p_safe):
                    continue
            filtered.append(p_safe)
        if len(filtered) < 2:
            filtered = [pts[0], pts[0] + np.array([0.5, 0.0, 0.0])]
        pts = np.asarray(filtered, dtype=np.float64)

        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cumulative = np.concatenate([[0.0], np.cumsum(seg)])
        for i in range(1, len(cumulative)):
            if cumulative[i] <= cumulative[i - 1]:
                cumulative[i] = cumulative[i - 1] + 0.05
        total_length = float(cumulative[-1])
        if total_length < 1e-3:
            total_length = 1.0
            cumulative = np.linspace(0.0, 1.0, len(pts))

        self._spline = CubicSpline(cumulative, pts, axis=0, bc_type="natural")
        self._spline_t0 = self._patrol_time
        self._spline_duration = max(total_length / max(self.cruise_speed, 0.15), 0.5)
        self._segment_start = start_position.copy()

    def _advance_waypoint_if_captured(self, position: np.ndarray) -> None:
        target = self._waypoints[self._active_index]
        if np.linalg.norm(position - target) > self.capture_radius:
            return
        self._active_index = (self._active_index + 1) % len(self._waypoints)
        self._rebuild_spline(position.copy())

    def update(self, position: np.ndarray, dt: float) -> None:
        """Advance patrol clock and cycle waypoints — never terminates."""
        self._patrol_time += max(dt, 1e-6)
        self._advance_waypoint_if_captured(position)

    def sample(self, position: np.ndarray) -> TrajectorySample:
        """Sample smooth position/velocity along the active cubic spline."""
        if self._spline is None:
            self._rebuild_spline(position)

        elapsed = self._patrol_time - self._spline_t0
        if elapsed >= self._spline_duration:
            self._active_index = (self._active_index + 1) % len(self._waypoints)
            self._rebuild_spline(position.copy())
            elapsed = 0.0

        u = float(np.clip(elapsed, 0.0, self._spline_duration))
        pos = np.asarray(self._spline(u), dtype=np.float64)
        vel = np.asarray(self._spline(u, 1), dtype=np.float64)

        b = self.scene.bounds
        z_lo = float(b.min_corner[2] + 0.9)
        z_hi = float(b.max_corner[2] - 0.4)
        pos[2] = float(np.clip(pos[2], z_lo, z_hi))
        vel[2] = float(np.clip(vel[2], -0.5, 0.5))

        speed = np.linalg.norm(vel[:2])
        if speed < 0.05:
            next_wp = self._waypoints[self._active_index]
            direction = next_wp[:2] - pos[:2]
            n = np.linalg.norm(direction)
            if n > 1e-3:
                vel[:2] = (direction / n) * self.cruise_speed * 0.3

        vel[2] = np.clip(vel[2], -0.35, 0.35)

        rep = self._collision.obstacle_repulsion_acceleration(position, vel)
        dt_est = 0.02
        vel = vel + rep * dt_est
        pos = pos + rep * (dt_est * dt_est) * 0.5
        pos = self._collision.push_to_free_space(pos)

        speed_xy = float(np.linalg.norm(vel[:2]))
        max_speed = self.cruise_speed
        min_clear, _, _ = self._collision.closest_obstacle(position, max_range=2.5)
        if min_clear < 1.0:
            max_speed *= max(0.2, min_clear / 1.0)

        if speed_xy > max_speed and speed_xy > 1e-6:
            vel[:2] *= max_speed / speed_xy

        yaw = float(np.arctan2(vel[1], vel[0] + 1e-9))
        return TrajectorySample(position=pos, velocity=vel, yaw=yaw)

    @property
    def patrol_time(self) -> float:
        return self._patrol_time

    @property
    def active_waypoint(self) -> np.ndarray:
        return self._waypoints[self._active_index].copy()
