"""
Mesh collision — multi-sphere drone proxy + triangle-normal resolution.

Uses Open3D RaycastingScene primitive normals from scene_loader.cast_rays.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from core.math3d import quat_to_rotation_matrix
from scene_loader import RedwoodScene


@dataclass
class CollisionParams:
    # Compact small indoor-scanning quadcopter modelled as a single 0.18 m bounding
    # sphere (arm_length=0).  The old multi-sphere model (0.32 m at each ±0.18 m
    # arm) gave a 1.0 m span perpendicular to travel, so the drone could not pass
    # an 0.8 m doorway.  A single 0.18 m disc lets the 0.2 m occupancy grid keep a
    # 1-cell (0.2 m) inflation AND leave an 0.8 m doorway (4 cells) open through the
    # middle — the catch-22 that confined the drone to one room.
    body_radius: float = 0.18
    skin: float = 0.04
    restitution: float = 0.12
    friction: float = 0.55
    max_correction_per_iter: float = 0.2
    solver_iterations: int = 10
    sweep_enabled: bool = True
    clearance_goal: float = 0.72
    arm_length: float = 0.18


@dataclass
class AvoidanceParams:
    horizon: float = 2.2
    max_repulsion_accel: float = 2.8
    num_horizontal_rays: int = 18
    num_vertical_rays: int = 6


class MeshCollisionSolver:
    """
    Three-sphere proxy: body center + left rotor + right rotor (body Y axis).
    """

    def __init__(
        self,
        scene: RedwoodScene,
        collision: CollisionParams | None = None,
        avoidance: AvoidanceParams | None = None,
    ):
        self.scene = scene
        self.cp = collision or CollisionParams()
        self.ap = avoidance or AvoidanceParams()
        self._probe_dirs = self._fibonacci_sphere(28)

    @staticmethod
    def _fibonacci_sphere(n: int) -> np.ndarray:
        i = np.arange(n, dtype=np.float64)
        phi = np.arccos(1.0 - 2.0 * (i + 0.5) / n)
        theta = np.pi * (1.0 + 5.0**0.5) * i
        return np.stack(
            [np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)],
            axis=1,
        )

    def proxy_offsets_body(self) -> np.ndarray:
        # Single bounding-sphere model (see body_radius note): the perpendicular
        # arm proxies made the drone too wide to fit standard doorways.
        return np.zeros((1, 3), dtype=np.float64)

    def proxy_world_positions(
        self,
        center: np.ndarray,
        quaternion: np.ndarray,
    ) -> np.ndarray:
        center = np.asarray(center, dtype=np.float64)
        R = quat_to_rotation_matrix(np.asarray(quaternion, dtype=np.float64))
        offsets = self.proxy_offsets_body()
        return center + (R @ offsets.T).T

    def _mesh_floor_z(self, position: np.ndarray) -> float:
        true_floor_z = float(self.scene.bounds.min_corner[2])
        origin = np.asarray(position, dtype=np.float64).copy()
        origin[2] += 0.05
        down = np.array([[0.0, 0.0, -1.0]])
        dists, points, _, _ = self.scene.cast_rays(origin[None, :], down, max_distance=8.0)
        if np.isfinite(dists[0]) and dists[0] < 7.5:
            hit_z = float(points[0][2])
            # Furniture / carpet near the real floor — treat it as the floor itself
            # to avoid raising the collision clamp by the furniture height.
            if hit_z <= true_floor_z + 0.22:
                return true_floor_z
            return hit_z
        return true_floor_z

    def closest_obstacle_at(
        self,
        position: np.ndarray,
        max_range: float = 3.5,
    ) -> Tuple[float, np.ndarray, np.ndarray]:
        pos = np.asarray(position, dtype=np.float64)
        eps = 0.04
        origins = np.repeat(pos[None, :], len(self._probe_dirs), axis=0)
        origins = origins + self._probe_dirs * eps

        dists, points, normals, _ = self.scene.cast_rays(
            origins, self._probe_dirs, max_distance=max_range
        )
        center_dists = dists + eps
        idx = int(np.argmin(center_dists))
        return float(center_dists[idx]), points[idx], normals[idx]

    def closest_obstacle(
        self, position: np.ndarray, max_range: float = 3.5
    ) -> Tuple[float, np.ndarray, np.ndarray]:
        return self.closest_obstacle_at(position, max_range)

    def is_position_free(self, position: np.ndarray, margin: float | None = None) -> bool:
        m = margin if margin is not None else self.cp.clearance_goal
        d, _, _ = self.closest_obstacle_at(position)
        return d >= m

    def push_to_free_space(self, position: np.ndarray, max_iters: int = 12) -> np.ndarray:
        p = np.asarray(position, dtype=np.float64).copy()
        for _ in range(max_iters):
            dist, _, normal = self.closest_obstacle_at(p)
            if dist >= self.cp.clearance_goal:
                return p
            n = normal / (np.linalg.norm(normal) + 1e-9)
            push = (self.cp.body_radius + self.cp.clearance_goal - dist) + self.cp.skin
            p = p + n * min(push, self.cp.max_correction_per_iter * 2.0)
        return p

    def _sweep_point(
        self,
        prev: np.ndarray,
        nxt: np.ndarray,
    ) -> Tuple[np.ndarray, bool]:
        if not self.cp.sweep_enabled:
            return nxt, False

        prev = np.asarray(prev, dtype=np.float64)
        nxt = np.asarray(nxt, dtype=np.float64)
        delta = nxt - prev
        travel = float(np.linalg.norm(delta))
        # Skip sweep when travel is smaller than skin thickness — any motion below
        # skin cannot tunnel through real geometry, and the skin offset would place
        # the ray origin past the destination, causing false-positive hits on distant
        # obstacles that trigger spurious 4-cm teleports every substep.
        if travel < self.cp.skin:
            return nxt, False

        direction = delta / travel
        origin = prev + direction * self.cp.skin
        dists, _, _, _ = self.scene.cast_rays(
            origin[None, :],
            direction[None, :],
            max_distance=travel + self.cp.body_radius,
        )
        hit = float(dists[0])
        if not np.isfinite(hit) or hit > travel + self.cp.body_radius:
            return nxt, False

        safe = max(self.cp.skin, hit - self.cp.body_radius - self.cp.skin)
        return prev + direction * safe, True

    def _resolve_proxy_point(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, bool]:
        dist, _, normal = self.closest_obstacle_at(pos, max_range=2.5)
        if dist >= self.cp.body_radius:
            return pos, vel, False

        n = normal / (np.linalg.norm(normal) + 1e-9)
        penetration = self.cp.body_radius - dist
        correction = min(penetration + self.cp.skin, self.cp.max_correction_per_iter)
        pos = pos + n * correction

        vn = float(np.dot(vel, n))
        if vn < 0.0:
            vel = vel - (1.0 + self.cp.restitution) * vn * n
            vt = vel - np.dot(vel, n) * n
            vel = np.dot(vel, n) * n + vt * (1.0 - self.cp.friction)
        return pos, vel, True

    def resolve(
        self,
        prev_position: np.ndarray,
        position: np.ndarray,
        velocity: np.ndarray,
        quaternion: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        center = np.asarray(position, dtype=np.float64).copy()
        vel = np.asarray(velocity, dtype=np.float64).copy()
        prev_center = np.asarray(prev_position, dtype=np.float64)

        if quaternion is None:
            quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        center, _ = self._sweep_point(prev_center, center)

        prev_proxies = self.proxy_world_positions(prev_center, quaternion)
        proxies = self.proxy_world_positions(center, quaternion)

        for k in range(len(proxies)):
            proxies[k], _ = self._sweep_point(prev_proxies[k], proxies[k])

        any_hit = False
        for _ in range(self.cp.solver_iterations):
            proxy_vel = vel.copy()
            for i in range(len(proxies)):
                proxies[i], proxy_vel, hit = self._resolve_proxy_point(proxies[i], proxy_vel)
                any_hit = any_hit or hit
                if hit:
                    vel = proxy_vel
            if not any_hit:
                break

            correction = proxies.mean(axis=0) - center
            center = center + correction
            proxies = self.proxy_world_positions(center, quaternion)

        floor_z = self._mesh_floor_z(center) + self.cp.body_radius + 0.02
        if center[2] < floor_z:
            center[2] = floor_z
            if vel[2] < 0.0:
                vel[2] = -vel[2] * self.cp.restitution

        b = self.scene.bounds
        margin = self.cp.body_radius + 0.15
        ceiling_z = float(b.max_corner[2]) - margin
        if center[2] > ceiling_z:
            center[2] = ceiling_z
            if vel[2] > 0.0:
                vel[2] = -vel[2] * self.cp.restitution
        for i in range(2):
            lo = b.min_corner[i] + margin
            hi = b.max_corner[i] - margin
            if center[i] < lo:
                center[i] = lo
                vel[i] = max(0.0, vel[i] * 0.2)
            elif center[i] > hi:
                center[i] = hi
                vel[i] = min(0.0, vel[i] * 0.2)

        center, vel = self.enforce_clearance(center, vel, quaternion)
        return center, vel

    def enforce_clearance(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        quaternion: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        center = np.asarray(position, dtype=np.float64).copy()
        vel = np.asarray(velocity, dtype=np.float64).copy()
        if quaternion is None:
            quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        for _ in range(16):
            proxies = self.proxy_world_positions(center, quaternion)
            worst_pen = 0.0
            worst_n = np.array([0.0, 0.0, 1.0])
            for p in proxies:
                dist, _, n = self.closest_obstacle_at(p, max_range=2.0)
                if dist < self.cp.body_radius:
                    pen = self.cp.body_radius - dist
                    if pen > worst_pen:
                        worst_pen = pen
                        worst_n = n / (np.linalg.norm(n) + 1e-9)
            if worst_pen <= 0.0:
                break
            center = center + worst_n * (worst_pen + self.cp.skin)
            vn = float(np.dot(vel, worst_n))
            if vn < 0.0:
                vel = vel - (1.0 + self.cp.restitution) * vn * worst_n

        return center, vel

    def obstacle_repulsion_acceleration(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
    ) -> np.ndarray:
        pos = np.asarray(position, dtype=np.float64)
        vel = np.asarray(velocity, dtype=np.float64)
        accel = np.zeros(3, dtype=np.float64)
        h = self.ap.horizon

        yaw = float(np.arctan2(vel[1], vel[0] + 1e-9))
        angles = np.linspace(-0.85 * np.pi, 0.85 * np.pi, self.ap.num_horizontal_rays)
        horiz_dirs = np.stack(
            [np.cos(yaw + angles), np.sin(yaw + angles), np.zeros_like(angles)],
            axis=1,
        )
        vert_angles = np.linspace(-0.35, 0.35, self.ap.num_vertical_rays)
        vert_dirs = []
        for va in vert_angles:
            c, s = np.cos(yaw), np.sin(yaw)
            vert_dirs.append([c * np.cos(va), s * np.cos(va), np.sin(va)])
        dirs = np.vstack([horiz_dirs, np.asarray(vert_dirs)])

        origins = np.repeat(pos[None, :], len(dirs), axis=0) + dirs * 0.03
        dists, _, normals, _ = self.scene.cast_rays(origins, dirs, max_distance=h)

        for dist, n in zip(dists, normals):
            if dist >= h - 0.05:
                continue
            n_norm = np.linalg.norm(n)
            if n_norm < 1e-6:
                continue
            n_hat = n / n_norm
            strength = ((h - dist) / h) ** 2
            accel += strength * n_hat

        speed = float(np.linalg.norm(accel))
        if speed > self.ap.max_repulsion_accel:
            accel *= self.ap.max_repulsion_accel / speed

        min_d = float(np.min(dists)) if len(dists) else h
        if min_d < 0.9:
            scale = max(0.15, min_d / 0.9)
            accel -= vel * (1.0 - scale) * 0.8

        return accel

    def segment_is_clear(
        self,
        start: np.ndarray,
        end: np.ndarray,
        samples: int = 10,
    ) -> bool:
        a = np.asarray(start, dtype=np.float64)
        b = np.asarray(end, dtype=np.float64)
        for t in np.linspace(0.0, 1.0, samples):
            p = a + t * (b - a)
            d, _, _ = self.closest_obstacle_at(p, max_range=self.cp.clearance_goal + 0.5)
            if d < self.cp.clearance_goal:
                return False
        return True
