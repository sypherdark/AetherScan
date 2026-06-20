"""
Semantic sensor navigation — motion from LiDAR + discovered spatial map.

Flight FSM: EXPLORING → STUCK_BACKUP → STUCK_SPIN → EXPLORING.
VFH uses configuration-space inflation (body radius + safety margin).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np

from core.collision import MeshCollisionSolver
from core.discovery_map import DiscoveryMap
from core.navigation import TrajectorySample
from core.semantic_space import AnalyzedIndoorSpace
from core.sensors import MeshSensorSuite, ObstacleLabel, SensorConfig, SensorFrame
from scene_loader import RedwoodScene


class NavState(Enum):
    EXPLORING = auto()
    STUCK_BACKUP = auto()
    STUCK_SPIN = auto()
    FRONTIER_SCAN = auto()
    SCAN_HOLD = auto()
    ALTITUDE_SCAN = auto()


@dataclass
# NOTE: body_radius right-sized to a realistic small indoor drone (≈0.4 m
# prop-to-prop) so it can pass standard 0.8 m doorways; see collision.py.
class DroneFootprint:
    """Configuration-space radius for VFH inflation (meters)."""

    body_radius: float = 0.18
    safety_margin: float = 0.15

    @property
    def inflation_radius(self) -> float:
        return self.body_radius + self.safety_margin


@dataclass
class NavigatorConfig:
    cruise_speed: float = 0.65
    cruise_altitude: float = 1.5
    vfh_bins: int = 36
    # Standoff distances are measured from the sensor (≈ body centre), so they MUST
    # exceed the body radius (0.32 m) or the drone only stops once its body is
    # already inside the wall — which is exactly how it kept wedging.  Start slowing
    # at 0.9 m, stop with ~0.13 m of body-to-wall clearance at 0.45 m.
    safety_distance: float = 0.9
    stop_distance: float = 0.45
    stuck_distance: float = 0.22
    stuck_time: float = 2.5
    stuck_timer_accel: float = 3.0
    lookahead_time: float = 0.5
    frontier_weight: float = 0.65
    footprint: DroneFootprint = field(default_factory=DroneFootprint)
    backup_speed: float = 0.25
    backup_duration: float = 1.2
    # Rate at which yaw eases toward the direction of travel during exploration.
    # A quadcopter is holonomic (it can translate any direction without facing it),
    # so yaw is decoupled from motion — we rotate gently just so the forward sensors
    # and camera look where the drone is going, never demanding a destabilising slew.
    explore_yaw_rate: float = 1.1
    # Max rate-of-change of the commanded XY velocity reference (m/s²).  Caps the
    # tilt the cascade will demand when the planned travel direction snaps.
    max_cmd_accel: float = 2.5
    spin_angle_rad: float = np.pi / 2.0
    # Spin rate must be fast enough that a 90° recovery turn completes in ~1 s; too
    # slow (≤0.8 rad/s) and the drone never finishes a turn, so it stays in STUCK_SPIN
    # almost permanently instead of returning to EXPLORING.  Attitude stability during
    # the spin is handled by the yaw-torque cap in the controller (not by slowing the
    # slew), so 1.35 rad/s is safe.
    spin_yaw_rate: float = 1.35
    path_lookahead_m: float = 2.0
    frontier_scan_yaw_rate: float = 0.75
    frontier_scan_angle_rad: float = np.pi / 2.0
    memory_cell_size_m: float = 0.75
    linger_radius_m: float = 1.5
    linger_time_s: float = 7.0
    blocked_cost_threshold: float = 0.4
    max_histogram_cost: float = 1e6
    scan_hold_duration: float = 3.0
    # Five altitude planes: start at cruise → low → high → near-floor → near-ceiling.
    # Beginning at 1.0 (cruise altitude = 1.45 m AGL) gives the drone immediate
    # full-room visibility on the first pass.  Subsequent planes fill in low and
    # high coverage for a complete 3-D reconstruction.
    scan_altitude_fractions: Tuple[float, float, float, float, float] = (
        1.0, 0.55, 1.45, 0.30, 1.65
    )
    altitude_scan_climb_rate: float = 0.35
    altitude_scan_settle_s: float = 1.5
    scan_plane_cycle_s: float = 30.0
    ceiling_body_margin_m: float = 0.38
    backup_yaw_rate: float = 0.9

class SensorNavigator:
    def __init__(
        self,
        scene: RedwoodScene,
        analyzed: AnalyzedIndoorSpace,
        cruise_altitude: float = 1.5,
        cruise_speed: float = 0.5,
        sensor_config: SensorConfig | None = None,
        nav_config: NavigatorConfig | None = None,
    ):
        self.scene = scene
        self.analyzed = analyzed
        self.cfg = nav_config or NavigatorConfig(
            cruise_speed=cruise_speed, cruise_altitude=cruise_altitude
        )
        self.sensors = MeshSensorSuite(scene, analyzed, sensor_config)
        self.collision = MeshCollisionSolver(scene)
        self.discovery = DiscoveryMap.from_analyzed_space(analyzed)

        self._nav_state = NavState.EXPLORING
        self._state_timer = 0.0
        self._goal_yaw_world = 0.0
        self._stuck_timer = 0.0
        self._last_progress_pos: Optional[np.ndarray] = None
        self._last_frame: Optional[SensorFrame] = None
        self._intent_path: List[np.ndarray] = []
        self._backup_yaw_world = 0.0
        self._spin_target_yaw = 0.0
        self._spatial_memory: Dict[Tuple[int, int], int] = {}
        self._linger_anchor: Optional[np.ndarray] = None
        self._linger_timer = 0.0
        self._scan_plane_index = 0
        self._sector_cell: Optional[Tuple[int, int]] = None
        # World-space anchor of the last sector scan.  SCAN_HOLD re-triggers only
        # after the drone has travelled ~one cell-width from here — distance-based
        # hysteresis that, unlike a bare cell-index compare, does not flip-flop
        # when the drone hovers exactly on a memory-cell boundary.
        self._sector_anchor: Optional[np.ndarray] = None
        # Committed frontier goal (grid cell) — the drone keeps heading to this
        # frontier until it reaches or explores it, instead of re-selecting the
        # nearest frontier every tick (which made it oscillate in place near spawn).
        self._frontier_goal_ij: Optional[Tuple[int, int]] = None
        self._goal_hold_t: float = 0.0  # seconds the current goal has been held
        self._cmd_vel_prev = np.zeros(3, dtype=np.float64)  # smoothed velocity reference
        self._repulse_prev = np.zeros(3, dtype=np.float64)  # rate-limited repulsion
        self._altitude_cycle_timer = 0.0
        self._altitude_scan_target_z: float = 0.0

    def clear_spatial_memory(self) -> None:
        """Reset visit tracking and blocked regions (end of mission / new run)."""
        self._spatial_memory.clear()
        self._linger_anchor = None
        self._linger_timer = 0.0
        self.discovery.clear_blocked_cells()
        self._scan_plane_index = 0
        self._sector_cell = None
        self._sector_anchor = None
        self._altitude_cycle_timer = 0.0

    def _scan_altitude_agl(self) -> float:
        """Low / mid / high survey planes as fractions of cruise AGL."""
        fracs = self.cfg.scan_altitude_fractions
        idx = min(self._scan_plane_index, len(fracs) - 1)
        return float(self.cfg.cruise_altitude * fracs[idx])

    def _ceiling_cap_z(self, frame: SensorFrame, position: np.ndarray) -> float:
        margin = self.cfg.ceiling_body_margin_m
        b = self.scene.bounds
        box_cap = float(b.max_corner[2]) - margin
        if frame.ceiling_range_m < 6.0:
            sensor_cap = float(position[2] + frame.ceiling_range_m - margin)
            return min(box_cap, sensor_cap)
        return box_cap

    def _altitude_target_z(self, frame: SensorFrame, position: np.ndarray) -> float:
        # Use the normalised scene floor (always at bounds.min_corner[2] ≈ 0) as
        # the reference.  The sensor floor_range is unreliable when furniture or
        # rugs (Z≈0.13 m in Replica scenes) occlude the view of the real floor,
        # which causes the target to hover just above the furniture top instead of
        # ascending to the cruise altitude.
        true_floor_z = float(self.scene.bounds.min_corner[2])

        # Sensor-based floor estimate — only trust it when it sees a genuine floor
        # (below the drone by more than 0.5 m, i.e. not a piece of furniture).
        # When the sensor hits furniture we fall back to the scene floor.
        if frame.floor_range_m < 5.9:
            sensor_floor_z = float(position[2] - frame.floor_range_m)
            if sensor_floor_z > true_floor_z + 0.25:
                # Ray hit furniture/objects, not the real floor — ignore it
                floor_z = true_floor_z
            else:
                floor_z = max(sensor_floor_z, true_floor_z)
        else:
            floor_z = true_floor_z

        floor_z = max(floor_z, true_floor_z + 0.05)
        target = floor_z + self._scan_altitude_agl()
        target = min(target, self._ceiling_cap_z(frame, position))
        min_z = floor_z + self.cfg.footprint.body_radius + 0.12
        return max(target, min_z)

    def _memory_cell(self, position: np.ndarray) -> Tuple[int, int]:
        # np.floor (not int(), which truncates toward zero) so cell indexing is
        # uniform across the origin — int() makes a double-width cell straddling 0.
        r = self.cfg.memory_cell_size_m
        return (int(np.floor(position[0] / r)), int(np.floor(position[1] / r)))

    def _update_spatial_memory(self, position: np.ndarray, dt: float) -> None:
        cell = self._memory_cell(position)
        self._spatial_memory[cell] = self._spatial_memory.get(cell, 0) + 1

        if self._linger_anchor is None:
            self._linger_anchor = position.copy()
            return

        if float(np.linalg.norm(position - self._linger_anchor)) <= self.cfg.linger_radius_m:
            self._linger_timer += dt
        else:
            self._linger_anchor = position.copy()
            self._linger_timer = 0.0

        if self._linger_timer >= self.cfg.linger_time_s:
            # Lingering too long → push exploration onward: temporarily block the
            # over-visited local cells for the frontier BFS AND drop the committed
            # goal so the planner routes to a fresh region.  Blocked cells are
            # cleared periodically (see plan()) so the map heals and the drone is
            # not permanently confined.
            self.discovery.mark_blocked_disk(float(position[0]), float(position[1]), 0.45)
            self._linger_timer = 0.0
            self._linger_anchor = position.copy()
            self._frontier_goal_ij = None

    def _bearing_to_least_visited(self, position: np.ndarray, yaw: float) -> float:
        """Prefer headings toward low visit-count cells (escape FRONTIER_SCAN loops)."""
        best_score = -1.0
        best_angle = yaw
        px, py = float(position[0]), float(position[1])
        for k in range(12):
            angle = yaw + (2.0 * np.pi * k / 12.0)
            probe = position + 2.0 * np.array([np.cos(angle), np.sin(angle), 0.0])
            visits = self._spatial_memory.get(self._memory_cell(probe), 0)
            score = 1.0 / (1.0 + visits)
            if score > best_score:
                best_score = score
                best_angle = angle
        return float(best_angle)

    def scan(
        self,
        position: np.ndarray,
        quaternion: np.ndarray,
        update_discovery: bool = True,
    ) -> SensorFrame:
        """
        Raycast sensors from *position*.

        Args:
            update_discovery: When False the discovery map is NOT updated.
                              Pass False while the mission is idle so that
                              coverage stays at 0% until the user starts a run.
        """
        frame = self.sensors.scan(position, quaternion)
        if update_discovery:
            self.discovery.integrate_scan(frame, scan_agl=self._scan_altitude_agl())
        self._last_frame = frame
        return frame

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return float((a - b + np.pi) % (2 * np.pi) - np.pi)

    def _bin_index(self, bearing_body: float, width: float, bins: int) -> int:
        return int((bearing_body + np.pi) / width) % bins

    def _mark_blocked_arc(
        self,
        hist: np.ndarray,
        bin_angles: np.ndarray,
        bearing_body: float,
        half_width: float,
        cost: float,
        width: float,
        bins: int,
    ) -> None:
        """Set all histogram bins within ±half_width of bearing to blocked cost."""
        if half_width <= 0.0:
            idx = self._bin_index(bearing_body, width, bins)
            hist[idx] = max(hist[idx], cost)
            return

        for i, angle in enumerate(bin_angles):
            if abs(self._angle_diff(float(angle), bearing_body)) <= half_width:
                hist[i] = max(hist[i], cost)

    def _build_polar_histogram(self, frame: SensorFrame) -> Tuple[np.ndarray, np.ndarray]:
        bins = self.cfg.vfh_bins
        hist = np.zeros(bins, dtype=np.float64)
        bin_angles = np.linspace(-np.pi, np.pi, bins, endpoint=False)
        width = 2 * np.pi / bins
        inflate_r = self.cfg.footprint.inflation_radius
        max_cost = self.cfg.max_histogram_cost

        for ret in frame.returns:
            if ret.label in (ObstacleLabel.FLOOR, ObstacleLabel.CEILING, ObstacleLabel.FREE):
                continue
            if ret.range_m >= self.sensors.cfg.lidar_max_range - 0.05:
                continue

            distance = max(float(ret.range_m), inflate_r + 1e-6)
            theta_expand = float(np.arcsin(min(1.0, inflate_r / distance)))

            weight = 1.0
            if ret.label == ObstacleLabel.WALL:
                # Reduced from 2.2 → 1.4: previous value pushed the drone too
                # far from walls, causing sparse wall coverage. Drone now flies
                # closer (while still safe) and accumulates more wall scan hits.
                weight = 1.4
            elif ret.label == ObstacleLabel.OBJECT:
                weight = 1.4
            cost = min(max_cost, weight * (1.0 / max(distance, 0.12)) ** 2)
            if ret.range_m < self.cfg.safety_distance:
                cost = min(max_cost, cost * 3.0)

            self._mark_blocked_arc(
                hist,
                bin_angles,
                float(ret.bearing_body),
                theta_expand,
                cost,
                width,
                bins,
            )

        kernel = np.array([0.15, 0.7, 0.15])
        hist = np.convolve(np.r_[hist[-1], hist, hist[0]], kernel, mode="same")[1:-1]
        return hist, bin_angles

    def _histogram_blocked(self, hist: np.ndarray) -> bool:
        max_cost = float(hist.max()) + 1e-6
        threshold = self.cfg.blocked_cost_threshold * max_cost
        return float(hist.min()) >= threshold

    def _histogram_sector_clear(
        self,
        hist: np.ndarray,
        bin_angles: np.ndarray,
        center_angle: float,
        half_width: float,
    ) -> bool:
        max_cost = float(hist.max()) + 1e-6
        threshold = self.cfg.blocked_cost_threshold * max_cost
        for i, angle in enumerate(bin_angles):
            if abs(self._angle_diff(float(angle), center_angle)) > half_width:
                continue
            if hist[i] > threshold:
                return False
        return True

    def _side_bins_clear(self, hist: np.ndarray, bin_angles: np.ndarray) -> bool:
        """True when ±45° side sectors are passable (corner escape without reverse)."""
        side_half = np.deg2rad(45.0)
        left = self._histogram_sector_clear(hist, bin_angles, np.pi / 2.0, side_half)
        right = self._histogram_sector_clear(hist, bin_angles, -np.pi / 2.0, side_half)
        return left or right

    def _best_side_spin_delta(self, hist: np.ndarray, bin_angles: np.ndarray) -> float:
        side_half = np.deg2rad(45.0)
        left_cost = self._sector_min_cost(hist, bin_angles, np.pi / 2.0, side_half)
        right_cost = self._sector_min_cost(hist, bin_angles, -np.pi / 2.0, side_half)
        return np.pi / 2.0 if left_cost <= right_cost else -np.pi / 2.0

    def _sector_min_cost(
        self,
        hist: np.ndarray,
        bin_angles: np.ndarray,
        center_angle: float,
        half_width: float,
    ) -> float:
        costs = []
        for i, angle in enumerate(bin_angles):
            if abs(self._angle_diff(float(angle), center_angle)) <= half_width:
                costs.append(float(hist[i]))
        return min(costs) if costs else 1e9

    def _rear_lidar_clear(self, frame: SensorFrame, min_dist: float = 0.42) -> bool:
        cone = np.deg2rad(50.0)
        rear = [
            r.range_m
            for r in frame.returns
            if r.label in (ObstacleLabel.WALL, ObstacleLabel.OBJECT)
            and abs(self._angle_diff(r.bearing_body, np.pi)) < cone
        ]
        return (not rear) or float(min(rear)) >= min_dist

    def _handle_forward_blocked(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        hist: np.ndarray,
        bin_angles: np.ndarray,
        dt: float,
    ) -> TrajectorySample:
        """Prefer side spin when flanks are open; reverse only if rear is clear."""
        if self._side_bins_clear(hist, bin_angles):
            delta = self._best_side_spin_delta(hist, bin_angles)
            self._nav_state = NavState.STUCK_SPIN
            self._state_timer = 0.0
            self._spin_target_yaw = float(frame.yaw) + delta
            self._stuck_timer = 0.0
            return self._plan_stuck_spin(position, frame, dt)

        if self._rear_lidar_clear(frame):
            self._enter_stuck_backup(frame.yaw)
            return self._plan_stuck_backup(position, frame, dt)

        delta = self._best_side_spin_delta(hist, bin_angles)
        self._nav_state = NavState.STUCK_SPIN
        self._state_timer = 0.0
        self._spin_target_yaw = float(frame.yaw) + delta
        return self._plan_stuck_spin(position, frame, dt)

    def _pick_heading(
        self,
        hist: np.ndarray,
        bin_angles: np.ndarray,
        goal_body: float,
    ) -> float:
        max_cost = float(hist.max()) + 1e-6
        threshold = self.cfg.blocked_cost_threshold * max_cost
        best_score = 1e9
        best_angle = goal_body

        for i in range(len(hist)):
            if hist[i] > threshold:
                continue
            angle = float(bin_angles[i])
            delta = abs(self._angle_diff(angle, goal_body))
            score = hist[i] + 0.7 * (delta / np.pi)
            if score < best_score:
                best_score = score
                best_angle = angle

        if best_score > 1e8:
            best_angle = float(bin_angles[int(np.argmin(hist))])
        return best_angle

    def _goal_yaw_from_pathfinder(
        self, position: np.ndarray, yaw: float
    ) -> Optional[float]:
        """
        Grid-routed bearing toward a lookahead waypoint on the BFS path to frontier.
        Returns None when no passable route exists.
        """
        path = self.discovery.find_path_to_frontier(position)
        if not path:
            return None

        px, py = float(position[0]), float(position[1])
        lookahead = self.cfg.path_lookahead_m
        target_ij = path[-1]
        accumulated = 0.0

        for k in range(1, len(path)):
            wx, wy = self.discovery._xy_center(path[k][0], path[k][1])
            step = float(np.hypot(wx - px, wy - py))
            accumulated += step
            px, py = wx, wy
            target_ij = path[k]
            if accumulated >= lookahead:
                break

        tx, ty = self.discovery._xy_center(target_ij[0], target_ij[1])
        routed = float(np.arctan2(ty - position[1], tx - position[0]))
        stats = self.discovery.coverage_stats()
        if stats["known_percent"] < 8.0:
            return routed
        return self.cfg.frontier_weight * routed + (1.0 - self.cfg.frontier_weight) * yaw

    def _enter_frontier_scan(self, yaw_world: float, position: np.ndarray) -> None:
        self._nav_state = NavState.FRONTIER_SCAN
        self._state_timer = 0.0
        escape_yaw = self._bearing_to_least_visited(position, yaw_world)
        self._spin_target_yaw = escape_yaw + self.cfg.frontier_scan_angle_rad

    def _speed_from_sensors(self, frame: SensorFrame, heading_body: float) -> float:
        """
        Quadratic deceleration from safety_distance down to stop_distance, then zero.
        """
        cone = np.deg2rad(42)
        ahead = [
            r.range_m
            for r in frame.returns
            if r.label in (ObstacleLabel.WALL, ObstacleLabel.OBJECT)
            and abs(self._angle_diff(r.bearing_body, heading_body)) < cone
        ]
        d_eff = min(ahead) if ahead else 8.0
        # Only let the omnidirectional proximity sensor govern speed when contact is
        # genuinely imminent.  Folding the full proximity_min in here meant ANY object
        # within stop_distance on ANY side (a wall the drone is flying parallel to,
        # furniture beside it) zeroed the forward speed — so in a furnished room the
        # drone permanently read speed≈0, accumulated stuck-time and spin-recovered
        # almost continuously.  Forward clearance is already handled by the 42° cone.
        imminent = self.cfg.stop_distance * 0.5
        if frame.proximity_min < imminent:
            d_eff = min(d_eff, frame.proximity_min)

        stop_d = self.cfg.stop_distance
        safe_d = self.cfg.safety_distance

        if d_eff <= stop_d:
            return 0.0
        if d_eff >= safe_d:
            return self.cfg.cruise_speed

        span = max(safe_d - stop_d, 1e-6)
        t = (d_eff - stop_d) / span
        return self.cfg.cruise_speed * float(t * t)

    def _altitude_velocity(self, frame: SensorFrame, position: np.ndarray) -> float:
        target_z = self._altitude_target_z(frame, position)
        err = target_z - float(position[2])
        vz = float(np.clip(err * 1.15, -0.42, 0.42))

        if frame.ceiling_range_m < 1.15:
            vz = min(vz, 0.05)
            if frame.ceiling_range_m < 0.85:
                vz = min(vz, -0.08)

        # Minimum-climb guard: use altitude-above-ground-level from scene bounds
        # (the normalised floor is always at scene.bounds.min_corner[2]).
        # This avoids spurious triggering when the sensor sees furniture as "floor".
        true_floor_z = float(self.scene.bounds.min_corner[2])
        agl = float(position[2]) - true_floor_z
        if agl < 0.55:
            # Drone is still very close to the actual floor — force climb
            vz = max(vz, 0.28)
        return float(np.clip(vz, -0.42, 0.42))

    def _maybe_cycle_scan_plane(self, dt: float) -> None:
        self._altitude_cycle_timer += dt
        if self._altitude_cycle_timer < self.cfg.scan_plane_cycle_s:
            return
        self._altitude_cycle_timer = 0.0
        n = len(self.cfg.scan_altitude_fractions)
        self._scan_plane_index = (self._scan_plane_index + 1) % n
        self.discovery.clear_blocked_cells()
        self._nav_state = NavState.ALTITUDE_SCAN
        self._state_timer = 0.0
        if self._last_frame is not None:
            self._altitude_scan_target_z = self._altitude_target_z(
                self._last_frame, self._last_frame.position
            )

    def _enter_scan_hold(self) -> None:
        self._nav_state = NavState.SCAN_HOLD
        self._state_timer = 0.0

    def _update_progress(self, position: np.ndarray, dt: float) -> None:
        if self._last_progress_pos is None:
            self._last_progress_pos = position.copy()
            return
        if float(np.linalg.norm(position - self._last_progress_pos)) > self.cfg.stuck_distance:
            self._stuck_timer = 0.0
            self._last_progress_pos = position.copy()
        else:
            self._stuck_timer += dt

    def _enter_stuck_backup(self, yaw_world: float) -> None:
        self._nav_state = NavState.STUCK_BACKUP
        self._state_timer = 0.0
        self._backup_yaw_world = float(yaw_world)
        self._stuck_timer = 0.0

    def _enter_stuck_spin(self, yaw_world: float) -> None:
        self._nav_state = NavState.STUCK_SPIN
        self._state_timer = 0.0
        self._spin_target_yaw = float(yaw_world) + self.cfg.spin_angle_rad

    def _plan_exploring(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        self._update_spatial_memory(position, dt)
        self._update_progress(position, dt)
        self._maybe_cycle_scan_plane(dt)

        # Holonomic un-wedge.  If the body is dangerously close to geometry, strafe
        # STRAIGHT away from the nearest obstacle.  A quadcopter can translate in any
        # direction, so the correct escape is a lateral push — NOT a spin-in-place
        # (which never moves the body off the wall and was trapping the drone for the
        # whole mission).  A path may still exist; we just un-wedge first, then plan.
        body_r = self.cfg.footprint.body_radius
        if frame.proximity_min < body_r + 0.12:
            away_world = float(frame.yaw + frame.proximity_body_angle + np.pi)
            esc = 0.4
            vel = np.array(
                [esc * np.cos(away_world), esc * np.sin(away_world),
                 self._altitude_velocity(frame, position)],
                dtype=np.float64,
            )
            pos_des = position + vel * self.cfg.lookahead_time
            pos_des[2] = self._altitude_target_z(frame, position)
            self._stuck_timer = 0.0
            self._intent_path = [position.copy(), pos_des.copy()]
            return TrajectorySample(position=pos_des, velocity=vel, yaw=float(frame.yaw))

        # ── Deliberative exploration: follow the global path to the best frontier ──
        # The occupancy grid integrates every tick, so we replan continuously.  The
        # BFS path already routes around mapped obstacles; we follow it holonomically
        # (translate straight toward a look-ahead point) instead of the old "rotate to
        # face, then creep forward" reactive steering that trapped the drone in
        # perpetual spin-recovery.
        # Goal-committed frontier selection.  Keep heading to the committed frontier
        # until we reach it (≈0.6 m) or it has been explored away; only then pick a
        # new one.  This stops the drone oscillating between equidistant frontiers.
        path: Optional[List[Tuple[int, int]]] = None
        goal = self._frontier_goal_ij
        if goal is not None:
            self._goal_hold_t += dt
            gx, gy = self.discovery._xy_center(*goal)
            reached = float(np.hypot(gx - position[0], gy - position[1])) < 0.6
            # Release on: (a) physically reached (we've now scanned it up close),
            # (b) a long safety cap so a goal can never trap us forever, or
            # (c) the route genuinely fails.  The goal is a vantage point chosen
            # for un-scanned area; "reached" is the correct completion signal for
            # both discovery and coverage goals.
            if reached or self._goal_hold_t > 25.0:
                self._frontier_goal_ij = None
            else:
                path = self.discovery.find_path_to_cell(position, goal)
                if not path:
                    self._frontier_goal_ij = None  # committed goal became unreachable
        if self._frontier_goal_ij is None:
            path = self.discovery.find_path_to_frontier(position)
            if path and len(path) >= 1:
                self._frontier_goal_ij = path[-1]
                self._goal_hold_t = 0.0

        if not path or len(path) < 1:
            # Nothing reachable from here — sweep toward unexplored space (or, if the
            # whole reachable area is mapped, the run is effectively complete).
            self._enter_frontier_scan(frame.yaw, position)
            return self._plan_frontier_scan(position, frame, dt)

        target_xy = self._path_lookahead_point(path, position, self.cfg.path_lookahead_m)
        to_target = target_xy - position[:2]
        dist = float(np.linalg.norm(to_target))
        travel_dir = float(np.arctan2(to_target[1], to_target[0])) if dist > 1e-6 else float(frame.yaw)

        # Speed: cruise, decelerated only by obstacles actually in the travel cone.
        # Move DECISIVELY toward the look-ahead point — do NOT throttle just because
        # the current frontier is nearby (near spawn every frontier is close, and the
        # old distance throttle made the drone creep so slowly it looked frozen).
        # Only ease off in the final approach to the very last waypoint of a short path.
        speed = float(min(self._speed_from_sensors(frame, self._angle_diff(travel_dir, frame.yaw)),
                          self.cfg.cruise_speed))
        if len(path) <= 2 and dist < 0.4:
            speed = min(speed, 0.25)
        if frame.proximity_min < self.cfg.stop_distance * 0.5:
            speed = 0.0
        if speed <= 1e-6:
            self._stuck_timer += dt * self.cfg.stuck_timer_accel

        vel_xy = speed * np.array([np.cos(travel_dir), np.sin(travel_dir)], dtype=np.float64)
        vz = self._altitude_velocity(frame, position)
        vel = np.array([vel_xy[0], vel_xy[1], vz], dtype=np.float64)

        # Yaw eases toward the travel direction, rate-limited (holonomic motion does
        # NOT wait for the turn).
        yaw_err = self._angle_diff(travel_dir, frame.yaw)
        yaw_cmd = float(frame.yaw) + float(
            np.clip(yaw_err, -self.cfg.explore_yaw_rate * dt, self.cfg.explore_yaw_rate * dt)
        )

        pos_des = position + vel * self.cfg.lookahead_time
        pos_des[2] = self._altitude_target_z(frame, position)
        # Visualise the actual planned grid route (next few waypoints).
        self._intent_path = [position.copy()] + [
            np.array([*self.discovery._xy_center(c[0], c[1]), pos_des[2]], dtype=np.float64)
            for c in path[1:8]
        ]
        return TrajectorySample(position=pos_des, velocity=vel, yaw=yaw_cmd)

    def _path_lookahead_point(
        self, path: List[Tuple[int, int]], position: np.ndarray, lookahead: float
    ) -> np.ndarray:
        """World-XY look-ahead point: walk along the grid path until ``lookahead``
        metres have accumulated, returning that waypoint (or the path's end)."""
        px, py = float(position[0]), float(position[1])
        acc = 0.0
        target = np.array(self.discovery._xy_center(path[-1][0], path[-1][1]), dtype=np.float64)
        for k in range(1, len(path)):
            wx, wy = self.discovery._xy_center(path[k][0], path[k][1])
            acc += float(np.hypot(wx - px, wy - py))
            px, py = wx, wy
            if acc >= lookahead:
                return np.array([wx, wy], dtype=np.float64)
        return target

    def _plan_scan_hold(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        """Hold position and zero velocity while sensors stabilize (3 s)."""
        self._state_timer += dt
        yaw = float(frame.yaw)
        vel = np.zeros(3, dtype=np.float64)
        if self._state_timer >= self.cfg.scan_hold_duration:
            self._nav_state = NavState.EXPLORING
            self._state_timer = 0.0
            # Re-anchor the sector scan at the current position so the next
            # SCAN_HOLD only fires after another full cell-width of travel.
            self._sector_anchor = position.copy()
        self._intent_path = [position.copy()]
        return TrajectorySample(position=position.copy(), velocity=vel, yaw=yaw)

    def _plan_altitude_scan(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        """Climb/descend to the next survey plane before resuming exploration."""
        self._state_timer += dt
        self._altitude_scan_target_z = self._altitude_target_z(frame, position)
        err = self._altitude_scan_target_z - float(position[2])
        vz = float(
            np.clip(
                err * self.cfg.altitude_scan_climb_rate * 2.0,
                -self.cfg.altitude_scan_climb_rate,
                self.cfg.altitude_scan_climb_rate,
            )
        )
        yaw = float(frame.yaw)
        vel = np.array([0.0, 0.0, vz], dtype=np.float64)
        if abs(err) < 0.14 and self._state_timer >= self.cfg.altitude_scan_settle_s:
            self._nav_state = NavState.EXPLORING
            self._state_timer = 0.0
            vel[2] = 0.0
        pos_des = position + vel * self.cfg.lookahead_time
        self._intent_path = [position.copy(), pos_des.copy()]
        return TrajectorySample(position=pos_des, velocity=vel, yaw=yaw)

    def _plan_stuck_backup(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        if not self._rear_lidar_clear(frame):
            self._nav_state = NavState.STUCK_SPIN
            self._state_timer = 0.0
            self._spin_target_yaw = float(frame.yaw) + self.cfg.spin_angle_rad
            return self._plan_stuck_spin(position, frame, dt)

        self._state_timer += dt
        yaw = float(self._backup_yaw_world)
        yaw_cmd = yaw + self.cfg.backup_yaw_rate * dt
        speed = self.cfg.backup_speed
        vel_xy = -speed * np.array([np.cos(yaw_cmd), np.sin(yaw_cmd)], dtype=np.float64)
        vz = self._altitude_velocity(frame, position)
        vel = np.array([vel_xy[0], vel_xy[1], vz])

        if self._state_timer >= self.cfg.backup_duration:
            self._enter_stuck_spin(frame.yaw)

        pos_des = position + vel * self.cfg.lookahead_time
        pos_des[2] = self._altitude_target_z(frame, position)   # braking reference for Z
        self._intent_path = [position.copy(), position + vel * 0.7]
        return TrajectorySample(position=pos_des, velocity=vel, yaw=float(yaw_cmd))

    def _plan_frontier_scan(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        """No global path is currently reachable — relocate HOLONOMICALLY toward
        the least-visited direction to uncover new frontiers.  Level flight, gentle
        yaw, obstacle-decelerated — NO spin-in-place (which destabilised attitude
        and never actually relocated the drone)."""
        self._state_timer += dt

        # The instant a real route opens up, hand straight back to path-following.
        if self.discovery.find_path_to_frontier(position):
            self._nav_state = NavState.EXPLORING
            self._state_timer = 0.0
            return self._plan_exploring(position, frame, dt)

        bearing = float(self.discovery.frontier_direction(position, frame.yaw))
        speed = self._speed_from_sensors(frame, self._angle_diff(bearing, frame.yaw))
        speed = float(min(speed, 0.35))
        if frame.proximity_min < self.cfg.stop_distance * 0.5:
            speed = 0.0
        vel_xy = speed * np.array([np.cos(bearing), np.sin(bearing)], dtype=np.float64)
        vz = self._altitude_velocity(frame, position)
        vel = np.array([vel_xy[0], vel_xy[1], vz], dtype=np.float64)
        yaw_err = self._angle_diff(bearing, frame.yaw)
        yaw_cmd = float(frame.yaw) + float(
            np.clip(yaw_err, -self.cfg.explore_yaw_rate * dt, self.cfg.explore_yaw_rate * dt)
        )
        # Bounded: after a while with no reachable frontier the run is effectively
        # done; drop back to EXPLORING (which will re-evaluate or idle in place).
        if self._state_timer > 6.0:
            self._nav_state = NavState.EXPLORING
            self._state_timer = 0.0
        pos_des = position + vel * self.cfg.lookahead_time
        pos_des[2] = self._altitude_target_z(frame, position)
        self._intent_path = [position.copy(), pos_des.copy()]
        return TrajectorySample(position=pos_des, velocity=vel, yaw=yaw_cmd)

    def _plan_stuck_spin(
        self,
        position: np.ndarray,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        self._state_timer += dt
        yaw = float(frame.yaw)
        err = self._angle_diff(self._spin_target_yaw, yaw)
        step = self.cfg.spin_yaw_rate * dt
        if abs(err) <= step:
            yaw_cmd = self._spin_target_yaw
            self._nav_state = NavState.EXPLORING
            self._stuck_timer = 0.0
            self._state_timer = 0.0
            self._last_progress_pos = position.copy()
        else:
            yaw_cmd = yaw + np.sign(err) * step

        # Use the altitude TARGET (not current position) for the Z component of
        # pos_des.  This gives the position PID a braking reference so the drone
        # decelerates as it approaches cruise altitude instead of over-shooting.
        target_z = self._altitude_target_z(frame, position)
        pos_des = position.copy()
        pos_des[2] = target_z
        vel = np.array([0.0, 0.0, self._altitude_velocity(frame, position)])
        self._intent_path = [position.copy()]
        return TrajectorySample(position=pos_des, velocity=vel, yaw=float(yaw_cmd))

    def plan(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        quaternion: np.ndarray,
        dt: float,
        frame: Optional[SensorFrame] = None,
    ) -> TrajectorySample:
        if frame is None:
            frame = self.scan(position, quaternion)

        if self._nav_state == NavState.EXPLORING:
            sample = self._plan_exploring(position, frame, dt)
        elif self._nav_state == NavState.SCAN_HOLD:
            sample = self._plan_scan_hold(position, frame, dt)
        elif self._nav_state == NavState.ALTITUDE_SCAN:
            sample = self._plan_altitude_scan(position, frame, dt)
        elif self._nav_state == NavState.FRONTIER_SCAN:
            sample = self._plan_frontier_scan(position, frame, dt)
        elif self._nav_state == NavState.STUCK_BACKUP:
            sample = self._plan_stuck_backup(position, frame, dt)
            if self._nav_state == NavState.STUCK_SPIN:
                sample = self._plan_stuck_spin(position, frame, dt)
        else:
            sample = self._plan_stuck_spin(position, frame, dt)

        return self._smooth_reference(position, sample, frame, dt)

    def _smooth_reference(
        self,
        position: np.ndarray,
        sample: TrajectorySample,
        frame: SensorFrame,
        dt: float,
    ) -> TrajectorySample:
        """Jerk/acceleration-limit the HORIZONTAL commanded-velocity reference.

        The planners emit a velocity that can step ~45°/tick when the frontier
        goal or path tangent changes.  The cascade then chases the discontinuity
        with a violent tilt (transients to 60°+).  Rate-limiting the change of the
        XY command (‖Δv‖ ≤ a_max·dt) gives the controller a reference it can track
        with a modest, smooth tilt.  Z (altitude) is left untouched — it has its
        own slow loop — and pos_des is recomputed from the smoothed velocity so the
        position-PID feed-forward stays consistent.
        """
        v_cmd = np.asarray(sample.velocity, dtype=np.float64).copy()
        prev = self._cmd_vel_prev
        dv = v_cmd[:2] - prev[:2]
        max_dv = self.cfg.max_cmd_accel * dt
        n = float(np.hypot(dv[0], dv[1]))
        if n > max_dv:
            dv = dv * (max_dv / n)
        v_xy = prev[:2] + dv
        smoothed = np.array([v_xy[0], v_xy[1], v_cmd[2]], dtype=np.float64)
        self._cmd_vel_prev = smoothed

        pos_des = position + smoothed * self.cfg.lookahead_time
        pos_des[2] = sample.position[2]  # preserve the altitude target / braking ref
        return TrajectorySample(position=pos_des, velocity=smoothed, yaw=sample.yaw)

    def repulsion_from_sensors(self, frame: SensorFrame) -> np.ndarray:
        if self._nav_state != NavState.EXPLORING:
            return np.zeros(3, dtype=np.float64)

        # Gentle, SHORT-RANGE safety nudge only.  The global path already routes
        # clear of mapped obstacles, so repulsion is just a reflex for close calls.
        # The old version (1.8 m range, weight 2.5, 3.5 m/s² cap) applied a near-
        # max lateral acceleration whenever a wall was within 1.8 m — which is
        # almost always indoors — pinning the drone at ~30° tilt and saturating the
        # controller so it could not actually translate out of corners.
        REACT_RANGE = 0.8
        accel = np.zeros(3)
        for ret in frame.walls_and_objects():
            if ret.range_m > REACT_RANGE:
                continue
            n = ret.surface_normal / (np.linalg.norm(ret.surface_normal) + 1e-9)
            w = 1.0 if ret.label == ObstacleLabel.WALL else 0.7
            accel += w * ((REACT_RANGE - ret.range_m) / REACT_RANGE) ** 2 * n
        # Horizontal-only (vertical clearance is owned by _altitude_velocity).
        accel[2] = 0.0
        mag = float(np.linalg.norm(accel))
        if mag > 1.2:
            accel *= 1.2 / mag
        # Rate-limit the repulsion vector.  It is fed straight into the controller's
        # ax/ay (bypassing the velocity-reference smoother), so a sudden close-call
        # near a wall stepped the lateral accel and snapped the tilt — the dominant
        # cause of the 50–60° transients on cluttered scenes.  Limiting ‖Δaccel‖ per
        # tick (~6 m/s³ jerk) keeps avoidance responsive but never violent.
        dprev = accel - self._repulse_prev
        max_da = 0.06  # per control tick (≈6 m/s² · 0.01 s)
        n = float(np.linalg.norm(dprev))
        if n > max_da:
            accel = self._repulse_prev + dprev * (max_da / n)
        self._repulse_prev = accel.copy()
        return accel

    def get_intent_path(self) -> List[List[float]]:
        return [p.tolist() for p in self._intent_path]

    def telemetry_extra(self) -> dict:
        extra = {
            "space_analysis": self.analyzed.summary(),
            "discovery": self.discovery.coverage_stats(),
            "discovered_map": self.discovery.local_occupancy_slice(
                self._last_frame.position if self._last_frame else np.zeros(3),
                radius_m=3.5,
                max_cells=200,
            ),
            "nav_state": self._nav_state.name,
        }
        return extra
