"""
Fixed-timestep simulation engine — sensor-driven navigation against 3D mesh.

Headless mode feeds the Next.js dashboard via ``redwood_sim.bridge`` (no Open3D window).
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

import numpy as np

from config import SimConfig
from core.collision import CollisionParams, MeshCollisionSolver
from core.controls import CascadingFlightController, FlightGains, QuadcopterParams
from core.navigation import TrajectorySample
from core.semantic_space import AnalyzedIndoorSpace
from core.sensor_navigation import SensorNavigator
from scene_loader import RedwoodScene
from visualization.camera_capture import capture_drone_view
from visualization.renderer import SimulationRenderer

CRUISE_ALTITUDE_AGL_M = 1.45


class SimulationEngine:
    def __init__(
        self,
        scene: RedwoodScene,
        config: SimConfig | None = None,
        headless: bool = False,
        scene_id: str = "apartment",
    ):
        self.config = config or SimConfig()
        self.scene = scene
        self.scene_id = scene_id
        self.headless = headless
        self.renderer: SimulationRenderer | None = None
        if not headless:
            self.renderer = SimulationRenderer(
                scene, self.config.window_width, self.config.window_height
            )

        spawn, cruise_agl = self._default_spawn(scene)
        # spawn XY is chosen at cruise altitude to avoid furniture columns.
        # Keep Z at cruise altitude so the drone starts stable without a climb phase
        # that could overshoot.  The collision clamp (floor_z = true_floor + body_radius
        # + 0.02 ≈ 0.34 m) would otherwise teleport the drone from floor+0.12 each
        # substep, building momentum that overshoots the altitude target badly.
        spawn[2] = float(scene.bounds.min_corner[2]) + cruise_agl

        params = QuadcopterParams(
            ground_effect_height=self.config.ground_effect_height,
            ground_effect_gain=self.config.ground_effect_gain,
        )
        self.collision = MeshCollisionSolver(
            scene,
            CollisionParams(
                body_radius=self.config.drone_body_radius,
                arm_length=params.arm_length,
            ),
        )
        print(f"[sim] Cruise spawn → ({spawn[0]:.2f}, {spawn[1]:.2f}, {spawn[2]:.2f}) m")

        self.controller = CascadingFlightController(params, FlightGains())
        self.controller.reset(spawn, yaw=0.0)
        self.controller.set_floor_height_fn(
            lambda p, col=self.collision: col._mesh_floor_z(
                np.asarray(p, dtype=np.float64)
            )
        )

        print("[sim] Analyzing 3D structure (walls, objects, free space)...")
        flight_z = float(spawn[2] + cruise_agl)
        self.analyzed_space = AnalyzedIndoorSpace.build(scene, flight_height=flight_z)
        summary = self.analyzed_space.summary()
        sem_tag = (
            f", semantic_triangles={len(scene._triangle_semantics):,}"
            if scene.has_triangle_semantics
            else ", semantics=heuristic"
        )
        print(
            f"[sim] Space model: {summary['wall_elements']} wall groups, "
            f"{summary['object_elements']} object clusters, "
            f"{summary['free_percent']}% navigable cells{sem_tag}"
        )

        self.navigator = SensorNavigator(
            scene,
            self.analyzed_space,
            cruise_altitude=cruise_agl,
            cruise_speed=self.config.patrol_speed,
        )
        # autonomous=True so the idle floor-target traj in run_navigation_phase keeps
        # the drone on the ground without it hovering in mid-air.
        self.controller.autonomous = True

        self.mission_state = "IDLE"
        self._mission_active = False
        self._elapsed = 0.0
        self._distance = 0.0
        self._coverage = 0.0
        self._mapped_cells: set[tuple[int, int]] = set()
        self._last_pos: Optional[np.ndarray] = None
        self._sensor_frame = None
        # Persistent, voxel-deduplicated reconstruction cloud (the authoritative
        # scan output).  Grows monotonically during a mission, reset on start.
        # Deduplication is what makes it stable: per-frame lidar hits are ~80%
        # duplicates, so the old "stream every hit" approach filled the client's
        # 200k buffer with redundant points in ~2 min and then FIFO-dropped the
        # OLDEST (first-scanned) points — areas appeared to be "forgotten".
        self._recon_points: List[List[float]] = []          # [x,y,z,label]
        self._recon_index: Dict[tuple, int] = {}             # voxel key -> idx
        self._recon_voxel = 0.05                              # 5 cm reconstruction voxel
        # Per-(x,y)-column vertical extent, used to tell tall walls from short
        # furniture during semantic refinement (a wall column reaches the ceiling;
        # a sofa/table column does not).  Keyed at a coarse 0.20 m XY resolution.
        self._col_extent: Dict[tuple, List[float]] = {}      # (xi,yi) -> [zmin,zmax]
        self._col_voxel = 0.20
        self._camera_gallery: List[Dict[str, Any]] = []
        self._last_snapshot_known_pct = 0.0
        self._last_snapshot_coverage = 0.0
        self._cruise_agl = cruise_agl

        # --- God mode -------------------------------------------------------
        # An accelerated survey mode: the drone cruises faster AND the bridge
        # steps the simulation multiple physics cycles per real-time tick, so a
        # full scan completes in a fraction of the wall-clock time.  Time
        # acceleration (god_substeps) is dynamics-neutral — it runs the *same*
        # stable controller faster — while the cruise boost is kept modest so
        # we don't reintroduce the tilt/altitude blow-ups that >0.9 m/s caused.
        self._god_mode = False
        self._base_cruise = float(self.navigator.cfg.cruise_speed)
        self.god_substeps = 1            # physics ticks per bridge loop iteration

        # --- State estimation (real-drone fidelity) -------------------------
        # A real indoor drone has no ground-truth pose; it estimates a drifting,
        # noisy pose.  When config.use_estimated_pose is set, the reconstruction
        # is built from this estimate (range/bearing reprojected through the
        # estimated pose) so the sim-to-real localization gap is measurable.
        from core.state_estimation import PoseEstimator
        from core.scan_matching import CorrelativeScanMatcher
        self.estimator = PoseEstimator()
        self._last_est = None            # most recent EstimatedState (diagnostics)
        # SLAM correction (roadmap item 3): a second occupancy map built purely
        # from the ESTIMATED pose ("perceived map" — what a real drone's mapper
        # holds), plus a correlative scan matcher that periodically re-localizes
        # the estimate against it, bounding the otherwise-unbounded drift.
        # Uncertainty-aware standoff (DARPA SubT lesson: degrade gracefully under
        # localization error): when flying on the estimate, planned goals are
        # offset from truth by the residual drift (~0.1–0.2 m), so the REACTIVE
        # standoffs — which work on true sensor ranges and never seal doorways
        # the way grid inflation does — absorb that margin.
        if self.config.use_estimated_pose:
            self.navigator.cfg.stop_distance += 0.10
            self.navigator.cfg.safety_distance += 0.15

        self.scan_matcher = CorrelativeScanMatcher()
        self._match_grid = None
        self._match_countdown = 0
        self._slam_corrections = 0
        self._last_match_score = 0.0
        self._slam_health = 1.0          # EMA of match score; gates cruise speed
        # Keyframe bookkeeping: the match grid only accepts a scan after the
        # drone has moved meaningfully since the last insert (see
        # _update_perceived_slam for why per-tick insertion diverges).
        self._kf_pos = None
        self._kf_yaw = 0.0

        self._accumulator = 0.0
        self._running = True
        self._register_keys()

    def set_god_mode(self, enabled: bool) -> None:
        """Toggle accelerated survey mode (faster cruise + time acceleration)."""
        enabled = bool(enabled)
        if enabled == self._god_mode:
            return
        self._god_mode = enabled
        self.god_substeps = 3 if enabled else 1
        self._apply_cruise_speed()

    def _apply_cruise_speed(self) -> None:
        """Single owner of the cruise-speed setting: base × god × SLAM health.

        Field practice from SLAM-payload drones (Emesent Hovermap): the autonomy
        must fly the vehicle IN A WAY THAT KEEPS SLAM HEALTHY — at the right
        speed, near features.  And the DARPA SubT teams' core lesson: state-
        estimation degradation cascades through the whole stack unless the
        system degrades gracefully.  So when the scan-match health drops (feature
        -poor view, immature map), the drone slows down instead of outrunning
        its own localization; speed restores when matching recovers.
        """
        factor = 1.6 if self._god_mode else 1.0
        if self.config.use_estimated_pose and self._slam_health < 0.5:
            factor *= 0.65
        self.navigator.cfg.cruise_speed = self._base_cruise * factor

    @staticmethod
    def _find_interior_floor_spawn(scene: RedwoodScene) -> np.ndarray:
        """
        Guarantee an interior spawn by grid-searching XY positions at known floor height.

        Strategy:
        1. Scene is normalised so the actual floor is at z = min_corner[2] ≈ 0.
        2. We sample XY candidates at cruise height (floor + 1.45 m) and require:
           a. >= 0.85 m horizontal clearance in 12 directions (not inside a wall).
           b. The downward sensor ray hits within 0.25 m of the true floor (no rug /
              furniture at the spawn column, which would confuse altitude control).
        3. We try progressively lower clearance requirements so we always find something.
        """
        b = scene.bounds
        cx, cy = float(b.center[0]), float(b.center[1])
        true_floor_z = float(b.min_corner[2])
        # Evaluate candidates at the cruise altitude — this is where the drone will
        # actually fly, so horizontal clearance at this height matters most.
        check_z = true_floor_z + CRUISE_ALTITUDE_AGL_M
        min_clearance = 0.85

        # Build candidates: AABB center first, then fine grid across the footprint
        candidates: list[tuple[float, float]] = [(cx, cy)]
        step = max(0.35, min(b.extent[0], b.extent[1]) * 0.10)
        for dx in np.arange(-b.extent[0] * 0.45, b.extent[0] * 0.45 + 0.01, step):
            for dy in np.arange(-b.extent[1] * 0.45, b.extent[1] * 0.45 + 0.01, step):
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    candidates.append((cx + float(dx), cy + float(dy)))

        angles = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        h_dirs = np.stack([np.cos(angles), np.sin(angles), np.zeros(12)], axis=1).astype(np.float64)
        dn = np.array([[0.0, 0.0, -1.0]])

        def _is_good(ox: float, oy: float, min_clear: float) -> bool:
            probe = np.array([ox, oy, check_z], dtype=np.float64)
            # Check horizontal clearance
            origins = np.tile(probe, (12, 1))
            dists, _, _, _ = scene.cast_rays(origins, h_dirs, max_distance=min_clear + 0.1)
            if float(np.min(dists)) < min_clear:
                return False
            # Check that the floor below is the real floor (no furniture blocking).
            # Fire a downward ray from check_z; expect to travel close to check_z
            # (almost all the way to Z=0).
            fd, fp, _, _ = scene.cast_rays(probe[None, :], dn, max_distance=check_z + 0.5)
            if not np.isfinite(fd[0]) or fd[0] > check_z + 0.3:
                return False  # no floor at all — skip
            hit_z = float(fp[0][2])
            # Accept if the ray hit the actual floor (close to true_floor_z)
            return hit_z <= true_floor_z + 0.20

        # Pick the most CENTRAL candidate that is genuinely ENCLOSED.
        #
        # A previous version scored by raw clearance ("most open"), but Replica
        # meshes are open scans, not sealed boxes — a point OUTSIDE the building has
        # the most clearance of all (its rays escape to infinity and read max range),
        # so the drone spawned in the open void next to the structure.  An interior
        # point is instead SURROUNDED: most horizontal rays hit a wall within the
        # building's extent.  We require that enclosure, a real floor below, and a
        # minimum wall clearance, then prefer the most central such point (central =
        # inside, and away from the corners that used to trap the planner at start).
        enclose_range = float(max(b.extent[0], b.extent[1])) + 1.0

        def _probe(ox: float, oy: float) -> tuple[float, float]:
            """Return (min_clearance, enclosure_fraction) at cruise height."""
            probe = np.array([ox, oy, check_z], dtype=np.float64)
            origins = np.tile(probe, (12, 1))
            dists, _, _, _ = scene.cast_rays(origins, h_dirs, max_distance=enclose_range)
            hit = dists < enclose_range - 0.05
            return float(np.min(dists)), float(np.mean(hit))

        # Two passes: first demand strong enclosure + clear floor + good clearance,
        # then relax so a spawn is always found.
        for min_clear, min_enclosure, need_floor in (
            (0.60, 0.80, True),
            (0.45, 0.70, True),
            (0.40, 0.60, False),
        ):
            best_probe: Optional[np.ndarray] = None
            best_dist = 1e9
            for ox, oy in candidates:
                ox = float(np.clip(ox, b.min_corner[0] + 0.4, b.max_corner[0] - 0.4))
                oy = float(np.clip(oy, b.min_corner[1] + 0.4, b.max_corner[1] - 0.4))
                clr, enclosure = _probe(ox, oy)
                if clr < min_clear or enclosure < min_enclosure:
                    continue  # inside a wall, or not surrounded (i.e. outside)
                probe = np.array([ox, oy, check_z], dtype=np.float64)
                if need_floor:
                    fd, fp, _, _ = scene.cast_rays(probe[None, :], dn, max_distance=check_z + 0.5)
                    if not np.isfinite(fd[0]) or float(fp[0][2]) > true_floor_z + 0.20:
                        continue  # no real floor below (void or furniture column)
                # Prefer the most central qualifying point (deepest inside the room).
                dist_from_center = float(np.hypot(ox - cx, oy - cy))
                if dist_from_center < best_dist:
                    best_dist = dist_from_center
                    best_probe = probe
            if best_probe is not None:
                return best_probe

        # Absolute fallback: scene centroid at cruise altitude
        return np.array([cx, cy, check_z], dtype=np.float64)

    @staticmethod
    def _default_spawn(scene: RedwoodScene) -> tuple[np.ndarray, float]:
        """
        Dynamically find the actual room interior and place drone just above the floor.
        Returns (spawn_xyz_in_ROS_frame, cruise_altitude_above_floor_m).
        """
        spawn = SimulationEngine._find_interior_floor_spawn(scene)
        print(f"[sim] Spawn resolved → ({spawn[0]:.2f}, {spawn[1]:.2f}, {spawn[2]:.2f}) m")
        return spawn, CRUISE_ALTITUDE_AGL_M

    def _register_keys(self) -> None:
        r = self.renderer
        if r is None or not hasattr(r, "vis"):
            return

        def toggle_auto(v):
            self.controller.autonomous = not self.controller.autonomous
            return False

        def emergency(v):
            s = self.controller.state
            self.controller.autonomous = True
            self.controller.manual_velocity[:] = 0
            s.velocity *= 0.15
            s.omega_b *= 0.15
            return False

        r.register_key(ord("A"), toggle_auto)
        r.register_key(32, emergency)
        r.register_key(256, lambda v: self._stop(v))

    def _stop(self, _vis) -> bool:
        self._running = False
        self.renderer._running = False
        return False

    def run_sensor_phase(self) -> "SensorFrame":
        """Simulation: raycast LiDAR/proximity against loaded mesh.

        The discovery map is only updated when a mission is active so that
        coverage reads exactly 0 % on the dashboard before the user starts.

        When idle (no active mission) we throttle to a lightweight proximity-only
        scan every ~0.5 s to keep CPU usage low while still providing real-time
        collision clearance for manual teleop.
        """
        from core.sensors import SensorFrame

        s = self.controller.state

        if not self._mission_active:
            # Idle: only run a full scan every 10th tick (~0.5 s) for the
            # dashboard lidar overlay; use the cached frame otherwise.
            self._idle_scan_counter = getattr(self, "_idle_scan_counter", 0) + 1
            if self._idle_scan_counter < 10 and self._sensor_frame is not None:
                return self._sensor_frame
            self._idle_scan_counter = 0

        # With use_estimated_pose the discovery map is integrated separately in
        # step_physics from the ESTIMATED frame (the raycast itself must always
        # happen from the true pose — it is the physical measurement).
        frame = self.navigator.scan(
            s.position,
            s.quaternion,
            update_discovery=(
                self._mission_active and not self.config.use_estimated_pose
            ),
        )
        self._sensor_frame = frame
        return frame

    def run_navigation_phase(self, dt: float, frame: "SensorFrame") -> tuple:
        """Navigation: plan from sensor frame only (no pose teleport)."""
        s = self.controller.state

        if self.mission_state == "LANDING":
            # Descend straight down to floor level; kill horizontal motion.
            floor_z = float(self.scene.bounds.min_corner[2]) + 0.08
            traj = TrajectorySample(
                position=np.array([s.position[0], s.position[1], floor_z], dtype=np.float64),
                velocity=np.array([0.0, 0.0, -0.6], dtype=np.float64),
                yaw=float(frame.yaw),
            )
            repulse = np.zeros(3)
        elif self._mission_active and self.controller.autonomous:
            if self.config.use_estimated_pose and self._last_est is not None:
                # Fly on the drone's OWN estimate: the planner, goal logic, and
                # path following all live in the estimated frame.  The resulting
                # trajectory is offset from truth by the (SLAM-bounded) drift —
                # exactly the regime a real drone operates in.  Velocity feedback
                # stays direct (VIO/optical-flow velocity is high quality).
                traj = self.navigator.plan(
                    self._last_est.position, s.velocity,
                    self._last_est.quaternion, dt, frame,
                )
            else:
                traj = self.navigator.plan(s.position, s.velocity, s.quaternion, dt, frame)
            repulse = self.navigator.repulsion_from_sensors(frame)
        else:
            # IDLE / PAUSED: target the floor so the drone sits on the ground
            # instead of hovering wherever it happens to be.
            floor_z = float(self.scene.bounds.min_corner[2]) + 0.12
            traj = TrajectorySample(
                position=np.array([s.position[0], s.position[1], floor_z], dtype=np.float64),
                velocity=np.zeros(3),
                yaw=float(frame.yaw),
            )
            repulse = np.zeros(3)
        return traj, repulse

    def run_physics_phase(self, traj: TrajectorySample, repulse: np.ndarray) -> None:
        """Physics: RK4 integration with per-substep mesh collision."""
        for _ in range(self.config.control_decimation):
            self.controller.step(
                traj,
                self.config.physics_dt,
                avoidance_accel=repulse,
                collision_solver=self.collision,
            )

    def step_physics(self, dt: float) -> None:
        """One control cycle: sensors → navigation → physics."""
        frame = self.run_sensor_phase()
        # Pose AT SCAN TIME — captured before physics advances the state.  The
        # estimator/SLAM/recon pipeline must de-rotate this frame's hits with
        # this exact pose: using the post-physics pose skews the body frame by
        # yaw_rate*dt (~1°/tick while turning), which the scan matcher then
        # faithfully measures and "corrects" — a runaway that wrecked the
        # estimate (measured: 1.7 m drift in the first minute).
        scan_pos = self.controller.state.position.copy()
        scan_quat = self.controller.state.quaternion.copy()

        # ── State estimation + SLAM BEFORE planning ──────────────────────────
        # The navigation phase must fly on the freshest corrected estimate; on a
        # real drone the estimator runs upstream of the planner, not after it.
        if self._mission_active:
            self._last_est = self.estimator.update(scan_pos, scan_quat, dt)
            if self.config.use_estimated_pose and self.mission_state == "EXPLORING":
                self._update_perceived_slam(frame, scan_pos, scan_quat)
                # The drone's map is built in ITS OWN estimated frame — the map a
                # real drone would navigate by.
                frame_est = self._frame_in_estimated_frame(frame, scan_pos, scan_quat)
                if frame_est is not None:
                    self.navigator.discovery.integrate_scan(
                        frame_est, scan_agl=self.navigator._scan_altitude_agl()
                    )

        traj, repulse = self.run_navigation_phase(dt, frame)
        self.run_physics_phase(traj, repulse)
        s = self.controller.state

        if not np.all(np.isfinite(s.position)) or not np.all(np.isfinite(s.velocity)):
            spawn, _ = self._default_spawn(self.scene)
            spawn = self.collision.push_to_free_space(spawn)
            self.controller.reset(spawn, yaw=0.0)

        if self._mission_active and self.mission_state == "EXPLORING":
            if self._last_pos is not None:
                self._distance += float(np.linalg.norm(s.position - self._last_pos))
            self._last_pos = s.position.copy()
            cell = (int(s.position[0] * 2), int(s.position[1] * 2))
            if cell not in self._mapped_cells:
                self._mapped_cells.add(cell)
            area = self.scene.bounds.extent[0] * self.scene.bounds.extent[1]
            self._coverage = min(
                99.5, 100.0 * len(self._mapped_cells) * 0.35 / max(area, 1.0)
            )
            # Fuse this frame's labelled hits into the persistent reconstruction
            # cloud here — once per physics tick — rather than in get_telemetry.
            # The old placement sampled only the latest frame at the 20 Hz
            # telemetry poll, so under god-mode time acceleration (multiple
            # ticks per poll) most scanned frames were dropped from the cloud.
            all_hits = frame.hit_points_labeled()   # (N, 4): xyz + semantic_class
            if len(all_hits):
                if self.config.use_estimated_pose:
                    all_hits = self._reproject_through_estimate(
                        all_hits, scan_pos, scan_quat, self._last_est)
                self._refine_semantics(all_hits)
                self._accumulate_recon(all_hits)
            self._maybe_capture_snapshot(frame)

    def _maybe_capture_snapshot(self, frame) -> None:
        """Trigger mock camera snapshot on new exploration sectors.

        Rate-limited to at most one capture per 4 seconds of real-time (and only
        when meaningful new coverage has been accumulated) so that the expensive
        raycast rendering inside capture_drone_view does not dominate the tick budget.
        """
        if not self._mission_active:
            return

        # Hard wall-clock rate limit — never capture more often than every 4 s
        now = time.perf_counter()
        if not hasattr(self, "_last_snapshot_time"):
            self._last_snapshot_time = 0.0
        if now - self._last_snapshot_time < 4.0:
            return

        stats = self.navigator.discovery.coverage_stats()
        known = float(stats.get("known_percent", 0.0))
        coverage_delta = self._coverage - self._last_snapshot_coverage
        known_delta = known - self._last_snapshot_known_pct
        new_structures = len(frame.detected_structures) > 0 and known_delta >= 0.15

        if known_delta < 0.18 and coverage_delta < 0.12 and not new_structures:
            return

        self._last_snapshot_time = now

        pos = self.controller.state.position
        yaw = float(frame.yaw)
        image_b64 = capture_drone_view(
            self.scene,          # full RedwoodScene — raycast + semantics
            pos,
            yaw,
            quaternion=np.asarray(self.controller.state.quaternion, dtype=np.float64),
        )
        stamp = {
            "id": len(self._camera_gallery) + 1,
            "timestamp_s": round(self._elapsed, 2),
            "position": [round(float(pos[0]), 2), round(float(pos[1]), 2), round(float(pos[2]), 2)],
            "yaw": round(float(yaw), 4),
            "coverage_pct": round(self._coverage, 1),
            "known_pct": round(known, 1),
            "descriptor": self._build_snapshot_descriptor(frame, known),
            "image_base64": image_b64 or "",
        }
        self._camera_gallery.append(stamp)
        if len(self._camera_gallery) > 300:
            self._camera_gallery = self._camera_gallery[-300:]
        self._last_snapshot_known_pct = known
        self._last_snapshot_coverage = self._coverage

    @staticmethod
    def _voxel_subsample(
        points: np.ndarray, voxel_size: float = 0.08, key_dims: int = 3
    ) -> np.ndarray:
        """
        Keep at most one point per voxel cell (first-hit occupancy).

        This limits point density to ~156 pts/m³ at 8 cm voxels, preventing
        objects from accumulating thousands of redundant hits while walls still
        get representative coverage.  Much faster than random subsampling and
        preserves geometric structure.

        ``key_dims`` controls how many leading columns are used to compute the
        voxel key — pass 3 when the array has extra columns beyond XYZ (e.g.
        a semantic-class label column) so those columns are preserved but not
        used for deduplication.
        """
        if len(points) == 0:
            return points
        keys = np.floor(points[:, :key_dims] / voxel_size).astype(np.int32)
        # Unique voxels — preserve first-encountered point per cell
        _, first_idx = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(first_idx)]

    def _refine_semantics(self, hits: np.ndarray) -> None:
        """Re-label hits in place using room geometry, not just the local normal.

        The sensor's normal-only class can't separate a wall from a wardrobe side
        (both are vertical planes) — measured ~80% of all points came back WALL.
        Here we use two scene-level signals a real drone genuinely accumulates:

          • absolute height vs the scene floor/ceiling — a horizontal surface near
            Z=floor is FLOOR, near the ceiling is CEILING, in between is a
            furniture top (OBJECT), regardless of which side the sensor guessed;
          • the vertical EXTENT of the occupied column at the hit's (x,y) — a wall
            spans (nearly) floor-to-ceiling, furniture does not.  The wide pitch
            rings make a single wall scan fill most of its column at once, so this
            is reliable even early in a mission.

        Sensor class codes: 2=WALL 3=OBJECT 4=FLOOR 5=CEILING (core.semantic_space).
        """
        floor_z = float(self.scene.bounds.min_corner[2])
        ceil_z = float(self.scene.bounds.max_corner[2])
        room_h = max(ceil_z - floor_z, 0.5)
        inv = 1.0 / self._col_voxel
        cols = self._col_extent

        # 1. Update per-column vertical extent from this frame's hits.
        for h in hits:
            key = (int(h[0] * inv), int(h[1] * inv))
            z = float(h[2])
            ext = cols.get(key)
            if ext is None:
                cols[key] = [z, z]
            else:
                if z < ext[0]:
                    ext[0] = z
                if z > ext[1]:
                    ext[1] = z

        # 2. Re-label.  Thresholds are fractions of room height so this works for
        #    any ceiling height.
        floor_band = floor_z + max(0.30, 0.12 * room_h)
        ceil_band = ceil_z - max(0.35, 0.14 * room_h)
        tall_top = ceil_z - max(0.45, 0.18 * room_h)   # column top must reach here
        tall_span = 0.45 * room_h                       # and span at least this
        for h in hits:
            z = float(h[2])
            sem = int(h[3])
            horizontal = sem in (4, 5)
            if horizontal:
                if z <= floor_band:
                    h[3] = 4          # FLOOR
                elif z >= ceil_band:
                    h[3] = 5          # CEILING
                else:
                    h[3] = 3          # mid-height horizontal = furniture top
                continue
            # Vertical / sloped surface → WALL vs OBJECT by column extent.
            key = (int(h[0] * inv), int(h[1] * inv))
            ext = cols.get(key)
            if ext is not None and ext[1] >= tall_top and (ext[1] - ext[0]) >= tall_span:
                h[3] = 2              # WALL
            else:
                h[3] = 3              # OBJECT (furniture, low fixtures)

    def _accumulate_recon(self, hits: np.ndarray) -> None:
        """Fuse labelled hits (N×4 [x,y,z,label]) into the deduplicated cloud."""
        inv = 1.0 / self._recon_voxel
        pts = self._recon_points
        idx = self._recon_index
        for h in hits:
            x = float(h[0]); y = float(h[1]); z = float(h[2])
            key = (int(round(x * inv)), int(round(y * inv)), int(round(z * inv)))
            if key in idx:
                continue
            idx[key] = len(pts)
            pts.append([round(x, 3), round(y, 3), round(z, 3), int(h[3])])

    def export_scan_deliverables(self) -> Dict[str, Any]:
        """Generate the scan deliverables (PLY cloud, GLB mesh, SVG floor plan)
        into ``dashboard/public/exports`` so the dashboard can serve them.

        CPU-heavy (Poisson reconstruction) — the bridge runs this in a worker
        thread, never on the physics loop.
        """
        from pathlib import Path
        from core.exporters import export_scan

        out_dir = Path(__file__).resolve().parents[2] / "dashboard" / "public" / "exports"
        manifest = export_scan(
            list(self._recon_points), self.navigator.discovery, out_dir,
            scene_id=self.scene_id,
        )
        manifest["urls"] = [f"/exports/{f['file']}" for f in manifest["files"]]
        return manifest

    def _reproject_through_estimate(self, hits: np.ndarray,
                                    true_pos: np.ndarray, true_quat: np.ndarray,
                                    est) -> np.ndarray:
        """Re-place world-frame hits as a real drone would: recover each return in
        the sensor body frame (what the hardware physically measures — range +
        bearing), then project it back to the world through the *estimated* pose.

        Under perfect localization this is the identity; under drift it ghosts the
        cloud by exactly the pose error, which is the real-world reconstruction
        failure this seam exists to expose.
        """
        from core.math3d import quat_to_rotation_matrix
        R_true = quat_to_rotation_matrix(true_quat)
        R_est = quat_to_rotation_matrix(est.quaternion)
        t_true = np.asarray(true_pos, dtype=np.float64)
        t_est = np.asarray(est.position, dtype=np.float64)
        xyz = np.asarray(hits[:, :3], dtype=np.float64)
        body = (xyz - t_true) @ R_true            # world→body (R_true^T applied on the right)
        world_est = body @ R_est.T + t_est        # body→world via estimated pose
        out = hits.copy()
        out[:, :3] = world_est
        return out

    def _frame_in_estimated_frame(self, frame, scan_pos: np.ndarray,
                                  scan_quat: np.ndarray):
        """Express a sensor frame in the drone's OWN estimated frame.

        The raycast happened from the true pose (physics); a real drone knows its
        returns only relative to its body, placed in the world via its estimate.
        Hits are de-rotated into the gravity-aligned body frame with the true
        yaw (roll/pitch/yaw-rate are IMU-observable) and re-projected through the
        estimated position + yaw.  Returns None when the frame has no returns.
        """
        from dataclasses import replace as dc_replace
        from core.math3d import quat_to_euler
        est = self._last_est
        if est is None or not len(frame.returns):
            return None
        yaw_true = float(quat_to_euler(scan_quat)[2])
        yaw_est = float(quat_to_euler(est.quaternion)[2])
        t_true = np.asarray(scan_pos, dtype=np.float64)
        t_est = np.asarray(est.position, dtype=np.float64)
        ct, st = np.cos(yaw_true), np.sin(yaw_true)
        ce, se = np.cos(yaw_est), np.sin(yaw_est)
        pts = np.asarray([r.hit_point for r in frame.returns], dtype=np.float64)
        rel = pts - t_true
        bx = rel[:, 0] * ct + rel[:, 1] * st
        by = -rel[:, 0] * st + rel[:, 1] * ct
        wx = t_est[0] + bx * ce - by * se
        wy = t_est[1] + bx * se + by * ce
        wz = pts[:, 2] - t_true[2] + t_est[2]
        returns_p = [
            dc_replace(r, hit_point=np.array([wx[i], wy[i], wz[i]]))
            for i, r in enumerate(frame.returns)
        ]
        return dc_replace(frame, position=t_est.copy(), yaw=yaw_est,
                          returns=returns_p)

    def _update_perceived_slam(self, frame, scan_pos: np.ndarray,
                               scan_quat: np.ndarray) -> None:
        """Re-localization on the ESTIMATED pose (the real-drone path).

        Hector/Cartographer-style ordering — MATCH FIRST, THEN INSERT:
        1. De-rotate this frame's obstacle endpoints into a gravity-aligned body
           frame (roll/pitch are IMU-observable; yaw + position drift).
        2. Align the scan against the high-resolution match grid (correlative
           matcher) and feed the offset into ``estimator.correct``.
        3. Only THEN insert the endpoints into the grid, at the CORRECTED pose.

        Three measured failure modes shaped this design:
        - Insert-at-raw-estimate + 1 Hz matching smeared drift into the map and
          corrections made drift WORSE (0.49 m vs 0.33 m at 5 min).
        - Matching against the 0.2 m navigation grid put the likelihood ridge at
          cell centres (±0.1 m off the true surface, differently per wall) — a
          wandering systematic bias.  Hence the dedicated 0.05 m MatchGrid.
        - Per-tick insertion at 5 Hz diverged at a CONSTANT rate (−0.7°/match,
          1.7 m drift in a minute): matching against cells inserted 0.2 s ago
          while feeding corrections back into the next insert leaves the common
          (estimate, map) rotation unconstrained — a gauge mode that noise kicks
          into a self-sustaining limit cycle (the matcher chases the map, the
          map follows the matcher).  Hence KEYFRAME insertion: the grid accepts
          a scan only after >0.25 m translation or >15° yaw since the last
          insert, so between keyframes the reference is FROZEN and corrections
          anchor to it instead of chasing it.
        Matching runs every 4th tick (5 Hz) — a realistic lidar/mapper rate.
        """
        from dataclasses import replace as dc_replace
        from core.math3d import quat_to_euler, quat_from_euler
        est = self._last_est
        if self._match_grid is None or est is None or not len(frame.returns):
            return

        self._match_countdown -= 1
        if self._match_countdown > 0:
            return
        self._match_countdown = 4    # 5 Hz at the 20 Hz control rate

        # Body-frame obstacle endpoints (walls/objects only — stable geometry).
        obs = frame.hit_points_obstacles()
        if len(obs) < 8:
            return
        yaw_true = float(quat_to_euler(scan_quat)[2])
        roll_e, pitch_e, yaw_est = (float(v) for v in quat_to_euler(est.quaternion))
        t_true = np.asarray(scan_pos, dtype=np.float64)
        t_est = np.asarray(est.position, dtype=np.float64)
        ct, st = np.cos(yaw_true), np.sin(yaw_true)
        rel = obs[:, :2] - t_true[:2]
        bx = rel[:, 0] * ct + rel[:, 1] * st          # leveled body frame
        by = -rel[:, 0] * st + rel[:, 1] * ct
        body_xy = np.column_stack([bx, by])

        # ── 1. Match against the grid BEFORE inserting anything ─────────────
        if len(body_xy) >= self.scan_matcher.cfg.min_points:
            result = self.scan_matcher.match(
                body_xy, t_est[:2], yaw_est, self._match_grid
            )
            self._last_match_score = result.score
            # Localization health (EMA of match score) → cruise-speed governor.
            # Only meaningful once the map has substance; an empty/immature grid
            # scores 0 without implying the *estimate* is bad.
            if self._match_grid.occupied().sum() >= 200:
                self._slam_health = 0.85 * self._slam_health + 0.15 * result.score
                self._apply_cruise_speed()
            if result.accepted and (result.dx or result.dy or result.dyaw):
                self.estimator.correct(
                    np.array([result.dx, result.dy, 0.0]), result.dyaw
                )
                self._slam_corrections += 1
                # Corrected pose for THIS tick's insert + recon reprojection
                t_est = t_est + np.array([result.dx, result.dy, 0.0])
                yaw_est = yaw_est + result.dyaw
                self._last_est = dc_replace(
                    est,
                    position=t_est,
                    quaternion=quat_from_euler(roll_e, pitch_e, yaw_est),
                    pos_error_m=float(np.linalg.norm(t_est - t_true)),
                    yaw_error_deg=float(np.degrees(
                        abs((yaw_est - yaw_true + np.pi) % (2 * np.pi) - np.pi))),
                )

        # ── 2. KEYFRAME insert at the (corrected) estimated pose ────────────
        is_kf = (
            self._kf_pos is None
            or float(np.linalg.norm(t_est[:2] - self._kf_pos)) > 0.25
            or abs((yaw_est - self._kf_yaw + np.pi) % (2 * np.pi) - np.pi)
            > np.deg2rad(15.0)
        )
        if is_kf:
            ce, se = np.cos(yaw_est), np.sin(yaw_est)
            wx = t_est[0] + bx * ce - by * se
            wy = t_est[1] + bx * se + by * ce
            self._match_grid.insert(np.column_stack([wx, wy]))
            self._kf_pos = t_est[:2].copy()
            self._kf_yaw = yaw_est

    @staticmethod
    def _build_snapshot_descriptor(frame, known_pct: float) -> str:
        walls = sum(1 for r in frame.returns if r.label == "wall")
        objs = sum(1 for r in frame.returns if r.label == "object")
        return (
            f"sector_scan|known={known_pct:.1f}%|walls={walls}|objects={objs}|"
            f"structures={len(frame.detected_structures)}"
        )

    def mission_command(self, command: str) -> None:
        cmd = command.lower().strip()
        if cmd in ("start", "start_mission"):
            self.mission_state = "EXPLORING"
            self._mission_active = True
            self.controller.autonomous = True
            self._elapsed = 0.0
            self._mapped_cells.clear()
            self._coverage = 0.0
            self._last_pos = self.controller.state.position.copy()
            self.navigator.clear_spatial_memory()
            self.navigator.discovery.reset()   # coverage always starts from 0%
            self.estimator.reset(self.controller.state.position)
            if self.config.use_estimated_pose:
                from core.scan_matching import MatchGrid
                b = self.scene.bounds
                # Pad the grid one search-window beyond the scene so drifted
                # projections near the boundary still land in-grid.
                pad = 1.0
                self._match_grid = MatchGrid(
                    origin_xy=b.min_corner[:2] - pad,
                    extent_xy=(b.max_corner[:2] - b.min_corner[:2]) + 2 * pad,
                )
                self._match_countdown = 0
                self._slam_corrections = 0
                self._last_match_score = 0.0
                self._slam_health = 1.0
                self._kf_pos = None
                self._kf_yaw = 0.0
                self._apply_cruise_speed()
            self._recon_points.clear()
            self._recon_index.clear()
            self._col_extent.clear()
            self._camera_gallery.clear()
            self._last_snapshot_known_pct = 0.0
            self._last_snapshot_coverage = 0.0
        elif cmd in ("pause", "pause_mission"):
            self.mission_state = "PAUSED"
            self.controller.autonomous = False
        elif cmd in ("resume", "resume_mission"):
            self.mission_state = "EXPLORING"
            self.controller.autonomous = True
        elif cmd in ("abort", "abort_mission"):
            self.mission_state = "RETURNING"
            self.controller.autonomous = True
        elif cmd == "land":
            self.mission_state = "LANDING"
            self._mission_active = False
            # autonomous=True so run_navigation_phase can feed the descent trajectory
            self.controller.autonomous = True
        elif cmd == "idle":
            self.mission_state = "IDLE"
            self._mission_active = False
            self.controller.autonomous = True   # floor-target traj in nav phase
            self.navigator.clear_spatial_memory()

    def tick(self, dt: float) -> None:
        """Advance simulation by one control step (headless dashboard loop)."""
        if self._mission_active and self.mission_state == "EXPLORING":
            self._elapsed += dt
        self.step_physics(dt)
        # Auto-complete landing: once the drone is within 0.12 m of the floor → IDLE
        if self.mission_state == "LANDING":
            s = self.controller.state
            floor_z = float(self.scene.bounds.min_corner[2]) + 0.12
            if abs(s.position[2] - floor_z) < 0.15 and abs(s.velocity[2]) < 0.30:
                self.mission_state = "IDLE"
                # autonomous stays True → nav phase keeps it on the floor

    def get_telemetry(self) -> Dict[str, Any]:
        s = self.controller.state
        frame = self._sensor_frame
        if frame is None:
            frame = self.navigator.scan(s.position, s.quaternion)

        # Lidar overlay (walls + objects only) — used for the green ray lines
        hits = frame.hit_points_obstacles()
        lidar_pts = hits.tolist() if len(hits) else []

        # NOTE: the reconstruction cloud is now accumulated in step_physics (once
        # per physics tick) so it captures every scanned frame regardless of the
        # telemetry poll rate.  The cloud is voxel-deduplicated and the bridge
        # streams only the NEW points since its last broadcast, so the client
        # cloud grows monotonically and never drops earlier surfaces.

        extra = self.navigator.telemetry_extra()
        sensor_telemetry = self.navigator.sensors.to_telemetry_dict(
            frame, discovery=extra.get("discovery")
        )
        patrol_path = self.navigator.get_intent_path()

        return {
            "type": "state",
            "scene": self.scene_id,
            "position": s.position.tolist(),
            "quaternion": s.quaternion.tolist(),
            "velocity": s.velocity.tolist(),
            "mission_state": self.mission_state,
            "coverage": round(
                extra.get("discovery", {}).get("known_percent", self._coverage), 2
            ),
            "elapsed_time": round(self._elapsed, 2),
            "distance_traveled": round(self._distance, 2),
            "armed": self._mission_active,
            "autonomous": self.controller.autonomous,
            "god_mode": self._god_mode,
            "localization": (
                {
                    "mode": "estimated" if self.config.use_estimated_pose else "ground_truth",
                    "pos_drift_m": round(self._last_est.pos_error_m, 3),
                    "yaw_drift_deg": round(self._last_est.yaw_error_deg, 2),
                    "slam_corrections": self._slam_corrections,
                    "slam_match_score": round(self._last_match_score, 2),
                    "slam_health": round(self._slam_health, 2),
                }
                if self._last_est is not None
                else {"mode": "ground_truth", "pos_drift_m": 0.0, "yaw_drift_deg": 0.0,
                      "slam_corrections": 0, "slam_match_score": 0.0}
            ),
            "navigation_mode": "semantic_discovery",
            "space_analysis": extra.get("space_analysis", {}),
            "discovery": extra.get("discovery", {}),
            "discovered_map": extra.get("discovered_map", []),
            "detected_structures": frame.detected_structures,
            "total_points": len(self._recon_points),
            "area_mapped": round(len(self._mapped_cells) * 0.35, 2),
            "patrol_path": patrol_path,
            "active_target": patrol_path[-1] if patrol_path else s.position.tolist(),
            "lidar": lidar_pts,
            # Full reconstruction cloud; the bridge replaces this with the delta
            # (new points since its last broadcast) for streaming efficiency, and
            # sends the full list to a freshly-connected client.
            "map_points": self._recon_points,
            "sensors": sensor_telemetry,
            "clearance_m": sensor_telemetry.get("nearest_obstacle_m", 0),
            "nav_state": extra.get("nav_state", "EXPLORING"),
            "coordinate_frame": "global_ros_z_up",
            "scene_bounds": {
                "min": self.scene.bounds.min_corner.tolist(),
                "max": self.scene.bounds.max_corner.tolist(),
                "center": self.scene.bounds.center.tolist(),
                "extent": self.scene.bounds.extent.tolist(),
            },
            "visual_mesh_url": getattr(self.scene, "visual_mesh_url", None),
            "camera_snapshot": (
                self._camera_gallery[-1] if self._camera_gallery else None
            ),
            "camera_gallery": list(self._camera_gallery),
        }

    def run(self) -> None:
        if self.renderer is None:
            raise RuntimeError("Renderer not initialized (headless mode)")

        self.controller.autonomous = True
        self.mission_state = "EXPLORING"
        self._mission_active = True
        print("Sensor-driven patrol (mesh LiDAR navigation).")
        print("Keys: A=toggle auto/manual, SPACE=recover, ESC=quit")
        print("Tip: use the unified dashboard instead: ./run-aetherscan.sh --dashboard")
        last = time.perf_counter()

        while self._running and self.renderer.poll():
            now = time.perf_counter()
            frame_dt = min(now - last, 0.05)
            last = now
            self._accumulator += frame_dt

            steps = 0
            while self._accumulator >= self.config.control_dt and steps < 10:
                self.step_physics(self.config.control_dt)
                self._accumulator -= self.config.control_dt
                steps += 1

            frame = self._sensor_frame
            patrol = (
                np.asarray(self.navigator.get_intent_path())
                if self.navigator.get_intent_path()
                else None
            )
            active = (
                np.asarray(patrol[-1])
                if patrol is not None and len(patrol)
                else self.controller.state.position
            )
            self.renderer.update(
                self.controller.state,
                patrol_waypoints=patrol,
                active_target=active,
            )
            self.renderer.render()

        if self.renderer is not None:
            self.renderer.close()
