"""
Drone discovery map — spatial knowledge built only from sensor measurements.

Starts empty (unknown). Each LiDAR scan updates free space, walls, and objects.
Navigation targets frontiers between known-free and unknown regions.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from core.semantic_space import SEMANTIC_NAMES, SemanticClass
from core.sensors import LidarReturn, SensorFrame


@dataclass
class DiscoveryConfig:
    resolution: float = 0.2
    max_range: float = 8.0
    low_object_band_m: float = 0.55
    high_scan_agl_m: float = 1.05
    # Obstacle inflation radius for planning.  Cells within this distance of an
    # occupied cell are treated as impassable so the planned path keeps the
    # drone's body (radius ≈ 0.32 m) clear of geometry — without it the BFS
    # routed the drone into cells one grid-step from a wall and it wedged.
    inflation_radius_m: float = 0.20
    # ── Log-odds evidence model (probabilistic occupancy) ─────────────────────
    # Real sensors are noisy: a single dropped/perturbed return must NOT flip a
    # cell's state (last-write-wins occupancy churned UNKNOWN↔FREE near walls,
    # manufacturing endless fake frontiers that trapped exploration in one zone).
    # Evidence accumulates per cell; classification only changes after the margin
    # is crossed.  At ~100 Hz scan rate a flip needs sustained contrary evidence.
    lo_occ_hit: int = 4        # wall/object hit
    lo_floor_hit: int = -2     # floor hit = open at flight altitude (2D grid!)
    lo_ray_free: int = -1      # ray passed through the cell
    lo_clamp: int = 20
    occ_threshold: int = 4     # logodds >= this  → occupied
    free_threshold: int = -2   # logodds <= this  → free
    # Anti-stuck blocked cells expire after this many scans (~12 s at 100 Hz) so
    # exploration hints stop permanently corrupting the map.
    blocked_expiry_scans: int = 1200


class DiscoveryMap:
    """2D semantic occupancy discovered by the drone."""

    def __init__(
        self,
        origin_xy: np.ndarray,
        grid_shape: Tuple[int, int],
        config: DiscoveryConfig | None = None,
    ):
        self.origin_xy = np.asarray(origin_xy, dtype=np.float64)
        self.grid_shape = grid_shape
        self.cfg = config or DiscoveryConfig()
        self.grid = np.full(grid_shape, SemanticClass.UNKNOWN, dtype=np.uint8)
        # Authoritative occupancy evidence (see DiscoveryConfig log-odds params).
        # self.grid stays as the SEMANTIC DISPLAY layer (telemetry, coverage
        # weighting); passability and frontiers read logodds only.
        self.logodds = np.zeros(grid_shape, dtype=np.int16)
        self.visit_count = np.zeros(grid_shape, dtype=np.uint16)
        self._total_scans = 0
        # cell -> scan index at which the block expires (anti-stuck, self-healing)
        self._blocked_cells: Dict[Tuple[int, int], int] = {}
        self._scan_agl: float = 1.45
        # Per-semantic hit counts for coverage deficit weighting
        self._sem_hits: Dict[int, int] = {int(c): 0 for c in SemanticClass}
        # Inflated-obstacle mask (rebuilt lazily before each frontier search).
        self._inflated = np.zeros(grid_shape, dtype=bool)
        # "Covered" = free cells the drone has physically flown CLOSE to (within
        # cover_radius).  A scanning drone must visit every room, not just glimpse
        # it from a doorway — free cells the lidar saw but the drone never
        # approached remain exploration targets so the whole space gets scanned
        # at close range.
        self._covered = np.zeros(grid_shape, dtype=bool)
        self._cover_radius_cells = 6   # ≈1.2 m close-scan radius

    def _rebuild_inflation(self) -> None:
        """Recompute the inflated-obstacle mask: True where a cell is within
        ``inflation_radius_m`` of any OCCUPIED cell (log-odds evidence).  Cheap
        iterative 4-neighbour dilation (a handful of passes on a small grid)."""
        occ = self.logodds >= self.cfg.occ_threshold
        steps = max(0, int(round(self.cfg.inflation_radius_m / self.cfg.resolution)))
        infl = occ.copy()
        for _ in range(steps):
            nxt = infl.copy()
            nxt[1:, :] |= infl[:-1, :]
            nxt[:-1, :] |= infl[1:, :]
            nxt[:, 1:] |= infl[:, :-1]
            nxt[:, :-1] |= infl[:, 1:]
            infl = nxt
        self._inflated = infl

    def _rebuild_covered(self) -> None:
        """Mask of free cells the drone has physically flown close to.  Dilate the
        visited cells (visit_count>0) by cover_radius so 'covered' means scanned at
        close range, not merely glimpsed by long-range lidar from another room."""
        cov = self.visit_count > 0
        for _ in range(self._cover_radius_cells):
            nxt = cov.copy()
            nxt[1:, :] |= cov[:-1, :]
            nxt[:-1, :] |= cov[1:, :]
            nxt[:, 1:] |= cov[:, :-1]
            nxt[:, :-1] |= cov[:, 1:]
            cov = nxt
        self._covered = cov

    def reset(self) -> None:
        """Wipe all discovered cells back to UNKNOWN (call at mission start)."""
        self.grid[:] = SemanticClass.UNKNOWN
        self.logodds[:] = 0
        self.visit_count[:] = 0
        self._total_scans = 0
        self._blocked_cells.clear()
        self._sem_hits = {int(c): 0 for c in SemanticClass}

    # ── Log-odds state predicates (authoritative occupancy) ───────────────────
    def _occ(self, i: int, j: int) -> bool:
        return int(self.logodds[i, j]) >= self.cfg.occ_threshold

    def _free_lo(self, i: int, j: int) -> bool:
        return int(self.logodds[i, j]) <= self.cfg.free_threshold

    def _unk_lo(self, i: int, j: int) -> bool:
        lo = int(self.logodds[i, j])
        return self.cfg.free_threshold < lo < self.cfg.occ_threshold

    def cell_known(self, i: int, j: int) -> bool:
        """True once evidence has classified the cell (free OR occupied)."""
        return self.in_bounds(i, j) and not self._unk_lo(i, j)

    def _add_evidence(self, i: int, j: int, delta: int) -> None:
        c = self.cfg.lo_clamp
        self.logodds[i, j] = max(-c, min(c, int(self.logodds[i, j]) + delta))

    def _blocked_active(self, i: int, j: int) -> bool:
        exp = self._blocked_cells.get((i, j))
        if exp is None:
            return False
        if self._total_scans >= exp:
            del self._blocked_cells[(i, j)]
            return False
        return True

    @classmethod
    def from_analyzed_space(cls, analyzed) -> DiscoveryMap:
        return cls(
            analyzed.origin_xy,
            analyzed.grid_shape,
            DiscoveryConfig(resolution=analyzed.config.grid_resolution),
        )

    def _ij(self, x: float, y: float) -> Tuple[int, int]:
        r = self.cfg.resolution
        i = int((x - self.origin_xy[0]) / r)
        j = int((y - self.origin_xy[1]) / r)
        return i, j

    def _xy_center(self, i: int, j: int) -> Tuple[float, float]:
        r = self.cfg.resolution
        return (
            float(self.origin_xy[0] + (i + 0.5) * r),
            float(self.origin_xy[1] + (j + 0.5) * r),
        )

    def _passable(self, i: int, j: int) -> bool:
        if not self.in_bounds(i, j):
            return False
        if self._blocked_active(i, j):
            return False
        if self._inflated[i, j]:
            return False  # too close to geometry for the drone body to fit
        return self._free_lo(i, j)

    def _unknown_corridor_passable(self, i: int, j: int) -> bool:
        """
        Tentatively allow UNKNOWN cells that border known FREE space.

        Lets BFS scrape along unexplored corridors when tight geometry or
        misclassified objects would otherwise seal a route.
        """
        if not self.in_bounds(i, j) or self._blocked_active(i, j):
            return False
        if not self._unk_lo(i, j):
            return False
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if self._passable(ni, nj):
                return True
        return False

    def _bfs_step_passable(self, i: int, j: int) -> bool:
        return self._passable(i, j) or self._unknown_corridor_passable(i, j)

    def mark_blocked_disk(self, x: float, y: float, radius_m: float = 0.45) -> int:
        """Mark grid cells in a disk as impassable for frontier BFS (anti-stuck memory)."""
        i0, j0 = self._ij(x, y)
        r_cells = int(radius_m / self.cfg.resolution) + 1
        marked = 0
        for di in range(-r_cells, r_cells + 1):
            for dj in range(-r_cells, r_cells + 1):
                if di * di + dj * dj > r_cells * r_cells:
                    continue
                i, j = i0 + di, j0 + dj
                if self.in_bounds(i, j):
                    # Blocks self-heal after blocked_expiry_scans so anti-stuck
                    # hints can't permanently seal off parts of the map.
                    self._blocked_cells[(i, j)] = self._total_scans + self.cfg.blocked_expiry_scans
                    marked += 1
        return marked

    def clear_blocked_cells(self) -> None:
        self._blocked_cells.clear()

    def _is_frontier_cell(self, i: int, j: int) -> bool:
        """Evidence-UNKNOWN cell bordering at least one evidence-FREE cell."""
        if not self.in_bounds(i, j):
            return False
        if not self._unk_lo(i, j):
            return False
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if self._passable(ni, nj):
                return True
        return False

    def _adjacent_to_frontier(self, i: int, j: int) -> bool:
        """FREE cell next to an UNKNOWN frontier cell."""
        if not self._passable(i, j):
            return False
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if self._is_frontier_cell(i + di, j + dj):
                return True
        return False

    def find_path_to_frontier(
        self, start_world: np.ndarray
    ) -> Optional[List[Tuple[int, int]]]:
        """
        BFS on the discovery grid to the best frontier-adjacent cell.

        "Best" = highest (semantic_deficit_multiplier / distance) score among
        the first MAX_CANDIDATES frontier-adjacent cells found.  This biases
        routing toward under-scanned surface types (walls, floors) instead of
        always taking the nearest frontier which tends to be object clusters.

        Routes through FREE cells and, when needed, tentatively through UNKNOWN
        cells adjacent to FREE (corridor scraping past misclassified seals).
        """

        # Refresh the inflated-obstacle mask so paths keep the body clear of walls.
        self._rebuild_inflation()

        start = np.asarray(start_world, dtype=np.float64)
        si, sj = self._ij(float(start[0]), float(start[1]))
        if not self.in_bounds(si, sj):
            return None

        # If the drone sits inside the inflated zone (e.g. it drifted near a wall),
        # BFS outward through any in-bounds cell to the nearest genuinely passable
        # cell and start there — this is the escape route back into open space.
        if not self._bfs_step_passable(si, sj):
            si, sj = self._nearest_passable(si, sj, max_radius=12) or (si, sj)
            if not self._bfs_step_passable(si, sj):
                return None

        if self._adjacent_to_frontier(si, sj):
            return [(si, sj)]

        deficit_weights = self.semantic_deficit_weights()
        q: deque[Tuple[int, int]] = deque([(si, sj)])
        parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {(si, sj): None}
        depth: Dict[Tuple[int, int], int] = {(si, sj): 0}
        neighbors = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, 1), (1, -1), (-1, -1),
        )

        self._rebuild_covered()
        # Candidates are reachable vantage points scored by how much UN-SCANNED
        # area they reveal in an 11×11 window (~1.1 m): UNKNOWN cells (discover new
        # space) PLUS free cells the drone hasn't physically covered yet (visit a
        # room the lidar only glimpsed).  Unifying discovery + coverage is what
        # pulls the drone through doorways into every room — pure frontier
        # exploration declared a room "done" the moment the lidar saw it from afar,
        # so the drone never flew in and stayed boxed in one quadrant for minutes.
        R = 5
        maxw = (2 * R + 1) ** 2
        candidates: List[Tuple[float, int, Tuple[int, int], int]] = []

        while q:
            i, j = q.popleft()
            d = depth[(i, j)]

            if self._free_lo(i, j):  # only score vantage points the drone can sit at
                value = 0
                for di in range(-R, R + 1):
                    for dj in range(-R, R + 1):
                        ni, nj = i + di, j + dj
                        if not self.in_bounds(ni, nj):
                            continue
                        if self._unk_lo(ni, nj) or (self._free_lo(ni, nj) and not self._covered[ni, nj]):
                            value += 1
                if value > 0:
                    sem_boost = self._frontier_semantic_multiplier(i, j, deficit_weights)
                    score = sem_boost * (0.3 + 2.5 * value / maxw) / (1.0 + 0.01 * d)
                    candidates.append((-score, d, (i, j), value))

            for di, dj in neighbors:
                ni, nj = i + di, j + dj
                if (ni, nj) in parent:
                    continue
                if not self._bfs_step_passable(ni, nj):
                    continue
                parent[(ni, nj)] = (i, j)
                depth[(ni, nj)] = d + 1
                q.append((ni, nj))

        if not candidates:
            return None

        # Prefer SUBSTANTIAL targets (≥14 un-scanned cells in the window); tiny
        # remnants are cleaned up only when nothing better remains.
        substantial = [c for c in candidates if c[3] >= 14]
        pool = substantial if substantial else candidates
        pool.sort()
        best_cell = self._tour_first_stop(pool, (si, sj)) or pool[0][2]

        path: List[Tuple[int, int]] = []
        cur: Optional[Tuple[int, int]] = best_cell
        while cur is not None:
            path.append(cur)
            cur = parent.get(cur)
        path.reverse()
        return path

    def _tour_first_stop(
        self,
        pool: List[Tuple[float, int, Tuple[int, int], int]],
        start_ij: Tuple[int, int],
    ) -> Optional[Tuple[int, int]]:
        """Pick the goal as the FIRST STOP of a global coverage tour.

        Greedy "best vantage now" exploration ignores global route optimality —
        it chases a rich far target and leaves a near pocket behind, then pays
        the corridor twice (the classic backtracking failure FUEL [Zhou et al.,
        2021] identifies; their fix is a coverage tour solved as an ATSP).
        Scaled to our grid: take the top spatially-distinct vantage candidates,
        order them as a shortest open tour from the drone, and commit to the
        tour's first leg.  The tour is re-derived on every goal request, so it
        adapts as the map grows (FUEL's "replan when frontiers change").
        """
        # 1. Spatially-distinct shortlist (best-scored representative per region)
        MIN_SEP = 8          # cells (~1.6 m) between tour stops
        MAX_STOPS = 7        # brute-force tour stays trivial at this size
        stops: List[Tuple[int, int]] = []
        for _neg, _d, cell, _v in pool:
            if all((cell[0] - c[0]) ** 2 + (cell[1] - c[1]) ** 2 >= MIN_SEP ** 2
                   for c in stops):
                stops.append(cell)
                if len(stops) >= MAX_STOPS:
                    break
        if len(stops) <= 1:
            return stops[0] if stops else None

        # 2. Shortest open tour from the drone through all stops (Euclidean leg
        #    costs — optimistic through walls, but adequate for ORDERING).
        import itertools

        def dist(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return float(np.hypot(a[0] - b[0], a[1] - b[1]))

        best_first: Optional[Tuple[int, int]] = None
        best_len = float("inf")
        for perm in itertools.permutations(stops):
            total = dist(start_ij, perm[0])
            for k in range(len(perm) - 1):
                total += dist(perm[k], perm[k + 1])
                if total >= best_len:
                    break
            if total < best_len:
                best_len = total
                best_first = perm[0]
        return best_first

    def _nearest_passable(
        self, si: int, sj: int, max_radius: int = 12
    ) -> Optional[Tuple[int, int]]:
        """BFS outward from (si,sj) through any in-bounds cell to the closest
        passable cell (used to escape an inflated/occupied start cell)."""
        seen = {(si, sj)}
        q: deque[Tuple[int, int]] = deque([(si, sj)])
        while q:
            i, j = q.popleft()
            if abs(i - si) + abs(j - sj) > max_radius:
                continue
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if (ni, nj) in seen or not self.in_bounds(ni, nj):
                    continue
                if self._bfs_step_passable(ni, nj):
                    return (ni, nj)
                seen.add((ni, nj))
                q.append((ni, nj))
        return None

    def find_path_to_cell(
        self, start_world: np.ndarray, goal_ij: Tuple[int, int]
    ) -> Optional[List[Tuple[int, int]]]:
        """BFS path from the drone to a SPECIFIC goal cell over passable cells.
        Used for goal-committed exploration (keep heading to the chosen frontier
        instead of re-picking the nearest one every tick and oscillating)."""
        self._rebuild_inflation()
        start = np.asarray(start_world, dtype=np.float64)
        si, sj = self._ij(float(start[0]), float(start[1]))
        if not self.in_bounds(si, sj) or not self.in_bounds(*goal_ij):
            return None
        if not self._bfs_step_passable(si, sj):
            esc = self._nearest_passable(si, sj, max_radius=12)
            if esc is None:
                return None
            si, sj = esc
        gi, gj = goal_ij
        q: deque[Tuple[int, int]] = deque([(si, sj)])
        parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {(si, sj): None}
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1))
        found = False
        while q:
            i, j = q.popleft()
            if abs(i - gi) <= 1 and abs(j - gj) <= 1:
                gi, gj = i, j
                found = True
                break
            for di, dj in neighbors:
                ni, nj = i + di, j + dj
                if (ni, nj) in parent or not self._bfs_step_passable(ni, nj):
                    continue
                parent[(ni, nj)] = (i, j)
                q.append((ni, nj))
        if not found:
            return None
        path: List[Tuple[int, int]] = []
        cur: Optional[Tuple[int, int]] = (gi, gj)
        while cur is not None:
            path.append(cur)
            cur = parent.get(cur)
        path.reverse()
        return path

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.grid_shape[0] and 0 <= j < self.grid_shape[1]

    def integrate_scan(self, frame: SensorFrame, scan_agl: Optional[float] = None) -> None:
        """Fuse one sensor frame into the discovery map."""
        if scan_agl is not None:
            self._scan_agl = float(scan_agl)
        self._total_scans += 1
        pos = frame.position
        oi, oj = self._ij(pos[0], pos[1])
        if self.in_bounds(oi, oj):
            self.visit_count[oi, oj] = min(65535, int(self.visit_count[oi, oj]) + 1)
            # The drone occupies this cell, so it is certainly traversable.
            self._add_evidence(oi, oj, self.cfg.lo_ray_free * 2)
            self.grid[oi, oj] = SemanticClass.FREE

        for ret in frame.returns:
            self._integrate_return(pos, ret)

    def _clear_ray_to_hit(self, origin: np.ndarray, hit: np.ndarray) -> None:
        """
        Accumulate FREE evidence along the ray up to (but not including) the hit
        cell.  Evidence-based: a single noisy ray nudges the cell, it does NOT
        flip it (the old hard overwrite churned WALL↔FREE near walls and spawned
        endless fake frontiers).
        """
        cells = self._ray_cells(origin[:2], hit[:2])
        for i, j in cells[:-1]:
            if not self.in_bounds(i, j):
                continue
            self._add_evidence(i, j, self.cfg.lo_ray_free)
            # Display layer follows evidence (never overrides a confirmed wall).
            if self._free_lo(i, j) and SemanticClass(int(self.grid[i, j])) in (
                SemanticClass.UNKNOWN,
                SemanticClass.FREE,
            ):
                self.grid[i, j] = SemanticClass.FREE

    def _is_low_furniture_hit(self, origin: np.ndarray, hit: np.ndarray) -> bool:
        """Hit below the current scan plane — chairs/tables at low altitude."""
        return float(hit[2]) < float(origin[2]) - self.cfg.low_object_band_m

    def _integrate_return(self, origin: np.ndarray, ret: LidarReturn) -> None:
        if ret.range_m >= self.cfg.max_range - 0.05:
            return

        hit = ret.hit_point
        sem = getattr(ret, "semantic_class", None)
        if sem is None:
            label = ret.label
            sem_map = {
                "wall": SemanticClass.WALL,
                "object": SemanticClass.OBJECT,
                "floor": SemanticClass.FLOOR,
                "ceiling": SemanticClass.CEILING,
                "free": SemanticClass.FREE,
            }
            sem = sem_map.get(label, SemanticClass.UNKNOWN)
        else:
            sem = SemanticClass(int(sem))

        # Track hits per semantic class for coverage deficit weighting
        self._sem_hits[int(sem)] = self._sem_hits.get(int(sem), 0) + 1

        self._clear_ray_to_hit(origin, hit)

        hi, hj = self._ij(hit[0], hit[1])
        if self.in_bounds(hi, hj):
            if (
                sem == SemanticClass.OBJECT
                and self._is_low_furniture_hit(origin, hit)
                and self._scan_agl >= self.cfg.high_scan_agl_m
            ):
                return
            if sem in (SemanticClass.WALL, SemanticClass.OBJECT):
                # Solid obstacle at flight altitude → occupied evidence.
                self._add_evidence(hi, hj, self.cfg.lo_occ_hit)
                if self._occ(hi, hj):
                    self.grid[hi, hj] = sem
            elif sem == SemanticClass.FLOOR:
                # CRITICAL: this is a 2D occupancy grid at FLIGHT altitude.  A
                # floor hit means the ray reached the GROUND — the column above
                # it is open air, i.e. TRAVERSABLE.  The old code wrote FLOOR
                # into the grid, which `_passable` treated as an obstacle: the
                # downward-pitched lidar rings painted impassable "floor walls"
                # around the drone, confining it to a corridor of its own making.
                self._add_evidence(hi, hj, self.cfg.lo_floor_hit)
                if self._free_lo(hi, hj):
                    self.grid[hi, hj] = SemanticClass.FLOOR  # display/semantics only
            elif sem == SemanticClass.CEILING:
                # Ceiling above → open space below it at flight altitude.
                self._add_evidence(hi, hj, self.cfg.lo_ray_free)
                if self._free_lo(hi, hj) and SemanticClass(int(self.grid[hi, hj])) == SemanticClass.UNKNOWN:
                    self.grid[hi, hj] = SemanticClass.FREE

    def _ray_cells(self, a: np.ndarray, b: np.ndarray) -> List[Tuple[int, int]]:
        steps = int(np.ceil(np.linalg.norm(b - a) / (self.cfg.resolution * 0.5))) + 1
        out: List[Tuple[int, int]] = []
        seen: set[Tuple[int, int]] = set()
        for t in np.linspace(0.0, 1.0, max(steps, 2)):
            p = a + t * (b - a)
            ij = self._ij(float(p[0]), float(p[1]))
            if ij not in seen:
                seen.add(ij)
                out.append(ij)
        return out

    def semantic_deficit_weights(self) -> Dict[int, float]:
        """
        Return a per-semantic-class weight inversely proportional to how well
        that class has been scanned relative to its expected share.

        Expected indoor distribution (empirical, Replica dataset):
          WALL=35%, FLOOR=20%, OBJECT=30%, CEILING=15%

        When a class is scanned below its expected share the weight > 1,
        biasing frontier selection toward areas that produce those hits.
        The weight is clamped to [0.5, 3.0] to avoid extreme steering.
        """
        expected = {
            int(SemanticClass.WALL):    0.35,
            int(SemanticClass.FLOOR):   0.20,
            int(SemanticClass.OBJECT):  0.30,
            int(SemanticClass.CEILING): 0.15,
        }
        structural = [
            int(SemanticClass.WALL),
            int(SemanticClass.FLOOR),
            int(SemanticClass.OBJECT),
            int(SemanticClass.CEILING),
        ]
        total = sum(self._sem_hits.get(c, 0) for c in structural)
        if total < 20:
            return {c: 1.0 for c in structural}  # not enough data yet

        weights: Dict[int, float] = {}
        for cls in structural:
            actual_frac = self._sem_hits.get(cls, 0) / total
            exp_frac = expected[cls]
            # deficit ratio: how much less we've scanned than expected
            deficit = exp_frac / max(actual_frac, 1e-4)
            weights[cls] = float(np.clip(deficit, 0.5, 3.0))
        return weights

    def coverage_stats(self) -> Dict[str, float]:
        total = self.grid.size
        # "Known" is defined by EVIDENCE (log-odds crossed a threshold), not by
        # the display layer — a single noisy ray no longer counts as knowledge.
        known = int(np.sum((self.logodds <= self.cfg.free_threshold)
                           | (self.logodds >= self.cfg.occ_threshold)))
        free = int(np.sum(self.logodds <= self.cfg.free_threshold))
        wall = int(np.sum(self.grid == SemanticClass.WALL))
        obj = int(np.sum(self.grid == SemanticClass.OBJECT))
        unknown = total - known

        # Per-class hit counts and deficit weights for instrumentation
        sem_total = sum(self._sem_hits.get(int(c), 0) for c in (
            SemanticClass.WALL, SemanticClass.FLOOR,
            SemanticClass.OBJECT, SemanticClass.CEILING,
        ))
        def _pct(cls):
            return round(100.0 * self._sem_hits.get(int(cls), 0) / max(sem_total, 1), 1)

        return {
            "known_percent": round(100.0 * known / max(total, 1), 2),
            "unknown_percent": round(100.0 * unknown / max(total, 1), 2),
            "free_percent": round(100.0 * free / max(total, 1), 2),
            "wall_cells": wall,
            "object_cells": obj,
            "scans": self._total_scans,
            # Per-semantic scan hit percentages (instrumentation)
            "hit_pct_wall":    _pct(SemanticClass.WALL),
            "hit_pct_floor":   _pct(SemanticClass.FLOOR),
            "hit_pct_object":  _pct(SemanticClass.OBJECT),
            "hit_pct_ceiling": _pct(SemanticClass.CEILING),
        }

    def _frontier_semantic_multiplier(
        self, i: int, j: int, deficit_weights: Dict[int, float]
    ) -> float:
        """
        Return a boost factor [1.0, 3.0] for a frontier cell based on the
        semantic classes of its known (non-UNKNOWN) neighbours.

        Structural surfaces (wall/floor) that are under-scanned get the highest
        multiplier, drawing the drone toward them.
        """
        best = 1.0
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if not self.in_bounds(ni, nj):
                continue
            cell_class = int(self.grid[ni, nj])
            w = deficit_weights.get(cell_class, 1.0)
            if w > best:
                best = w
        return best

    def frontier_direction(self, position: np.ndarray, yaw: float) -> float:
        """
        Bearing (world rad) toward highest-scoring frontier (unknown adjacent to free).

        Scoring now combines proximity with semantic coverage deficit so the drone
        is biased toward frontiers adjacent to under-scanned surface types
        (typically walls and floors early in a mission).
        """
        deficit_weights = self.semantic_deficit_weights()
        best_score = -1.0
        best_angle = yaw
        px, py = position[0], position[1]
        res = self.cfg.resolution

        for i in range(self.grid_shape[0]):
            for j in range(self.grid_shape[1]):
                # Evidence-based frontier: unknown-by-evidence next to free-by-evidence.
                if not self._unk_lo(i, j):
                    continue
                has_free_neighbor = False
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ni, nj = i + di, j + dj
                    if self.in_bounds(ni, nj) and self._free_lo(ni, nj):
                        has_free_neighbor = True
                        break
                if not has_free_neighbor:
                    continue

                cx = self.origin_xy[0] + (i + 0.5) * res
                cy = self.origin_xy[1] + (j + 0.5) * res
                dx, dy = cx - px, cy - py
                dist = np.hypot(dx, dy)
                if dist < 0.3:
                    continue
                angle = float(np.arctan2(dy, dx))

                # Base proximity score × semantic deficit multiplier
                sem_boost = self._frontier_semantic_multiplier(i, j, deficit_weights)
                score = sem_boost / (dist + 0.5)

                if score > best_score:
                    best_score = score
                    best_angle = angle

        return best_angle

    def local_occupancy_slice(
        self, position: np.ndarray, radius_m: float = 4.0, max_cells: int = 400
    ) -> List[Dict[str, object]]:
        """Sparse map chunk around drone for dashboard."""
        i0, j0 = self._ij(position[0], position[1])
        r_cells = int(radius_m / self.cfg.resolution)
        out: List[Dict[str, object]] = []
        res = self.cfg.resolution
        for di in range(-r_cells, r_cells + 1):
            for dj in range(-r_cells, r_cells + 1):
                i, j = i0 + di, j0 + dj
                if not self.in_bounds(i, j):
                    continue
                c = SemanticClass(int(self.grid[i, j]))
                out.append(
                    {
                        "x": round(self.origin_xy[0] + (i + 0.5) * res, 2),
                        "y": round(self.origin_xy[1] + (j + 0.5) * res, 2),
                        "type": SEMANTIC_NAMES.get(c, "unknown"),
                    }
                )
                if len(out) >= max_cells:
                    return out
        return out
