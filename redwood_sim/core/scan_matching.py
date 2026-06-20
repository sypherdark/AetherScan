"""
Correlative scan-to-map matching — the SLAM correction that bounds pose drift.

Roadmap item 3 of REALWORLD_READINESS.md.  With only odometry (core/state_estimation),
the pose estimate drifts without bound (~0.2 m over 13 m, growing ~sqrt(t)) and the
reconstruction ghosts.  A real drone bounds that drift by *re-localizing against its
own map*: every second or so, take the current LiDAR scan (a body-frame measurement —
the only thing hardware actually gives you), project it through the current pose
estimate, and search a small window of pose perturbations for the one that best
aligns the scan with the occupancy map built so far.  The best offset IS the drift
accumulated since that area was mapped; feeding its negative into the estimator
pins the estimate to the map frame.

This is the classic correlative scan matcher (Olson 2009):
- 2D (x, y, yaw) search — indoor drift is dominated by horizontal + yaw error;
  roll/pitch are gravity-observable and altitude has the rangefinder.
- Likelihood field = Gaussian of the Euclidean distance transform to the nearest
  occupied cell, sampled BILINEARLY at the exact endpoint positions.  Sub-cell
  smoothness is essential: a binary/dilated field on the 0.2 m grid produces
  score plateaus wider than the search step, the argmax then breaks ties
  arbitrarily, and each "correction" injects a quantized random kick — measured
  to make drift WORSE (0.57 m vs 0.20 m baseline) before this was fixed.
- Zero-offset margin gate: a correction is applied only when the best candidate
  beats the current pose's own score by a clear margin AND small offsets are
  preferred via a magnitude penalty, so a confident-but-flat match field yields
  *no* correction instead of noise.

Important honesty property: the matcher never touches the simulator's ground-truth
pose or mesh.  Inputs are (a) body-frame scan endpoints and (b) the map built from
*estimated* poses — both exist on real hardware.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
    _HAVE_SCIPY = True
except Exception:                                    # pragma: no cover
    _HAVE_SCIPY = False


@dataclass
class ScanMatchConfig:
    search_xy_m: float = 0.30        # ± horizontal search window
    step_xy_m: float = 0.05          # horizontal search step
    search_yaw_deg: float = 3.0      # ± yaw search window
    step_yaw_deg: float = 1.0        # yaw search step
    sigma_m: float = 0.10            # Gaussian width of the likelihood field
    min_points: int = 40             # skip when the scan is too sparse to trust
    min_score: float = 0.45          # mean likelihood the best pose must reach
    # The best candidate must beat the zero-offset score by this margin, else no
    # correction (prevents plateau-tie noise injection).
    margin: float = 0.015
    # Per-candidate penalty proportional to offset magnitude — prefers the
    # smallest correction on a flat score surface.
    offset_penalty: float = 0.02
    # Fraction of the found offset actually applied — damps oscillation between
    # successive corrections (the rest is recovered on the next match).
    damping: float = 0.7


@dataclass
class ScanMatchResult:
    dx: float
    dy: float
    dyaw: float
    score: float
    accepted: bool


class MatchGrid:
    """High-resolution 2D obstacle-endpoint grid used ONLY for scan matching.

    The navigation occupancy grid is 0.2 m — far too coarse to match against:
    its likelihood ridge sits at cell centres, up to 0.1 m off the true wall
    surface and differently per wall, so the matcher acquires a wandering,
    geometry-dependent bias (measured: corrections made drift WORSE).  Real
    systems match on ~5 cm grids (Hector, Cartographer).  This grid stores
    clamped hit counts at 0.05 m; a cell counts as a surface once hit twice
    (single noisy returns don't mint walls).
    """

    def __init__(self, origin_xy: np.ndarray, extent_xy: np.ndarray,
                 resolution: float = 0.05) -> None:
        self.resolution = float(resolution)
        self.origin_xy = np.asarray(origin_xy, dtype=np.float64)
        h = max(8, int(np.ceil(float(extent_xy[0]) / self.resolution)) + 2)
        w = max(8, int(np.ceil(float(extent_xy[1]) / self.resolution)) + 2)
        self.counts = np.zeros((h, w), dtype=np.float32)

    def insert(self, points_xy: np.ndarray) -> None:
        """Accumulate obstacle endpoints (world XY, already pose-corrected)."""
        if not len(points_xy):
            return
        i = np.floor((points_xy[:, 0] - self.origin_xy[0]) / self.resolution).astype(np.int32)
        j = np.floor((points_xy[:, 1] - self.origin_xy[1]) / self.resolution).astype(np.int32)
        h, w = self.counts.shape
        ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
        np.add.at(self.counts, (i[ok], j[ok]), 1.0)
        np.clip(self.counts, 0.0, 10.0, out=self.counts)

    def occupied(self) -> np.ndarray:
        return self.counts >= 2.0


class CorrelativeScanMatcher:
    """Matches body-frame scan endpoints against a DiscoveryMap occupancy grid."""

    def __init__(self, cfg: Optional[ScanMatchConfig] = None) -> None:
        self.cfg = cfg or ScanMatchConfig()

    def _likelihood_field(self, grid: MatchGrid) -> np.ndarray:
        """exp(-(d/σ)²) of the distance to the nearest occupied cell (float32)."""
        occ = grid.occupied()
        if not occ.any():
            return np.zeros(occ.shape, dtype=np.float32)
        res = grid.resolution
        if _HAVE_SCIPY:
            d = distance_transform_edt(~occ) * res
        else:                                        # pragma: no cover
            # Fallback: two-step dilation approximating a short-range field
            d = np.full(occ.shape, 3.0 * res, dtype=np.float64)
            d[occ] = 0.0
            dil = occ.copy()
            dil[1:, :] |= occ[:-1, :]; dil[:-1, :] |= occ[1:, :]
            dil[:, 1:] |= occ[:, :-1]; dil[:, :-1] |= occ[:, 1:]
            d[dil & ~occ] = res
        return np.exp(-(d / self.cfg.sigma_m) ** 2).astype(np.float32)

    @staticmethod
    def _bilinear(field: np.ndarray, fi: np.ndarray, fj: np.ndarray) -> np.ndarray:
        """Sample *field* at fractional indices (fi, fj); out-of-bounds → 0."""
        h, w = field.shape
        i0 = np.floor(fi).astype(np.int32)
        j0 = np.floor(fj).astype(np.int32)
        ti = (fi - i0).astype(np.float32)
        tj = (fj - j0).astype(np.float32)
        out = np.zeros(fi.shape, dtype=np.float32)
        ok = (i0 >= 0) & (i0 < h - 1) & (j0 >= 0) & (j0 < w - 1)
        if not ok.any():
            return out
        i0o, j0o, tio, tjo = i0[ok], j0[ok], ti[ok], tj[ok]
        f00 = field[i0o, j0o]
        f10 = field[i0o + 1, j0o]
        f01 = field[i0o, j0o + 1]
        f11 = field[i0o + 1, j0o + 1]
        out[ok] = (f00 * (1 - tio) * (1 - tjo) + f10 * tio * (1 - tjo)
                   + f01 * (1 - tio) * tjo + f11 * tio * tjo)
        return out

    def match(
        self,
        body_xy: np.ndarray,         # (N, 2) obstacle endpoints in the BODY frame
        est_pos_xy: np.ndarray,      # estimated (x, y)
        est_yaw: float,              # estimated yaw
        grid: MatchGrid,             # match grid built from corrected poses
    ) -> ScanMatchResult:
        cfg = self.cfg
        n = len(body_xy)
        if n < cfg.min_points:
            return ScanMatchResult(0.0, 0.0, 0.0, 0.0, False)

        field = self._likelihood_field(grid)
        res = grid.resolution
        ox, oy = float(grid.origin_xy[0]), float(grid.origin_xy[1])
        px, py = float(est_pos_xy[0]), float(est_pos_xy[1])

        # Integer multiples of the step so the zero offset is EXACTLY 0.0
        kx = int(round(cfg.search_xy_m / cfg.step_xy_m))
        ky = int(round(cfg.search_yaw_deg / cfg.step_yaw_deg))
        shifts = cfg.step_xy_m * np.arange(-kx, kx + 1, dtype=np.float64)
        yaws = np.deg2rad(cfg.step_yaw_deg) * np.arange(-ky, ky + 1, dtype=np.float64)
        # Normalised offset-magnitude penalty per candidate
        max_off = cfg.search_xy_m * 2.0 + 1e-9
        max_yaw = np.deg2rad(cfg.search_yaw_deg) + 1e-9

        best = (0.0, 0.0, 0.0)
        best_obj = -np.inf
        best_score = 0.0
        score_zero = 0.0

        for dyaw in yaws:
            a = est_yaw + dyaw
            c, s = np.cos(a), np.sin(a)
            wx = px + body_xy[:, 0] * c - body_xy[:, 1] * s    # (N,)
            wy = py + body_xy[:, 0] * s + body_xy[:, 1] * c
            yaw_pen = cfg.offset_penalty * abs(dyaw) / max_yaw
            for dx in shifts:
                fi = (wx + dx - ox) / res - 0.5
                for dy in shifts:
                    fj = (wy + dy - oy) / res - 0.5
                    score = float(self._bilinear(field, fi, fj).mean())
                    if dx == 0.0 and dy == 0.0 and dyaw == 0.0:
                        score_zero = score
                    obj = score - yaw_pen - cfg.offset_penalty * (
                        (abs(dx) + abs(dy)) / max_off
                    )
                    if obj > best_obj:
                        best_obj = obj
                        best_score = score
                        best = (float(dx), float(dy), float(dyaw))

        accepted = (
            best_score >= cfg.min_score
            and best_score >= score_zero + cfg.margin
            and (best[0] != 0.0 or best[1] != 0.0 or best[2] != 0.0)
        )
        d = cfg.damping if accepted else 0.0
        return ScanMatchResult(
            best[0] * d, best[1] * d, best[2] * d, best_score, accepted
        )
