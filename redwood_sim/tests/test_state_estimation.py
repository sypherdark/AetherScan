"""Regression tests for the real-world readiness layer (estimator + scan matcher).

These lock in invariants that were each established by MEASUREMENT after a real
failure (see REALWORLD_READINESS.md and the docstrings in core/scan_matching.py):
tuning changes that re-break them caused 0.5–2.3 m drift runaways in the past.

Run:  redwood_sim/.venv/bin/python -m pytest redwood_sim/tests -q
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.scan_matching import CorrelativeScanMatcher, MatchGrid          # noqa: E402
from core.state_estimation import EstimatorConfig, PoseEstimator         # noqa: E402


# ─────────────────────────── helpers ────────────────────────────────────────

def synthetic_room_scan(yaw: float, n: int = 150):
    """Wall endpoints of an 8×6 m rectangular room seen from the origin."""
    pts = []
    for ang in np.linspace(-np.pi, np.pi, n, endpoint=False):
        dx, dy = np.cos(ang), np.sin(ang)
        ts = []
        if dx > 1e-9:
            ts.append(3.9 / dx)
        if dx < -1e-9:
            ts.append(-3.9 / dx)
        if dy > 1e-9:
            ts.append(2.9 / dy)
        if dy < -1e-9:
            ts.append(-2.9 / dy)
        t = min(v for v in ts if v > 0)
        pts.append([t * dx, t * dy])
    world = np.asarray(pts)
    c, s = np.cos(yaw), np.sin(yaw)
    body = np.column_stack(
        [world[:, 0] * c + world[:, 1] * s, -world[:, 0] * s + world[:, 1] * c]
    )
    return world, body


def room_grid(world_pts: np.ndarray) -> MatchGrid:
    grid = MatchGrid(np.array([-4.5, -3.5]), np.array([9.0, 7.0]))
    grid.insert(world_pts)
    grid.insert(world_pts)          # cells need 2 hits to count as surface
    return grid


# ─────────────────────────── scan matcher ───────────────────────────────────

def test_matcher_zero_drift_gives_no_correction():
    """Plateau-tie noise injection regression: a well-aligned scan must NOT be
    'corrected' (this exact failure ratcheted yaw at −0.7°/match)."""
    yaw = 0.3
    world, body = synthetic_room_scan(yaw)
    r = CorrelativeScanMatcher().match(body, np.zeros(2), yaw, room_grid(world))
    assert not r.accepted
    assert r.dx == r.dy == r.dyaw == 0.0


def test_matcher_recovers_known_offset():
    """A known pose error must be recovered to the search step, damped."""
    yaw = 0.3
    world, body = synthetic_room_scan(yaw)
    m = CorrelativeScanMatcher()
    r = m.match(body, np.array([0.15, -0.10]), yaw + np.deg2rad(2.0),
                room_grid(world))
    assert r.accepted
    d = m.cfg.damping
    assert r.dx == pytest.approx(-0.15 * d, abs=m.cfg.step_xy_m)
    assert r.dy == pytest.approx(0.10 * d, abs=m.cfg.step_xy_m)
    assert np.degrees(r.dyaw) == pytest.approx(-2.0 * d, abs=m.cfg.step_yaw_deg)


def test_matcher_rejects_sparse_scan():
    yaw = 0.0
    world, body = synthetic_room_scan(yaw, n=10)   # below min_points
    r = CorrelativeScanMatcher().match(body, np.zeros(2), yaw, room_grid(world))
    assert not r.accepted and r.score == 0.0


def test_matcher_rejects_empty_map():
    yaw = 0.0
    _, body = synthetic_room_scan(yaw)
    empty = MatchGrid(np.array([-4.5, -3.5]), np.array([9.0, 7.0]))
    r = CorrelativeScanMatcher().match(body, np.zeros(2), yaw, empty)
    assert not r.accepted


# ─────────────────────────── pose estimator ─────────────────────────────────

def test_estimator_drift_grows_without_correction():
    """The bias random walk must dominate the white-noise floor over 300 s.

    A random walk is not monotone (a single seed can wander back toward zero),
    so the invariant is: the PEAK error far exceeds what white noise alone can
    produce (~0.05 m for 3 axes at σ=0.01), and a bias has accumulated.
    Checked across several seeds.
    """
    pos = np.array([0.0, 0.0, 1.5])
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    maxes, biases = [], []
    for seed in (3, 7, 11):
        est = PoseEstimator(EstimatorConfig(rng_seed=seed))
        est.reset(pos)
        errs = [est.update(pos, quat, 0.05).pos_error_m for _ in range(6000)]
        maxes.append(max(errs))
        biases.append(float(np.linalg.norm(est._pos_bias)))
    assert min(maxes) > 0.10          # every seed: peak ≫ white-noise floor
    assert np.mean(biases) > 0.08     # an accumulated bias exists on average


def test_estimator_z_bounded():
    """Altitude is pinned by the downward ToF — Z bias must not random-walk."""
    est = PoseEstimator(EstimatorConfig(rng_seed=3))
    pos = np.array([0.0, 0.0, 1.5])
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    est.reset(pos)
    for _ in range(6000):
        e = est.update(pos, quat, 0.05)
    assert abs(e.position[2] - 1.5) < 0.10        # white noise only
    assert abs(est._pos_bias[2]) < 1e-12


def test_estimator_correction_cancels_bias():
    est = PoseEstimator(EstimatorConfig(rng_seed=3))
    pos = np.array([0.0, 0.0, 1.5])
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    est.reset(pos)
    for _ in range(2000):
        est.update(pos, quat, 0.05)
    est.correct(-est._pos_bias.copy(), -est._yaw_bias)
    assert float(np.linalg.norm(est._pos_bias)) < 1e-9
    assert abs(est._yaw_bias) < 1e-9


def test_estimator_roll_pitch_bounded():
    """Roll/pitch are gravity-observable: only white jitter, never a bias walk."""
    from core.math3d import quat_to_euler
    est = PoseEstimator(EstimatorConfig(rng_seed=5))
    pos = np.zeros(3)
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    est.reset(pos)
    rolls = []
    for _ in range(4000):
        e = est.update(pos, quat, 0.05)
        rolls.append(quat_to_euler(e.quaternion)[0])
    assert abs(float(np.mean(rolls))) < 0.005     # zero-mean
    assert float(np.std(rolls)) < 0.02            # bounded jitter
