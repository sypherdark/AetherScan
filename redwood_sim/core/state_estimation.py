"""
Onboard pose estimation — the seam that makes AetherScan honest about the real world.

On a real indoor drone there is **no GPS and no ground-truth pose**.  The drone
estimates where it is from IMU + camera/LiDAR (visual-inertial or LiDAR-inertial
odometry), and that estimate **drifts without bound** until a SLAM back-end closes
the loop.  Until this module existed, every perception/mapping/reconstruction
consumer in the codebase silently read the simulator's *perfect* pose — the single
biggest reason the stack would not transfer to hardware (see REALWORLD_READINESS.md).

`PoseEstimator` models that drift with the standard robotics odometry-error model:
a slowly random-walking bias (the unbounded VIO drift) plus per-sample white noise
(the high-frequency jitter).  It is deliberately *not* a full strapdown INS — the
goal is to reproduce the **failure mode** (a locally-accurate, globally-drifting
pose) so the rest of the system can be measured and hardened against it, and so a
SLAM correction has a well-defined seam (`correct`) to inject into later.

Frames: ROS Z-up, same as the rest of the backend.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from core.math3d import quat_from_euler, quat_to_euler, quat_normalize


@dataclass
class EstimatorConfig:
    # Position drift: a random-walk velocity bias integrated into position.  ~0.5 %%
    # of distance travelled is typical for decent VIO; we drive it via a bias whose
    # random walk has this strength (m per sqrt(second)).
    pos_drift_rw: float = 0.012          # bias random-walk strength  [m/s/√s]
    pos_white_noise: float = 0.010       # per-sample position jitter [m, 1σ]
    # Yaw is the dominant indoor VIO error (gravity bounds roll/pitch, yaw is free).
    yaw_drift_rw: float = 0.0035         # yaw bias random-walk        [rad/√s]
    yaw_white_noise: float = 0.004       # per-sample yaw jitter       [rad, 1σ]
    # Roll/pitch are gravity-observable on a real IMU → small, bounded noise only.
    tilt_white_noise: float = 0.006      # per-sample roll/pitch jitter [rad, 1σ]
    # Altitude is bounded on a real indoor drone by the downward ToF rangefinder
    # (+barometer), so the Z bias does NOT random-walk — only white noise remains.
    # Without this, flying navigation on the estimate would drift into the floor
    # or ceiling for a reason real hardware doesn't have.
    z_bounded: bool = True
    rng_seed: Optional[int] = None


@dataclass
class EstimatedState:
    position: np.ndarray
    quaternion: np.ndarray
    # Diagnostics (not available on real hardware — for measuring the sim-to-real gap)
    pos_error_m: float = 0.0
    yaw_error_deg: float = 0.0


class PoseEstimator:
    """Turns a true pose into a realistically-drifting *estimated* pose.

    Usage per control tick::

        est = estimator.update(true_position, true_quaternion, dt)
        # feed est.position / est.quaternion to scan / mapping / reconstruction

    A future SLAM back-end calls :meth:`correct` with a measured pose correction to
    bound the drift (loop closure / scan-matching).  Today nothing calls it, which
    is exactly why the drift is unbounded — the measurement that motivates SLAM.
    """

    def __init__(self, cfg: Optional[EstimatorConfig] = None) -> None:
        self.cfg = cfg or EstimatorConfig()
        self._rng = np.random.default_rng(self.cfg.rng_seed)
        self._pos_bias = np.zeros(3, dtype=np.float64)   # random-walking position bias
        self._yaw_bias = 0.0                             # random-walking yaw bias
        self._initialized = False

    def reset(self, position: np.ndarray, yaw: float = 0.0) -> None:
        """Anchor the estimator at a known start pose (drift starts from zero)."""
        self._pos_bias[:] = 0.0
        self._yaw_bias = 0.0
        self._initialized = True

    def correct(self, pos_correction: np.ndarray, yaw_correction: float = 0.0) -> None:
        """Inject a SLAM/loop-closure correction that shrinks accumulated drift.

        `pos_correction` is added to the bias (so a perfect correction of
        ``-self._pos_bias`` zeroes the drift).  Wired for the Phase-3 SLAM back-end.
        """
        self._pos_bias += np.asarray(pos_correction, dtype=np.float64)
        self._yaw_bias += float(yaw_correction)

    def update(self, true_position: np.ndarray, true_quaternion: np.ndarray,
               dt: float) -> EstimatedState:
        cfg = self.cfg
        if not self._initialized:
            self.reset(true_position)

        dt = max(float(dt), 1e-4)
        sdt = np.sqrt(dt)

        # ── Random-walk the biases (unbounded drift — the real VIO failure mode) ──
        self._pos_bias += self._rng.normal(0.0, cfg.pos_drift_rw * sdt, size=3)
        if cfg.z_bounded:
            self._pos_bias[2] = 0.0      # altitude pinned by the downward ToF
        self._yaw_bias += float(self._rng.normal(0.0, cfg.yaw_drift_rw * sdt))

        # ── Estimated position = truth + accumulated drift + white jitter ──
        est_pos = (np.asarray(true_position, dtype=np.float64)
                   + self._pos_bias
                   + self._rng.normal(0.0, cfg.pos_white_noise, size=3))

        # ── Estimated attitude: yaw drifts, roll/pitch only jitter (gravity-bounded) ──
        roll, pitch, yaw = quat_to_euler(true_quaternion)
        est_roll = roll + self._rng.normal(0.0, cfg.tilt_white_noise)
        est_pitch = pitch + self._rng.normal(0.0, cfg.tilt_white_noise)
        est_yaw = yaw + self._yaw_bias + self._rng.normal(0.0, cfg.yaw_white_noise)
        est_quat = quat_normalize(quat_from_euler(est_roll, est_pitch, est_yaw))

        pos_err = float(np.linalg.norm(est_pos - np.asarray(true_position)))
        yaw_err = float(np.degrees(abs((est_yaw - yaw + np.pi) % (2 * np.pi) - np.pi)))
        return EstimatedState(est_pos, est_quat, pos_err, yaw_err)
