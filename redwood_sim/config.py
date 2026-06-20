"""Global simulation configuration."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SimConfig:
    physics_dt: float = 0.002
    control_dt: float = 0.01
    control_decimation: int = 5
    waypoint_capture_radius: float = 0.65
    patrol_speed: float = 0.62
    drone_body_radius: float = 0.18
    ground_effect_height: float = 0.5
    ground_effect_gain: float = 0.22
    window_width: int = 1600
    window_height: int = 1000
    voxel_downsample: float = 0.03
    default_scene: str = "apartment"
    # Real-drone fidelity: when True, perception/mapping/reconstruction consume an
    # *estimated* (drifting, noisy) pose from core.state_estimation instead of the
    # simulator's ground truth — the seam that lets us measure the sim-to-real gap
    # and, later, validate a SLAM correction.  Default False keeps the demo on the
    # perfect-pose path.  See REALWORLD_READINESS.md.
    use_estimated_pose: bool = False
