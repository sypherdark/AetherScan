# AetherScan — Deployment Roadmap

Autonomous indoor **3D-scanning drone**. Goal: a navigation + perception stack
realistic enough to transfer to real hardware with minimal re-tuning, and a
product polished enough to ship.

> **Reality check on "works first-go on a real drone":** no simulator guarantees
> that. Sim-to-real always needs final calibration against the actual airframe and
> sensors. This roadmap minimises the gap (realistic sensing, identified dynamics,
> robust estimation, planners that tolerate uncertainty) and builds the hooks
> (flight-log export, system-ID, HITL) to close the last mile on real hardware.

## Target platform (assumed — adjust to your hardware)
- ~1.45 kg quadrotor, 0.18 m arm.
- **360° 2D LiDAR** (RPLIDAR-class): 0.15–12 m, ±2 cm, ~10 Hz.
- **Forward depth camera** (RealSense D435-class): 87°×58° FOV, 0.3–6 m, ~1% range noise, ~30 Hz.
- **Downward** 1-D ToF + optical-flow (altitude + body-velocity).
- 6-axis IMU. **No GPS** (indoor). Jetson-class companion compute.

## Phases

### Phase 1 — Perception realism  *(in progress)*
- [x] Range noise (σ = base + %·d), depth quantisation, dropout (range + grazing), noisy normals — `core/sensors.py`.
- [x] Geometry-only classification — stop reading ground-truth mesh semantics.
- [x] Disable precomputed-map injection (no a-priori structural oracle).
- [ ] Split sensors into true devices: 360° 2D LiDAR plane + forward depth cone (limited FOV) + downward ToF, each with its own rate.
- [ ] Sensor latency / update-rate decimation (sensors slower than the control loop).

### Phase 2 — State estimation (biggest sim-to-real gap)
- [x] Replace ground-truth pose into the planner with an **estimated** pose. (`core/state_estimation.py`, `config.use_estimated_pose`)
- [x] IMU + optical-flow dead-reckoning with realistic drift. (random-walk position+yaw bias + white noise; roll/pitch gravity-bounded)
- [x] 2D LiDAR scan-matching correction — `core/scan_matching.py` (Olson 2009 correlative matcher + keyframe insertion). Measured: drift bounded 0.04–0.10 m vs unbounded 0.138→0.326 m without.
- [ ] Global loop closure (pose graph) — drift bounded only against recent map; long exploration missions still accumulate.
- [ ] Expose estimate covariance to the planner (plan under uncertainty).

### Phase 3 — Mapping & planning
- [x] Probabilistic **occupancy grid** (2D/2.5D) — log-odds per-cell evidence, built only from sensor returns. (`core/discovery_map.py`)
- [x] Frontier-based exploration on the grid — BFS to exhaustion, FUEL-style global tour ordering (+15–56% coverage/m).
- [x] **Coverage planner** — unified discovery ∪ coverage objective; 11×11 vantage scoring with visit-count dilation. 7/9 rooms reached (compact drone model + 0.20 m inflation fixed doorway sealing).
- [x] Navigation closed on estimated pose — coverage parity vs ground truth (apt_1 35.2% vs 37.0%, room_0 89.8% vs 87.2%).
- [ ] Global planner upgrade (D* Lite / MPC) for large-scale scenes and replanning under uncertainty.

### Phase 4 — Dynamics & control fidelity
- [ ] Motor model: thrust curve, spin-up time constant, saturation, latency.
- [ ] Realistic IMU (bias, noise) feeding the estimator.
- [ ] PX4-style loop-rate separation; actuator delay.
- [ ] System-ID hooks + flight-log (rosbag/CSV) export for real-airframe calibration.

### Phase 5 — Product
- [ ] UI overhaul: live map/occupancy, coverage %, viewpoint planner, health.
- [ ] Mission planner (define area, altitude bands, scan density).
- [ ] Telemetry/health, structured logging, fault handling.
- [ ] Packaging, config profiles per airframe, tests + CI.

## Known weak points (tracked)
- Only `apartment_1` has a backend collision mesh; others load the full ~2 M-tri
  Replica mesh at start. `build_collision_mesh.py` can't parse Replica semantic PLY
  via open3d — port to trimesh and batch-build.
- `convert_replica_to_glb.py` emits inconsistent up-axes; dashboard now auto-detects,
  but the converter should be normalised to canonical Y-up.
- Project is not its own git repo (git root is the home dir) — needs `git init` + CI.
