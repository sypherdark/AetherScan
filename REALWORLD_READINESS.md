# AetherScan — Real-Drone Readiness Audit

> **The question:** *If I take the navigation, sensing, and reconstruction code and
> drop it onto a real drone, would it actually work?*
>
> **Honest answer: not yet — and there is exactly one root cause that dominates
> everything else.** The algorithms (frontier+coverage exploration, log-odds
> occupancy, holonomic path-following, voxel-deduplicated reconstruction) are real
> robotics techniques and would transfer. What would *not* transfer is the
> assumption they all silently depend on: **a perfect, drift-free pose, for free,
> at every tick.**

Audited against the actual code on `main` (2026-06-11).

---

## Severity 1 — would break on contact with reality

### 1.1 The whole stack consumes ground-truth pose
Every consumer — `SensorSuite.scan(position, quaternion)`, the discovery-map
update, the reconstruction cloud, and the planner — is fed
`controller.state.position / .quaternion`, which is the **simulator's exact
integrated truth**. There is no estimator anywhere in `core/` (grep for
EKF/IMU/odometry/VIO/SLAM → nothing).

A real indoor drone has **no GPS** and therefore no ground-truth position. It must
*estimate* its pose from IMU + a camera/LiDAR (VIO or LiDAR-inertial odometry), and
that estimate **drifts without bound** until something closes the loop. Today the
code would receive a drifting pose and:
- smear the occupancy grid (walls double up, doorways move),
- ghost the reconstruction (the same wall scanned twice from two poses lands in two
  places),
- send the planner toward stale coordinates.

**This is the single most important gap. Everything in Severity 2 matters only after
this is addressed.**

### 1.2 No loop closure / SLAM back-end
Even a *good* VIO drifts ~0.1–1 % of distance travelled. Over a 200 m indoor survey
that is metres of error. Without a SLAM back-end (pose-graph + loop closure, or
scan-matching against the map) the map and cloud are globally inconsistent. The code
has none — the map is built by direct accumulation, which is only valid under perfect
localization.

### 1.3 Reconstruction stores world-frame points, not measurements
`hit_points_labeled()` returns points already in the world frame, computed from the
true pose against the true mesh. A real sensor measures **range + bearing in the body
frame**; the world position is *derived* by applying the (drifting) pose. So the
reconstruction's accuracy is currently capped by an assumption it never tests.

## Severity 2 — would degrade quality / need hardware-specific work

### 2.1 Sensor field-of-view is idealized — NOW CONFIGURABLE AND MEASURED
`sensors.py` is calibrated as a **360° 2D LiDAR** (RPLIDAR-class) blended with a few
vertical rays. That is a legitimate sensor — *if that is the hardware*.
`SensorConfig.lidar_fov_deg` now models a fixed depth camera (set ~87 for
D435-class). **Measured (180 s): an 87° wedge halves coverage (room_0 86→49 %,
apartment_1 34→19 %) and degrades stability (blind-side reactions, tilt spikes
>60°).** Conclusion: with the current planner the hardware requirement is a 360°
LiDAR; a camera-only build additionally needs yaw-sweep scanning behaviour and a
short-term obstacle memory for avoidance (open work item).
*(Good news: the range/normal-noise model is already realistic — σ grows with range,
grazing angles are penalized.)*

### 2.2 Collision avoidance partly "knows" the map
The C-space inflation + BFS planner operate on the discovered grid (fine), but the
backend collision solver uses a BVH over the **known mesh**. On a real drone there is
no known mesh — reactive avoidance must run purely off live sensor returns. The
repulsion-from-sensors path already does this; the BVH safety net does not exist in
reality and may be masking cases where the reactive layer alone would clip a wall.

### 2.3 Control loop assumes perfect state feedback
The cascaded PID reads exact position/velocity/attitude. Real estimators deliver
**noisy, latent (20–50 ms delayed)** state. Gains tuned against perfect feedback can
ring or go unstable against delayed/noisy feedback. Needs a hardware-in-loop or
noise+latency injection pass on the controller.

### 2.4 No actuator model, latency, or comms budget
No motor spin-up dynamics, no sensor→compute→actuator latency, no wireless dropout
handling. A real autonomy stack must tolerate a dropped frame and a late command.

## Severity 3 — polish / completeness

- Semantic labels are geometry heuristics (normal+height). Real semantics need a
  perception model or are simply unavailable — the product should not over-promise.
- Metric scale: a monocular front-end has scale ambiguity; needs stereo/depth or IMU
  scale recovery. (A LiDAR build is fine here.)
- Power/thermal budget for running VIO+SLAM+planning on an onboard SoM is unmodeled.

---

## Strategy — order of attack (Agent 4)

1. ✅ **Introduce a state-estimation seam** so perception/mapping/reconstruction consume
   an *estimated* pose (with realistic drift + noise), not ground truth.
   → `core/state_estimation.py`, `config.use_estimated_pose` (default off).
2. ✅ **Quantify the gap:** apartment_1, 120 s / 13 m: drift 0.20 m end / 0.27 m max
   (grows ~√t), reconstruction +32% ghost points vs ground truth.
3. ✅ **Add a SLAM correction** to *bound* the drift → `core/scan_matching.py`
   (correlative matcher, Olson 2009) + keyframe match grid in the engine.
   **Measured (apartment_1, 300 s): without SLAM drift grows 0.138→0.219 m
   (max 0.326); with SLAM it holds flat at 0.04–0.10 m (max 0.192) and
   reconstruction ghosting drops 22%.** Three failure modes were found by
   measurement and fixed on the way — see the docstrings in
   `scan_matching.py` / `engine._update_perceived_slam` (coarse-grid likelihood
   ridges; insert-at-raw-estimate map smear; the per-tick-insertion gauge-mode
   limit cycle, fixed by keyframe insertion).
   *Still open at this layer: global loop closure (pose graph) — drift during
   long pure-exploration stretches is bounded only relative to the recent map.*
4. ✅ **Enforce the real sensor FOV** — `SensorConfig.lidar_fov_deg` (default 360).
   Measured: 87° camera-class FOV halves coverage and destabilizes avoidance →
   360° LiDAR is the validated hardware baseline; camera-only needs yaw-sweep +
   obstacle memory (follow-up).
5./6. ✅ **Close the navigation loop on the estimate.**  With
   `use_estimated_pose=True` the discovery map, frontier goals, path following,
   and trajectory setpoints all live in the drone's OWN estimated frame (raycasts
   still happen from the true pose — they are the physical measurement; hits are
   re-expressed through the estimate).  Altitude bias is pinned
   (`EstimatorConfig.z_bounded` — the downward ToF bounds Z on real hardware).
   **Measured (300 s, SLAM on): flying on its own estimate the drone reaches
   coverage parity with ground-truth navigation (apartment_1 35.2 % vs 37.0 %;
   room_0 89.8 % vs 87.2 %) with LOWER tilt and fewer proximity contacts, and
   end drift held at 0.03–0.04 m.**  The full real-drone pipeline — noisy
   sensing → drifting odometry → scan-match correction → mapping → planning →
   control — closes the loop end-to-end in sim.
   *Remaining idealization at this layer: velocity feedback is direct (VIO/flow
   velocity is high-quality in practice) and estimator latency is not yet
   modeled.*

**Regression suite:** `redwood_sim/tests/test_state_estimation.py` (8 tests) locks
the measured invariants: matcher exact-recovery + zero-drift-no-correction (the
plateau-noise runaway), sparse/empty-map rejection, unbounded drift without
correction, bounded Z, correction algebra, gravity-bounded roll/pitch.
Run: `redwood_sim/.venv/bin/python -m pytest redwood_sim/tests -q`.

## Research-grounded upgrades (2026-06-11)

Benchmarked the project against published systems and field deployments:

- **FUEL** (Zhou et al., RA-L 2021) identifies greedy frontier exploration's core
  failure — locally-best target selection ignores global route optimality and
  pays corridors twice.  Adopted scaled-down: `discovery_map._tour_first_stop`
  orders the top spatially-distinct vantage candidates as a shortest open tour
  and commits to the first leg, re-derived per goal request.
  **Measured: +15–56 % coverage-per-meter at equal coverage.**
- **Emesent Hovermap** field practice: autonomy must fly the vehicle *in a way
  that keeps SLAM healthy*.  Adopted: `slam_health` (EMA of match score) gates
  cruise speed — the drone slows when matching degrades instead of outrunning
  its own localization.
- **DARPA SubT** teams (CERBERUS, CoSTAR/NeBula): state-estimation failures
  cascade through the stack unless the system degrades gracefully.  Adopted:
  uncertainty-aware reactive standoffs in estimate-nav (+0.10/+0.15 m — never
  grid inflation, which seals doorways).  **Measured: wall grazes 140→52,
  end drift 0.148→0.058 m on the cluttered scene.**

Items 1–6 convert the headline risk from a hand-wave into measured engineering.
**Updated answer to the question at the top: in simulation, yes — the autonomy
stack now demonstrably works without ground truth.**  What remains before
hardware: estimator latency injection, global loop closure for long missions,
the camera-FOV mitigations (if not flying a 360° LiDAR), actuator/comms
modeling, and a hardware-in-the-loop pass.
