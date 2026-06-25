# AetherScan Hardware — System Specification

This is the **contract** between the flight software and the physical drone.
Every requirement below traces to a specific software constant or a documented
requirement in `REALWORLD_READINESS.md` / `ROADMAP.md`. If you change a number
here, change it in the software too (and re-run `cad/check_against_software.py`).

> The whole point of this project's hardware phase: **build the drone the
> simulation was actually tuned against** — not a generic quad we hope behaves
> similarly.

---

## 1. Airframe envelope (from the physics model)

| Requirement | Value | Source | Why it matters |
|---|---|---|---|
| All-up weight (AUW) | **1.45 kg** | `physics.py:28` `mass` | Thrust/power/inertia were all tuned to this mass |
| Arm length (centre→motor) | **0.18 m** | `physics.py:33` `arm_length` | Sets the 360 mm diagonal wheelbase + control moment arm |
| Roll/pitch inertia | **0.014 kg·m²** | `physics.py:30-31` | Mass distribution target; attitude PID gains assume it |
| Yaw inertia | **0.026 kg·m²** | `physics.py:32` | Yaw-rate response; allocator headroom assumes it |
| Max tilt | **0.48 rad (27.5°)** | `physics.py:39` | Structural + control envelope (never exceed in design loads) |
| Per-motor max thrust | **≈ 12.1 N (1.23 kgf)** | `physics.py:149` `T_max=0.85·mg` | ⇒ 4.9 kgf total ⇒ **3.4:1 TWR** — the propulsion sizing target |
| Body collision radius | **0.18 m** | `config.py:15` | Prop guards + geometry must fit inside this in the planner |

**Propulsion sizing** that satisfies the TWR target: 4× iFlight XING2 2806.5
1300KV on 7" props at 4S deliver ~1.3–1.6 kgf each → 5.2–6.4 kgf total → TWR
3.6–4.4:1. Meets the ≥3.4:1 requirement with margin for the avionics payload.

## 2. Sensor suite (from the autonomy stack)

| Sensor | Selected part | Software requirement | Source |
|---|---|---|---|
| 360° 2D LiDAR | Slamtec RPLIDAR A2M12 (0.15–12 m) | The **validated** primary sensor; 87° camera-only FOV halves coverage | `REALWORLD_READINESS.md` §2.1, `sensors.py` 360° baseline |
| Forward depth + IMU | Intel RealSense D435i (87° HFOV) | Forward depth cone for reactive avoidance | `ROADMAP.md` target platform |
| Downward ToF + flow | Matek 3901-L0X (VL53L1X + PMW3901) | Altitude + body-velocity; pins estimator Z bias | `state_estimation.py` `z_bounded`, `ROADMAP.md` |
| IMU (×redundant) | Pixhawk 6C internal + D435i | 6-axis for the estimator front-end | `ROADMAP.md` |
| GPS | **none** | Indoor — no GPS by design | `REALWORLD_READINESS.md` §1.1 |

**Geometric constraints these impose on the CAD:**
- The RPLIDAR scan plane must clear the prop disc with an unobstructed 360°
  horizon → it sits on the **top mast** (`frame.lidar_mast()`).
- The D435i 87° forward cone must be clear of frame/guards ahead of the nose
  (+X). Forward = the drone's heading and the depth look-direction.
- The downward flow/ToF needs a clear nadir cone → landing gear is tall enough
  (`Frame.leg_height`) and the battery does not intrude on the cone.

## 3. Compute & autopilot split

| Layer | Part | Runs |
|---|---|---|
| Companion (high-level autonomy) | Jetson Orin Nano 8GB | The ported `redwood_sim/core` stack: sensing → discovery map → SLAM → frontier/coverage planner → setpoint generation |
| Autopilot (low-level flight) | Pixhawk 6C (PX4) | Attitude/rate control, motor mixing, EKF2 sensor fusion, failsafes |
| Link | MAVLink (UART) | Companion sends offboard position/velocity setpoints; FC streams state back |

This mirrors the simulation's split exactly: `redwood_sim` (high-level, what the
companion runs) vs. `controls.py`/`physics.py` (low-level, what PX4 + the
airframe do). The WebSocket bridge in sim becomes the MAVLink link in hardware.

## 4. Power architecture (see specs/power-budget.md)

4S LiPo → 4-in-1 ESC (motors) + power module (telemetry) + **custom Power &
Sensor Distribution Board (PSDB)** providing the 5 V rails for the Jetson, FC,
and sensors, plus the UART/I2C breakout. The PSDB is the deliverable of the
electrical/KiCad phase.

## 5. Coordinate frame agreement

The software is **Z-up ROS/ENU** (X=forward, Y=left, Z=up — see
`project-aetherscan` memory and README). The CAD uses the **same** frame:
`+X` is the nose / D435 look direction, `+Z` is up. Mount the FC and D435 so
their body axes match this — otherwise the ported estimator and planner inherit
a rotation the simulation never had.

---

## Open items before this spec is "build-ready"
- [ ] Mass distribution iteration to actually hit Ixx/Iyy/Izz (not just total mass) — see mass-budget.md.
- [ ] PSDB schematic + PCB (electrical phase, KiCad review skill).
- [ ] Sensor brackets (D435 nose mount, flow/ToF belly mount) as separate CAD modules.
- [ ] Vibration isolation for the FC (soft-mount grommets) and the D435i IMU.
- [ ] Thermal: Jetson airflow path vs. prop wash.
