# Power Budget

**Pack: 4S LiPo 6000 mAh = 14.8 V nominal × 6.0 Ah = 88.8 Wh.**

## Draw

| Load | Power (W) | Notes |
|---|---:|---|
| Propulsion (hover) | ~205 | 1.45 kg at ~7 g/W for a 7" setup → ~207 W; use 205 W planning figure |
| Jetson Orin Nano | 12 | 7–15 W typical; 25 W max mode (cap it to 15 W for endurance) |
| RPLIDAR A2M12 | 3 | motor + scan |
| RealSense D435i | 2.5 | USB3, depth streaming |
| Pixhawk 6C + flow/ToF | 3 | autopilot + downward sensors |
| **Avionics subtotal** | **~20.5** | the non-propulsion payload |
| **Total hover** | **~225 W** | |

## Endurance

Usable energy at 80% depth-of-discharge: `88.8 × 0.8 = 71 Wh`.
Hover endurance: `71 / 225 ≈ 0.32 h ≈ **19 min**`.
Realistic indoor scanning (accel/decel, not pure hover): **~15–17 min usable**.
Ample for room-to-apartment scans (the sim missions run 2–5 min sim-time).

## Rails the custom PSDB must provide

| Rail | Voltage | Continuous | Feeds |
|---|---|---:|---|
| Battery | 14.8 V (4S) | 60 A peak | 4-in-1 ESC (motors) |
| Companion 5V | 5.0 V | **6 A** | Jetson Orin Nano (5 V/4 A typical, 5 A peak) |
| Avionics 5V | 5.0 V | 3 A | Pixhawk 6C, RPLIDAR, flow/ToF, D435i hub |
| Telemetry | analog | — | PM02 voltage/current sense → FC ADC |

**Design rule:** the Jetson gets its **own** regulator (Mateksys BEC12S-PRO, 8 A)
— never share a rail between a 5 A compute spike and the flight controller, or a
Jetson brownout takes the autopilot with it. The PSDB routes both rails plus the
sensor UART/I2C/USB breakout. This board is the electrical-phase deliverable.

## Safety / failsafe (maps to software)
- PX4 low-battery failsafe at 3.5 V/cell → land. The sim has no battery model
  yet; add a `battery_remaining` telemetry field when porting (`ROADMAP.md`
  Phase 5 health) so the mission planner can return-to-home before depletion.
- Companion brownout → PX4 holds position on its own EKF2 (it does not depend on
  the companion for stability — only for *where to go*). This is exactly the
  sim's high-level/low-level split.
