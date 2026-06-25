# AetherScan Electrical

The electrical system, and the one **custom PCB** in the build: the **Power &
Sensor Distribution Board (PSDB)**.

Most of the drone is COTS boards that already handle their own power (the 4-in-1
ESC, the Pixhawk, the Jetson dev kit). The PSDB exists to do the two things no
off-the-shelf board does cleanly for *this* sensor/compute payload:

1. **Isolate the compute rail.** The Jetson can spike to ~5 A at 5 V. If that
   shares a regulator with the flight controller, a compute transient can brown
   out the autopilot. The PSDB gives the Jetson its own 8 A BEC and the avionics
   their own 3 A rail.
2. **Break out the sensor bus.** One tidy board with labelled UART/I2C/USB/5V
   headers for the RPLIDAR, D435i, and flow/ToF — instead of a wiring rat's nest.

> **Tooling (chosen 2026-06-23):** the board is authored **as code** with
> [atopile](https://github.com/atopile/atopile) (`ato`) and reviewed with the
> standard KiCad ERC/DRC + DFM review — same
> code→build→verify loop as the airframe CAD. The code-defined PSDB lives in
> [pcb/](pcb/) and builds today (`make pcb`) with real LCSC-sourced parts. The
> component-level design intent is in [psdb-design.md](psdb-design.md) and
> [emi-thermal-failsafe.md](emi-thermal-failsafe.md).

## Power tree

```
            ┌──────────────── 4S LiPo (14.8 V, XT60) ─────────────────┐
            │                                                          │
        [PM02 power module] ──(V/I analog)──► Pixhawk 6C ADC           │
            │  (pass-through 14.8 V)                                   │
            ▼                                                          ▼
     ┌──────────────┐                                        [4-in-1 ESC] ──► 4× motor
     │     PSDB      │                                         (14.8 V, DShot from FC)
     │  14.8V in     │
     │   ├─ BEC 5V/8A ───────► Jetson Orin Nano (5V/4-5A)
     │   └─ BEC 5V/3A ──┬────► Pixhawk 6C (5V)
     │                  ├────► RPLIDAR A2M12 (5V)
     │                  ├────► RealSense D435i hub (5V)
     │                  └────► Matek 3901-L0X (5V)
     └──────────────┘
```

## Signal / data tree

```
  Jetson Orin Nano ──USB3──► RealSense D435i       (depth + IMU)
  Jetson Orin Nano ──USB───► RPLIDAR A2M12         (360° scan)
  Jetson Orin Nano ──UART──► Pixhawk 6C  (TELEM2)  MAVLink offboard setpoints ↕ state
  Pixhawk 6C ──UART/I2C────► Matek 3901-L0X        (flow + downward ToF → EKF2)
  Pixhawk 6C ──DShot───────► 4-in-1 ESC            (motor commands)
```

This is the hardware image of the software's high-level/low-level split:
- **Jetson = the `redwood_sim` autonomy stack** (perception, mapping, SLAM,
  planning, setpoint generation).
- **Pixhawk/PX4 = `controls.py` + `physics.py`'s real-world counterpart**
  (attitude/rate control, sensor fusion, motor mixing, failsafe).
- **The MAVLink UART = the WebSocket bridge** (`bridge/server.py`), in hardware.

## PSDB requirements (what the KiCad schematic must satisfy)

| # | Requirement |
|---|---|
| E1 | XT60 input, 14.8 V, reverse-polarity protected |
| E2 | 5 V / 8 A buck for the Jetson, isolated from E3 |
| E3 | 5 V / 3 A buck for FC + sensors |
| E4 | Bulk + local decoupling sized for the Jetson's 5 A transient |
| E5 | Labelled breakout headers: 4× 5 V, UART×2, I2C×1, GND planes |
| E6 | Mounting holes on the 30.5×30.5 stack pattern (fits the frame) |
| E7 | Fits within the plate footprint (≤ 50×50 mm), ≤ 25 g |
| E8 | Test points on both 5 V rails + battery sense |

## Wiring / installation notes
- Star-ground at the PSDB; keep the ESC high-current loop away from sensor signal
  lines (EMI into the IMU/flow sensor).
- Soft-mount the Pixhawk (grommets) — prop/motor vibration corrupts the IMU and
  therefore the estimator (the thing the whole REALWORLD audit is about).
- Keep the RPLIDAR's USB and the D435i's USB3 on separate Jetson controllers if
  possible (USB3 RF noise can desense the LiDAR link).
