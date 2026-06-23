# AetherScan — Converged Airframe Spec

The single-page reference the council (`DESIGN_COUNCIL.md`) converged on. This is
what `cad/frame.py` builds and what the specs/BOM must agree with.

## Topology
- **Symmetric quad-X**, 360 mm motor-to-motor diagonal (`arm = 0.18 m`, exact).
- Four **identical** arms (round 16 mm carbon tube) → one clamp, one spare.
- +X = nose/forward, Z = up (flight-software frame).

## Structure
- **Lofted carbon/printed canopy** fairing (132×96 mm base tapering to 78×54 mm,
  46 mm tall) — clean prop inflow, houses battery low + avionics.
- **Tapered carbon frustum mast** (Ø34→Ø18 mm, 82 mm) — stiff, raises the LiDAR.
- **Twin curved skids** — clear the belly flow/ToF sensor and the battery.
- One fastener system throughout: **M3**.

## Sensors (placement = the council's hard requirements)
| Sensor | Placement | Why |
|---|---|---|
| RPLIDAR A2M12 | Top of mast, scan plane ≈135 mm, **damped mount** | unobstructed 360° horizon above the prop disc; damping protects the IMU/estimator |
| RealSense D435i | Nose boom **ahead of the prop disc**, **12° down-tilt**, axis +X | no blade in the 87° FOV; forward + floor view |
| Matek 3901-L0X | Belly centre, facing −Z | clear nadir cone for flow + ToF |

## Propulsion & power
- 4× iFlight XING2 2806.5 1300KV, 7" props, 4-in-1 65 A ESC → TWR ≈ 3.6–4.4.
- **Removable** full prop-guard rings (Ø~200 mm, indoor collision tolerance).
- 4S pack; **nominal guarded config uses 5200 mAh to hit 1.45 kg AUW** (a 6000 mAh
  open-area config trades guards for ~2 min more endurance).
- Custom **PSDB**: isolated 5 V/8 A (Jetson) + 5 V/3 A (avionics), star ground.

## Compute split (= the simulation's architecture, in hardware)
- **Jetson Orin Nano** runs the `redwood_sim` autonomy (perception→SLAM→planning).
- **Pixhawk 6C / PX4** runs low-level flight control (the real `controls.py`).
- **MAVLink UART** between them = the sim's WebSocket bridge.
- Jetson heatsink sits in the rotor downwash (free forced-air cooling).

## Mass & inertia
- AUW: **1.45 kg** (guarded, 5200 mAh) — on contract.
- CG: on the thrust centre, ~17 mm above the rotor plane.
- **Inertia: CAD-derived ≈ Ixx 0.0075 / Iyy 0.011 / Izz 0.012 kg·m².** The sim's
  0.014/0.014/0.026 is physically unreachable for this airframe → reconcile the
  **sim** down + re-tune the controller (gated decision; `specs/inertia-findings.md`).

## Portability (follow-up CAD)
- Folding arms with an **exact, latched 0.18 m deployed hard stop**.
- Tool-less removable LiDAR mast; belly quick-release battery.
- Strain-relieved motor leads at the hinges.

## Traceability
Every dimension lives in `cad/parameters.py`; the software-derived ones are cited
to their source line and guarded by `cad/check_against_software.py`. Change the
sim → the check fails → the hardware is updated to match. That loop is the
guarantee that the structure, the electronics, and the software stay in accordance.
