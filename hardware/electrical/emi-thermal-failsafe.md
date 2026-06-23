# EMI, Thermal & Failsafe Plan

Closes the electrical design-level open items (ELEC's deployment-readiness
holdout): EMI control, Jetson thermals, and power telemetry + failsafe behaviour.

## 1. EMI — the USB3 ↔ LiDAR ↔ IMU problem

USB3 (the D435, 5 Gbps) radiates a broadband ~2.4 GHz noise floor that can desense
the RPLIDAR link and inject into the IMU — and the IMU is the front end of the
estimator the whole project protects. Mitigations (layered, cheapest first):

1. **Physical separation + orientation** — D435 USB3 cable routed down the opposite
   side of the canopy from the RPLIDAR UART and the FC; cross at 90° if unavoidable.
2. **Shielding** — shielded USB3 cable, shield grounded at the host end only; a thin
   copper-tape shield can over the FC/IMU if a bench scan shows IMU noise.
3. **Filtering** — common-mode choke on the USB3 pair at the PSDB; ferrite beads on
   every 5 V rail (see psdb-design.md); series R + small C on UART/I²C.
4. **Grounding** — 4-layer PSDB with a solid inner GND plane; star ground at the
   shunt; no high-current ESC return through the sensor ground.

**Acceptance test:** stream D435 depth + RPLIDAR simultaneously on the bench and
confirm (a) no rise in IMU gyro noise density vs USB3-idle, (b) no LiDAR dropped
scans. This is a bring-up gate, not a flight gate.

## 2. Thermal — Jetson Orin Nano in the canopy

| Load | Power | Cooling | Margin |
|---|---|---|---|
| Jetson Orin Nano | 7–15 W (cap 15 W mode) | finned heatsink **in the rotor downwash** (~2 m/s forced air) | a 40×40 mm finned sink at h≈30 W/m²K, ~0.02 m² fin area sheds ~15 W at ΔT≈25 °C → junction well under the 97 °C throttle point |
| 4-in-1 ESC | hover ~14 A total (3.4 A/motor) of a 65 A rating | open-air on the bottom plate | barely warm; huge margin |
| PSDB 8 A buck | ~3 W | copper pour + vias, downwash side | fine |

**Design actions:** ventilation slots in the canopy aligned with the downwash
column over the Jetson sink; mount the Jetson sink fins along the airflow; if a
bench soak at 15 W in still air exceeds 80 °C, add a 25 mm fan (the dev-kit fan
already fits). Throttling would degrade the autonomy (dropped frames → worse SLAM),
so this is treated as a real requirement, not an afterthought.

## 3. Telemetry & failsafe — graceful degradation

Power telemetry: **PM02** → Pixhawk analog (native battery estimator) **and**
**INA226** → Jetson over I²C (companion-side energy model for the mission planner).

PX4 battery failsafe (4S, account for sag):
- **Warn** at 3.5 V/cell (14.0 V) → mission planner begins wrap-up / heads home.
- **Critical / land** at 3.4 V/cell (13.6 V) → `COM_LOW_BAT_ACT` = return or land.
- **Emergency** at 3.3 V/cell → immediate land.

Companion (Jetson) brownout or MAVLink loss:
- PX4 keeps flying on its **own EKF2** and holds position — it never depends on the
  companion for *stability*, only for *where to go*. On link loss it enters HOLD
  then `COM_OBL_ACT` (RTL/land). This is the hardware image of the simulation's
  high-level (autonomy) / low-level (flight) split: the bridge can drop and the
  aircraft stays safe.

Add to the sim when porting (`ROADMAP.md` Phase 5 health): a `battery_remaining`
telemetry field so the mission planner triggers return-to-home before depletion —
the one piece of this failsafe chain the simulation does not yet model.

## Status
With psdb-design.md, this closes the electrical design at the **design level**.
Remaining electrical work is **build-phase**: KiCad layout/Gerbers (kicad-happy
tool), fabrication, and the two bench bring-up gates above — all gated on funded
parts and the PCB tool, not on further design.
