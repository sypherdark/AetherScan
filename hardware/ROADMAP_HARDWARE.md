# AetherScan — Hardware Roadmap

**Premise:** we have a validated autonomy stack in simulation and **no budget yet
for parts**. So we de-risk everything that costs only time now, and stage the
spend so the first dollar buys the highest-confidence component. Each phase
produces a concrete artifact and leaves the project in accordance with the
software (`cad/check_against_software.py` gates every CAD change).

Legend: ✅ done · 🔧 in progress · ⬜ not started · 💰 needs funds

---

## Phase H0 — Paper drone (no spend) — 🔧

Everything that makes the build *real on paper* before buying anything.

- ✅ Lock the hardware contract to the software (`cad/parameters.py` + checker).
- ✅ BOM of real, currently-purchasable parts (`bom/bom.csv`) — $1,659.
- ✅ System spec, mass budget, power budget (`specs/`).
- ✅ Parametric airframe CAD that exports GLB/STEP/STL (`cad/frame.py`).
- ⬜ Sensor brackets (D435 nose, flow/ToF belly, LiDAR deck detailing).
- ⬜ Compute tray (Jetson + FC) as its own CAD module.
- ⬜ Full assembly with real per-part masses → compute the inertia tensor and
  iterate to hit Ixx/Iyy/Izz, not just total mass (mass-budget.md).
- ⬜ Power & Sensor Distribution Board (PSDB) schematic + PCB (electrical phase,
  kicad-happy skill).

**Exit:** a complete digital twin — geometry, mass properties, wiring, and a PCB
you could send to JLCPCB — all consistent with `redwood_sim`.

## Phase H1 — Bench brain (~$530) — 💰

Buy the compute + sensing first; they're the parts the autonomy code actually
runs on, and they're useful on a desk with zero flight risk.

- ⬜ Jetson Orin Nano 8GB ($249) — port `redwood_sim/core` to run onboard.
- ⬜ RPLIDAR A2M12 ($320) wait — combine with H2; or start with cheaper A1M8
  ($100) for bring-up, upgrade later.
- ⬜ RealSense D435i ($320).
- ⬜ Bring up sensors on the Jetson; replay the sim's perception pipeline against
  *live* sensor data. **This validates the #1 sim-to-real risk (pose/SLAM) on
  real measurements without ever leaving the bench.**

**Exit:** the autonomy stack consumes real LiDAR + depth on real silicon and
produces a map. No props spinning yet.

## Phase H2 — Airframe (~$400) — 💰

- ⬜ Cut/print the frame from `cad/` exports (carbon plates, tubes, 3D-printed
  brackets + feet).
- ⬜ Motors, ESC, props, battery, Pixhawk 6C.
- ⬜ Assemble; verify mass + CG against mass-budget.md.
- ⬜ PX4 setup, ESC calibration, **tethered** hover; confirm TWR and tune the
  attitude loop (the expected bounded sim-to-real delta).

**Exit:** stable manual hover; measured thrust/mass/CG match the spec.

## Phase H3 — Integration (~$120 + PSDB) — 💰

- ⬜ Fabricate the PSDB (JLCPCB, ~$12 + assembly).
- ⬜ Mount Jetson + sensors on the airframe; full power harness.
- ⬜ MAVLink offboard link: companion → PX4 setpoints (this is the sim's
  WebSocket bridge, in hardware).
- ⬜ Indoor autonomous flight, GPS-denied, on the drone's own estimate —
  the exact scenario `REALWORLD_READINESS.md` proves in sim.

**Exit:** the drone flies a scan autonomously and exports a reconstruction —
the simulation, made real.

## Phase H4 — Hardening — 💰

- ⬜ Vibration isolation, thermal, EMI.
- ⬜ Battery failsafe + return-to-home wired into the mission planner.
- ⬜ Global loop closure (the open software item) validated on hardware logs.
- ⬜ Flight-log → re-tune loop (system-ID hooks, `ROADMAP.md` Phase 4).

---

## Spend staging summary

| Phase | Cost | Buys | Risk retired |
|---|---:|---|---|
| H0 | $0 | time | geometry, mass, power, PCB all proven on paper |
| H1 | ~$530 | compute + sensors | **the dominant sim-to-real risk**, on the bench |
| H2 | ~$400 | airframe + propulsion | flight dynamics vs. the tuned model |
| H3 | ~$130 | PSDB + integration | full autonomous indoor scan |
| H4 | — | hardening | reliability |

Total to first autonomous scan ≈ **$1,060** (H1–H3), not the full $1,659 — the
bench-first ordering means even a partial budget makes real progress.
