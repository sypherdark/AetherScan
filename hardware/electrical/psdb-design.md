# PSDB — Detailed Design (schematic-ready)

Component-level design of the Power & Sensor Distribution Board, satisfying
requirements E1–E8 in [README.md](README.md). This is the complete electrical
design; only the **PCB layout/Gerbers** wait on the KiCad tool. With
this, an engineer can capture the schematic directly.

Board: **4-layer** (Sig / GND / PWR / Sig) — the inner ground plane is the
cheapest EMI win we get. 50×50 mm, 30.5×30.5 mm M3 mounting, target ≤ 25 g.

## Power stages

| Net | Stage | Part (real) | Spec | Notes |
|---|---|---|---|---|
| VBAT | Input | XT60 + TVS **SMBJ20A** + bulk **470 µF/35 V** low-ESR + 4×10 µF MLCC | 4S 12.0–16.8 V, 60 A peak pass-through to ESC | reverse-polarity via series P-FET **CSD18540** (ideal-diode style) |
| +5V_COMP | Buck #1 | **TI TPS568230** (4.5–17 Vin, 5 V, 8 A integrated-FET sync buck) | 5.0 V @ 6 A cont (8 A peak) | Jetson Orin Nano rail — **isolated**; 2.2 µH + 2×22 µF out |
| +5V_AVI | Buck #2 | **TI LMR33630** (3.8–36 Vin, 3 A sync buck) | 5.0 V @ 3 A | FC + RPLIDAR + flow/ToF + D435 hub |
| I_SENSE | Telemetry | **TI INA226** (I²C, ±0.1 %) over a 2 mΩ shunt | pack V + total I | augments the PM02; streams to the Jetson for endurance modelling |

The Pixhawk still gets pack V/I from the **Holybro PM02** on its analog ADC (its
native battery estimator); the INA226 is the *companion-side* digital telemetry so
the autonomy can reason about remaining energy.

## EMI filtering (built into each net)
- **Each 5 V output:** ferrite bead (**BLM18 series, 600 Ω@100 MHz**) + 10 µF after
  the bead → pi filter into the load.
- **USB3 to D435 (5 Gbps):** routed on its own layer pair over solid GND, away from
  the LiDAR UART and the IMU; **common-mode choke (90 Ω@100 MHz)** on the
  differential pair; shielded cable, shield grounded one end.
- **RPLIDAR UART + flow/ToF I²C:** twisted/short, series 33 Ω + 22 pF where they
  leave the board.
- **Star ground** at the INA226 shunt return; power-stage switching loops kept tight
  and off the sensor layer.

## Connector map
| Connector | Net | To |
|---|---|---|
| XT60 | VBAT | battery |
| 2-pin 4 mm bullet | VBAT | 4-in-1 ESC |
| 6-pin JST-GH | PM02 passthrough | Pixhawk POWER1 |
| XT30 | +5V_COMP | Jetson (barrel/2-pin) |
| 4-pin JST-GH | +5V_AVI + GND | Pixhawk 5 V in |
| 4× JST-GH | +5V_AVI / UART / I²C | RPLIDAR, flow/ToF, D435 hub, spare |

## Thermal on-board
The 8 A buck at ~92 % efficiency dissipates ~3 W → copper pour + ≥9 thermal vias
under the IC; keep it on the downwash side of the canopy with the Jetson heatsink.

## Verification before fab
DC-load each rail to spec, scope ripple (< 50 mV target on +5V_COMP under a 0→5 A
step), confirm INA226 reads within 2 % of a bench meter, and bring up the D435 +
RPLIDAR simultaneously to check the USB3↔LiDAR isolation on real traffic.
