# Mass Budget

**Target AUW: 1450 g** (`physics.py:28` `mass = 1.45`). The simulation's thrust,
power, and inertia were all tuned to this number — landing within a few percent
keeps the real drone inside the regime we validated.

| Component | Part | Mass (g) |
|---|---|---:|
| 4× motor | iFlight XING2 2806.5 | 140 |
| 4× propeller | HQProp 7x4x3 | 20 |
| 4-in-1 ESC | Holybro Tekko32 F4 65A | 35 |
| Battery | Tattu R-Line 4S 6000 mAh | 520 |
| Power module | Holybro PM02 | 36 |
| 5V BEC | Mateksys BEC12S-PRO | 12 |
| Custom PSDB | AetherScan board | 25 |
| Autopilot | Pixhawk 6C | 35 |
| Companion | Jetson Orin Nano + heatsink | 150 |
| 360 LiDAR | RPLIDAR A2M12 | 190 |
| Depth camera | RealSense D435i | 72 |
| Flow + ToF | Matek 3901-L0X | 10 |
| Frame (plates+tubes+legs+mast) | carbon + print | 180 |
| Prop guards | 7" ducted set | 40 |
| Fasteners + wiring | M3 + harness | 27 |
| **Total** | | **1442 g** |

**Margin to target: +8 g (0.6%).** Within tolerance. The battery is the primary
tuning lever — a 4S 5200 mAh (~440 g) drops AUW to ~1362 g if a lighter build is
wanted; a 4S 8000 mAh (~680 g) pushes to ~1602 g (recheck TWR and re-tune).

## Inertia — the harder target

Hitting 1450 g total is easy; hitting **Ixx=Iyy=0.014, Izz=0.026 kg·m²**
(`physics.py:30-32`) requires the right *distribution*. Quick sanity check:
radius of gyration `√(0.014/1.45) ≈ 0.098 m` ≈ 100 mm — consistent with a 360 mm
frame whose heavy items (battery, Jetson, LiDAR) sit within ~100 mm of centre.

**Action:** once brackets are modelled, assign each part its real mass in the CAD
assembly and compute the inertia tensor (build123d / trimesh both expose it).
Shift the battery tray fore/aft and the LiDAR mast height to converge on the
targets. Until then, the attitude-loop gains may need a small re-tune on first
flight — that's the expected sim-to-real delta, and it's bounded because the
*total* mass and geometry already match.

> Tracking note: this is the inertia item in system-spec.md §"Open items".
