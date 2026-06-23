# AetherScan — Hardware

The physical drone that the `redwood_sim` autonomy stack is designed to fly.
Everything here is **traceable to the software**: the frame is sized to the
physics model, the sensor suite is the one the real-world readiness audit
validated, and a checker (`cad/check_against_software.py`) fails CI if the two
ever drift apart.

> **Status:** Phase H0 (paper drone) — no parts bought yet. Geometry, BOM, mass,
> power, and wiring are designed and self-consistent; the airframe CAD exports
> real GLB/STEP/STL. See [ROADMAP_HARDWARE.md](ROADMAP_HARDWARE.md).

## The drone, in one table

> Airframe converged through a multidisciplinary design review —
> [design/DESIGN_COUNCIL.md](design/DESIGN_COUNCIL.md) (aero, mech, electrical,
> software + systems argue the tradeoffs) → [design/design-spec.md](design/design-spec.md).

| | |
|---|---|
| Class | 360 mm-wheelbase quad-X, 7" props, removable prop-guard rings (indoor) |
| AUW | 1.45 kg (matches `physics.py`) |
| TWR | ~3.6–4.4 : 1 |
| Endurance | ~15–17 min usable (4S 6000 mAh) |
| Compute | Jetson Orin Nano 8GB (autonomy) + Pixhawk 6C / PX4 (flight) |
| Primary sensor | Slamtec RPLIDAR A2M12 (360° 2D) |
| Also | RealSense D435i (fwd depth+IMU), Matek 3901-L0X (flow+ToF) |
| Localization | GPS-denied; onboard estimate + SLAM (as proven in sim) |
| Parts cost | ~$1,659 (bench-first staging reaches first flight at ~$1,060) |

## Layout

```
hardware/
├── README.md                  ← you are here
├── ROADMAP_HARDWARE.md        ← funding-aware build plan (H0→H4)
├── Makefile                   ← make check | cad | preview | all
├── design/
│   ├── DESIGN_COUNCIL.md      ← multidisciplinary design review 1 (the WHY)
│   ├── DESIGN_REVIEW_2.md     ← review 2: + CEO seat, live-researched, voted
│   └── design-spec.md         ← the converged airframe spec (the WHAT)
├── specs/
│   ├── system-spec.md         ← THE CONTRACT: every req traced to the software
│   ├── mass-budget.md         ← 1442 g build, inertia-matching plan
│   └── power-budget.md        ← 88.8 Wh pack, rails, endurance
├── bom/
│   └── bom.csv                ← real, currently-purchasable parts + prices
├── cad/                       ← parametric airframe (build123d)
│   ├── parameters.py          ← SINGLE SOURCE OF TRUTH for all dimensions
│   ├── check_against_software.py ← asserts parameters == redwood_sim constants
│   ├── frame.py               ← primary structure → out/frame.{glb,step,stl}
│   ├── sensor_mounts.py       ← D435 nose bracket + belly flow/ToF mount
│   ├── compute_tray.py        ← Jetson tray + soft-mounted FC pad
│   ├── assembly.py            ← full assembly + mass/inertia tensor vs physics.py
│   ├── render_preview.py      ← headless STL → PNG (no GPU needed)
│   └── out/                   ← generated (gitignored)
├── electrical/
│   ├── README.md              ← power + signal trees, PSDB requirements E1–E8
│   └── pcb/                   ← KiCad PSDB project (via kicad-happy skill)
└── datasheets/                ← (gitignored; links in bom.csv)
```

## How accordance with the software is enforced

`cad/parameters.py` declares every dimension. The ones that originate in the
flight software (mass, arm length, inertia, tilt limit, body radius) are cited to
their exact source line. `cad/check_against_software.py` parses
`redwood_sim/core/physics.py` and `redwood_sim/config.py` and asserts they still
match — so you **cannot** change the simulated drone without the hardware build
failing its check, and vice-versa.

```bash
make check     # verify hardware ⇄ software agreement
make cad       # build the airframe → out/frame.{glb,step,stl}
make parts     # build sensor brackets + compute tray
make assembly  # full assembly + mass/inertia tensor → out/assembly.*
make inertia   # just the mass/inertia numbers vs physics.py (no build123d)
make preview STL=out/assembly.stl   # render any STL → PNG
make all       # check → cad → parts → assembly → preview
```

> **Heads-up — a real finding:** `make inertia` shows the buildable drone has
> ~20–54% **less** rotational inertia than `physics.py` assumes (Izz −54%). The
> sim's Izz=0.026 is physically unreachable for a 360 mm quad; the real value is
> ~0.012. This needs a sim reconciliation + controller re-tune — see
> [specs/inertia-findings.md](specs/inertia-findings.md).

## Toolchain

- **CAD:** [build123d](https://github.com/gumyr/build123d) via `uv` (zero-setup:
  `uvx --from build123d python ...`). The build123d skill is installed at
  `~/.claude/skills/build123d`.
- **Preview render:** matplotlib + trimesh in the `redwood_sim/.venv` (headless,
  no GPU).
- **PCB:** [kicad-happy](https://github.com/aklofas/kicad-happy) (pending).

## What's done vs. next
- ✅ Contract, BOM, specs, parametric airframe, preview pipeline, electrical design.
- ⬜ Sensor brackets + compute tray CAD modules.
- ⬜ Full assembly with per-part masses → inertia-match to `physics.py`.
- ⬜ PSDB schematic + layout (kicad-happy).

See [ROADMAP_HARDWARE.md](ROADMAP_HARDWARE.md) for the staged plan.
