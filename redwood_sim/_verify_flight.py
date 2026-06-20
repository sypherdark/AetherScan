#!/usr/bin/env python3
"""
Verification harness for the flight-controller fixes.

Loads the apartment_1 Replica scene exactly as the engine does, starts a
mission, and ticks 2500 control steps.  Reports:
  - Z standard deviation (altitude hold quality; target < 20 mm)
  - roll/pitch stability (no flip)
  - nav-state histogram (must leave STUCK_SPIN; must accumulate coverage)
  - final coverage %
"""
from __future__ import annotations

import sys
import warnings
from collections import Counter
from pathlib import Path

warnings.filterwarnings("ignore")

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from config import SimConfig
from scene_loader import load_semantic_redwood_scene
from simulation.engine import SimulationEngine

SCENE_ID = "apartment_1"
TICKS = 2500
DT = 0.01  # 10 ms control tick
DASHBOARD_MESHES = ROOT.parent / "dashboard" / "public" / "meshes"


def main() -> None:
    loaded = load_semantic_redwood_scene(SCENE_ID, DASHBOARD_MESHES)
    if loaded is None:
        print(f"FAIL: could not load semantic scene {SCENE_ID}")
        sys.exit(1)
    scene, _ = loaded

    eng = SimulationEngine(scene, SimConfig(), headless=True, scene_id=SCENE_ID)
    eng.mission_command("start")

    z_vals: list[float] = []
    roll_vals: list[float] = []
    pitch_vals: list[float] = []
    states: Counter = Counter()
    xy_visited: set = set()

    print(f"\n{'tick':>6}  {'nav_state':>15}  {'X':>7}  {'Y':>7}  {'Z':>7}  "
          f"{'roll':>6}  {'pitch':>6}  {'cov%':>5}")
    for tick in range(TICKS):
        eng.tick(DT)
        s = eng.controller.state
        z = float(s.position[2])
        roll = float(np.degrees(s.euler[0]))
        pitch = float(np.degrees(s.euler[1]))
        z_vals.append(z)
        roll_vals.append(roll)
        pitch_vals.append(pitch)
        ns = getattr(eng.navigator, "_nav_state", None)
        ns_name = ns.name if ns is not None else "?"
        states[ns_name] += 1
        xy_visited.add((round(float(s.position[0]), 1), round(float(s.position[1]), 1)))

        if tick % 100 == 0:
            print(f"{tick:6d}  {ns_name:>15}  {s.position[0]:7.3f}  {s.position[1]:7.3f}  "
                  f"{z:7.3f}  {roll:6.1f}  {pitch:6.1f}  {eng._coverage:5.1f}")

    z_arr = np.array(z_vals)
    z_std = float(z_arr.std())
    roll_max = float(np.abs(roll_vals).max())
    pitch_max = float(np.abs(pitch_vals).max())

    print("\n── Metrics ───────────────────────────────────────────────")
    print(f"  Z std         : {z_std * 1000:.1f} mm   (target < 20)")
    print(f"  Z range       : {z_arr.min():.3f} → {z_arr.max():.3f} m")
    print(f"  |roll| max    : {roll_max:.1f}°")
    print(f"  |pitch| max   : {pitch_max:.1f}°")
    print(f"  unique XY     : {len(xy_visited)}  (0.1 m cells visited)")
    print(f"  coverage      : {eng._coverage:.1f}%")
    print(f"  nav states    : {dict(states)}")
    print("\n── Pass/Fail ─────────────────────────────────────────────")
    print(f"  altitude hold : {'PASS' if z_std < 0.020 else 'FAIL'}")
    print(f"  no flip       : {'PASS' if roll_max < 45 and pitch_max < 45 else 'FAIL'}")
    print(f"  explored      : {'PASS' if len(xy_visited) > 15 else 'FAIL'}  "
          f"(left spawn cell, visited {len(xy_visited)} cells)")
    stuck_frac = states.get("STUCK_SPIN", 0) / TICKS
    print(f"  not stuck     : {'PASS' if stuck_frac < 0.5 else 'FAIL'}  "
          f"(STUCK_SPIN {stuck_frac*100:.0f}% of ticks)")


if __name__ == "__main__":
    main()
