"""
Full airframe assembly + rigid-body mass properties.

This is where "hardware in accordance with software" gets quantitative. Every
real component is placed at its mount location with its datasheet mass, and we
compute the assembled centre of gravity and inertia tensor — then compare against
the values the flight controller was tuned to (physics.py Ixx/Iyy/Izz).

Total mass matching is easy; the *inertia* is the distribution, and it tells us
whether the simulated drone and the buildable drone are actually the same rigid
body. If they diverge, that's a real finding to reconcile — not something to
paper over.

Each component is a uniform box: own inertia (1/12·m·(b²+c²)) + parallel-axis to
the system CG. Coordinates: mm, Z-up, +X forward (same frame as the software).

Run:  uvx --from build123d python hardware/cad/assembly.py        (builds + renders proxies)
  or: <sim-venv>/python hardware/cad/assembly.py --inertia-only   (numbers only, no build123d)
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import parameters as P

F = P.FRAME
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)


@dataclass
class Part:
    name: str
    mass_g: float
    pos_mm: tuple[float, float, float]      # centroid, frame coords
    dims_mm: tuple[float, float, float]      # bounding box (for own inertia)


# Layout anchors (match frame.py's streamlined geometry).
_Z_CANOPY_BOT = -F.canopy_height / 2
_Z_CANOPY_TOP = _Z_CANOPY_BOT + F.canopy_height
_Z_MAST_TOP = _Z_CANOPY_TOP + F.mast_h
_Z_LIDAR = _Z_MAST_TOP + F.lidar.height / 2
_NOSE_X = F.canopy_base_len / 2 - 4 + F.nose_boom_len

d = P.ARM_LENGTH_MM / (2 ** 0.5)                        # motor x/y offset (127.3)

COMPONENTS: list[Part] = [
    # Heavy three — these dominate CG and inertia.
    # Battery low in the canopy (low CG), shifted aft to offset the nose camera.
    Part("battery",   F.battery.mass_g, (-20, 0, _Z_CANOPY_BOT + F.battery.height / 2 + 2),
         (F.battery.length, F.battery.width, F.battery.height)),
    # LiDAR high on the mast — also the dominant Ixx/Iyy contributor.
    Part("lidar",     F.lidar.mass_g,   (0, 0, _Z_LIDAR),
         (F.lidar.diameter, F.lidar.diameter, F.lidar.height)),
    Part("jetson",    F.companion.mass_g, (0, 0, _Z_CANOPY_TOP - F.companion.height / 2 - 2),
         (F.companion.length, F.companion.width, F.companion.height)),
    # Avionics in the canopy core.
    Part("fc",        F.fc.mass_g,  (0, 0, 2), (F.fc.length, F.fc.width, F.fc.height)),
    Part("esc",       F.esc.mass_g, (0, 0, -8), (F.esc.length, F.esc.width, F.esc.height)),
    # Forward depth camera in the nose pod (tilted; treat as box).
    Part("d435",      F.depth.mass_g, (_NOSE_X, 0, -2),
         (25, F.depth.length, F.depth.height)),
    Part("flow_tof",  F.flow.mass_g, (0, 0, _Z_CANOPY_BOT - 10),
         (F.flow.length, F.flow.width, F.flow.height)),
    # Propulsion at the arm tips.
    *[Part(f"motor_{i}", F.motor.mass_g, (sx * d, sy * d, 6),
           (F.motor.boss_diameter, F.motor.boss_diameter, F.motor.height))
      for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (-1, -1), (1, -1)])],
    *[Part(f"prop_{i}", F.motor.mass_g * 0.14, (sx * d, sy * d, F.prop_z),
           (F.motor.prop_inch * 25.4, 12, 3))
      for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (-1, -1), (1, -1)])],
    # Frame structure (~180 g budget), modelled as proxies at their real spots.
    Part("canopy", 65, (0, 0, 0), (F.canopy_base_len, F.canopy_base_wid, F.canopy_height)),
    *[Part(f"arm_{i}", 15, (sx * d / 2, sy * d / 2, 0), (P.ARM_LENGTH_MM, F.arm_od, F.arm_od))
      for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (-1, -1), (1, -1)])],
    Part("mast", 16, (0, 0, _Z_CANOPY_TOP + F.mast_h / 2), (2 * F.mast_base_r, 2 * F.mast_base_r, F.mast_h)),
    Part("skids", 32, (0, 0, -F.skid_leg_h / 2), (F.skid_foot_len, F.skid_span, F.skid_leg_h)),
    # Power electronics + harness.
    Part("power_module", 36, (0, 0, -2), (40, 25, 15)),
    Part("bec",          12, (0, 0, 4), (25, 15, 10)),
    Part("psdb",         25, (0, 0, -2), (50, 50, 10)),
    Part("wiring",       27, (0, 0, 0), (90, 90, 20)),
    # Prop guard rings — outboard at the motors, adding rotational inertia.
    *[Part(f"guard_{i}", 16, (sx * d, sy * d, F.prop_z),
           (2 * F.guard_ring_or, 2 * F.guard_ring_or, F.guard_ring_h))
      for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (-1, -1), (1, -1)])],
]


def mass_properties():
    import numpy as np

    masses = np.array([p.mass_g for p in COMPONENTS]) / 1000.0       # kg
    pos = np.array([p.pos_mm for p in COMPONENTS]) / 1000.0          # m
    dims = np.array([p.dims_mm for p in COMPONENTS]) / 1000.0        # m

    M = masses.sum()
    cg = (masses[:, None] * pos).sum(axis=0) / M

    I = np.zeros((3, 3))
    for m, r, (a, b, c) in zip(masses, pos, dims):
        # own inertia of a uniform box about its centroid
        Io = m / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])
        # parallel axis to system CG
        dr = r - cg
        Ipa = m * (np.dot(dr, dr) * np.eye(3) - np.outer(dr, dr))
        I += Io + Ipa
    return M, cg, I


def report():
    import numpy as np

    M, cg, I = mass_properties()
    targets = {"Ixx": P.IXX, "Iyy": P.IYY, "Izz": P.IZZ}
    got = {"Ixx": I[0, 0], "Iyy": I[1, 1], "Izz": I[2, 2]}

    print(f"Total mass : {M*1000:7.1f} g   (target {P.MASS_BUDGET_G:.0f} g)")
    print(f"CG (mm)    : x={cg[0]*1000:+6.1f}  y={cg[1]*1000:+6.1f}  z={cg[2]*1000:+6.1f}")
    print(f"             (x,y near 0 = balanced; z is the CG height above origin)")
    print("\nInertia about CG (kg·m²):")
    print(f"  {'axis':5s} {'computed':>10s} {'target':>9s} {'error':>8s}")
    for k in ("Ixx", "Iyy", "Izz"):
        err = (got[k] - targets[k]) / targets[k] * 100
        print(f"  {k:5s} {got[k]:10.4f} {targets[k]:9.4f} {err:+7.0f}%")

    print("\nInterpretation:")
    for k in ("Ixx", "Iyy", "Izz"):
        err = (got[k] - targets[k]) / targets[k] * 100
        if abs(err) <= 20:
            print(f"  {k}: within ±20% — attitude gains transfer with at most a small re-tune.")
        else:
            lever = "raise the LiDAR mast / move mass outboard" if k != "Izz" \
                else "the sim's Izz looks high for a 360 mm quad — consider reconciling it down"
            print(f"  {k}: {err:+.0f}% — {lever}.")
    return M, cg, I


def build_visual():
    """Frame + translucent component proxies, for a sanity render."""
    from build123d import Box, Pos, export_gltf, export_stl
    import frame as frame_mod

    model = frame_mod.airframe()
    for p in COMPONENTS:
        if p.name.startswith(("canopy", "arm", "mast", "skids", "guard")):
            continue  # already real geometry in the frame
        x, y, z = p.pos_mm
        a, b, c = p.dims_mm
        model = model + (Pos(x, y, z) * Box(a, b, c))
    export_gltf(model, str(OUT / "assembly.glb"), binary=True)
    export_stl(model, str(OUT / "assembly.stl"))
    bb = model.bounding_box()
    print(f"\nAssembly envelope: "
          f"{bb.max.X-bb.min.X:.0f} × {bb.max.Y-bb.min.Y:.0f} × {bb.max.Z-bb.min.Z:.0f} mm")
    print(f"Exported → {OUT}/assembly.*")


if __name__ == "__main__":
    report()
    if "--inertia-only" not in sys.argv:
        build_visual()
