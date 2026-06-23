"""
Compute tray — carries the Jetson Orin Nano (top deck) and gives the Pixhawk 6C a
soft-mounted pad (bottom deck). Bolts to the plate hole patterns from frame.py.

Run:  uvx --from build123d python hardware/cad/compute_tray.py
Out:  out/compute_tray.{glb,stl}
"""

from __future__ import annotations

from pathlib import Path

from build123d import Box, Cylinder, Pos, export_gltf, export_stl

import parameters as P

F = P.FRAME
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)


def jetson_tray():
    """A plate sized to the Jetson carrier, with its M2.5 pattern and lightening
    cut-outs, raised on standoffs above the top plate for airflow under the board."""
    z = F.plate_gap + 2 * F.plate_thickness + 6  # 6 mm air gap above top plate
    plate = Pos(0, 0, z) * Box(
        F.companion.length + 8, F.companion.width + 8, F.plate_thickness
    )
    # Jetson mount holes (M2.5, 86×58).
    bx, by = F.companion.mount_bolt_circle_x / 2, F.companion.mount_bolt_circle_y / 2
    for ax in (-1, 1):
        for ay in (-1, 1):
            plate = plate - (
                Pos(ax * bx, ay * by, z)
                * Cylinder(radius=F.companion.mount_hole / 2, height=5)
            )
    # Airflow / weight cut-outs.
    for ax in (-1, 1):
        plate = plate - (Pos(ax * 28, 0, z) * Cylinder(radius=12, height=5))
    # Standoffs down to the top plate.
    posts = None
    for ax in (-1, 1):
        for ay in (-1, 1):
            post = Pos(ax * bx, ay * by, z - 3) * Cylinder(
                radius=F.standoff_od / 2, height=6
            )
            posts = post if posts is None else posts + post
    return plate + posts


def fc_pad():
    """Soft-mount pad for the Pixhawk on the bottom plate (grommet holes).
    Vibration isolation here directly protects the estimator the whole REALWORLD
    audit is about."""
    z = F.plate_thickness + 4
    pad = Pos(0, 0, z) * Box(F.fc.length + 6, F.fc.width + 6, F.plate_thickness)
    bx, by = F.fc.mount_bolt_circle_x / 2, F.fc.mount_bolt_circle_y / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            pad = pad - (
                Pos(sx * bx, sy * by, z)
                * Cylinder(radius=F.fc.mount_hole / 2, height=5)
            )
    # Four grommet posts.
    posts = None
    for sx in (-1, 1):
        for sy in (-1, 1):
            post = Pos(sx * bx, sy * by, z - 2) * Cylinder(radius=4, height=4)
            posts = post if posts is None else posts + post
    return pad + posts


def main():
    j = jetson_tray()
    p = fc_pad()
    asm = j + p
    export_gltf(asm, str(OUT / "compute_tray.glb"), binary=True)
    export_stl(asm, str(OUT / "compute_tray.stl"))
    print(f"Compute tray vol {asm.volume/1000:.1f} cm³")
    print(f"Exported → {OUT}/compute_tray.*")


if __name__ == "__main__":
    main()
