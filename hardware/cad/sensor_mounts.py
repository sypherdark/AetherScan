"""
Sensor brackets — each bolts to holes already defined by the frame, so it can be
printed and revised on its own.

  • D435 nose bracket  — holds the RealSense D435i at the +X nose, look axis
                          along +X (= the software's forward/heading axis), with
                          its 87° HFOV cone clear of the frame and prop guards.
  • Belly flow mount   — holds the Matek 3901-L0X looking straight down (−Z) at
                          the centre of the belly, nadir cone clear of battery.

Run:  uvx --from build123d python hardware/cad/sensor_mounts.py
Out:  out/d435_bracket.{glb,stl}, out/flow_mount.{glb,stl}
"""

from __future__ import annotations

from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, export_gltf, export_stl

import parameters as P

F = P.FRAME
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

# Nose offset: forward of the front arms, clear of the 7" disc + guard.
NOSE_X = P.ARM_LENGTH_MM + 20.0          # 200 mm forward of centre
NOSE_Z = F.plate_thickness / 2           # at the bottom-plate level


def d435_bracket():
    """An L-bracket: a vertical face the D435 screws to, on a foot that bolts to
    the bottom plate. Camera sits at NOSE_X looking +X."""
    wall = 3.0
    face_w, face_h = 40.0, 30.0           # vertical mounting face for the camera
    foot_l = 30.0

    # Vertical face at the nose, normal along +X.
    face = Pos(NOSE_X, 0, NOSE_Z + face_h / 2) * Box(wall, face_w, face_h)
    # 2× M3 holes for the D435 (it has M3 + 1/4-20; use the two M3 at 45 mm... here 26 mm apart)
    for sy in (-1, 1):
        face = face - (
            Pos(NOSE_X, sy * 13, NOSE_Z + face_h / 2) * Rot(0, 90, 0)
            * Cylinder(radius=F.depth.mount_hole / 2 + 0.1, height=wall + 2)
        )
    # Horizontal foot back to the plate.
    foot = Pos(NOSE_X - foot_l / 2, 0, NOSE_Z) * Box(foot_l, face_w, wall)
    for sy in (-1, 1):
        foot = foot - (
            Pos(NOSE_X - foot_l + 6, sy * 14, NOSE_Z)
            * Cylinder(radius=F.screw_clear / 2, height=wall + 2)
        )
    # Gusset so the nose load doesn't fold the L.
    gusset = Pos(NOSE_X - 8, 0, NOSE_Z + 8) * Rot(0, 45, 0) * Box(wall, face_w, 16)
    return face + foot + gusset


def flow_mount():
    """A small plate under the belly centre that holds the 3901-L0X facing −Z.
    Stands off below the battery line so the nadir view is unobstructed."""
    plate = 24.0
    drop = 12.0                            # below the bottom plate
    z = -drop
    body = Pos(0, 0, z) * Box(plate, plate, 3.0)
    # Sensor mounting holes (M2 on the module's 20 mm pattern).
    r = F.flow.mount_bolt_circle / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            body = body - (
                Pos(sx * r / 1.414, sy * r / 1.414, z)
                * Cylinder(radius=F.flow.mount_hole / 2, height=5)
            )
    # Sensor aperture (clear nadir cone).
    body = body - (Pos(0, 0, z) * Cylinder(radius=6, height=5))
    # Two posts up to the bottom plate.
    posts = None
    for sx in (-1, 1):
        post = Pos(sx * 9, 0, z / 2) * Cylinder(radius=F.standoff_od / 2, height=drop)
        posts = post if posts is None else posts + post
    return body + posts


def main():
    d = d435_bracket()
    f = flow_mount()
    export_gltf(d, str(OUT / "d435_bracket.glb"), binary=True)
    export_stl(d, str(OUT / "d435_bracket.stl"))
    export_gltf(f, str(OUT / "flow_mount.glb"), binary=True)
    export_stl(f, str(OUT / "flow_mount.stl"))
    print(f"D435 bracket vol {d.volume/1000:.1f} cm³ | flow mount vol {f.volume/1000:.1f} cm³")
    print(f"Exported → {OUT}/d435_bracket.* , flow_mount.*")


if __name__ == "__main__":
    main()
