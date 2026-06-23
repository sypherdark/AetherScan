"""
AetherScan airframe — primary structure.

A 360 mm-wheelbase quad-X carbon frame sized to the flight-software contract
(see parameters.py). Builds the load-bearing structure only — sensor brackets
and the compute tray live in their own modules and bolt to the holes defined
here, so each piece can be printed/cut and revised independently.

Geometry, top view (X-config, +X = nose / forward = D435 look direction):

        FL  o-----.        .-----o  FR
             \     \      /     /
              \     [ stack ]  /          stack = bottom plate + standoffs + top
              /     /      \     \         plate; FC + ESC + companion mount here
        RL  o-----'        '-----o  RR
                         (RPLIDAR on a mast above; battery + flow/ToF below)

Run:  uvx --from build123d python hardware/cad/frame.py
Out:  hardware/cad/out/frame.{glb,step,stl}
"""

from __future__ import annotations

import math
from pathlib import Path

from build123d import (
    Box,
    Cylinder,
    Pos,
    Rot,
    export_gltf,
    export_step,
    export_stl,
)

import parameters as P

F = P.FRAME
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)


def _hole(d: float, h: float):
    """A through-hole cutter (slightly taller than the stock it cuts)."""
    return Cylinder(radius=d / 2, height=h + 2)


def bottom_plate():
    """Main structural plate: arms fuse into it; FC/ESC mount on top of it."""
    plate = Box(F.center_plate_len, F.center_plate_wid, F.plate_thickness)

    # Four arms emanate at 45° (quad-X). Each is a square carbon tube running
    # from the plate centre out to the motor axis at ARM_LENGTH_MM.
    arm_run = P.ARM_LENGTH_MM
    for (mx, my) in P.motor_positions():
        ang = math.degrees(math.atan2(my, mx))
        midx, midy = mx / 2, my / 2
        arm = (
            Pos(midx, midy, 0)
            * Rot(0, 0, ang)
            * Box(arm_run, F.arm_tube, F.arm_tube)
        )
        plate = plate + arm

        # Motor mount pad + bolt pattern at the tip.
        pad = Pos(mx, my, 0) * Box(
            F.motor_mount_pad, F.motor_mount_pad, F.plate_thickness
        )
        plate = plate + pad
        # Centre bore (wiring) + 4× M3 on the motor bolt circle.
        plate = plate - (Pos(mx, my, 0) * _hole(8.0, F.arm_tube))
        r = F.motor.mount_bolt_circle / 2
        for k in range(4):
            a = math.radians(45 + 90 * k)
            hx, hy = mx + r * math.cos(a), my + r * math.sin(a)
            plate = plate - (Pos(hx, hy, 0) * _hole(F.motor.mount_hole, F.arm_tube))

    # FC mounting holes (Pixhawk 6C, 76×35) centred on the plate.
    bx, by = F.fc.mount_bolt_circle_x / 2, F.fc.mount_bolt_circle_y / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            plate = plate - (
                Pos(sx * bx, sy * by, 0) * _hole(F.fc.mount_hole, F.plate_thickness)
            )

    # Corner standoff holes (link to top plate) just inside the plate corners.
    sx, sy = F.center_plate_len / 2 - 6, F.center_plate_wid / 2 - 6
    for ax in (-1, 1):
        for ay in (-1, 1):
            plate = plate - (
                Pos(ax * sx, ay * sy, 0) * _hole(F.screw_clear, F.plate_thickness)
            )
    return plate


def top_plate():
    """Upper deck: carries the companion computer and the LiDAR mast."""
    z = F.plate_gap + F.plate_thickness
    plate = Pos(0, 0, z) * Box(
        F.center_plate_len, F.center_plate_wid, F.plate_thickness
    )

    # Jetson Orin Nano mount (86×58, M2.5).
    bx, by = F.companion.mount_bolt_circle_x / 2, F.companion.mount_bolt_circle_y / 2
    for ax in (-1, 1):
        for ay in (-1, 1):
            plate = plate - (
                Pos(ax * bx, ay * by, z)
                * _hole(F.companion.mount_hole, F.plate_thickness)
            )

    # Mast holes (3× on the LiDAR bolt circle, centred → clear 360° horizon).
    r = F.lidar.mount_bolt_circle / 2
    for k in range(3):
        a = math.radians(90 + 120 * k)
        plate = plate - (
            Pos(r * math.cos(a), r * math.sin(a), z)
            * _hole(F.screw_clear, F.plate_thickness)
        )

    # Corner standoff holes (match bottom plate).
    sx, sy = F.center_plate_len / 2 - 6, F.center_plate_wid / 2 - 6
    for ax in (-1, 1):
        for ay in (-1, 1):
            plate = plate - (
                Pos(ax * sx, ay * sy, z) * _hole(F.screw_clear, F.plate_thickness)
            )
    return plate


def standoffs():
    """Four corner standoffs separating the two plates."""
    sx, sy = F.center_plate_len / 2 - 6, F.center_plate_wid / 2 - 6
    z = F.plate_thickness / 2
    parts = None
    for ax in (-1, 1):
        for ay in (-1, 1):
            s = Pos(ax * sx, ay * sy, z + F.plate_gap / 2) * Cylinder(
                radius=F.standoff_od / 2, height=F.plate_gap
            )
            parts = s if parts is None else parts + s
    return parts


def landing_gear():
    """Four legs below the bottom plate — clears the battery and the downward
    optical-flow/ToF cone (parameters.Frame.leg_height)."""
    sx, sy = F.center_plate_len / 2 - 10, F.center_plate_wid / 2 - 10
    parts = None
    for ax in (-1, 1):
        for ay in (-1, 1):
            leg = Pos(ax * sx, ay * sy, -F.leg_height / 2) * Cylinder(
                radius=F.leg_od / 2, height=F.leg_height
            )
            parts = leg if parts is None else parts + leg
    # Skid feet
    foot_z = -F.leg_height
    for ay in (-1, 1):
        skid = Pos(0, ay * sy, foot_z) * Box(2 * sx + F.leg_od, F.leg_od, F.leg_od)
        parts = parts + skid
    return parts


def lidar_mast():
    """Three posts raising the RPLIDAR A2M12 above the prop disc so its 360°
    scan plane has an unobstructed horizon — the single most important
    geometric requirement from REALWORLD_READINESS.md §2.1."""
    top_z = F.plate_gap + 2 * F.plate_thickness
    r = F.lidar.mount_bolt_circle / 2
    parts = None
    for k in range(3):
        a = math.radians(90 + 120 * k)
        post = Pos(r * math.cos(a), r * math.sin(a), top_z + F.mast_height / 2) * Cylinder(
            radius=F.mast_od / 2, height=F.mast_height
        )
        parts = post if parts is None else parts + post
    # LiDAR deck
    deck_z = top_z + F.mast_height
    deck = Pos(0, 0, deck_z) * Cylinder(radius=r + 8, height=F.plate_thickness)
    for k in range(3):
        a = math.radians(90 + 120 * k)
        deck = deck - (
            Pos(r * math.cos(a), r * math.sin(a), deck_z)
            * _hole(F.screw_clear, F.plate_thickness)
        )
    return parts + deck


def airframe():
    return (
        bottom_plate()
        + top_plate()
        + standoffs()
        + landing_gear()
        + lidar_mast()
    )


def main():
    model = airframe()
    bb = model.bounding_box()
    sx = bb.max.X - bb.min.X
    sy = bb.max.Y - bb.min.Y
    sz = bb.max.Z - bb.min.Z
    diag = math.hypot(sx, sy)  # motor-to-motor diagonal = the wheelbase spec
    print(f"Airframe bounding box: {sx:.1f} × {sy:.1f} × {sz:.1f} mm")
    print(f"  motor-to-motor diagonal ≈ {diag - F.motor_mount_pad:.0f} mm "
          f"(spec wheelbase {P.WHEELBASE_DIAG_MM:.0f} mm)")
    print(f"Volume: {model.volume / 1000:.1f} cm³")

    export_gltf(model, str(OUT / "frame.glb"), binary=True)
    export_step(model, str(OUT / "frame.step"))
    export_stl(model, str(OUT / "frame.stl"))
    print(f"Exported → {OUT}/frame.{{glb,step,stl}}")


if __name__ == "__main__":
    main()
