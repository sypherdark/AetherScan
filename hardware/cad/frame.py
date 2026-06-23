"""
AetherScan airframe — streamlined structure (design-council revision).

A 360 mm quad-X built like a real indoor scanning drone, not a stack of plates:

  • Lofted, rounded canopy fairing (clean prop inflow, houses battery+avionics).
  • Round carbon-tube arms with motor bosses.
  • Full prop-guard rings (Flyability-class collision tolerance for indoor work).
  • A tapered central mast lifting the RPLIDAR scan plane clear above the prop
    disc — the single hard geometric requirement (REALWORLD_READINESS.md §2.1).
  • A nose pod on a short boom carrying the D435 ahead of the prop disc, tilted
    12° down (forward + floor view) so no blade enters its 87° FOV.
  • Twin curved skids that clear the belly flow/ToF sensor and the battery.

Frame: mm, Z-up, +X = nose / forward (same axes as the flight software).

Run:  uvx --from build123d python hardware/cad/frame.py
Out:  out/frame.{glb,step,stl}
"""

from __future__ import annotations

import math
from pathlib import Path

from build123d import (
    Axis,
    BuildPart,
    BuildSketch,
    Cylinder,
    Plane,
    Pos,
    Rectangle,
    Rot,
    Cone,
    export_gltf,
    export_step,
    export_stl,
    fillet,
    loft,
)

import parameters as P

F = P.FRAME
OUT = Path(__file__).parent / "out"
OUT.mkdir(exist_ok=True)

# Reference heights (origin z=0 is the arm/motor plane).
Z_CANOPY_BOT = -F.canopy_height / 2          # canopy straddles the arm plane
Z_CANOPY_TOP = Z_CANOPY_BOT + F.canopy_height
Z_MAST_TOP = Z_CANOPY_TOP + F.mast_h
ARM_RUN = P.ARM_LENGTH_MM                     # centre → motor


def canopy():
    """Tapered, rounded fairing lofted from a wide rounded base to a narrow top."""
    with BuildPart() as part:
        with BuildSketch(Plane.XY.offset(Z_CANOPY_BOT)) as s0:
            Rectangle(F.canopy_base_len, F.canopy_base_wid)
            fillet(s0.vertices(), F.canopy_fillet)
        with BuildSketch(Plane.XY.offset(Z_CANOPY_TOP)) as s1:
            Rectangle(F.canopy_top_len, F.canopy_top_wid)
            fillet(s1.vertices(), F.canopy_fillet * 0.7)
        loft()
    return part.part


def arm(mx: float, my: float):
    """Round carbon tube from the canopy out to the motor, + motor boss."""
    ang = math.degrees(math.atan2(my, mx))
    midx, midy = mx / 2, my / 2
    tube = (
        Pos(midx, midy, 0)
        * Rot(0, 0, ang)
        * Rot(0, 90, 0)
        * Cylinder(radius=F.arm_od / 2, height=ARM_RUN)
    )
    boss = Pos(mx, my, F.motor_boss_h / 2) * Cylinder(
        radius=F.motor_boss_od / 2, height=F.motor_boss_h
    )
    # M3 motor bolt circle.
    r = F.motor.mount_bolt_circle / 2
    for k in range(4):
        a = math.radians(45 + 90 * k)
        boss = boss - (
            Pos(mx + r * math.cos(a), my + r * math.sin(a), F.motor_boss_h / 2)
            * Cylinder(radius=F.motor.mount_hole / 2, height=F.motor_boss_h + 2)
        )
    return tube + boss


def prop_guard(mx: float, my: float):
    """A full ring around the prop disc + struts back to the arm."""
    z = F.prop_z
    ring = Pos(mx, my, z) * (
        Cylinder(radius=F.guard_ring_or, height=F.guard_ring_h)
        - Cylinder(radius=F.guard_ring_ir, height=F.guard_ring_h + 2)
    )
    # Two struts from the ring inner edge toward the frame centre.
    inward = math.atan2(-my, -mx)
    parts = ring
    for off in (-0.42, 0.42):
        a = inward + off
        sx = mx + F.guard_ring_ir * math.cos(a)
        sy = my + F.guard_ring_ir * math.sin(a)
        ex = mx + 0.45 * (0 - mx)
        ey = my + 0.45 * (0 - my)
        midx, midy = (sx + ex) / 2, (sy + ey) / 2
        length = math.hypot(ex - sx, ey - sy)
        sang = math.degrees(math.atan2(ey - sy, ex - sx))
        strut = (
            Pos(midx, midy, z)
            * Rot(0, 0, sang)
            * Rot(0, 90, 0)
            * Cylinder(radius=F.guard_strut_od / 2, height=length)
        )
        parts = parts + strut
    return parts


def lidar_mast():
    """Tapered streamlined post + LiDAR puck on top (scan plane above the props)."""
    mast = Pos(0, 0, Z_CANOPY_TOP + F.mast_h / 2) * Cone(
        bottom_radius=F.mast_base_r, top_radius=F.mast_top_r, height=F.mast_h
    )
    puck = Pos(0, 0, Z_MAST_TOP + F.lidar.height / 2) * Cylinder(
        radius=F.lidar.diameter / 2, height=F.lidar.height
    )
    # Scan-plane indicator slot (purely visual cue of the 360° plane).
    puck = puck - (
        Pos(0, 0, Z_MAST_TOP + F.lidar.scan_plane_offset)
        * (Cylinder(radius=F.lidar.diameter / 2 + 1, height=2)
           - Cylinder(radius=F.lidar.diameter / 2 - 3, height=4))
    )
    return mast + puck


def nose_pod():
    """Short boom + faired pod holding the D435, tilted 12° down."""
    boom = (
        Pos(F.canopy_base_len / 2 - 4 + F.nose_boom_len / 2, 0, -2)
        * Rot(0, 90, 0)
        * Cylinder(radius=F.nose_boom_od / 2, height=F.nose_boom_len)
    )
    px = F.canopy_base_len / 2 - 4 + F.nose_boom_len
    pod_center = Pos(px, 0, -2) * Rot(0, F.nose_cam_tilt_deg, 0)
    with BuildPart() as pod:
        with BuildSketch() as s:
            Rectangle(F.nose_pod_len, F.nose_pod_wid)
            fillet(s.vertices(), 8)
        from build123d import extrude
        extrude(amount=F.nose_pod_h, both=True)
    body = pod_center * pod.part
    # Lens
    lens = pod_center * Pos(F.nose_pod_len / 2, 0, 0) * Rot(0, 90, 0) * Cylinder(
        radius=9, height=8
    )
    return boom + body + lens


def skids():
    """Twin curved skids: a foot tube each side on two angled legs."""
    parts = None
    for sy in (-1, 1):
        y = sy * F.skid_span / 2
        foot = Pos(0, y, -F.skid_leg_h) * Rot(90, 0, 0) * Cylinder(
            radius=F.skid_tube_od / 2, height=F.skid_foot_len
        )
        parts = foot if parts is None else parts + foot
        for lx in (-1, 1):
            x = lx * F.skid_foot_len / 2 * 0.7
            # angled leg from canopy bottom down-out to the foot
            top = (x * 0.5, y * 0.6, Z_CANOPY_BOT + 4)
            bot = (x, y, -F.skid_leg_h)
            midx = (top[0] + bot[0]) / 2
            midy = (top[1] + bot[1]) / 2
            midz = (top[2] + bot[2]) / 2
            length = math.dist(top, bot)
            # orient cylinder (default +Z) to the leg direction
            dx, dy, dz = bot[0] - top[0], bot[1] - top[1], bot[2] - top[2]
            yaw = math.degrees(math.atan2(dy, dx))
            pitch = math.degrees(math.atan2(math.hypot(dx, dy), dz))
            leg = (
                Pos(midx, midy, midz)
                * Rot(0, 0, yaw)
                * Rot(0, pitch, 0)
                * Cylinder(radius=F.skid_tube_od / 2, height=length)
            )
            parts = parts + leg
    return parts


def airframe():
    model = canopy()
    for (mx, my) in P.motor_positions():
        model = model + arm(mx, my) + prop_guard(mx, my)
    model = model + lidar_mast() + nose_pod() + skids()
    return model


def main():
    model = airframe()
    bb = model.bounding_box()
    sx, sy, sz = bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z
    diag = math.hypot(P.WHEELBASE_DIAG_MM, P.WHEELBASE_DIAG_MM) / (2 ** 0.5)
    print(f"Airframe envelope: {sx:.0f} × {sy:.0f} × {sz:.0f} mm")
    print(f"  motor-to-motor diagonal: {P.WHEELBASE_DIAG_MM:.0f} mm (spec)")
    print(f"  LiDAR scan plane at z≈{Z_MAST_TOP + F.lidar.scan_plane_offset:.0f} mm "
          f"(prop disc z={F.prop_z:.0f} mm → clear horizon)")
    print(f"Volume: {model.volume/1000:.1f} cm³")
    export_gltf(model, str(OUT / "frame.glb"), binary=True)
    export_step(model, str(OUT / "frame.step"))
    export_stl(model, str(OUT / "frame.stl"))
    print(f"Exported → {OUT}/frame.{{glb,step,stl}}")


if __name__ == "__main__":
    main()
