"""
AetherScan airframe — single source of truth for every dimension.

Every CAD module imports from here. Nothing downstream hardcodes a number.
The values that originate in the FLIGHT SOFTWARE are cited to their source file
and line so the hardware can never silently drift from the simulation it was
tuned against.  `check_against_software.py` parses those source files and
asserts the values below still match — run it in CI before trusting any export.

Units: millimetres and grams everywhere in CAD (build123d default is mm).
The software works in metres/kg; conversions are explicit and labelled.
"""

from __future__ import annotations

from dataclasses import dataclass, field


# ─────────────────────────────────────────────────────────────────────────────
# 1. THE CONTRACT — values that come straight from the flight software.
#    DO NOT edit these to fit a part. If a part forces a change, change the
#    software constant first, re-tune, then update here. (That is the whole
#    point of check_against_software.py.)
# ─────────────────────────────────────────────────────────────────────────────

# redwood_sim/core/physics.py : QuadcopterParams
MASS_KG: float = 1.45            # physics.py:28  mass
ARM_LENGTH_M: float = 0.18       # physics.py:33  arm_length (centre → motor axis)
IXX: float = 0.014               # physics.py:30
IYY: float = 0.014               # physics.py:31
IZZ: float = 0.026               # physics.py:32
MAX_TILT_RAD: float = 0.48       # physics.py:39  (≈27.5°) — control/struct envelope

# redwood_sim/config.py : SimConfig
BODY_RADIUS_M: float = 0.18      # config.py:15  drone_body_radius (collision proxy)

# Derived flight numbers (not edited — computed from the contract)
ARM_LENGTH_MM: float = ARM_LENGTH_M * 1000.0          # 180 mm
WHEELBASE_DIAG_MM: float = 2 * ARM_LENGTH_MM          # 360 mm (motor-to-motor, X)
MASS_BUDGET_G: float = MASS_KG * 1000.0               # 1450 g all-up-weight target


# ─────────────────────────────────────────────────────────────────────────────
# 2. SELECTED REAL COMPONENTS — dimensions taken from manufacturer datasheets.
#    Part numbers live in hardware/bom/bom.csv. If you swap a part, update the
#    dataclass here and the BOM row together.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Motor:
    """iFlight XING2 2806.5 — 28 mm stator, M3 mounting, Ø19 mm bolt circle."""
    name: str = "iFlight XING2 2806.5 1300KV"
    boss_diameter: float = 28.0      # stator OD the mount cups
    height: float = 25.0
    mount_bolt_circle: float = 19.0  # M3 holes on Ø19 (16/19 pattern → use 19)
    mount_hole: float = 3.2          # M3 clearance
    mass_g: float = 35.0
    prop_inch: float = 7.0           # HQProp 7x4x3 — 178 mm dia


@dataclass(frozen=True)
class Esc:
    """Holybro Tekko32 F4 Metal 4-in-1 65A — single stacked board."""
    name: str = "Holybro Tekko32 F4 4in1 65A"
    length: float = 41.0
    width: float = 41.0
    height: float = 8.0
    mount_bolt_circle: float = 30.5  # 30.5×30.5 standard stack
    mount_hole: float = 3.2          # M3 (use soft grommets)
    mass_g: float = 35.0


@dataclass(frozen=True)
class FlightController:
    """Holybro Pixhawk 6C — PX4 autopilot. MAVLink offboard to the companion."""
    name: str = "Holybro Pixhawk 6C"
    length: float = 84.8
    width: float = 44.0
    height: float = 17.0
    mount_bolt_circle_x: float = 76.0
    mount_bolt_circle_y: float = 35.0
    mount_hole: float = 3.2
    mass_g: float = 34.6


@dataclass(frozen=True)
class Companion:
    """NVIDIA Jetson Orin Nano 8GB dev kit — runs the ported autonomy stack."""
    name: str = "NVIDIA Jetson Orin Nano 8GB"
    length: float = 100.0            # carrier board
    width: float = 79.0
    height: float = 30.0             # with heatsink+fan
    mount_bolt_circle_x: float = 86.0
    mount_bolt_circle_y: float = 58.0
    mount_hole: float = 2.6          # M2.5
    mass_g: float = 150.0            # board + heatsink + fan


@dataclass(frozen=True)
class Lidar:
    """Slamtec RPLIDAR A2M12 — 360° 2D, 0.15–12 m. THE validated sensor
    (REALWORLD_READINESS.md §2.1). Must sit ABOVE the prop disc with a clear
    360° horizon — drives the top-mast height."""
    name: str = "Slamtec RPLIDAR A2M12"
    diameter: float = 76.0
    height: float = 41.0
    mount_bolt_circle: float = 76.0  # 3× M3 on the base
    mount_hole: float = 3.2
    mass_g: float = 190.0
    scan_plane_offset: float = 30.0  # scan plane height above its own base


@dataclass(frozen=True)
class DepthCamera:
    """Intel RealSense D435i — forward depth + IMU. 87° HFOV cone must be
    unobstructed by frame/guards ahead of the nose."""
    name: str = "Intel RealSense D435i"
    length: float = 90.0
    width: float = 25.0
    height: float = 25.0
    mount_hole: float = 2.0          # 2× M3 on D435 is actually 1/4-20 + M3; use M3 bracket
    mass_g: float = 72.0
    hfov_deg: float = 87.0


@dataclass(frozen=True)
class FlowTof:
    """Matek 3901-L0X — PMW3901 optical flow + VL53L1X ToF, downward.
    Needs a clear nadir view (no battery/leg in the cone)."""
    name: str = "Matek Optical Flow 3901-L0X"
    length: float = 21.0
    width: float = 16.0
    height: float = 6.0
    mount_bolt_circle: float = 20.0
    mount_hole: float = 2.2          # M2
    mass_g: float = 10.0


@dataclass(frozen=True)
class Battery:
    """4S LiPo 6000 mAh — the single biggest mass lever in the budget.
    Sized so the all-up weight lands on MASS_BUDGET_G (see specs/mass-budget.md)."""
    name: str = "Tattu R-Line 4S 6000mAh"
    length: float = 138.0
    width: float = 43.0
    height: float = 35.0
    mass_g: float = 520.0
    cells: int = 4
    capacity_mah: int = 6000


# ─────────────────────────────────────────────────────────────────────────────
# 3. FRAME GEOMETRY — our design choices, constrained by sections 1 & 2.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Frame:
    # Plate stack
    plate_thickness: float = 2.0         # 2 mm carbon
    plate_gap: float = 30.0              # standoff height between bottom & top plate
    center_plate_len: float = 110.0
    center_plate_wid: float = 90.0
    corner_fillet: float = 6.0

    # Arms (square carbon tube, motor at the far end at ARM_LENGTH_MM)
    arm_tube: float = 12.0               # 12×12 mm carbon tube
    arm_wall: float = 1.5
    motor_mount_pad: float = 30.0        # square pad at the arm tip under the motor

    # Standoff / fastener system (whole airframe uses ONE screw size: M3)
    standoff_od: float = 6.0
    screw: float = 3.0
    screw_clear: float = 3.2

    # Landing gear
    leg_height: float = 70.0             # clears the downward ToF cone + battery
    leg_od: float = 10.0

    # Top sensor mast (raises RPLIDAR above the prop disc for clear 360° horizon)
    mast_height: float = 60.0
    mast_od: float = 8.0

    # ── Streamlined geometry (design-council revision, see hardware/design/) ──────
    # Lofted canopy: a tapered shell instead of two flat plates. Bottom carries
    # the battery + avionics; top tapers to the mast. Rounded for clean airflow
    # and a real fairing.
    canopy_base_len: float = 132.0
    canopy_base_wid: float = 96.0
    canopy_top_len: float = 78.0
    canopy_top_wid: float = 54.0
    canopy_height: float = 46.0
    canopy_fillet: float = 18.0

    # Round carbon tube arms (replace square tube — lower drag, standard clamps).
    arm_od: float = 16.0
    motor_boss_od: float = 30.0
    motor_boss_h: float = 9.0
    prop_z: float = 26.0                 # prop-disc height above origin

    # Prop guard rings (indoor safety + collision tolerance, Flyability-class).
    # Inner radius clears the 7" disc (89 mm) with margin; thin rim.
    guard_ring_or: float = 100.0
    guard_ring_ir: float = 94.0
    guard_ring_h: float = 12.0
    guard_strut_od: float = 5.0

    # Streamlined LiDAR mast (tapered frustum). Height chosen so the RPLIDAR scan
    # plane sits ABOVE the prop disc for an unobstructed 360° horizon.
    mast_base_r: float = 17.0
    mast_top_r: float = 9.0
    mast_h: float = 82.0

    # Nose camera pod on a short boom (puts the D435 ahead of the prop disc so the
    # 87° FOV never sees a blade; tilted down for combined forward/floor view).
    nose_boom_len: float = 44.0
    nose_boom_od: float = 12.0
    nose_pod_len: float = 30.0
    nose_pod_wid: float = 34.0
    nose_pod_h: float = 28.0
    nose_cam_tilt_deg: float = 12.0

    # Skid landing gear (twin curved skids — stiff, light, clears belly sensor).
    skid_tube_od: float = 9.0
    skid_leg_h: float = 74.0
    skid_span: float = 124.0
    skid_foot_len: float = 158.0

    motor: Motor = field(default_factory=Motor)
    esc: Esc = field(default_factory=Esc)
    fc: FlightController = field(default_factory=FlightController)
    companion: Companion = field(default_factory=Companion)
    lidar: Lidar = field(default_factory=Lidar)
    depth: DepthCamera = field(default_factory=DepthCamera)
    flow: FlowTof = field(default_factory=FlowTof)
    battery: Battery = field(default_factory=Battery)


FRAME = Frame()


# Convenience: the four motor positions in the X-configuration (top view, mm).
# Front-right, front-left, rear-left, rear-right — matches a standard PX4 quad-X.
def motor_positions() -> list[tuple[float, float]]:
    d = ARM_LENGTH_MM / (2 ** 0.5)   # X-arm: each motor is L away along the diagonal
    return [(+d, +d), (-d, +d), (-d, -d), (+d, -d)]


if __name__ == "__main__":
    print(f"AUW target          : {MASS_BUDGET_G:.0f} g")
    print(f"Arm length          : {ARM_LENGTH_MM:.0f} mm  (diag wheelbase {WHEELBASE_DIAG_MM:.0f} mm)")
    print(f"Motor positions (mm): {[(round(x,1), round(y,1)) for x, y in motor_positions()]}")
    print(f"Prop                 : {FRAME.motor.prop_inch}\"  (Ø{FRAME.motor.prop_inch*25.4:.0f} mm)")
