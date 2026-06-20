#!/usr/bin/env python3
"""
AetherScan — Synthetic Indoor Environment Generator
=====================================================
Builds a production-grade, semantically-annotated indoor environment for drone
navigation testing.  Outputs:

  dashboard/public/meshes/apartment_collision.ply   — physics/collision mesh (Z-up ROS)
  dashboard/public/meshes/apartment_collision_labels.npy — per-triangle semantic labels
  dashboard/public/meshes/apartment.ply             — visual mesh with vertex colours
  dashboard/public/meshes/apartment.glb             — PBR visual mesh (GLB)

Design principles (mirrors Meta Replica):
  • Every triangle is explicitly labelled: WALL / FLOOR / CEILING / OBJECT
  • Real-world dimensions (metres)
  • Object-level semantic grouping (distinct furniture pieces)
  • Z-up ROS coordinate frame, floor at z = 0
  • Storage target: < 80 MB total

Usage:
  redwood_sim/.venv/bin/python scripts/build_synthetic_environment.py

Requirements: numpy, open3d, trimesh  (all present in the venv)
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REDWOOD = ROOT / "redwood_sim"
MESHES = ROOT / "dashboard" / "public" / "meshes"
sys.path.insert(0, str(REDWOOD))

from core.semantic_space import SemanticClass  # noqa: E402

import trimesh
import trimesh.creation as tc

# ── Semantic colour palette (vertex colours for the PLY visual) ───────────────
SEG_COLORS = {
    SemanticClass.FLOOR:   np.array([180, 140, 100, 255], dtype=np.uint8),   # oak wood
    SemanticClass.CEILING: np.array([240, 238, 232, 255], dtype=np.uint8),   # off-white
    SemanticClass.WALL:    np.array([210, 205, 195, 255], dtype=np.uint8),   # plaster
    SemanticClass.OBJECT:  np.array([120,  90,  60, 255], dtype=np.uint8),   # walnut
    SemanticClass.UNKNOWN: np.array([ 80,  80,  80, 255], dtype=np.uint8),
}

# Material colours for GLB (RGB 0-1 float, PBR)
MAT_PROPS = {
    "floor":     dict(color=[0.68, 0.52, 0.35, 1.0], roughness=0.60, metalness=0.02),
    "ceiling":   dict(color=[0.94, 0.93, 0.90, 1.0], roughness=0.92, metalness=0.00),
    "wall":      dict(color=[0.82, 0.80, 0.75, 1.0], roughness=0.88, metalness=0.00),
    "wood":      dict(color=[0.42, 0.28, 0.16, 1.0], roughness=0.65, metalness=0.02),
    "fabric":    dict(color=[0.35, 0.42, 0.52, 1.0], roughness=0.90, metalness=0.00),
    "metal":     dict(color=[0.72, 0.74, 0.76, 1.0], roughness=0.28, metalness=0.82),
    "white":     dict(color=[0.96, 0.96, 0.96, 1.0], roughness=0.80, metalness=0.00),
    "dark_wood": dict(color=[0.25, 0.16, 0.08, 1.0], roughness=0.60, metalness=0.02),
    "ceramic":   dict(color=[0.90, 0.88, 0.86, 1.0], roughness=0.30, metalness=0.02),
    "glass":     dict(color=[0.65, 0.80, 0.85, 0.4], roughness=0.05, metalness=0.10),
    "leather":   dict(color=[0.18, 0.12, 0.08, 1.0], roughness=0.70, metalness=0.00),
    "rug":       dict(color=[0.48, 0.32, 0.24, 1.0], roughness=0.95, metalness=0.00),
    "plant":     dict(color=[0.18, 0.42, 0.12, 1.0], roughness=0.90, metalness=0.00),
    "appliance": dict(color=[0.88, 0.88, 0.86, 1.0], roughness=0.20, metalness=0.70),
}

# ── Layout constants ───────────────────────────────────────────────────────────
WALL_T   = 0.14    # wall thickness (m)
CEIL_H   = 2.75    # ceiling height (m)
FLOOR_T  = 0.10    # floor slab thickness
CEIL_T   = 0.10    # ceiling slab thickness
DOOR_W   = 0.90    # door width
DOOR_H   = 2.10    # door height

# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

Piece = Tuple[trimesh.Trimesh, SemanticClass, str]   # (mesh, label, material_key)


def _mat(key: str) -> trimesh.visual.material.PBRMaterial:
    p = MAT_PROPS[key]
    return trimesh.visual.material.PBRMaterial(
        baseColorFactor=p["color"],
        roughnessFactor=p["roughness"],
        metallicFactor=p["metalness"],
    )


def box(
    cx: float, cy: float, cz: float,
    sx: float, sy: float, sz: float,
    label: SemanticClass,
    mat: str,
) -> Piece:
    """Axis-aligned box centred at (cx, cy, cz) with extents sx×sy×sz."""
    m = tc.box(extents=[sx, sy, sz])
    m.apply_translation([cx, cy, cz])
    m.visual.material = _mat(mat)
    return m, label, mat


def cylinder(
    cx: float, cy: float, cz_base: float,
    radius: float, height: float,
    label: SemanticClass, mat: str,
    sections: int = 18,
) -> Piece:
    m = tc.cylinder(radius=radius, height=height, sections=sections)
    m.apply_translation([cx, cy, cz_base + height / 2])
    m.visual.material = _mat(mat)
    return m, label, mat


def wall_segment(
    x0: float, y0: float, x1: float, y1: float,
    z0: float = 0.0, z1: float = CEIL_H,
) -> Piece:
    """Straight wall segment from (x0,y0,z0) to (x1,y1,z1) with thickness WALL_T."""
    dx, dy = x1 - x0, y1 - y0
    length = float(np.hypot(dx, dy))
    angle  = float(np.arctan2(dy, dx))
    cx = (x0 + x1) / 2
    cy = (y0 + y1) / 2
    cz = (z0 + z1) / 2
    height = z1 - z0
    m = tc.box(extents=[length, WALL_T, height])
    m.apply_transform(trimesh.transformations.rotation_matrix(angle, [0, 0, 1]))
    m.apply_translation([cx, cy, cz])
    m.visual.material = _mat("wall")
    return m, SemanticClass.WALL, "wall"


def floor_slab(
    x0: float, y0: float, x1: float, y1: float,
    mat: str = "floor",
    z: float = 0.0,
) -> Piece:
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    sx, sy = abs(x1 - x0), abs(y1 - y0)
    m = tc.box(extents=[sx, sy, FLOOR_T])
    m.apply_translation([cx, cy, z - FLOOR_T / 2])
    m.visual.material = _mat(mat)
    return m, SemanticClass.FLOOR, mat


def ceiling_slab(
    x0: float, y0: float, x1: float, y1: float,
    z: float = CEIL_H,
) -> Piece:
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    sx, sy = abs(x1 - x0), abs(y1 - y0)
    m = tc.box(extents=[sx, sy, CEIL_T])
    m.apply_translation([cx, cy, z + CEIL_T / 2])
    m.visual.material = _mat("ceiling")
    return m, SemanticClass.CEILING, "ceiling"


# ─────────────────────────────────────────────────────────────────────────────
# Furniture primitives
# ─────────────────────────────────────────────────────────────────────────────

def sofa(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    """L-shape sofa: base + back + arm-left + arm-right + cushions."""
    pieces: List[Piece] = []
    # Seat base  2.2 × 0.9 × 0.42
    pieces.append(box(0, 0, 0.21,  2.2, 0.9, 0.42, SemanticClass.OBJECT, "fabric"))
    # Back rest   2.2 × 0.18 × 0.62, against -Y
    pieces.append(box(0, -0.36, 0.52,  2.2, 0.18, 0.62, SemanticClass.OBJECT, "fabric"))
    # Left arm
    pieces.append(box(-1.01, 0, 0.52,  0.18, 0.9, 0.62, SemanticClass.OBJECT, "fabric"))
    # Right arm
    pieces.append(box(+1.01, 0, 0.52,  0.18, 0.9, 0.62, SemanticClass.OBJECT, "fabric"))
    # 3 seat cushions
    for i, cx2 in enumerate([-0.73, 0.0, 0.73]):
        pieces.append(box(cx2, 0.05, 0.44, 0.68, 0.82, 0.06, SemanticClass.OBJECT, "fabric"))
    # Legs (4)
    for lx, ly in [(-0.95, 0.35), (0.95, 0.35), (-0.95, -0.35), (0.95, -0.35)]:
        pieces.append(box(lx, ly, 0.06, 0.06, 0.06, 0.12, SemanticClass.OBJECT, "dark_wood"))

    out: List[Piece] = []
    angle = np.radians(yaw_deg)
    R = trimesh.transformations.rotation_matrix(angle, [0, 0, 1])
    for (m, lbl, mat) in pieces:
        m.apply_transform(R)
        m.apply_translation([cx, cy, 0])
        out.append((m, lbl, mat))
    return out


def coffee_table(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.22, 1.1, 0.6, 0.04, SemanticClass.OBJECT, "dark_wood"),  # top
        box(0, 0, 0.11, 1.0, 0.5, 0.02, SemanticClass.OBJECT, "dark_wood"),  # lower shelf
    ]
    for lx, ly in [(-0.48, 0.22), (0.48, 0.22), (-0.48, -0.22), (0.48, -0.22)]:
        pieces.append(box(lx, ly, 0.11, 0.04, 0.04, 0.22, SemanticClass.OBJECT, "dark_wood"))
    return _transform_group(pieces, cx, cy, yaw_deg)


def armchair(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.22, 0.85, 0.85, 0.44, SemanticClass.OBJECT, "leather"),
        box(0, -0.38, 0.56, 0.85, 0.09, 0.50, SemanticClass.OBJECT, "leather"),
        box(-0.42, 0, 0.52, 0.08, 0.85, 0.46, SemanticClass.OBJECT, "leather"),
        box(+0.42, 0, 0.52, 0.08, 0.85, 0.46, SemanticClass.OBJECT, "leather"),
    ]
    return _transform_group(pieces, cx, cy, yaw_deg)


def tv_unit(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.22, 1.8, 0.45, 0.44, SemanticClass.OBJECT, "dark_wood"),   # cabinet
        box(0, -0.02, 0.92, 1.4, 0.08, 0.82, SemanticClass.OBJECT, "appliance"),  # TV screen
        box(0, -0.02, 0.92, 1.44, 0.04, 0.86, SemanticClass.OBJECT, "dark_wood"),  # TV frame
    ]
    return _transform_group(pieces, cx, cy, yaw_deg)


def bookshelf(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 1.05, 0.9, 0.30, 2.10, SemanticClass.OBJECT, "wood"),  # frame
    ]
    # 5 shelves
    for shelf_z in [0.28, 0.62, 0.96, 1.30, 1.64]:
        pieces.append(box(0, 0, shelf_z, 0.86, 0.28, 0.03, SemanticClass.OBJECT, "wood"))
    # Books (visual blocks on each shelf)
    for shelf_z in [0.32, 0.66, 1.00, 1.34, 1.68]:
        for bx_off in np.linspace(-0.35, 0.35, 6):
            w = np.random.uniform(0.03, 0.06)
            h = np.random.uniform(0.15, 0.26)
            pieces.append(box(float(bx_off), 0.02, shelf_z + h/2, w, 0.22, h,
                              SemanticClass.OBJECT, "fabric"))
    return _transform_group(pieces, cx, cy, yaw_deg)


def dining_table(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.76, 1.8, 0.9, 0.04, SemanticClass.OBJECT, "wood"),
    ]
    for lx, ly in [(-0.82, 0.36), (0.82, 0.36), (-0.82, -0.36), (0.82, -0.36)]:
        pieces.append(box(lx, ly, 0.38, 0.06, 0.06, 0.76, SemanticClass.OBJECT, "wood"))
    return _transform_group(pieces, cx, cy, yaw_deg)


def dining_chair(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.24, 0.45, 0.45, 0.06, SemanticClass.OBJECT, "wood"),   # seat
        box(0, -0.20, 0.60, 0.45, 0.04, 0.68, SemanticClass.OBJECT, "wood"),  # back
    ]
    for lx, ly in [(-0.19, 0.19), (0.19, 0.19), (-0.19, -0.19), (0.19, -0.19)]:
        pieces.append(box(lx, ly, 0.24, 0.04, 0.04, 0.48, SemanticClass.OBJECT, "wood"))
    return _transform_group(pieces, cx, cy, yaw_deg)


def bed_king(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        # Frame
        box(0, 0, 0.17, 1.92, 2.10, 0.34, SemanticClass.OBJECT, "dark_wood"),
        # Mattress
        box(0, 0, 0.38, 1.80, 2.00, 0.28, SemanticClass.OBJECT, "white"),
        # Pillow × 2
        box(-0.40, -0.82, 0.54, 0.60, 0.45, 0.12, SemanticClass.OBJECT, "white"),
        box(+0.40, -0.82, 0.54, 0.60, 0.45, 0.12, SemanticClass.OBJECT, "white"),
        # Headboard
        box(0, -1.01, 0.80, 1.92, 0.12, 1.20, SemanticClass.OBJECT, "dark_wood"),
        # Duvet
        box(0, 0.10, 0.54, 1.78, 1.55, 0.10, SemanticClass.OBJECT, "white"),
    ]
    return _transform_group(pieces, cx, cy, yaw_deg)


def nightstand(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.30, 0.50, 0.40, 0.60, SemanticClass.OBJECT, "dark_wood"),
        box(0, 0, 0.32, 0.44, 0.34, 0.02, SemanticClass.OBJECT, "dark_wood"),  # drawer
        # table lamp
        cylinder(0.05, 0, 0.60, 0.12, 0.38, SemanticClass.OBJECT, "white"),
        box(0.05, 0, 0.61, 0.04, 0.04, 0.36, SemanticClass.OBJECT, "metal"),
    ]
    return _transform_group(pieces, cx, cy, yaw_deg)


def wardrobe(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 1.15, 1.80, 0.60, 2.30, SemanticClass.OBJECT, "dark_wood"),
        # doors (slightly proud)
        box(-0.46, -0.30, 1.15, 0.84, 0.02, 2.18, SemanticClass.OBJECT, "dark_wood"),
        box(+0.46, -0.30, 1.15, 0.84, 0.02, 2.18, SemanticClass.OBJECT, "dark_wood"),
    ]
    return _transform_group(pieces, cx, cy, yaw_deg)


def desk(cx: float, cy: float, yaw_deg: float = 0.0) -> List[Piece]:
    pieces: List[Piece] = [
        box(0, 0, 0.74, 1.40, 0.65, 0.03, SemanticClass.OBJECT, "wood"),
    ]
    for lx, ly in [(-0.64, 0.28), (0.64, 0.28), (-0.64, -0.28), (0.64, -0.28)]:
        pieces.append(box(lx, ly, 0.37, 0.04, 0.04, 0.74, SemanticClass.OBJECT, "wood"))
    # Monitor
    pieces.append(box(0, -0.22, 1.08, 0.55, 0.06, 0.38, SemanticClass.OBJECT, "appliance"))
    pieces.append(box(0, -0.20, 0.79, 0.06, 0.22, 0.12, SemanticClass.OBJECT, "appliance"))  # stand
    return _transform_group(pieces, cx, cy, yaw_deg)


def kitchen_counter_l(ox: float, oy: float) -> List[Piece]:
    """L-shaped kitchen counter, origin at inner corner of L."""
    pieces: List[Piece] = []
    # Horizontal run: 2.8 m along X
    pieces.append(box(ox + 1.4, oy, 0.45, 2.8, 0.60, 0.90, SemanticClass.OBJECT, "white"))
    pieces.append(box(ox + 1.4, oy, 0.90, 2.8, 0.60, 0.04, SemanticClass.OBJECT, "ceramic"))  # worktop
    # Vertical run: 1.6 m along Y
    pieces.append(box(ox, oy + 0.8 + 0.30, 0.45, 0.60, 1.90, 0.90, SemanticClass.OBJECT, "white"))
    pieces.append(box(ox, oy + 0.8 + 0.30, 0.90, 0.60, 1.90, 0.04, SemanticClass.OBJECT, "ceramic"))
    # Sink basin
    pieces.append(box(ox + 0.7, oy, 0.89, 0.6, 0.50, 0.15, SemanticClass.OBJECT, "metal"))
    # Fridge
    pieces.append(box(ox + 2.55, oy, 1.05, 0.70, 0.72, 2.10, SemanticClass.OBJECT, "appliance"))
    # Stove (hob + oven)
    pieces.append(box(ox + 1.5, oy, 0.91, 0.60, 0.58, 0.03, SemanticClass.OBJECT, "appliance"))
    pieces.append(box(ox + 1.5, oy, 0.45, 0.60, 0.58, 0.90, SemanticClass.OBJECT, "appliance"))
    # Upper cabinets (horizontal run)
    pieces.append(box(ox + 1.4, oy - 0.17, 2.10, 2.6, 0.34, 0.65, SemanticClass.OBJECT, "white"))
    return [(m, l, mat) for (m, l, mat) in pieces]


def potted_plant(cx: float, cy: float) -> List[Piece]:
    pieces: List[Piece] = [
        cylinder(cx, cy, 0, 0.16, 0.30, SemanticClass.OBJECT, "ceramic"),
        cylinder(cx, cy, 0.30, 0.08, 0.45, SemanticClass.OBJECT, "plant", sections=10),
        cylinder(cx, cy + 0.06, 0.60, 0.06, 0.30, SemanticClass.OBJECT, "plant", sections=8),
        cylinder(cx - 0.08, cy, 0.55, 0.06, 0.25, SemanticClass.OBJECT, "plant", sections=8),
    ]
    return pieces


def _transform_group(
    pieces: List[Piece], cx: float, cy: float, yaw_deg: float
) -> List[Piece]:
    angle = np.radians(yaw_deg)
    R = trimesh.transformations.rotation_matrix(angle, [0, 0, 1])
    out: List[Piece] = []
    for m, lbl, mat in pieces:
        m.apply_transform(R)
        m.apply_translation([cx, cy, 0])
        out.append((m, lbl, mat))
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Room layout builder
# ─────────────────────────────────────────────────────────────────────────────

def build_apartment() -> List[Piece]:
    """
    Apartment layout (all coordinates in metres, Z-up, origin at bottom-left):

       0         4.5        8.5   10.0
    9.0┌──────────┬──────────┬─────┐
       │          │  STUDY / │BATH │
    7.0│          │  OFFICE  │2    │
       │ BEDROOM  ├──────────┤     │
    5.5│  4.5×5.5 │ KITCHEN  ├─────┤
       │          │ 3.5×3.5  │     │
    2.5├──────────┴────┬─────┘ HAL │
       │  LIVING ROOM  │ 1.5m  WAY │
    0.0│  8.5 × 2.5    │           │
       └───────────────┴───────────┘
    (Y axis runs bottom to top)

    Interior openings for doors cut out of wall segments.
    """
    pieces: List[Piece] = []

    # ── Exterior shell ────────────────────────────────────────────────────────
    EXT_X0, EXT_Y0 = 0.0, 0.0
    EXT_X1, EXT_Y1 = 10.0, 9.0

    # South wall
    pieces.append(wall_segment(EXT_X0, EXT_Y0, EXT_X1, EXT_Y0))
    # North wall
    pieces.append(wall_segment(EXT_X0, EXT_Y1, EXT_X1, EXT_Y1))
    # West wall
    pieces.append(wall_segment(EXT_X0, EXT_Y0, EXT_X0, EXT_Y1))
    # East wall
    pieces.append(wall_segment(EXT_X1, EXT_Y0, EXT_X1, EXT_Y1))

    # ── Interior walls ─────────────────────────────────────────────────────────
    # Bedroom / living divider (x=4.5, y=0 → y=5.5) with door at y=1.0
    pieces.append(wall_segment(4.5, 0.0, 4.5, 0.9))           # below door
    pieces.append(wall_segment(4.5, 0.9 + DOOR_W, 4.5, 5.5))  # above door (to kitchen level)

    # Bedroom / office divider (y=5.5, x=0 → x=4.5)
    pieces.append(wall_segment(0.0, 5.5, 4.5 - DOOR_W, 5.5))  # with door near east
    # door gap at x = 4.5-DOOR_W to 4.5  (no wall piece needed)

    # Office / hallway-east divider (x=8.5, y=2.5 → y=9.0)
    pieces.append(wall_segment(8.5, 2.5, 8.5, 5.0))
    pieces.append(wall_segment(8.5, 5.0 + DOOR_W, 8.5, 9.0))

    # Kitchen / living divider (y=2.5, x=4.5 → x=8.5) — open plan, no wall

    # Bathroom west wall (x=8.5 to x=10.0; y=6.5 → y=9.0)
    pieces.append(wall_segment(8.5, 6.5, 10.0, 6.5))          # south bath wall
    # Bathroom door gap is in the east wall at y=7.5

    # ── Floors ────────────────────────────────────────────────────────────────
    pieces.append(floor_slab(EXT_X0, EXT_Y0, EXT_X1, EXT_Y1))

    # Rug in living room
    rug = tc.box(extents=[3.5, 2.0, 0.008])
    rug.apply_translation([6.25, 1.25, 0.005])
    rug.visual.material = _mat("rug")
    pieces.append((rug, SemanticClass.FLOOR, "rug"))

    # ── Ceilings ──────────────────────────────────────────────────────────────
    pieces.append(ceiling_slab(EXT_X0, EXT_Y0, EXT_X1, EXT_Y1))

    # ── Ceiling recessed light circles (visual) ────────────────────────────────
    light_positions = [
        (2.25, 1.25), (6.5, 1.25), (2.25, 4.0), (6.5, 4.0),
        (2.25, 7.25), (6.0, 7.25), (9.25, 7.75),
    ]
    for lx, ly in light_positions:
        ring = tc.annulus(r_min=0.06, r_max=0.12, height=0.02, sections=16)
        ring.apply_translation([lx, ly, CEIL_H + CEIL_T])
        ring.visual.material = _mat("metal")
        pieces.append((ring, SemanticClass.CEILING, "metal"))

    # ── Living room — sofa + coffee table only (open floor for drone patrol) ───
    pieces.extend(sofa(7.5, 0.65, yaw_deg=0))
    pieces.extend(coffee_table(7.5, 1.55))

    # ── Kitchen / dining — L-counter + one table ──────────────────────────────
    pieces.extend(kitchen_counter_l(4.65, 2.65))
    pieces.extend(dining_table(6.8, 4.0))

    # ── Bedroom — bed + one nightstand ────────────────────────────────────────
    pieces.extend(bed_king(2.25, 3.2, yaw_deg=0))
    pieces.extend(nightstand(0.5, 2.0))

    # ── Office — desk only ────────────────────────────────────────────────────
    pieces.extend(desk(6.5, 8.55, yaw_deg=180))

    # ── Bathroom — toilet + vanity only ──────────────────────────────────────
    pieces.append(box(8.8, 8.5, 0.22, 0.42, 0.66, 0.44, SemanticClass.OBJECT, "ceramic"))
    pieces.append(box(8.8, 8.82, 0.35, 0.42, 0.14, 0.10, SemanticClass.OBJECT, "ceramic"))
    pieces.append(box(9.6, 8.55, 0.85, 0.65, 0.46, 0.04, SemanticClass.OBJECT, "ceramic"))

    return pieces


# ─────────────────────────────────────────────────────────────────────────────
# Mesh assembly and export
# ─────────────────────────────────────────────────────────────────────────────

def apply_vertex_colors(mesh: trimesh.Trimesh, label: SemanticClass) -> trimesh.Trimesh:
    col = SEG_COLORS[label]
    vc = np.tile(col, (len(mesh.vertices), 1))
    # Floor: add wood-grain procedural noise
    if label == SemanticClass.FLOOR:
        v = mesh.vertices
        noise = (np.sin(v[:, 0] * 14) * 0.5 + 0.5) * 22
        noise += (np.sin(v[:, 1] * 3 + 0.7) * 0.5 + 0.5) * 12
        vc = vc.astype(np.int32)
        vc[:, 0] = np.clip(vc[:, 0] + noise.astype(np.int32), 0, 255)
        vc[:, 1] = np.clip(vc[:, 1] - noise.astype(np.int32) // 3, 0, 255)
        vc = vc.astype(np.uint8)
    # Walls: subtle plaster variation
    elif label == SemanticClass.WALL:
        v = mesh.vertices
        noise = (np.sin(v[:, 0] * 8) * np.cos(v[:, 2] * 6)) * 8
        vc = vc.astype(np.int32)
        vc[:, :3] = np.clip(vc[:, :3] + noise[:, None].astype(np.int32), 160, 255)
        vc = vc.astype(np.uint8)
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=vc)
    return mesh


def build_and_export() -> None:
    MESHES.mkdir(parents=True, exist_ok=True)
    np.random.seed(42)

    print("[env] Building apartment geometry…")
    pieces = build_apartment()
    print(f"[env] {len(pieces)} geometry pieces assembled")

    # ── Separate visual (GLB) vs collision (PLY) builds ───────────────────────
    collision_meshes: List[trimesh.Trimesh] = []
    visual_meshes:    List[trimesh.Trimesh] = []
    labels_per_tri:   List[np.ndarray]      = []

    for i, (mesh, label, mat) in enumerate(pieces):
        if not isinstance(mesh, trimesh.Trimesh):
            continue
        if len(mesh.faces) == 0:
            continue

        # Collision copy (no materials needed)
        c = mesh.copy()
        c.visual = trimesh.visual.ColorVisuals()
        collision_meshes.append(c)
        labels_per_tri.append(
            np.full(len(c.faces), int(label), dtype=np.uint8)
        )

        # Visual copy with vertex colours
        v = mesh.copy()
        v = apply_vertex_colors(v, label)
        visual_meshes.append(v)

    # ── Collision PLY ──────────────────────────────────────────────────────────
    print("[env] Merging collision mesh…")
    collision = trimesh.util.concatenate(collision_meshes)
    trimesh.repair.fix_winding(collision)
    collision.process(validate=False)
    all_labels = np.concatenate(labels_per_tri)

    # Centre XY, floor at z=0
    verts = collision.vertices
    cx = float((verts[:, 0].min() + verts[:, 0].max()) / 2)
    cy = float((verts[:, 1].min() + verts[:, 1].max()) / 2)
    collision.apply_translation([-cx, -cy, 0])

    col_path = MESHES / "apartment_collision.ply"
    collision.export(str(col_path))
    print(f"[env] Collision PLY → {col_path}  "
          f"({len(collision.faces):,} faces, {col_path.stat().st_size // 1024} KB)")

    # ── Semantic labels ────────────────────────────────────────────────────────
    label_path = MESHES / "apartment_collision_labels.npy"
    np.save(label_path, all_labels[:len(collision.faces)])
    uniq, cnts = np.unique(all_labels[:len(collision.faces)], return_counts=True)
    print(f"[env] Labels → {label_path}")
    for u, c in zip(uniq, cnts):
        pct = 100 * c / len(collision.faces)
        print(f"       {SemanticClass(int(u)).name:<10} {c:>7,}  ({pct:.1f}%)")

    # ── Visual PLY (vertex-coloured) ───────────────────────────────────────────
    print("[env] Merging visual (vertex-colour) mesh…")
    visual = trimesh.util.concatenate(visual_meshes)
    visual.apply_translation([-cx, -cy, 0])

    ply_path = MESHES / "apartment.ply"
    visual.export(str(ply_path))
    print(f"[env] Visual PLY  → {ply_path}  "
          f"({len(visual.faces):,} faces, {ply_path.stat().st_size // 1024} KB)")

    # ── GLB (PBR materials) ────────────────────────────────────────────────────
    print("[env] Building PBR GLB…")
    # Re-build from pieces with proper materials applied
    glb_meshes: List[trimesh.Trimesh] = []
    for (mesh, label, mat) in pieces:
        if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
            continue
        g = mesh.copy()
        g.apply_translation([-cx, -cy, 0])
        glb_meshes.append(g)

    scene = trimesh.Scene()
    for i, m in enumerate(glb_meshes):
        scene.add_geometry(m, node_name=f"obj_{i:04d}")

    glb_path = MESHES / "apartment.glb"
    scene.export(str(glb_path))
    mb = glb_path.stat().st_size / 1_048_576
    print(f"[env] GLB         → {glb_path}  ({mb:.1f} MB)")

    print("\n[env] ✓ Environment built successfully.")
    print(f"      Bounds XY: ({collision.vertices[:,0].min():.2f}, "
          f"{collision.vertices[:,1].min():.2f}) → "
          f"({collision.vertices[:,0].max():.2f}, "
          f"{collision.vertices[:,1].max():.2f})")
    print(f"      Height Z:  0.00 → {collision.vertices[:,2].max():.2f} m")
    print(f"      Total size: {sum(p.stat().st_size for p in [col_path, label_path, ply_path, glb_path]) // 1_048_576} MB")


if __name__ == "__main__":
    build_and_export()
