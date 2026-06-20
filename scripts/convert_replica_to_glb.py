#!/usr/bin/env python3
"""
convert_replica_to_glb.py
=========================
Convert all Replica dataset scenes (PLY with vertex RGB) to decimated GLB files
that can be served by the Next.js dashboard.

The Replica dataset uses an X-up (Habitat) coordinate frame.  This script
rotates each mesh to Y-up (Three.js / GLTF standard) so the reveal shader's
coordinate math works without any additional transform.

The bridge's _replica_normalize_matrix translates so floor-Z=0 and XY-centred;
that translation becomes the GLB position prop in the dashboard, which the
ScanReconstruction shader subtracts in vWorldPos.  No rotation is needed in
that pipeline — just the Three.js ROS→Y-up mapping: (rx, rz, −ry).

Usage:
    python scripts/convert_replica_to_glb.py [--scenes room_0 office_0 ...]
    python scripts/convert_replica_to_glb.py --all   # all 18 scenes
    python scripts/convert_replica_to_glb.py         # only missing ones
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT            = Path(__file__).resolve().parent.parent
REPLICA_BASE    = Path("/Volumes/SanDisk C/replica")
DASHBOARD_MESH  = ROOT / "dashboard" / "public" / "meshes"

# Target face count per scene after decimation
# (Replica meshes are ~2M faces — we reduce to ~300K for fast web loading)
TARGET_FACES = 300_000

ALL_SCENES = [
    "apartment_0", "apartment_1", "apartment_2",
    "frl_apartment_0", "frl_apartment_1", "frl_apartment_2",
    "frl_apartment_3", "frl_apartment_4", "frl_apartment_5",
    "hotel_0",
    "office_0", "office_1", "office_2", "office_3", "office_4",
    "room_0", "room_1", "room_2",
]


def convert_scene(scene_id: str, force: bool = False) -> bool:
    """Return True on success."""
    ply_path = REPLICA_BASE / scene_id / "mesh.ply"
    out_path = DASHBOARD_MESH / f"{scene_id}.glb"

    if not ply_path.exists():
        print(f"  ✗ {scene_id}: {ply_path} not found (USB mounted?)")
        return False

    if out_path.exists() and not force:
        size_mb = out_path.stat().st_size / 1e6
        print(f"  ✓ {scene_id}: already exists ({size_mb:.1f} MB) — skip (use --force to redo)")
        return True

    try:
        import trimesh
        import open3d as o3d
    except ImportError as e:
        print(f"  ✗ {scene_id}: missing dependency: {e}")
        return False

    print(f"\n▶ {scene_id}")
    t0 = time.perf_counter()

    # ── Load via trimesh (handles non-triangulated polygon PLY faces) ──────────
    print(f"  Loading {ply_path.name}...", end=" ", flush=True)
    raw = trimesh.load(str(ply_path), process=False, force="mesh")
    if isinstance(raw, trimesh.Scene):
        raw = trimesh.util.concatenate(list(raw.geometry.values()))
    n_verts = len(raw.vertices)
    n_faces = len(raw.faces)
    # Extract vertex colours (Uint8 RGBA or RGB from PLY)
    vtx_colors_f32: np.ndarray | None = None
    if hasattr(raw.visual, 'vertex_colors') and raw.visual.vertex_colors is not None:
        vc_raw = np.asarray(raw.visual.vertex_colors, dtype=np.uint8)
        if vc_raw.shape[1] >= 3:
            vtx_colors_f32 = vc_raw[:, :3].astype(np.float32) / 255.0
    print(f"{n_verts:,} verts  {n_faces:,} faces  vtx_colors={vtx_colors_f32 is not None}")

    # ── Normalize + convert to Three.js Y-up ─────────────────────────────────
    # The Replica mesh.ply coordinate system has Z as the vertical axis
    # (floor-to-ceiling range ~2.94m) and X/Y as the horizontal floor plan.
    #
    # Physics simulation (ROS Z-up) uses:
    #   physics_x = raw_x - cx  (horizontal, centred)
    #   physics_y = raw_y - cy  (horizontal, centred)
    #   physics_z = raw_z + |floor_z|  (vertical, floor at 0)
    #
    # Three.js Y-up mapping (aligned with reveal shader's vWorldPos expectations):
    #   three_x = physics_x = raw_x - cx
    #   three_y = physics_z = raw_z - floor_z   ← altitude (Y = up in Three.js)
    #   three_z = -physics_y = -(raw_y - cy)    ← ROS Y-forward → Three.js -Z
    #
    # With mesh_norm_offset=[0,0,0] (physics collision already pre-normalised),
    # vWorldPos = three_xyz and the reveal grid toCell(rx,ry) ↔ vWorldPos.xz match.
    verts_raw = np.asarray(raw.vertices, dtype=np.float64)
    cx = 0.5 * (verts_raw[:, 0].min() + verts_raw[:, 0].max())
    cy = 0.5 * (verts_raw[:, 1].min() + verts_raw[:, 1].max())
    floor_z = verts_raw[:, 2].min()
    verts_yup = np.column_stack([
        verts_raw[:, 0] - cx,       # Three.js X = physics X
        verts_raw[:, 2] - floor_z,  # Three.js Y = physics Z (altitude, floor=0)
        -(verts_raw[:, 1] - cy),    # Three.js Z = -physics Y
    ])

    # ── Build open3d mesh for quadric decimation ──────────────────────────────
    o3d_mesh = o3d.geometry.TriangleMesh()
    o3d_mesh.vertices  = o3d.utility.Vector3dVector(verts_yup)
    o3d_mesh.triangles = o3d.utility.Vector3iVector(raw.faces.astype(np.int32))
    if vtx_colors_f32 is not None:
        o3d_mesh.vertex_colors = o3d.utility.Vector3dVector(vtx_colors_f32.astype(np.float64))

    # ── Decimate ──────────────────────────────────────────────────────────────
    ratio = TARGET_FACES / max(n_faces, 1)
    if ratio < 1.0:
        print(f"  Decimating {n_faces:,} → ~{TARGET_FACES:,} faces ({ratio:.1%})...", end=" ", flush=True)
        o3d_mesh = o3d_mesh.simplify_quadric_decimation(
            target_number_of_triangles=TARGET_FACES,
        )
        n_faces_dec = len(np.asarray(o3d_mesh.triangles))
        print(f"{n_faces_dec:,} faces")
    else:
        print(f"  Under target ({n_faces:,} faces) — no decimation")

    # ── Recompute normals ─────────────────────────────────────────────────────
    o3d_mesh.compute_vertex_normals()

    # ── Convert back to trimesh for GLB export ────────────────────────────────
    verts_out = np.asarray(o3d_mesh.vertices, dtype=np.float32)
    faces_out = np.asarray(o3d_mesh.triangles, dtype=np.int32)
    norms_out = np.asarray(o3d_mesh.vertex_normals, dtype=np.float32)

    tri = trimesh.Trimesh(vertices=verts_out, faces=faces_out, vertex_normals=norms_out, process=False)

    # Vertex colours (decimation may change vertex count — interpolate if needed)
    if o3d_mesh.has_vertex_colors():
        vc = (np.asarray(o3d_mesh.vertex_colors) * 255).astype(np.uint8)
        tri.visual = trimesh.visual.ColorVisuals(mesh=tri, vertex_colors=vc)

    # ── Export ────────────────────────────────────────────────────────────────
    DASHBOARD_MESH.mkdir(parents=True, exist_ok=True)
    print(f"  Exporting GLB...", end=" ", flush=True)
    tri.export(str(out_path))
    size_mb = out_path.stat().st_size / 1e6
    elapsed = time.perf_counter() - t0
    print(f"{size_mb:.1f} MB  ({elapsed:.1f}s total)")
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description="Convert Replica PLY scenes to dashboard GLBs")
    ap.add_argument("--scenes", nargs="+", metavar="SCENE", help="Specific scenes to convert")
    ap.add_argument("--all",    action="store_true", help="Convert all 18 Replica scenes")
    ap.add_argument("--force",  action="store_true", help="Re-convert even if GLB already exists")
    args = ap.parse_args()

    if args.all:
        scenes = ALL_SCENES
    elif args.scenes:
        scenes = args.scenes
    else:
        # Default: convert only missing ones
        scenes = [s for s in ALL_SCENES
                  if not (DASHBOARD_MESH / f"{s}.glb").exists()]
        if not scenes:
            print("All GLBs already exist. Use --all --force to regenerate.")
            return
        print(f"Converting {len(scenes)} missing scene(s)...")

    ok = sum(convert_scene(s, force=args.force) for s in scenes)
    print(f"\n{'─'*50}")
    print(f"Done: {ok}/{len(scenes)} scenes converted → {DASHBOARD_MESH}")


if __name__ == "__main__":
    main()
