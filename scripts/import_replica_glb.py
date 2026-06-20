"""
import_replica_glb.py
=====================
Converts a Replica apartment GLB (X-up coordinate frame) into:
  1. apartment_replica_visual.glb  — Z-up GLB for dashboard (PBR textures preserved)
  2. apartment_replica_collision.ply — Z-up triangle mesh for physics raycasting (downsampled)
  3. apartment_replica_labels.npy   — per-triangle semantic labels (FLOOR/WALL/OBJECT/CEILING)

Coordinate transform applied:
  Replica X-up frame → ROS Z-up ENU frame
  new_X = old_Y
  new_Y = old_Z
  new_Z = old_X + |X_floor|  (shift floor to Z=0)

Usage:
  python scripts/import_replica_glb.py [--scene apartment_1]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
REPLICA_DATA = ROOT / "redwood_sim" / "data" / "replica"
DASHBOARD_MESHES = ROOT / "dashboard" / "public" / "meshes"

# Semantic label IDs (must match scene_loader.py)
FLOOR   = 0
WALL    = 1
OBJECT  = 2
CEILING = 3
LABEL_NAMES = {FLOOR: "FLOOR", WALL: "WALL", OBJECT: "OBJECT", CEILING: "CEILING"}


# ---------------------------------------------------------------------------
# Coordinate-frame helpers
# ---------------------------------------------------------------------------

def detect_x_up_frame(mesh) -> float:
    """
    Verify the mesh uses X as the height axis and return the X-coordinate of
    the floor (the lowest dense cluster of horizontal faces).
    """
    normals = mesh.face_normals
    centroids = mesh.triangles_center
    mostly_x = np.abs(normals[:, 0]) > 0.85
    if mostly_x.sum() == 0:
        raise ValueError("Could not detect X-up frame: no mostly-X normal faces found.")
    x_cents = centroids[mostly_x, 0]
    # Floor is the bottom cluster (lowest ~20%)
    floor_x = float(np.percentile(x_cents, 10))
    ceil_x  = float(np.percentile(x_cents, 90))
    height  = ceil_x - floor_x
    print(f"  Detected X-up frame: floor X={floor_x:.3f}m  ceiling X={ceil_x:.3f}m  height={height:.2f}m")
    return floor_x


def build_xup_to_zup_matrix(floor_x: float) -> np.ndarray:
    """
    Build 4×4 homogeneous transform:  X-up → Z-up, floor shifted to Z=0.

      new_X = old_Y
      new_Y = old_Z
      new_Z = old_X - floor_x
    """
    T = np.zeros((4, 4))
    T[0, 1] = 1.0   # new_X = old_Y
    T[1, 2] = 1.0   # new_Y = old_Z
    T[2, 0] = 1.0   # new_Z = old_X
    T[2, 3] = -floor_x  # shift so floor → Z=0
    T[3, 3] = 1.0
    return T


# ---------------------------------------------------------------------------
# Semantic labeller (height-band + normal direction)
# ---------------------------------------------------------------------------

def label_triangles(mesh) -> np.ndarray:
    """
    Classify every triangle as FLOOR / WALL / OBJECT / CEILING.

    After the Z-up transform, the classification criteria:
      - FLOOR  : large +Z normal component AND Z centroid in bottom 10%
      - CEILING: large -Z normal component AND Z centroid in top 10%
      - WALL   : large horizontal normal (small |Z| component)
      - OBJECT : everything else (furniture tops, oblique faces)
    """
    normals   = mesh.face_normals         # (N, 3)
    centroids = mesh.triangles_center     # (N, 3)
    n_tri = len(normals)

    # Z extent for percentile thresholds
    z_vals   = centroids[:, 2]
    z_min, z_max = float(z_vals.min()), float(z_vals.max())
    z_range  = z_max - z_min
    floor_z_thresh = z_min + z_range * 0.12   # bottom 12%
    ceil_z_thresh  = z_max - z_range * 0.12   # top 12%

    nz      = normals[:, 2]
    abs_nz  = np.abs(nz)

    labels = np.full(n_tri, OBJECT, dtype=np.int32)

    # Floor: upward-facing AND low centroid
    floor_mask = (nz > 0.70) & (z_vals <= floor_z_thresh)
    labels[floor_mask] = FLOOR

    # Ceiling: downward-facing AND high centroid
    ceil_mask = (nz < -0.70) & (z_vals >= ceil_z_thresh)
    labels[ceil_mask] = CEILING

    # Wall: near-vertical normal (mostly horizontal)
    wall_mask = (abs_nz < 0.25) & (labels == OBJECT)
    labels[wall_mask] = WALL

    # Reclassify leftover upward-facing mid-height faces as OBJECT (furniture tops)
    # (already OBJECT by default, just leave them)

    counts = {name: int((labels == lid).sum()) for lid, name in LABEL_NAMES.items()}
    total  = n_tri
    print("  Semantic distribution:")
    for name, cnt in counts.items():
        print(f"    {name:8s}: {cnt:7,} triangles ({100*cnt/total:.1f}%)")
    return labels


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def process_scene(scene_id: str) -> None:
    import trimesh

    glb_path = REPLICA_DATA / scene_id / "mesh.glb"
    if not glb_path.exists():
        sys.exit(f"[import] GLB not found: {glb_path}")

    print(f"\n[import] Loading {glb_path} ...")
    scene = trimesh.load(str(glb_path))
    raw_mesh = trimesh.util.concatenate(list(scene.geometry.values()))
    print(f"  Loaded: {len(raw_mesh.vertices):,} vertices, {len(raw_mesh.faces):,} faces")

    # ---- 1. Detect coordinate frame ----
    floor_x = detect_x_up_frame(raw_mesh)
    T = build_xup_to_zup_matrix(floor_x)

    # ---- 2. Apply transform to full mesh ----
    print("[import] Applying X-up → Z-up coordinate transform ...")
    visual_mesh = raw_mesh.copy()
    visual_mesh.apply_transform(T)

    z_min = float(visual_mesh.bounds[0, 2])
    z_max = float(visual_mesh.bounds[1, 2])
    print(f"  After transform: Z=[{z_min:.3f}, {z_max:.3f}]m  "
          f"(floor≈{z_min:.2f}, ceiling≈{z_max:.2f})")

    # ---- 3. Export visual GLB (full resolution, PBR textures preserved) ----
    # NOTE: We export GLB (not PLY) because the Replica mesh uses UV-mapped PBR
    # textures. PLY doesn't support UV textures; the dashboard uses the GLB
    # directly via Three.js GLTF loader.
    visual_glb = DASHBOARD_MESHES / f"{scene_id}.glb"
    print(f"[import] Exporting visual GLB → {visual_glb}")
    visual_mesh.export(str(visual_glb))
    print(f"  Written: {visual_glb.stat().st_size / 1e6:.1f} MB")

    # ---- 4. Downsample collision mesh ----
    # Use uniform face-index subsampling (every Nth face). Works on Python 3.9
    # and avoids open3d segfaults on large meshes. A 10x reduction on a real
    # scan still covers all structural surfaces for collision + raycasting.
    target_faces = 35_000
    step = max(1, len(visual_mesh.faces) // target_faces)
    sub_faces = visual_mesh.faces[::step]
    collision_mesh = trimesh.Trimesh(
        vertices=visual_mesh.vertices, faces=sub_faces, process=False
    )
    print(f"[import] Collision mesh: {len(collision_mesh.vertices):,} verts, "
          f"{len(collision_mesh.faces):,} faces (1-in-{step} stride)")

    # ---- 5. Export collision PLY ----
    coll_ply = DASHBOARD_MESHES / f"{scene_id}_collision.ply"
    print(f"[import] Exporting collision PLY → {coll_ply}")
    collision_mesh.export(str(coll_ply))
    print(f"  Written: {coll_ply.stat().st_size / 1e6:.1f} MB")

    # ---- 6. Semantic labelling on collision mesh ----
    print("[import] Generating semantic labels ...")
    labels = label_triangles(collision_mesh)
    labels_path = DASHBOARD_MESHES / f"{scene_id}_collision_labels.npy"
    np.save(str(labels_path), labels)
    print(f"  Written: {labels_path}")

    # ---- 7. Summary ----
    print(f"\n✅ Done!  Files written to {DASHBOARD_MESHES}/")
    print(f"   Visual PLY  : {scene_id}.ply")
    print(f"   Collision PLY: {scene_id}_collision.ply")
    print(f"   Labels NPY  : {scene_id}_collision_labels.npy")
    print()
    print("To test, restart the sim bridge with:")
    print(f"  cd redwood_sim && python -m bridge --scene {scene_id}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Import Replica GLB → AetherScan pipeline")
    p.add_argument("--scene", default="apartment_1",
                   help="Replica scene folder name under data/replica/")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    process_scene(args.scene)
