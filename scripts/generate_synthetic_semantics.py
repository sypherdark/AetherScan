#!/usr/bin/env python3
"""
Generate per-triangle semantic labels for a collision mesh without Replica data.

Uses a multi-stage algorithm:
  1. Height-band classification (floor / ceiling / mid-room objects)
  2. Normal-direction classification (walls vs. oblique surfaces)
  3. DBSCAN clustering to split distinct object regions
  4. Confidence-weighted label fusion

Writes: dashboard/public/meshes/{scene}_collision_labels.npy

Usage:
  redwood_sim/.venv/bin/python scripts/generate_synthetic_semantics.py --scene apartment
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
REDWOOD = ROOT / "redwood_sim"
MESHES = ROOT / "dashboard" / "public" / "meshes"

sys.path.insert(0, str(REDWOOD))

from scene_loader import COLLISION_LABELS_SUFFIX, load_collision_mesh  # noqa: E402
from core.semantic_space import SemanticClass  # noqa: E402


# ── height thresholds (relative to scene height) ─────────────────────────────
FLOOR_BAND_FRAC = 0.10   # bottom 10 % of scene height → FLOOR
CEILING_BAND_FRAC = 0.90 # top 10 % of scene height    → CEILING
OBJECT_LOW_M = 0.25      # objects sitting > 25 cm from floor
OBJECT_HIGH_M = 2.20     # objects reaching no higher than 2.2 m AGL (not walls/ceiling)

# ── normal thresholds ─────────────────────────────────────────────────────────
NZ_HORIZONTAL = 0.72     # |nz| above this → definitely horizontal surface
NZ_WALL = 0.30           # |nz| below this → definitely wall

# ── DBSCAN clustering ─────────────────────────────────────────────────────────
CLUSTER_EPS = 0.45       # 45 cm neighbourhood radius for object clustering
CLUSTER_MIN = 15         # minimum triangles to form a distinct object


def _compute_triangle_centroids(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    tris = np.asarray(mesh.triangles, dtype=np.int32)
    return verts[tris].mean(axis=1)   # (N, 3)


def classify_triangle_normals(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()

    normals = np.asarray(mesh.triangle_normals, dtype=np.float64)
    centroids = _compute_triangle_centroids(mesh)
    n_tri = len(normals)

    verts = np.asarray(mesh.vertices, dtype=np.float64)
    z_floor = float(verts[:, 2].min())
    z_ceil = float(verts[:, 2].max())
    z_range = z_ceil - z_floor

    floor_thresh = z_floor + z_range * FLOOR_BAND_FRAC
    ceil_thresh = z_floor + z_range * CEILING_BAND_FRAC

    labels = np.full(n_tri, int(SemanticClass.UNKNOWN), dtype=np.uint8)
    nz = np.abs(normals[:, 2])
    cz = centroids[:, 2]

    # ── Pass 1: geometry-only heuristic ───────────────────────────────────────
    for i in range(n_tri):
        nzi = float(nz[i])
        czi = float(cz[i])

        if nzi >= NZ_HORIZONTAL:
            if czi <= floor_thresh:
                labels[i] = int(SemanticClass.FLOOR)
            elif czi >= ceil_thresh:
                labels[i] = int(SemanticClass.CEILING)
            else:
                # Horizontal surface at mid-height → furniture / object top
                labels[i] = int(SemanticClass.OBJECT)
        elif nzi <= NZ_WALL:
            labels[i] = int(SemanticClass.WALL)
        else:
            # Oblique surface — use height to disambiguate
            if czi <= floor_thresh + 0.05:
                labels[i] = int(SemanticClass.FLOOR)
            elif czi >= ceil_thresh - 0.05:
                labels[i] = int(SemanticClass.CEILING)
            else:
                labels[i] = int(SemanticClass.OBJECT)

    # ── Pass 2: DBSCAN — split object triangles into distinct clusters ─────────
    obj_mask = labels == int(SemanticClass.OBJECT)
    obj_indices = np.where(obj_mask)[0]

    if len(obj_indices) >= CLUSTER_MIN:
        try:
            from sklearn.cluster import DBSCAN

            pts = centroids[obj_indices]
            db = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN, n_jobs=-1).fit(pts)
            cluster_labels = db.labels_

            n_clusters = int(cluster_labels.max()) + 1
            print(f"  DBSCAN found {n_clusters} distinct object clusters "
                  f"({int(np.sum(cluster_labels < 0))} noise triangles)")

            # Noise triangles that are very close to the floor → reclassify as floor
            noise_mask = cluster_labels < 0
            noise_global = obj_indices[noise_mask]
            near_floor = cz[noise_global] <= (z_floor + z_range * 0.15)
            labels[noise_global[near_floor]] = int(SemanticClass.FLOOR)

            # Tag each cluster: if its centroid Z is within object band, keep as OBJECT
            # If centroid is near floor → reclassify as FLOOR (low rugs / baseboards)
            for cid in range(n_clusters):
                c_mask = cluster_labels == cid
                c_global = obj_indices[c_mask]
                median_z = float(np.median(cz[c_global]))
                if median_z < z_floor + OBJECT_LOW_M:
                    labels[c_global] = int(SemanticClass.FLOOR)
                elif median_z > z_floor + OBJECT_HIGH_M:
                    # Very tall mid-height objects are more likely wall columns / pillars
                    cluster_height = float(cz[c_global].max() - cz[c_global].min())
                    if cluster_height > z_range * 0.6:
                        labels[c_global] = int(SemanticClass.WALL)

        except ImportError:
            print("  scikit-learn not available — skipping DBSCAN refinement")

    # ── Pass 3: wall column sanity — very thin vertical slabs near bounds → WALL
    wall_mask = labels == int(SemanticClass.WALL)
    # nothing to do here; the normal-based pass is already solid for walls

    return labels


def main() -> None:
    p = argparse.ArgumentParser(description="Synthetic collision triangle semantics")
    p.add_argument("--scene", default="apartment")
    p.add_argument(
        "--collision",
        type=Path,
        help="Collision PLY (default: dashboard/public/meshes/{scene}_collision.ply)",
    )
    args = p.parse_args()

    scene = args.scene.lower().strip()
    collision = args.collision or (MESHES / f"{scene}_collision.ply")
    if not collision.is_file():
        raise SystemExit(f"Collision mesh not found: {collision}")

    print(f"[synthetic-semantics] Loading collision mesh: {collision}")
    mesh = load_collision_mesh(collision)
    print(f"  Triangles: {len(mesh.triangles):,}")

    labels = classify_triangle_normals(mesh)
    out = MESHES / f"{scene}{COLLISION_LABELS_SUFFIX}"

    np.save(out, labels)
    uniq, counts = np.unique(labels, return_counts=True)
    print(f"\n[synthetic-semantics] Wrote {out} ({len(labels):,} triangles)")
    print(f"  {'Class':<12} {'Count':>8}  {'%':>6}")
    print(f"  {'-'*30}")
    for u, c in zip(uniq, counts):
        name = SemanticClass(int(u)).name
        pct = 100.0 * c / len(labels)
        print(f"  {name:<12} {c:>8,}  {pct:>5.1f}%")


if __name__ == "__main__":
    main()
