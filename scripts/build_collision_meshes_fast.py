#!/usr/bin/env python3
"""
Build fast geometry-only collision meshes for every Replica scene.

Only apartment_1 shipped with a collision PLY, so every other scene loaded the
full ~2–9 M-triangle Replica mesh and rebuilt a BVH on each switch (~24 s freeze,
which read as a crash).  This produces a decimated `{scene}_collision.ply` in the
SAME normalised frame the GLB visual uses, so the bridge picks it up (priority #1
in `load_semantic_redwood_scene`) and scene-switching becomes near-instant.

Geometry-only is sufficient: the realistic sensor model now classifies surfaces
from geometry (normal + height), so per-triangle semantic labels are not needed
for navigation — which sidesteps the label/decimation alignment problem entirely.

    redwood_sim/.venv/bin/python scripts/build_collision_meshes_fast.py [--force] [--scenes a b]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPLICA = ROOT / "redwood_sim" / "data" / "replica"
MESHES = ROOT / "dashboard" / "public" / "meshes"
sys.path.insert(0, str(ROOT / "redwood_sim"))

from scene_loader import _replica_normalize_matrix  # noqa: E402

TARGET_TRIS = 120_000
ALL_SCENES = [
    "apartment_0", "apartment_1", "apartment_2",
    "frl_apartment_0", "frl_apartment_1", "frl_apartment_2",
    "frl_apartment_3", "frl_apartment_4", "frl_apartment_5",
    "hotel_0",
    "office_0", "office_1", "office_2", "office_3", "office_4",
    "room_0", "room_1", "room_2",
]


def build(scene: str, force: bool) -> bool:
    import trimesh
    import open3d as o3d

    out = MESHES / f"{scene}_collision.ply"
    if out.exists() and not force:
        print(f"  ✓ {scene}: collision PLY exists — skip")
        return True

    src = REPLICA / scene / "mesh.ply"
    if not src.exists():
        print(f"  ✗ {scene}: {src} not found (USB mounted?)")
        return False

    t0 = time.perf_counter()
    print(f"\n▶ {scene}  loading {src.name} …", flush=True)
    raw = trimesh.load(str(src), process=False, force="mesh")
    if isinstance(raw, trimesh.Scene):
        raw = trimesh.util.concatenate(tuple(raw.geometry.values()))
    V = np.asarray(raw.vertices, dtype=np.float64)
    F = np.asarray(raw.faces, dtype=np.int32)
    print(f"  {len(V):,} verts  {len(F):,} faces", flush=True)

    # Normalise into the backend frame: Z-up, XY-centred, floor at Z=0 — the same
    # transform convert_replica_to_glb / the bridge use, so collision ↔ visual ↔
    # drone telemetry all share one frame.
    T = _replica_normalize_matrix(V)
    Vn = (T[:3, :3] @ V.T).T + T[:3, 3]

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(Vn),
        o3d.utility.Vector3iVector(F),
    )
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    n0 = len(mesh.triangles)
    if n0 > TARGET_TRIS:
        print(f"  decimating {n0:,} → ~{TARGET_TRIS:,} …", flush=True)
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=TARGET_TRIS)
    mesh.compute_vertex_normals()

    MESHES.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out), mesh, write_ascii=False)
    mb = out.stat().st_size / 1e6
    print(f"  ✓ {scene}: {len(mesh.triangles):,} tris  {mb:.1f} MB  ({time.perf_counter()-t0:.1f}s)")
    return True


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--scenes", nargs="+")
    args = ap.parse_args()
    scenes = args.scenes or ALL_SCENES
    ok = sum(build(s, args.force) for s in scenes)
    print(f"\n{'-' * 50}\nBuilt/kept {ok}/{len(scenes)} collision meshes → {MESHES}")


if __name__ == "__main__":
    main()
