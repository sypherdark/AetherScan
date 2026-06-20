#!/usr/bin/env python3
"""
Build a simplified watertight collision mesh for backend raycasting.

Outputs:
  dashboard/public/meshes/{scene}_collision.ply
  dashboard/public/meshes/{scene}_collision_labels.npy  (when --semantic-source is set)

Usage:
  # Plain mesh (no per-triangle semantics):
  redwood_sim/.venv/bin/python scripts/build_collision_mesh.py --scene apartment \\
      --source dashboard/public/meshes/apartment.ply

  # Replica semantic mesh → collision + triangle class labels:
  redwood_sim/.venv/bin/python scripts/build_collision_mesh.py --scene apartment \\
      --semantic-source redwood_sim/data/replica/apartment_0/habitat/mesh_semantic.ply \\
      --semantic-json redwood_sim/data/replica/apartment_0/habitat/semantic.json
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

from scene_loader import (  # noqa: E402
    COLLISION_LABELS_SUFFIX,
    _apply_transform_to_mesh,
    _build_triangle_semantic_labels,
    _load_replica_instance_map,
    _replica_normalize_matrix,
    canonicalize_indoor_mesh_z_up,
    load_collision_mesh,
    load_semantic_mesh,
)


def _repair_watertight(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    try:
        import trimesh

        tm = trimesh.Trimesh(
            vertices=np.asarray(mesh.vertices),
            faces=np.asarray(mesh.triangles),
            process=False,
        )
        trimesh.repair.fill_holes(tm)
        tm.remove_duplicate_faces()
        tm.remove_degenerate_faces()
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(np.asarray(tm.vertices, dtype=np.float64)),
            o3d.utility.Vector3iVector(np.asarray(tm.faces, dtype=np.int32)),
        )
    except Exception as exc:
        print(f"[build-collision] trimesh repair skipped: {exc}")
    return mesh


def _triangle_centroids(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    tris = np.asarray(mesh.triangles, dtype=np.int32)
    return verts[tris].mean(axis=1)


def _remap_labels(orig_mesh: o3d.geometry.TriangleMesh, orig_labels: np.ndarray, new_mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    from scipy.spatial import cKDTree

    orig_c = _triangle_centroids(orig_mesh)
    new_c = _triangle_centroids(new_mesh)
    tree = cKDTree(orig_c)
    _, idx = tree.query(new_c, k=1)
    return orig_labels[np.asarray(idx, dtype=np.int64)]


def _simplify(mesh: o3d.geometry.TriangleMesh, max_tris: int) -> o3d.geometry.TriangleMesh:
    n = len(mesh.triangles)
    if n <= max_tris:
        return mesh
    target = max(1000, int(max_tris))
    try:
        simplified = mesh.simplify_quadric_decimation(target_number_of_triangles=target)
        if len(simplified.triangles) > 0:
            return simplified
    except Exception as exc:
        print(f"[build-collision] quadric decimation failed ({exc}); using vertex clustering")
    voxel = float(mesh.get_axis_aligned_bounding_box().get_extent().max() / 80.0)
    return mesh.simplify_vertex_clustering(voxel_size=max(voxel, 0.02))


def build_collision_mesh(
    source: Path,
    out: Path,
    *,
    max_tris: int = 100_000,
    skip_canonicalize: bool = False,
    labels_out: Path | None = None,
    labels: np.ndarray | None = None,
    mesh_in: o3d.geometry.TriangleMesh | None = None,
) -> tuple[o3d.geometry.TriangleMesh, np.ndarray | None]:
    if mesh_in is not None:
        mesh = mesh_in
    else:
        source = Path(source)
        if not source.is_file():
            raise FileNotFoundError(source)

        if source.name.endswith("_semantic.ply") or source.name == "mesh_semantic.ply":
            mesh = load_semantic_mesh(source)
            skip_canonicalize = True
        else:
            raw = o3d.io.read_triangle_mesh(str(source))
            if raw.is_empty() or len(raw.triangles) == 0:
                raise ValueError(f"Source has no triangles: {source}")
            mesh = raw if skip_canonicalize else canonicalize_indoor_mesh_z_up(raw)

    if labels is None and labels_out is None:
        print(f"[build-collision] input triangles: {len(mesh.triangles):,}")
        mesh = _repair_watertight(mesh)
        mesh = _simplify(mesh, max_tris)
        mesh = _repair_watertight(mesh)
    else:
        if labels is None:
            raise ValueError("labels required when building semantic collision")
        if len(labels) != len(mesh.triangles):
            raise ValueError(
                f"Label count {len(labels)} != triangle count {len(mesh.triangles)}"
            )
        print(f"[build-collision] semantic input triangles: {len(mesh.triangles):,}")
        orig = mesh
        orig_labels = labels.copy()
        mesh = _repair_watertight(mesh)
        mesh = _simplify(mesh, max_tris)
        mesh = _repair_watertight(mesh)
        labels = _remap_labels(orig, orig_labels, mesh)
        print(
            f"[build-collision] semantic classes preserved: "
            f"{len(np.unique(labels))} unique labels"
        )

    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()

    verts = np.asarray(mesh.vertices)
    mn = verts.min(axis=0)
    mesh.translate(np.array([0.0, 0.0, -float(mn[2])]))

    out.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out), mesh)
    print(f"[build-collision] wrote {out} ({len(mesh.triangles):,} triangles)")
    load_collision_mesh(out)

    if labels is not None and labels_out is not None:
        np.save(labels_out, labels.astype(np.uint8))
        print(f"[build-collision] wrote labels → {labels_out} ({len(labels):,})")

    return mesh, labels


def build_from_semantic_source(
    scene: str,
    semantic_ply: Path,
    semantic_json: Path | None,
    *,
    max_tris: int = 100_000,
) -> None:
    semantic_ply = Path(semantic_ply)
    mesh = load_semantic_mesh(semantic_ply)
    instance_map = _load_replica_instance_map(semantic_json)
    labels = _build_triangle_semantic_labels(semantic_ply, instance_map)

    if len(labels) != len(mesh.triangles):
        raise ValueError(
            f"Semantic label triangles {len(labels)} != mesh triangles {len(mesh.triangles)}"
        )

    transform = _replica_normalize_matrix(np.asarray(mesh.vertices, dtype=np.float64))
    _apply_transform_to_mesh(mesh, transform)

    out = MESHES / f"{scene}_collision.ply"
    labels_out = MESHES / f"{scene}{COLLISION_LABELS_SUFFIX}"

    build_collision_mesh(
        semantic_ply,
        out,
        max_tris=max_tris,
        skip_canonicalize=True,
        labels_out=labels_out,
        labels=labels,
        mesh_in=mesh,
    )


def main() -> None:
    p = argparse.ArgumentParser(description="Build backend collision PLY from a source mesh")
    p.add_argument("--scene", default="apartment", help="Scene id (output: {scene}_collision.ply)")
    p.add_argument(
        "--source",
        type=Path,
        help="Input triangle mesh (.ply). Default: dashboard/public/meshes/{scene}.ply",
    )
    p.add_argument(
        "--semantic-source",
        type=Path,
        help="Replica habitat/mesh_semantic.ply (enables triangle class labels)",
    )
    p.add_argument(
        "--semantic-json",
        type=Path,
        help="Replica semantic.json mapping object_id → class name",
    )
    p.add_argument("--max-tris", type=int, default=100_000)
    p.add_argument(
        "--skip-canonicalize",
        action="store_true",
        help="Source is already Z-up ROS (e.g. exported from Blender)",
    )
    args = p.parse_args()

    scene = args.scene.lower().strip()

    if args.semantic_source is not None:
        build_from_semantic_source(
            scene,
            args.semantic_source,
            args.semantic_json,
            max_tris=args.max_tris,
        )
        return

    source = args.source or (MESHES / f"{scene}.ply")
    out = MESHES / f"{scene}_collision.ply"
    build_collision_mesh(
        source,
        out,
        max_tris=args.max_tris,
        skip_canonicalize=args.skip_canonicalize,
    )


if __name__ == "__main__":
    main()
