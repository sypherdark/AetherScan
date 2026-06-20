#!/usr/bin/env python3
"""Flip apartment.ply 180° so ceiling points +Z (Z-up, floor at z=0). Run once after backup."""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent
MESH = ROOT.parent / "dashboard" / "public" / "meshes" / "apartment.ply"
BACKUP = MESH.with_suffix(".ply.bak")


def main() -> None:
    if not MESH.is_file():
        raise SystemExit(f"Missing mesh: {MESH}")

    mesh = o3d.io.read_triangle_mesh(str(MESH))
    if mesh.is_empty():
        raise SystemExit("Empty mesh")

    verts = np.asarray(mesh.vertices)
    mn, mx = verts.min(axis=0), verts.max(axis=0)
    print(f"Before: min={mn}, max={mx}")

    # 180° about X: (x, y, z) -> (x, -y, -z) — inverts vertical if room was upside-down
    R = mesh.get_rotation_matrix_from_xyz((np.pi, 0.0, 0.0))
    mesh.rotate(R, center=(0.0, 0.0, 0.0))

    verts = np.asarray(mesh.vertices)
    mn, mx = verts.min(axis=0), verts.max(axis=0)
    mesh.translate(np.array([0.0, 0.0, -mn[2]]))  # floor -> z=0

    mesh.compute_vertex_normals()
    if not BACKUP.exists():
        shutil.copy2(MESH, BACKUP)
        print(f"Backup: {BACKUP}")

    o3d.io.write_triangle_mesh(str(MESH), mesh)
    mn, mx = np.asarray(mesh.vertices).min(0), np.asarray(mesh.vertices).max(0)
    print(f"After:  min={mn}, max={mx}")
    print(f"Wrote: {MESH}")


if __name__ == "__main__":
    main()
