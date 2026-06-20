#!/usr/bin/env python3
"""One-time repair: canonicalize dashboard PLY files to Z-up ROS (floor z=0)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "redwood_sim"))

import open3d as o3d  # noqa: E402

from scene_loader import canonicalize_indoor_mesh_z_up, load_triangle_mesh  # noqa: E402


def repair_mesh(path: Path) -> None:
    if not path.is_file():
        print(f"Skip missing: {path}")
        return
    mesh = load_triangle_mesh(path)
    mesh = canonicalize_indoor_mesh_z_up(mesh)
    o3d.io.write_triangle_mesh(str(path), mesh)
    import numpy as np

    v = np.asarray(mesh.vertices)
    mn, mx = v.min(0), v.max(0)
    print(f"Repaired {path.name}: extent={(mx - mn).round(3).tolist()} z=[{mn[2]:.3f}, {mx[2]:.3f}]")


def main() -> None:
    meshes = ROOT / "dashboard" / "public" / "meshes"
    for name in ("apartment", "boardroom"):
        repair_mesh(meshes / f"{name}.ply")


if __name__ == "__main__":
    main()
