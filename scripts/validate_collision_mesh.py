#!/usr/bin/env python3
"""
Validate backend collision mesh raycasting before starting the sim bridge.

Usage:
  redwood_sim/.venv/bin/python scripts/validate_collision_mesh.py --scene apartment
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REDWOOD = ROOT / "redwood_sim"
MESHES = ROOT / "dashboard" / "public" / "meshes"

sys.path.insert(0, str(REDWOOD))

from scene_loader import (  # noqa: E402
    RedwoodScene,
    resolve_dashboard_collision_mesh,
)


def validate(scene_id: str, meshes_dir: Path) -> int:
    collision = resolve_dashboard_collision_mesh(scene_id, meshes_dir)
    if collision is None:
        print(f"ERROR: no collision mesh for scene '{scene_id}' in {meshes_dir}")
        return 1

    print(f"[validate] loading collision: {collision}")
    scene = RedwoodScene(collision, voxel_downsample=0.0, compute_normals=True)
    scene.log_mesh_info()

    b = scene.bounds
    spawn = np.array(
        [
            float(b.min_corner[0] + 1.2),
            float(b.min_corner[1] + 1.2),
            float(b.min_corner[2] + 0.15),
        ],
        dtype=np.float64,
    )

    probes: list[np.ndarray] = [spawn]
    cx, cy, cz = b.center
    for z in (0.15, 1.0, 1.45, 2.5):
        probes.append(np.array([cx, cy, float(b.min_corner[2]) + z]))
    for dx, dy in [(-1, -1), (1, 1), (-1, 1), (1, -1)]:
        probes.append(
            np.array(
                [
                    float(cx + dx * 0.35 * (b.max_corner[0] - b.min_corner[0])),
                    float(cy + dy * 0.35 * (b.max_corner[1] - b.min_corner[1])),
                    1.45,
                ]
            )
        )

    directions = np.array(
        [
            [1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [0, 0, 1],
            [0, 0, -1],
        ],
        dtype=np.float64,
    )

    hits = 0
    total = 0
    for origin in probes:
        origins = np.tile(origin, (len(directions), 1))
        dists, _, _, _ = scene.cast_rays(origins, directions, max_distance=12.0)
        for d in dists:
            total += 1
            if np.isfinite(d) and d < 11.9:
                hits += 1

    rate = hits / max(total, 1)
    print(f"[validate] ray hit rate: {hits}/{total} ({rate * 100:.1f}%)")
    print(f"[validate] spawn probe: {spawn.tolist()}")

    if rate < 0.35:
        print("FAIL: collision mesh may be hollow or misaligned — rebuild with build_collision_mesh.py")
        return 1
    print("PASS: collision mesh responds to raycasts")
    return 0


def main() -> None:
    p = argparse.ArgumentParser(description="Validate collision PLY raycasting")
    p.add_argument("--scene", default="apartment")
    p.add_argument("--meshes-dir", type=Path, default=MESHES)
    args = p.parse_args()
    raise SystemExit(validate(args.scene, args.meshes_dir))


if __name__ == "__main__":
    main()
