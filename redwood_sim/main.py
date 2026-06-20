#!/usr/bin/env python3
"""
AetherScan indoor quadcopter simulation — production entry point.

Meshes use global coordinates (shared with dashboard PLY). Collision runs
every physics_dt inside RK4 integration.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import SimConfig
from scene_loader import MeshUnavailableError, RedwoodScene, resolve_mesh_path
from simulation.engine import SimulationEngine


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="AetherScan indoor autonomous quadcopter simulation")
    p.add_argument("--scene", default="apartment", help="Scene name or mesh cache key")
    p.add_argument("--mesh", type=str, default=None, help="Explicit .ply path")
    p.add_argument(
        "--download-real",
        action="store_true",
        help="Attempt Open3D / RWS download when cache is empty (default: cache + bundled Replica only)",
    )
    p.add_argument("--voxel", type=float, default=0.03, help="Mesh simplification voxel size (0=off)")
    p.add_argument("--dt", type=float, default=0.002, help="Physics timestep (s)")
    p.add_argument(
        "--bridge",
        action="store_true",
        help="Headless WebSocket bridge for dashboard (no Open3D window)",
    )
    p.add_argument("--port", type=int, default=8765, help="Bridge port (--bridge only)")
    p.add_argument(
        "--center-mesh",
        action="store_true",
        help="Legacy: re-center mesh at origin (breaks dashboard alignment)",
    )
    return p.parse_args()


def load_scene(mesh_path: Path, voxel: float, center_mesh: bool) -> RedwoodScene:
    scene = RedwoodScene(
        mesh_path,
        voxel_downsample=voxel,
        center_mesh=center_mesh,
    )
    scene.log_mesh_info()
    print(
        f"Bounds min={scene.bounds.min_corner} max={scene.bounds.max_corner} "
        f"extent={scene.bounds.extent}"
    )
    return scene


def main() -> None:
    args = parse_args()
    if args.bridge:
        from bridge.server import main as bridge_main

        sys.argv = [
            "bridge",
            "--scene",
            args.scene,
            "--port",
            str(args.port),
            "--voxel",
            str(args.voxel),
            "--dt",
            str(args.dt),
        ]
        if args.center_mesh:
            sys.argv.append("--center-mesh")
        bridge_main()
        return

    cfg = SimConfig(physics_dt=args.dt)
    if args.mesh:
        mesh_path = resolve_mesh_path(args.mesh, args.scene)
    elif args.download_real:
        from scene_loader import download_redwood_mesh

        mesh_path = download_redwood_mesh(args.scene)
    else:
        try:
            mesh_path = resolve_mesh_path(None, args.scene)
        except MeshUnavailableError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            sys.exit(1)
    print(f"Loading environment: {mesh_path}")
    scene = load_scene(mesh_path, args.voxel, center_mesh=args.center_mesh)

    engine = SimulationEngine(scene, cfg, scene_id=args.scene)
    engine.run()


if __name__ == "__main__":
    main()
