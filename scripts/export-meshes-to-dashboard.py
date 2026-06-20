#!/usr/bin/env python3
"""Copy triangulated authentic meshes into dashboard/public/meshes."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REDWOOD = ROOT / "redwood_sim"
OUT = ROOT / "dashboard" / "public" / "meshes"

sys.path.insert(0, str(REDWOOD))

from scene_loader import (  # noqa: E402
    MeshUnavailableError,
    ensure_triangle_mesh_file,
    find_bundled_mesh,
    resolve_authentic_mesh_cache,
)


def _copy_mesh(src: Path, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    print(f"  {dest} ← {src} ({dest.stat().st_size:,} bytes)")


def _resolve_for_export(name: str) -> Path:
    cache = REDWOOD / "data" / "redwood"
    try:
        raw = resolve_authentic_mesh_cache(name, cache)
    except MeshUnavailableError:
        bundled = find_bundled_mesh(name)
        if bundled is None:
            raise
        raw = bundled
    return ensure_triangle_mesh_file(raw)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ("apartment", "boardroom"):
        try:
            src = _resolve_for_export(name)
        except MeshUnavailableError as exc:
            print(f"ERROR: {name}: {exc}")
            sys.exit(1)
        _copy_mesh(src, OUT / f"{name}.ply")

    print(f"\nTriangulated meshes ready at {OUT}")


if __name__ == "__main__":
    main()
