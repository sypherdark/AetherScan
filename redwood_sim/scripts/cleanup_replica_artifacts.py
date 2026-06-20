#!/usr/bin/env python3
"""One-shot purge of partial Replica archive chunks and incomplete extractions."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPLICA_DIR = ROOT / "data" / "replica"

PART_GLOBS = (
    "replica_v1_0.tar.gz.part*",
    "replica_v1_0.tar.gz",
    "replica_v1_0.tar.gz.*",
    "*.zip",
)


def _dir_size(path: Path) -> int:
    total = 0
    if path.is_file():
        return path.stat().st_size
    for child in path.rglob("*"):
        if child.is_file():
            total += child.stat().st_size
    return total


def main() -> int:
    if not REPLICA_DIR.is_dir():
        print(f"[cleanup] Nothing to do — {REPLICA_DIR} does not exist.")
        return 0

    before = _dir_size(REPLICA_DIR)
    removed: list[str] = []

    for pattern in PART_GLOBS:
        for path in REPLICA_DIR.glob(pattern):
            if path.is_file():
                size = path.stat().st_size
                path.unlink()
                removed.append(f"{path.name} ({size / (1024**3):.2f} GiB)")

    for child in sorted(REPLICA_DIR.iterdir()):
        if not child.is_dir():
            continue
        mesh = child / "mesh.ply"
        semantic = child / "habitat" / "mesh_semantic.ply"
        if mesh.is_file() and semantic.is_file():
            continue
        size = _dir_size(child)
        shutil.rmtree(child)
        removed.append(f"{child.name}/ ({size / (1024**2):.1f} MiB incomplete)")

    after = _dir_size(REPLICA_DIR)
    freed = before - after

    print(f"[cleanup] Target: {REPLICA_DIR}")
    if removed:
        for line in removed:
            print(f"  removed: {line}")
    else:
        print("  no matching artifacts found")

    print(f"[cleanup] Before: {before / (1024**3):.2f} GiB")
    print(f"[cleanup] After:  {after / (1024**3):.2f} GiB")
    print(f"[cleanup] Reclaimed: {freed / (1024**3):.2f} GiB ({freed / (1024**2):.0f} MiB)")

    remaining = list(REPLICA_DIR.iterdir()) if REPLICA_DIR.is_dir() else []
    if not remaining:
        print("[cleanup] replica/ directory is now empty.")
    else:
        print(f"[cleanup] Remaining entries: {[p.name for p in remaining]}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
