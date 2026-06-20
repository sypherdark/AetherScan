"""
import_scannet_scene.py — Stage 2: ScanNet integration
=======================================================

Converts a downloaded ScanNet scene into the AetherScan pipeline format.

ScanNet per-scene files needed (from the official download script):
  <scene_id>_vh_clean_2.ply          — vertex-coloured surface mesh
  <scene_id>_vh_clean_2.labels.ply   — per-vertex semantic label mesh (same topology)
  <scene_id>.aggregation.json        — object instance aggregation

ScanNet 40 → AetherScan 4-class label mapping:
  floor, rug, carpet          → FLOOR (0)
  wall, door, window, curtain → WALL  (1)
  ceiling                     → CEILING (3)
  everything else             → OBJECT (2)

Usage:
  1. Request ScanNet access:  http://www.scan-net.org/ScanNet/
     (fill out the ToU form and email it; automated reply within minutes)

  2. Download a single scene (apartment scene example):
     python download-scannet.py --id scene0000_00 --type _vh_clean_2.ply
     python download-scannet.py --id scene0000_00 --type _vh_clean_2.labels.ply
     python download-scannet.py --id scene0000_00 --type .aggregation.json

  3. Run this script:
     python scripts/import_scannet_scene.py \\
       --ply /path/to/scene0000_00_vh_clean_2.ply \\
       --labels /path/to/scene0000_00_vh_clean_2.labels.ply \\
       --out-name scannet_apartment

  4. Launch:
     ./run-aetherscan.sh --dashboard --scene scannet_apartment
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DASHBOARD_MESHES = ROOT / "dashboard" / "public" / "meshes"

# ScanNet v2 40-class label → AetherScan 4-class
FLOOR, WALL, OBJECT, CEILING = 0, 1, 2, 3
SCANNET_LABEL_MAP: dict[int, int] = {}

# Build the map from the ScanNet 40-category labels
# Category IDs sourced from ScanNet's scannet-labels.combined.tsv
# fmt: off
_FLOOR_IDS   = {2, 4, 22}            # floor, carpet, rug
_WALL_IDS    = {1, 8, 9, 19, 21}     # wall, door, window, curtain, blinds
_CEILING_IDS = {3}                    # ceiling
# Everything else → OBJECT (2)
# fmt: on

for _id in range(0, 42):
    if _id in _FLOOR_IDS:
        SCANNET_LABEL_MAP[_id] = FLOOR
    elif _id in _WALL_IDS:
        SCANNET_LABEL_MAP[_id] = WALL
    elif _id in _CEILING_IDS:
        SCANNET_LABEL_MAP[_id] = CEILING
    else:
        SCANNET_LABEL_MAP[_id] = OBJECT


def read_ply_vertex_labels(labels_ply: Path) -> np.ndarray:
    """
    Extract per-vertex integer labels from a ScanNet `_vh_clean_2.labels.ply`.

    The labels are stored as a vertex scalar property named 'label' (uint16).
    """
    data = labels_ply.read_bytes()
    header_end = data.find(b"end_header\n")
    if header_end < 0:
        raise ValueError(f"No PLY end_header in {labels_ply}")
    header = data[:header_end].decode("ascii", errors="ignore")

    n_vertices = 0
    properties: list[str] = []
    binary_little = False
    for line in header.splitlines():
        if line.startswith("element vertex"):
            n_vertices = int(line.split()[-1])
        elif line.startswith("property"):
            parts = line.split()
            properties.append(parts[-1])   # property name
        elif "binary_little_endian" in line:
            binary_little = True

    if n_vertices == 0:
        raise ValueError("Could not parse vertex count from PLY header")

    # Build numpy dtype from property list
    dtype_map = {
        "float": "f4", "double": "f8",
        "uchar": "u1", "uint8": "u1",
        "short": "i2", "int16": "i2",
        "ushort": "u2", "uint16": "u2",
        "int": "i4", "int32": "i4",
        "uint": "u4", "uint32": "u4",
        "char": "i1",
    }
    dtype_fields: list[tuple[str, str]] = []
    for line in header.splitlines():
        if not line.startswith("property "):
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        ply_type = parts[1].lower()
        prop_name = parts[2]
        np_type = dtype_map.get(ply_type, "u1")
        dtype_fields.append((prop_name, np_type))

    if not dtype_fields:
        raise ValueError("Could not parse PLY vertex dtype")

    vertex_dtype = np.dtype(dtype_fields)
    body_start = header_end + len(b"end_header\n")
    vertex_data = np.frombuffer(
        data[body_start: body_start + n_vertices * vertex_dtype.itemsize],
        dtype=vertex_dtype,
    )

    # Find the label field
    for field in ["label", "scalar_label", "Label"]:
        if field in vertex_data.dtype.names:
            return vertex_data[field].astype(np.int32)

    raise ValueError(f"No 'label' field found in {labels_ply}. Fields: {vertex_data.dtype.names}")


def vertex_labels_to_face_labels(
    faces: np.ndarray, vertex_labels: np.ndarray
) -> np.ndarray:
    """
    Majority-vote per-face label from the three vertex labels of each triangle.
    """
    v0 = vertex_labels[faces[:, 0]]
    v1 = vertex_labels[faces[:, 1]]
    v2 = vertex_labels[faces[:, 2]]
    # Simple majority: if two or three vertices agree, use that label.
    # Fallback: use vertex 0 label.
    face_labels = v0.copy()
    mask_1_2 = v1 == v2
    face_labels[mask_1_2] = v1[mask_1_2]
    mask_0_1 = v0 == v1
    face_labels[mask_0_1] = v0[mask_0_1]
    return face_labels


def map_scannet_to_aetherscan(raw_labels: np.ndarray) -> np.ndarray:
    """Convert ScanNet 40-class labels → AetherScan 4-class labels."""
    out = np.vectorize(lambda x: SCANNET_LABEL_MAP.get(int(x), OBJECT))(raw_labels)
    return out.astype(np.int32)


def reorient_to_zup(mesh) -> tuple:
    """
    ScanNet uses Z-up already (+Z = up, floor near Z=0). Shift so floor is
    exactly at Z=0 and return (transformed_mesh, floor_z_offset).
    """
    import trimesh

    normals = mesh.face_normals
    cents   = mesh.triangles_center
    mostly_z = np.abs(normals[:, 2]) > 0.85
    floor_z = float(np.percentile(cents[mostly_z & (normals[:, 2] > 0.5), 2], 5)) \
              if mostly_z.sum() > 0 else float(mesh.bounds[0, 2])
    print(f"  ScanNet floor Z detected: {floor_z:.3f}m")
    T = np.eye(4)
    T[2, 3] = -floor_z
    m2 = mesh.copy()
    m2.apply_transform(T)
    return m2, floor_z


def process_scene(args: argparse.Namespace) -> None:
    import trimesh

    ply_path    = Path(args.ply)
    labels_path = Path(args.labels)
    out_name    = args.out_name

    if not ply_path.exists():
        sys.exit(f"[import] Mesh PLY not found: {ply_path}")
    if not labels_path.exists():
        sys.exit(f"[import] Labels PLY not found: {labels_path}")

    DASHBOARD_MESHES.mkdir(parents=True, exist_ok=True)

    print(f"\n[import] Loading ScanNet mesh: {ply_path}")
    mesh = trimesh.load(str(ply_path))
    if hasattr(mesh, 'geometry'):
        mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
    print(f"  {len(mesh.vertices):,} vertices, {len(mesh.faces):,} faces")

    # ---- Reorient to Z-up, floor at Z=0 ----
    print("[import] Checking orientation (ScanNet is Z-up natively)...")
    mesh, floor_z = reorient_to_zup(mesh)
    print(f"  After shift: Z=[{mesh.bounds[0,2]:.3f}, {mesh.bounds[1,2]:.3f}]m")

    # ---- Read semantic labels from labels PLY ----
    print(f"[import] Reading per-vertex labels: {labels_path}")
    vertex_labels = read_ply_vertex_labels(labels_path)
    print(f"  {len(vertex_labels):,} vertex labels read")

    # ---- Convert vertex → face labels ----
    print("[import] Mapping vertex labels → face labels...")
    raw_face_labels = vertex_labels_to_face_labels(mesh.faces, vertex_labels)
    face_labels = map_scannet_to_aetherscan(raw_face_labels)

    names = {FLOOR: "FLOOR", WALL: "WALL", OBJECT: "OBJECT", CEILING: "CEILING"}
    total = len(face_labels)
    print("  4-class semantic distribution:")
    for lid, name in names.items():
        cnt = int((face_labels == lid).sum())
        print(f"    {name:8s}: {cnt:7,} ({100*cnt/total:.1f}%)")

    # ---- Downsample collision mesh ----
    target_faces = 40_000
    step = max(1, len(mesh.faces) // target_faces)
    coll_faces = mesh.faces[::step]
    coll_mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=coll_faces, process=False)
    coll_labels = face_labels[::step]
    print(f"[import] Collision mesh: {len(coll_mesh.faces):,} faces (1-in-{step} stride)")

    # ---- Export collision PLY ----
    coll_ply = DASHBOARD_MESHES / f"{out_name}_collision.ply"
    print(f"[import] Writing collision PLY → {coll_ply}")
    coll_mesh.export(str(coll_ply))

    # ---- Export labels ----
    # Regenerate labels from the actually-exported mesh to avoid count mismatch
    coll_reloaded = trimesh.load(str(coll_ply))
    final_n = len(coll_reloaded.faces)
    if final_n != len(coll_labels):
        # Pad/truncate to match
        adj = np.full(final_n, OBJECT, dtype=np.int32)
        n = min(final_n, len(coll_labels))
        adj[:n] = coll_labels[:n]
        coll_labels = adj

    labels_out = DASHBOARD_MESHES / f"{out_name}_collision_labels.npy"
    np.save(str(labels_out), coll_labels)
    print(f"[import] Labels saved → {labels_out}")

    # ---- Export visual PLY (vertex colours preserved) ----
    visual_ply = DASHBOARD_MESHES / f"{out_name}.ply"
    print(f"[import] Writing visual PLY → {visual_ply}")
    mesh.export(str(visual_ply))
    print(f"  {visual_ply.stat().st_size / 1e6:.1f} MB")

    print(f"\n✅ ScanNet scene '{out_name}' imported to {DASHBOARD_MESHES}/")
    print(f"   Visual PLY   : {out_name}.ply")
    print(f"   Collision PLY: {out_name}_collision.ply")
    print(f"   Labels NPY   : {out_name}_collision_labels.npy")
    print()
    print("To launch with this scene:")
    print(f"  ./run-aetherscan.sh --dashboard --scene {out_name}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Import ScanNet scene into AetherScan")
    p.add_argument("--ply",      required=True, help="Path to *_vh_clean_2.ply")
    p.add_argument("--labels",   required=True, help="Path to *_vh_clean_2.labels.ply")
    p.add_argument("--out-name", default="scannet_apartment",
                   help="Output scene name (used as --scene flag and file prefix)")
    return p.parse_args()


if __name__ == "__main__":
    process_scene(parse_args())
