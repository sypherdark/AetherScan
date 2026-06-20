"""
Redwood Indoor Dataset mesh loading, spatial queries, and collision sensing.

Meshes are kept in raw global coordinates (no re-centering) so Python physics,
LiDAR, and the dashboard PLY viewer share the same frame.
"""

from __future__ import annotations

import json
import logging
import os
import re
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

REPLICA_MISSING_MSG = (
    "Replica dataset not found locally. Install a scene under "
    "redwood_sim/data/replica/<scene_id>/mesh.ply (and habitat/mesh_semantic.ply), "
    "or run once with network access so Open3D can cache RedwoodIndoor meshes."
)


class MeshUnavailableError(FileNotFoundError):
    """No authentic mesh could be resolved (no procedural fallback)."""


MIN_DASHBOARD_MESH_BYTES = 256
COLLISION_MESH_SUFFIX = "_collision.ply"
SEMANTIC_MESH_SUFFIX = "_semantic.ply"
COLLISION_LABELS_SUFFIX = "_collision_labels.npy"

# Scene alias → Replica folder name under data/replica/{id}/
REPLICA_SCENE_ALIASES: Dict[str, str] = {
    "apartment": "apartment_0",
    "office": "office_0",
    "boardroom": "room_0",
    "livingroom": "apartment_0",
}


def is_collision_mesh_path(path: Path | str) -> bool:
    return Path(path).name.endswith(COLLISION_MESH_SUFFIX)


def is_semantic_mesh_path(path: Path | str) -> bool:
    name = Path(path).name.lower()
    return name.endswith(SEMANTIC_MESH_SUFFIX) or name == "mesh_semantic.ply"


def replica_lookup_ids(scene_id: str) -> List[str]:
    """Candidate Replica directory names for a dashboard scene id."""
    scene_id = scene_id.lower().strip()
    if scene_id.startswith("replica:"):
        rid = scene_id.split(":", 1)[1].strip()
        return [rid] if rid else []
    out: List[str] = []
    alias = REPLICA_SCENE_ALIASES.get(scene_id)
    if alias:
        out.append(alias)
    if scene_id not in out:
        out.append(scene_id)
    return out


def resolve_dashboard_semantic_bundle(
    scene_id: str, meshes_dir: Path
) -> Optional[Tuple[Path, Optional[Path]]]:
    """Return (semantic_ply, semantic_json) from dashboard meshes when present."""
    scene_id = scene_id.lower().strip()
    meshes_dir = Path(meshes_dir)
    semantic_ply = meshes_dir / f"{scene_id}{SEMANTIC_MESH_SUFFIX}"
    if not semantic_ply.is_file():
        habitat = meshes_dir / scene_id / "habitat" / "mesh_semantic.ply"
        if habitat.is_file():
            semantic_ply = habitat
        else:
            return None
    semantic_json = meshes_dir / f"{scene_id}_semantic.json"
    if not semantic_json.is_file():
        alt = meshes_dir / scene_id / "habitat" / "semantic.json"
        semantic_json = alt if alt.is_file() else None
    return semantic_ply, semantic_json


def resolve_collision_label_sidecar(collision_ply: Path) -> Optional[Path]:
    """``{scene}_collision_labels.npy`` beside ``{scene}_collision.ply``."""
    stem = collision_ply.name[: -len(COLLISION_MESH_SUFFIX)]
    sidecar = collision_ply.with_name(f"{stem}{COLLISION_LABELS_SUFFIX}")
    return sidecar if sidecar.is_file() else None


def load_collision_semantic_labels(sidecar: Path, n_triangles: int) -> np.ndarray:
    SemanticClass = _semantic_class()
    labels = np.load(sidecar)
    labels = np.asarray(labels, dtype=np.uint8).reshape(-1)
    if len(labels) == n_triangles:
        return labels
    padded = np.full(n_triangles, int(SemanticClass.UNKNOWN), dtype=np.uint8)
    n_copy = min(n_triangles, len(labels))
    padded[:n_copy] = labels[:n_copy]
    logger.warning(
        "[semantic] Label count %d != triangles %d; padded/truncated",
        len(labels),
        n_triangles,
    )
    return padded


def resolve_dashboard_collision_mesh(
    scene_id: str, meshes_dir: Path
) -> Optional[Path]:
    """Prefer ``{scene}_1_collision.ply`` (versioned), then ``{scene}_collision.ply``;
    fall back to legacy ``{scene}.ply``."""
    scene_id = scene_id.lower().strip()
    meshes_dir = Path(meshes_dir)
    # Versioned collision mesh (e.g. apartment_1_collision.ply) has priority —
    # it is the full Replica export and carries the correct label sidecar.
    versioned = meshes_dir / f"{scene_id}_1{COLLISION_MESH_SUFFIX}"
    if versioned.is_file() and versioned.stat().st_size >= MIN_DASHBOARD_MESH_BYTES:
        return versioned
    collision = meshes_dir / f"{scene_id}{COLLISION_MESH_SUFFIX}"
    if collision.is_file() and collision.stat().st_size >= MIN_DASHBOARD_MESH_BYTES:
        return collision
    legacy = meshes_dir / f"{scene_id}.ply"
    if legacy.is_file() and legacy.stat().st_size >= MIN_DASHBOARD_MESH_BYTES:
        logger.warning(
            "[dual-mesh] No %s — using legacy collision %s",
            collision.name,
            legacy.name,
        )
        return legacy
    return None


def resolve_dashboard_visual_url(scene_id: str, meshes_dir: Path) -> Optional[str]:
    """Dashboard URL for PBR visual (``.glb`` preferred, ``.ply`` fallback).

    Checks several naming conventions so that e.g. ``apartment_1.glb`` is found
    even when the scene_id is just ``"apartment"`` or ``"replica:apartment_1"``.
    """
    scene_id = scene_id.lower().strip()
    # Strip namespace prefix (e.g. "replica:apartment_1" → "apartment_1")
    if ":" in scene_id:
        scene_id = scene_id.split(":", 1)[1]
    meshes_dir = Path(meshes_dir)

    # Candidates in preference order: exact match, then versioned variant (_1),
    # then PLY fallback.
    candidates = [
        meshes_dir / f"{scene_id}.glb",      # exact name match  (apartment_1.glb)
        meshes_dir / f"{scene_id}_1.glb",    # versioned variant  (apartment_1.glb when id=apartment)
    ]
    for glb in candidates:
        if glb.is_file():
            return f"/meshes/{glb.name}"

    ply = meshes_dir / f"{scene_id}.ply"
    if ply.is_file():
        return f"/meshes/{ply.name}"
    return None

REPLICA_ARCHIVE_GLOBS = (
    "replica_v1_0.tar.gz.part*",
    "replica_v1_0.tar.gz",
    "replica_v1_0.tar.gz.*",
)


def _semantic_class():
    from core.semantic_space import SemanticClass

    return SemanticClass

REPLICA_CLASS_KEYWORDS: Tuple[Tuple[str, str], ...] = (
    ("ceiling", "CEILING"),
    ("floor", "FLOOR"),
    ("rug", "FLOOR"),
    ("carpet", "FLOOR"),
    ("wall", "WALL"),
    ("door", "WALL"),
    ("window", "WALL"),
    ("drywall", "WALL"),
    ("panel", "WALL"),
    ("railing", "WALL"),
    ("blinds", "WALL"),
    ("mirror", "WALL"),
    ("table", "OBJECT"),
    ("desk", "OBJECT"),
    ("chair", "OBJECT"),
    ("sofa", "OBJECT"),
    ("bed", "OBJECT"),
    ("cabinet", "OBJECT"),
    ("shelf", "OBJECT"),
    ("counter", "OBJECT"),
    ("appliance", "OBJECT"),
    ("plant", "OBJECT"),
    ("lamp", "OBJECT"),
    ("pillow", "OBJECT"),
    ("box", "OBJECT"),
    ("tv", "OBJECT"),
    ("picture", "OBJECT"),
    ("towel", "OBJECT"),
    ("clothes", "OBJECT"),
)

try:
    import open3d as o3d
except ImportError as exc:
    raise ImportError("open3d is required: pip install open3d") from exc

try:
    import trimesh
except ImportError:
    trimesh = None

REDWOOD_MESH_URLS_LEGACY = {
    "apartment": "http://redwood-data.org/indoor/models/mesh/apartment.ply",
    "boardroom": "http://redwood-data.org/indoor/models/mesh/boardroom.ply",
}

OPEN3D_REDWOOD_SCENES = {
    "apartment": "RedwoodIndoorLivingRoom1",
    "livingroom": "RedwoodIndoorLivingRoom1",
    "office": "RedwoodIndoorOffice1",
    "boardroom": "RedwoodIndoorOffice2",
    "kitchen": "RedwoodIndoorLivingRoom2",
    "lobby": "RedwoodIndoorOffice2",
    "hallway": "RedwoodIndoorOffice1",
    "copyroom": "RedwoodIndoorOffice1",
}


@dataclass
class SceneBounds:
    min_corner: np.ndarray
    max_corner: np.ndarray
    center: np.ndarray
    extent: np.ndarray

    @property
    def diagonal(self) -> float:
        return float(np.linalg.norm(self.extent))


@dataclass
class MeshStats:
    vertices: int
    triangles: int
    path: str
    bounds_min: np.ndarray
    bounds_max: np.ndarray


@dataclass
class ReplicaSceneAssets:
    """Paths inside an extracted Replica scene directory."""

    scene_id: str
    root_dir: Path
    visual_mesh: Path
    semantic_mesh: Path
    semantic_json: Optional[Path] = None


def _replica_class_name_to_semantic(name: str) -> int:
    SemanticClass = _semantic_class()
    lower = name.lower().strip()
    for keyword, sem_name in REPLICA_CLASS_KEYWORDS:
        if keyword in lower:
            return int(getattr(SemanticClass, sem_name))
    if any(k in lower for k in ("void", "unlabeled", "unknown")):
        return int(SemanticClass.UNKNOWN)
    return int(SemanticClass.OBJECT)


def _walk_replica_semantic_json(node: object, out: Dict[int, int]) -> None:
    if isinstance(node, dict):
        raw_id = node.get("id", node.get("objectId", node.get("segmentId")))
        raw_name = node.get("class", node.get("name", node.get("label")))
        if raw_id is not None and raw_name:
            try:
                oid = int(raw_id)
            except (TypeError, ValueError):
                oid = None
            if oid is not None:
                out[oid] = _replica_class_name_to_semantic(str(raw_name))
        for value in node.values():
            _walk_replica_semantic_json(value, out)
    elif isinstance(node, list):
        for item in node:
            _walk_replica_semantic_json(item, out)


def _load_replica_instance_map(json_path: Optional[Path]) -> Dict[int, int]:
    if json_path is None or not json_path.is_file():
        return {}
    try:
        data = json.loads(json_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        logger.warning("[replica] Could not parse %s: %s", json_path, exc)
        return {}
    mapping: Dict[int, int] = {}
    _walk_replica_semantic_json(data, mapping)
    logger.info("[replica] Loaded %d semantic instance mappings from %s", len(mapping), json_path.name)
    return mapping


def _fan_triangulate_object_ids(face_vertex_counts: List[int], face_object_ids: List[int]) -> np.ndarray:
    """Match Open3D fan triangulation: (v0,v1,v2), (v0,v2,v3), ... per n-gon face."""
    tri_object_ids: List[int] = []
    for n_verts, object_id in zip(face_vertex_counts, face_object_ids):
        if n_verts < 3:
            continue
        for _ in range(n_verts - 2):
            tri_object_ids.append(object_id)
    return np.asarray(tri_object_ids, dtype=np.int32)


# PLY scalar type → (struct format char, byte size, numpy dtype string).
_PLY_TYPES: Dict[str, Tuple[str, int, str]] = {
    "char": ("b", 1, "i1"), "int8": ("b", 1, "i1"),
    "uchar": ("B", 1, "u1"), "uint8": ("B", 1, "u1"),
    "short": ("h", 2, "i2"), "int16": ("h", 2, "i2"),
    "ushort": ("H", 2, "u2"), "uint16": ("H", 2, "u2"),
    "int": ("i", 4, "i4"), "int32": ("i", 4, "i4"),
    "uint": ("I", 4, "u4"), "uint32": ("I", 4, "u4"),
    "float": ("f", 4, "f4"), "float32": ("f", 4, "f4"),
    "double": ("d", 8, "f8"), "float64": ("d", 8, "f8"),
}


def _parse_ply_elements(header: str) -> "List[dict]":
    """Parse a PLY header into ordered elements, each with its property list.

    Each element: {"name", "count", "props":[...]}.  A scalar property is
    {"kind":"scalar","type","name"}; a list property is
    {"kind":"list","count_type","value_type","name"}.  Parsing the real property
    types (instead of assuming everything is a 4-byte float/int) is essential:
    Replica vertices mix float xyz/normals with uchar rgb, and faces use a uint8
    list count plus a uint16 object_id — the old fixed-size assumptions misaligned
    the binary read and crashed scene loading.
    """
    elements: List[dict] = []
    cur: Optional[dict] = None
    for raw in header.splitlines():
        parts = raw.split()
        if not parts:
            continue
        if parts[0] == "element":
            cur = {"name": parts[1], "count": int(parts[2]), "props": []}
            elements.append(cur)
        elif parts[0] == "property" and cur is not None:
            if parts[1] == "list":
                cur["props"].append({
                    "kind": "list", "count_type": parts[2],
                    "value_type": parts[3], "name": parts[4],
                })
            else:
                cur["props"].append({"kind": "scalar", "type": parts[1], "name": parts[2]})
    return elements


def _read_replica_face_object_ids(ply_path: Path) -> Tuple[np.ndarray, int]:
    """
    Parse Replica habitat/mesh_semantic.ply face ``object_id`` properties.
    Returns per-triangle object_id array (after fan triangulation) and vertex count.
    """
    data = ply_path.read_bytes()
    header_end = data.find(b"end_header\n")
    if header_end < 0:
        header_end = data.find(b"end_header\r\n")
    if header_end < 0:
        raise ValueError(f"Invalid PLY header: {ply_path}")
    header = data[:header_end].decode("ascii", errors="replace")
    if "object_id" not in header:
        raise ValueError(f"PLY missing face object_id property: {ply_path}")

    is_ascii = "format ascii" in header
    little_endian = "binary_little_endian" in header
    elements = _parse_ply_elements(header)
    vtx_el = next((e for e in elements if e["name"] == "vertex"), None)
    face_el = next((e for e in elements if e["name"] == "face"), None)
    if vtx_el is None or face_el is None:
        raise ValueError(f"PLY missing vertex/face element: {ply_path}")
    n_verts = vtx_el["count"]
    n_faces = face_el["count"]

    # Body begins right after the "end_header\n" terminator.
    body = data[header_end + len(b"end_header\n"):]
    if body.startswith(b"\r"):
        body = body[1:]

    face_counts: List[int] = []
    face_object_ids: List[int] = []

    if is_ascii:
        lines = body.decode("ascii", errors="replace").splitlines()
        idx = n_verts  # skip vertex lines
        for _ in range(n_faces):
            if idx >= len(lines):
                break
            parts = lines[idx].split()
            idx += 1
            if len(parts) < 2:
                continue
            face_counts.append(int(parts[0]))
            face_object_ids.append(int(parts[-1]))
        tri_ids = _fan_triangulate_object_ids(face_counts, face_object_ids)
        return tri_ids, n_verts

    # ── Binary: compute the REAL vertex stride from the property types ─────────
    endian = "<" if little_endian else ">"
    np_end = "<" if little_endian else ">"
    vert_stride = 0
    for p in vtx_el["props"]:
        if p["kind"] != "scalar":
            raise ValueError(f"Unexpected list property in vertex element: {ply_path}")
        vert_stride += _PLY_TYPES[p["type"]][1]
    faces_body = body[n_verts * vert_stride:]

    # Face element = one list property (vertex indices) followed by scalar
    # properties (object_id is one of them).
    list_prop = next((p for p in face_el["props"] if p["kind"] == "list"), None)
    if list_prop is None:
        raise ValueError(f"Face element has no list property: {ply_path}")
    trailing = [p for p in face_el["props"] if p["kind"] == "scalar"]
    oid_pos = next((i for i, p in enumerate(trailing) if p["name"] == "object_id"), -1)
    if oid_pos < 0:
        raise ValueError(f"Face element has no object_id scalar: {ply_path}")

    cnt_fmt, cnt_size, cnt_np = _PLY_TYPES[list_prop["count_type"]]
    idx_fmt, idx_size, idx_np = _PLY_TYPES[list_prop["value_type"]]

    # ── Fast path: uniform triangle fan (the Replica common case) ──────────────
    # Build a packed numpy record dtype matching the on-disk layout and read all
    # faces in one vectorised call; verify every face really is a triangle.
    try:
        fields = [("__cnt", np_end + cnt_np), ("__idx", np_end + idx_np, (3,))]
        for p in trailing:
            fields.append((p["name"], np_end + _PLY_TYPES[p["type"]][2]))
        rec = np.dtype(fields)
        if len(faces_body) >= n_faces * rec.itemsize:
            arr = np.frombuffer(faces_body, dtype=rec, count=n_faces)
            if bool(np.all(arr["__cnt"] == 3)):
                # All triangles → fan triangulation is identity.
                return arr["object_id"].astype(np.int64), n_verts
    except (ValueError, KeyError):
        pass  # fall through to the robust per-face parser

    # ── Robust path: variable-length faces ────────────────────────────────────
    offset = 0
    blen = len(faces_body)
    trailing_fmt = "".join(_PLY_TYPES[p["type"]][0] for p in trailing)
    trailing_size = sum(_PLY_TYPES[p["type"]][1] for p in trailing)
    for _ in range(n_faces):
        if offset + cnt_size > blen:
            break
        (count,) = struct.unpack_from(endian + cnt_fmt, faces_body, offset)
        offset += cnt_size + count * idx_size
        if offset + trailing_size > blen:
            break
        vals = struct.unpack_from(endian + trailing_fmt, faces_body, offset)
        offset += trailing_size
        face_counts.append(int(count))
        face_object_ids.append(int(vals[oid_pos]))

    tri_ids = _fan_triangulate_object_ids(face_counts, face_object_ids)
    return tri_ids, n_verts


def _build_triangle_semantic_labels(
    semantic_ply: Path,
    instance_map: Dict[int, int],
) -> np.ndarray:
    SemanticClass = _semantic_class()
    tri_object_ids, _ = _read_replica_face_object_ids(semantic_ply)
    labels = np.full(len(tri_object_ids), int(SemanticClass.UNKNOWN), dtype=np.uint8)
    for i, oid in enumerate(tri_object_ids):
        sem = instance_map.get(int(oid))
        if sem is None:
            sem = int(SemanticClass.OBJECT) if oid > 0 else int(SemanticClass.UNKNOWN)
        labels[i] = int(sem)
    return labels


def _replica_normalize_matrix(vertices: np.ndarray) -> np.ndarray:
    """4x4 transform: center XY at origin, floor (min Z) at Z=0."""
    verts = np.asarray(vertices, dtype=np.float64)
    mn = verts.min(axis=0)
    mx = verts.max(axis=0)
    cx = 0.5 * (float(mn[0]) + float(mx[0]))
    cy = 0.5 * (float(mn[1]) + float(mx[1]))
    floor_z = float(mn[2])
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = -cx
    T[1, 3] = -cy
    T[2, 3] = -floor_z
    return T


def _apply_transform_vertices(vertices: np.ndarray, transform: np.ndarray) -> np.ndarray:
    v = np.asarray(vertices, dtype=np.float64)
    ones = np.ones((len(v), 1), dtype=np.float64)
    hom = np.hstack([v, ones])
    # Slice [:, :3] produces a non-contiguous view; force C-contiguous copy so
    # that o3d.utility.Vector3dVector() doesn't crash on macOS.
    out = np.ascontiguousarray((hom @ transform.T)[:, :3], dtype=np.float64)
    return out


def _apply_transform_to_mesh(mesh: o3d.geometry.TriangleMesh, transform: np.ndarray) -> None:
    # Clearing pre-computed normals before vertex reassignment prevents a
    # segfault in open3d on macOS when the mesh already has normals attached.
    # They are recomputed downstream in RedwoodScene.__init__.
    mesh.vertex_normals = o3d.utility.Vector3dVector([])
    mesh.triangle_normals = o3d.utility.Vector3dVector([])
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    mesh.vertices = o3d.utility.Vector3dVector(_apply_transform_vertices(verts, transform))


def purge_replica_archive_artifacts() -> int:
    """
    Remove stale partial Replica archive chunks under data/replica/ (no network I/O).
    Returns bytes reclaimed.
    """
    replica_base = Path(__file__).parent / "data" / "replica"
    if not replica_base.is_dir():
        return 0

    freed = 0
    for pattern in REPLICA_ARCHIVE_GLOBS:
        for path in replica_base.glob(pattern):
            if path.is_file():
                freed += path.stat().st_size
                path.unlink()
                logger.info("[replica] Removed stale archive artifact: %s", path.name)

    for child in list(replica_base.iterdir()):
        if not child.is_dir():
            continue
        mesh = child / "mesh.ply"
        semantic = child / "habitat" / "mesh_semantic.ply"
        if mesh.is_file() and semantic.is_file():
            continue
        for f in child.rglob("*"):
            if f.is_file():
                freed += f.stat().st_size
        shutil.rmtree(child, ignore_errors=True)
        logger.info("[replica] Removed incomplete scene directory: %s", child.name)

    return freed


def download_replica_scene(scene_id: str) -> Optional[ReplicaSceneAssets]:
    """
    Resolve locally installed Replica assets under data/replica/{id}/.

    Does not download over the network. Returns None when assets are missing so
    callers must use cached Redwood or bundled Replica assets.
    """
    scene_id = scene_id.strip().lower()
    if scene_id.startswith("replica:"):
        scene_id = scene_id.split(":", 1)[1].strip()

    replica_base = Path(__file__).parent / "data" / "replica"
    purge_replica_archive_artifacts()

    scene_dir = replica_base / scene_id
    visual = scene_dir / "mesh.ply"
    semantic = scene_dir / "habitat" / "mesh_semantic.ply"

    if not visual.is_file():
        logger.debug(REPLICA_MISSING_MSG)   # not an error — dashboard uses its own pipeline
        return None

    if not semantic.is_file():
        logger.debug(
            "Replica collision mesh missing at %s. %s",
            semantic,
            REPLICA_MISSING_MSG,
        )
        return None

    logger.info("[replica] Using local scene '%s' at %s", scene_id, scene_dir)
    return _replica_assets_from_dir(scene_id, scene_dir)


def _replica_assets_from_dir(scene_id: str, scene_dir: Path) -> ReplicaSceneAssets:
    semantic_json = scene_dir / "habitat" / "semantic.json"
    if not semantic_json.is_file():
        semantic_json = scene_dir / "semantic.json"
    return ReplicaSceneAssets(
        scene_id=scene_id,
        root_dir=scene_dir,
        visual_mesh=scene_dir / "mesh.ply",
        semantic_mesh=scene_dir / "habitat" / "mesh_semantic.ply",
        semantic_json=semantic_json if semantic_json.is_file() else None,
    )


def _triangulated_cache_path(source: Path) -> Path:
    if source.stem.endswith("_triangulated"):
        return source
    return source.with_name(f"{source.stem}_triangulated.ply")


def canonicalize_indoor_mesh_z_up(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    """
    Poisson Redwood meshes arrive Y-up (~3 m on Y, long axis on Z).

    Rotate to Z-up ROS (height on +Z), floor at z=0, XY centered. Idempotent.
    """
    if mesh.is_empty() or len(mesh.vertices) == 0:
        return mesh

    verts = np.asarray(mesh.vertices, dtype=np.float64)
    mn, mx = verts.min(axis=0), verts.max(axis=0)
    ext = mx - mn

    # Y-up scan: short Y (~3 m ceiling), long Z corridor — not yet Z-up.
    if ext[2] > ext[1] and ext[1] < 4.5:
        R = mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))
        mesh.rotate(R, center=(0.0, 0.0, 0.0))
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        mn, mx = verts.min(axis=0), verts.max(axis=0)

    center = 0.5 * (mn + mx)
    mesh.translate([-center[0], -center[1], -mn[2]])
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh


def reconstruct_mesh_from_point_cloud(
    pc: o3d.geometry.PointCloud,
    voxel_size: float = 0.06,
    poisson_depth: int = 8,
) -> o3d.geometry.TriangleMesh:
    """
    Build a watertight triangle mesh from a dense scan point cloud.

    Open3D RedwoodIndoor datasets ship vertex-only PLYs; physics raycasting requires triangles.
    """
    if pc.is_empty():
        raise ValueError("Cannot reconstruct mesh from empty point cloud")

    if len(pc.points) > 120_000:
        pc = pc.voxel_down_sample(float(voxel_size))
        logger.info("[mesh] Downsampled point cloud to %d points", len(pc.points))

    pc.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=max(voxel_size * 3.0, 0.12),
            max_nn=30,
        )
    )
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pc, depth=int(poisson_depth)
    )
    if len(densities) > 0 and len(densities) == len(mesh.vertices):
        dens = np.asarray(densities, dtype=np.float64)
        keep = dens >= float(np.quantile(dens, 0.03))
        if int(np.sum(keep)) > 500:
            mesh.remove_vertices_by_mask(~keep)

    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) == 0:
        raise ValueError("Poisson reconstruction produced no triangles")
    mesh.compute_vertex_normals()
    logger.info(
        "[mesh] Reconstructed surface: %d vertices, %d triangles",
        len(mesh.vertices),
        len(mesh.triangles),
    )
    return mesh


def load_triangle_mesh(path: Path) -> o3d.geometry.TriangleMesh:
    """Load geometry from disk; auto-reconstruct if the file is a point cloud."""
    path = Path(path)
    tri_cache = _triangulated_cache_path(path)
    if tri_cache.is_file() and tri_cache.stat().st_size > 50_000:
        cached = o3d.io.read_triangle_mesh(str(tri_cache))
        if len(cached.triangles) > 0:
            return canonicalize_indoor_mesh_z_up(cached)

    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty() and trimesh is not None:
        tm = trimesh.load(str(path), force="mesh")
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(tuple(tm.geometry.values()))
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(np.asarray(tm.vertices, dtype=np.float64)),
            o3d.utility.Vector3iVector(np.asarray(tm.faces, dtype=np.int32)),
        )

    if mesh.is_empty():
        pc = o3d.io.read_point_cloud(str(path))
        if not pc.is_empty():
            mesh = reconstruct_mesh_from_point_cloud(pc)
        else:
            raise ValueError(f"Failed to parse mesh or point cloud: {path}")
    elif len(mesh.triangles) == 0 and len(mesh.vertices) > 0:
        logger.info("[mesh] %s is a point cloud — reconstructing triangle surface", path.name)
        pc = o3d.io.read_point_cloud(str(path))
        if pc.is_empty():
            pc = o3d.geometry.PointCloud()
            pc.points = mesh.vertices
            if mesh.has_vertex_colors():
                pc.colors = mesh.vertex_colors
        mesh = reconstruct_mesh_from_point_cloud(pc)

    if len(mesh.triangles) == 0:
        raise ValueError(f"Mesh has no triangles after load: {path}")

    mesh = canonicalize_indoor_mesh_z_up(mesh)

    if tri_cache != path and tri_cache != path.resolve():
        try:
            tri_cache.parent.mkdir(parents=True, exist_ok=True)
            o3d.io.write_triangle_mesh(str(tri_cache), mesh)
            logger.info("[mesh] Cached triangulated surface → %s", tri_cache)
        except OSError as exc:
            logger.warning("[mesh] Could not cache triangulated mesh: %s", exc)

    return mesh


def load_semantic_mesh(path: Path) -> o3d.geometry.TriangleMesh:
    """
    Load Replica-style semantic collision mesh without Poisson canonicalization.

    Expects triangle soup in global coordinates (Replica habitat/mesh_semantic.ply).
    """
    path = Path(path)
    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty() and trimesh is not None:
        tm = trimesh.load(str(path), force="mesh")
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(tuple(tm.geometry.values()))
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(np.asarray(tm.vertices, dtype=np.float64)),
            o3d.utility.Vector3iVector(np.asarray(tm.faces, dtype=np.int32)),
        )
    if mesh.is_empty() or len(mesh.triangles) == 0:
        raise ValueError(f"Semantic mesh has no triangles: {path}")
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh


def load_collision_mesh(path: Path) -> o3d.geometry.TriangleMesh:
    """
    Load a pre-built watertight collision surface in Z-up ROS coordinates.

    Skips Poisson canonicalization — collision assets are authored or built via
    ``scripts/build_collision_mesh.py``.
    """
    path = Path(path)
    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty() and trimesh is not None:
        tm = trimesh.load(str(path), force="mesh")
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(tuple(tm.geometry.values()))
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(np.asarray(tm.vertices, dtype=np.float64)),
            o3d.utility.Vector3iVector(np.asarray(tm.faces, dtype=np.int32)),
        )
    if mesh.is_empty() or len(mesh.triangles) == 0:
        raise ValueError(f"Collision mesh has no triangles: {path}")
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh


def ensure_triangle_mesh_file(source: Path) -> Path:
    """Return path to a triangle-mesh PLY (reconstructs and caches when needed)."""
    source = Path(source)
    existing = o3d.io.read_triangle_mesh(str(source))
    if len(existing.triangles) > 0:
        return source
    load_triangle_mesh(source)
    tri_cache = _triangulated_cache_path(source)
    return tri_cache if tri_cache.is_file() else source


class RedwoodScene:
    """
    Loads indoor mesh in global Z-up ROS coordinates, exposes tensor raycasting.

    Vertices are stored as (x, y, z) with the floor in the XY plane and +Z up.
    The dashboard maps Z-up → Three.js Y-up via group Rx(-π/2) and (x, z, -y).
    """

    def __init__(
        self,
        mesh_path: str | Path,
        voxel_downsample: float = 0.02,
        compute_normals: bool = True,
        center_mesh: bool = False,
        visual_mesh_path: Optional[str | Path] = None,
        replica_normalize: bool = False,
        triangle_semantics: Optional[np.ndarray] = None,
        visual_mesh_url: Optional[str] = None,
    ):
        self.mesh_path = Path(mesh_path)
        if not self.mesh_path.is_file():
            raise FileNotFoundError(f"Mesh not found: {self.mesh_path}")

        self.visual_mesh_path: Optional[Path] = (
            Path(visual_mesh_path) if visual_mesh_path is not None else None
        )
        self.visual_mesh_url: Optional[str] = visual_mesh_url
        self._triangle_semantics: Optional[np.ndarray] = (
            np.asarray(triangle_semantics, dtype=np.uint8)
            if triangle_semantics is not None
            else None
        )
        self._normalize_transform = np.eye(4, dtype=np.float64)

        self._mesh_o3d = self._load_mesh(self.mesh_path)
        if replica_normalize:
            self._apply_replica_normalization()
        elif center_mesh:
            self._center_mesh_legacy()

        if self._triangle_semantics is None and is_collision_mesh_path(self.mesh_path):
            sidecar = resolve_collision_label_sidecar(self.mesh_path)
            if sidecar is not None:
                self._triangle_semantics = load_collision_semantic_labels(
                    sidecar, len(self._mesh_o3d.triangles)
                )
                logger.info(
                    "[semantic] Linked %d primitive records from %s",
                    len(self._triangle_semantics),
                    sidecar.name,
                )

        if self.visual_mesh_path is not None and self.visual_mesh_path.is_file():
            self._visual_mesh_o3d = self._load_mesh(self.visual_mesh_path)
            if replica_normalize:
                _apply_transform_to_mesh(self._visual_mesh_o3d, self._normalize_transform)
        else:
            self._visual_mesh_o3d = None

        if self._triangle_semantics is not None and voxel_downsample > 0:
            logger.info(
                "[replica] Skipping voxel downsample to preserve triangle semantic ids"
            )
            voxel_downsample = 0.0

        if (
            self._triangle_semantics is not None
            and len(self._triangle_semantics) != len(self._mesh_o3d.triangles)
        ):
            n_tri = len(self._mesh_o3d.triangles)
            logger.warning(
                "[replica] Triangle label count %d != mesh triangles %d; padding/truncating",
                len(self._triangle_semantics),
                n_tri,
            )
            padded = np.full(n_tri, int(_semantic_class().UNKNOWN), dtype=np.uint8)
            n_copy = min(n_tri, len(self._triangle_semantics))
            padded[:n_copy] = self._triangle_semantics[:n_copy]
            self._triangle_semantics = padded

        if (
            voxel_downsample > 0
            and len(self._mesh_o3d.vertices) > 100
            and not is_collision_mesh_path(self.mesh_path)
        ):
            self._mesh_o3d = self._mesh_o3d.simplify_vertex_clustering(
                voxel_size=float(voxel_downsample)
            )
        if compute_normals or not self._mesh_o3d.has_vertex_normals():
            self._mesh_o3d.compute_vertex_normals()
        if not self._mesh_o3d.has_triangle_normals():
            self._mesh_o3d.compute_triangle_normals()

        self._scene = o3d.t.geometry.RaycastingScene()
        self._tm = o3d.t.geometry.TriangleMesh.from_legacy(self._mesh_o3d)
        self._scene.add_triangles(self._tm)

        self.bounds = self._compute_bounds()
        self.stats = MeshStats(
            vertices=len(self._mesh_o3d.vertices),
            triangles=len(self._mesh_o3d.triangles),
            path=str(self.mesh_path),
            bounds_min=self.bounds.min_corner.copy(),
            bounds_max=self.bounds.max_corner.copy(),
        )

        n_verts = len(self._mesh_o3d.vertices)
        # Collision/semantic meshes are used for navigation analysis only —
        # a modest point cloud density is sufficient and keeps DBSCAN fast.
        # Visual / non-collision meshes benefit from a richer sample.
        if is_collision_mesh_path(self.mesh_path):
            n_samples = min(25_000, max(2_000, n_verts * 2))
        else:
            n_samples = min(200_000, max(2_000, n_verts * 40))
        self._point_cloud = self._mesh_o3d.sample_points_uniformly(number_of_points=n_samples)
        self._point_cloud.estimate_normals()

    @property
    def mesh(self) -> o3d.geometry.TriangleMesh:
        return self._mesh_o3d

    @property
    def visual_mesh(self) -> Optional[o3d.geometry.TriangleMesh]:
        return self._visual_mesh_o3d

    @property
    def point_cloud(self) -> o3d.geometry.PointCloud:
        return self._point_cloud

    @classmethod
    def from_replica(
        cls,
        assets: ReplicaSceneAssets,
        voxel_downsample: float = 0.02,
        dashboard_meshes_dir: Optional[Path] = None,
    ) -> RedwoodScene:
        """
        Load Replica visual (mesh.ply) + collision (habitat/mesh_semantic.ply) with normalization.
        """
        if not assets.semantic_mesh.is_file():
            raise FileNotFoundError(f"Replica semantic mesh missing: {assets.semantic_mesh}")

        instance_map = _load_replica_instance_map(assets.semantic_json)
        tri_labels = _build_triangle_semantic_labels(assets.semantic_mesh, instance_map)

        visual_url: Optional[str] = None
        visual_path = assets.visual_mesh if assets.visual_mesh.is_file() else None
        scene = cls(
            assets.semantic_mesh,
            voxel_downsample=voxel_downsample,
            replica_normalize=True,
            triangle_semantics=tri_labels,
            visual_mesh_path=visual_path,
        )
        if visual_path is not None and dashboard_meshes_dir is not None:
            dashboard_meshes_dir.mkdir(parents=True, exist_ok=True)
            out_name = f"replica_{assets.scene_id}.ply"
            out_path = dashboard_meshes_dir / out_name
            print(f"[replica] Exporting visual mesh for dashboard → {out_path}")
            mesh_out = scene.visual_mesh or scene.mesh
            o3d.io.write_triangle_mesh(str(out_path), mesh_out)
            scene.visual_mesh_url = f"/meshes/{out_name}"
        return scene

    def classify_primitive(
        self, primitive_id: int, hit: np.ndarray, normal: np.ndarray, origin: np.ndarray
    ):
        SemanticClass = _semantic_class()
        if (
            self._triangle_semantics is not None
            and 0 <= primitive_id < len(self._triangle_semantics)
        ):
            sem = SemanticClass(int(self._triangle_semantics[primitive_id]))
            # When the label is a genuine semantic class (not UNKNOWN), use it.
            # UNKNOWN means the sidecar had no data for this primitive, so fall
            # through to the normal-based geometric classifier below.
            if sem != SemanticClass.UNKNOWN:
                conf = 1.0 if sem == SemanticClass.WALL else 0.92
                return sem, int(primitive_id), conf

        # Geometric fallback: classify by surface normal and hit altitude.
        # Avoid np.linalg.norm overhead (called millions of times per second):
        # use the squared magnitude inline instead.
        nx, ny, nz_raw = float(normal[0]), float(normal[1]), float(normal[2])
        mag = (nx * nx + ny * ny + nz_raw * nz_raw) ** 0.5 + 1e-9
        nz = abs(nz_raw / mag)
        if nz >= 0.72:
            if hit[2] < origin[2] - 0.05:
                return SemanticClass.FLOOR, -1, 0.85
            return SemanticClass.CEILING, -1, 0.85
        if nz <= 0.35:
            return SemanticClass.WALL, -1, 0.75
        return SemanticClass.OBJECT, -1, 0.7

    def _apply_replica_normalization(self) -> None:
        verts = np.asarray(self._mesh_o3d.vertices, dtype=np.float64)
        self._normalize_transform = _replica_normalize_matrix(verts)
        _apply_transform_to_mesh(self._mesh_o3d, self._normalize_transform)
        print(
            "[replica] Normalized scene: XY centered, floor Z=0 "
            f"(translation {self._normalize_transform[:3, 3]})"
        )

    @property
    def has_triangle_semantics(self) -> bool:
        return self._triangle_semantics is not None and len(self._triangle_semantics) > 0

    @staticmethod
    def _load_mesh(path: Path) -> o3d.geometry.TriangleMesh:
        if is_collision_mesh_path(path):
            return load_collision_mesh(path)
        if is_semantic_mesh_path(path):
            return load_semantic_mesh(path)
        return load_triangle_mesh(path)

    def _center_mesh_legacy(self) -> None:
        """Optional legacy centering — disabled by default for dashboard alignment."""
        verts = np.asarray(self._mesh_o3d.vertices, dtype=np.float64)
        verts -= verts.mean(axis=0)
        verts[:, 2] -= float(verts[:, 2].min()) - 0.05
        self._mesh_o3d.vertices = o3d.utility.Vector3dVector(verts)

    def _compute_bounds(self) -> SceneBounds:
        verts = np.asarray(self._mesh_o3d.vertices)
        mn = verts.min(axis=0)
        mx = verts.max(axis=0)
        return SceneBounds(
            min_corner=mn,
            max_corner=mx,
            center=0.5 * (mn + mx),
            extent=mx - mn,
        )

    def log_mesh_info(self) -> None:
        s = self.stats
        print(
            f"[mesh] {s.path}\n"
            f"       vertices={s.vertices:,}  triangles={s.triangles:,}\n"
            f"       bounds min={s.bounds_min} max={s.bounds_max}\n"
            f"       frame=global (center_mesh=False)"
        )
        if self.visual_mesh_url:
            print(f"       dashboard_visual={self.visual_mesh_url}")
        if self.has_triangle_semantics:
            uniq = len(np.unique(self._triangle_semantics))
            print(f"       semantic_triangles={len(self._triangle_semantics):,} ({uniq} classes)")
        if s.triangles < 1:
            raise RuntimeError(f"Mesh has no triangles — raycasting will not work: {s.path}")
        center = self.bounds.center
        probe_o = center + np.array([0.0, 0.0, 0.5], dtype=np.float64)
        probe_d = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
        dist, _, _, _ = self.cast_rays(
            np.tile(probe_o, (3, 1)).astype(np.float64),
            probe_d,
            max_distance=20.0,
        )
        finite = dist[np.isfinite(dist) & (dist < 19.9)]
        if len(finite) == 0:
            raise RuntimeError(
                f"RaycastingScene found no hits from mesh center — check PLY: {s.path}"
            )
        print(
            f"       raycast_ok: {len(finite)} hits from center "
            f"(sample ranges {finite.min():.2f}–{finite.max():.2f} m)"
        )

    def cast_rays(
        self,
        origins: np.ndarray,
        directions: np.ndarray,
        max_distance: float = 30.0,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Cast rays against triangle mesh. Returns distances, hit points,
        **triangle primitive normals**, and primitive (triangle) ids.
        """
        origins = np.asarray(origins, dtype=np.float32)
        directions = np.asarray(directions, dtype=np.float32)
        dirs = directions / (np.linalg.norm(directions, axis=1, keepdims=True) + 1e-9)

        rays = np.concatenate([origins, dirs], axis=1)
        rays_t = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)

        ans = self._scene.cast_rays(rays_t)
        t_hit = ans["t_hit"].numpy()
        distances = np.where(np.isfinite(t_hit), t_hit, max_distance)
        distances = np.clip(distances, 0.0, max_distance)
        points = origins.astype(np.float64) + dirs.astype(np.float64) * distances[:, None]

        normals = self._normals_from_cast(ans, points, distances, max_distance)
        primitive_ids = np.full(len(points), -1, dtype=np.int32)
        if "primitive_ids" in ans:
            raw_ids = ans["primitive_ids"].numpy()
            if raw_ids is not None and len(raw_ids) == len(points):
                primitive_ids = np.asarray(raw_ids, dtype=np.int32)
        return distances, points, normals, primitive_ids

    def _normals_from_cast(
        self,
        ans: dict,
        points: np.ndarray,
        distances: np.ndarray,
        max_distance: float,
    ) -> np.ndarray:
        n_rays = len(points)
        out = np.zeros((n_rays, 3), dtype=np.float64)
        hit_mask = np.isfinite(distances) & (distances < max_distance - 0.01)

        if not np.any(hit_mask):
            return out

        prim_normals = None
        if "primitive_normals" in ans:
            raw = ans["primitive_normals"].numpy()
            if raw is not None and len(raw) == n_rays:
                prim_normals = np.asarray(raw, dtype=np.float64)

        if prim_normals is not None:
            # Vectorised normalisation — avoids a Python loop over every ray hit.
            hit_idx = np.where(hit_mask)[0]
            if len(hit_idx):
                ns = prim_normals[hit_idx]           # (k, 3)
                mag = np.sqrt((ns * ns).sum(axis=1, keepdims=True))  # (k, 1)
                valid = (mag[:, 0] > 1e-6)
                good = hit_idx[valid]
                bad  = hit_idx[~valid]
                if len(good):
                    out[good] = ns[valid] / mag[valid]
                for i in bad:
                    out[i] = self._fallback_normal_at_point(points[i])
            return out

        for i in np.where(hit_mask)[0]:
            out[i] = self._fallback_normal_at_point(points[i])
        return out

    def _fallback_normal_at_point(self, point: np.ndarray) -> np.ndarray:
        kdt = o3d.geometry.KDTreeFlann(self._point_cloud)
        nrms = np.asarray(self._point_cloud.normals)
        _, idx, _ = kdt.search_knn_vector_3d(point, 8)
        n = nrms[idx].mean(axis=0)
        n_norm = np.linalg.norm(n)
        return n / n_norm if n_norm > 1e-6 else np.array([0.0, 0.0, 1.0])

    def lidar_scan_2d(
        self,
        origin: np.ndarray,
        height: float,
        num_rays: int = 360,
        z_offset: float = 0.0,
        fov_rad: float = 2.0 * np.pi,
        max_range: float = 20.0,
    ) -> np.ndarray:
        origin = np.asarray(origin, dtype=np.float64).copy()
        origin[2] = height + z_offset
        angles = np.linspace(-fov_rad / 2, fov_rad / 2, num_rays, endpoint=False)
        directions = np.stack(
            [np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1
        )
        _, points, _, _ = self.cast_rays(
            origin[None, :].repeat(num_rays, 0), directions, max_range
        )
        return points

    def proximity_distance(self, position: np.ndarray, num_dirs: int = 26) -> float:
        position = np.asarray(position, dtype=np.float64)
        phi = np.linspace(0, np.pi, int(np.sqrt(num_dirs)) + 1)
        theta = np.linspace(0, 2 * np.pi, int(np.sqrt(num_dirs)) + 1)
        dirs = []
        for p in phi:
            for t in theta:
                dirs.append(
                    [np.sin(p) * np.cos(t), np.sin(p) * np.sin(t), np.cos(p)]
                )
        dirs = np.asarray(dirs)
        dists, _, _, _ = self.cast_rays(
            np.repeat(position[None, :], len(dirs), axis=0), dirs, max_distance=5.0
        )
        return float(np.min(dists))

    def is_inside_bounds(self, position: np.ndarray, margin: float = 0.35) -> bool:
        p = np.asarray(position)
        return bool(
            np.all(p >= self.bounds.min_corner + margin)
            and np.all(p <= self.bounds.max_corner - margin)
        )


def _download_open3d_redwood(scene_name: str, cache_dir: Path) -> Path:
    o3d_name = OPEN3D_REDWOOD_SCENES.get(scene_name.lower(), "RedwoodIndoorOffice1")
    dataset_cls = getattr(o3d.data, o3d_name, None)
    if dataset_cls is None:
        raise RuntimeError(f"Open3D dataset class not found: {o3d_name}")

    print(f"Resolving Open3D dataset '{o3d_name}'...")
    ds = dataset_cls(data_root=str(cache_dir / "open3d_cache"))
    if hasattr(ds, "download"):
        ds.download()

    ply_path = Path(getattr(ds, "point_cloud_path", "") or "")
    if not ply_path.is_file():
        search_roots = [
            Path(getattr(ds, "extract_dir", "")),
            Path(getattr(ds, "data_root", cache_dir / "open3d_cache")),
            cache_dir / "open3d_cache",
        ]
        ply_candidates: list[Path] = []
        for root in search_roots:
            if root and Path(root).is_dir():
                ply_candidates.extend(Path(root).glob("**/*.ply"))
        if not ply_candidates:
            raise FileNotFoundError("Open3D dataset has no .ply under cache")
        ply_path = max(ply_candidates, key=lambda p: p.stat().st_size)
    out = cache_dir / f"{scene_name}.ply"
    shutil.copy2(ply_path, out)
    print(f"Cached scan → {out}")
    return ensure_triangle_mesh_file(out)


def _download_redwood_3dscan(scan_id: str, cache_dir: Path) -> Path:
    try:
        import redwood_3dscan as rws
    except ImportError as exc:
        raise RuntimeError(
            "Install redwood-3dscan: pip install redwood-3dscan"
        ) from exc

    data_dir = cache_dir / "rws"
    data_dir.mkdir(parents=True, exist_ok=True)
    os.chdir(data_dir)
    print(f"Downloading redwood-3dscan mesh {scan_id}...")
    rws.download_mesh(scan_id)
    ply = data_dir / "data" / "mesh" / f"{scan_id}.ply"
    if not ply.is_file():
        raise FileNotFoundError(f"redwood-3dscan mesh missing: {ply}")
    out = cache_dir / f"scan_{scan_id}.ply"
    shutil.copy2(ply, out)
    return out


def _data_root() -> Path:
    return Path(__file__).parent / "data"


def find_bundled_mesh(scene_name: str) -> Optional[Path]:
    """
    Resolve a high-fidelity local scan (Replica visual mesh preferred for export).

    Search order:
      data/replica/{scene_name}/mesh.ply
      data/replica/*/mesh.ply (first available)
      data/samples/{scene_name}.ply
    """
    scene_name = scene_name.lower().strip()
    if scene_name.startswith("replica:"):
        scene_name = scene_name.split(":", 1)[1].strip()

    replica_base = _data_root() / "replica"
    if replica_base.is_dir():
        named = replica_base / scene_name / "mesh.ply"
        if named.is_file() and named.stat().st_size > 50_000:
            return named
        for child in sorted(replica_base.iterdir()):
            if not child.is_dir():
                continue
            visual = child / "mesh.ply"
            if visual.is_file() and visual.stat().st_size > 50_000:
                logger.info("[mesh] Using bundled Replica visual: %s", visual)
                return visual

    sample = _data_root() / "samples" / f"{scene_name}.ply"
    if sample.is_file() and sample.stat().st_size > 50_000:
        return sample

    return None


def resolve_authentic_mesh_cache(scene_name: str, cache_dir: Path) -> Path:
    """
    Return path to an authentic mesh in the redwood cache, downloading if possible.

    Never generates procedural geometry.
    """
    scene_name = scene_name.lower().strip()
    cache_dir.mkdir(parents=True, exist_ok=True)
    out = cache_dir / f"{scene_name}.ply"

    if out.is_file():
        if out.stat().st_size > 50_000:
            tri = _triangulated_cache_path(out)
            if tri.is_file() and tri.stat().st_size > 50_000:
                return tri
            try:
                return ensure_triangle_mesh_file(out)
            except Exception as exc:
                logger.warning("[mesh] Re-triangulating cache %s: %s", out, exc)
        else:
            logger.warning("[mesh] Ignoring undersized cache %s (%d bytes)", out, out.stat().st_size)

    errors: list[str] = []

    try:
        return _download_open3d_redwood(scene_name, cache_dir)
    except Exception as e:
        errors.append(f"Open3D: {e}")

    try:
        return _download_redwood_3dscan("00033", cache_dir)
    except Exception as e:
        errors.append(f"redwood-3dscan: {e}")

    bundled = find_bundled_mesh(scene_name)
    if bundled is not None:
        shutil.copy2(bundled, out)
        logger.info("[mesh] Copied bundled scan → %s", out)
        return ensure_triangle_mesh_file(out)

    bundled_any = find_bundled_mesh("apartment")
    if bundled_any is not None and scene_name != "apartment":
        shutil.copy2(bundled_any, out)
        logger.warning("[mesh] Scene '%s' missing; using bundled %s", scene_name, bundled_any)
        return ensure_triangle_mesh_file(out)

    detail = "\n  - ".join(errors) if errors else "no download attempts succeeded"
    raise MeshUnavailableError(
        f"Unable to resolve authentic mesh for '{scene_name}'. "
        f"Install data/replica/<id>/mesh.ply or enable Open3D Redwood download.\n  - {detail}"
    )


def download_redwood_mesh(
    scene_name: str = "apartment",
    cache_dir: str | Path | None = None,
) -> Path:
    """Load or fetch a survey-grade mesh path (no procedural fallback)."""
    scene_name = scene_name.lower().strip()
    if scene_name.startswith("replica:"):
        replica_id = scene_name.split(":", 1)[1].strip()
        assets = download_replica_scene(replica_id)
        if assets is not None:
            return assets.semantic_mesh
        bundled = find_bundled_mesh(replica_id)
        if bundled is not None:
            cache = Path(cache_dir or _data_root() / "redwood")
            cache.mkdir(parents=True, exist_ok=True)
            out = cache / f"replica_{replica_id}.ply"
            shutil.copy2(bundled, out)
            return out
        raise MeshUnavailableError(
            f"Replica scene '{replica_id}' not found under data/replica/. {REPLICA_MISSING_MSG}"
        )

    cache = Path(cache_dir or _data_root() / "redwood")
    return resolve_authentic_mesh_cache(scene_name, cache)


def resolve_mesh_path(
    mesh_path: Optional[str | Path] = None,
    scene_name: str = "apartment",
) -> Path:
    if mesh_path is not None:
        p = Path(mesh_path)
        if not p.is_file():
            raise MeshUnavailableError(f"Mesh file not found: {p}")
        return p
    scene_name = scene_name.lower().strip()
    if scene_name.startswith("replica:"):
        return download_redwood_mesh(scene_name)
    default = _data_root() / "redwood" / f"{scene_name}.ply"
    if default.is_file() and default.stat().st_size > 50_000:
        return default
    dash = Path(__file__).resolve().parent.parent / "dashboard" / "public" / "meshes" / f"{scene_name}.ply"
    if dash.is_file() and dash.stat().st_size >= 256:
        return dash
    return download_redwood_mesh(scene_name)


def is_replica_scene_name(scene_name: str) -> bool:
    return scene_name.lower().startswith("replica:")


def load_replica_redwood_scene(
    scene_name: str,
    voxel_downsample: float = 0.02,
    dashboard_meshes_dir: Optional[Path] = None,
) -> Optional[RedwoodScene]:
    """Load a locally installed Replica digital-twin scene, or None if missing."""
    replica_id = scene_name.split(":", 1)[1].strip() if ":" in scene_name else scene_name
    assets = download_replica_scene(replica_id)
    if assets is None:
        return None
    return RedwoodScene.from_replica(
        assets,
        voxel_downsample=0.0,
        dashboard_meshes_dir=dashboard_meshes_dir,
    )


def load_semantic_redwood_scene(
    scene_id: str,
    meshes_dir: Path,
    *,
    center_mesh: bool = False,
) -> Optional[Tuple[RedwoodScene, str]]:
    """
    Load semantic collision for navigation when Replica or dashboard semantic assets exist.

    Priority (fastest first):
      1. dashboard/public/meshes/{scene}_collision.ply — pre-decimated mesh (fast BVH).
         Loaded with optional label sidecar; falls back to geometric classification if
         no sidecar exists (classify_primitive now handles UNKNOWN → normal-based fallback).
      2. dashboard/public/meshes/{scene}_semantic.ply (+ optional json)
      3. data/replica/{id}/habitat/mesh_semantic.ply — full Replica mesh (slow; last resort).
    """
    meshes_dir = Path(meshes_dir)
    scene_id = scene_id.lower().strip()

    # ── 1. Pre-built collision PLY (fastest: small, optimized for ray casting) ─────
    collision = resolve_dashboard_collision_mesh(scene_id, meshes_dir)
    if collision is not None and is_collision_mesh_path(collision):
        # Require a minimum size so we don't use an empty/tiny placeholder
        min_bytes = 500_000  # 500 KB
        if collision.stat().st_size >= min_bytes:
            sidecar = resolve_collision_label_sidecar(collision)
            _should_normalize = scene_id in REPLICA_SCENE_ALIASES or any(
                scene_id == v for v in REPLICA_SCENE_ALIASES.values()
            ) or bool(download_replica_scene(scene_id))  # any Replica scene
            if sidecar is not None:
                print(f"[semantic] Collision + labels → {collision}")
                mesh = load_collision_mesh(collision)
                labels = load_collision_semantic_labels(sidecar, len(mesh.triangles))
            else:
                print(f"[semantic] Collision mesh (geometric labels) → {collision}")
                labels = None
            scene = RedwoodScene(
                collision,
                voxel_downsample=0.0,
                center_mesh=center_mesh,
                replica_normalize=_should_normalize,
                triangle_semantics=labels,
                visual_mesh_url=resolve_dashboard_visual_url(scene_id, meshes_dir),
            )
            return scene, scene_id

    # ── 2. Dashboard semantic bundle ──────────────────────────────────────────────
    bundle = resolve_dashboard_semantic_bundle(scene_id, meshes_dir)
    if bundle is not None:
        semantic_ply, semantic_json = bundle
        instance_map = _load_replica_instance_map(semantic_json)
        tri_labels = _build_triangle_semantic_labels(semantic_ply, instance_map)
        print(f"[semantic] Dashboard semantic mesh → {semantic_ply}")
        scene = RedwoodScene(
            semantic_ply,
            voxel_downsample=0.0,
            center_mesh=center_mesh,
            replica_normalize=True,
            triangle_semantics=tri_labels,
            visual_mesh_url=resolve_dashboard_visual_url(scene_id, meshes_dir),
        )
        return scene, scene_id

    # ── 3. Full Replica dataset (slow — only when no pre-built collision PLY) ─────
    for replica_id in replica_lookup_ids(scene_id):
        assets = download_replica_scene(replica_id)
        if assets is None:
            continue
        print(f"[semantic] Replica scene '{replica_id}' → {assets.semantic_mesh}")
        scene = RedwoodScene.from_replica(
            assets,
            voxel_downsample=0.0,
            dashboard_meshes_dir=meshes_dir,
        )
        visual_url = resolve_dashboard_visual_url(scene_id, meshes_dir)
        if visual_url:
            scene.visual_mesh_url = visual_url
        return scene, f"replica:{replica_id}"

    return None
