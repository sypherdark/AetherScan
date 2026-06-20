"""
Scan deliverables — the export formats commercial scanning pipelines expect.

Three artifacts, generated from a completed (or in-progress) scan:
- ``scan_cloud.ply``  — the deduplicated reconstruction point cloud, coloured by
  semantic class (PLY is the lingua franca: SuperSplat, CloudCompare, Revit
  importers, game engines all read it).
- ``scan_mesh.glb``   — a watertight-ish Poisson surface reconstruction of the
  cloud (GLB loads directly into Blender/Unity/three.js viewers).
- ``floor_plan.svg``  — a dimensioned 2D floor plan from the occupancy grid:
  walls dark, furniture/objects lighter, scanned floor pale, with overall
  dimensions and the scanned-area figure.

Everything works from data the drone actually gathered (the recon cloud and the
discovery grid) — no peeking at the ground-truth mesh.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Dict, List

import numpy as np

# Colours per semantic class (index = core.semantic_space.SemanticClass), RGB 0-1
_CLASS_COLORS = np.array([
    [0.55, 0.57, 0.60],   # 0 UNKNOWN  – grey
    [0.40, 0.85, 0.45],   # 1 FREE     – green
    [0.30, 0.55, 0.95],   # 2 WALL     – blue
    [0.95, 0.60, 0.20],   # 3 OBJECT   – orange
    [0.55, 0.45, 0.35],   # 4 FLOOR    – brown
    [0.75, 0.75, 0.80],   # 5 CEILING  – light grey
], dtype=np.float64)


def _cloud_to_o3d(points: List[List[float]]):
    import open3d as o3d
    arr = np.asarray(points, dtype=np.float64)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(arr[:, :3])
    cls = np.clip(arr[:, 3].astype(int), 0, len(_CLASS_COLORS) - 1)
    pc.colors = o3d.utility.Vector3dVector(_CLASS_COLORS[cls])
    return pc


def export_point_cloud(points: List[List[float]], out_path: Path) -> Dict:
    """Write the labelled reconstruction cloud as a coloured binary PLY."""
    import open3d as o3d
    pc = _cloud_to_o3d(points)
    o3d.io.write_point_cloud(str(out_path), pc)
    return {"file": out_path.name, "points": len(pc.points),
            "bytes": out_path.stat().st_size}


def export_mesh(points: List[List[float]], out_path: Path,
                poisson_depth: int = 7) -> Dict:
    """Poisson-reconstruct a surface from the cloud and write GLB."""
    import open3d as o3d
    pc = _cloud_to_o3d(points)
    pc.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=30)
    )
    # Orient normals toward the room interior: the sensor flew INSIDE the space,
    # so every scanned surface faces the cloud centroid.  (The tangent-plane
    # propagation alternative is O(n·k) fragile and crashed on 100k+ clouds.)
    centroid = np.asarray(pc.points).mean(axis=0)
    pc.orient_normals_towards_camera_location(camera_location=centroid)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pc, depth=poisson_depth
    )
    # Crop the low-density halo Poisson extrapolates beyond the scanned surface
    dens = np.asarray(densities)
    mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.06))
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(out_path), mesh)
    return {"file": out_path.name, "vertices": len(mesh.vertices),
            "triangles": len(mesh.triangles), "bytes": out_path.stat().st_size}


def export_floor_plan_svg(dmap, out_path: Path, scene_id: str = "") -> Dict:
    """Render the discovery grid as a dimensioned SVG floor plan.

    Walls = dark, objects/furniture = orange, scanned floor = pale.  The drawing
    is annotated with overall width/depth in metres and the scanned floor area
    (free cells × cell area) — the figures a Polycam-style deliverable carries.
    """
    from core.semantic_space import SemanticClass

    res = dmap.cfg.resolution
    grid = dmap.grid
    lo = dmap.logodds
    h, w = grid.shape

    occupied = lo >= dmap.cfg.occ_threshold
    known = occupied | (lo <= dmap.cfg.free_threshold)
    if not known.any():
        raise ValueError("nothing scanned yet — start a mission first")

    ii, jj = np.where(known)
    i0, i1 = int(ii.min()), int(ii.max()) + 1
    j0, j1 = int(jj.min()), int(jj.max()) + 1
    width_m = (i1 - i0) * res
    depth_m = (j1 - j0) * res

    free = (lo <= dmap.cfg.free_threshold)
    area_m2 = float(free.sum()) * res * res

    # SVG: x ← grid i (world X), y ← grid j flipped (world Y up in plan view)
    PX = 14                      # px per cell
    MARGIN = 70
    W = (i1 - i0) * PX + 2 * MARGIN
    H = (j1 - j0) * PX + 2 * MARGIN

    def cell_rect(i: int, j: int, fill: str, opacity: float = 1.0) -> str:
        x = MARGIN + (i - i0) * PX
        y = MARGIN + (j1 - 1 - j) * PX
        return (f'<rect x="{x}" y="{y}" width="{PX}" height="{PX}" '
                f'fill="{fill}" fill-opacity="{opacity}"/>')

    cells: List[str] = []
    wall_c, obj_c, floor_c = "#1e293b", "#f59e0b", "#dbeafe"
    for i in range(i0, i1):
        for j in range(j0, j1):
            if occupied[i, j]:
                sem = int(grid[i, j])
                if sem == int(SemanticClass.OBJECT):
                    cells.append(cell_rect(i, j, obj_c, 0.9))
                else:
                    cells.append(cell_rect(i, j, wall_c))
            elif free[i, j]:
                cells.append(cell_rect(i, j, floor_c, 0.8))

    x_right = MARGIN + (i1 - i0) * PX
    y_bot = MARGIN + (j1 - j0) * PX
    dims = f'''
  <g stroke="#475569" stroke-width="1.5" fill="none">
    <line x1="{MARGIN}" y1="{y_bot + 22}" x2="{x_right}" y2="{y_bot + 22}"/>
    <line x1="{x_right + 22}" y1="{MARGIN}" x2="{x_right + 22}" y2="{y_bot}"/>
  </g>
  <text x="{(MARGIN + x_right) / 2}" y="{y_bot + 40}" text-anchor="middle"
        font-family="monospace" font-size="15" fill="#334155">{width_m:.1f} m</text>
  <text x="{x_right + 40}" y="{(MARGIN + y_bot) / 2}" text-anchor="middle"
        font-family="monospace" font-size="15" fill="#334155"
        transform="rotate(90 {x_right + 40} {(MARGIN + y_bot) / 2})">{depth_m:.1f} m</text>
  <text x="{MARGIN}" y="{MARGIN - 34}" font-family="sans-serif" font-size="19"
        font-weight="bold" fill="#0f172a">AetherScan floor plan{(" — " + scene_id) if scene_id else ""}</text>
  <text x="{MARGIN}" y="{MARGIN - 14}" font-family="monospace" font-size="13"
        fill="#475569">scanned area {area_m2:.1f} m²  ·  grid {res:.2f} m  ·  {time.strftime("%Y-%m-%d %H:%M")}</text>
  <g font-family="monospace" font-size="12" fill="#475569">
    <rect x="{MARGIN}" y="{H - 30}" width="12" height="12" fill="{wall_c}"/>
    <text x="{MARGIN + 18}" y="{H - 20}">wall</text>
    <rect x="{MARGIN + 70}" y="{H - 30}" width="12" height="12" fill="{obj_c}"/>
    <text x="{MARGIN + 88}" y="{H - 20}">object</text>
    <rect x="{MARGIN + 156}" y="{H - 30}" width="12" height="12" fill="{floor_c}"/>
    <text x="{MARGIN + 174}" y="{H - 20}">scanned floor</text>
  </g>'''

    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'viewBox="0 0 {W} {H}">\n'
           f'<rect width="{W}" height="{H}" fill="#f8fafc"/>\n'
           + "\n".join(cells) + dims + "\n</svg>\n")
    out_path.write_text(svg)
    return {"file": out_path.name, "area_m2": round(area_m2, 1),
            "width_m": round(width_m, 1), "depth_m": round(depth_m, 1),
            "bytes": out_path.stat().st_size}


def export_scan(points: List[List[float]], dmap, out_dir: Path,
                scene_id: str = "") -> Dict:
    """Generate all deliverables into *out_dir*; returns a manifest dict."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = f"{scene_id or 'scan'}_{stamp}"
    manifest: Dict = {"files": [], "errors": []}

    if len(points) >= 100:
        try:
            r = export_point_cloud(points, out_dir / f"{base}_cloud.ply")
            manifest["files"].append(r)
        except Exception as exc:                     # pragma: no cover
            manifest["errors"].append(f"cloud: {exc}")
        try:
            r = export_mesh(points, out_dir / f"{base}_mesh.glb")
            manifest["files"].append(r)
        except Exception as exc:                     # pragma: no cover
            manifest["errors"].append(f"mesh: {exc}")
    else:
        manifest["errors"].append("cloud too small (<100 points) — scan first")

    try:
        r = export_floor_plan_svg(dmap, out_dir / f"{base}_floorplan.svg", scene_id)
        manifest["files"].append(r)
    except Exception as exc:
        manifest["errors"].append(f"floorplan: {exc}")

    return manifest
