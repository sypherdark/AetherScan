"""
Headless preview renderer for CAD STL exports.

render-glb wants a GPU we don't have in this environment, so we rasterize the
STL with matplotlib instead — no display, no OpenGL. Good enough to eyeball that
a part looks right before committing.

Run with the sim venv (has trimesh + matplotlib):
    redwood_sim/.venv/bin/python hardware/cad/render_preview.py out/frame.stl
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402


def _add(ax, tris, colors, c, r, elev, azim, title):
    coll = Poly3DCollection(tris, facecolors=colors, edgecolors=(0, 0, 0, 0.10), linewidths=0.15)
    ax.add_collection3d(coll)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X fwd", fontsize=8)
    ax.set_ylabel("Y left", fontsize=8)
    ax.set_zlabel("Z up", fontsize=8)
    ax.tick_params(labelsize=6)
    ax.set_title(title, fontsize=10)


def render(stl_path: Path, out_png: Path):
    mesh = trimesh.load(stl_path, force="mesh")
    tris = mesh.triangles

    n = mesh.face_normals
    light = np.array([0.35, -0.5, 0.78])
    light = light / np.linalg.norm(light)
    shade = np.clip(n @ light, 0.18, 1.0)
    base = np.array([0.27, 0.47, 0.72])
    colors = np.clip(base[None, :] * shade[:, None] + 0.08, 0, 1)

    v = mesh.vertices
    c = v.mean(axis=0)
    r = (v.max(axis=0) - v.min(axis=0)).max() / 2

    # Four views: iso, top, front (+X toward viewer), side.
    views = [
        (24, -58, "isometric"),
        (89, -90, "top (XY)"),
        (4, 0, "front (+X / nose)"),
        (4, -90, "side (+Y)"),
    ]
    fig = plt.figure(figsize=(13, 11), dpi=130)
    for i, (elev, azim, label) in enumerate(views, 1):
        ax = fig.add_subplot(2, 2, i, projection="3d")
        _add(ax, tris, colors, c, r, elev, azim, label)
    fig.suptitle(f"{stl_path.stem}  —  {2*r:.0f} mm envelope", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_png, bbox_inches="tight")
    print(f"Rendered → {out_png}")


if __name__ == "__main__":
    src = Path(sys.argv[1] if len(sys.argv) > 1 else "out/frame.stl")
    dst = src.with_name(src.stem + "_preview.png")
    render(src, dst)
