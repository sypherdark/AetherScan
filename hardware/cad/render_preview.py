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


def render(stl_path: Path, out_png: Path, elev: float = 22, azim: float = -55):
    mesh = trimesh.load(stl_path, force="mesh")
    tris = mesh.triangles  # (n, 3, 3)

    # Simple lambert shading from face normals.
    n = mesh.face_normals
    light = np.array([0.3, -0.5, 0.8])
    light = light / np.linalg.norm(light)
    shade = np.clip(n @ light, 0.15, 1.0)
    base = np.array([0.25, 0.45, 0.75])
    colors = np.clip(base[None, :] * shade[:, None] + 0.08, 0, 1)

    fig = plt.figure(figsize=(9, 7), dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    coll = Poly3DCollection(tris, facecolors=colors, edgecolors=(0, 0, 0, 0.12), linewidths=0.2)
    ax.add_collection3d(coll)

    v = mesh.vertices
    c = v.mean(axis=0)
    r = (v.max(axis=0) - v.min(axis=0)).max() / 2
    for lo, hi, setter in (
        (c[0] - r, c[0] + r, ax.set_xlim),
        (c[1] - r, c[1] + r, ax.set_ylim),
        (c[2] - r, c[2] + r, ax.set_zlim),
    ):
        setter(lo, hi)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X (fwd)")
    ax.set_ylabel("Y (left)")
    ax.set_zlabel("Z (up)")
    ax.set_title(f"{stl_path.stem}  —  {2*r:.0f} mm envelope")
    fig.tight_layout()
    fig.savefig(out_png, bbox_inches="tight")
    print(f"Rendered → {out_png}")


if __name__ == "__main__":
    src = Path(sys.argv[1] if len(sys.argv) > 1 else "out/frame.stl")
    dst = src.with_name(src.stem + "_preview.png")
    render(src, dst)
