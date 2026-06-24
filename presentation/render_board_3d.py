"""Coloured dark render of the assembled PSDB 3D model (/tmp/psdb_board/*.stl)."""
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np, trimesh
from pathlib import Path
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SRC = Path("/tmp/psdb_board")
OUT = "/Users/oubaidfradi/DRONE/presentation/assets/psdb_board_3d.png"
BG = (7/255, 9/255, 12/255)
COLORS = {
    "board": (0.05, 0.30, 0.17),   # PCB green
    "elec":  (0.12, 0.18, 0.42),   # electrolytic can (blue)
    "mlcc":  (0.80, 0.66, 0.40),   # ceramic caps (tan)
    "ind":   (0.18, 0.18, 0.21),   # inductors (dark)
    "res":   (0.09, 0.09, 0.10),   # resistors (black)
    "conn":  (0.85, 0.68, 0.20),   # connectors/headers (gold)
}
ORDER = ["board", "elec", "ind", "mlcc", "res", "conn"]

light = np.array([0.35, -0.45, 0.82]); light /= np.linalg.norm(light)
fig = plt.figure(figsize=(7.6, 5.4), dpi=200); fig.patch.set_facecolor(BG)
ax = fig.add_subplot(111, projection="3d"); ax.set_facecolor(BG)

allv = []
for name in ORDER:
    f = SRC / f"{name}.stl"
    if not f.exists():
        continue
    m = trimesh.load(f, force="mesh")
    sh = np.clip(m.face_normals @ light, 0.16, 1.0)
    base = np.array(COLORS[name])
    col = np.clip(base[None, :] * (0.32 + 0.95 * sh[:, None]), 0, 1)
    ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=col,
                        edgecolors=(0, 0, 0, 0.22), linewidths=0.08))
    allv.append(m.vertices)

v = np.vstack(allv); c = v.mean(0)
r = max(np.ptp(v[:,0]), np.ptp(v[:,1])) / 2 * 0.78
ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
ax.set_box_aspect((1, 1, 0.62)); ax.view_init(elev=20, azim=-50); ax.set_axis_off()
fig.tight_layout(pad=0)
fig.savefig(OUT, facecolor=BG, bbox_inches="tight", pad_inches=0.04)
print("wrote", OUT)
