"""Dark, brand-matched single-view render of the airframe STL for the brief."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np, trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

stl = sys.argv[1] if len(sys.argv) > 1 else "/Users/oubaidfradi/DRONE/hardware/cad/out/frame.stl"
out = sys.argv[2] if len(sys.argv) > 2 else "/Users/oubaidfradi/DRONE/presentation/assets/airframe_dark.png"
BG = (7/255, 9/255, 12/255)

m = trimesh.load(stl, force="mesh")
tris = m.triangles
n = m.face_normals
light = np.array([0.4, -0.5, 0.8]); light /= np.linalg.norm(light)
sh = np.clip(n @ light, 0.12, 1.0)
# brand cyan-steel gradient by shading
base = np.array([0.16, 0.55, 0.66])
col = np.clip(base[None, :] * (0.35 + 0.9 * sh[:, None]), 0, 1)

fig = plt.figure(figsize=(7.2, 5.4), dpi=200)
fig.patch.set_facecolor(BG)
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor(BG)
pc = Poly3DCollection(tris, facecolors=col, edgecolors=(0.05, 0.07, 0.09, 0.5), linewidths=0.15)
ax.add_collection3d(pc)
v = m.vertices; cct = v.mean(0); r = (v.max(0) - v.min(0)).max() / 2 * 0.62
ax.set_xlim(cct[0]-r, cct[0]+r); ax.set_ylim(cct[1]-r, cct[1]+r); ax.set_zlim(cct[2]-r, cct[2]+r)
ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=20, azim=-58)
ax.set_axis_off()
fig.tight_layout(pad=0)
fig.savefig(out, facecolor=BG, bbox_inches="tight", pad_inches=0.05)
print("wrote", out)
