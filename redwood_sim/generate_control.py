import open3d as o3d
import numpy as np
import os

# Create a box: X=10, Y=10, Z=3
mesh = o3d.geometry.TriangleMesh.create_box(width=10.0, height=10.0, depth=3.0)

# Translate so the floor is at Z=0, and X/Y are centered at 0,0
mesh.translate(np.array([-5.0, -5.0, 0.0]))

# Compute perfect normals so lighting works
mesh.compute_vertex_normals()

# Ensure the output directory exists
out_path = "../dashboard/public/meshes/control_room.ply"  # Adjust path if necessary
os.makedirs(os.path.dirname(out_path), exist_ok=True)

o3d.io.write_triangle_mesh(out_path, mesh)
print(f"Perfect control room generated at {out_path}")
