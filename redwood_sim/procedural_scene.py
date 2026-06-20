"""
Procedural box-mesh generation has been removed.

Use authentic scans via ``scene_loader.download_redwood_mesh`` (Open3D Redwood,
RWS cache, or bundled ``data/replica/{scene_id}/mesh.ply``).
"""

from __future__ import annotations


class ProceduralMeshRemovedError(RuntimeError):
    """Raised when legacy code paths request procedural geometry."""

    def __init__(self, scene_name: str = "unknown") -> None:
        super().__init__(
            f"Procedural mesh '{scene_name}' is no longer supported. "
            "Install a Replica scene under redwood_sim/data/replica/<id>/mesh.ply "
            "or allow Open3D to download RedwoodIndoor assets on first run."
        )
