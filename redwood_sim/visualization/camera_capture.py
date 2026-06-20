"""
Synthetic semantic camera from the drone's forward-facing pose.

Replaces the previous Open3D mesh renderer, which rendered the sparse
gray collision PLY and produced meaningless "black dots on white" images.

This implementation fires a grid of rays (one per pixel) against the same
collision mesh the physics engine uses, then colours each pixel by the
semantic label (wall / floor / ceiling / object) of the hit surface, with
depth-based shading and basic Lambertian lighting.  The result is a
meaningful first-person view that correctly shows the indoor structure.

Coordinate frame: ROS Z-up (same as RedwoodScene / physics sim).
  Body +X = forward, +Y = left, +Z = up.
"""

from __future__ import annotations

import base64
import math
import logging
from io import BytesIO
from typing import Optional

import numpy as np

from core.math3d import quat_to_rotation_matrix

logger = logging.getLogger(__name__)

# ── Camera parameters ────────────────────────────────────────────────────────
CAMERA_FOV_H_DEG: float = 72.0   # horizontal field of view
CAMERA_MAX_RANGE: float = 8.0    # raycast distance cap (metres)

# ── Semantic class → base RGB (uint8) ────────────────────────────────────────
# Values mirror core.semantic_space.SemanticClass (IntEnum):
#   UNKNOWN=0  FREE=1  WALL=2  OBJECT=3  FLOOR=4  CEILING=5
_LABEL_RGB: dict[int, tuple[int, int, int]] = {
    0: (85,  82,  78),   # UNKNOWN  – dark neutral gray
    1: (20,  25,  35),   # FREE     – near-black (open air / ray miss)
    2: (162, 155, 146),  # WALL     – light plaster gray
    3: (164, 118,  80),  # OBJECT   – warm furniture amber
    4: (112,  91,  68),  # FLOOR    – warm hardwood brown
    5: (198, 208, 224),  # CEILING  – cool blue-white
}
_MISS_RGB: tuple[int, int, int] = (10, 15, 25)   # no hit
_DEPTH_EXP: float = 0.50                          # depth-shading power curve
_AMBIENT:   float = 0.30                          # minimum brightness fraction


# ── Helper: build a LUT array so colour lookups are O(1) ─────────────────────
_MAX_SEM = max(_LABEL_RGB) + 2
_LUT = np.zeros((_MAX_SEM, 3), dtype=np.float32)
for _k, _v in _LABEL_RGB.items():
    if _k < _MAX_SEM:
        _LUT[_k] = [c / 255.0 for c in _v]
del _k, _v


def _camera_axes(
    position: np.ndarray,
    yaw: float,
    quaternion: Optional[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Return (forward, right, up) unit vectors in ROS Z-up world frame.

    The camera is horizon-levelled (roll=0, pitch=0) regardless of body
    attitude — this avoids showing the ceiling or floor during normal flight.
    """
    if quaternion is not None:
        R = quat_to_rotation_matrix(np.asarray(quaternion, dtype=np.float64))
        fwd = R @ np.array([1.0, 0.0, 0.0])
    else:
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])

    # Project forward into the XY plane (level the horizon)
    fwd[2] = 0.0
    fn = float(np.linalg.norm(fwd))
    fwd = fwd / fn if fn > 1e-9 else np.array([1.0, 0.0, 0.0])

    up = np.array([0.0, 0.0, 1.0])

    # right = forward × up  (ROS: +Y is left, so cross gives world-right)
    right = np.cross(fwd, up)
    rn = float(np.linalg.norm(right))
    right = right / rn if rn > 1e-9 else np.array([0.0, -1.0, 0.0])

    return fwd, right, up


def _build_ray_dirs(
    forward: np.ndarray,
    right: np.ndarray,
    up: np.ndarray,
    width: int,
    height: int,
    fov_h_deg: float,
) -> np.ndarray:
    """
    Build (height*width, 3) array of normalised ray directions.

    Pixel (col, row) maps to:
        dir = forward + right*tan_h*(2u-1) - up*tan_v*(2v-1)
    where u,v ∈ [0,1] with (0,0) = top-left corner.
    """
    tan_h = math.tan(math.radians(fov_h_deg / 2.0))
    tan_v = tan_h * height / width  # preserve aspect ratio

    # NDC coordinates: [-1, 1] across each axis
    us = (np.arange(width,  dtype=np.float32) + 0.5) / width  * 2.0 - 1.0
    vs = (np.arange(height, dtype=np.float32) + 0.5) / height * 2.0 - 1.0
    uu, vv = np.meshgrid(us, vs, indexing="xy")  # both (H, W)

    # dirs[H, W, 3]
    dirs = (
        forward[None, None, :].astype(np.float32)
        + right[None, None, :].astype(np.float32) * (uu[:, :, None] * tan_h)
        - up[None, None, :].astype(np.float32)    * (vv[:, :, None] * tan_v)
    )  # (H, W, 3)

    dirs = dirs.reshape(-1, 3).astype(np.float64)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs /= np.maximum(norms, 1e-9)
    return dirs  # (N, 3)


def _array_to_jpeg_b64(rgb: np.ndarray) -> str:
    """Encode HxWx3 uint8 RGB to base64 JPEG (PNG fallback)."""
    arr = np.asarray(rgb, dtype=np.uint8)
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    try:
        from PIL import Image
        buf = BytesIO()
        Image.fromarray(arr).save(buf, format="JPEG", quality=80)
        return base64.b64encode(buf.getvalue()).decode("ascii")
    except ImportError:
        pass
    try:
        import open3d as o3d
        import tempfile
        from pathlib import Path
        img = o3d.geometry.Image(arr)
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
            path = Path(tmp.name)
        o3d.io.write_image(str(path), img)
        data = path.read_bytes()
        path.unlink(missing_ok=True)
        return base64.b64encode(data).decode("ascii")
    except Exception:
        return ""


def capture_drone_view(
    scene,
    position: np.ndarray,
    yaw: float,
    quaternion: Optional[np.ndarray] = None,
    width: int = 320,
    height: int = 240,
) -> Optional[str]:
    """
    Render a forward-facing semantic view from *position*.

    Parameters
    ----------
    scene:
        ``RedwoodScene`` instance — provides ``cast_rays`` and semantic labels
        via ``_triangle_semantics`` / ``classify_primitive``.
    position:
        Drone world position in ROS Z-up frame (metres).
    yaw:
        Body yaw angle (radians).
    quaternion:
        Optional body quaternion (w,x,y,z) for attitude.  When provided, the
        horizontal component of the body forward is used instead of raw yaw.
    width, height:
        Output image resolution.  320×240 is the default — large enough for
        meaningful detail but fast enough for real-time capture (~76 k rays).

    Returns
    -------
    str | None
        Base64-encoded JPEG string, or None on failure.
    """
    try:
        pos = np.asarray(position, dtype=np.float64)
        forward, right, up = _camera_axes(pos, float(yaw), quaternion)

        n_rays = width * height
        dirs = _build_ray_dirs(forward, right, up, width, height, CAMERA_FOV_H_DEG)

        # Single batched raycast — Open3D uses all CPU cores internally
        origins = np.broadcast_to(pos[None, :], (n_rays, 3)).copy().astype(np.float64)
        dists, hit_pts, normals, prim_ids = scene.cast_rays(
            origins, dirs, max_distance=CAMERA_MAX_RANGE
        )

        # ── Determine semantic class for every hit ────────────────────────────
        dists = np.asarray(dists, dtype=np.float64)
        prim_ids = np.asarray(prim_ids, dtype=np.int64)
        hit_mask = np.isfinite(dists) & (dists < CAMERA_MAX_RANGE - 0.05)

        sem_classes = np.zeros(n_rays, dtype=np.int32)  # default UNKNOWN

        tri_sem = getattr(scene, "_triangle_semantics", None)
        if tri_sem is not None:
            # Vectorised path: index into the semantic label array directly
            valid = hit_mask & (prim_ids >= 0) & (prim_ids < len(tri_sem))
            sem_classes[valid] = tri_sem[prim_ids[valid]].astype(np.int32)
        else:
            # Fallback: per-ray Python call (slower, but works without labels)
            for i in np.flatnonzero(hit_mask):
                sc, _, _ = scene.classify_primitive(
                    int(prim_ids[i]), hit_pts[i], normals[i], pos
                )
                sem_classes[i] = int(sc)

        # ── Vectorised colour computation ─────────────────────────────────────
        # Depth shading: objects further away appear darker
        depth_t = np.where(
            hit_mask,
            np.power(
                np.clip(1.0 - dists / CAMERA_MAX_RANGE, 0.0, 1.0),
                _DEPTH_EXP,
            ),
            0.0,
        )  # (N,)

        # Lambertian shading from the camera direction (flat normals give some 3-D feel)
        normals_arr = np.asarray(normals, dtype=np.float32).reshape(n_rays, 3)
        # Dot product with camera forward (view-aligned light)
        lambert = np.abs(
            normals_arr[:, 0] * float(forward[0])
            + normals_arr[:, 1] * float(forward[1])
            + normals_arr[:, 2] * float(forward[2])
        )  # (N,) in [0, 1]

        brightness = np.where(
            hit_mask,
            _AMBIENT + (1.0 - _AMBIENT) * (0.55 * depth_t + 0.45 * lambert),
            0.0,
        )  # (N,)

        # Clamp semantic indices to LUT range
        sem_clamped = np.clip(sem_classes, 0, _MAX_SEM - 1)
        base_colors = _LUT[sem_clamped]          # (N, 3) float32 in [0, 1]
        shaded = base_colors * brightness[:, None]  # (N, 3)

        # Mark background pixels
        miss = ~hit_mask
        shaded[miss] = [c / 255.0 for c in _MISS_RGB]

        # ── Assemble image and encode ─────────────────────────────────────────
        img_flat = np.clip(shaded * 255.0, 0, 255).astype(np.uint8)
        img = img_flat.reshape(height, width, 3)

        # Subtle vignette — darken edges (enhances depth perception)
        cx = np.linspace(-1.0, 1.0, width,  dtype=np.float32)
        cy = np.linspace(-1.0, 1.0, height, dtype=np.float32)
        xx, yy = np.meshgrid(cx, cy)
        vignette = np.clip(1.0 - 0.45 * (xx ** 2 + yy ** 2), 0.0, 1.0)[:, :, None]
        img = np.clip(img.astype(np.float32) * vignette, 0, 255).astype(np.uint8)

        return _array_to_jpeg_b64(img)

    except Exception:
        logger.exception("capture_drone_view failed")
        return None
