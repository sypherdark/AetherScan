# AetherScan — Claude Code Handoff Context

Technical specification for migrating and operating the AetherScan indoor autonomous drone simulation stack. Python owns all physics and semantics; React/Three.js is display-only.

---

## System Overview

AetherScan implements a **decoupled, authoritative dual-mesh architecture**:

| Layer | Runtime | Responsibility |
|-------|---------|----------------|
| **Physics backend** | `redwood_sim/` (Python, headless) | RK4 rigid-body integration @ **500 Hz** (`physics_dt = 0.002`), mesh raycasting collision, LiDAR, semantic navigation, WebSocket telemetry |
| **Presentation frontend** | `dashboard/` (Next.js + React Three Fiber) | Async 3D viewport — **no physics** — loads visual mesh only |

### Dual-mesh split

1. **Collision mesh** (`{scene}_collision.ply`, &lt;100k triangles)
   - Loaded exclusively by `RedwoodScene` in Python via Open3D `RaycastingScene`
   - Drives `MeshCollisionSolver`, LiDAR `cast_rays`, and per-triangle semantic classification
   - Authored with `scripts/build_collision_mesh.py` or shipped pre-built under `dashboard/public/meshes/`

2. **Visual mesh** (`{scene}.glb` preferred, `{scene}.ply` clay fallback)
   - Loaded only by `MeshEnvironment.tsx` in the dashboard
   - GLB: PBR materials via `@react-three/drei` `useGLTF`
   - PLY: flat-shaded clay (`#d1d5db`) via `PLYLoader`
   - **Never** used for collision or raycasting on the frontend

### Physics pipeline

```
SimulationEngine.tick(control_dt=0.01)
  └─ 5× RK4 micro-steps @ physics_dt=0.002  (core/physics.py)
       └─ MeshCollisionSolver.resolve() after each micro-step
            └─ RedwoodScene.cast_rays() → Open3D RaycastingScene
```

### Semantic navigation pipeline

```
RedwoodScene (collision mesh + optional _collision_labels.npy)
  └─ AnalyzedIndoorSpace.build()  (core/semantic_space.py)
       └─ nav_grid + SpatialElement catalog
  └─ SensorNavigator + DiscoveryMap  (core/sensor_navigation.py, discovery_map.py)
       └─ classify_hit(primitive_id) → SemanticClass
```

### Scene load priority (`bridge/server.py` → `load_scene`)

1. `replica:{id}` — full Replica digital twin (`habitat/mesh_semantic.ply` + `semantic.json`)
2. `load_semantic_redwood_scene()` — dashboard semantic bundle or collision + labels
3. `resolve_dashboard_collision_mesh()` — `{scene}_collision.ply`
4. Fallback — cached Redwood scan (`data/redwood/{scene}.ply`)

---

## Current State Configuration

### Active scene: `apartment`

| Asset | Path | Status |
|-------|------|--------|
| Collision PLY | `dashboard/public/meshes/apartment_collision.ply` | Present (~99,999 tris, 3.7 MB) |
| Collision labels | `dashboard/public/meshes/apartment_collision_labels.npy` | Present (99,999 labels, synthetic normal-heuristic) |
| Visual GLB | `dashboard/public/meshes/apartment.glb` | **Missing** — PLY clay fallback active |
| Visual PLY | `dashboard/public/meshes/apartment.ply` | Present (17 MB, Z-up ROS) |
| Replica dataset | `redwood_sim/data/replica/apartment_0/` | **Not installed** (README only) |

### Low-storage semantic workaround

Full Replica scenes are multi-GB. To enable **per-triangle semantic pathfinding** without downloading Replica:

```bash
# 1. Ensure collision mesh exists (already built for apartment)
ls dashboard/public/meshes/apartment_collision.ply

# 2. Generate synthetic per-triangle labels from surface normals
redwood_sim/.venv/bin/python scripts/generate_synthetic_semantics.py --scene apartment

# 3. Restart bridge — expect log: semantic_triangles=99,999
./run-aetherscan.sh --dashboard
```

`scripts/generate_synthetic_semantics.py` classifies each collision triangle by normal vector and centroid height into `SemanticClass` values (`WALL`, `FLOOR`, `CEILING`, `OBJECT`) and writes `apartment_collision_labels.npy`. `RedwoodScene.__init__` auto-binds the sidecar when present.

**Production semantics** (Replica-accurate labels):

```bash
# After placing Replica assets under redwood_sim/data/replica/apartment_0/
redwood_sim/.venv/bin/python scripts/build_collision_mesh.py \
  --scene apartment \
  --semantic-source redwood_sim/data/replica/apartment_0/habitat/mesh_semantic.ply \
  --semantic-json redwood_sim/data/replica/apartment_0/habitat/semantic.json

redwood_sim/.venv/bin/python scripts/validate_collision_mesh.py --scene apartment
```

### Launch

```bash
./run-aetherscan.sh --dashboard
# Physics bridge: ws://127.0.0.1:8765
# Dashboard:      http://localhost:3000
```

---

## Coordinate Space Matrix

All simulation, collision, and telemetry use **Z-up ROS / ENU indoor frame**:

- **X** — forward / corridor axis
- **Y** — lateral (left-positive)
- **Z** — altitude (floor ≈ 0)

Three.js uses **Y-up** (right-handed). The mapping is:

$$\text{Three}(x_t, y_t, z_t) = (x_{ros},\; z_{ros},\; -y_{ros})$$

Implemented in `dashboard/src/lib/ros-three.ts`:

```typescript
export function rosToThree(x, y, z): [number, number, number] {
  return [x, z, -y]
}
```

### Group rotation (mesh + drone)

`MeshEnvironment.tsx` wraps all scene geometry in:

```tsx
<group rotation={[-Math.PI / 2, 0, 0]}>
```

This is a **−π/2 radian rotation about the X-axis**, equivalent to applying the linear map:

$$\begin{bmatrix} x' \\ y' \\ z' \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \\ 0 & -1 & 0 \end{bmatrix} \begin{bmatrix} x \\ y \\ z \end{bmatrix}$$

Combined with `rosToThree`, world positions align: ROS $(X, Y, Z)$ → Three canvas $(X, Z, -Y)$.

### Drone pose

`DroneModel.tsx` applies the same transform to position and quaternion:

- Position lerp: `(position[0], position[2], -position[1])`
- Quaternion remap: `new THREE.Quaternion(x, z, y, w)` from body `[w, x, y, z]`

### Bounds contract

`scene_bounds` from the bridge are **ROS Z-up AABB** (`min`/`max` as `[x, y, z]`). The frontend stores them in `drone-store.sceneAabb` without re-mapping; only rendered entities pass through `rosToThree`.

---

## The Network Contract

**Endpoint:** `ws://127.0.0.1:8765` (override via `NEXT_PUBLIC_SIM_BRIDGE_URL`)

**Server:** `redwood_sim/bridge/server.py` (`python -m bridge --scene apartment`)

**Client:** `dashboard/src/hooks/useSimBridge.ts` via `dashboard/src/lib/sim-bridge-client.ts`

### Inbound (dashboard → bridge)

| `op` | Fields | Effect |
|------|--------|--------|
| `mission` | `command`: `"start"` \| `"stop"` \| … | `SimulationEngine.mission_command()` |
| `set_autonomous` | `enabled`: `bool` | Toggle autonomous flight |
| `set_scene` | `scene`: `"apartment"` \| `"control_room"` \| … | Hot-reload scene; emits `scene_changed` |

### Outbound — `hello` (on connect)

```json
{
  "type": "hello",
  "scene": "apartment",
  "visual_mesh_url": "/meshes/apartment.ply",
  "scene_bounds": {
    "min": [x_min, y_min, z_min],
    "max": [x_max, y_max, z_max]
  }
}
```

### Outbound — `scene_changed` (after `set_scene`)

```json
{
  "type": "scene_changed",
  "scene": "apartment",
  "visual_mesh_url": "/meshes/apartment.ply",
  "scene_bounds": { "min": [...], "max": [...] }
}
```

Frontend handlers (`useSimBridge.ts`): `applyVisualMeshUrl` → `drone-store.visualMeshUrl`; `applySceneBounds` → `drone-store.sceneAabb`.

### Outbound — `state` (default 20 Hz)

```json
{
  "type": "state",
  "scene": "apartment",
  "coordinate_frame": "global_ros_z_up",
  "position": [x, y, z],
  "quaternion": [w, x, y, z],
  "velocity": [vx, vy, vz],
  "mission_state": "IDLE",
  "armed": false,
  "autonomous": true,
  "navigation_mode": "semantic_discovery",
  "coverage": 0.0,
  "elapsed_time": 0.0,
  "distance_traveled": 0.0,
  "scene_bounds": {
    "min": [x_min, y_min, z_min],
    "max": [x_max, y_max, z_max],
    "center": [cx, cy, cz],
    "extent": [ex, ey, ez]
  },
  "visual_mesh_url": "/meshes/apartment.ply",
  "lidar": [[x, y, z], ...],
  "patrol_path": [[x, y, z], ...],
  "map_points": [[x, y, z], ...],
  "discovered_map": [
    { "x": 12, "y": 8, "type": "free" },
    { "x": 13, "y": 8, "type": "wall" }
  ],
  "discovery": {
    "known_percent": 4.2,
    "cells_discovered": 120
  },
  "space_analysis": {
    "wall_elements": 14,
    "object_elements": 6,
    "free_percent": 62.1
  },
  "sensors": {
    "min_range_m": 0.42,
    "front_range_m": 1.8,
    "proximity_m": 0.42,
    "open_direction_deg": 90.0,
    "wall_hits": 3,
    "structures": [
      { "id": 1, "kind": "wall", "range_m": 1.2 }
    ]
  },
  "camera_snapshot": { "id": 1, "image_base64": "...", "..." : "..." },
  "camera_gallery": []
}
```

### Semantic class enum (backend)

| Value | Name | Dashboard `discovered_map.type` |
|-------|------|--------------------------------|
| 0 | `unknown` | `unknown` |
| 1 | `free` | `free` |
| 2 | `wall` | `wall` |
| 3 | `object` | `object` |
| 4 | `floor` | `floor` |
| 5 | `ceiling` | `ceiling` |

---

## Active Directory Mapping

```
drone software/                          # Workspace root
├── HANDOFF_CONTEXT.md                   # This file
├── run-aetherscan.sh                    # Primary launcher (--dashboard | --local | --docker)
├── scripts/
│   ├── build_collision_mesh.py          # Replica → collision PLY + labels
│   ├── generate_synthetic_semantics.py    # Normal-heuristic labels (no Replica)
│   ├── validate_collision_mesh.py       # Raycast hit-rate QA
│   └── export-meshes-to-dashboard.py    # Copy Redwood cache → dashboard meshes
│
├── dashboard/                           # Next.js presentation layer (NO physics)
│   ├── public/
│   │   └── meshes/                      # ★ All runtime mesh assets
│   │       ├── {scene}.glb              # PBR visual (optional)
│   │       ├── {scene}.ply              # Clay visual fallback
│   │       ├── {scene}_collision.ply    # Backend collision (<100k tris)
│   │       └── {scene}_collision_labels.npy  # Per-triangle SemanticClass[]
│   └── src/
│       ├── hooks/useSimBridge.ts        # WebSocket client + store hydration
│       ├── stores/drone-store.ts        # Global sim/telemetry state
│       ├── lib/
│       │   ├── ros-three.ts             # ROS → Three coordinate map
│       │   └── scenes.ts                # Scene spawn/bounds defaults
│       └── components/three/
│           ├── MeshEnvironment.tsx      # GLB/PLY loader + Rx(-π/2) group
│           ├── ScanEnvironment.tsx      # Scene router + Suspense
│           ├── DroneModel.tsx           # Position/quaternion remap
│           ├── DiscoveredMap.tsx        # 2D semantic discovery overlay
│           ├── LidarScan.tsx            # Live lidar point cloud
│           └── PathVisualization.tsx    # Patrol path polyline
│
└── redwood_sim/                         # ★ Authoritative physics backend
    ├── bridge/
    │   └── server.py                    # WebSocket server :8765
    ├── config.py                        # physics_dt, control_dt, defaults
    ├── scene_loader.py                  # Mesh resolution, dual-mesh, semantics
    ├── simulation/
    │   └── engine.py                    # SimulationEngine, get_telemetry()
    ├── core/
    │   ├── physics.py                   # RK4 @ 500 Hz
    │   ├── collision.py                 # MeshCollisionSolver
    │   ├── sensors.py                   # LiDAR + primitive_id hits
    │   ├── semantic_space.py            # AnalyzedIndoorSpace, classify_hit
    │   ├── discovery_map.py             # Fog-of-war grid discovery
    │   └── sensor_navigation.py         # SensorNavigator
    ├── data/
    │   ├── redwood/                     # Cached Open3D Redwood scans (~4.7 GB)
    │   └── replica/                     # Replica install target (optional)
    ├── invert_apartment.py              # One-time mesh axis fix (historical)
    └── generate_control.py              # 10×10×3 m control_room test box
```

### Asset placement rules

| File pattern | Consumer | Notes |
|--------------|----------|-------|
| `{scene}_collision.ply` | Python only | Must be Z-up ROS, watertight, ≤100k tris |
| `{scene}_collision_labels.npy` | Python only | `uint8` array, length = triangle count |
| `{scene}.glb` | React only | Swap-and-play PBR; no code change |
| `{scene}.ply` | React only (fallback) | Clay rendering; also used if no collision mesh |

### Key invariants (do not break)

1. **Never run physics in the dashboard** — all poses come from `state.position`
2. **Collision and visual meshes are independent** — different files, different loaders
3. **Coordinates stay Z-up ROS in Python** — only the Three.js group applies Rx(−π/2)
4. **`scene_bounds` are ROS-frame** — do not pre-transform before sending over WebSocket
5. **Static mesh URLs** — no cache-busting query strings on `useLoader`/`useGLTF` paths

---

## Quick verification checklist

```bash
# Synthetic semantics (no Replica)
redwood_sim/.venv/bin/python scripts/generate_synthetic_semantics.py --scene apartment

# Bridge boot should log semantic_triangles
cd redwood_sim && .venv/bin/python -m bridge --scene apartment

# Dashboard
cd dashboard && npm run dev
# Confirm: drone inside apartment, mesh horizontal, discovery map coloring
```

---

*Generated for Claude Code migration. Last updated: 2026-06-02.*
