/** Indoor scene definitions — ROS coords (x, y horizontal, z altitude). */

// All Replica dataset scenes + legacy aliases
export type SceneId =
  // Legacy aliases (kept for backward-compat)
  | 'apartment' | 'office' | 'boardroom'
  // Replica dataset — all 18 scenes
  | 'apartment_0' | 'apartment_1' | 'apartment_2'
  | 'frl_apartment_0' | 'frl_apartment_1' | 'frl_apartment_2'
  | 'frl_apartment_3' | 'frl_apartment_4' | 'frl_apartment_5'
  | 'hotel_0'
  | 'office_0' | 'office_1' | 'office_2' | 'office_3' | 'office_4'
  | 'room_0'   | 'room_1'   | 'room_2'

export type SceneBounds = {
  xmin: number
  xmax: number
  ymin: number
  ymax: number
  z: number
  zmin?: number
  zmax?: number
}

export type SceneAabb = {
  min: [number, number, number]
  max: [number, number, number]
  center?: [number, number, number]
  extent?: [number, number, number]
}

export interface SceneConfig {
  id: SceneId
  label: string
  /** PBR visual GLB under /meshes/ (preferred) */
  visualUrl?: string
  /** Legacy PLY fallback when GLB is not yet provided */
  meshUrl?: string
  /** Backend-only collision mesh: /meshes/{id}_collision.ply */
  collisionMeshUrl?: string
  /** Drone spawn [x, y, z] — overridden by sim telemetry when connected */
  spawn: [number, number, number]
  /** Wander / grid limits — overridden when scene_bounds received from sim */
  bounds: SceneBounds
  gridCenter: [number, number, number]
  gridSize: number
  meshOffset: [number, number, number]
}

/** Default placeholder bounds (overridden by AABB from sim bridge). */
const DEFAULT_BOUNDS: SceneBounds = { xmin: -5, xmax: 5, ymin: -4, ymax: 4, z: 1.5 }
const DEFAULT_SPAWN: [number, number, number] = [0, 0, 0.15]

/**
 * Build a minimal SceneConfig for a Replica scene ID.
 * Bounds, spawn, and GLB URL are updated at runtime from the bridge hello.
 */
function replicaScene(id: SceneId, label: string): SceneConfig {
  return {
    id,
    label,
    visualUrl: `/meshes/${id}.glb`,
    spawn: DEFAULT_SPAWN,
    bounds: DEFAULT_BOUNDS,
    gridCenter: [0, 0, 0],
    gridSize: 24,
    meshOffset: [0, 0, 0],
  }
}

export const SCENES: Record<SceneId, SceneConfig> = {
  // ── Legacy aliases ──────────────────────────────────────────────────────────
  apartment: {
    id: 'apartment',
    label: 'Apartment (alias → apartment_0)',
    visualUrl: '/meshes/apartment_1.glb',
    meshUrl: '/meshes/apartment.ply',
    collisionMeshUrl: '/meshes/apartment_collision.ply',
    spawn: [1.2, 1.2, 0.15],
    bounds: { xmin: 0, xmax: 12, ymin: 0, ymax: 10, z: 1.5 },
    gridCenter: [6, 0, 5],
    gridSize: 24,
    meshOffset: [0, 0, 0],
  },
  office: {
    id: 'office',
    label: 'Office (alias → office_0)',
    visualUrl: '/meshes/office_0.glb',
    spawn: DEFAULT_SPAWN,
    bounds: DEFAULT_BOUNDS,
    gridCenter: [0, 0, 0],
    gridSize: 32,
    meshOffset: [0, 0, 0],
  },
  boardroom: {
    id: 'boardroom',
    label: 'Boardroom (legacy)',
    meshUrl: '/meshes/boardroom.ply',
    spawn: [9, 7, 1.8],
    bounds: { xmin: 0, xmax: 18, ymin: 0, ymax: 14, z: 1.8 },
    gridCenter: [9, 0, 7],
    gridSize: 36,
    meshOffset: [0, 0, 0],
  },

  // ── Replica — Apartments ────────────────────────────────────────────────────
  apartment_0: replicaScene('apartment_0', 'Apartment 0 (Replica)'),
  apartment_1: {
    ...replicaScene('apartment_1', 'Apartment 1 (Replica) ★'),
    visualUrl: '/meshes/apartment_1.glb',  // high-quality existing asset
  },
  apartment_2: replicaScene('apartment_2', 'Apartment 2 (Replica)'),

  // ── Replica — FRL Apartments ────────────────────────────────────────────────
  frl_apartment_0: replicaScene('frl_apartment_0', 'FRL Apartment 0 (Replica)'),
  frl_apartment_1: replicaScene('frl_apartment_1', 'FRL Apartment 1 (Replica)'),
  frl_apartment_2: replicaScene('frl_apartment_2', 'FRL Apartment 2 (Replica)'),
  frl_apartment_3: replicaScene('frl_apartment_3', 'FRL Apartment 3 (Replica)'),
  frl_apartment_4: replicaScene('frl_apartment_4', 'FRL Apartment 4 (Replica)'),
  frl_apartment_5: replicaScene('frl_apartment_5', 'FRL Apartment 5 (Replica)'),

  // ── Replica — Hotel ─────────────────────────────────────────────────────────
  hotel_0: replicaScene('hotel_0', 'Hotel 0 (Replica)'),

  // ── Replica — Offices ───────────────────────────────────────────────────────
  office_0: replicaScene('office_0', 'Office 0 (Replica)'),
  office_1: replicaScene('office_1', 'Office 1 (Replica)'),
  office_2: replicaScene('office_2', 'Office 2 (Replica)'),
  office_3: replicaScene('office_3', 'Office 3 (Replica)'),
  office_4: replicaScene('office_4', 'Office 4 (Replica)'),

  // ── Replica — Rooms ─────────────────────────────────────────────────────────
  room_0: replicaScene('room_0', 'Room 0 (Replica)'),
  room_1: replicaScene('room_1', 'Room 1 (Replica)'),
  room_2: replicaScene('room_2', 'Room 2 (Replica)'),
}

export const SCENE_LIST = Object.values(SCENES)

// ── AABB helpers (same as before) ────────────────────────────────────────────

export function boundsFromAabb(aabb: SceneAabb, margin = 0.35): SceneBounds {
  const [x0, y0, z0] = aabb.min
  const [x1, y1, z1] = aabb.max
  return {
    xmin: x0 + margin,
    xmax: x1 - margin,
    ymin: y0 + margin,
    ymax: y1 - margin,
    z: Math.max(1.0, Math.min(2.2, z1 - z0 - 0.5)),
    zmin: z0,
    zmax: z1,
  }
}

export function gridCenterFromAabb(aabb: SceneAabb): [number, number, number] {
  const [x0, , z0] = aabb.min
  const [x1, , z1] = aabb.max
  return [0.5 * (x0 + x1), 0, 0.5 * (z0 + z1)]
}

export function gridSizeFromAabb(aabb: SceneAabb): number {
  const [x0, y0] = aabb.min
  const [x1, y1] = aabb.max
  const span = Math.max(x1 - x0, y1 - y0)
  return Math.ceil(span + 4)
}

export function mergeSceneWithAabb(
  base: SceneConfig,
  aabb: SceneAabb | null | undefined
): SceneConfig {
  if (!aabb?.min?.length || !aabb?.max?.length) return base
  return {
    ...base,
    bounds: boundsFromAabb(aabb),
    gridCenter: gridCenterFromAabb(aabb),
    gridSize: gridSizeFromAabb(aabb),
  }
}

export function getScene(id: SceneId, aabb?: SceneAabb | null): SceneConfig {
  const base = SCENES[id] ?? SCENES.apartment_1
  return mergeSceneWithAabb(base, aabb)
}
