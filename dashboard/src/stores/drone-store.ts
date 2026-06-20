import { create } from 'zustand'
import type { SceneAabb, SceneId } from '@/lib/scenes'
import { getScene } from '@/lib/scenes'

export type ActivePanel = 'map' | 'nav' | 'camera' | 'metrics' | 'teleop' | 'settings'

/** Live telemetry source; DISCONNECTED = no fake motion. */
export type LinkStatus = 'DISCONNECTED' | 'SIM' | 'ROS'

export interface CameraSnapshot {
  id: number
  timestamp_s: number
  position: [number, number, number]
  /** Drone yaw (radians, ROS Z-up frame) at the moment of capture */
  yaw?: number
  coverage_pct: number
  known_pct: number
  descriptor: string
  image_base64?: string
}

interface DroneState {
  position: [number, number, number]
  /** Body quaternion [w, x, y, z] from physics sim */
  orientation: [number, number, number, number]
  /** Velocity in ROS Z-up frame [vx, vy, vz] m/s */
  velocity: [number, number, number]
  trail: [number, number, number][]
  plannedPath: [number, number, number][]
  /** Each entry is [x, y, z] or [x, y, z, semantic_class] in ROS Z-up frame */
  mapPoints: [number, number, number, number?][]
  lidarHits: [number, number, number][]
  totalPoints: number
  areaMapped: number
  missionState: string
  coverage: number
  elapsedTime: number
  distanceTraveled: number
  armed: boolean
  godMode: boolean
  exportState: { status: 'idle' | 'running' | 'done'; urls: string[]; errors: string[] }
  battery: number
  showStats: boolean
  activePanel: ActivePanel
  linkStatus: LinkStatus
  rosConnected: boolean
  simConnected: boolean
  sceneId: SceneId
  sceneAabb: SceneAabb | null
  /** PBR visual from sim bridge or scene config (GLB preferred) */
  visualMeshUrl: string | null
  /**
   * Normalization offset [ros_x, ros_y, ros_z] sent by the bridge so the
   * dashboard can align the GLB visual mesh with the simulation frame.
   */
  meshNormOffset: [number, number, number] | null
  sensorMinRange: number
  sensorFrontRange: number
  sensorProximity: number
  sensorOpenDeg: number
  sensorWallHits: number
  navigationMode: string
  discoveryKnownPct: number
  spaceWallElements: number
  spaceObjectElements: number
  nearbyStructures: { id: number; kind: string; range_m: number; confidence?: number }[]
  discoveredMapCells: { x: number; y: number; type: string }[]
  cameraGallery: CameraSnapshot[]
  latestCameraSnapshot: CameraSnapshot | null

  setPosition: (pos: [number, number, number]) => void
  setOrientation: (q: [number, number, number, number]) => void
  setVelocity: (v: [number, number, number]) => void
  addTrailPoint: (pos: [number, number, number]) => void
  setPlannedPath: (path: [number, number, number][]) => void
  addMapPoints: (points: [number, number, number, number?][]) => void
  setMapPoints: (points: [number, number, number, number?][]) => void
  setTotalPoints: (n: number) => void
  setAreaMapped: (area: number) => void
  setMissionState: (state: string) => void
  setCoverage: (pct: number) => void
  setElapsedTime: (t: number) => void
  setDistanceTraveled: (d: number) => void
  setArmed: (armed: boolean) => void
  setGodMode: (godMode: boolean) => void
  setExportState: (exportState: { status: 'idle' | 'running' | 'done'; urls: string[]; errors: string[] }) => void
  setBattery: (level: number) => void
  toggleStats: () => void
  setActivePanel: (panel: ActivePanel) => void
  setRosConnected: (c: boolean) => void
  setSimConnected: (c: boolean) => void
  freezeAtSceneSpawn: () => void
  setLidarHits: (pts: [number, number, number][]) => void
  setSceneId: (id: SceneId) => void
  setSceneAabb: (aabb: SceneAabb | null) => void
  setVisualMeshUrl: (url: string | null) => void
  setMeshNormOffset: (offset: [number, number, number] | null) => void
  setSensorTelemetry: (s: {
    minRange?: number
    frontRange?: number
    proximity?: number
    openDeg?: number
    wallHits?: number
    navigationMode?: string
    knownPercent?: number
    wallElements?: number
    objectElements?: number
    structures?: { id: number; kind: string; range_m: number; confidence?: number }[]
    discoveredMap?: { x: number; y: number; type: string }[]
  }) => void
  addCameraSnapshots: (snapshots: CameraSnapshot[]) => void
}

function computeLinkStatus(sim: boolean, ros: boolean): LinkStatus {
  if (sim) return 'SIM'
  if (ros) return 'ROS'
  return 'DISCONNECTED'
}

const initialScene = getScene('apartment')

export const useDroneStore = create<DroneState>((set, get) => ({
  position: [...initialScene.spawn],
  orientation: [1, 0, 0, 0],
  velocity: [0, 0, 0],
  trail: [],
  plannedPath: [],
  mapPoints: [],
  lidarHits: [],
  totalPoints: 0,
  areaMapped: 0,
  missionState: 'IDLE',
  coverage: 0,
  elapsedTime: 0,
  distanceTraveled: 0,
  armed: false,
  godMode: false,
  exportState: { status: 'idle' as const, urls: [], errors: [] },
  battery: 100,
  showStats: false,
  activePanel: 'map',
  linkStatus: 'DISCONNECTED',
  rosConnected: false,
  simConnected: false,
  sceneId: 'apartment',
  sceneAabb: null,
  meshNormOffset: null,
  visualMeshUrl: initialScene.visualUrl ?? initialScene.meshUrl ?? null,
  sensorMinRange: 99,
  sensorFrontRange: 99,
  sensorProximity: 99,
  sensorOpenDeg: 0,
  sensorWallHits: 0,
  navigationMode: 'offline',
  discoveryKnownPct: 0,
  spaceWallElements: 0,
  spaceObjectElements: 0,
  nearbyStructures: [],
  discoveredMapCells: [],
  cameraGallery: [],
  latestCameraSnapshot: null,

  setPosition: (pos) => {
    if (get().linkStatus === 'DISCONNECTED') return
    set({ position: pos })
  },
  setOrientation: (q) => {
    if (get().linkStatus === 'DISCONNECTED') return
    set({ orientation: q })
  },
  setVelocity: (v) => {
    if (get().linkStatus === 'DISCONNECTED') return
    set({ velocity: v })
  },
  addTrailPoint: (pos) => {
    if (get().linkStatus === 'DISCONNECTED') return
    const trail = get().trail
    const last = trail[trail.length - 1]
    if (last) {
      const dx = pos[0] - last[0]
      const dy = pos[1] - last[1]
      const dz = pos[2] - last[2]
      if (dx * dx + dy * dy + dz * dz < 0.01) return
    }
    set({ trail: [...trail, pos].slice(-500) })
  },
  setPlannedPath: (path) => {
    if (get().linkStatus === 'DISCONNECTED') return
    set({ plannedPath: path })
  },
  addMapPoints: (points: [number, number, number, number?][]) => {
    if (get().linkStatus === 'DISCONNECTED') return
    // Points arriving here are voxel-deduplicated deltas from the bridge, so the
    // cloud grows monotonically.  The 500k cap is a hard safety ceiling only
    // (a whole building is well under it) — it must never bite during a normal
    // scan, otherwise oldest-scanned areas would drop out.
    set((s) => ({ mapPoints: [...s.mapPoints, ...points].slice(-500000) }))
  },
  // Replace the whole cloud (used when the bridge sends a full snapshot on
  // connect / after a mission reset, flagged by map_points_reset).
  setMapPoints: (points: [number, number, number, number?][]) =>
    set({ mapPoints: points.slice(-500000) }),
  setTotalPoints: (n) => set({ totalPoints: n }),
  setAreaMapped: (area) => set({ areaMapped: area }),
  setMissionState: (state) => set({ missionState: state }),
  setCoverage: (pct) => set({ coverage: pct }),
  setElapsedTime: (t) => set({ elapsedTime: t }),
  setDistanceTraveled: (d) => set({ distanceTraveled: d }),
  setArmed: (armed) => set({ armed }),
  setGodMode: (godMode) => set({ godMode }),
  setExportState: (exportState) => set({ exportState }),
  setBattery: (level) => set({ battery: level }),
  toggleStats: () => set((s) => ({ showStats: !s.showStats })),
  setActivePanel: (panel) => set({ activePanel: panel }),
  setRosConnected: (c) =>
    set((s) => {
      const rosConnected = c
      const linkStatus = computeLinkStatus(s.simConnected, rosConnected)
      return { rosConnected, linkStatus }
    }),
  setSimConnected: (c) =>
    set((s) => {
      const simConnected = c
      const linkStatus = computeLinkStatus(simConnected, s.rosConnected)
      return { simConnected, linkStatus }
    }),
  freezeAtSceneSpawn: () => {
    const scene = getScene(get().sceneId)
    set({
      linkStatus: 'DISCONNECTED',
      simConnected: false,
      position: [...scene.spawn],
      orientation: [1, 0, 0, 0],
      plannedPath: [],
      lidarHits: [],
      cameraGallery: [],
      latestCameraSnapshot: null,
      armed: false,
      missionState: 'IDLE',
    })
  },
  setLidarHits: (pts) => {
    if (get().linkStatus === 'DISCONNECTED') return
    set({ lidarHits: pts })
  },
  setSensorTelemetry: (s) =>
    set({
      sensorMinRange: s.minRange ?? get().sensorMinRange,
      sensorFrontRange: s.frontRange ?? get().sensorFrontRange,
      sensorProximity: s.proximity ?? get().sensorProximity,
      sensorOpenDeg: s.openDeg ?? get().sensorOpenDeg,
      sensorWallHits: s.wallHits ?? get().sensorWallHits,
      navigationMode: s.navigationMode ?? get().navigationMode,
      discoveryKnownPct: s.knownPercent ?? get().discoveryKnownPct,
      spaceWallElements: s.wallElements ?? get().spaceWallElements,
      spaceObjectElements: s.objectElements ?? get().spaceObjectElements,
      nearbyStructures: s.structures ?? get().nearbyStructures,
      discoveredMapCells: s.discoveredMap ?? get().discoveredMapCells,
    }),
  addCameraSnapshots: (snapshots) => {
    if (!snapshots.length) return
    set((state) => {
      const seen = new Set(state.cameraGallery.map((g) => g.id))
      const merged = [...state.cameraGallery]
      for (const snap of snapshots) {
        if (!seen.has(snap.id)) {
          merged.push(snap)
          seen.add(snap.id)
        }
      }
      const gallery = merged.slice(-300)
      return {
        cameraGallery: gallery,
        latestCameraSnapshot: gallery[gallery.length - 1] ?? null,
      }
    })
  },
  setSceneAabb: (aabb) => set({ sceneAabb: aabb }),
  setVisualMeshUrl: (url) => set({ visualMeshUrl: url }),
  setMeshNormOffset: (offset) => set({ meshNormOffset: offset }),
  setSceneId: (id) => {
    // Use the scene's own defaults (no stale AABB) — switching scenes must NOT
    // inherit the previous scene's bounds or mesh-alignment offset, or the new
    // structure renders misplaced until the bridge happens to re-send a hello.
    const scene = getScene(id, null)
    set({
      sceneId: id,
      visualMeshUrl: scene.visualUrl ?? scene.meshUrl ?? null,
      sceneAabb: null,          // recomputed from the new mesh / next bridge hello
      meshNormOffset: null,     // GLBs are pre-centred; clear cross-scene offset
      position: [...scene.spawn],
      trail: [],
      plannedPath: [],
      mapPoints: [],
      coverage: 0,
      cameraGallery: [],
      latestCameraSnapshot: null,
      elapsedTime: 0,
      distanceTraveled: 0,
      missionState: 'IDLE',
      lidarHits: [],
    })
  },
}))
