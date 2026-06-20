'use client'

import { useEffect, useRef, useCallback, useState } from 'react'
import { useDroneStore, type CameraSnapshot } from '@/stores/drone-store'
import type { SceneAabb, SceneId } from '@/lib/scenes'
import { setSimBridgeSocket, simBridgeSend } from '@/lib/sim-bridge-client'

const SIM_URLS = [
  process.env.NEXT_PUBLIC_SIM_BRIDGE_URL || 'ws://127.0.0.1:8765',
  'ws://localhost:8765',
]

interface SimBridgeHook {
  connected: boolean
  send: (msg: Record<string, unknown>) => void
  setScene: (sceneId: SceneId) => void
  missionCommand: (command: string) => void
}

export function useSimBridge(): SimBridgeHook {
  const [connected, setConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout>>()
  const rosConnected = useDroneStore((s) => s.rosConnected)

  const setPosition = useDroneStore((s) => s.setPosition)
  const setOrientation = useDroneStore((s) => s.setOrientation)
  const setVelocity = useDroneStore((s) => s.setVelocity)
  const setMissionState = useDroneStore((s) => s.setMissionState)
  const setCoverage = useDroneStore((s) => s.setCoverage)
  const setElapsedTime = useDroneStore((s) => s.setElapsedTime)
  const setTotalPoints = useDroneStore((s) => s.setTotalPoints)
  const setAreaMapped = useDroneStore((s) => s.setAreaMapped)
  const setDistanceTraveled = useDroneStore((s) => s.setDistanceTraveled)
  const setArmed = useDroneStore((s) => s.setArmed)
  const setGodMode = useDroneStore((s) => s.setGodMode)
  const addTrailPoint = useDroneStore((s) => s.addTrailPoint)
  const addMapPoints = useDroneStore((s) => s.addMapPoints)
  const setMapPoints = useDroneStore((s) => s.setMapPoints)
  const setPlannedPath = useDroneStore((s) => s.setPlannedPath)
  const setLidarHits = useDroneStore((s) => s.setLidarHits)
  const setSensorTelemetry = useDroneStore((s) => s.setSensorTelemetry)
  const setSimConnected = useDroneStore((s) => s.setSimConnected)
  const freezeAtSceneSpawn = useDroneStore((s) => s.freezeAtSceneSpawn)
  const addCameraSnapshots = useDroneStore((s) => s.addCameraSnapshots)
  const setSceneAabb = useDroneStore((s) => s.setSceneAabb)
  const setVisualMeshUrl = useDroneStore((s) => s.setVisualMeshUrl)
  const setMeshNormOffset = useDroneStore((s) => s.setMeshNormOffset)

  const applyVisualMeshUrl = useCallback(
    (raw: unknown) => {
      if (typeof raw === 'string' && raw.length > 0) {
        setVisualMeshUrl(raw)
      }
    },
    [setVisualMeshUrl]
  )

  const applySceneBounds = useCallback(
    (raw: unknown) => {
      if (!raw || typeof raw !== 'object') return
      const b = raw as { min?: number[]; max?: number[] }
      if (b.min?.length === 3 && b.max?.length === 3) {
        setSceneAabb({
          min: [b.min[0], b.min[1], b.min[2]],
          max: [b.max[0], b.max[1], b.max[2]],
        })
      }
    },
    [setSceneAabb]
  )

  const handleTelemetry = useCallback(
    (msg: Record<string, unknown>) => {
      if (msg.type === 'hello') {
        applySceneBounds(msg.scene_bounds)
        applyVisualMeshUrl(msg.visual_mesh_url)
        // Receive normalization offset so the GLB can be aligned to sim space
        const norm = msg.mesh_norm_offset
        if (Array.isArray(norm) && norm.length === 3) {
          setMeshNormOffset(norm as [number, number, number])
        }
        return
      }
      if (msg.type === 'scene_changed') {
        applyVisualMeshUrl(msg.visual_mesh_url)
        applySceneBounds(msg.scene_bounds)
        return
      }
      if (msg.type === 'export_started') {
        useDroneStore.getState().setExportState({ status: 'running', urls: [], errors: [] })
        return
      }
      if (msg.type === 'export_complete') {
        useDroneStore.getState().setExportState({
          status: 'done',
          urls: Array.isArray(msg.urls) ? (msg.urls as string[]) : [],
          errors: Array.isArray(msg.errors) ? (msg.errors as string[]) : [],
        })
        return
      }
      if (msg.type !== 'state') return

      applySceneBounds(msg.scene_bounds)

      const pos = msg.position as [number, number, number] | undefined
      if (pos?.length === 3) {
        setPosition(pos)
        addTrailPoint(pos)
      }

      const quat = msg.quaternion as [number, number, number, number] | undefined
      if (quat?.length === 4) setOrientation(quat)

      const vel = msg.velocity as [number, number, number] | undefined
      if (vel?.length === 3) setVelocity(vel)

      if (typeof msg.mission_state === 'string') setMissionState(msg.mission_state)
      if (typeof msg.coverage === 'number') setCoverage(msg.coverage)
      if (typeof msg.elapsed_time === 'number') setElapsedTime(msg.elapsed_time)
      if (typeof msg.distance_traveled === 'number') setDistanceTraveled(msg.distance_traveled)
      if (typeof msg.total_points === 'number') setTotalPoints(msg.total_points)
      if (typeof msg.area_mapped === 'number') setAreaMapped(msg.area_mapped)
      if (typeof msg.armed === 'boolean') setArmed(msg.armed)
      if (typeof msg.god_mode === 'boolean') setGodMode(msg.god_mode)

      const patrol = msg.patrol_path as [number, number, number][] | undefined
      if (patrol?.length) setPlannedPath(patrol)

      const lidar = msg.lidar as [number, number, number][] | undefined
      if (lidar?.length) setLidarHits(lidar)

      const mapPts = msg.map_points as [number, number, number, number?][] | undefined
      // Full snapshot (on connect / mission reset) REPLACES the cloud; otherwise
      // the message carries a deduplicated delta that we append.
      if (msg.map_points_reset) setMapPoints(mapPts ?? [])
      else if (mapPts?.length) addMapPoints(mapPts)

      const sensors = msg.sensors as Record<string, unknown> | undefined
      const discovery = msg.discovery as Record<string, unknown> | undefined
      const gallery = msg.camera_gallery as CameraSnapshot[] | undefined
      if (gallery?.length) addCameraSnapshots(gallery)
      else {
        const single = msg.camera_snapshot as CameraSnapshot | undefined
        if (single?.id) addCameraSnapshots([single])
      }

      if (sensors) {
        setSensorTelemetry({
          minRange: Number(sensors.min_range_m ?? 99),
          frontRange: Number(sensors.front_range_m ?? 99),
          proximity: Number(sensors.proximity_m ?? 99),
          openDeg: Number(sensors.open_direction_deg ?? 0),
          wallHits: Number(sensors.wall_hits ?? 0),
          navigationMode: String(msg.navigation_mode ?? 'sensor'),
          knownPercent: Number(discovery?.known_percent ?? 0),
          wallElements: Number(
            (msg.space_analysis as Record<string, unknown>)?.wall_elements ?? 0
          ),
          objectElements: Number(
            (msg.space_analysis as Record<string, unknown>)?.object_elements ?? 0
          ),
          structures: (sensors.structures as { id: number; kind: string; range_m: number }[]) ?? [],
          discoveredMap: (msg.discovered_map as { x: number; y: number; type: string }[]) ?? [],
        })
      }
    },
    [
      setPosition,
      setOrientation,
      setMissionState,
      setCoverage,
      setElapsedTime,
      setDistanceTraveled,
      setTotalPoints,
      setAreaMapped,
      setArmed,
      setGodMode,
      setVelocity,
      addTrailPoint,
      addMapPoints,
      setMapPoints,
      setPlannedPath,
      setLidarHits,
      setSensorTelemetry,
      addCameraSnapshots,
      applySceneBounds,
      applyVisualMeshUrl,
      setMeshNormOffset,
    ]
  )

  const send = useCallback((msg: Record<string, unknown>) => {
    simBridgeSend(msg)
  }, [])

  const missionCommand = useCallback(
    (command: string) => send({ op: 'mission', command }),
    [send]
  )

  const setScene = useCallback(
    (sceneId: SceneId) => send({ op: 'set_scene', scene: sceneId }),
    [send]
  )

  const connect = useCallback(() => {
    if (rosConnected) return

    const url = SIM_URLS[0]
    try {
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => {
        setSimBridgeSocket(ws)
        setConnected(true)
        setSimConnected(true)
      }

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data) as Record<string, unknown>
          handleTelemetry(data)
        } catch {
          /* ignore */
        }
      }

      ws.onclose = () => {
        setSimBridgeSocket(null)
        setConnected(false)
        setSimConnected(false)
        if (!useDroneStore.getState().rosConnected) {
          freezeAtSceneSpawn()
        }
        reconnectTimer.current = setTimeout(connect, 2500)
      }

      ws.onerror = () => ws.close()
    } catch {
      reconnectTimer.current = setTimeout(connect, 2500)
    }
  }, [rosConnected, handleTelemetry, setSimConnected, freezeAtSceneSpawn])

  useEffect(() => {
    if (rosConnected) {
      wsRef.current?.close()
      setConnected(false)
      setSimConnected(false)
      return
    }
    connect()
    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      wsRef.current?.close()
    }
  }, [rosConnected, connect, setSimConnected])

  return { connected, send, setScene, missionCommand }
}
