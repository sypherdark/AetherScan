'use client'

import { useEffect, useRef, useCallback, useState } from 'react'
import { useDroneStore } from '@/stores/drone-store'

const BRIDGE_URLS = [
  process.env.NEXT_PUBLIC_ROSBRIDGE_URL || 'ws://localhost:9090',
  'ws://localhost:9090',
  'ws://localhost:9091',
]

interface RosBridgeHook {
  connected: boolean
  callService: (name: string, type: string, request: Record<string, unknown>) => void
}

export function useRosBridge(): RosBridgeHook {
  const [connected, setConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)
  const urlIndex = useRef(0)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout>>()
  const serviceId = useRef(0)

  const setPosition = useDroneStore((s) => s.setPosition)
  const setMissionState = useDroneStore((s) => s.setMissionState)
  const setCoverage = useDroneStore((s) => s.setCoverage)
  const setElapsedTime = useDroneStore((s) => s.setElapsedTime)
  const setTotalPoints = useDroneStore((s) => s.setTotalPoints)
  const setAreaMapped = useDroneStore((s) => s.setAreaMapped)
  const setDistanceTraveled = useDroneStore((s) => s.setDistanceTraveled)
  const setArmed = useDroneStore((s) => s.setArmed)
  const addTrailPoint = useDroneStore((s) => s.addTrailPoint)
  const addMapPoints = useDroneStore((s) => s.addMapPoints)
  const setRosConnected = useDroneStore((s) => s.setRosConnected)
  const freezeAtSceneSpawn = useDroneStore((s) => s.freezeAtSceneSpawn)

  const subscribe = (ws: WebSocket, topic: string, type: string) => {
    ws.send(
      JSON.stringify({
        op: 'subscribe',
        topic,
        type,
        throttle_rate: 100,
      })
    )
  }

  const handleMessage = useCallback(
    (data: { op?: string; topic?: string; msg?: Record<string, unknown> }) => {
      if (data.op !== 'publish') return

      switch (data.topic) {
        case '/aetherscan/mission/status': {
          try {
            const status = JSON.parse((data.msg as { data: string }).data)
            setMissionState(status.state || 'IDLE')
            setCoverage(status.coverage_percent || 0)
            setElapsedTime(status.elapsed_time_sec || 0)
            setDistanceTraveled(status.distance_traveled_m || 0)
            setTotalPoints(status.total_points || 0)
            if (status.position) {
              const p = status.position as [number, number, number]
              setPosition(p)
              addTrailPoint(p)
            }
          } catch {
            /* ignore */
          }
          break
        }
        case '/aetherscan/odom': {
          const pose = data.msg as {
            pose?: { pose?: { position?: { x: number; y: number; z: number } } }
          }
          const pos = pose?.pose?.pose?.position
          if (pos) {
            setPosition([pos.x, pos.y, pos.z])
            addTrailPoint([pos.x, pos.y, pos.z])
          }
          break
        }
        case '/aetherscan/map/stats': {
          try {
            const stats = JSON.parse((data.msg as { data: string }).data)
            setTotalPoints(stats.total_points || 0)
            setAreaMapped(stats.area_m2 || 0)
          } catch {
            /* ignore */
          }
          break
        }
        case '/aetherscan/map/point_cloud': {
          const pts = parsePointCloud(data.msg)
          if (pts.length) addMapPoints(pts)
          break
        }
        case '/aetherscan/controller/status': {
          const statusStr = (data.msg as { data?: string })?.data || ''
          setArmed(statusStr.includes('armed=True'))
          break
        }
      }
    },
    [
      setMissionState,
      setCoverage,
      setElapsedTime,
      setDistanceTraveled,
      setTotalPoints,
      setPosition,
      addTrailPoint,
      setAreaMapped,
      addMapPoints,
      setArmed,
    ]
  )

  const connect = useCallback(() => {
    const url = BRIDGE_URLS[urlIndex.current % BRIDGE_URLS.length]

    try {
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => {
        setConnected(true)
        setRosConnected(true)
        subscribe(ws, '/aetherscan/mission/status', 'std_msgs/msg/String')
        subscribe(ws, '/aetherscan/odom', 'nav_msgs/msg/Odometry')
        subscribe(ws, '/aetherscan/map/stats', 'std_msgs/msg/String')
        subscribe(ws, '/aetherscan/map/point_cloud', 'sensor_msgs/msg/PointCloud2')
        subscribe(ws, '/aetherscan/controller/status', 'std_msgs/msg/String')
      }

      ws.onmessage = (event) => {
        try {
          handleMessage(JSON.parse(event.data))
        } catch {
          /* ignore */
        }
      }

      ws.onclose = () => {
        setConnected(false)
        setRosConnected(false)
        if (!useDroneStore.getState().simConnected) {
          freezeAtSceneSpawn()
        }
        urlIndex.current += 1
        reconnectTimer.current = setTimeout(connect, 2500)
      }

      ws.onerror = () => ws.close()
    } catch {
      reconnectTimer.current = setTimeout(connect, 2500)
    }
  }, [handleMessage, setRosConnected, freezeAtSceneSpawn])

  const callService = useCallback(
    (name: string, type: string, request: Record<string, unknown>) => {
      if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
        return
      }
      serviceId.current += 1
      wsRef.current.send(
        JSON.stringify({
          op: 'call_service',
          id: `call_${serviceId.current}`,
          service: name,
          type,
          args: request,
        })
      )
    },
    []
  )

  useEffect(() => {
    connect()
    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      wsRef.current?.close()
    }
  }, [connect])

  return { connected, callService }
}

function parsePointCloud(msg: unknown): [number, number, number][] {
  const m = msg as {
    width?: number
    point_step?: number
    data?: string | number[]
    fields?: { name: string; offset: number }[]
  }
  if (!m?.width || !m.data) return []

  let bytes: Uint8Array
  if (typeof m.data === 'string') {
    const bin = atob(m.data)
    bytes = new Uint8Array(bin.length)
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i)
  } else {
    bytes = new Uint8Array(m.data as number[])
  }

  const xOff = m.fields?.find((f) => f.name === 'x')?.offset ?? 0
  const yOff = m.fields?.find((f) => f.name === 'y')?.offset ?? 4
  const zOff = m.fields?.find((f) => f.name === 'z')?.offset ?? 8
  const step = m.point_step ?? 12
  const pts: [number, number, number][] = []
  const view = new DataView(bytes.buffer)

  for (let i = 0; i < Math.min(m.width, 2000); i += 4) {
    const o = i * step
    pts.push([
      view.getFloat32(o + xOff, true),
      view.getFloat32(o + yOff, true),
      view.getFloat32(o + zOff, true),
    ])
  }
  return pts
}
