'use client'

import { useMemo } from 'react'
import * as THREE from 'three'
import { useDroneStore } from '@/stores/drone-store'
import { rosPointToThree } from '@/lib/ros-three'

/** 2D LiDAR rays from the integrated physics sim (ROS Z-up → Three.js Y-up). */
export function LidarScan() {
  const position = useDroneStore((s) => s.position)
  const hits = useDroneStore((s) => s.lidarHits)

  const geometry = useMemo(() => {
    if (!hits.length) return null
    const origin = new THREE.Vector3(...rosPointToThree(position))
    const verts: number[] = []
    for (const p of hits) {
      const [hx, hy, hz] = rosPointToThree(p)
      verts.push(origin.x, origin.y, origin.z)
      verts.push(hx, hy, hz)
    }
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3))
    return geo
  }, [hits, position])

  if (!geometry) return null

  return (
    <lineSegments geometry={geometry}>
      <lineBasicMaterial color="#34d399" transparent opacity={0.45} />
    </lineSegments>
  )
}
