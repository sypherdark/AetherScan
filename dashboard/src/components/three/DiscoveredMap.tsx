'use client'

import { useMemo } from 'react'
import * as THREE from 'three'
import { useDroneStore } from '@/stores/drone-store'
import { rosToThree } from '@/lib/ros-three'

const CELL_COLORS: Record<string, [number, number, number]> = {
  unknown: [0.45, 0.45, 0.48],
  free: [0.95, 0.95, 0.95],
  wall: [0.08, 0.08, 0.08],
  object: [0.95, 0.55, 0.12],
  floor: [0.55, 0.42, 0.28],
  ceiling: [0.35, 0.55, 0.85],
}

/** Top-down discovered cells around the drone (ROS xy → Three xz). */
export function DiscoveredMap() {
  const position = useDroneStore((s) => s.position)
  const discovered = useDroneStore((s) => s.discoveredMapCells)

  const points = useMemo(() => {
    if (!discovered?.length) return null
    const verts: number[] = []
    const colors: number[] = []
    for (const c of discovered) {
      const dx = c.x - position[0]
      const dy = c.y - position[1]
      if (dx * dx + dy * dy > 16) continue
      const [tx, ty, tz] = rosToThree(c.x, c.y, 0.08)
      verts.push(tx, ty, tz)
      const rgb = CELL_COLORS[c.type] ?? CELL_COLORS.unknown
      colors.push(rgb[0], rgb[1], rgb[2])
    }
    if (!verts.length) return null
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3))
    geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))
    return geo
  }, [discovered, position])

  if (!points) return null

  return (
    <points geometry={points}>
      <pointsMaterial size={0.12} vertexColors sizeAttenuation />
    </points>
  )
}
