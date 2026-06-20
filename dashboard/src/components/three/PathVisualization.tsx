'use client'

import { useMemo } from 'react'
import * as THREE from 'three'
import { useDroneStore } from '@/stores/drone-store'
import { rosPointToThree } from '@/lib/ros-three'

export function PathVisualization() {
  const path = useDroneStore((s) => s.plannedPath)
  const trail = useDroneStore((s) => s.trail)

  const plannedLine = useMemo(() => {
    if (path.length < 2) return null
    const points = path.map((p) => {
      const [x, y, z] = rosPointToThree(p)
      return new THREE.Vector3(x, y, z)
    })
    const geometry = new THREE.BufferGeometry().setFromPoints(points)
    return geometry
  }, [path])

  const trailLine = useMemo(() => {
    if (trail.length < 2) return null
    const points = trail.map((p) => {
      const [x, y, z] = rosPointToThree(p)
      return new THREE.Vector3(x, y, z)
    })
    const geometry = new THREE.BufferGeometry().setFromPoints(points)
    return geometry
  }, [trail])

  return (
    <>
      {plannedLine && (
        <line>
          <bufferGeometry attach="geometry" {...plannedLine} />
          <lineDashedMaterial
            color="#06b6d4"
            dashSize={0.2}
            gapSize={0.1}
            linewidth={2}
          />
        </line>
      )}

      {trailLine && (
        <line>
          <bufferGeometry attach="geometry" {...trailLine} />
          <lineBasicMaterial color="#22c55e" linewidth={1} transparent opacity={0.6} />
        </line>
      )}

      {path.map((p, i) => {
        const [x, y, z] = rosPointToThree(p)
        return (
          <mesh key={i} position={[x, y, z]}>
            <sphereGeometry args={[0.08]} />
            <meshBasicMaterial color="#06b6d4" transparent opacity={0.5} />
          </mesh>
        )
      })}
    </>
  )
}
