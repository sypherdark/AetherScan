'use client'

import { useRef, useMemo } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { useDroneStore } from '@/stores/drone-store'

/**
 * Semantic class IDs sent by the Python bridge (mirrors SemanticClass enum).
 *  0 UNKNOWN  1 FREE  2 WALL  3 OBJECT  4 FLOOR  5 CEILING
 */
const SEMANTIC_COLORS: Record<number, [number, number, number]> = {
  0: [0.45, 0.45, 0.45],   // UNKNOWN  — mid grey
  1: [0.20, 0.75, 0.20],   // FREE     — green (rarely visible)
  2: [0.78, 0.82, 0.88],   // WALL     — cool light grey / plaster
  3: [0.92, 0.55, 0.18],   // OBJECT   — warm orange / furniture
  4: [0.62, 0.50, 0.35],   // FLOOR    — sandy tan
  5: [0.55, 0.80, 1.00],   // CEILING  — sky blue
}
const FALLBACK_COLOR: [number, number, number] = [0.45, 0.45, 0.45]

function semanticColor(label: number | undefined): [number, number, number] {
  if (label === undefined) return heightFallback(0)
  return SEMANTIC_COLORS[label] ?? FALLBACK_COLOR
}

/** Height-based jet gradient — used as fallback when no label is provided. */
function heightFallback(t: number): [number, number, number] {
  const c = t < 0.25
    ? [0, t / 0.25, 1]
    : t < 0.5
    ? [0, 1, 1 - (t - 0.25) / 0.25]
    : t < 0.75
    ? [(t - 0.5) / 0.25, 1, 0]
    : [1, 1 - (t - 0.75) / 0.25, 0]
  return c as [number, number, number]
}

// Hard ceiling on rendered points; the bridge cloud is voxel-deduplicated so a
// whole building stays well under this.
const MAX_POINTS = 500_000

/**
 * Streaming point-cloud renderer.
 *
 * Points arrive as an ever-growing, deduplicated array in the store.  This
 * component uploads ONLY the newly-added points into a preallocated GPU buffer
 * each frame (tracking a high-water mark) and never rebuilds the whole cloud —
 * the previous implementation reallocated a Float32Array for ALL points every
 * frame, which at 100k+ points stalled the main thread (and, combined with the
 * old 200k FIFO buffer, made early-scanned areas vanish).
 */
export function PointCloudRenderer() {
  const pointsRef = useRef<THREE.Points>(null)
  const mapPoints = useDroneStore((s) => s.mapPoints)
  const uploaded = useRef(0)

  const { geometry, positions, colors, material } = useMemo(() => {
    const positions = new Float32Array(MAX_POINTS * 3)
    const colors = new Float32Array(MAX_POINTS * 3)
    const geo = new THREE.BufferGeometry()
    const posAttr = new THREE.BufferAttribute(positions, 3)
    const colAttr = new THREE.BufferAttribute(colors, 3)
    posAttr.setUsage(THREE.DynamicDrawUsage)
    colAttr.setUsage(THREE.DynamicDrawUsage)
    geo.setAttribute('position', posAttr)
    geo.setAttribute('color', colAttr)
    geo.setDrawRange(0, 0)
    const mat = new THREE.PointsMaterial({
      size: 0.045,
      vertexColors: true,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.95,
    })
    return { geometry: geo, positions, colors, material: mat }
  }, [])

  useFrame(() => {
    const pts = mapPoints
    const total = Math.min(pts.length, MAX_POINTS)

    // Cloud shrank → the mission reset (store replaced the array); rebuild.
    if (total < uploaded.current) uploaded.current = 0

    if (total === uploaded.current) return // nothing new this frame

    for (let i = uploaded.current; i < total; i++) {
      const p = pts[i]
      const idx = i * 3
      // ROS Z-up → Three.js Y-up:  (rx, ry, rz) → (rx, rz, -ry)
      positions[idx] = p[0]
      positions[idx + 1] = p[2]
      positions[idx + 2] = -p[1]
      const c = p[3] !== undefined
        ? semanticColor(p[3])
        : heightFallback(Math.max(0, Math.min(1, p[2] / 3.5)))
      colors[idx] = c[0]
      colors[idx + 1] = c[1]
      colors[idx + 2] = c[2]
    }

    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute
    const colAttr = geometry.getAttribute('color') as THREE.BufferAttribute
    posAttr.needsUpdate = true
    colAttr.needsUpdate = true
    geometry.setDrawRange(0, total)
    if (uploaded.current === 0) geometry.computeBoundingSphere()
    uploaded.current = total
  })

  return <points ref={pointsRef} geometry={geometry} material={material} frustumCulled={false} />
}
