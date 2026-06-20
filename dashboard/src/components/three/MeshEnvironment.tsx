'use client'

import { useLayoutEffect, useMemo } from 'react'
import { useLoader } from '@react-three/fiber'
import { useGLTF, Environment } from '@react-three/drei'
import * as THREE from 'three'
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js'
import { useDroneStore } from '@/stores/drone-store'
import type { SceneAabb } from '@/lib/scenes'
import { rosToThree } from '@/lib/ros-three'

function isGlbUrl(url: string): boolean {
  const lower = url.toLowerCase()
  return lower.endsWith('.glb') || lower.endsWith('.gltf')
}

/** Signed up-axis: which local axis is vertical, and whether +axis points up. */
type SignedUp = { axis: 0 | 1 | 2; positive: boolean }

/**
 * Detect a GLB/GLTF model's vertical ("up") axis — SIGNED — so the corrective
 * rotation can fix sideways (wrong axis) AND upside-down (180°) meshes.
 *
 * Project assets are normalised to canonical Y-up at conversion time, so for our
 * own scenes this is a no-op safety net; it remains load-bearing for arbitrary
 * user-supplied meshes.
 *
 * Method (validated against all 18 Replica scenes): for each of the 6 signed
 * directions, accumulate evidence that the room is upright that way —
 *   • FLOOR evidence: faces near-perpendicular to the axis, in the bottom 20%
 *     band (w.r.t. that direction), whose normal points ALONG the direction
 *     (a floor faces up into the room);
 *   • CEILING evidence: faces in the top band whose normal points AGAINST it.
 * Normal direction is the decisive discriminator (measured margins ±0.6–1.0);
 * raw band area alone confuses ceilings with flipped floors.  If the best and
 * runner-up axes are within 30%, fall back to thinnest-extent (indoor rooms are
 * reliably wider than tall), keeping the evidence-derived sign.
 */
function detectUpAxis(object: THREE.Object3D): SignedUp {
  const box = new THREE.Box3().setFromObject(object)
  const min = box.min
  const max = box.max
  const ext = new THREE.Vector3().subVectors(max, min)
  const minArr = [min.x, min.y, min.z]
  const maxArr = [max.x, max.y, max.z]
  const extArr = [ext.x, ext.y, ext.z]

  // Per-axis evidence.  AXIS comes from inward-facing horizontal slabs at the
  // extremes (floor+ceiling pattern — identical for upright and 180°-flipped,
  // so it is sign-agnostic).  SIGN comes from the area-weighted mass quantile:
  // furniture/clutter concentrates in the lower half of an upright room, so a
  // quantile < 0.5 means +axis is up (normals alone CANNOT separate a floor
  // from a ceiling — both are inward-facing slabs at opposite ends).
  const slabScore = [0, 0, 0]   // inward slab area at the band extremes
  const massMoment = [0, 0, 0]  // Σ area · normalised coordinate along axis
  let totalArea = 0
  const vA = new THREE.Vector3()
  const vB = new THREE.Vector3()
  const vC = new THREE.Vector3()
  const e1 = new THREE.Vector3()
  const e2 = new THREE.Vector3()
  const nrm = new THREE.Vector3()
  const cen = new THREE.Vector3()

  object.updateWorldMatrix(true, true)
  object.traverse((child) => {
    const mesh = child as THREE.Mesh
    if (!mesh.isMesh || !mesh.geometry) return
    const pos = mesh.geometry.getAttribute('position') as THREE.BufferAttribute | undefined
    if (!pos) return
    const mat = mesh.matrixWorld
    const index = mesh.geometry.getIndex()
    const triCount = Math.floor((index ? index.count : pos.count) / 3)
    if (triCount === 0) return
    // Sample up to ~24k triangles for speed on large meshes; margins are wide.
    const stride = Math.max(1, Math.floor(triCount / 24000))
    for (let t = 0; t < triCount; t += stride) {
      const i0 = index ? index.getX(t * 3) : t * 3
      const i1 = index ? index.getX(t * 3 + 1) : t * 3 + 1
      const i2 = index ? index.getX(t * 3 + 2) : t * 3 + 2
      vA.fromBufferAttribute(pos, i0).applyMatrix4(mat)
      vB.fromBufferAttribute(pos, i1).applyMatrix4(mat)
      vC.fromBufferAttribute(pos, i2).applyMatrix4(mat)
      e1.subVectors(vB, vA)
      e2.subVectors(vC, vA)
      nrm.crossVectors(e1, e2)
      const a = nrm.length() * 0.5 // triangle area = |e1×e2|/2
      if (a < 1e-9) continue
      nrm.divideScalar(a * 2) // unit normal
      cen.copy(vA).add(vB).add(vC).divideScalar(3)
      totalArea += a
      const nArr = [nrm.x, nrm.y, nrm.z]
      const cArr = [cen.x, cen.y, cen.z]
      for (let k = 0; k < 3; k++) {
        const q = extArr[k] > 1e-9 ? (cArr[k] - minArr[k]) / extArr[k] : 0.5
        massMoment[k] += a * q
        if (Math.abs(nArr[k]) <= 0.85) continue // not horizontal for this axis
        const lowBand = q < 0.2
        const highBand = q > 0.8
        // Inward-facing slab: floor (low band, normal toward interior = +k) or
        // ceiling (high band, normal toward interior = -k).
        if ((lowBand && nArr[k] > 0) || (highBand && nArr[k] < 0)) slabScore[k] += a
      }
    }
  })

  const order = [0, 1, 2].sort((a, b) => slabScore[b] - slabScore[a]) as (0 | 1 | 2)[]
  let axis = order[0]
  // Ambiguity guard: if the winner doesn't clearly beat the runner-up, trust
  // the thinnest extent (rooms are reliably wider than tall).
  if (!(slabScore[axis] > 1e-6 && slabScore[axis] >= 1.3 * slabScore[order[1]])) {
    axis = 0
    if (extArr[1] <= extArr[0] && extArr[1] <= extArr[2]) axis = 1
    else if (extArr[2] <= extArr[0] && extArr[2] <= extArr[1]) axis = 2
  }
  // Sign from the mass quantile along the chosen axis — CONSERVATIVE.  Correct
  // scans measure q ≈ 0.47–0.55 (ceilings and tall walls keep it near 0.5), so
  // a q<0.5 rule would 180°-flip valid scenes.  Only flip when the mesh is
  // decisively top-heavy, i.e. unambiguously stored upside-down.
  const quantile = totalArea > 1e-9 ? massMoment[axis] / totalArea : 0.5
  return { axis, positive: quantile < 0.62 }
}

/** Corrective rotation mapping the model's signed up direction → Three's +Y. */
function uprightRotationFor(up: SignedUp): [number, number, number] {
  const { axis, positive } = up
  if (axis === 0) return positive ? [0, 0, Math.PI / 2] : [0, 0, -Math.PI / 2]   // ±X → +Y
  if (axis === 1) return positive ? [0, 0, 0] : [Math.PI, 0, 0]                  // -Y → 180° about X
  return positive ? [-Math.PI / 2, 0, 0] : [Math.PI / 2, 0, 0]                   // ±Z → +Y
}

function aabbFromZUpGeometry(geometry: THREE.BufferGeometry): SceneAabb {
  geometry.computeBoundingBox()
  const box = geometry.boundingBox!
  return {
    min: [box.min.x, box.min.y, box.min.z],
    max: [box.max.x, box.max.y, box.max.z],
  }
}

type MeshEnvironmentProps = {
  url: string
  offset?: [number, number, number]
}

function SceneLighting() {
  const sceneAabb = useDroneStore((s) => s.sceneAabb)

  const ceilY = useMemo(() => {
    if (!sceneAabb) return 8
    return sceneAabb.max[2] * 0.9
  }, [sceneAabb])

  // Ceiling-mounted light grid — 5 fixtures spread across the room
  const ceilLights = useMemo((): [number, number, number][] => {
    if (!sceneAabb) return [[0, ceilY, 0]]
    const cx = (sceneAabb.min[0] + sceneAabb.max[0]) / 2
    const cy = (sceneAabb.min[1] + sceneAabb.max[1]) / 2
    const rx = (sceneAabb.max[0] - sceneAabb.min[0]) * 0.28
    const ry = (sceneAabb.max[1] - sceneAabb.min[1]) * 0.28
    return [
      [cx,      ceilY, cy     ],
      [cx + rx, ceilY, cy + ry],
      [cx - rx, ceilY, cy + ry],
      [cx + rx, ceilY, cy - ry],
      [cx - rx, ceilY, cy - ry],
    ]
  }, [sceneAabb, ceilY])

  return (
    <>
      {/* Neutral studio IBL — drives crisp PBR reflections that read as surface
          texture, without the warm wash of the old "apartment" preset. */}
      <Environment preset="warehouse" background={false} />

      {/* Low cold ambient — keeps the shadows deep so the scene stays mysterious
          while still lifting the darkest crevices off pure black. */}
      <ambientLight intensity={0.5} color="#aeb9c4" />

      {/* Primary key — a single hard, cool light raking across the geometry from
          high to one side.  Raking light + sharp shadows is what reveals the
          micro-relief of the scan (this is where "texture" comes from). */}
      <directionalLight
        position={[6, ceilY + 3, 4]}
        intensity={3.1}
        color="#e8eef4"
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-far={60}
        shadow-camera-left={-20}
        shadow-camera-right={20}
        shadow-camera-top={20}
        shadow-camera-bottom={-20}
        shadow-bias={-0.0003}
      />

      {/* Cold rim from behind for edge separation against the void. */}
      <directionalLight position={[-9, ceilY * 0.6, -8]} intensity={0.9} color="#6f8aa6" />

      {/* A few dim, cool pools of light — atmosphere, not illumination. */}
      {ceilLights.slice(0, 3).map((pos, i) => (
        <pointLight key={i} position={pos} intensity={1.3} distance={11} decay={2} color="#bcccd8" />
      ))}

      {/* Cold sky / black floor hemisphere — moonlit feel. */}
      <hemisphereLight args={['#26323d', '#000000', 0.4]} />
    </>
  )
}

/** PLY renderer — splits geometry into 4 material zones including transparent ceiling. */
function LoadedPlyMesh({ url, offset = [0, 0, 0] }: MeshEnvironmentProps) {
  const geometry = useLoader(PLYLoader, url) as THREE.BufferGeometry
  const setSceneAabb = useDroneStore((s) => s.setSceneAabb)

  const { floorGeom, wallGeom, objectGeom, ceilGeom, rosAabb } = useMemo(() => {
    const g = geometry.clone()
    const rosAabb = aabbFromZUpGeometry(g)
    g.computeVertexNormals()

    const pos  = g.getAttribute('position') as THREE.BufferAttribute
    const norm = g.getAttribute('normal')   as THREE.BufferAttribute
    const idx  = g.index

    const floorIdx: number[] = []
    const wallIdx:  number[] = []
    const objIdx:   number[] = []
    const ceilIdx:  number[] = []

    // Ceiling threshold: top 12 % of Z extent (in Z-up space, before Rx rotation)
    const zRange   = rosAabb.max[2] - rosAabb.min[2]
    const ceilZMin = rosAabb.max[2] - zRange * 0.12
    const floorZMax = rosAabb.min[2] + zRange * 0.12

    const triCount = Math.floor((idx ? idx.count : pos.count) / 3)

    for (let t = 0; t < triCount; t++) {
      const i0 = idx ? idx.getX(t * 3)     : t * 3
      const i1 = idx ? idx.getX(t * 3 + 1) : t * 3 + 1
      const i2 = idx ? idx.getX(t * 3 + 2) : t * 3 + 2

      const nz    = (norm.getZ(i0) + norm.getZ(i1) + norm.getZ(i2)) / 3
      const absNz = Math.abs(nz)
      // Z centroid in ROS Z-up frame (pos.getZ in the pre-rotation PLY)
      const cz    = (pos.getZ(i0) + pos.getZ(i1) + pos.getZ(i2)) / 3

      if (absNz > 0.7) {
        if (cz >= ceilZMin) {
          ceilIdx.push(i0, i1, i2)           // top horizontal → ceiling (transparent)
        } else if (cz <= floorZMax) {
          floorIdx.push(i0, i1, i2)          // bottom horizontal → floor
        } else {
          objIdx.push(i0, i1, i2)            // mid-height horizontal → furniture top
        }
      } else if (absNz < 0.3) {
        wallIdx.push(i0, i1, i2)             // near-vertical → wall
      } else {
        objIdx.push(i0, i1, i2)              // oblique → object side
      }
    }

    function subGeom(indices: number[]) {
      const sg = g.clone()
      sg.setIndex(indices)
      sg.computeVertexNormals()
      return sg
    }

    return {
      floorGeom:  subGeom(floorIdx),
      wallGeom:   subGeom(wallIdx),
      objectGeom: subGeom(objIdx),
      ceilGeom:   subGeom(ceilIdx),
      rosAabb,
    }
  }, [geometry])

  useLayoutEffect(() => {
    setSceneAabb(rosAabb)
    return () => {
      floorGeom.dispose()
      wallGeom.dispose()
      objectGeom.dispose()
      ceilGeom.dispose()
    }
  }, [floorGeom, wallGeom, objectGeom, ceilGeom, rosAabb, setSceneAabb])

  return (
    <>
      <SceneLighting />
      <group position={offset} rotation={[-Math.PI / 2, 0, 0]}>
        {/* Floor — dark polished stone with a faint cold sheen */}
        <mesh geometry={floorGeom} receiveShadow>
          <meshStandardMaterial color="#16191d" roughness={0.42} metalness={0.18} envMapIntensity={0.9} side={THREE.DoubleSide} />
        </mesh>

        {/* Walls — matte charcoal plaster */}
        <mesh geometry={wallGeom} castShadow receiveShadow>
          <meshStandardMaterial color="#23262b" roughness={0.9} metalness={0.0} envMapIntensity={0.5} side={THREE.DoubleSide} />
        </mesh>

        {/* Furniture / objects — graphite with a hint of metal */}
        <mesh geometry={objectGeom} castShadow receiveShadow>
          <meshStandardMaterial color="#2b2f35" roughness={0.6} metalness={0.22} envMapIntensity={0.8} side={THREE.DoubleSide} />
        </mesh>

        {/* Ceiling — frosted glass panel, fully transparent so camera sees inside */}
        <mesh geometry={ceilGeom}>
          <meshStandardMaterial
            color="#e8f0ff"
            roughness={0.05}
            metalness={0.1}
            transparent
            opacity={0.15}
            depthWrite={false}
            side={THREE.DoubleSide}
          />
        </mesh>
      </group>
    </>
  )
}

/** GLB renderer — makes ceiling-height horizontal faces transparent so the interior is visible. */
function LoadedGltfMesh({ url, offset = [0, 0, 0] }: MeshEnvironmentProps) {
  const { scene } = useGLTF(url)
  const setSceneAabb = useDroneStore((s) => s.setSceneAabb)
  const model = useMemo(() => scene.clone(true), [scene])

  // Per-mesh up-axis detection → corrective rotation so every GLB renders
  // upright, regardless of whether it was exported Z-up or Y-up.
  const upAxis = useMemo(() => detectUpAxis(model), [model])
  const uprightRotation = useMemo(() => uprightRotationFor(upAxis), [upAxis])

  // The GLB is exported ALREADY in the simulation's normalised frame (the
  // converter centres XY and puts the floor at 0 — the same transform the bridge
  // applies to the collision mesh / point cloud).  So it renders at the origin
  // and overlays the point cloud directly.  We must NOT add the bridge's
  // mesh_norm_offset here: that is the RAW→normalised translation, and adding it
  // to an already-normalised GLB double-shifts it off the scan (the cause of the
  // mesh floating away from the point cloud).
  const groupOffset = useMemo<[number, number, number]>(() => offset, [offset])

  useLayoutEffect(() => {
    // Ceiling threshold is measured along the model's DETECTED SIGNED up-axis
    // (local ±X/±Y/±Z) — the "ceiling" is the far end of the up direction, so
    // for a negative-up mesh it sits at the axis MINIMUM.
    const axisKey = (['x', 'y', 'z'] as const)[upAxis.axis]
    const box = new THREE.Box3().setFromObject(model)
    const upRange  = box.max[axisKey] - box.min[axisKey]
    const ceilThresh = upAxis.positive
      ? box.max[axisKey] - upRange * 0.12
      : box.min[axisKey] + upRange * 0.12

    model.traverse((child) => {
      if (!(child as THREE.Mesh).isMesh) return
      const mesh = child as THREE.Mesh

      // Ceiling geometry: centroid in the top 12% band along the signed up-axis
      const geo = mesh.geometry
      geo.computeBoundingBox()
      const centroidUp = (geo.boundingBox!.min[axisKey] + geo.boundingBox!.max[axisKey]) / 2
      geo.computeVertexNormals()

      const isCeiling = upAxis.positive ? centroidUp > ceilThresh : centroidUp < ceilThresh

      if (isCeiling) {
        mesh.material = new THREE.MeshStandardMaterial({
          color: '#e8f0ff',
          roughness: 0.05,
          metalness: 0.1,
          transparent: true,
          opacity: 0.15,
          depthWrite: false,
          side: THREE.DoubleSide,
        })
      } else {
        mesh.castShadow    = true
        mesh.receiveShadow = true
        // Boost IBL influence on the baked Replica textures
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material]
        mats.forEach((mat) => {
          const m = mat as THREE.MeshStandardMaterial
          if (m.isMeshStandardMaterial) {
            // Subtle env reflections + a touch of spec break-up so the baked
            // vertex colours read as a real, slightly worn surface in the dark.
            m.envMapIntensity = 0.85
            m.roughness = Math.min(1, (m.roughness ?? 0.9) * 0.85 + 0.1)
            m.metalness = Math.max(m.metalness ?? 0, 0.04)
            m.needsUpdate = true
          }
        })
      }
    })

    // Don't override sceneAabb here — the bridge sends the normalised bounds
    // which are the ground truth for point-cloud alignment.
  }, [model, upAxis])

  return (
    <>
      <SceneLighting />
      {/* uprightRotation is auto-detected from the model's geometry so the mesh
          stands up whether it was exported Z-up (→ Rx(-π/2), ROS convention) or
          Y-up (→ identity).  groupOffset aligns the original Replica coordinates
          to the sim's normalised (centred, floor-at-0) coordinate frame. */}
      <group position={groupOffset} rotation={uprightRotation}>
        <primitive object={model} />
      </group>
    </>
  )
}

export function MeshEnvironment({ url, offset = [0, 0, 0] }: MeshEnvironmentProps) {
  if (isGlbUrl(url)) {
    return <LoadedGltfMesh url={url} offset={offset} />
  }
  return <LoadedPlyMesh url={url} offset={offset} />
}
