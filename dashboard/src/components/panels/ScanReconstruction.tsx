'use client'

/**
 * ScanReconstruction — high-fidelity 3-D progressive reveal
 * ──────────────────────────────────────────────────────────
 * Architecture (v4)
 * ─────────────────
 * • A 512×512 Float32 DataTexture (the "reveal grid") lives in XZ physics
 *   space.  Each cell covers ≈2.7 cm² of floor plan.
 * • Every LIDAR hit writes 1.0 into the cell at (rx, -ry) — where (rx,ry,rz)
 *   is the hit in ROS Z-up physics frame.  Camera-snapshot positions also
 *   splash a wider radius so un-lit areas near the drone fill in naturally.
 * • The GLB fragment shader converts vWorldPos (already in physics Three.js
 *   space due to the normalization transform exactly cancelling the GLB
 *   position offset) to a grid UV in O(1) — no loop, no DataTexture 256-entry
 *   cap, no rolling window.
 * • Once a cell is lit it stays lit.  The grid resets only on scene change.
 * • A pulsing cyan scan ring follows the drone in real time.
 * • Hemisphere + directional lighting give surfaces depth and shadow so the
 *   emerging structure reads as genuinely 3-D.
 *
 * Coordinate proof
 * ────────────────
 *   _replica_normalize_matrix applies:  normalized = original + T[:3,3]
 *   The GLB is placed at position       [T[0,3], T[2,3], −T[1,3]] in Three.js.
 *   Therefore:  vWorldPos = glb_vertex + offset
 *             = (orig_x, orig_z, −orig_y) + (T[0,3], T[2,3], −T[1,3])
 *             = (orig_x + T[0,3], orig_z + T[2,3], −orig_y − T[1,3])
 *             = (normalized_x, normalized_z, −normalized_y)
 *             = (rx, rz, −ry)  in physics Three.js Y-up frame  ✓
 *   So lidar hit (rx,ry,rz) → grid at (rx, −ry) matches vWorldPos.xz exactly.
 */

import { useMemo, useEffect, useRef } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { OrbitControls, useGLTF } from '@react-three/drei'
import * as THREE from 'three'
import { Download, Loader2, FileBox, Map as MapIcon, Boxes } from 'lucide-react'
import { useDroneStore } from '@/stores/drone-store'
import { simBridgeSend } from '@/lib/sim-bridge-client'

// ─── Grid config ──────────────────────────────────────────────────────────────
/** Texels per axis.  512 ≈ 2.7 cm/cell for a 14 m-wide room. */
const GRID_RES  = 512
/** Grid covers ±GRID_HALF metres in physics Three.js XZ. */
const GRID_HALF = 10.0
const GRID_SPAN = GRID_HALF * 2   // 20 m total

// ─── Module-level reveal state (outlives React tree) ─────────────────────────
const _gridData = new Float32Array(GRID_RES * GRID_RES)

const _gridTex = new THREE.DataTexture(
  _gridData, GRID_RES, GRID_RES,
  THREE.RedFormat, THREE.FloatType,
)
_gridTex.minFilter   = THREE.LinearFilter
_gridTex.magFilter   = THREE.LinearFilter
_gridTex.wrapS       = THREE.ClampToEdgeWrapping
_gridTex.wrapT       = THREE.ClampToEdgeWrapping
_gridTex.needsUpdate = true

/** Uniforms shared across all mesh materials — stable references. */
const _uni = {
  tGrid    : { value: _gridTex },
  uHalfSpan: { value: GRID_HALF },
  uSpan    : { value: GRID_SPAN },
  uDroneXZ : { value: new THREE.Vector2(0, 0) },
  uScanTime: { value: 0.0 },
}

let _lastSceneId = ''

function resetGrid(): void {
  _gridData.fill(0)
  _gridTex.needsUpdate = true
}

/** Paint a filled circle of `r` cells around grid cell (cx, cz). */
function splatCircle(cx: number, cz: number, r: number): void {
  const r2 = r * r
  const x0 = Math.max(0, cx - r), x1 = Math.min(GRID_RES - 1, cx + r)
  const z0 = Math.max(0, cz - r), z1 = Math.min(GRID_RES - 1, cz + r)
  for (let iz = z0; iz <= z1; iz++) {
    const dz = iz - cz
    for (let ix = x0; ix <= x1; ix++) {
      const dx = ix - cx
      if (dx * dx + dz * dz <= r2) _gridData[iz * GRID_RES + ix] = 1.0
    }
  }
}

/** Map physics (rx, ry_ros) to grid cell, or null if out-of-bounds. */
function toCell(rx: number, ry: number): [number, number] | null {
  const u = (rx  + GRID_HALF) / GRID_SPAN   // Three.js X = ROS X
  const v = (-ry + GRID_HALF) / GRID_SPAN   // Three.js Z = -ROS Y
  if (u < 0 || u > 1 || v < 0 || v > 1) return null
  return [
    Math.min(GRID_RES - 1, (u * GRID_RES) | 0),
    Math.min(GRID_RES - 1, (v * GRID_RES) | 0),
  ]
}

// ─── GLSL shaders ─────────────────────────────────────────────────────────────
const VERT = /* glsl */`
  attribute vec3 color;          /* vertex colour from GLB/PLY (may be zero)  */

  varying vec2 vUv;
  varying vec3 vWorldPos;
  varying vec3 vVertColor;

  void main() {
    vUv        = uv;
    vWorldPos  = (modelMatrix * vec4(position, 1.0)).xyz;
    vVertColor = color;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`

const FRAG = /* glsl */`
  precision highp float;

  uniform sampler2D tDiffuse;
  uniform float     uHasMap;       /* 1=UV texture,  0=use vertex colour / flat */
  uniform float     uHasVtxColor;  /* 1=vertex colours present in geometry      */
  uniform vec3      uBaseColor;

  uniform sampler2D tGrid;      /* 512×512 XZ occupancy (physics Three.js) */
  uniform float     uHalfSpan;
  uniform float     uSpan;

  uniform vec2  uDroneXZ;       /* drone physics Three.js (x, z) */
  uniform float uScanTime;

  varying vec2 vUv;
  varying vec3 vWorldPos;
  varying vec3 vVertColor;

  /* Weighted 3×3 tap for soft reveal edges */
  float sampleGrid(vec2 uv) {
    float s  = 1.5 / 512.0;
    float c  = texture2D(tGrid, uv).r;
    float nb = texture2D(tGrid, uv + vec2( s,  0)).r
             + texture2D(tGrid, uv + vec2(-s,  0)).r
             + texture2D(tGrid, uv + vec2( 0,  s)).r
             + texture2D(tGrid, uv + vec2( 0, -s)).r;
    float diag = texture2D(tGrid, uv + vec2( s,  s)).r
               + texture2D(tGrid, uv + vec2(-s,  s)).r
               + texture2D(tGrid, uv + vec2( s, -s)).r
               + texture2D(tGrid, uv + vec2(-s, -s)).r;
    return (c * 4.0 + nb * 2.0 + diag) / 20.0;
  }

  void main() {
    /* ── Grid lookup O(1) ──────────────────────────────────────────────────── */
    vec2 gridUv = (vWorldPos.xz + uHalfSpan) / uSpan;
    bool inGrid = gridUv.x >= 0.001 && gridUv.x <= 0.999
               && gridUv.y >= 0.001 && gridUv.y <= 0.999;

    float revealed = inGrid ? sampleGrid(clamp(gridUv, 0.001, 0.999)) : 0.0;

    /* ── Animated scan ring at drone XZ position ───────────────────────────── */
    float d2 = distance(vWorldPos.xz, uDroneXZ);

    /* Outer halo — fades in behind the frontier */
    float halo  = smoothstep(4.5, 1.0, d2) * smoothstep(0.0, 0.8, d2);
    float pulse = 0.35 + 0.65 * sin(uScanTime * 2.4 - d2 * 1.6);

    /* Tight sweep ring — orbits outward */
    float sweepR = 1.6 + sin(uScanTime * 2.5) * 0.6;
    float sweep  = smoothstep(0.5, 0.0, abs(d2 - sweepR));

    float scan = halo * pulse * 0.4 + sweep * 0.85;

    /* ── Combine ──────────────────────────────────────────────────────────── */
    float alpha = max(revealed, scan);
    if (alpha < 0.005) discard;

    /* ── Surface colour: UV texture → vertex colour → flat colour ─────────── */
    vec4 col;
    if (uHasMap > 0.5) {
      col = texture2D(tDiffuse, vUv);
    } else if (uHasVtxColor > 0.5) {
      /* gamma-correct vertex colour (PLY stores sRGB bytes ≈ 0–1 linear here) */
      col = vec4(pow(vVertColor, vec3(1.0 / 1.4)), 1.0);
    } else {
      col = vec4(uBaseColor, 1.0);
    }

    /* Reveal frontier: cyan fringe at the 0→1 transition */
    float edge = smoothstep(0.0, 0.3, revealed) * (1.0 - smoothstep(0.7, 1.0, revealed));
    col.rgb = mix(col.rgb, vec3(0.0, 0.85, 1.0), edge * 0.6);

    /* Scan ring tint */
    float scanFrac = scan / (alpha + 0.001);
    col.rgb = mix(col.rgb, vec3(0.05, 0.9, 1.0), clamp(scanFrac * 0.7, 0.0, 0.8));

    /* Subtle height vignette — floor and ceiling slightly darker */
    float hv = smoothstep(0.0, 0.5, 1.0 - abs(vWorldPos.y - 1.2) / 2.0);
    col.rgb *= mix(0.55, 1.0, hv);

    gl_FragColor = vec4(col.rgb, alpha);
  }
`

// ─── Material factory ─────────────────────────────────────────────────────────
function makeRevealMat(src: THREE.Material, hasVtxColor: boolean): THREE.ShaderMaterial {
  const std = src as THREE.MeshStandardMaterial
  const map = std.map ?? null
  const col = std.color ?? new THREE.Color(0.75, 0.75, 0.75)
  return new THREE.ShaderMaterial({
    uniforms: {
      tDiffuse     : { value: map },
      uHasMap      : { value: map ? 1.0 : 0.0 },
      uHasVtxColor : { value: hasVtxColor ? 1.0 : 0.0 },
      uBaseColor   : { value: new THREE.Color(col.r, col.g, col.b) },
      tGrid     : _uni.tGrid,
      uHalfSpan : _uni.uHalfSpan,
      uSpan     : _uni.uSpan,
      uDroneXZ  : _uni.uDroneXZ,
      uScanTime : _uni.uScanTime,
    },
    vertexShader  : VERT,
    fragmentShader: FRAG,
    transparent   : true,
    side          : THREE.DoubleSide,
    depthWrite    : false,
  })
}

// ─── GLB mesh with reveal shader ─────────────────────────────────────────────
function RevealMeshInner({ url }: { url: string }) {
  const meshNormOffset = useDroneStore(s => s.meshNormOffset)
  const { scene: gltfScene } = useGLTF(url)

  const revealScene = useMemo(() => {
    const clone = gltfScene.clone(true)
    clone.traverse(child => {
      if (!(child instanceof THREE.Mesh)) return
      // Detect vertex colour attribute on geometry
      const hasVtxColor = !!(child.geometry as THREE.BufferGeometry)
        .attributes.color
      child.material = Array.isArray(child.material)
        ? child.material.map(m => makeRevealMat(m, hasVtxColor))
        : makeRevealMat(child.material, hasVtxColor)
    })
    return clone
  }, [gltfScene])

  const offset = useMemo<[number, number, number]>(() => {
    if (!meshNormOffset) return [0, 0, 0]
    const [nx, ny, nz] = meshNormOffset
    return [nx, nz, -ny]
  }, [meshNormOffset])

  return <primitive object={revealScene} position={offset} />
}

function RevealMesh() {
  const visualUrl = useDroneStore(s => s.visualMeshUrl)
  // Only render when we have a GLB/GLTF (skip PLY or null)
  if (!visualUrl || (!visualUrl.endsWith('.glb') && !visualUrl.endsWith('.gltf'))) return null
  return <RevealMeshInner url={visualUrl} />
}

// ─── Consume lidar + gallery → reveal grid ───────────────────────────────────
function RevealUpdater() {
  const lidarHits  = useDroneStore(s => s.lidarHits)
  const gallery    = useDroneStore(s => s.cameraGallery)
  const position   = useDroneStore(s => s.position)
  const sceneId    = useDroneStore(s => s.sceneId)

  // Scene change → reset
  useEffect(() => {
    if (sceneId !== _lastSceneId) {
      _lastSceneId = sceneId
      resetGrid()
    }
  }, [sceneId])

  // LIDAR hits — precise surface points, small radius (r=2, ≈5 cm)
  useEffect(() => {
    if (!lidarHits.length) return
    let dirty = false
    for (const [rx, ry] of lidarHits) {
      const cell = toCell(rx, ry)
      if (!cell) continue
      splatCircle(cell[0], cell[1], 2)
      dirty = true
    }
    if (dirty) _gridTex.needsUpdate = true
  }, [lidarHits])

  // Camera gallery — broader drone-position splat (r=20, ≈55 cm)
  // Fills areas the lidar might miss (far walls, ceiling)
  useEffect(() => {
    let dirty = false
    for (const snap of gallery) {
      const [rx, ry] = snap.position
      const cell = toCell(rx, ry)
      if (!cell) continue
      splatCircle(cell[0], cell[1], 20)
      dirty = true
    }
    if (dirty) _gridTex.needsUpdate = true
    // run only when new snapshots arrive (length changes)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [gallery.length])

  // Drone XZ for scan ring
  useEffect(() => {
    _uni.uDroneXZ.value.set(position[0], -position[1])
  }, [position])

  return null
}

// ─── Per-frame animation clock ────────────────────────────────────────────────
function AnimClock() {
  useFrame(({ clock }) => { _uni.uScanTime.value = clock.elapsedTime })
  return null
}

// ─── Floating drone marker ────────────────────────────────────────────────────
function DroneMarker() {
  const pos = useDroneStore(s => s.position)
  const ref = useRef<THREE.Group>(null)

  useFrame(({ clock }) => {
    if (!ref.current) return
    const t = clock.elapsedTime
    ref.current.position.y = pos[2] + 0.25 + 0.07 * Math.sin(t * 4.2)
  })

  // Physics Three.js: x=rx, y=rz (height), z=-ry
  const base: [number, number, number] = [pos[0], pos[2], -pos[1]]

  return (
    <group ref={ref} position={base}>
      <mesh>
        <sphereGeometry args={[0.11, 14, 14]} />
        <meshBasicMaterial color="#22d3ee" />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.19, 0.31, 28]} />
        <meshBasicMaterial color="#22d3ee" transparent opacity={0.38} side={THREE.DoubleSide} />
      </mesh>
      {/* Down-beam */}
      <mesh position={[0, -0.22, 0]}>
        <cylinderGeometry args={[0.008, 0.035, 0.38, 6]} />
        <meshBasicMaterial color="#22d3ee" transparent opacity={0.45} />
      </mesh>
    </group>
  )
}

// ─── Scene floor-plan grid ────────────────────────────────────────────────────
function FloorGrid() {
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.05, 0]}>
      <planeGeometry args={[24, 24, 24, 24]} />
      <meshBasicMaterial color="#091520" wireframe transparent opacity={0.3} />
    </mesh>
  )
}

// ─── HUD stats ────────────────────────────────────────────────────────────────
function StatsHud() {
  const gallery      = useDroneStore(s => s.cameraGallery)
  const coverage     = useDroneStore(s => s.coverage)
  const knownPct     = useDroneStore(s => s.discoveryKnownPct)
  const lidarHits    = useDroneStore(s => s.lidarHits)
  const missionState = useDroneStore(s => s.missionState)
  const elapsed      = useDroneStore(s => s.elapsedTime)
  const totalPts     = useDroneStore(s => s.totalPoints)

  const isActive  = missionState === 'EXPLORING' || missionState === 'RETURNING'
  const isLanding = missionState === 'LANDING'
  const fmt = (s: number) =>
    `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`

  return (
    <div className="absolute inset-0 pointer-events-none flex flex-col justify-between p-3">
      <div className="flex items-center gap-2 flex-wrap">
        <span className="bg-cyan-500/15 border border-cyan-500/35 text-cyan-300 text-[10px] font-mono px-2 py-0.5 rounded uppercase tracking-wider">
          3D Reconstruction
        </span>
        <span className={[
          'text-[10px] font-mono px-2 py-0.5 rounded border',
          isActive  ? 'bg-emerald-500/15 border-emerald-500/35 text-emerald-300'
          : isLanding ? 'bg-amber-500/15  border-amber-500/35  text-amber-300'
          :             'bg-slate-700/40  border-slate-600/40  text-slate-400',
        ].join(' ')}>
          {missionState || 'IDLE'}
        </span>
        {isActive && <span className="text-[10px] font-mono text-slate-400">{fmt(elapsed)}</span>}
      </div>

      <div className="flex gap-2 flex-wrap">
        <Chip label="Map pts"  value={totalPts.toLocaleString()}       color="cyan"    />
        <Chip label="Lidar"    value={lidarHits.length.toString()}      color="sky"     />
        <Chip label="Frames"   value={gallery.length.toLocaleString()}  color="cyan"    />
        <Chip label="Scanned"  value={`${knownPct.toFixed(1)}%`}       color="emerald" />
        <Chip label="Coverage" value={`${coverage.toFixed(1)}%`}       color="amber"   />
      </div>
    </div>
  )
}

type CC = 'cyan' | 'sky' | 'emerald' | 'amber'
function Chip({ label, value, color }: { label: string; value: string; color: CC }) {
  const cls: Record<CC, string> = {
    cyan   : 'border-cyan-500/40    text-cyan-300    bg-cyan-500/10',
    sky    : 'border-sky-500/40     text-sky-300     bg-sky-500/10',
    emerald: 'border-emerald-500/40 text-emerald-300 bg-emerald-500/10',
    amber  : 'border-amber-500/40   text-amber-300   bg-amber-500/10',
  }
  return (
    <div className={`border rounded px-2 py-1 text-center min-w-[60px] ${cls[color]}`}>
      <div className="text-[8px] font-mono opacity-60 uppercase tracking-wider leading-none mb-0.5">{label}</div>
      <div className="text-[11px] font-mono font-semibold leading-none">{value}</div>
    </div>
  )
}

// ─── Root export ──────────────────────────────────────────────────────────────
function exportIcon(url: string) {
  if (url.endsWith('.svg')) return <MapIcon className="w-3.5 h-3.5" />
  if (url.endsWith('.glb')) return <Boxes className="w-3.5 h-3.5" />
  return <FileBox className="w-3.5 h-3.5" />
}

function exportLabel(url: string) {
  if (url.endsWith('.svg')) return 'Floor plan (SVG)'
  if (url.endsWith('.glb')) return 'Mesh (GLB)'
  return 'Point cloud (PLY)'
}

/** Scan deliverables: PLY cloud, GLB mesh, SVG floor plan (Polycam/DJI-Terra-
 *  style exports).  Generated by the bridge off the physics thread. */
function ExportControls() {
  const exportState = useDroneStore((s) => s.exportState)
  const simConnected = useDroneStore((s) => s.simConnected)
  const totalPoints = useDroneStore((s) => s.totalPoints)
  const running = exportState.status === 'running'
  const canExport = simConnected && totalPoints > 100 && !running

  return (
    <div className="absolute top-3 right-3 z-10 flex flex-col items-end gap-1.5">
      <button
        type="button"
        onClick={() => canExport && simBridgeSend({ op: 'export_scan' })}
        disabled={!canExport}
        className="flex items-center gap-1.5 px-3 h-8 rounded-lg text-xs font-semibold
                   bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-300 transition-colors
                   disabled:opacity-40 disabled:cursor-not-allowed"
        title="Export scan deliverables: point cloud (PLY), mesh (GLB), floor plan (SVG)"
      >
        {running ? <Loader2 className="w-4 h-4 animate-spin" /> : <Download className="w-4 h-4" />}
        {running ? 'Exporting…' : 'Export scan'}
      </button>
      {exportState.status === 'done' && exportState.urls.map((url) => (
        <a
          key={url}
          href={url}
          download
          className="flex items-center gap-1.5 px-2.5 h-7 rounded-md text-[11px] font-mono
                     bg-slate-800/80 hover:bg-slate-700/80 text-slate-200 transition-colors"
        >
          {exportIcon(url)}
          {exportLabel(url)}
        </a>
      ))}
      {exportState.status === 'done' && exportState.errors.map((e) => (
        <div key={e} className="px-2.5 py-1 rounded-md text-[11px] font-mono bg-red-900/60 text-red-300 max-w-64">
          {e}
        </div>
      ))}
    </div>
  )
}

export function ScanReconstruction() {
  const gallery    = useDroneStore(s => s.cameraGallery)
  const sceneAabb  = useDroneStore(s => s.sceneAabb)

  const camPos = useMemo<[number, number, number]>(() => {
    if (!sceneAabb) return [9, 6, 9]
    const span = Math.max(
      sceneAabb.max[0] - sceneAabb.min[0],
      sceneAabb.max[1] - sceneAabb.min[1],
    )
    const d = span * 0.88
    return [d * 0.6, d * 0.5, d * 0.9]
  }, [sceneAabb])

  // Scene center for orbit target (physics Three.js)
  const orbitTarget = useMemo<[number, number, number]>(() => {
    if (!sceneAabb) return [0, 1.2, 0]
    const cx = (sceneAabb.min[0] + sceneAabb.max[0]) * 0.5
    const cy = (sceneAabb.min[2] + sceneAabb.max[2]) * 0.5   // Three.js Y = ROS Z
    const cz = -(sceneAabb.min[1] + sceneAabb.max[1]) * 0.5  // Three.js Z = -ROS Y
    return [cx, cy, cz]
  }, [sceneAabb])

  return (
    <div className="glass-card flex-1 flex flex-col min-h-0 min-w-0 overflow-hidden relative">
      <Canvas
        style={{ background: '#03080e' }}
        camera={{ position: camPos, fov: 46, near: 0.05, far: 120 }}
        gl={{ antialias: true, alpha: false }}
      >
        <color attach="background" args={['#03080e']} />

        {/* Lighting for depth perception */}
        <hemisphereLight args={['#1e4a72', '#0a180f', 0.65]} position={[0, 10, 0]} />
        <directionalLight position={[4, 10, 5]} intensity={1.0} color="#b8d8f0" />
        <directionalLight position={[-3, 4, -6]} intensity={0.25} color="#304860" />
        <ambientLight intensity={0.12} />

        <FloorGrid />
        <RevealMesh />
        <RevealUpdater />
        <AnimClock />
        {gallery.length > 0 && <DroneMarker />}

        <OrbitControls
          autoRotate
          autoRotateSpeed={0.8}
          enableDamping
          dampingFactor={0.06}
          minDistance={1.5}
          maxDistance={60}
          maxPolarAngle={Math.PI * 0.82}
          target={orbitTarget}
        />
      </Canvas>

      <StatsHud />
      <ExportControls />

      {gallery.length === 0 && (
        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
          <div className="text-center px-6">
            <div className="text-5xl mb-3 opacity-15">⬡</div>
            <div className="text-slate-300 text-sm font-mono mb-1">No scan data yet</div>
            <div className="text-slate-500 text-xs font-mono leading-relaxed">
              Start a mission — the apartment structure will<br />
              be revealed surface-by-surface as the LIDAR scans it.
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
