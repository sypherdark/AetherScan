'use client'

import { Canvas } from '@react-three/fiber'
import { OrbitControls, PerspectiveCamera, Stats, ContactShadows, BakeShadows } from '@react-three/drei'
import { PointCloudRenderer } from './PointCloudRenderer'
import { DroneModel } from './DroneModel'
import { PathVisualization } from './PathVisualization'
import { LidarScan } from './LidarScan'
import { DiscoveredMap } from './DiscoveredMap'
import { ScanEnvironment } from './ScanEnvironment'
import { useDroneStore } from '@/stores/drone-store'
import { getScene, mergeSceneWithAabb } from '@/lib/scenes'

export default function SceneViewer() {
  const showStats = useDroneStore((s) => s.showStats)
  const sceneId = useDroneStore((s) => s.sceneId)
  const sceneAabb = useDroneStore((s) => s.sceneAabb)
  const scene = mergeSceneWithAabb(getScene(sceneId), sceneAabb)
  const [gx, gy, gz] = scene.gridCenter

  // Camera positioned at ~45° angle, looking into the room from above
  const camX = gx + scene.gridSize * 0.45
  const camY = scene.gridSize * 0.55
  const camZ = gz + scene.gridSize * 0.45

  return (
    <Canvas
      className="w-full h-full"
      gl={{ antialias: true, alpha: false, preserveDrawingBuffer: true, toneMapping: 4 /* ACESFilmicToneMapping */, toneMappingExposure: 1.12 }}
      shadows="soft"
    >
      {/* Obsidian void — near-black background */}
      <color attach="background" args={['#040405']} />

      {/* Cold atmospheric depth fog that fades geometry into the dark */}
      <fog attach="fog" args={['#050507', 14, 48]} />

      <PerspectiveCamera
        makeDefault
        position={[camX, camY, camZ]}
        fov={55}
        near={0.1}
        far={120}
      />
      <OrbitControls
        enableDamping
        dampingFactor={0.08}
        maxPolarAngle={Math.PI * 0.82}
        minDistance={1.5}
        maxDistance={45}
        target={[gx, 0, gz]}
        rotateSpeed={0.6}
        zoomSpeed={0.8}
      />

      {/* Scene geometry + environment lighting — managed inside MeshEnvironment */}
      <ScanEnvironment />

      {/* Contact shadow projected onto the floor plane */}
      <ContactShadows
        position={[gx, -0.02, gz]}
        opacity={0.55}
        scale={scene.gridSize * 1.2}
        blur={2.5}
        far={10}
        color="#000a18"
      />

      {/* Simulation overlays */}
      <PointCloudRenderer />
      <DroneModel />
      <PathVisualization />
      <LidarScan />
      <DiscoveredMap />

      {/* World-space axis indicator (small, bottom-left of scene) */}
      <axesHelper args={[1.5]} position={[gx - scene.gridSize * 0.45, 0, gz - scene.gridSize * 0.45]} />

      {showStats && <Stats />}
    </Canvas>
  )
}
