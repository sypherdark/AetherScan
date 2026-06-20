'use client'

import { Suspense } from 'react'
import { Text } from '@react-three/drei'
import { useDroneStore } from '@/stores/drone-store'
import { getScene, mergeSceneWithAabb } from '@/lib/scenes'
import { NoMeshLoaded } from './OfficeEnvironment'
import { MeshEnvironment } from './MeshEnvironment'
import { MeshErrorBoundary } from './MeshErrorBoundary'

/** In-scene placeholder shown when no mesh is available or a mesh fails to load. */
function MeshPlaceholder({
  sceneId,
  offset,
  reason,
}: {
  sceneId: string
  offset: [number, number, number]
  reason: string
}) {
  return (
    <group position={offset}>
      <NoMeshLoaded sceneId={sceneId} />
      <Text position={[0, 2.2, 0]} fontSize={0.28} color="#fca5a5" anchorX="center" anchorY="middle" maxWidth={8}>
        {reason}
      </Text>
      <Text position={[0, 1.6, 0]} fontSize={0.16} color="#94a3b8" anchorX="center" anchorY="middle" maxWidth={10}>
        Place a GLB/PLY in dashboard/public/meshes/ or start the physics bridge
      </Text>
    </group>
  )
}

export function ScanEnvironment() {
  const sceneId = useDroneStore((s) => s.sceneId)
  const sceneAabb = useDroneStore((s) => s.sceneAabb)
  const visualMeshUrl = useDroneStore((s) => s.visualMeshUrl)
  const config = mergeSceneWithAabb(getScene(sceneId), sceneAabb)
  // Priority: the SELECTED scene's own GLB first.  sceneId is kept in lock-step
  // with the bridge (useSimBridge.syncSceneFromBridge), so config.visualUrl always
  // matches the active scene and the displayed structure can never lag behind a
  // scene switch (the old "bridge URL first" order showed the previous mesh when
  // the bridge's visual_mesh_url arrived late or stale).  Bridge/PLY are fallbacks.
  const displayUrl = config.visualUrl ?? visualMeshUrl ?? config.meshUrl

  if (!displayUrl) {
    return <MeshPlaceholder sceneId={sceneId} offset={config.meshOffset} reason={`No mesh loaded for "${sceneId}"`} />
  }

  return (
    // Boundary keeps a missing/corrupt asset from white-screening the whole app:
    // useGLTF throws on a 404 (Next serves an HTML page the GLB parser rejects),
    // and <Suspense> only catches loading promises, not thrown errors.
    // resetKey=displayUrl lets a subsequent valid scene retry after a failure.
    <MeshErrorBoundary
      resetKey={displayUrl}
      fallback={
        <MeshPlaceholder sceneId={sceneId} offset={config.meshOffset} reason={`Mesh unavailable for "${sceneId}"`} />
      }
    >
      <Suspense
        fallback={
          <mesh position={config.meshOffset}>
            <boxGeometry args={[0.5, 0.5, 0.5]} />
            <meshBasicMaterial color="#06b6d4" wireframe />
          </mesh>
        }
      >
        <MeshEnvironment url={displayUrl} offset={config.meshOffset} />
      </Suspense>
    </MeshErrorBoundary>
  )
}
