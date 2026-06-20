'use client'

import { useEffect, useRef } from 'react'
import { useDroneStore } from '@/stores/drone-store'
import { simBridgeSend } from '@/lib/sim-bridge-client'

/**
 * Tell the physics bridge when the user explicitly picks a new indoor scene.
 *
 * IMPORTANT: we must NOT send set_scene on initial connect — the bridge
 * already has its own scene loaded (potentially apartment_1 or any --scene
 * flag). Sending the store default ('apartment') on connect would silently
 * override whatever scene the bridge was started with.
 */
export function useSceneSync() {
  const sceneId = useDroneStore((s) => s.sceneId)
  const simConnected = useDroneStore((s) => s.simConnected)
  // Track whether we've seen the first connection — skip the initial sync
  const seenFirstConnect = useRef(false)

  useEffect(() => {
    if (!simConnected) {
      // Reset so the next connection also skips the first fire
      seenFirstConnect.current = false
      return
    }
    if (!seenFirstConnect.current) {
      // First time connected — bridge already knows its scene; don't override
      seenFirstConnect.current = true
      return
    }
    // User changed the scene while already connected — propagate to bridge
    simBridgeSend({ op: 'set_scene', scene: sceneId })
  }, [sceneId, simConnected])
}
