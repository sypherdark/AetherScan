'use client'

import { useCallback } from 'react'
import { useRosBridge } from '@/hooks/useRosBridge'
import { useDroneStore } from '@/stores/drone-store'
import { simBridgeSend } from '@/lib/sim-bridge-client'

/** Mission controls only when ROS or physics bridge is connected — no offline fallback. */
export function useMissionActions() {
  const { callService } = useRosBridge()
  const simConnected = useDroneStore((s) => s.simConnected)
  const rosConnected = useDroneStore((s) => s.rosConnected)
  const linkStatus = useDroneStore((s) => s.linkStatus)

  const canCommand = linkStatus === 'SIM' || linkStatus === 'ROS'

  const start = useCallback(() => {
    if (!canCommand) return
    if (rosConnected) {
      callService('/aetherscan/start_mission', 'std_srvs/srv/Trigger', {})
      return
    }
    if (simConnected) {
      simBridgeSend({ op: 'mission', command: 'start' })
    }
  }, [canCommand, rosConnected, simConnected, callService])

  const pause = useCallback(() => {
    if (!canCommand) return
    if (rosConnected) {
      callService('/aetherscan/pause_mission', 'std_srvs/srv/Trigger', {})
      return
    }
    if (simConnected) {
      simBridgeSend({ op: 'mission', command: 'pause' })
    }
  }, [canCommand, rosConnected, simConnected, callService])

  const resume = useCallback(() => {
    if (!canCommand) return
    if (rosConnected) {
      callService('/aetherscan/resume_mission', 'std_srvs/srv/Trigger', {})
      return
    }
    if (simConnected) {
      simBridgeSend({ op: 'mission', command: 'resume' })
    }
  }, [canCommand, rosConnected, simConnected, callService])

  const abort = useCallback(() => {
    if (!canCommand) return
    if (rosConnected) {
      callService('/aetherscan/abort_mission', 'std_srvs/srv/Trigger', {})
      return
    }
    if (simConnected) {
      simBridgeSend({ op: 'mission', command: 'abort' })
    }
  }, [canCommand, rosConnected, simConnected, callService])

  /** Land: drone descends to floor and stops scanning. Bound to Return key. */
  const land = useCallback(() => {
    if (!canCommand) return
    if (rosConnected) {
      callService('/aetherscan/abort_mission', 'std_srvs/srv/Trigger', {})
      return
    }
    if (simConnected) {
      simBridgeSend({ op: 'mission', command: 'land' })
    }
  }, [canCommand, rosConnected, simConnected, callService])

  return { start, pause, resume, abort, land, simConnected, rosConnected, canCommand }
}
