'use client'

import { Suspense, useEffect } from 'react'
import dynamic from 'next/dynamic'
import { Sidebar } from '@/components/layout/Sidebar'
import { Header } from '@/components/layout/Header'
import { MissionControl } from '@/components/panels/MissionControl'
import { MetricsPanel } from '@/components/panels/MetricsPanel'
import { SystemStatus } from '@/components/panels/SystemStatus'
import { ScanReconstruction } from '@/components/panels/ScanReconstruction'
import { TeleopPanel } from '@/components/panels/TeleopPanel'
import { SettingsPanel } from '@/components/panels/SettingsPanel'
import { NavigationPanel } from '@/components/panels/NavigationPanel'
import { useRosBridge } from '@/hooks/useRosBridge'
import { useSimBridge } from '@/hooks/useSimBridge'
import { useSceneSync } from '@/hooks/useSceneSync'
import { useMissionActions } from '@/hooks/useMissionActions'
import { useDroneStore } from '@/stores/drone-store'

const SceneViewer = dynamic(() => import('@/components/three/SceneViewer'), {
  ssr: false,
  loading: () => (
    <div className="w-full h-full flex items-center justify-center bg-slate-900">
      <div className="text-cyan-400 animate-pulse">Loading 3D Viewer...</div>
    </div>
  ),
})

function MainViewport() {
  const activePanel = useDroneStore((s) => s.activePanel)

  if (activePanel === 'camera') {
    // "3D Scan" panel — progressive point-cloud reconstruction that builds as
    // the drone scans, revealing the structure bit by bit on a dark background.
    return (
      <div className="flex-1 flex min-h-0">
        <ScanReconstruction />
      </div>
    )
  }
  if (activePanel === 'teleop') {
    return <TeleopPanel />
  }
  if (activePanel === 'nav') {
    return (
      <div className="flex-1 flex gap-3 min-h-0">
        <div className="flex-[2] glass-card overflow-hidden min-h-0">
          <Suspense fallback={null}>
            <SceneViewer />
          </Suspense>
        </div>
        <div className="flex-1 min-w-[240px]">
          <NavigationPanel />
        </div>
      </div>
    )
  }
  if (activePanel === 'settings') {
    return <SettingsPanel />
  }
  if (activePanel === 'metrics') {
    return (
      <div className="flex-1 flex gap-3 min-h-0">
        <div className="flex-[2] glass-card overflow-hidden">
          <Suspense fallback={null}>
            <SceneViewer />
          </Suspense>
        </div>
        <div className="flex-1 flex flex-col gap-3 min-w-[280px]">
          <MetricsPanel />
          <SystemStatus />
        </div>
      </div>
    )
  }

  return (
    <div className="flex-1 flex gap-3 min-h-0">
      <div className="flex-[3] glass-card overflow-hidden">
        <Suspense fallback={null}>
          <SceneViewer />
        </Suspense>
      </div>
      <div className="flex-1 flex flex-col gap-3 min-w-[280px] max-w-[320px]">
        <SystemStatus />
        <MetricsPanel />
      </div>
    </div>
  )
}

export default function DashboardPage() {
  const { connected: rosConnected } = useRosBridge()
  const { connected: simConnected } = useSimBridge()
  useSceneSync()
  const linkStatus = useDroneStore((s) => s.linkStatus)
  const { land } = useMissionActions()

  // Return/Enter key → land the drone immediately (fires from any focus)
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Enter' && !e.repeat) land()
    }
    // Capture phase so the canvas never swallows it
    window.addEventListener('keydown', onKey, true)
    return () => window.removeEventListener('keydown', onKey, true)
  }, [land])

  return (
    <div className="h-screen w-screen flex flex-col overflow-hidden">
      <Header rosConnected={rosConnected} simConnected={simConnected} linkStatus={linkStatus} />

      <div className="flex flex-1 overflow-hidden">
        <Sidebar />

        <main className="flex-1 flex flex-col p-3 gap-3 overflow-hidden">
          <MainViewport />

          <div className="h-[140px] shrink-0">
            <MissionControl />
          </div>
        </main>
      </div>
    </div>
  )
}
