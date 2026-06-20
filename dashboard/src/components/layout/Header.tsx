'use client'

import { Activity, Radio, WifiOff } from 'lucide-react'
import type { LinkStatus } from '@/stores/drone-store'

interface HeaderProps {
  rosConnected: boolean
  simConnected?: boolean
  linkStatus: LinkStatus
}

export function Header({ rosConnected, simConnected, linkStatus }: HeaderProps) {
  const disconnected = linkStatus === 'DISCONNECTED'

  return (
    <header className="h-14 shrink-0 flex items-center justify-between px-6 border-b border-slate-700/50 bg-slate-900/80 backdrop-blur-md">
      <div className="flex items-center gap-3">
        <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-cyan-400 to-blue-500 flex items-center justify-center">
          <Activity className="w-5 h-5 text-white" />
        </div>
        <h1 className="text-lg font-semibold tracking-tight">
          <span className="text-cyan-400">Aether</span>
          <span className="text-white">Scan</span>
        </h1>
        <span className="text-xs text-slate-500 ml-2">v1.0</span>
      </div>

      <div className="flex items-center gap-4 text-sm">
        {disconnected && (
          <div className="flex items-center gap-2 px-2 py-1 rounded bg-red-950/60 border border-red-800/50">
            <WifiOff className="w-4 h-4 text-red-400" />
            <span className="text-red-400 font-medium text-xs tracking-wide">DISCONNECTED</span>
          </div>
        )}
        {simConnected && (
          <div className="flex items-center gap-2">
            <Radio className="w-4 h-4 text-cyan-400" />
            <span className="text-cyan-400">Physics sim</span>
          </div>
        )}
        <div className="flex items-center gap-2">
          <Radio className={`w-4 h-4 ${rosConnected ? 'text-green-400' : 'text-slate-500'}`} />
          <span className={rosConnected ? 'text-green-400' : 'text-slate-500'}>
            {rosConnected ? 'ROS Connected' : 'ROS Offline'}
          </span>
        </div>
      </div>
    </header>
  )
}
