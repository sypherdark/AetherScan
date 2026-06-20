'use client'

import { Play, Pause, Square, RotateCcw, Zap } from 'lucide-react'
import { useDroneStore } from '@/stores/drone-store'
import { useMissionActions } from '@/hooks/useMissionActions'
import { simBridgeSend } from '@/lib/sim-bridge-client'

export function MissionControl() {
  const missionState = useDroneStore((s) => s.missionState)
  const elapsedTime = useDroneStore((s) => s.elapsedTime)
  const coverage = useDroneStore((s) => s.coverage)
  const godMode = useDroneStore((s) => s.godMode)
  const { start, pause, resume, abort, canCommand } = useMissionActions()
  const btnDisabled = !canCommand ? 'opacity-40 pointer-events-none' : ''

  const toggleGodMode = () => {
    if (!canCommand) return
    simBridgeSend({ op: 'set_god_mode', enabled: !godMode })
  }

  const formatTime = (seconds: number) => {
    const m = Math.floor(seconds / 60)
    const s = Math.floor(seconds % 60)
    return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`
  }

  const stateColors: Record<string, string> = {
    IDLE: 'bg-slate-500',
    PREFLIGHT_CHECK: 'bg-yellow-500',
    TAKEOFF: 'bg-blue-500',
    EXPLORING: 'bg-cyan-500',
    SCANNING: 'bg-purple-500',
    RETURNING: 'bg-orange-500',
    LANDING: 'bg-yellow-500',
    COMPLETE: 'bg-green-500',
    PAUSED: 'bg-amber-500',
    ERROR: 'bg-red-500',
  }

  return (
    <div className="glass-card h-full p-4 flex items-center gap-6">
      {/* State Badge */}
      <div className="flex items-center gap-3">
        <div className={`w-3 h-3 rounded-full ${stateColors[missionState] || 'bg-slate-500'} animate-pulse-slow`} />
        <div>
          <div className="text-xs text-slate-400 uppercase tracking-wider">Mission</div>
          <div className="text-sm font-semibold">{missionState}</div>
        </div>
      </div>

      {/* Divider */}
      <div className="w-px h-12 bg-slate-700" />

      {/* Controls */}
      <div className={`flex items-center gap-2 ${btnDisabled}`}>
        <button
          type="button"
          onClick={start}
          disabled={!canCommand}
          className="w-10 h-10 rounded-lg bg-green-500/20 hover:bg-green-500/30 text-green-400 flex items-center justify-center transition-colors disabled:cursor-not-allowed"
          title="Start Mission"
        >
          <Play className="w-5 h-5" />
        </button>
        <button
          type="button"
          onClick={pause}
          className="w-10 h-10 rounded-lg bg-amber-500/20 hover:bg-amber-500/30 text-amber-400 flex items-center justify-center transition-colors"
          title="Pause"
        >
          <Pause className="w-5 h-5" />
        </button>
        <button
          type="button"
          onClick={abort}
          className="w-10 h-10 rounded-lg bg-red-500/20 hover:bg-red-500/30 text-red-400 flex items-center justify-center transition-colors"
          title="Abort"
        >
          <Square className="w-5 h-5" />
        </button>
        <button
          type="button"
          onClick={resume}
          className="w-10 h-10 rounded-lg bg-blue-500/20 hover:bg-blue-500/30 text-blue-400 flex items-center justify-center transition-colors"
          title="Resume"
        >
          <RotateCcw className="w-5 h-5" />
        </button>
        <button
          type="button"
          onClick={toggleGodMode}
          aria-pressed={godMode}
          className={`h-10 px-3 rounded-lg flex items-center gap-1.5 transition-colors ${
            godMode
              ? 'bg-fuchsia-500/30 text-fuchsia-300 ring-1 ring-fuchsia-400/60'
              : 'bg-slate-500/20 hover:bg-fuchsia-500/20 text-slate-400 hover:text-fuchsia-300'
          }`}
          title="God Mode — accelerated survey (faster cruise + time acceleration)"
        >
          <Zap className="w-4 h-4" />
          <span className="text-xs font-semibold tracking-wide">GOD</span>
        </button>
      </div>

      {/* Divider */}
      <div className="w-px h-12 bg-slate-700" />

      {/* Progress */}
      <div className="flex-1">
        <div className="flex justify-between text-xs text-slate-400 mb-1">
          <span>Coverage</span>
          <span className="tabular-nums">{coverage.toFixed(1)}%</span>
        </div>
        <div className="h-2 bg-slate-700 rounded-full overflow-hidden">
          <div
            className="h-full bg-gradient-to-r from-cyan-500 to-blue-500 rounded-full transition-all duration-500"
            style={{ width: `${Math.min(100, coverage)}%` }}
          />
        </div>
      </div>

      {/* Divider */}
      <div className="w-px h-12 bg-slate-700" />

      {/* Timer */}
      <div className="text-center">
        <div className="text-xs text-slate-400 uppercase tracking-wider">Time</div>
        <div className="text-xl font-mono text-white tabular-nums">
          {formatTime(elapsedTime)}
        </div>
      </div>
    </div>
  )
}
