'use client'

import { Battery, Wifi, Compass, ArrowUp } from 'lucide-react'
import { useDroneStore } from '@/stores/drone-store'

export function SystemStatus() {
  const position = useDroneStore((s) => s.position)
  const armed = useDroneStore((s) => s.armed)
  const battery = useDroneStore((s) => s.battery)

  const batteryColor = battery > 50 ? 'text-green-400' : battery > 20 ? 'text-amber-400' : 'text-red-400'

  return (
    <div className="glass-card p-4">
      <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">
        System Status
      </h3>

      {/* Armed indicator */}
      <div className="flex items-center justify-between mb-3">
        <span className="text-xs text-slate-400">Status</span>
        <span className={`text-xs font-bold px-2 py-0.5 rounded ${
          armed ? 'bg-green-500/20 text-green-400' : 'bg-slate-600/50 text-slate-400'
        }`}>
          {armed ? 'ARMED' : 'DISARMED'}
        </span>
      </div>

      {/* Battery */}
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <Battery className={`w-4 h-4 ${batteryColor}`} />
          <span className="text-xs text-slate-400">Battery</span>
        </div>
        <span className={`text-sm font-semibold tabular-nums ${batteryColor}`}>
          {battery.toFixed(0)}%
        </span>
      </div>

      {/* Position */}
      <div className="space-y-1.5 pt-2 border-t border-slate-700/50">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Compass className="w-3.5 h-3.5 text-slate-500" />
            <span className="text-xs text-slate-500">Position</span>
          </div>
        </div>
        <div className="grid grid-cols-3 gap-1 text-center">
          {['X', 'Y', 'Z'].map((axis, i) => (
            <div key={axis} className="bg-slate-800/50 rounded px-2 py-1">
              <div className="text-[10px] text-slate-500">{axis}</div>
              <div className="text-xs font-mono tabular-nums text-slate-200">
                {position[i].toFixed(2)}
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Altitude */}
      <div className="flex items-center justify-between mt-3 pt-2 border-t border-slate-700/50">
        <div className="flex items-center gap-2">
          <ArrowUp className="w-4 h-4 text-blue-400" />
          <span className="text-xs text-slate-400">Altitude</span>
        </div>
        <span className="text-sm font-semibold tabular-nums text-blue-400">
          {position[2].toFixed(2)} m
        </span>
      </div>
    </div>
  )
}
