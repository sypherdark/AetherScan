'use client'

import { Map, Route, Clock, Gauge, Layers } from 'lucide-react'
import { useDroneStore } from '@/stores/drone-store'

export function MetricsPanel() {
  const totalPoints = useDroneStore((s) => s.totalPoints)
  const areaMapped = useDroneStore((s) => s.areaMapped)
  const distance = useDroneStore((s) => s.distanceTraveled)
  const coverage = useDroneStore((s) => s.coverage)

  const metrics = [
    {
      icon: Map,
      label: 'Area Mapped',
      value: `${areaMapped.toFixed(1)}`,
      unit: 'm²',
      color: 'text-cyan-400',
    },
    {
      icon: Layers,
      label: 'Points',
      value: totalPoints > 1000000
        ? `${(totalPoints / 1000000).toFixed(1)}M`
        : totalPoints > 1000
        ? `${(totalPoints / 1000).toFixed(0)}K`
        : `${totalPoints}`,
      unit: '',
      color: 'text-purple-400',
    },
    {
      icon: Route,
      label: 'Distance',
      value: `${distance.toFixed(1)}`,
      unit: 'm',
      color: 'text-green-400',
    },
    {
      icon: Gauge,
      label: 'Coverage',
      value: `${coverage.toFixed(1)}`,
      unit: '%',
      color: 'text-amber-400',
    },
  ]

  return (
    <div className="glass-card flex-1 p-4 overflow-hidden">
      <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">
        Metrics
      </h3>
      <div className="space-y-3">
        {metrics.map(({ icon: Icon, label, value, unit, color }) => (
          <div key={label} className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Icon className={`w-4 h-4 ${color}`} />
              <span className="text-xs text-slate-400">{label}</span>
            </div>
            <div className="text-sm font-semibold tabular-nums">
              <span className="text-white">{value}</span>
              <span className="text-slate-500 ml-0.5 text-xs">{unit}</span>
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}
