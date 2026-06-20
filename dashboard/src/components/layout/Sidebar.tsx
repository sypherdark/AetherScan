'use client'

import { Map, Navigation, ScanLine, Settings, BarChart3, Gamepad2 } from 'lucide-react'
import { clsx } from 'clsx'
import { useDroneStore, type ActivePanel } from '@/stores/drone-store'

const navItems: { icon: typeof Map; label: string; id: ActivePanel }[] = [
  { icon: Map, label: 'Map View', id: 'map' },
  { icon: Navigation, label: 'Navigation', id: 'nav' },
  { icon: ScanLine, label: '3D Scan', id: 'camera' },
  { icon: BarChart3, label: 'Metrics', id: 'metrics' },
  { icon: Gamepad2, label: 'Teleop', id: 'teleop' },
  { icon: Settings, label: 'Settings', id: 'settings' },
]

export function Sidebar() {
  const activePanel = useDroneStore((s) => s.activePanel)
  const setActivePanel = useDroneStore((s) => s.setActivePanel)

  return (
    <aside className="w-16 shrink-0 flex flex-col items-center py-4 gap-2 border-r border-slate-700/50 bg-slate-900/60">
      {navItems.map(({ icon: Icon, label, id }) => (
        <button
          key={id}
          type="button"
          onClick={() => setActivePanel(id)}
          title={label}
          aria-label={label}
          aria-current={activePanel === id ? 'page' : undefined}
          className={clsx(
            'w-10 h-10 rounded-lg flex items-center justify-center transition-all duration-200',
            activePanel === id
              ? 'bg-cyan-500/20 text-cyan-400 shadow-lg shadow-cyan-500/10 ring-1 ring-cyan-500/40'
              : 'text-slate-500 hover:text-slate-300 hover:bg-slate-800'
          )}
        >
          <Icon className="w-5 h-5" />
        </button>
      ))}
    </aside>
  )
}
