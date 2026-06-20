'use client'

import { useDroneStore } from '@/stores/drone-store'
import { SCENE_LIST, type SceneId } from '@/lib/scenes'
import { useSimBridge } from '@/hooks/useSimBridge'

// Group Replica scenes for the dropdown
const SCENE_GROUPS: { label: string; prefix: string }[] = [
  { label: 'Apartments',     prefix: 'apartment' },
  { label: 'FRL Apartments', prefix: 'frl_apartment' },
  { label: 'Hotel',          prefix: 'hotel' },
  { label: 'Offices',        prefix: 'office' },
  { label: 'Rooms',          prefix: 'room' },
]

export function SettingsPanel() {
  const showStats    = useDroneStore((s) => s.showStats)
  const toggleStats  = useDroneStore((s) => s.toggleStats)
  const linkStatus   = useDroneStore((s) => s.linkStatus)
  const rosConnected = useDroneStore((s) => s.rosConnected)
  const simConnected = useDroneStore((s) => s.simConnected)
  const sceneId      = useDroneStore((s) => s.sceneId)
  const setSceneId   = useDroneStore((s) => s.setSceneId)
  const { setScene } = useSimBridge()

  const handleSceneChange = (newId: SceneId) => {
    setSceneId(newId)          // update local store (visual, bounds, etc.)
    if (simConnected) {
      setScene(newId)           // reload physics sim with new scene
    }
  }

  // Separate Replica scenes from legacy ones
  const legacyScenes  = SCENE_LIST.filter(s =>
    ['apartment', 'office', 'boardroom'].includes(s.id)
  )
  const replicaScenes = SCENE_LIST.filter(s =>
    !['apartment', 'office', 'boardroom'].includes(s.id)
  )

  return (
    <div className="glass-card flex-1 p-4 overflow-auto">
      <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">
        Settings
      </h3>

      {/* ── Scene selector ──────────────────────────────────────────────────── */}
      <div className="py-2 border-b border-slate-700/50">
        <span className="text-sm text-slate-300 block mb-1.5">Indoor environment</span>
        <select
          value={sceneId}
          onChange={(e) => handleSceneChange(e.target.value as SceneId)}
          className="w-full bg-slate-800 border border-slate-600 rounded px-2 py-1.5 text-sm text-slate-100"
        >
          {/* Legacy */}
          <optgroup label="Legacy">
            {legacyScenes.map(s => (
              <option key={s.id} value={s.id}>{s.label}</option>
            ))}
          </optgroup>

          {/* Replica groups */}
          {SCENE_GROUPS.map(grp => {
            const items = replicaScenes.filter(s => s.id.startsWith(grp.prefix))
            if (!items.length) return null
            return (
              <optgroup key={grp.prefix} label={`Replica — ${grp.label}`}>
                {items.map(s => (
                  <option key={s.id} value={s.id}>{s.label}</option>
                ))}
              </optgroup>
            )
          })}
        </select>
        <p className="text-[10px] text-slate-500 mt-1 leading-relaxed">
          Replica scenes load from the dataset on the SanDisk USB.
          {!simConnected && (
            <span className="text-amber-500/80"> Connect the bridge to switch scenes live.</span>
          )}
        </p>
      </div>

      {/* ── Display ─────────────────────────────────────────────────────────── */}
      <label className="flex items-center justify-between py-2 border-b border-slate-700/50">
        <span className="text-sm text-slate-300">Show FPS stats</span>
        <input type="checkbox" checked={showStats} onChange={toggleStats}
               className="accent-cyan-500" />
      </label>

      {/* ── Connection status ───────────────────────────────────────────────── */}
      <div className="py-3 space-y-2 text-sm">
        <p>
          <span className="text-slate-500">ROS bridge: </span>
          <span className={rosConnected ? 'text-green-400' : 'text-amber-400'}>
            {rosConnected ? 'Connected' : 'Offline'}
          </span>
        </p>
        <p>
          <span className="text-slate-500">Physics sim: </span>
          <span className={simConnected ? 'text-cyan-400' : 'text-slate-500'}>
            {simConnected ? 'Connected' : 'Offline'}
          </span>
        </p>
        <p>
          <span className="text-slate-500">Mode: </span>
          <span className="text-slate-200">
            {linkStatus === 'ROS' ? 'Live ROS2'
              : linkStatus === 'SIM' ? 'Integrated physics'
              : 'DISCONNECTED'}
          </span>
        </p>

        <p className="text-[10px] text-slate-500 pt-2 leading-relaxed">
          Rosbridge: {process.env.NEXT_PUBLIC_ROSBRIDGE_URL || 'ws://localhost:9090'}<br />
          Sim bridge: {process.env.NEXT_PUBLIC_SIM_BRIDGE_URL || 'ws://127.0.0.1:8765'}
        </p>

        {/* Current scene info */}
        <div className="mt-2 p-2 rounded bg-slate-800/60 border border-slate-700/40 text-[10px] font-mono text-slate-400">
          <span className="text-slate-500">scene: </span>
          <span className="text-cyan-300">{sceneId}</span>
        </div>
      </div>
    </div>
  )
}
