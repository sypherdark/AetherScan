'use client'

import { useDroneStore } from '@/stores/drone-store'

export function NavigationPanel() {
  const plannedPath = useDroneStore((s) => s.plannedPath)
  const coverage = useDroneStore((s) => s.coverage)
  const missionState = useDroneStore((s) => s.missionState)
  const simConnected = useDroneStore((s) => s.simConnected)
  const sensorMin = useDroneStore((s) => s.sensorMinRange)
  const sensorFront = useDroneStore((s) => s.sensorFrontRange)
  const sensorProx = useDroneStore((s) => s.sensorProximity)
  const sensorOpen = useDroneStore((s) => s.sensorOpenDeg)
  const sensorWalls = useDroneStore((s) => s.sensorWallHits)
  const navMode = useDroneStore((s) => s.navigationMode)
  const knownPct = useDroneStore((s) => s.discoveryKnownPct)
  const wallElts = useDroneStore((s) => s.spaceWallElements)
  const objElts = useDroneStore((s) => s.spaceObjectElements)
  const structures = useDroneStore((s) => s.nearbyStructures)

  const rangeColor = (m: number) =>
    m < 0.5 ? 'text-red-400' : m < 1.2 ? 'text-amber-400' : 'text-green-400'

  return (
    <div className="glass-card flex-1 p-4 overflow-auto">
      <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">
        Semantic navigation
      </h3>
      <dl className="space-y-2 text-sm">
        <div className="flex justify-between">
          <dt className="text-slate-500">Mission</dt>
          <dd className="text-cyan-400 font-medium">{missionState}</dd>
        </div>
        <div className="flex justify-between">
          <dt className="text-slate-500">Mode</dt>
          <dd className="text-slate-200 text-xs">{navMode}</dd>
        </div>

        {simConnected && (
          <>
            <div className="pt-2 border-t border-slate-700/50">
              <p className="text-[10px] text-slate-500 uppercase tracking-wider mb-2">
                3D space model (mesh analysis)
              </p>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Wall structures</dt>
              <dd className="tabular-nums">{wallElts}</dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Object clusters</dt>
              <dd className="tabular-nums">{objElts}</dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Discovered map</dt>
              <dd className="tabular-nums text-cyan-400">{knownPct.toFixed(1)}%</dd>
            </div>

            <div className="pt-2 border-t border-slate-700/50">
              <p className="text-[10px] text-slate-500 uppercase tracking-wider mb-2">
                Live sensors (this frame)
              </p>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Nearest obstacle</dt>
              <dd className={`tabular-nums font-medium ${rangeColor(sensorMin)}`}>
                {sensorMin.toFixed(2)} m
              </dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Front / proximity</dt>
              <dd className={`tabular-nums ${rangeColor(Math.min(sensorFront, sensorProx))}`}>
                {sensorFront.toFixed(2)} / {sensorProx.toFixed(2)} m
              </dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Open gap</dt>
              <dd className="tabular-nums">{sensorOpen.toFixed(0)}°</dd>
            </div>
            <div className="flex justify-between">
              <dt className="text-slate-500">Walls in scan</dt>
              <dd className="tabular-nums">{sensorWalls}</dd>
            </div>

            {structures.length > 0 && (
              <div className="pt-2">
                <p className="text-[10px] text-slate-500 mb-1">Detected nearby</p>
                <ul className="text-xs space-y-0.5 max-h-24 overflow-auto">
                  {structures.slice(0, 8).map((s) => (
                    <li key={`${s.kind}-${s.id}`} className="flex justify-between text-slate-300">
                      <span>
                        {s.kind} #{s.id}
                      </span>
                      <span className="tabular-nums text-slate-500">{s.range_m.toFixed(2)} m</span>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </>
        )}

        <div className="flex justify-between pt-2 border-t border-slate-700/50">
          <dt className="text-slate-500">Coverage</dt>
          <dd className="tabular-nums">{coverage.toFixed(1)}%</dd>
        </div>
        <div className="flex justify-between">
          <dt className="text-slate-500">Intent path</dt>
          <dd className="tabular-nums">{plannedPath.length} pts</dd>
        </div>
      </dl>
      <p className="text-xs text-slate-500 mt-4">
        The drone raycasts the mesh, labels walls/objects, builds a discovery map, and
        moves toward unknown frontiers while avoiding measured obstacles.
      </p>
    </div>
  )
}
