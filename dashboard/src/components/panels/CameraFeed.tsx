'use client'

import { useDroneStore } from '@/stores/drone-store'

function snapshotSrc(snap: { image_base64?: string }): string | null {
  const b64 = snap.image_base64?.trim()
  if (!b64) return null
  const head = b64.startsWith('/9j/') ? 'image/jpeg' : 'image/png'
  return `data:${head};base64,${b64}`
}

export function CameraFeed() {
  const position = useDroneStore((s) => s.position)
  const linkStatus = useDroneStore((s) => s.linkStatus)
  const latest = useDroneStore((s) => s.latestCameraSnapshot)
  const gallery = useDroneStore((s) => s.cameraGallery)

  const previewSrc = latest ? snapshotSrc(latest) : null

  return (
    <div className="glass-card flex-1 p-4 flex flex-col min-h-0 min-w-0">
      <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">
        Exploration Camera
      </h3>

      <div className="flex-1 relative rounded-lg overflow-hidden bg-slate-950 border border-slate-700/50 min-h-[140px]">
        {previewSrc ? (
          <img
            src={previewSrc}
            alt={latest?.descriptor ?? 'Drone snapshot'}
            className="absolute inset-0 w-full h-full object-cover"
          />
        ) : (
          <div className="absolute inset-0 flex items-center justify-center bg-slate-900">
            <p className="text-slate-500 text-xs px-4 text-center">
              {linkStatus === 'DISCONNECTED'
                ? 'No feed — connect physics bridge'
                : 'Awaiting first sector snapshot…'}
            </p>
          </div>
        )}
        <div className="absolute inset-0 flex flex-col justify-end p-3 bg-gradient-to-t from-black/70 to-transparent pointer-events-none">
          {latest ? (
            <>
              <p className="text-cyan-300/90 text-xs font-mono truncate">{latest.descriptor}</p>
              <p className="text-slate-400 text-[10px] mt-1">
                #{latest.id} · t={latest.timestamp_s}s · known {latest.known_pct}%
              </p>
              <p className="text-slate-500 text-[10px]">
                @ ({latest.position[0]}, {latest.position[1]}, {latest.position[2]})
              </p>
            </>
          ) : null}
        </div>
        <div className="absolute top-2 left-2 text-[10px] font-mono text-cyan-300/90 bg-black/50 px-2 py-1 rounded pointer-events-none">
          ALT {position[2].toFixed(2)}m
        </div>
      </div>

      {gallery.length > 0 && (
        <div className="mt-3 flex gap-2 overflow-x-auto pb-1 scrollbar-thin">
          {[...gallery].reverse().map((snap) => {
            const src = snapshotSrc(snap)
            return (
              <div
                key={snap.id}
                className="shrink-0 w-20 h-14 rounded border border-slate-600/60 overflow-hidden relative bg-slate-900"
                title={snap.descriptor}
              >
                {src ? (
                  <img src={src} alt="" className="absolute inset-0 w-full h-full object-cover" />
                ) : (
                  <div className="absolute inset-0 flex items-center justify-center text-[8px] text-slate-500">
                    …
                  </div>
                )}
                <div className="absolute bottom-0 inset-x-0 bg-black/60 text-[9px] text-center text-slate-300 py-0.5">
                  #{snap.id}
                </div>
              </div>
            )
          })}
        </div>
      )}
    </div>
  )
}
