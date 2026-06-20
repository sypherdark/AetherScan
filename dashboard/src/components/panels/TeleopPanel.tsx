'use client'

import { useRosBridge } from '@/hooks/useRosBridge'
import { useDroneStore } from '@/stores/drone-store'

export function TeleopPanel() {
  const { callService } = useRosBridge()
  const armed = useDroneStore((s) => s.armed)
  const setMissionState = useDroneStore((s) => s.setMissionState)
  const setArmed = useDroneStore((s) => s.setArmed)

  const arm = () => {
    setArmed(true)
    setMissionState('IDLE')
  }

  const takeoff = () => {
    setArmed(true)
    setMissionState('TAKEOFF')
    setTimeout(() => setMissionState('EXPLORING'), 1500)
  }

  return (
    <div className="glass-card flex-1 p-4 overflow-auto">
      <h3 className="text-xs font-medium text-slate-400 uppercase tracking-wider mb-3">
        Teleoperation
      </h3>
      <p className="text-xs text-slate-500 mb-4">
        Keyboard teleop runs in the ROS terminal:{' '}
        <code className="text-cyan-400">ros2 run aetherscan_teleop keyboard_teleop</code>
      </p>
      <div className="grid grid-cols-3 gap-2 max-w-[200px] mx-auto text-center text-xs">
        <div />
        <kbd className="bg-slate-800 rounded py-2">W</kbd>
        <div />
        <kbd className="bg-slate-800 rounded py-2">A</kbd>
        <kbd className="bg-slate-800 rounded py-2">S</kbd>
        <kbd className="bg-slate-800 rounded py-2">D</kbd>
      </div>
      <div className="mt-4 flex flex-wrap gap-2">
        <button
          type="button"
          onClick={arm}
          className="px-3 py-2 rounded-lg bg-slate-700 hover:bg-slate-600 text-sm"
        >
          Arm
        </button>
        <button
          type="button"
          onClick={takeoff}
          className="px-3 py-2 rounded-lg bg-cyan-600/30 hover:bg-cyan-600/50 text-cyan-300 text-sm"
        >
          Takeoff + Explore
        </button>
        <button
          type="button"
          onClick={() => callService('/aetherscan/abort_mission', 'std_srvs/srv/Trigger', {})}
          className="px-3 py-2 rounded-lg bg-red-600/20 hover:bg-red-600/40 text-red-300 text-sm"
        >
          Land
        </button>
      </div>
      <p className="mt-3 text-xs text-slate-500">
        Status: <span className={armed ? 'text-green-400' : 'text-slate-400'}>{armed ? 'ARMED' : 'DISARMED'}</span>
      </p>
    </div>
  )
}
