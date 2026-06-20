export interface DronePosition {
  x: number
  y: number
  z: number
}

export interface MissionStatus {
  state: string
  state_id: number
  elapsed_time_sec: number
  coverage_percent: number
  distance_traveled_m: number
  total_points: number
  position: [number, number, number] | null
}

export interface MapStats {
  total_points: number
  area_m2: number
  volume_m3: number
  bounds_min: [number, number, number]
  bounds_max: [number, number, number]
  timestamp: string
}

export interface SensorStatus {
  imu: boolean
  camera: boolean
  lidar: boolean
  rangefinder: boolean
}
