'use client'

import { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import { useDroneStore } from '@/stores/drone-store'

// Rotor positions in drone body frame (Three.js Y-up, drone is Z-up)
const ROTOR_OFFSETS: [number, number, number][] = [
  [ 0.17,  0.04,  0.17],
  [ 0.17,  0.04, -0.17],
  [-0.17,  0.04,  0.17],
  [-0.17,  0.04, -0.17],
]

// Arm directions matching rotors
const ARM_DIRECTIONS: [number, number, number][] = [
  [ 1, 0,  1],
  [ 1, 0, -1],
  [-1, 0,  1],
  [-1, 0, -1],
]

export function DroneModel() {
  const groupRef = useRef<THREE.Group>(null)
  const rotorRefs = useRef<(THREE.Mesh | null)[]>([null, null, null, null])
  const blurRefs  = useRef<(THREE.Mesh | null)[]>([null, null, null, null])

  const position    = useDroneStore((s) => s.position)
  const orientation = useDroneStore((s) => s.orientation)
  const armed       = useDroneStore((s) => s.armed)
  const velocity    = useDroneStore((s) => s.velocity)

  // Derive rotor RPM from speed proxy (increases with horizontal velocity)
  const speedXY = velocity
    ? Math.sqrt(velocity[0] ** 2 + velocity[1] ** 2)
    : 0

  useFrame((_, delta) => {
    if (!groupRef.current) return

    // Z-up ROS → Y-up Three.js viewport
    const target = new THREE.Vector3(position[0], position[2], -position[1])
    groupRef.current.position.lerp(target, 0.22)   // faster follow for realism

    const [w, x, y, z] = orientation
    const qBody = new THREE.Quaternion(x, z, y, w)
    groupRef.current.quaternion.slerp(qBody, 0.18)

    if (armed) {
      // Base RPM + speed-dependent boost
      const rpm = 28 + speedXY * 4
      rotorRefs.current.forEach((rotor, i) => {
        if (rotor) rotor.rotation.y += delta * rpm * (i % 2 === 0 ? 1 : -1)
      })
      // Blur disc opacity ramps with RPM
      const blurOpacity = Math.min(0.55, 0.2 + (rpm / 60))
      blurRefs.current.forEach((blur) => {
        if (blur) (blur.material as THREE.MeshBasicMaterial).opacity = blurOpacity
      })
    } else {
      blurRefs.current.forEach((blur) => {
        if (blur) (blur.material as THREE.MeshBasicMaterial).opacity = 0
      })
    }
  })

  return (
    <group ref={groupRef}>
      {/* Main body */}
      <mesh castShadow>
        <boxGeometry args={[0.28, 0.065, 0.28]} />
        <meshStandardMaterial color="#1a1a1a" metalness={0.75} roughness={0.25} />
      </mesh>

      {/* Body top plate (slightly lighter, reveals panel seam) */}
      <mesh position={[0, 0.038, 0]} castShadow>
        <boxGeometry args={[0.22, 0.008, 0.22]} />
        <meshStandardMaterial color="#282828" metalness={0.5} roughness={0.4} />
      </mesh>

      {/* LiDAR dome on top */}
      <mesh position={[0, 0.068, 0]} castShadow>
        <cylinderGeometry args={[0.042, 0.042, 0.065, 20]} />
        <meshStandardMaterial color="#0a0a0a" metalness={0.9} roughness={0.15} />
      </mesh>
      {/* LiDAR glass ring */}
      <mesh position={[0, 0.068, 0]}>
        <cylinderGeometry args={[0.044, 0.044, 0.02, 20]} />
        <meshStandardMaterial
          color="#00d4ff"
          emissive="#00aaff"
          emissiveIntensity={armed ? 1.4 : 0.2}
          transparent
          opacity={0.6}
          metalness={0}
          roughness={0}
        />
      </mesh>

      {/* Camera gimbal housing (front) */}
      <mesh position={[0.14, -0.02, 0]} rotation={[0.3, 0, 0]} castShadow>
        <sphereGeometry args={[0.022, 8, 6]} />
        <meshStandardMaterial color="#111" metalness={0.6} roughness={0.4} />
      </mesh>
      {/* Camera lens */}
      <mesh position={[0.162, -0.022, 0]} rotation={[0, Math.PI / 2, 0]}>
        <cylinderGeometry args={[0.01, 0.01, 0.004, 12]} />
        <meshStandardMaterial color="#050505" metalness={0.9} roughness={0.05} />
      </mesh>

      {/* Arms (X-shape) */}
      {ARM_DIRECTIONS.map((dir, i) => {
        const len = 0.22
        const cx = dir[0] * 0.09
        const cz = dir[2] * 0.09
        const angle = Math.atan2(dir[2], dir[0])
        return (
          <mesh
            key={i}
            position={[cx, 0, cz]}
            rotation={[0, -angle, 0]}
            castShadow
          >
            <boxGeometry args={[len, 0.018, 0.022]} />
            <meshStandardMaterial color="#1c1c1c" metalness={0.6} roughness={0.3} />
          </mesh>
        )
      })}

      {/* Motor bell housings */}
      {ROTOR_OFFSETS.map((pos, i) => (
        <mesh key={i} position={pos} castShadow>
          <cylinderGeometry args={[0.026, 0.022, 0.028, 12]} />
          <meshStandardMaterial color="#2a2a2a" metalness={0.8} roughness={0.2} />
        </mesh>
      ))}

      {/* Rotor blades */}
      {ROTOR_OFFSETS.map((pos, i) => (
        <mesh
          key={i}
          position={[pos[0], pos[1] + 0.016, pos[2]]}
          ref={(el) => { rotorRefs.current[i] = el }}
          castShadow
        >
          <cylinderGeometry args={[0.072, 0.008, 0.004, 2]} />
          <meshStandardMaterial
            color="#2c4a7c"
            metalness={0.3}
            roughness={0.5}
          />
        </mesh>
      ))}

      {/* Rotor blur discs (visible only when spinning) */}
      {ROTOR_OFFSETS.map((pos, i) => (
        <mesh
          key={i}
          position={[pos[0], pos[1] + 0.018, pos[2]]}
          ref={(el) => { blurRefs.current[i] = el }}
          rotation={[-Math.PI / 2, 0, 0]}
        >
          <circleGeometry args={[0.078, 32]} />
          <meshBasicMaterial
            color="#4080cc"
            transparent
            opacity={0}
            depthWrite={false}
            side={THREE.DoubleSide}
          />
        </mesh>
      ))}

      {/* Navigation LEDs */}
      {/* Front — green */}
      <pointLight position={[0.16, -0.01, 0]} intensity={armed ? 0.4 : 0} distance={0.5} color="#00ff88" />
      <mesh position={[0.155, -0.018, 0]}>
        <sphereGeometry args={[0.006, 6, 6]} />
        <meshBasicMaterial color={armed ? '#00ff88' : '#112211'} />
      </mesh>

      {/* Rear — red */}
      <pointLight position={[-0.16, -0.01, 0]} intensity={armed ? 0.35 : 0} distance={0.4} color="#ff2222" />
      <mesh position={[-0.155, -0.018, 0]}>
        <sphereGeometry args={[0.006, 6, 6]} />
        <meshBasicMaterial color={armed ? '#ff2222' : '#221111'} />
      </mesh>

      {/* Left / Right — blue status */}
      {([[ 0, -0.018,  0.14], [0, -0.018, -0.14]] as [number, number, number][]).map((p, i) => (
        <mesh key={i} position={p}>
          <sphereGeometry args={[0.005, 6, 6]} />
          <meshBasicMaterial color={armed ? '#0099ff' : '#111122'} />
        </mesh>
      ))}
    </group>
  )
}
