'use client'

/**
 * Legacy procedural office boxes removed — scenes must provide meshUrl.
 */
export function OfficeEnvironment() {
  return (
    <group>
      <mesh position={[0, 1.2, 0]}>
        <boxGeometry args={[2.4, 0.08, 0.02]} />
        <meshBasicMaterial color="#f87171" />
      </mesh>
      <mesh position={[0, 0.5, 0]}>
        <boxGeometry args={[6, 0.6, 0.02]} />
        <meshBasicMaterial color="#94a3b8" transparent opacity={0.85} />
      </mesh>
    </group>
  )
}

export function NoMeshLoaded({ sceneId }: { sceneId: string }) {
  return (
    <group position={[0, 1, 0]}>
      <mesh>
        <boxGeometry args={[4, 1.2, 0.05]} />
        <meshStandardMaterial color="#1e293b" emissive="#450a0a" emissiveIntensity={0.35} />
      </mesh>
      <mesh position={[0, 0, 0.06]}>
        <planeGeometry args={[3.6, 0.5]} />
        <meshBasicMaterial color="#fca5a5" />
      </mesh>
    </group>
  )
}
