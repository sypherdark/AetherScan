/** Z-up ROS → Y-up Three.js (X=right, Y=up, Z=-forward). Matches MeshEnvironment Rx(-π/2). */
export function rosToThree(
  x: number,
  y: number,
  z: number
): [number, number, number] {
  return [x, z, -y]
}

export function rosPointToThree(
  pos: [number, number, number]
): [number, number, number] {
  return rosToThree(pos[0], pos[1], pos[2])
}
