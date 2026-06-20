/** Shared sim-bridge socket (single connection for the dashboard). */

let socket: WebSocket | null = null

export function setSimBridgeSocket(ws: WebSocket | null) {
  socket = ws
}

export function simBridgeSend(msg: Record<string, unknown>) {
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(msg))
  }
}

export function isSimBridgeOpen(): boolean {
  return socket?.readyState === WebSocket.OPEN
}
