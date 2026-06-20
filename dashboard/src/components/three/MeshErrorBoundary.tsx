'use client'

import { Component, type ReactNode } from 'react'

type Props = {
  /** Identity that, when changed, resets the boundary so a new scene can retry. */
  resetKey?: string
  fallback: ReactNode
  children: ReactNode
  onError?: (error: Error) => void
}

type State = { hasError: boolean }

/**
 * Error boundary for the 3D mesh subtree.
 *
 * `useGLTF`/`useLoader` throw when an asset 404s or fails to parse (e.g. a
 * missing Replica GLB returns Next.js's HTML 404 page, which the GLTF binary
 * parser rejects).  A thrown error is NOT caught by <Suspense> — without a
 * boundary it unmounts the entire React tree and white-screens the dashboard.
 * This boundary catches it and renders a graceful in-scene placeholder instead,
 * so switching to a scene whose mesh is unavailable degrades cleanly.
 *
 * It must live INSIDE the <Canvas> (react-three-fiber) tree so the fallback is
 * itself valid 3D content.
 */
export class MeshErrorBoundary extends Component<Props, State> {
  state: State = { hasError: false }

  static getDerivedStateFromError(): State {
    return { hasError: true }
  }

  componentDidCatch(error: Error) {
    // eslint-disable-next-line no-console
    console.warn('[MeshErrorBoundary] mesh failed to load:', error?.message ?? error)
    this.props.onError?.(error)
  }

  componentDidUpdate(prev: Props) {
    // Reset when the caller switches to a different scene/url so the new asset
    // gets a fresh attempt instead of staying stuck on the previous failure.
    if (this.state.hasError && prev.resetKey !== this.props.resetKey) {
      this.setState({ hasError: false })
    }
  }

  render() {
    if (this.state.hasError) return <>{this.props.fallback}</>
    return <>{this.props.children}</>
  }
}
