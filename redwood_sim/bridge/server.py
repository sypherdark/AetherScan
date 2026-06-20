"""
WebSocket bridge — streams quadcopter physics to the unified dashboard (port 8765).

The dashboard is the only UI; this process has no Open3D window.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import traceback
from pathlib import Path
from typing import Any, Dict, Optional, Set

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import websockets
    from websockets.server import WebSocketServerProtocol, serve
except ImportError as exc:
    raise ImportError("pip install websockets") from exc

from config import SimConfig
from scene_loader import (
    REPLICA_MISSING_MSG,
    MeshUnavailableError,
    RedwoodScene,
    ensure_triangle_mesh_file,
    find_bundled_mesh,
    is_collision_mesh_path,
    is_replica_scene_name,
    load_replica_redwood_scene,
    load_semantic_redwood_scene,
    resolve_dashboard_collision_mesh,
    resolve_dashboard_visual_url,
    resolve_mesh_path,
)
from simulation.engine import SimulationEngine

WORKSPACE_ROOT = ROOT.parent
DASHBOARD_MESHES = WORKSPACE_ROOT / "dashboard" / "public" / "meshes"

SCENE_ALIASES = {
    "office": "office",
    "apartment": "apartment",
    "boardroom": "boardroom",
    "livingroom": "apartment",
    "control": "control_room",
    "control_room": "control_room",
    # Real Meta Replica scan (apartment_1 from ai-habitat/habitat_test_scenes)
    "apartment_1": "apartment_1",
    "replica_apartment": "apartment_1",
    "real": "apartment_1",
}


def load_scene(
    scene_id: str,
    voxel: float,
    center_mesh: bool = False,
) -> tuple[RedwoodScene, str]:
    raw_id = scene_id
    if is_replica_scene_name(raw_id):
        replica_key = raw_id.lower()
        print(f"[sim-bridge] Loading Replica scene {replica_key}")
        scene = load_replica_redwood_scene(
            replica_key,
            voxel_downsample=voxel,
            dashboard_meshes_dir=DASHBOARD_MESHES,
        )
        if scene is not None:
            scene.log_mesh_info()
            return scene, replica_key
        pass  # Replica not installed; falling through to dashboard-mesh pipeline

    scene_id = SCENE_ALIASES.get(scene_id.lower(), scene_id.lower())
    if is_replica_scene_name(raw_id):
        scene_id = "apartment"

    semantic_loaded = load_semantic_redwood_scene(
        scene_id, DASHBOARD_MESHES, center_mesh=center_mesh
    )
    if semantic_loaded is not None:
        scene, sid = semantic_loaded
        scene.log_mesh_info()
        if scene.has_triangle_semantics:
            print(f"[sim-bridge] Semantic labels: {len(scene._triangle_semantics):,} triangles")
        return scene, sid

    collision_path = resolve_dashboard_collision_mesh(scene_id, DASHBOARD_MESHES)
    if collision_path is not None:
        mesh_path = (
            collision_path
            if is_collision_mesh_path(collision_path)
            else ensure_triangle_mesh_file(collision_path)
        )
    else:
        try:
            mesh_path = ensure_triangle_mesh_file(resolve_mesh_path(scene_name=scene_id))
        except MeshUnavailableError as exc:
            bundled = find_bundled_mesh(scene_id)
            if bundled is None:
                raise SystemExit(f"[sim-bridge] FATAL: {exc}") from exc
            mesh_path = ensure_triangle_mesh_file(bundled)

    visual_url = resolve_dashboard_visual_url(scene_id, DASHBOARD_MESHES)
    collision_voxel = 0.0 if is_collision_mesh_path(mesh_path) else voxel

    print(f"[sim-bridge] Loading {scene_id}")
    print(f"             collision: {mesh_path}")
    if visual_url:
        print(f"             visual:    {visual_url}")

    scene = RedwoodScene(
        mesh_path,
        voxel_downsample=collision_voxel,
        center_mesh=center_mesh,
        visual_mesh_url=visual_url,
    )
    scene.log_mesh_info()
    return scene, scene_id


class SimBridgeServer:
    def __init__(self, engine: SimulationEngine, rate_hz: float = 20.0):
        self.engine = engine
        self.dt = engine.config.control_dt
        self.broadcast_interval = 1.0 / rate_hz
        self.clients: Set[WebSocketServerProtocol] = set()
        self._reload_scene_id: Optional[str] = None
        # ID of the last camera snapshot that was included in a broadcast.
        # The engine assigns monotonically increasing IDs so this works even
        # when the rolling gallery window drops old entries.
        # register() sends the full gallery to new clients; broadcast_loop
        # only sends snapshots with id > this value.
        self._broadcast_last_snap_id: int = 0
        # Reconstruction-cloud high-water mark: count of recon points already
        # broadcast.  Each delta sends recon_points[mark:]; register() sends the
        # full cloud to a new client.  Reset when it shrinks (mission restart).
        self._recon_sent: int = 0
        self._send_lock = asyncio.Lock()

    async def register(self, ws: WebSocketServerProtocol) -> None:
        self.clients.add(ws)
        hello: Dict[str, Any] = {"type": "hello", "scene": self.engine.scene_id}
        visual_url = getattr(self.engine.scene, "visual_mesh_url", None)
        if visual_url:
            hello["visual_mesh_url"] = visual_url
        b = self.engine.scene.bounds
        hello["scene_bounds"] = {
            "min": b.min_corner.tolist(),
            "max": b.max_corner.tolist(),
            "center": ((b.min_corner + b.max_corner) * 0.5).tolist(),
            "extent": (b.max_corner - b.min_corner).tolist(),
        }
        # Expose the normalization offset (ROS Z-up) so the dashboard can align
        # the visual GLB mesh with the simulation's normalised coordinate frame.
        norm = getattr(self.engine.scene, "_normalize_transform", None)
        if norm is not None:
            # T[i,3] are the XYZ translation components of the 4×4 matrix
            hello["mesh_norm_offset"] = [
                float(norm[0, 3]),  # ROS X
                float(norm[1, 3]),  # ROS Y
                float(norm[2, 3]),  # ROS Z
            ]
        # Through the same lock so the initial hello/full-state can't interleave
        # with a concurrent broadcast on this socket.
        async with self._send_lock:
            await ws.send(json.dumps(hello))
            await ws.send(self._telemetry_payload())

    async def unregister(self, ws: WebSocketServerProtocol) -> None:
        self.clients.discard(ws)

    async def handler(self, ws: WebSocketServerProtocol) -> None:
        await self.register(ws)
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                await self._handle_message(msg)
        finally:
            await self.unregister(ws)

    async def _handle_message(self, msg: Dict[str, Any]) -> None:
        try:
            op = msg.get("op")
            if op == "mission":
                self.engine.mission_command(str(msg.get("command", "")))
            elif op == "set_autonomous":
                self.engine.controller.autonomous = bool(msg.get("enabled", True))
            elif op == "set_god_mode":
                self.engine.set_god_mode(bool(msg.get("enabled", False)))
            elif op == "export_scan":
                # Poisson meshing takes seconds — keep it off the physics loop.
                asyncio.create_task(self._run_export())
            elif op == "set_scene":
                self._reload_scene_id = str(msg.get("scene", "apartment"))
        except Exception as exc:
            print(f"[sim-bridge] message handler error: {exc}")
            traceback.print_exc()

    async def _run_export(self) -> None:
        """Generate scan deliverables in a worker thread, then notify clients."""
        try:
            await self._broadcast(json.dumps({"type": "export_started"}))
            manifest = await asyncio.to_thread(self.engine.export_scan_deliverables)
            manifest["type"] = "export_complete"
            await self._broadcast(json.dumps(manifest))
            print(f"[sim-bridge] export complete: {[f['file'] for f in manifest['files']]}"
                  + (f" errors={manifest['errors']}" if manifest.get("errors") else ""))
        except Exception as exc:
            traceback.print_exc()
            await self._broadcast(json.dumps(
                {"type": "export_complete", "files": [], "urls": [],
                 "errors": [str(exc)]}))

    async def physics_loop(self) -> None:
        # When idle (no active mission) we run the physics loop at 10 Hz instead
        # of 100 Hz to keep CPU low.  Full rate resumes the moment a mission starts.
        IDLE_DT = 0.10   # 10 Hz idle
        while True:
            try:
                if self._reload_scene_id:
                    scene_id = self._reload_scene_id
                    self._reload_scene_id = None
                    try:
                        await self._reload_engine(scene_id)
                    except Exception as exc:
                        print(f"[sim-bridge] scene reload failed ({scene_id}): {exc}")
                        traceback.print_exc()
                # God mode runs several physics ticks per real-time loop pass
                # (time acceleration): same stable dynamics, faster wall-clock.
                substeps = getattr(self.engine, "god_substeps", 1) if self.engine._mission_active else 1
                for _ in range(substeps):
                    self.engine.tick(self.dt)
            except Exception as exc:
                print(f"[sim-bridge] physics step error: {exc}")
                traceback.print_exc()
                self.engine.mission_state = "ERROR"
                self.engine.controller.autonomous = False
            sleep_dt = self.dt if self.engine._mission_active else IDLE_DT
            await asyncio.sleep(sleep_dt)

    async def _reload_engine(self, scene_id: str) -> None:
        voxel = self.engine.config.voxel_downsample
        cfg = self.engine.config
        scene, sid = load_scene(scene_id, voxel)
        self.engine = SimulationEngine(scene, cfg, headless=True, scene_id=sid)
        b = self.engine.scene.bounds
        changed: Dict[str, Any] = {
            "type": "scene_changed",
            "scene": sid,
            "scene_bounds": {
                "min": b.min_corner.tolist(),
                "max": b.max_corner.tolist(),
            },
        }
        visual_url = getattr(self.engine.scene, "visual_mesh_url", None)
        if visual_url:
            changed["visual_mesh_url"] = visual_url
        await self._broadcast(json.dumps(changed))
        # New engine → no snapshots have been broadcast yet
        self._broadcast_last_snap_id = 0

    def _telemetry_payload(self, delta_only: bool = False) -> str:
        """Serialise a telemetry frame.

        When *delta_only* is True the ``camera_gallery`` field is replaced with
        only the snapshots whose ``id`` is greater than the last one that was
        broadcast.  The internal high-water-mark is always advanced after each
        call so the next delta picks up right where this one left off.

        Use delta_only=False (the default) for the initial full-state message
        sent to a freshly connected client — they receive the full history, and
        the high-water-mark is advanced so the next broadcast sends nothing
        redundant.
        """
        try:
            data = self.engine.get_telemetry()
            full_gallery: list = data.get("camera_gallery") or []
            if delta_only:
                new_snaps = [s for s in full_gallery
                             if s.get("id", 0) > self._broadcast_last_snap_id]
                data["camera_gallery"] = new_snaps
            # Advance high-water-mark to the highest id seen so far
            if full_gallery:
                self._broadcast_last_snap_id = max(
                    self._broadcast_last_snap_id,
                    max(s.get("id", 0) for s in full_gallery),
                )
            # Reconstruction cloud: delta for broadcasts, full for a new client.
            recon: list = data.get("map_points") or []
            if len(recon) < self._recon_sent:
                self._recon_sent = 0          # mission restarted → cloud reset
            if delta_only:
                data["map_points"] = recon[self._recon_sent:]
            # else: keep the full cloud for the freshly-connected client
            data["map_points_reset"] = (not delta_only)  # client clears then loads full
            self._recon_sent = len(recon)
            return json.dumps(data)
        except Exception as exc:
            print(f"[sim-bridge] telemetry serialization error: {exc}")
            traceback.print_exc()
            return json.dumps(
                {
                    "type": "state",
                    "scene": self.engine.scene_id,
                    "mission_state": "ERROR",
                    "nav_state": "EXPLORING",
                    "error": str(exc),
                }
            )

    async def broadcast_loop(self) -> None:
        while True:
            await self._broadcast(self._telemetry_payload(delta_only=True))
            await asyncio.sleep(self.broadcast_interval)

    async def _broadcast(self, payload: str) -> None:
        if not self.clients:
            return
        # Serialise ALL sends through one lock.  Two coroutines (physics_loop's
        # scene_changed and broadcast_loop's 20 Hz state) used to write the same
        # socket concurrently, tripping the websockets drain assertion
        # (`assert waiter is None or waiter.cancelled()`), which dropped the
        # connection mid scene-switch and left the client on a stale scene.
        async with self._send_lock:
            dead: list[WebSocketServerProtocol] = []
            for ws in list(self.clients):  # snapshot to avoid mutation-during-iteration
                try:
                    await ws.send(payload)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.clients.discard(ws)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="AetherScan physics bridge for dashboard")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument(
        "--scene",
        default="apartment",
        help="apartment | control_room | office | boardroom | replica:office_1 (etc.)",
    )
    p.add_argument("--rate", type=float, default=20.0, help="Telemetry broadcast Hz")
    p.add_argument("--voxel", type=float, default=0.03)
    p.add_argument("--dt", type=float, default=0.002)
    p.add_argument(
        "--center-mesh",
        action="store_true",
        help="Legacy mesh re-centering (breaks dashboard alignment)",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = SimConfig(physics_dt=args.dt, voxel_downsample=args.voxel)
    scene, scene_id = load_scene(args.scene, args.voxel, center_mesh=args.center_mesh)
    engine = SimulationEngine(scene, cfg, headless=True, scene_id=scene_id)
    bridge = SimBridgeServer(engine, rate_hz=args.rate)

    async def run() -> None:
        async with serve(bridge.handler, args.host, args.port):
            print(
                f"[sim-bridge] ws://{args.host}:{args.port} "
                f"scene={scene_id} (dashboard is the UI)"
            )
            await asyncio.gather(bridge.physics_loop(), bridge.broadcast_loop())

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n[sim-bridge] stopped")


if __name__ == "__main__":
    main()
