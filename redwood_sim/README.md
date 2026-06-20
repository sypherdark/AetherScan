# AetherScan Indoor Quadcopter Simulation

Production-grade 6-DoF indoor autonomous patrol simulation with Open3D visualization.

## Architecture

```
redwood_sim/
├── main.py                 # Entry point
├── config.py               # Timestep & patrol parameters
├── core/
│   ├── physics.py          # RK4 rigid body, drag, ground effect
│   ├── navigation.py       # Infinite CubicSpline waypoint patrol
│   ├── controls.py         # Quaternion cascading PID
│   └── math3d.py           # Quaternion / rotation utilities
├── visualization/
│   └── renderer.py         # PBR Filament (fallback: legacy visualizer)
├── simulation/
│   └── engine.py           # Fixed-timestep loop
├── scene_loader.py         # Mesh load + raycasting
└── procedural_scene.py     # Procedural mesh fallback
```

## Run

```bash
./setup-macos.sh          # once: Python 3.11 venv + deps
./run.sh --scene apartment
```

Or:

```bash
.venv/bin/python main.py --scene apartment
```

## Behaviour

- **Autonomous patrol** starts immediately — infinite waypoint loop with `scipy.interpolate.CubicSpline` (no early stop).
- **Physics:** RK4 integration, cross-frame inertia, linear/angular drag, ground-effect thrust boost below 0.5 m AGL.
- **Control:** Outer position loop → velocity → inner quaternion attitude torque loop.
- **Graphics:** Open3D `MaterialRecord` PBR when Filament GUI is available; otherwise lit legacy mesh.

## Controls (legacy visualizer)

| Key | Action |
|-----|--------|
| A | Toggle autonomous / manual |
| SPACE | Recover hover |
| ESC | Quit |

## Options

```bash
python main.py --scene boardroom --procedural
python main.py --download-real --scene office
python main.py --mesh /path/to/room.ply --voxel 0.02
```
