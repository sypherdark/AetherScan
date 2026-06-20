# AetherScan on macOS — Quick Fix Guide

## What was broken

1. **Docker volume** overwrote the built ROS workspace → simulation crashed immediately.
2. **Gazebo** does not run reliably on Apple Silicon → replaced with **indoor physics simulator** (same office layout, LiDAR, IMU, odometry).
3. **Dashboard sidebar** only changed highlight — now switches Map / Nav / Camera / Metrics / Teleop / Settings views.
4. **Mission buttons** work in **demo mode** (offline) and via **ROS** when Docker is running.

---

## Run (unified — recommended)

One command starts **simulation + 3D dashboard** (Docker):

```bash
cd "/Users/oubaidfradi/drone software "
./run-aetherscan.sh
```

Open **http://localhost:3000** → pick **Apartment** or **Boardroom** under Settings for `.ply` indoor meshes.

### Local dev (sim in Docker, dashboard hot-reload)

```bash
./run-aetherscan.sh --local
```

### Legacy (two terminals)

```bash
./run-simulation-docker.sh   # terminal 1 — rosbridge :9090
./run-dashboard.sh           # terminal 2 — http://localhost:3000
```

Click **Play** (mission start) to begin autonomous scanning.

---

## Sidebar buttons

| Button | Action |
|--------|--------|
| Map | 3D office + point cloud + drone |
| Navigation | 3D view + nav stats |
| Camera | Camera panel |
| Metrics | 3D + metrics |
| Teleop | Arm / takeoff / land controls |
| Settings | FPS toggle, connection info |

---

## Without Docker

Dashboard still works in **Demo mode** (amber label). Mission buttons animate a realistic scan in the 3D office.

---

## Redwood physics sim (`redwood_sim/`)

Open3D + **Apple CLT Python 3.9** (`/usr/bin/python3`) on ARM Macs is a bad match (segfaults in visualization). Use Homebrew Python:

```bash
cd "/Users/oubaidfradi/drone software /redwood_sim"
./setup-macos.sh
./run.sh --scene apartment
```

`run.sh` prefers `.venv` → Homebrew `python3.11` → `python3`, and warns if you're still on CLT Python.
