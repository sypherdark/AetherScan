# AetherScan - Autonomous Indoor 3D Scanning Drone

<p align="center">
  <strong>Professional indoor mapping simulation powered by ROS2 + Gazebo</strong>
</p>

---

## Overview

AetherScan is a complete autonomous indoor scanning drone simulation system. It combines a realistic Gazebo simulation environment with a full ROS2 software stack for autonomous 3D mapping of indoor spaces.

### Key Capabilities

- **Fully Autonomous Mapping** — Start a mission and the drone explores, maps, and returns home automatically
- **Real-time 3D Reconstruction** — High-quality point cloud generation and mesh reconstruction using RTAB-Map
- **Intelligent Exploration** — Frontier-based exploration with coverage path planning
- **Obstacle Avoidance** — Real-time collision avoidance using depth camera and LiDAR
- **Web Dashboard** — Beautiful real-time 3D visualization built with Next.js + Three.js
- **Multiple Environments** — Office, warehouse, and apartment simulation worlds
- **Manual Override** — Keyboard teleoperation available at any time

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    AetherScan System                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Gazebo   │  │  SLAM    │  │Navigation│  │ Mission  │   │
│  │Simulation│──│ Pipeline │──│  Stack   │──│ Control  │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
│       │              │              │              │         │
│       ▼              ▼              ▼              ▼         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │  Sensor  │  │ 3D Map   │  │  Path    │  │  State   │   │
│  │  Feeds   │  │ Building │  │ Planning │  │ Machine  │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            Web Dashboard (Next.js + Three.js)         │   │
│  │  • 3D Point Cloud Viewer  • Mission Control          │   │
│  │  • Camera Feed            • Performance Metrics      │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (recommended) or macOS with Docker
- **RAM**: 16 GB minimum (32 GB recommended)
- **GPU**: NVIDIA GPU recommended for Gazebo rendering
- **Disk**: 10 GB free space

### Software Dependencies
- ROS2 Humble Hawksbill
- Gazebo Garden (gz-sim)
- RTAB-Map ROS2 package
- Node.js 18+ (for dashboard)
- Python 3.10+

---

## Installation

### Option 1: Docker (Recommended)

Unified **simulation + 3D dashboard** (indoor `.ply` meshes, mission control, live point cloud):

```bash
./run-aetherscan.sh
# → http://localhost:3000  |  rosbridge ws://localhost:9090
```

Local dashboard dev with Docker sim only:

```bash
./run-aetherscan.sh --local
```

Export / refresh mesh assets for the dashboard:

```bash
python3 scripts/export-meshes-to-dashboard.py
```

### Option 2: Native Installation

#### 1. Install ROS2 Humble
```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-rtabmap-ros \
  ros-humble-rosbridge-server \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  python3-colcon-common-extensions
```

#### 2. Build the Workspace
```bash
cd aetherscan_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

#### 3. Install Dashboard
```bash
cd dashboard
npm install
```

---

## Quick Start

### Launch Full Autonomous Scanning Mission

```bash
# Terminal 1: Start simulation + all systems
source aetherscan_ws/install/setup.bash
ros2 launch aetherscan_bringup simulation.launch.py world:=office_environment

# Terminal 2: Start autonomous mission
ros2 launch aetherscan_bringup autonomous_scan.launch.py

# Terminal 3: Web dashboard
cd dashboard && npm run dev
# Open http://localhost:3000
```

### Launch with Manual Teleop

```bash
# Terminal 1: Simulation
ros2 launch aetherscan_bringup teleop_mode.launch.py world:=warehouse

# Terminal 2: Keyboard control
ros2 run aetherscan_teleop keyboard_teleop
```

### Available Worlds

| World | Description | Size |
|-------|-------------|------|
| `office_environment` | Multi-room office with corridors | 20m × 15m |
| `warehouse` | Large warehouse with shelving | 30m × 20m |
| `apartment` | Residential apartment | ~60 m² |

---

## Keyboard Teleop Controls

| Key | Action |
|-----|--------|
| `W` / `S` | Forward / Backward |
| `A` / `D` | Strafe Left / Right |
| `Q` / `E` | Yaw Left / Right |
| `R` / `F` | Altitude Up / Down |
| `T` | Takeoff |
| `L` | Land |
| `Space` | Emergency Stop |
| `M` | Switch to Autonomous Mode |

---

## Mission Control via ROS2 Services

```bash
# Start a scanning mission
ros2 service call /aetherscan/start_mission std_srvs/srv/Trigger

# Pause mission
ros2 service call /aetherscan/pause_mission std_srvs/srv/Trigger

# Resume mission
ros2 service call /aetherscan/resume_mission std_srvs/srv/Trigger

# Abort and return home
ros2 service call /aetherscan/abort_mission std_srvs/srv/Trigger
```

---

## Web Dashboard

The web dashboard provides real-time visualization at `http://localhost:3000`:

- **3D Map Viewer** — Interactive point cloud with orbit controls
- **Drone Tracking** — Real-time position and orientation
- **Camera Feed** — Live RGB and depth streams
- **Mission Panel** — Start/stop controls with progress tracking
- **Metrics** — Coverage %, area mapped, point count, flight time

---

## Configuration

Key parameters can be adjusted in `aetherscan_ws/src/*/config/`:

- `control_params.yaml` — PID gains, velocity limits, safety envelope
- `exploration_params.yaml` — Frontier detection, coverage settings
- `slam_params.yaml` — RTAB-Map parameters, voxel sizes
- `mission_params.yaml` — Scan altitude, speed, timeout

---

## Project Structure

```
aetherscan_ws/src/
├── aetherscan_description/   # Drone URDF/SDF models and sensors
├── aetherscan_gazebo/        # Gazebo worlds and launch files
├── aetherscan_slam/          # SLAM pipeline (RTAB-Map integration)
├── aetherscan_navigation/    # Exploration, path planning, avoidance
├── aetherscan_perception/    # Point cloud processing, mesh reconstruction
├── aetherscan_control/       # Flight controller, trajectory tracking
├── aetherscan_teleop/        # Keyboard teleoperation
├── aetherscan_mission/       # Mission state machine, metrics
└── aetherscan_bringup/       # Top-level launch files, RViz config
dashboard/                    # Next.js + Three.js web interface
docker/                       # Docker Compose setup
config/                       # Global configuration
```

---

## Performance Metrics

The system tracks and reports:
- **Coverage**: Percentage of accessible area mapped
- **Density**: Average point cloud density (points/m²)
- **Accuracy**: Mapping quality score
- **Efficiency**: Time to complete coverage
- **Distance**: Total flight path length

---

## License

MIT License — See [LICENSE](LICENSE) for details.
