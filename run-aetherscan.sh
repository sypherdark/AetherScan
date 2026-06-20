#!/bin/bash
# Launch AetherScan: ROS simulation + unified 3D dashboard
# Usage:
#   ./run-aetherscan.sh              # Docker: sim + dashboard
#   ./run-aetherscan.sh --local      # Docker sim + local Next.js dev
#   ./run-aetherscan.sh --dashboard  # Dashboard + redwood_sim physics bridge
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

export_meshes() {
  if [ -f "$ROOT/dashboard/public/meshes/apartment.ply" ]; then
    return 0
  fi
  echo "==> Copying authentic meshes for dashboard (requires cache or data/replica/)..."
  if [ -x "$ROOT/redwood_sim/.venv/bin/python" ]; then
    "$ROOT/redwood_sim/.venv/bin/python" "$ROOT/scripts/export-meshes-to-dashboard.py"
  elif command -v python3 &>/dev/null; then
    python3 "$ROOT/scripts/export-meshes-to-dashboard.py"
  else
    echo "Warning: no Python — mesh scenes may be empty. Install Python or run redwood_sim/setup-macos.sh"
    mkdir -p "$ROOT/dashboard/public/meshes"
  fi
}

docker_ready() {
  docker info &>/dev/null
}

MODE="${1:-docker}"
# Optional --scene <id> flag (for --dashboard mode). Defaults to 'apartment_1' (real Replica scan).
SCENE="apartment_1"
for i in "$@"; do
  if [ "$i" = "--scene" ]; then shift; SCENE="${1:-apartment_1}"; break; fi
  shift
done

export_meshes

case "$MODE" in
  --local)
    if ! docker_ready; then
      echo "Docker Desktop is not running."
      echo ""
      echo "  Option A — Start Docker Desktop, wait until it says Running, then:"
      echo "    ./run-aetherscan.sh --local"
      echo ""
      echo "  Option B — Dashboard + physics bridge (no ROS):"
      echo "    ./run-aetherscan.sh --dashboard"
      echo ""
      read -r -p "Start dashboard only now? [y/N] " ans
      case "$ans" in
        [yY]|[yY][eE][sS]) exec "$0" --dashboard ;;
        *) exit 1 ;;
      esac
    fi
    echo "==> Starting simulation container (rosbridge :9090)..."
    docker rm -f aetherscan-simulation 2>/dev/null || true
    (cd "$ROOT/docker" && docker compose up --build -d aetherscan)
    echo "==> Starting dashboard dev server (http://localhost:3000)..."
    export NEXT_PUBLIC_ROSBRIDGE_URL="${NEXT_PUBLIC_ROSBRIDGE_URL:-ws://localhost:9090}"
    cd "$ROOT/dashboard"
    npm install
    npm run dev
    ;;
  --dashboard)
    export NEXT_PUBLIC_ROSBRIDGE_URL="${NEXT_PUBLIC_ROSBRIDGE_URL:-ws://localhost:9090}"
    export NEXT_PUBLIC_SIM_BRIDGE_URL="${NEXT_PUBLIC_SIM_BRIDGE_URL:-ws://127.0.0.1:8765}"
    SIM_PID=""
    SIM_LOG="${TMPDIR:-/tmp}/aetherscan-sim-bridge.log"
    cleanup_sim() {
      if [ -n "${SIM_PID:-}" ]; then
        kill "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
      fi
    }
    trap cleanup_sim EXIT INT TERM

    wait_for_bridge() {
      local py="$ROOT/redwood_sim/.venv/bin/python"
      local i=0
      while [ "$i" -lt 45 ]; do
        if "$py" -c "import socket; s=socket.socket(); s.settimeout(0.4); s.connect(('127.0.0.1',8765)); s.close()" 2>/dev/null; then
          return 0
        fi
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
          return 1
        fi
        sleep 1
        i=$((i + 1))
      done
      return 1
    }

    if [ -x "$ROOT/redwood_sim/.venv/bin/python" ]; then
      echo "==> Starting integrated physics (headless, ws://127.0.0.1:8765)..."
      : > "$SIM_LOG"
      (
        cd "$ROOT/redwood_sim"
        .venv/bin/python -m pip install -q websockets 2>/dev/null || true
        exec .venv/bin/python -m bridge --scene "$SCENE"
      ) >>"$SIM_LOG" 2>&1 &
      SIM_PID=$!
      echo "    Bridge log: $SIM_LOG"
      if ! wait_for_bridge; then
        echo "ERROR: Physics bridge failed to start. Last log lines:"
        tail -n 25 "$SIM_LOG" 2>/dev/null || true
        echo ""
        echo "  Fix: cd redwood_sim && ./setup-macos.sh"
        exit 1
      fi
      echo "    Physics bridge ready."
    else
      echo "ERROR: redwood_sim venv missing. Run: cd redwood_sim && ./setup-macos.sh"
      exit 1
    fi
    cd "$ROOT/dashboard"
    npm install
    echo "Unified dashboard: http://localhost:3000"
    echo "  Physics bridge: ${NEXT_PUBLIC_SIM_BRIDGE_URL}"
    npm run dev
    ;;
  --docker|docker)
    if ! docker_ready; then
      echo "Docker Desktop is not running."
      echo "  Install: https://www.docker.com/products/docker-desktop/"
      echo "  Or use:  ./run-aetherscan.sh --dashboard"
      exit 1
    fi
    docker rm -f aetherscan-simulation aetherscan-dashboard 2>/dev/null || true
    echo "==> Building & starting simulation + dashboard..."
    echo "    Dashboard:  http://localhost:3000"
    echo "    Rosbridge:  ws://localhost:9090"
    cd "$ROOT/docker"
    docker compose up --build --force-recreate --remove-orphans
    ;;
  *)
    echo "Usage: $0 [--docker | --local | --dashboard]"
    exit 1
    ;;
esac
