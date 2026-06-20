#!/bin/bash
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT/docker"

if ! command -v docker &>/dev/null; then
  echo "Install Docker Desktop: https://www.docker.com/products/docker-desktop/"
  exit 1
fi

if ! docker info &>/dev/null; then
  echo "Docker Desktop is not running. Open it and wait until it is ready."
  exit 1
fi

echo "Stopping old containers..."
docker rm -f aetherscan-simulation aetherscan-rosbridge aetherscan-dashboard 2>/dev/null || true
docker compose down --remove-orphans 2>/dev/null || true

echo "Building & starting simulation only (use ./run-aetherscan.sh for sim + dashboard)."
echo "First build: ~5–15 min. Rosbridge: ws://localhost:9090"
echo ""
docker compose up --build --force-recreate --remove-orphans aetherscan
