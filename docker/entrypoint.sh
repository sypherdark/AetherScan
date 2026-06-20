#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

WS="/aetherscan/aetherscan_ws"

if [ ! -f "${WS}/install/setup.bash" ]; then
  echo ">> Building AetherScan workspace (first run)..."
  cd "${WS}"
  colcon build --symlink-install
fi

source "${WS}/install/setup.bash"
exec "$@"
