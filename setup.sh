# AetherScan Development Environment Setup
#
# Prerequisites:
# - ROS2 Humble: https://docs.ros.org/en/humble/Installation.html
# - Gazebo Garden: https://gazebosim.org/docs/garden/install
# - Node.js 18+: https://nodejs.org/
#
# Quick setup:
#   chmod +x setup.sh && ./setup.sh

#!/bin/bash
set -e

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}"
echo "╔═══════════════════════════════════════╗"
echo "║       AetherScan Setup Script         ║"
echo "║   Indoor Scanning Drone Simulation    ║"
echo "╚═══════════════════════════════════════╝"
echo -e "${NC}"

# Check ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}[!] ROS2 not sourced. Attempting to source Humble...${NC}"
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}[✓] Sourced ROS2 Humble${NC}"
    else
        echo "[✗] ROS2 Humble not found. Please install it first."
        exit 1
    fi
fi

echo -e "\n${CYAN}[1/4] Installing ROS2 dependencies...${NC}"
sudo apt-get update -qq
sudo apt-get install -y -qq \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    python3-colcon-common-extensions \
    python3-pip

echo -e "\n${CYAN}[2/4] Installing Python dependencies...${NC}"
pip3 install -q open3d scikit-learn transforms3d numpy scipy

echo -e "\n${CYAN}[3/4] Building ROS2 workspace...${NC}"
cd "$(dirname "$0")/aetherscan_ws"
colcon build --symlink-install
source install/setup.bash
echo -e "${GREEN}[✓] ROS2 workspace built successfully${NC}"

echo -e "\n${CYAN}[4/4] Installing dashboard dependencies...${NC}"
cd "$(dirname "$0")/dashboard"
if command -v npm &> /dev/null; then
    npm install
    echo -e "${GREEN}[✓] Dashboard dependencies installed${NC}"
else
    echo -e "${YELLOW}[!] npm not found. Skipping dashboard setup.${NC}"
    echo "    Install Node.js 18+ to use the web dashboard."
fi

echo -e "\n${GREEN}╔═══════════════════════════════════════╗${NC}"
echo -e "${GREEN}║     Setup Complete! 🚀                ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════╝${NC}"
echo ""
echo "To start the simulation:"
echo "  source aetherscan_ws/install/setup.bash"
echo "  ros2 launch aetherscan_bringup simulation.launch.py"
echo ""
echo "To start the dashboard:"
echo "  cd dashboard && npm run dev"
echo ""
