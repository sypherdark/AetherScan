FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    python3-scipy \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    open3d \
    scikit-learn \
    transforms3d

WORKDIR /aetherscan

# Copy workspace
COPY aetherscan_ws /aetherscan/aetherscan_ws

# Build ROS2 workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /aetherscan/aetherscan_ws && \
    colcon build --symlink-install"

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "aetherscan_bringup", "simulation.launch.py"]
