# Multi-stage Dockerfile for ROS2 Roomba Coverage Planner
# Stage 1: Builder
FROM ros:jazzy-ros-core AS builder

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-smoother \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-slam-toolbox \
    ros-jazzy-slam-toolbox-toolbox \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geometry2 \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir \
    numpy \
    opencv-python \
    scipy \
    shapely

# Create workspace
WORKDIR /root/roomba_ws

# Copy source code (excluding build/install/log)
COPY src ./src
COPY .

# Install rosdep dependencies
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install && \
    rm -rf build/*/install_manifest.txt"

# Stage 2: Runtime
FROM ros:jazzy-ros-core

# Install runtime dependencies only
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-smoother \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-slam-toolbox \
    ros-jazzy-slam-toolbox-toolbox \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geometry2 \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install runtime Python packages
RUN pip install --no-cache-dir \
    numpy \
    opencv-python \
    scipy \
    shapely

# Create workspace and copy built artifacts
WORKDIR /root/roomba_ws
COPY --from=builder /root/roomba_ws/install ./install
COPY --from=builder /root/roomba_ws/src ./src
COPY --from=builder /root/roomba_ws/maps ./maps

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/roomba_ws/install/setup.bash" >> /root/.bashrc

# Create entrypoint script
RUN mkdir -p /entrypoint && \
    echo '#!/bin/bash\n\
set -e\n\
\n\
echo "ROS2 Roomba Coverage Planner - Docker Setup"\n\
echo "=========================================="\n\
echo ""\n\
echo "Environment:"\n\
echo "  ROS_DISTRO: $ROS_DISTRO"\n\
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"\n\
echo ""\n\
\n\
source /opt/ros/jazzy/setup.bash\n\
source /root/roomba_ws/install/setup.bash\n\
\n\
exec "$@"\n\
' > /entrypoint/entrypoint.sh && chmod +x /entrypoint/entrypoint.sh

ENTRYPOINT ["/entrypoint/entrypoint.sh"]
CMD ["bash"]
