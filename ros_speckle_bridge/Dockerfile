# Multi-stage Dockerfile for ROS Speckle Bridge
# Target: ROS 2 Humble (Ubuntu 22.04) / Jazzy (Ubuntu 24.04)

# Build argument for ROS distro
ARG ROS_DISTRO=rolling

# ==============================================================================
# Stage 1: Builder
# ==============================================================================
FROM ros:${ROS_DISTRO} AS builder

# Set working directory
WORKDIR /workspace

# Install build dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    specklepy \
    numpy \
    tabulate

# Copy source code
COPY bim_interfaces ./src/bim_interfaces
COPY ros_speckle_bridge ./src/ros_speckle_bridge

# Create default params.yaml from example if it doesn't exist
RUN if [ ! -f ./src/ros_speckle_bridge/config/params.yaml ]; then \
    cp ./src/ros_speckle_bridge/config/params.example.yaml ./src/ros_speckle_bridge/config/params.yaml; \
    fi

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install

# ==============================================================================
# Stage 2: Runtime
# ==============================================================================
FROM ros:${ROS_DISTRO}-ros-base AS runtime

# Set working directory
WORKDIR /workspace

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (runtime only)
RUN pip3 install --no-cache-dir \
    specklepy \
    numpy \
    tabulate

# Copy built artifacts from builder
COPY --from=builder /workspace/install ./install

# Setup ROS environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# Create cache directory
RUN mkdir -p /root/.ros/speckle_cache

# Environment variables
ENV PYTHONUNBUFFERED=1
ENV ROS_DOMAIN_ID=0

# Entrypoint script
COPY ros_speckle_bridge/docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros_speckle_bridge", "bridge.launch.py"]
