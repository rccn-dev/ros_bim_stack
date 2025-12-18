# Installation Guide

## Requirements

- ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- Python 3.10+
- Speckle account with Personal Access Token

## Native Build

```bash
cd ~/ros2_ws/src
git clone <repository-url> ros_bim_stack

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install specklepy numpy tabulate

colcon build --symlink-install
source install/setup.bash
```

## Docker Build

```bash
# Copy environment template
cp .env.template .env

# Edit .env and set:
# - SPECKLE_TOKEN
# - SPECKLE_HOST (if not default)
# - RMW_IMPLEMENTATION (e.g., rmw_cyclonedds_cpp for macOS/remote)

# Edit config/params.yaml and set stream_id
nano ros_speckle_bridge/config/params.yaml

# Start the bridge
docker compose up --build
```

## Dev Container (macOS with Colima)

See [dev_container.md](dev_container.md) for complete setup instructions.

```bash
# Install prerequisites
brew install colima docker docker-compose

# Start Colima
colima start --cpu 4 --memory 8 --disk 100

# Open in VS Code
# Command Palette → "Dev Containers: Open Folder in Container"
```
