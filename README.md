# ROS BIM Stack

ROS 2 stack that bridges Building Information Modeling (BIM) data from Speckle to the ROS 2 ecosystem. Designed with vendor-agnostic interfaces, offline resilience, and production deployment considerations.

## Overview

This project provides two ROS 2 packages:

- **bim_interfaces**: Vendor-agnostic message and service definitions for BIM data
- **ros_speckle_bridge**: Python implementation for Speckle API integration

## Architecture

```
ros_bim_stack/
├── bim_interfaces/          # CMake package - message definitions
│   ├── msg/
│   │   ├── Property.msg
│   │   ├── BimObject.msg
│   │   └── BimPartition.msg
│   └── srv/
│       └── QueryBim.srv
│
└── ros_speckle_bridge/      # Python package - Speckle driver
    ├── config/
    │   └── params.yaml
    ├── launch/
    │   └── bridge.launch.py
    └── ros_speckle_bridge/
        ├── bridge_node.py
        ├── speckle_client.py
        ├── converter.py
        └── cache_manager.py
```

## Quick Start

```bash
# Set Speckle token
export SPECKLE_TOKEN="your_token_here"

# Build workspace
cd ~/ros2_ws/src
git clone <repository-url> ros_bim_stack
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install specklepy numpy tabulate
colcon build --symlink-install
source install/setup.bash

# Configure stream
ros2 run ros_speckle_bridge list_streams
nano src/ros_bim_stack/ros_speckle_bridge/config/params.yaml  # Set stream_id

# Launch
ros2 launch ros_speckle_bridge bridge.launch.py
```

## ROS Interface

### Topics

- `/bim/objects` - `bim_interfaces/BimObject` - Individual BIM elements
- `/bim/visualization` - `visualization_msgs/MarkerArray` - RViz markers

### Services

- `/bim/query` - `bim_interfaces/QueryBim` - Query objects by filter

### TF Frames

- `map → bim_origin` - Static transform with datum offset

## Documentation

- [Installation](doc/installation.md) - Native, Docker, and Dev Container setup
- [Configuration](doc/configuration.md) - Authentication and parameters
- [Usage](doc/usage.md) - Launch, visualization, and queries
- [API Reference](doc/api.md) - Topics, services, messages, and coordinate systems
- [Development](doc/development.md) - Building, testing, and contributing
- [Deployment](doc/deployment.md) - Docker, Kubernetes, and systemd
- [Extending](doc/extending.md) - Adding providers and custom functionality
- [Troubleshooting](doc/troubleshooting.md) - Common issues and solutions

## System Requirements

- ROS 2: Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
- Python: 3.10+
- Speckle account with Personal Access Token