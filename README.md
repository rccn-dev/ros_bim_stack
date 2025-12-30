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
│       ├── QueryBim.srv
│       └── FetchStream.srv
│
├── ros_speckle_bridge/      # Python package - Speckle driver
│   ├── config/
│   │   └── params.yaml
│   ├── launch/
│   │   └── bridge.launch.py
│   ├── ros_speckle_bridge/
│   │   ├── bridge_node.py
│   │   ├── speckle_client.py
│   │   ├── converter.py
│   │   ├── cache_manager.py
│   │   └── plugins/          # 🆕 Plugin system
│   │       ├── base.py       # Plugin interface
│   │       ├── loader.py     # Plugin loader
│   │       └── mesh_exporter.py  # Built-in mesh export plugin
│   └── tests/
│
├── examples/                 # 🆕 Integration examples
│   └── gazebo_integration/   # Gazebo Harmonic demo
│       ├── ros_speckle_gazebo/
│       │   └── gazebo_spawner.py  # Gazebo spawner plugin
│       ├── config/
│       ├── docker-compose.yml
│       └── README.md
│
├── Dockerfile               # Multi-stage Docker build
├── docker-compose.yml       # Production deployment
└── scripts/                 # Helper scripts
    └── run_tests.sh         # Test execution script
```

### Plugin Architecture

The bridge now supports **extensible output plugins** to keep the core package lightweight while enabling integration with various simulators and tools:

- **Core Package**: No simulation dependencies
- **Plugins**: Optional, loaded dynamically based on configuration
- **Examples**: Integration examples live in `examples/` directory

See [Plugin Development Guide](doc/extending.md) for creating custom plugins.

## Quick Start

### Using Docker (Recommended)

The easiest way to run the project is using Docker Compose.

1.  **Configure Environment**
    ```bash
    cp .env.template .env
    # Edit .env with your SPECKLE_TOKEN
    ```

2.  **Configure Bridge**
    ```bash
    # Edit stream_id in params.yaml
    nano ros_speckle_bridge/config/params.yaml
    ```

3.  **Run Tests**
    ```bash
    ./scripts/run_tests.sh
    ```

4.  **Start Bridge**
    ```bash
    docker-compose up --build
    ```

### Manual Installation

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

## Visualization & Networking

This stack supports multiple ways to visualize BIM data, depending on your environment.

### 1. Foxglove Studio (Web-based, Headless-friendly)
The easiest way to visualize data without a local ROS installation.
- Run the bridge with the visualization profile: `docker compose --profile viz up`
- Open [Foxglove Studio](https://app.foxglove.dev)
- Connect to `ws://localhost:8765`

### 2. RViz (Native ROS)
If you have ROS 2 installed locally (or via RoboStack on macOS):
- Run the bridge: `docker compose up`
- Run RViz: `ros2 launch ros_speckle_bridge bridge.launch.py rviz:=true`

### 3. Simulation Integration

The bridge can export BIM data to various simulators via plugins:

**Gazebo Harmonic** (Modern Gazebo):
```bash
cd examples/gazebo_integration
./run_demo.sh
```

See [examples/gazebo_integration](examples/gazebo_integration/README.md) for details.

**Other Simulators:**
- [NVIDIA Isaac Sim](examples/isaac_sim_integration/README.md) *(coming soon)*
- [Unity](examples/unity_integration/README.md) *(coming soon)*
- Create your own - see [Plugin Development](doc/extending.md)

### 4. Cross-Machine Networking (e.g., macOS to Linux)
If running RViz on a different machine, we recommend using **CycloneDDS** for better stability:

**On the Linux Host:**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
docker compose up
```

**On the Remote Machine (macOS/RoboStack):**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
rviz2
```

## ROS Interface
Humble (Ubuntu 22.04) or 
### Topics

- `/bim/objects` - `bim_interfaces/BimObject` - Individual BIM elements
- `/bim/visualization` - `visualization_msgs/MarkerArray` - RViz markers

### Services

- `/bim/query` - `bim_interfaces/QueryBim` - Query objects by filter
- `/bim/fetch` - `bim_interfaces/FetchStream` - Manually trigger a fetch from Speckle

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

- ROS 2: Rolling (Ubuntu 24.04)
- Python: 3.10+
- Speckle account with Personal Access Token