# Gazebo Harmonic Integration Example

This example demonstrates how to spawn BIM objects from Speckle into Gazebo Harmonic (the modern successor to Gazebo Classic).

## Architecture

```
┌─────────────────┐
│ Speckle Server  │ (Cloud BIM data)
└────────┬────────┘
         │ HTTPS
    ┌────▼─────────────┐
    │ speckle_bridge   │ Core bridge (publishes /bim/objects)
    │ + mesh_exporter  │ Plugin: exports meshes to /shared_meshes
    └────┬─────────────┘
         │ ROS 2 Topics
    ┌────▼───────────────┐
    │ gazebo_spawner     │ Plugin: spawns entities in Gazebo
    │                    │ Subscribes to /bim/objects
    └────┬───────────────┘
         │ Gazebo Services
    ┌────▼────────────┐
    │ Gazebo Harmonic │ Physics simulation
    └─────────────────┘
```

## Quick Start

### Prerequisites

- Docker and Docker Compose
- X11 forwarding configured (for GUI)

```bash
# Allow X11 connections
xhost +local:docker
```

### Run the Demo

```bash
cd examples/gazebo_integration
docker compose up
```

This will:
1. Start the Speckle bridge with mesh export enabled
2. Launch Gazebo Harmonic
3. Spawn BIM objects as Gazebo entities

### Environment Variables

Create a `.env` file:

```bash
# Speckle credentials
SPECKLE_TOKEN=your_token_here
SPECKLE_STREAM_ID=your_stream_id

# Optional
ROS_DOMAIN_ID=0
GAZEBO_VERBOSE=1
```

## Directory Structure

```
gazebo_integration/
├── config/
│   ├── gazebo_params.yaml    # ROS parameters
│   └── plugins.yaml           # Plugin configuration
├── meshes/                    # Exported mesh files (auto-generated)
├── ros_speckle_gazebo/
│   └── gazebo_spawner.py      # Gazebo spawner plugin
├── run_demo.sh                # Quick start script
└── docker-compose.yml         # Service orchestration
```

**Note**: The `meshes/` directory is created automatically and contains exported mesh files. You can safely delete it - meshes will be regenerated on next run.

## Plugin Configuration

The Gazebo spawner is configured via ROS parameters:

```yaml
# config/gazebo_params.yaml
/**:
  ros__parameters:
    plugins:
      - type: "mesh_exporter"
        enabled: true
        config:
          output_path: "/shared_meshes"
          format: "dae"  # Collada for Gazebo
          
      - type: "gazebo_spawner"
        enabled: true
        config:
          spawn_service: "/world/default/create"
          frame_id: "world"
          default_physics:
            static: false
            mass: 1.0
            friction: 0.8
```

## Customization

### Modify Spawning Logic

Edit [ros_speckle_gazebo/gazebo_spawner.py](ros_speckle_gazebo/gazebo_spawner.py) to customize:
- Collision geometry generation
- Material properties
- Physics parameters
- Object placement

### Change Simulation World

Edit [docker-compose.yml](docker-compose.yml) to load a custom world file:

```yaml
gazebo:
  command: gz sim -r ./worlds/custom_world.sdf
  volumes:
    - ./worlds:/worlds
```

## Troubleshooting

### No GUI / Black screen

Ensure X11 forwarding is enabled:
```bash
xhost +local:docker
echo $DISPLAY  # Should output something like :0 or :1
```

### Objects not spawning

Check logs:
```bash
docker compose logs gazebo_spawner
```

Common issues:
- Mesh files not generated (check `mesh_exporter` plugin)
- Invalid SDF format (check conversion logic)
- Service timeout (Gazebo may be slow to start)

### Performance issues

For headless simulation, use:
```bash
docker compose -f docker-compose.headless.yml up
```

## Development

### Build Custom Spawner

```bash
cd ros_speckle_gazebo
colcon build
source install/setup.bash
```

### Run Tests

```bash
colcon test --packages-select ros_speckle_gazebo
```

## Next Steps

- Explore [Isaac Sim integration](../isaac_sim_integration/README.md) *(coming soon)*
- Learn about [creating custom plugins](../../doc/extending.md)
- Read about [plugin architecture](../../doc/development.md#plugin-system)
