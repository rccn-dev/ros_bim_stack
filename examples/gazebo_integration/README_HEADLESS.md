# Headless Gazebo Integration with Remote RViz

This example runs Gazebo Harmonic in **headless mode** (no GUI) and uses **CycloneDDS** for network communication, allowing you to visualize in RViz from another machine.

## Architecture

```
┌─────────────────────────┐
│  Docker Host            │
│  ┌───────────────────┐  │
│  │ Bridge + Gazebo   │  │──┐
│  │ (Headless)        │  │  │ CycloneDDS
│  └───────────────────┘  │  │ (Network)
└─────────────────────────┘  │
                             │
┌─────────────────────────┐  │
│  Remote Machine         │  │
│  ┌───────────────────┐  │  │
│  │ RViz              │◄─┘
│  │ (Visualization)   │  │
│  └───────────────────┘  │
└─────────────────────────┘
```

## Setup

### 1. Configure Environment

```bash
cd examples/gazebo_integration
cp .env.template .env
# Edit .env with your Speckle credentials
```

### 2. Start Headless Services

```bash
./run_demo.sh
```

This starts:
- Speckle Bridge (fetches BIM data, publishes ROS topics)
- Gazebo Harmonic (headless simulation)
- CycloneDDS (network middleware)

### 3. Visualize from Remote Machine

On your **other machine** (same network):

```bash
# Install ROS 2 Humble and CycloneDDS
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedds-cpp

# Set environment
export ROS_DOMAIN_ID=0  # Match the domain from .env
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optional: Use the same CycloneDDS config
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml

# Source ROS
source /opt/ros/humble/setup.bash

# Launch RViz
rviz2
```

## RViz Configuration

Add these displays in RViz:

1. **TF** - Shows coordinate frames
   - Fixed Frame: `map`

2. **MarkerArray** - Shows BIM geometry
   - Topic: `/bim/visualization`

3. **RobotModel** (optional) - If spawning robots

## Topics Published

```bash
# From remote machine, list topics
ros2 topic list

# Expected topics:
/bim/objects              # BIM object data
/bim/visualization        # Visualization markers
/tf                       # Transforms
/tf_static                # Static transforms
```

## Network Requirements

- **Same network**: Both machines must be on the same LAN
- **Multicast enabled**: Router must allow multicast (most do)
- **Firewall**: May need to allow UDP ports 7400-7500

## Troubleshooting

### Can't see topics from remote machine

```bash
# On remote machine, check domain ID
echo $ROS_DOMAIN_ID  # Should match docker host

# Check RMW
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp

# List participants
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

### Multicast issues

If on different subnets, add peers in `config/cyclonedds.xml`:

```xml
<Discovery>
    <Peers>
        <Peer address="192.168.1.100"/>  <!-- IP of docker host -->
    </Peers>
</Discovery>
```

### Check DDS discovery

```bash
# On docker host
docker exec -it ros_speckle_bridge_gazebo bash
ros2 topic list

# On remote machine
ros2 topic list

# Both should show the same topics
```

## Performance Tips

- **Bandwidth**: BIM visualization can be large, use filters in `gazebo_params.yaml`
- **Update rate**: Adjust marker publishing rate if network is slow
- **Compression**: Consider using compressed topics for large meshes

## Files

- `config/cyclonedds.xml` - CycloneDDS network configuration
- `config/gazebo_params.yaml` - Bridge parameters
- `config/plugins.yaml` - Plugin configuration
- `.env` - Environment variables (Speckle credentials, ROS domain)

## Advanced: Multi-Machine Setup

To run on different networks (VPN, internet):

1. Use **peer discovery** instead of multicast
2. Configure static peers in `cyclonedds.xml`
3. Set up port forwarding if behind NAT

See [CycloneDDS documentation](https://github.com/eclipse-cyclonedds/cyclonedds) for details.

## See Also

- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst)
- [ROS 2 DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- [Main Plugin Documentation](../../doc/plugins.md)
