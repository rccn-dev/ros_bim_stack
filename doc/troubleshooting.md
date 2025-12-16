# Troubleshooting Guide

## Authentication Issues

### SPECKLE_TOKEN not set

**Symptom:**
```
RuntimeError: SPECKLE_TOKEN environment variable not set
```

**Solution:**
```bash
export SPECKLE_TOKEN="your_token_here"
```

For Docker:
```bash
# Edit .env file
echo "SPECKLE_TOKEN=your_token_here" > .env
docker compose up
```

### Invalid token

**Symptom:**
```
Failed to authenticate with Speckle: Unauthorized
```

**Solution:**
1. Generate new token at https://app.speckle.systems/profile
2. Verify token has necessary permissions
3. Update environment variable

## Configuration Issues

### Stream ID not set

**Symptom:**
```
FATAL: Parameter 'stream_id' is required but not set!
```

**Solution:**
```bash
# List available streams
ros2 run ros_speckle_bridge list_streams

# Edit config
nano ros_speckle_bridge/config/params.yaml
# (If file doesn't exist, copy from params.example.yaml)
# Set stream_id: "your_stream_id_here"
```

### Invalid stream ID

**Symptom:**
```
Failed to fetch stream: Stream not found
```

**Solution:**
1. Verify stream ID from Speckle web interface
2. Check token has access to this stream
3. Confirm stream exists and is not deleted

## Offline/Network Issues

### No cache and offline

**Symptom:**
```
FATAL: No cache found and unable to connect to Speckle
```

**Solution:**
1. Connect to internet for initial data fetch
2. Or manually copy cache from another machine to `~/.ros/speckle_cache/`

### Cache corrupted

**Symptom:**
```
Failed to load cache: JSON decode error
```

**Solution:**
```bash
# Clear cache and re-fetch
rm -rf ~/.ros/speckle_cache/*
# Ensure internet connection
ros2 launch ros_speckle_bridge bridge.launch.py
```

## Visualization Issues

### Empty RViz markers

**Symptom:**
No markers visible in RViz

**Checklist:**
1. Verify Fixed Frame is set to `map`
2. Check topic subscription:
   ```bash
   ros2 topic echo /bim/visualization --once
   ```
3. Verify objects pass category filters:
   ```bash
   ros2 topic echo /bim/objects --once
   ```
4. Check marker scale (might be too small)

### Wrong coordinate frame

**Symptom:**
Objects appear far from origin or in wrong orientation

**Solution:**
1. Verify `datum` parameter in `params.yaml`
2. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```
3. Adjust datum offset to align BIM and ROS origins

## Docker Issues

### Cannot connect to Docker daemon

**Symptom:**
```
Cannot connect to the Docker daemon
```

**Solution for macOS/Colima:**
```bash
colima status
colima start --cpu 4 --memory 8 --disk 100
```

### Container exits immediately

**Symptom:**
Container starts then stops

**Solution:**
```bash
# View logs
docker compose logs

# Common causes:
# - Missing SPECKLE_TOKEN
# - Invalid stream_id
# - Network connectivity
```

### Network mode host issues

**Symptom:**
ROS topics not visible across containers

**Solution:**
```yaml
# docker-compose.yml
services:
  speckle_bridge:
    network_mode: host  # Required for ROS DDS discovery
```

For non-Linux systems, use `ROS_DOMAIN_ID` instead.

## Build Issues

### Message generation fails

**Symptom:**
```
CMake Error: Cannot find source file msg/BimObject.msg
```

**Solution:**
```bash
# Verify file exists
ls bim_interfaces/msg/

# Clean and rebuild
colcon build --packages-select bim_interfaces --cmake-clean-cache
```

### Python import errors

**Symptom:**
```
ModuleNotFoundError: No module named 'bim_interfaces'
```

**Solution:**
```bash
# Source workspace
source install/setup.bash

# Verify package built
ros2 pkg list | grep bim_interfaces

# Rebuild if necessary
colcon build --packages-select bim_interfaces
```

### Missing dependencies

**Symptom:**
```
ModuleNotFoundError: No module named 'specklepy'
```

**Solution:**
```bash
pip3 install specklepy numpy tabulate
```

## Runtime Issues

### Node crashes on startup

**Symptom:**
Node exits with traceback

**Solution:**
```bash
# Run with debug logging
ros2 launch ros_speckle_bridge bridge.launch.py --log-level debug

# Check for:
# - Missing configuration
# - Network issues
# - Invalid data format
```

### High memory usage

**Symptom:**
Process uses excessive RAM

**Cause:**
Large BIM models with many objects

**Solution:**
1. Use category filters to limit objects:
   ```yaml
   filters:
     allow: ["Walls", "Floors"]  # Only essential categories
   ```
2. Increase available memory
3. Process in batches if possible

### Slow performance

**Symptom:**
Long startup time or lag

**Solution:**
1. Ensure cache is being used (check logs)
2. Reduce number of published objects with filters
3. Verify network latency to Speckle server
4. Use local Speckle server if available

## Data Issues

### Objects missing properties

**Symptom:**
`parameters` array is empty

**Cause:**
Properties not exposed in Speckle export

**Solution:**
1. Verify properties exist in source BIM model
2. Check Speckle export settings
3. Ensure converter extracts properties correctly

### Incorrect geometry

**Symptom:**
Object positions or sizes wrong

**Checklist:**
1. Verify coordinate transformation (Y-up to Z-up)
2. Check datum offset applied correctly
3. Verify bounding box calculation
4. Confirm units match (meters expected)

### Category filtering not working

**Symptom:**
Unwanted categories still published

**Solution:**
```bash
# Check current filters
ros2 param get /speckle_bridge filters.allow
ros2 param get /speckle_bridge filters.deny

# Verify case sensitivity
# Categories must match exactly: "Walls" not "walls"
```

## Service Issues

### Query service not responding

**Symptom:**
```
Service not available
```

**Solution:**
```bash
# Check service exists
ros2 service list | grep query

# Verify node is running
ros2 node list | grep speckle_bridge

# Check service type
ros2 service type /bim/query
```

### Empty query results

**Symptom:**
Service returns 0 objects

**Solution:**
1. Verify objects exist:
   ```bash
   ros2 topic echo /bim/objects --once
   ```
2. Check query parameters match object categories
3. Verify case sensitivity in category names

## Diagnostic Commands

### Check node status
```bash
ros2 node list
ros2 node info /speckle_bridge
```

### Check topics
```bash
ros2 topic list
ros2 topic info /bim/objects
ros2 topic hz /bim/objects
```

### Check services
```bash
ros2 service list
ros2 service type /bim/query
```

### Check parameters
```bash
ros2 param list /speckle_bridge
ros2 param get /speckle_bridge stream_id
```

### Check TF frames
```bash
ros2 run tf2_ros tf2_echo map bim_origin
ros2 run tf2_tools view_frames
```

### Check logs
```bash
# ROS logs
ros2 log level /speckle_bridge debug

# System logs (if using systemd)
journalctl -u ros-speckle-bridge -f

# Docker logs
docker compose logs -f
```

## Getting Help

If issues persist:

1. Check logs with `--log-level debug`
2. Verify configuration matches documentation
3. Test with minimal example (single stream, no filters)
4. Review recent changes to configuration or code
5. Consult Speckle documentation for API-specific issues
