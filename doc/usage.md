# Usage Guide

## Launch Bridge Node

```bash
ros2 launch ros_speckle_bridge bridge.launch.py
```

With custom config:
```bash
ros2 launch ros_speckle_bridge bridge.launch.py config_file:=/path/to/custom_params.yaml
```

## Visualization in RViz

```bash
rviz2
```

Configuration:
1. Set **Fixed Frame** to `map`
2. Add **MarkerArray** display
3. Set topic to `/bim/visualization`
4. Adjust marker display properties as needed

## Query BIM Objects

### Service Call Examples

Query all walls:
```bash
ros2 service call /bim/query bim_interfaces/srv/QueryBim \
  "{categories: ['Walls']}"
```

Query multiple categories:
```bash
ros2 service call /bim/query bim_interfaces/srv/QueryBim \
  "{categories: ['Walls', 'Floors', 'Doors']}"
```

Query by element type:
```bash
ros2 service call /bim/query bim_interfaces/srv/QueryBim \
  "{element_types: ['Basic Wall: Generic 200mm']}"
```

## Monitor Topics

List all BIM topics:
```bash
ros2 topic list | grep bim
```

Echo BIM objects:
```bash
ros2 topic echo /bim/objects
```

Check message rate:
```bash
ros2 topic hz /bim/objects
```

View topic info:
```bash
ros2 topic info /bim/objects
```

## Inspect TF Tree

View transforms:
```bash
ros2 run tf2_tools view_frames
```

Echo specific transform:
```bash
ros2 run tf2_ros tf2_echo map bim_origin
```

## Offline Operation

The bridge caches data at `~/.ros/speckle_cache/`.

**First run (online):**
- Fetches from Speckle API
- Saves to local cache
- Publishes data

**Subsequent runs:**
- Loads from cache immediately
- Attempts background update if online
- Falls back to cache if offline

**No cache and offline:**
- Node exits with error
- Requires internet connection for initial fetch

### Cache Management

View cache location:
```bash
ls -lh ~/.ros/speckle_cache/
```

Clear all cache:
```bash
rm -rf ~/.ros/speckle_cache/*
```

Clear specific stream:
```bash
rm ~/.ros/speckle_cache/<stream_id>_*
```
