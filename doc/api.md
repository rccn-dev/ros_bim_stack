# API Reference

## Topics

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/bim/objects` | `bim_interfaces/BimObject` | Individual BIM elements with geometry and metadata |
| `/bim/visualization` | `visualization_msgs/MarkerArray` | Visualization markers for RViz |

### Message Definitions

#### BimObject.msg

```
std_msgs/Header header
string uuid                    # Unique identifier (Speckle object ID)
string category                # BIM category (e.g., "Walls", "Floors")
string element_type            # Element type (e.g., "Basic Wall: Generic 200mm")
geometry_msgs/Pose pose        # Position and orientation in ROS frame
geometry_msgs/Vector3 scale    # Bounding box dimensions [x, y, z]
Property[] parameters          # Array of BIM properties
```

#### Property.msg

```
string key                     # Property name (e.g., "Height", "Fire Rating")
string value                   # Property value as string
string type                    # Data type hint (e.g., "Length", "Text", "Number")
```

#### BimPartition.msg

```
std_msgs/Header header
string uuid                         # Unique identifier
string name                         # Partition name (e.g., "Room 101", "Level 1")
string partition_type               # Type (e.g., "Room", "Level", "Zone")
geometry_msgs/Pose pose             # Center position and orientation
geometry_msgs/Vector3 dimensions    # Bounding box size [x, y, z]
Property[] properties               # Custom properties
string[] contained_object_uuids     # UUIDs of objects within partition
```

## Services

### /bim/query

Query BIM objects by filter criteria.

**Service Type:** `bim_interfaces/QueryBim`

**Request:**
```
string[] categories         # Filter by categories (empty = all)
string[] element_types      # Filter by element types (empty = all)
float64[3] bbox_min         # Bounding box minimum [x, y, z] (optional)
float64[3] bbox_max         # Bounding box maximum [x, y, z] (optional)
string[] property_filters   # Property filters "key:value" format (optional)
```

**Response:**
```
BimObject[] objects         # Array of matching BIM objects
bool success                # Query success status
string message              # Error or info message
```

**Example:**
```bash
ros2 service call /bim/query bim_interfaces/srv/QueryBim \
  "{categories: ['Walls', 'Floors']}"
```

## TF Frames

### Static Transforms

| Parent Frame | Child Frame | Description |
|--------------|-------------|-------------|
| `map` | `bim_origin` | BIM reference frame with datum offset |

Transform parameters:
- **Translation:** `[datum.x, datum.y, datum.z]` from config
- **Rotation:** Identity quaternion `[0, 0, 0, 1]`

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | `https://app.speckle.systems` | Speckle server URL |
| `stream_id` | string | `""` | Target stream ID (required) |
| `commit_id` | string | `"latest"` | Commit version or "latest" |
| `datum` | double[3] | `[0.0, 0.0, 0.0]` | Coordinate offset [x, y, z] |
| `frame_id` | string | `"map"` | ROS base frame |
| `bim_frame_id` | string | `"bim_origin"` | BIM reference frame |
| `filters.allow` | string[] | `[]` | Allowed categories (empty = all) |
| `filters.deny` | string[] | `[]` | Denied categories |

## Coordinate Systems

### BIM Coordinate Frame (Input)

Standard BIM convention with Y-axis up:
- **X-axis:** Right
- **Y-axis:** Up
- **Z-axis:** Forward

### ROS Coordinate Frame (Output)

Standard ROS convention with Z-axis up:
- **X-axis:** Right (unchanged)
- **Y-axis:** Forward (was Z)
- **Z-axis:** Up (was Y)

### Transformation Pipeline

1. **Datum offset:** Subtract `datum` from BIM coordinates
2. **Rotation:** Apply 90° rotation around X-axis (Y-up → Z-up)
3. **Output:** Coordinates in ROS `map` frame

Rotation matrix:
```
[1   0   0]
[0   0   1]
[0  -1   0]
```

This transformation is handled automatically by the `converter.py` module.
