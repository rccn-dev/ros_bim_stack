# Extending the Stack

## Adding New BIM Providers

The `bim_interfaces` package is vendor-agnostic. To add support for other BIM platforms:

### 1. Create New Bridge Package

```bash
cd ~/ros2_ws/src/ros_bim_stack
ros2 pkg create ros_revit_bridge \
  --build-type ament_python \
  --dependencies rclpy bim_interfaces geometry_msgs visualization_msgs tf2_ros
```

### 2. Package Structure

```
ros_revit_bridge/
├── package.xml
├── setup.py
├── config/
│   └── params.yaml
├── launch/
│   └── bridge.launch.py
└── ros_revit_bridge/
    ├── __init__.py
    ├── bridge_node.py
    ├── revit_client.py      # Provider-specific API client
    ├── converter.py          # Convert to BimObject messages
    └── cache_manager.py      # Optional: reuse from ros_speckle_bridge
```

### 3. Implement Converter

Key requirement: Convert provider data to `bim_interfaces/BimObject`:

```python
from bim_interfaces.msg import BimObject, Property
from geometry_msgs.msg import Pose, Vector3

def revit_to_bim_object(revit_element, header):
    """Convert Revit element to BimObject message"""
    msg = BimObject()
    msg.header = header
    
    # Map provider-specific fields
    msg.uuid = str(revit_element.Id)
    msg.category = revit_element.Category.Name
    msg.element_type = revit_element.get_Parameter("Type").AsValueString()
    
    # Extract geometry
    msg.pose = extract_pose(revit_element)
    msg.scale = extract_scale(revit_element)
    
    # Extract parameters
    msg.parameters = extract_parameters(revit_element)
    
    return msg
```

### 4. Follow Same Patterns

- Use caching for offline resilience
- Apply coordinate transformations (provider frame → ROS frame)
- Support category filtering
- Publish to same topics: `/bim/objects`, `/bim/visualization`
- Implement same service: `/bim/query`

### 5. Update Dependencies

`package.xml`:
```xml
<depend>bim_interfaces</depend>
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>visualization_msgs</depend>
```

`setup.py`:
```python
install_requires=[
    'setuptools',
    'revitapi',  # Provider-specific library
]
```

## Extending BIM Interfaces

### Add New Message Fields

Edit `bim_interfaces/msg/BimObject.msg`:

```diff
 std_msgs/Header header
 string uuid
 string category
 string element_type
 geometry_msgs/Pose pose
 geometry_msgs/Vector3 scale
 Property[] parameters
+string material_name
+float64 cost_estimate
```

Rebuild:
```bash
colcon build --packages-select bim_interfaces
```

Update all bridge packages to populate new fields.

### Add New Message Type

Create `bim_interfaces/msg/BimSystem.msg`:

```
std_msgs/Header header
string uuid
string system_name
string system_type
string[] component_uuids
Property[] properties
```

Add to `CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Property.msg"
  "msg/BimObject.msg"
  "msg/BimPartition.msg"
  "msg/BimSystem.msg"  # New message
  "srv/QueryBim.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
```

### Add New Service

Create `bim_interfaces/srv/GetObjectByUuid.srv`:

```
# Request
string uuid

---

# Response
BimObject object
bool success
string message
```

Add to `CMakeLists.txt` and implement in bridge node.

## Custom Converters

### Geometry Conversion

Override `_extract_geometry()` for custom geometry:

```python
class CustomConverter(Converter):
    def _extract_geometry(self, obj):
        """Custom geometry extraction"""
        pose = Pose()
        scale = Vector3()
        
        # Custom logic for specific BIM software
        if hasattr(obj, 'Location'):
            point = obj.Location.Point
            pose.position.x = point.X
            pose.position.y = point.Y
            pose.position.z = point.Z
        
        # Custom bounding box calculation
        bbox = obj.get_BoundingBox(None)
        if bbox:
            scale.x = bbox.Max.X - bbox.Min.X
            scale.y = bbox.Max.Y - bbox.Min.Y
            scale.z = bbox.Max.Z - bbox.Min.Z
        
        return pose, scale
```

### Property Mapping

Map provider-specific properties to standard format:

```python
def extract_properties(self, element):
    """Map Revit parameters to Property messages"""
    properties = []
    
    # Standard mappings
    property_map = {
        'Mark': 'Identifier',
        'Comments': 'Description',
        'Fire Rating': 'FireRating',
    }
    
    for revit_param, std_name in property_map.items():
        param = element.LookupParameter(revit_param)
        if param and param.HasValue:
            prop = Property()
            prop.key = std_name
            prop.value = str(param.AsValueString())
            prop.type = self._map_parameter_type(param.StorageType)
            properties.append(prop)
    
    return properties
```

## Plugin System

### Architecture

For multi-provider support in a single node:

```python
class BridgeFactory:
    """Factory for provider-specific bridges"""
    
    @staticmethod
    def create(provider_type, config):
        if provider_type == 'speckle':
            from .speckle_bridge import SpeckleBridge
            return SpeckleBridge(config)
        elif provider_type == 'revit':
            from .revit_bridge import RevitBridge
            return RevitBridge(config)
        else:
            raise ValueError(f"Unknown provider: {provider_type}")
```

### Configuration

`params.yaml`:
```yaml
/**:
  ros__parameters:
    provider: "speckle"  # or "revit", "ifc", etc.
    
    speckle:
      host: "https://app.speckle.systems"
      stream_id: "..."
    
    revit:
      server: "localhost"
      project_path: "/path/to/project.rvt"
```

## Coordinate Transformations

### Custom Frame Transforms

For non-standard coordinate systems:

```python
class CustomConverter(Converter):
    def __init__(self, frame_id="map", datum=None, rotation_matrix=None):
        super().__init__(frame_id, datum)
        
        # Override rotation for different coordinate convention
        if rotation_matrix is not None:
            self.rotation_matrix = np.array(rotation_matrix)
```

Example for Z-down to Z-up:
```python
rotation_matrix = [
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0]
]
converter = CustomConverter(rotation_matrix=rotation_matrix)
```

## Visualization Extensions

### Custom Markers

Override `create_marker()` for custom visualization:

```python
def create_marker(self, bim_obj, marker_id):
    """Create custom marker based on object type"""
    marker = Marker()
    marker.header = bim_obj.header
    marker.id = marker_id
    
    # Different shapes for different categories
    if bim_obj.category == "Walls":
        marker.type = Marker.CUBE
    elif bim_obj.category == "Columns":
        marker.type = Marker.CYLINDER
    elif bim_obj.category == "Windows":
        marker.type = Marker.CUBE
        marker.color.a = 0.3  # Transparent
    
    marker.pose = bim_obj.pose
    marker.scale = bim_obj.scale
    
    return marker
```

### Mesh Visualization

For detailed geometry:

```python
from visualization_msgs.msg import Marker

def create_mesh_marker(self, bim_obj, mesh_data):
    """Create mesh marker from triangulated geometry"""
    marker = Marker()
    marker.type = Marker.TRIANGLE_LIST
    marker.header = bim_obj.header
    
    # Add vertices
    for vertex in mesh_data.vertices:
        point = Point()
        point.x = vertex[0]
        point.y = vertex[1]
        point.z = vertex[2]
        marker.points.append(point)
    
    # Add colors
    marker.color.a = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    
    return marker
```

## Testing Extensions

### Mock BIM Data

Create test fixtures for new providers:

```python
# tests/fixtures/revit_data.py
def create_mock_revit_element():
    """Create mock Revit element for testing"""
    element = Mock()
    element.Id = "12345"
    element.Category.Name = "Walls"
    element.get_Parameter.return_value.AsValueString.return_value = "Basic Wall"
    return element
```

### Integration Tests

Test provider-specific conversion:

```python
import pytest
from ros_revit_bridge.converter import RevitConverter

def test_revit_to_bim_conversion():
    converter = RevitConverter()
    revit_element = create_mock_revit_element()
    
    bim_obj = converter.revit_to_bim_object(revit_element, header)
    
    assert bim_obj.uuid == "12345"
    assert bim_obj.category == "Walls"
    assert bim_obj.element_type == "Basic Wall"
```
