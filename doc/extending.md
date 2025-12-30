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

---

## Creating Output Plugins

The ROS Speckle Bridge supports **output plugins** to extend functionality without adding dependencies to the core package.

### Plugin Architecture Overview

**Why Plugins?**
- Keep core package lightweight (no simulator dependencies)
- Users enable only features they need
- Third parties can create custom integrations
- Clean separation of concerns

**Plugin Types:**
- **Built-in**: Shipped with `ros_speckle_bridge` (e.g., `mesh_exporter`)
- **Example**: Integration demos in `examples/` (e.g., `gazebo_spawner`)
- **Custom**: Your own plugins in separate packages

### Creating a Basic Plugin

All plugins inherit from `OutputPlugin`:

```python
from ros_speckle_bridge.plugins.base import OutputPlugin
from bim_interfaces.msg import BimObject
from typing import List, Dict, Any

class MyCustomPlugin(OutputPlugin):
    """Your plugin description"""
    
    def initialize(self, config: Dict[str, Any]) -> bool:
        """
        Called once at startup.
        
        Args:
            config: Plugin configuration from params.yaml
            
        Returns:
            True if successful, False to disable plugin
        """
        try:
            # Initialize your resources
            self.server_url = config.get('server_url', 'localhost')
            self.port = config.get('port', 8080)
            
            self.node.get_logger().info(
                f"MyPlugin initialized: {self.server_url}:{self.port}"
            )
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Init failed: {e}")
            return False
    
    def on_objects_received(self, objects: List[BimObject]) -> None:
        """
        Called when BIM objects are received.
        
        This is where your main logic goes.
        
        Args:
            objects: List of BIM objects from the bridge
        """
        self.node.get_logger().info(f"Processing {len(objects)} objects")
        
        for obj in objects:
            self.process_object(obj)
    
    def shutdown(self) -> None:
        """Called when node is shutting down"""
        self.node.get_logger().info("MyPlugin shut down")
```

### Configuring Plugins

Add to your `params.yaml`:

```yaml
/**:
  ros__parameters:
    plugins:
      # Load from file path (development)
      - type: "/path/to/my_plugin.py:MyCustomPlugin"
        enabled: true
        config:
          server_url: "localhost"
          port: 8080
      
      # Load from Python module (production)
      - type: "my_package.plugins.MyCustomPlugin"
        enabled: true
        config:
          api_key: "secret"
      
      # Built-in plugin (short name)
      - type: "mesh_exporter"
        enabled: true
        config:
          output_path: "/tmp/meshes"
          format: "obj"
```

### Example Plugins

#### Database Logger
```python
import psycopg2
from ros_speckle_bridge.plugins.base import OutputPlugin

class DatabaseLoggerPlugin(OutputPlugin):
    def initialize(self, config):
        self.conn = psycopg2.connect(**config['db_config'])
        self.cursor = self.conn.cursor()
        return True
    
    def on_objects_received(self, objects):
        for obj in objects:
            self.cursor.execute("""
                INSERT INTO bim_objects (id, category, position_x, position_y, position_z)
                VALUES (%s, %s, %s, %s, %s)
                ON CONFLICT (id) DO UPDATE SET ...
            """, (obj.id, obj.category, obj.pose.position.x, ...))
        self.conn.commit()
    
    def shutdown(self):
        self.cursor.close()
        self.conn.close()
```

#### REST API Publisher
```python
from flask import Flask, jsonify
from threading import Thread
from ros_speckle_bridge.plugins.base import OutputPlugin

class RestApiPlugin(OutputPlugin):
    def initialize(self, config):
        self.app = Flask(__name__)
        self.objects = []
        
        @self.app.route('/objects')
        def get_objects():
            return jsonify([{'id': o.id, 'category': o.category} for o in self.objects])
        
        Thread(target=lambda: self.app.run(port=5000), daemon=True).start()
        return True
    
    def on_objects_received(self, objects):
        self.objects = objects
```

### Complete Example: Gazebo Integration

See [examples/gazebo_integration](../examples/gazebo_integration/README.md) for a full working example that:
- Exports meshes to Collada format
- Spawns BIM objects in Gazebo Harmonic
- Handles physics and collision geometry

### Plugin Best Practices

**1. Error Handling**
```python
def on_objects_received(self, objects):
    for obj in objects:
        try:
            self.process_object(obj)
        except Exception as e:
            self.node.get_logger().error(f"Failed to process {obj.id}: {e}")
            continue  # Don't stop on single failure
```

**2. Lazy Imports**
```python
def initialize(self, config):
    try:
        import tensorflow as tf  # Import only when plugin is enabled
        self.model = tf.load_model(config['model_path'])
        return True
    except ImportError:
        self.node.get_logger().error("TensorFlow not installed")
        return False
```

**3. Configuration Validation**
```python
def initialize(self, config):
    required = ['server_url', 'api_key']
    for key in required:
        if key not in config:
            self.node.get_logger().error(f"Missing config: {key}")
            return False
    return True
```

### Plugin Testing

```python
import pytest
from my_plugin import MyCustomPlugin
from unittest.mock import Mock

def test_plugin_initialization():
    mock_node = Mock()
    plugin = MyCustomPlugin(mock_node)
    
    assert plugin.initialize({'server_url': 'localhost'}) == True

def test_object_processing():
    plugin = MyCustomPlugin(Mock())
    plugin.initialize({})
    
    mock_obj = Mock(id='test', category='Wall')
    plugin.on_objects_received([mock_obj])
    # Assert expected behavior
```

### Next Steps

- Review [examples/gazebo_integration](../examples/gazebo_integration/) for complete example
- Check built-in [mesh_exporter.py](../ros_speckle_bridge/ros_speckle_bridge/plugins/mesh_exporter.py)
- See [OutputPlugin base class](../ros_speckle_bridge/ros_speckle_bridge/plugins/base.py)

