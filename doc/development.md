# Development Guide

## Building

### Build All Packages

```bash
colcon build --symlink-install
```

### Build Specific Package

```bash
colcon build --packages-select bim_interfaces
colcon build --packages-select ros_speckle_bridge
```

### Debug Build

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Clean Build

```bash
rm -rf build install log
colcon build --symlink-install
```

## Testing

### Docker-based Testing (Recommended)

The project includes a convenience script to run tests in a Docker container. This ensures a consistent environment and handles dependencies automatically.

```bash
./run_tests.sh
```

This script will:
1.  Build the test container.
2.  Run unit tests.
3.  Run integration tests (mocked).
4.  Run live tests (if `SPECKLE_TOKEN` and `TEST_STREAM_ID` are set).

### Run All Tests (Local)

```bash
colcon test
colcon test-result --verbose
```

### Test Specific Package

```bash
colcon test --packages-select bim_interfaces
colcon test-result --verbose
```

### Python Unit Tests

```bash
cd ros_speckle_bridge
python3 -m pytest tests/
```

## Code Quality

### Linting

```bash
ament_flake8 ros_speckle_bridge
ament_pep257 ros_speckle_bridge
```

### Formatting

```bash
# Python (if black is installed)
black ros_speckle_bridge/ros_speckle_bridge/

# Check without modifying
black --check ros_speckle_bridge/ros_speckle_bridge/
```

## Package Structure

### bim_interfaces (CMake Package)

```
bim_interfaces/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── msg/                    # Message definitions
│   ├── Property.msg
│   ├── BimObject.msg
│   └── BimPartition.msg
└── srv/                    # Service definitions
    └── QueryBim.srv
```

### ros_speckle_bridge (Python Package)

```
ros_speckle_bridge/
├── package.xml                      # Package metadata
├── setup.py                         # Python package setup
├── resource/                        # Package marker
├── config/                          # Configuration files
│   └── params.yaml
├── launch/                          # Launch files
│   └── bridge.launch.py
├── ros_speckle_bridge/              # Python module
│   ├── __init__.py
│   ├── bridge_node.py               # Main ROS node
│   ├── speckle_client.py            # Speckle API client
│   ├── converter.py                 # Data conversion
│   ├── cache_manager.py             # Offline caching
│   └── list_streams.py              # CLI utility
├── tests/                           # Unit tests
│   └── mock_data/
│       └── sample_response.json
├── Dockerfile                       # Multi-stage build
├── docker-compose.yml               # Container orchestration
└── .env.template                    # Environment template
```

## Adding New Features

### Add New Message Type

1. Create message file in `bim_interfaces/msg/`:
   ```bash
   touch bim_interfaces/msg/NewMessage.msg
   ```

2. Edit `bim_interfaces/CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/Property.msg"
     "msg/BimObject.msg"
     "msg/BimPartition.msg"
     "msg/NewMessage.msg"  # Add this line
     # ...
   )
   ```

3. Rebuild:
   ```bash
   colcon build --packages-select bim_interfaces
   ```

### Add New Service

1. Create service file in `bim_interfaces/srv/`:
   ```bash
   touch bim_interfaces/srv/NewService.srv
   ```

2. Add to `CMakeLists.txt` similar to messages

3. Rebuild package

### Extend Bridge Node

1. Add new method to `bridge_node.py`:
   ```python
   def new_feature(self):
       """Implementation"""
       pass
   ```

2. Create new publisher/subscriber/service as needed

3. Update `initialize()` or add to constructor

4. Rebuild and test:
   ```bash
   colcon build --packages-select ros_speckle_bridge
   ros2 launch ros_speckle_bridge bridge.launch.py
   ```

## Debugging

### Enable Debug Logging

```bash
ros2 launch ros_speckle_bridge bridge.launch.py --log-level debug
```

### Python Debugger

Add breakpoint in code:
```python
import pdb; pdb.set_trace()
```

Or use VS Code debugger with provided `launch.json`.

### Check Node Status

```bash
ros2 node list
ros2 node info /speckle_bridge
```

### Monitor Performance

```bash
# CPU and memory usage
top -p $(pgrep -f speckle_bridge)

# Message statistics
ros2 topic hz /bim/objects
ros2 topic bw /bim/objects
```

## Contributing

### Code Style

- Follow PEP 8 for Python
- Use type hints where applicable
- Document functions with docstrings
- Keep line length ≤ 100 characters

### Commit Messages

Format: `<type>: <subject>`

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `refactor`: Code restructuring
- `test`: Test additions/changes
- `chore`: Maintenance tasks

Example:
```
feat: add bounding box filter to query service
fix: correct coordinate transformation for rotated objects
docs: update installation guide for Jazzy
```

### Pull Request Process

1. Fork repository
2. Create feature branch
3. Make changes with tests
4. Ensure all tests pass
5. Update documentation
6. Submit pull request

## Module Documentation

### speckle_client.py

**Purpose:** Wrapper around specklepy for API interaction

**Key classes:**
- `SpeckleClient`: Handles authentication and data retrieval

**Methods:**
- `get_stream()`: Fetch stream metadata
- `get_commit()`: Fetch commit metadata
- `receive_objects()`: Download BIM data
- `list_streams()`: List available streams

### converter.py

**Purpose:** Convert Speckle objects to ROS messages

**Key classes:**
- `Converter`: Handles coordinate transformation and message creation

**Methods:**
- `transform_point()`: Apply datum and rotation
- `speckle_to_bim_object()`: Convert to BimObject message
- `create_marker_array()`: Generate visualization markers

### cache_manager.py

**Purpose:** Local caching for offline resilience

**Key classes:**
- `CacheManager`: Manages cache storage and retrieval

**Methods:**
- `save_cache()`: Store Speckle data locally
- `load_cache()`: Retrieve cached data
- `has_cache()`: Check cache existence

### bridge_node.py

**Purpose:** Main ROS 2 node implementation

**Key classes:**
- `SpeckleBridgeNode`: ROS node that publishes BIM data

**Methods:**
- `initialize()`: Setup client and fetch data
- `_process_data()`: Convert and publish BIM objects
- `handle_query()`: Process service requests
