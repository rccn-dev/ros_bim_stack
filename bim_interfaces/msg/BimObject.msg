# BimObject.msg
# Represents a single BIM element with its geometry and metadata

std_msgs/Header header

# Identity
string uuid                         # Speckle/BIM unique identifier
string category                     # BIM category (e.g., "Walls", "Floors", "Doors")
string element_type                 # Specific type (e.g., "Basic Wall: Generic 200mm")

# Spatial Transform
geometry_msgs/Pose pose             # Position and orientation in ROS frame
geometry_msgs/Vector3 scale         # Bounding box dimensions (x, y, z)
float64[] geometry                  # Flattened vertex list [x1, y1, z1, ...]

# Metadata
Property[] parameters               # All BIM properties/parameters
