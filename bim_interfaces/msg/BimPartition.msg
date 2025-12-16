# BimPartition.msg
# Represents a spatial partition or zone within the BIM model (e.g., rooms, spaces, levels)

std_msgs/Header header

string uuid                         # Unique identifier
string name                         # Partition name (e.g., "Room 101", "Level 1")
string partition_type               # Type (e.g., "Room", "Level", "Zone")

# Spatial definition (simplified as bounding volume)
geometry_msgs/Pose pose             # Center position and orientation
geometry_msgs/Vector3 dimensions    # Bounding box size (x, y, z)

# Associated data
Property[] properties               # Custom properties
string[] contained_object_uuids     # UUIDs of objects within this partition
