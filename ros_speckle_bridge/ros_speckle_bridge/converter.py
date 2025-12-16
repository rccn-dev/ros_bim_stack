#!/usr/bin/env python3
"""
Converter Module
Converts Speckle BIM objects to ROS messages and visualization markers
Handles coordinate frame transformations (BIM Y-up to ROS Z-up)
"""

import logging
import numpy as np
from typing import List, Dict, Any, Optional, Tuple
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from bim_interfaces.msg import BimObject, Property


class Converter:
    """Converts Speckle objects to ROS messages with coordinate transformations"""

    # Category colors for visualization (RGB 0-1 scale)
    CATEGORY_COLORS = {
        "Walls": (0.8, 0.8, 0.8, 0.7),      # Light gray
        "Floors": (0.6, 0.4, 0.2, 0.7),     # Brown
        "Roofs": (0.5, 0.2, 0.2, 0.7),      # Dark red
        "Doors": (0.4, 0.6, 0.8, 0.7),      # Blue
        "Windows": (0.6, 0.8, 0.9, 0.5),    # Light blue
        "Columns": (0.7, 0.7, 0.7, 0.8),    # Gray
        "Beams": (0.6, 0.6, 0.6, 0.8),      # Gray
        "Stairs": (0.5, 0.5, 0.5, 0.7),     # Medium gray
        "Furniture": (0.8, 0.6, 0.4, 0.6),  # Beige
        "default": (0.5, 0.5, 0.5, 0.6)     # Default gray
    }

    def __init__(self, frame_id: str = "map", datum: Optional[List[float]] = None):
        """
        Initialize converter
        
        Args:
            frame_id: ROS frame ID for generated messages
            datum: [x, y, z] offset to subtract from BIM coordinates
        """
        self.logger = logging.getLogger(__name__)
        self.frame_id = frame_id
        self.datum = np.array(datum if datum else [0.0, 0.0, 0.0])
        
        # Rotation matrix: BIM Y-up to ROS Z-up (90° rotation around X-axis)
        # X: right (same), Y: up -> forward, Z: forward -> up
        self.rotation_matrix = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0,  1.0],
            [0.0, -1.0,  0.0]
        ])
        
        self.logger.info(f"Converter initialized: frame={frame_id}, datum={self.datum}")

    def transform_point(self, bim_point: List[float]) -> Tuple[float, float, float]:
        """
        Transform BIM coordinates to ROS coordinates
        
        Args:
            bim_point: [x, y, z] in BIM frame (Y-up)
            
        Returns:
            (x, y, z) in ROS frame (Z-up)
        """
        # Apply datum offset
        point = np.array(bim_point) - self.datum
        
        # Apply rotation (Y-up to Z-up)
        transformed = self.rotation_matrix @ point
        
        return tuple(transformed)

    def extract_properties(self, speckle_obj: Any) -> List[Property]:
        """
        Extract BIM properties from Speckle object
        
        Args:
            speckle_obj: Speckle object or dictionary
            
        Returns:
            List of Property messages
        """
        properties = []
        
        # Get object as dictionary
        if hasattr(speckle_obj, '__dict__'):
            obj_dict = speckle_obj.__dict__
        elif isinstance(speckle_obj, dict):
            obj_dict = speckle_obj
        else:
            return properties
        
        # Extract relevant properties
        exclude_keys = {'id', 'speckle_type', 'applicationId', 'totalChildrenCount', 
                       'units', 'bbox', 'displayValue', 'renderMaterial', 
                       'elements', '@', 'children'}
        
        for key, value in obj_dict.items():
            if key.startswith('_') or key in exclude_keys:
                continue
            
            # Handle nested parameter objects
            if hasattr(value, '__dict__') and hasattr(value, 'name'):
                prop = Property()
                prop.key = getattr(value, 'name', key)
                prop.value = str(getattr(value, 'value', value))
                prop.type = getattr(value, 'units', 'Text')
                properties.append(prop)
            elif isinstance(value, (str, int, float, bool)):
                prop = Property()
                prop.key = key
                prop.value = str(value)
                prop.type = self._infer_type(value)
                properties.append(prop)
        
        return properties

    def _infer_type(self, value: Any) -> str:
        """Infer property type from value"""
        if isinstance(value, bool):
            return "Boolean"
        elif isinstance(value, int):
            return "Integer"
        elif isinstance(value, float):
            return "Number"
        else:
            return "Text"

    def speckle_to_bim_object(self, speckle_obj: Any, header: Header) -> Optional[BimObject]:
        """
        Convert Speckle object to BimObject message
        
        Args:
            speckle_obj: Speckle object
            header: ROS message header
            
        Returns:
            BimObject message or None if conversion fails
        """
        try:
            msg = BimObject()
            msg.header = header
            
            # Extract identity
            if hasattr(speckle_obj, 'id'):
                msg.uuid = str(speckle_obj.id)
            elif isinstance(speckle_obj, dict) and 'id' in speckle_obj:
                msg.uuid = str(speckle_obj['id'])
            else:
                return None
            
            # Extract category
            msg.category = self._extract_category(speckle_obj)
            
            # Extract element type
            msg.element_type = self._extract_element_type(speckle_obj)
            
            # Extract and transform geometry
            msg.pose, msg.scale = self._extract_geometry(speckle_obj)
            
            # Extract properties
            msg.parameters = self.extract_properties(speckle_obj)
            
            return msg
            
        except Exception as e:
            self.logger.error(f"Failed to convert Speckle object: {e}")
            return None

    def _extract_category(self, obj: Any) -> str:
        """Extract category from Speckle object"""
        if hasattr(obj, 'category'):
            return str(obj.category)
        elif hasattr(obj, 'speckle_type'):
            # Parse from speckle_type (e.g., "Objects.BuiltElements.Wall")
            parts = str(obj.speckle_type).split('.')
            if len(parts) > 2:
                return parts[-1]
        elif isinstance(obj, dict):
            if 'category' in obj:
                return str(obj['category'])
            elif 'speckle_type' in obj:
                parts = str(obj['speckle_type']).split('.')
                if len(parts) > 2:
                    return parts[-1]
        return "Unknown"

    def _extract_element_type(self, obj: Any) -> str:
        """Extract element type/family from Speckle object"""
        if hasattr(obj, 'type'):
            return str(obj.type)
        elif hasattr(obj, 'family'):
            return str(obj.family)
        elif isinstance(obj, dict):
            if 'type' in obj:
                return str(obj['type'])
            elif 'family' in obj:
                return str(obj['family'])
        return "Generic"

    def _extract_geometry(self, obj: Any) -> Tuple[Pose, Vector3]:
        """
        Extract pose and bounding box from Speckle object
        
        Returns:
            (Pose, Vector3) for position/orientation and scale
        """
        pose = Pose()
        scale = Vector3()
        
        # Default values
        scale.x = 1.0
        scale.y = 1.0
        scale.z = 1.0
        
        # Try to extract bounding box
        bbox = None
        if hasattr(obj, 'bbox'):
            bbox = obj.bbox
        elif isinstance(obj, dict) and 'bbox' in obj:
            bbox = obj['bbox']
        
        if bbox:
            # Calculate center and dimensions from bbox
            if hasattr(bbox, 'min') and hasattr(bbox, 'max'):
                min_pt = [bbox.min.x, bbox.min.y, bbox.min.z]
                max_pt = [bbox.max.x, bbox.max.y, bbox.max.z]
            elif isinstance(bbox, dict):
                min_pt = [bbox.get('min', {}).get('x', 0),
                         bbox.get('min', {}).get('y', 0),
                         bbox.get('min', {}).get('z', 0)]
                max_pt = [bbox.get('max', {}).get('x', 0),
                         bbox.get('max', {}).get('y', 0),
                         bbox.get('max', {}).get('z', 0)]
            else:
                return pose, scale
            
            # Calculate center in BIM coordinates
            center_bim = [(min_pt[i] + max_pt[i]) / 2.0 for i in range(3)]
            
            # Transform to ROS coordinates
            center_ros = self.transform_point(center_bim)
            pose.position.x = center_ros[0]
            pose.position.y = center_ros[1]
            pose.position.z = center_ros[2]
            
            # Calculate dimensions (in BIM frame, then rotate)
            dims_bim = [max_pt[i] - min_pt[i] for i in range(3)]
            dims_ros = self.rotation_matrix @ np.array(dims_bim)
            scale.x = abs(dims_ros[0])
            scale.y = abs(dims_ros[1])
            scale.z = abs(dims_ros[2])
        
        # Identity orientation (no rotation)
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        return pose, scale

    def create_marker(self, bim_obj: BimObject, marker_id: int) -> Marker:
        """
        Create visualization marker from BimObject
        
        Args:
            bim_obj: BimObject message
            marker_id: Unique marker ID
            
        Returns:
            Marker message for RViz
        """
        marker = Marker()
        marker.header = bim_obj.header
        marker.ns = bim_obj.category
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set pose and scale
        marker.pose = bim_obj.pose
        marker.scale = bim_obj.scale
        
        # Set color based on category
        color = self.CATEGORY_COLORS.get(bim_obj.category, 
                                         self.CATEGORY_COLORS["default"])
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        
        # Marker lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker

    def create_marker_array(self, bim_objects: List[BimObject]) -> MarkerArray:
        """
        Create MarkerArray from list of BimObjects
        
        Args:
            bim_objects: List of BimObject messages
            
        Returns:
            MarkerArray for visualization
        """
        marker_array = MarkerArray()
        
        for idx, bim_obj in enumerate(bim_objects):
            marker = self.create_marker(bim_obj, idx)
            marker_array.markers.append(marker)
        
        return marker_array
