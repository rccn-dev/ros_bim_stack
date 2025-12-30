"""
Base Plugin Interface
All output plugins must inherit from OutputPlugin
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional
from rclpy.node import Node
from bim_interfaces.msg import BimObject


class OutputPlugin(ABC):
    """
    Base class for output plugins.
    
    Plugins extend the bridge's functionality without adding dependencies
    to the core package. Examples: Gazebo spawner, Isaac Sim integration,
    mesh export, database logging.
    """
    
    def __init__(self, node: Node):
        """
        Initialize the plugin with a reference to the ROS 2 node.
        
        Args:
            node: The parent SpeckleBridgeNode instance
        """
        self.node = node
        self._initialized = False
    
    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> bool:
        """
        Called once at startup to initialize the plugin.
        
        Args:
            config: Plugin-specific configuration dictionary
            
        Returns:
            True if initialization successful, False otherwise
        """
        pass
    
    @abstractmethod
    def on_objects_received(self, objects: List[BimObject]) -> None:
        """
        Called whenever new BIM objects are received and processed.
        
        This is the main callback where plugins perform their work.
        
        Args:
            objects: List of BIM objects from the bridge
        """
        pass
    
    def on_objects_updated(self, objects: List[BimObject]) -> None:
        """
        Called when existing BIM objects are updated (optional).
        
        Default implementation calls on_objects_received().
        Override if you need different behavior for updates vs new objects.
        
        Args:
            objects: Updated BIM objects
        """
        self.on_objects_received(objects)
    
    def shutdown(self) -> None:
        """
        Called when the node is shutting down.
        
        Override to clean up resources (close connections, files, etc.)
        Default implementation does nothing.
        """
        pass
    
    def is_initialized(self) -> bool:
        """Check if plugin was successfully initialized"""
        return self._initialized
    
    @classmethod
    def get_plugin_name(cls) -> str:
        """
        Return the plugin's name for logging/identification.
        
        Default returns class name. Override for custom names.
        """
        return cls.__name__
