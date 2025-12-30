"""
Plugin Loader - Dynamically loads and manages plugins
"""

import importlib
import importlib.util
from typing import List, Dict, Any, Optional, Type
from pathlib import Path
from rclpy.node import Node

from .base import OutputPlugin


class PluginLoader:
    """
    Loads and manages output plugins for the Speckle Bridge.
    
    Supports loading plugins from:
    1. Built-in plugins (short names: 'mesh_exporter', 'gazebo_spawner')
    2. External Python modules (via module path)
    3. Local files (via absolute file path)
    """
    
    # Registry of built-in plugins (short name -> module path)
    BUILTIN_PLUGINS = {
        'mesh_exporter': 'ros_speckle_bridge.plugins.mesh_exporter',
    }
    
    def __init__(self, node: Node):
        """
        Initialize plugin loader.
        
        Args:
            node: Parent ROS 2 node
        """
        self.node = node
        self.plugins: List[OutputPlugin] = []
    
    def load_plugins(self, plugin_configs: List[Dict[str, Any]]) -> List[OutputPlugin]:
        """
        Load all plugins from configuration.
        
        Args:
            plugin_configs: List of plugin configuration dictionaries.
                           Each must have 'type' and 'enabled' keys.
        
        Returns:
            List of successfully loaded and initialized plugins
        """
        self.plugins = []
        
        for config in plugin_configs:
            # Handle enabled as bool or string ("true"/"false")
            enabled = config.get('enabled', False)
            if isinstance(enabled, str):
                enabled = enabled.lower() in ('true', '1', 'yes')
            
            if not enabled:
                self.node.get_logger().debug(
                    f"Skipping disabled plugin: {config.get('type', 'unknown')}"
                )
                continue
            
            plugin = self._load_single_plugin(config)
            if plugin:
                self.plugins.append(plugin)
        
        self.node.get_logger().info(
            f"Loaded {len(self.plugins)} plugin(s): "
            f"{[p.get_plugin_name() for p in self.plugins]}"
        )
        
        return self.plugins
    
    def _load_single_plugin(self, config: Dict[str, Any]) -> Optional[OutputPlugin]:
        """
        Load and initialize a single plugin.
        
        Args:
            config: Plugin configuration with 'type' and optional 'config' keys
        
        Returns:
            Initialized plugin instance or None if loading failed
        """
        plugin_type = config.get('type')
        if not plugin_type:
            self.node.get_logger().error("Plugin config missing 'type' field")
            return None
        
        try:
            # Load the plugin class
            plugin_class = self._import_plugin_class(plugin_type)
            
            # Instantiate the plugin
            plugin = plugin_class(self.node)
            
            # Initialize with config
            plugin_config = config.get('config', {})
            if plugin.initialize(plugin_config):
                plugin._initialized = True
                self.node.get_logger().info(
                    f"✓ Loaded plugin: {plugin.get_plugin_name()}"
                )
                return plugin
            else:
                self.node.get_logger().error(
                    f"Plugin initialization failed: {plugin_type}"
                )
                return None
                
        except Exception as e:
            self.node.get_logger().error(
                f"Failed to load plugin '{plugin_type}': {e}"
            )
            return None
    
    def _import_plugin_class(self, plugin_type: str) -> Type[OutputPlugin]:
        """
        Import a plugin class from a module path or file.
        
        Supports three formats:
        1. 'mesh_exporter' - built-in plugin (from BUILTIN_PLUGINS registry)
        2. 'package.module.ClassName' - fully qualified class path
        3. '/path/to/plugin.py:ClassName' - file path with class name
        
        Args:
            plugin_type: Plugin identifier string
            
        Returns:
            Plugin class (subclass of OutputPlugin)
            
        Raises:
            ImportError: If plugin cannot be found or imported
            ValueError: If imported class is not a valid OutputPlugin
        """
        # Case 1: Built-in plugin (short name from registry)
        if plugin_type in self.BUILTIN_PLUGINS:
            module_path = self.BUILTIN_PLUGINS[plugin_type]
            try:
                module = importlib.import_module(module_path)
                # Auto-discover OutputPlugin subclass
                for attr_name in dir(module):
                    attr = getattr(module, attr_name)
                    if (isinstance(attr, type) and 
                        issubclass(attr, OutputPlugin) and 
                        attr is not OutputPlugin):
                        return attr
                raise ImportError(f"No OutputPlugin subclass found in {module_path}")
            except ImportError as e:
                raise ImportError(f"Built-in plugin '{plugin_type}' not found: {e}")
        
        # Case 2: File path with class name ('/path/to/file.py:ClassName')
        elif ':' in plugin_type:
            file_path, class_name = plugin_type.split(':', 1)
            spec = importlib.util.spec_from_file_location("custom_plugin", file_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"Cannot load plugin from file: {file_path}")
            
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            if not hasattr(module, class_name):
                raise ImportError(f"Class '{class_name}' not found in {file_path}")
            
            plugin_class = getattr(module, class_name)
            
        # Case 3: Fully qualified module path ('package.module.ClassName')
        else:
            parts = plugin_type.rsplit('.', 1)
            if len(parts) != 2:
                raise ImportError(f"Invalid plugin type format: {plugin_type}")
            
            module_path, class_name = parts
            module = importlib.import_module(module_path)
            
            if not hasattr(module, class_name):
                raise ImportError(f"Class '{class_name}' not found in {module_path}")
            
            plugin_class = getattr(module, class_name)
        
        # Validate that it's a proper OutputPlugin subclass
        if not issubclass(plugin_class, OutputPlugin):
            raise ValueError(
                f"{plugin_class.__name__} is not a subclass of OutputPlugin"
            )
        
        return plugin_class
    
    def notify_objects_received(self, objects: List) -> None:
        """
        Notify all loaded plugins that new objects were received.
        
        Args:
            objects: List of BimObject messages
        """
        for plugin in self.plugins:
            try:
                plugin.on_objects_received(objects)
            except Exception as e:
                self.node.get_logger().error(
                    f"Plugin {plugin.get_plugin_name()} failed: {e}"
                )
    
    def shutdown_all(self) -> None:
        """Shutdown all loaded plugins"""
        for plugin in self.plugins:
            try:
                plugin.shutdown()
            except Exception as e:
                self.node.get_logger().error(
                    f"Error shutting down plugin {plugin.get_plugin_name()}: {e}"
                )
