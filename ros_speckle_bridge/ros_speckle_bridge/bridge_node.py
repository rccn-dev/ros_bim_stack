#!/usr/bin/env python3
"""
Bridge Node - Main Entry Point
Production-ready ROS 2 node that bridges Speckle BIM data to ROS ecosystem
"""

import os
import sys
import logging
from typing import List, Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster

from bim_interfaces.msg import BimObject
from bim_interfaces.srv import QueryBim, FetchStream

from .speckle_client import SpeckleClient
from .converter import Converter
from .cache_manager import CacheManager
from .plugins.loader import PluginLoader


class SpeckleBridgeNode(Node):
    """ROS 2 node that publishes BIM data from Speckle to ROS topics"""

    def __init__(self):
        super().__init__('speckle_bridge')
        
        # Configure logging
        self._setup_logging()
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize components
        self.cache_manager = CacheManager()
        self.converter = None
        self.speckle_client = None
        self.plugin_loader = PluginLoader(self)
        
        # Publishers
        self.object_pub = self.create_publisher(
            BimObject, 
            '/bim/objects', 
            10
        )
        self.viz_pub = self.create_publisher(
            MarkerArray, 
            '/bim/visualization', 
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Service server
        self.query_service = self.create_service(
            QueryBim,
            '/bim/query',
            self.handle_query
        )
        
        self.fetch_service = self.create_service(
            FetchStream,
            '/bim/fetch',
            self.handle_fetch
        )
        
        # Internal state
        self.bim_objects: List[BimObject] = []
        self.raw_data: Optional[Any] = None
        
        # Initialize
        self.get_logger().info("Speckle Bridge Node starting...")
        self.initialize()

    def _setup_logging(self):
        """Configure Python logging to match ROS 2 logger"""
        logging.basicConfig(
            level=logging.INFO,
            format='[%(name)s] [%(levelname)s] %(message)s'
        )

    def _declare_parameters(self):
        """Declare ROS 2 parameters with defaults"""
        self.declare_parameter('host', 'https://app.speckle.systems')
        self.declare_parameter('stream_id', '')
        self.declare_parameter('commit_id', 'latest')
        self.declare_parameter('datum', [0.0, 0.0, 0.0])
        
        string_array_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter('filters.allow', [], string_array_descriptor)
        self.declare_parameter('filters.deny', [], string_array_descriptor)
        
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('bim_frame_id', 'bim_origin')
        self.declare_parameter('auto_fetch', True)
        
        # Plugin configuration file path
        self.declare_parameter('plugins_file', '')

    def initialize(self):
        """Initialize Speckle client and fetch data"""
        try:
            # Get parameters
            host = self.get_parameter('host').value
            stream_id = self.get_parameter('stream_id').value
            commit_id = self.get_parameter('commit_id').value
            datum = self.get_parameter('datum').value
            frame_id = self.get_parameter('frame_id').value
            bim_frame_id = self.get_parameter('bim_frame_id').value
            
            # Initialize converter
            self.converter = Converter(frame_id=frame_id, datum=datum)
            
            # Publish static transform from map to bim_origin
            self._publish_static_transform(frame_id, bim_frame_id, datum)
            
            # Load plugins
            self._load_plugins()
            
            # Initialize Speckle Client (uses SPECKLE_TOKEN from env)
            try:
                self.speckle_client = SpeckleClient(host=host)
                self.get_logger().info(f"Speckle client initialized for host: {host}")
            except Exception as e:
                self.get_logger().warn(f"Speckle client initialization failed: {e}. Will rely on cache if available.")

            # Try to load from cache first
            cached_data = self.cache_manager.load_cache(stream_id, commit_id) if stream_id else None
            auto_fetch = self.get_parameter('auto_fetch').value
            
            if cached_data:
                self.get_logger().info("Loading BIM data from cache...")
                self.raw_data = cached_data
                self._process_data(cached_data)
                
                # Try to update cache in background if online
                if self.speckle_client and auto_fetch:
                    self.get_logger().info("Online - checking for updates...")
                    self._fetch_and_cache_data(stream_id, commit_id)
            elif stream_id and auto_fetch:
                # No cache - must fetch
                if not self.speckle_client:
                    self.get_logger().fatal("No cache and no Speckle client (check SPECKLE_TOKEN)!")
                    return

                self.get_logger().info(f"Fetching stream {stream_id} from Speckle...")
                self._fetch_and_cache_data(stream_id, commit_id)
            else:
                self.get_logger().info("Auto-fetch disabled or no stream_id. Waiting for service call...")
            
            self.get_logger().info(
                f"Initialized successfully with {len(self.bim_objects)} BIM objects"
            )
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")

    def handle_fetch(self, request: FetchStream.Request, response: FetchStream.Response):
        """Handle request to fetch a specific stream"""
        try:
            stream_id = request.stream_id
            commit_id = request.commit_id
            
            if not stream_id:
                response.success = False
                response.message = "stream_id is required"
                return response

            if not self.speckle_client:
                response.success = False
                response.message = "Speckle client not initialized (check SPECKLE_TOKEN)"
                return response

            self.get_logger().info(f"Manual fetch requested for stream: {stream_id}")
            self._fetch_and_cache_data(stream_id, commit_id)
            
            response.success = True
            response.message = f"Successfully fetched {len(self.bim_objects)} objects"
            response.object_count = len(self.bim_objects)
            
        except Exception as e:
            self.get_logger().error(f"Fetch failed: {e}")
            response.success = False
            response.message = str(e)
            
        return response

    def _fetch_and_cache_data(self, stream_id: str, commit_id: str):
        """Fetch data from Speckle and update cache"""
        try:
            # Fetch from Speckle
            raw_data = self.speckle_client.receive_objects(stream_id, commit_id)
            
            # Save to cache
            self.cache_manager.save_cache(stream_id, commit_id, raw_data)
            
            # Process data
            self.raw_data = raw_data
            self._process_data(raw_data)
            
        except Exception as e:
            self.get_logger().error(f"Failed to fetch data: {e}")
            raise

    def _process_data(self, raw_data: Any):
        """Process raw Speckle data into BIM objects"""
        self.get_logger().info("Processing BIM data...")
        
        # Extract all objects recursively
        all_objects = self._extract_all_objects(raw_data)
        
        self.get_logger().info(f"Found {len(all_objects)} total objects")
        
        # Apply filters
        filtered_objects = self._apply_filters(all_objects)
        
        self.get_logger().info(f"After filtering: {len(filtered_objects)} objects")
        
        # Convert to BIM messages
        header = Header()
        header.frame_id = self.get_parameter('frame_id').value
        header.stamp = self.get_clock().now().to_msg()
        
        self.bim_objects = []
        for obj in filtered_objects:
            bim_obj = self.converter.speckle_to_bim_object(obj, header)
            if bim_obj:
                self.bim_objects.append(bim_obj)
        
        self.get_logger().info(f"Converted {len(self.bim_objects)} BIM objects")
        
        # Publish data
        self._publish_objects()
        self._publish_visualization()
        
        # Notify plugins
        self.plugin_loader.notify_objects_received(self.bim_objects)

    def _extract_all_objects(self, data: Any, objects: Optional[List] = None) -> List:
        """Recursively extract all Speckle objects from nested structure using GraphTraversal"""
        if objects is None:
            objects = []
            
        try:
            from specklepy.objects.graph_traversal.traversal import GraphTraversal
            from specklepy.objects import Base
            
            # Create traversal engine
            traversal = GraphTraversal([])
            
            # Traverse and visit every object
            for context in traversal.traverse(data):
                obj = context.current
                
                # Filter out non-geometric/structural objects if needed
                # For now, we include everything that looks like a Base object
                if isinstance(obj, Base):
                    objects.append(obj)
                    
            self.get_logger().info(f"Extracted {len(objects)} objects using GraphTraversal")
            return objects
            
        except ImportError:
            self.get_logger().warning("GraphTraversal not available, falling back to manual recursion")
            return self._manual_extract_all_objects(data, objects)
            
    def _manual_extract_all_objects(self, data: Any, objects: Optional[List] = None) -> List:
        """Recursively extract all Speckle objects from nested structure (Fallback)"""
        if objects is None:
            objects = []
        
        # Handle dictionary
        if isinstance(data, dict):
            # Check if this is a BIM element
            if 'speckle_type' in data or 'category' in data:
                objects.append(data)
            
            # Recurse into children/elements
            for key in ['elements', 'children', '@elements', '@children']:
                if key in data:
                    children = data[key]
                    if isinstance(children, list):
                        for child in children:
                            self._manual_extract_all_objects(child, objects)
                    else:
                        self._manual_extract_all_objects(children, objects)
        
        # Handle Speckle objects with attributes
        elif hasattr(data, '__dict__'):
            # Check if this is a BIM element
            if hasattr(data, 'speckle_type') or hasattr(data, 'category'):
                objects.append(data)
            
            # Recurse into children
            for attr in ['elements', 'children']:
                if hasattr(data, attr):
                    children = getattr(data, attr)
                    if isinstance(children, list):
                        for child in children:
                            self._manual_extract_all_objects(child, objects)
                    elif children is not None:
                        self._manual_extract_all_objects(children, objects)
        
        # Handle lists
        elif isinstance(data, list):
            for item in data:
                self._manual_extract_all_objects(item, objects)
        
        return objects

    def _apply_filters(self, objects: List) -> List:
        """Apply category filters to object list"""
        try:
            p_allow = self.get_parameter('filters.allow')
            self.get_logger().info(f"Parameter filters.allow: {p_allow.type_} = {p_allow.value}")
            allow_list = p_allow.value
        except Exception as e:
            self.get_logger().warn(f"Error getting filters.allow: {e}. Using empty list.")
            allow_list = []

        try:
            p_deny = self.get_parameter('filters.deny')
            deny_list = p_deny.value
        except Exception as e:
            self.get_logger().warn(f"Error getting filters.deny: {e}. Using empty list.")
            deny_list = []
        
        # If no filters, return all
        if not allow_list and not deny_list:
            return objects
        
        filtered = []
        for obj in objects:
            category = self.converter._extract_category(obj)
            
            # Apply deny filter first
            if deny_list and category in deny_list:
                continue
            
            # Apply allow filter
            if allow_list:
                if category in allow_list:
                    filtered.append(obj)
            else:
                filtered.append(obj)
        
        return filtered

    def _publish_objects(self):
        """Publish BIM objects to topic"""
        for obj in self.bim_objects:
            self.object_pub.publish(obj)
        
        self.get_logger().info(f"Published {len(self.bim_objects)} BIM objects")

    def _publish_visualization(self):
        """Publish visualization markers"""
        marker_array = self.converter.create_marker_array(self.bim_objects)
        self.viz_pub.publish(marker_array)
        
        self.get_logger().info(f"Published {len(marker_array.markers)} visualization markers")

    def _publish_static_transform(self, parent_frame: str, child_frame: str, 
                                  translation: List[float]):
        """Publish static TF transform from map to bim_origin"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        
        transform.transform.rotation.w = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Published static transform: {parent_frame} -> {child_frame}"
        )
    
    def _load_plugins(self):
        """Load and initialize plugins from configuration file"""
        try:
            import yaml
            
            # Get plugins file path from ROS parameter
            plugins_file = self.get_parameter('plugins_file').value if self.has_parameter('plugins_file') else None
            
            # Fallback paths
            if not plugins_file or not os.path.exists(plugins_file):
                # Try common locations
                search_paths = [
                    '/config_override/plugins.yaml',
                    os.path.join(os.path.dirname(__file__), '../../config/plugins.yaml'),
                    'config/plugins.yaml'
                ]
                
                for path in search_paths:
                    if os.path.exists(path):
                        plugins_file = path
                        break
            
            if not plugins_file or not os.path.exists(plugins_file):
                self.get_logger().info("No plugins configuration file found, skipping plugins")
                return
            
            # Load YAML file
            with open(plugins_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            plugin_configs = config_data.get('plugins', [])
            
            if not plugin_configs:
                self.get_logger().info("No plugins configured in file")
                return
            
            self.get_logger().info(f"Loading plugins from: {plugins_file}")
            self.plugin_loader.load_plugins(plugin_configs)
            
        except Exception as e:
            self.get_logger().error(f"Failed to load plugins: {e}")
            import traceback
            self.get_logger().debug(traceback.format_exc())

    def handle_query(self, request: QueryBim.Request, response: QueryBim.Response):
        """Handle BIM query service requests"""
        try:
            self.get_logger().info(f"Query request: categories={request.categories}")
            
            # Filter by categories
            filtered = self.bim_objects
            if request.categories:
                filtered = [obj for obj in filtered if obj.category in request.categories]
            
            # Filter by element types
            if request.element_types:
                filtered = [obj for obj in filtered 
                          if obj.element_type in request.element_types]
            
            # TODO: Implement bounding box and property filters
            
            response.objects = filtered
            response.success = True
            response.message = f"Found {len(filtered)} matching objects"
            
            self.get_logger().info(f"Query result: {len(filtered)} objects")
            
        except Exception as e:
            self.get_logger().error(f"Query failed: {e}")
            response.success = False
            response.message = str(e)
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = SpeckleBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.plugin_loader.shutdown_all()
        if rclpy.ok():
            rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
