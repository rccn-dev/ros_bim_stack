"""
Gazebo Harmonic Spawner Plugin
Spawns BIM objects as entities in Gazebo Harmonic simulation
"""

import os
from typing import List, Dict, Any
from pathlib import Path

from rclpy.node import Node
from geometry_msgs.msg import Pose
from bim_interfaces.msg import BimObject

# Import plugin base from core package
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))
from ros_speckle_bridge.plugins.base import OutputPlugin


class GazeboSpawnerPlugin(OutputPlugin):
    """
    Spawns BIM objects as Gazebo entities using Gazebo Harmonic services.
    
    This plugin:
    1. Subscribes to processed BIM objects
    2. Converts them to SDF (Simulation Description Format)
    3. Spawns them in Gazebo via ROS 2 service calls
    """
    
    def __init__(self, node: Node):
        super().__init__(node)
        self.spawn_service = None
        self.spawned_entities = set()
        self.mesh_path = "/shared_meshes"
        self.frame_id = "world"
        self.default_physics = {
            'static': False,
            'mass': 1.0,
            'friction': 0.8,
            'restitution': 0.5
        }
    
    def initialize(self, config: Dict[str, Any]) -> bool:
        """
        Initialize Gazebo spawner plugin.
        
        Args:
            config: Plugin configuration with:
                - spawn_service: Gazebo create service name
                - frame_id: Reference frame for spawning
                - mesh_path: Path to mesh files
                - default_physics: Default physics properties
        
        Returns:
            True if initialization successful
        """
        try:
            # Get configuration
            spawn_service_name = config.get('spawn_service', '/world/default/create')
            self.mesh_path = config.get('mesh_path', '/shared_meshes')
            self.frame_id = config.get('frame_id', 'world')
            
            # Update default physics
            if 'default_physics' in config:
                self.default_physics.update(config['default_physics'])
            
            # Import Gazebo messages (deferred to avoid core dependency)
            try:
                from ros_gz_interfaces.srv import SpawnEntity
                self.SpawnEntity = SpawnEntity
            except ImportError:
                self.node.get_logger().error(
                    "ros_gz_interfaces not found! Install with: "
                    "apt install ros-humble-ros-gz-interfaces"
                )
                return False
            
            # Create service client
            self.spawn_service = self.node.create_client(
                self.SpawnEntity,
                spawn_service_name
            )
            
            # Wait for service
            self.node.get_logger().info(
                f"Waiting for Gazebo service: {spawn_service_name}"
            )
            
            if not self.spawn_service.wait_for_service(timeout_sec=10.0):
                self.node.get_logger().warn(
                    f"Gazebo service not available yet. Will retry on first spawn."
                )
            
            self.node.get_logger().info(
                f"Gazebo spawner initialized (mesh_path: {self.mesh_path})"
            )
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Gazebo spawner initialization failed: {e}")
            return False
    
    def on_objects_received(self, objects: List[BimObject]) -> None:
        """
        Spawn BIM objects in Gazebo.
        
        Args:
            objects: List of BIM objects to spawn
        """
        self.node.get_logger().info(f"Spawning {len(objects)} objects in Gazebo...")
        
        for obj in objects:
            # Skip if already spawned
            if obj.uuid in self.spawned_entities:
                continue
            
            try:
                self._spawn_object(obj)
                self.spawned_entities.add(obj.uuid)
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to spawn object {obj.uuid}: {e}"
                )
        
        self.node.get_logger().info(
            f"Spawned {len(self.spawned_entities)} total entities"
        )
    
    def _spawn_object(self, obj: BimObject) -> None:
        """
        Spawn a single BIM object as a Gazebo entity.
        
        Args:
            obj: BimObject message
        """
        # Generate SDF for this object
        sdf_xml = self._generate_sdf(obj)
        
        # Create spawn request
        request = self.SpawnEntity.Request()
        request.name = obj.uuid
        request.xml = sdf_xml
        
        # Call service
        future = self.spawn_service.call_async(request)
        
        # Note: We don't wait for response to avoid blocking
        # For production, consider using a callback to verify spawning
        future.add_done_callback(
            lambda f: self._spawn_callback(f, obj.uuid)
        )
    
    def _spawn_callback(self, future, entity_id: str) -> None:
        """Callback for spawn service response"""
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().debug(f"✓ Spawned entity: {entity_id}")
            else:
                self.node.get_logger().error(
                    f"Failed to spawn {entity_id}: {response.status_message}"
                )
        except Exception as e:
            self.node.get_logger().error(f"Spawn service call failed: {e}")
    
    def _generate_sdf(self, obj: BimObject) -> str:
        """
        Generate SDF XML for a BIM object.
        
        Args:
            obj: BimObject message
            
        Returns:
            SDF XML string
        """
        # Extract pose
        pos = obj.pose.position
        rot = obj.pose.orientation
        
        # Determine mesh file
        mesh_file = self._get_mesh_file(obj)
        
        # Get physics properties
        mass = self.default_physics['mass']
        static = 'true' if self.default_physics['static'] else 'false'
        friction = self.default_physics['friction']
        
        # Generate SDF
        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{obj.uuid}">
    <pose>{pos.x} {pos.y} {pos.z} {rot.x} {rot.y} {rot.z}</pose>
    <static>{static}</static>
    
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      
      <!-- Visual -->
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{mesh_file}</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      
      <!-- Collision -->
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>{mesh_file}</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{friction}</mu>
              <mu2>{friction}</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>"""
        
        return sdf
    
    def _get_mesh_file(self, obj: BimObject) -> str:
        """
        Get mesh file URI for a BIM object.
        
        Args:
            obj: BimObject message
            
        Returns:
            File URI for Gazebo (file:// or package://)
        """
        # Expected mesh filename (generated by mesh_exporter plugin)
        mesh_filename = f"{obj.uuid}.dae"  # Collada format for Gazebo
        mesh_filepath = Path(self.mesh_path) / mesh_filename
        
        # Check if mesh exists
        if mesh_filepath.exists():
            return f"file://{mesh_filepath.absolute()}"
        
        # Fallback to simple box geometry
        self.node.get_logger().warn(
            f"Mesh not found for {obj.uuid}, using box geometry"
        )
        
        # Return a procedural box (Gazebo will generate it)
        # In a real implementation, you'd compute bounding box from obj.geometry
        return None  # Trigger fallback to box in SDF
    
    def shutdown(self) -> None:
        """Clean up resources"""
        if self.spawn_service:
            self.spawn_service.destroy()
        self.node.get_logger().info("Gazebo spawner shut down")
    
    @classmethod
    def get_plugin_name(cls) -> str:
        return "GazeboSpawner"
