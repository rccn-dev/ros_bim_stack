"""
Mesh Exporter Plugin
Exports BIM object meshes to files for use by simulators
"""

import os
import json
from typing import List, Dict, Any
from pathlib import Path

from bim_interfaces.msg import BimObject
from ros_speckle_bridge.plugins.base import OutputPlugin


class MeshExporterPlugin(OutputPlugin):
    """
    Exports BIM object geometry as mesh files.
    
    Supports multiple formats:
    - Collada (.dae) - for Gazebo
    - OBJ (.obj) - for general 3D tools
    - STL (.stl) - for 3D printing, simple visualization
    - USD (.usd) - for Isaac Sim (requires pxr package)
    """
    
    SUPPORTED_FORMATS = ['dae', 'obj', 'stl', 'usd']
    
    def __init__(self, node):
        super().__init__(node)
        self.output_path = None
        self.format = 'dae'
        self.export_metadata = True
        self.exported_objects = set()
    
    def initialize(self, config: Dict[str, Any]) -> bool:
        """
        Initialize mesh exporter.
        
        Args:
            config: Configuration with:
                - output_path: Directory to export meshes
                - format: Mesh format (dae, obj, stl)
                - export_metadata: Whether to export metadata JSON
        
        Returns:
            True if successful
        """
        try:
            # Get output path
            self.output_path = Path(config.get('output_path', '/tmp/meshes'))
            self.format = config.get('format', 'dae').lower()
            self.export_metadata = config.get('export_metadata', True)
            
            # Validate format
            if self.format not in self.SUPPORTED_FORMATS:
                self.node.get_logger().error(
                    f"Unsupported format: {self.format}. "
                    f"Supported: {self.SUPPORTED_FORMATS}"
                )
                return False
            
            # Create output directory
            self.output_path.mkdir(parents=True, exist_ok=True)
            
            self.node.get_logger().info(
                f"Mesh exporter initialized: {self.output_path} (format: {self.format})"
            )
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Mesh exporter init failed: {e}")
            return False
    
    def on_objects_received(self, objects: List[BimObject]) -> None:
        """
        Export meshes for received BIM objects.
        
        Args:
            objects: List of BIM objects
        """
        self.node.get_logger().info(
            f"Exporting {len(objects)} meshes to {self.output_path}"
        )
        
        exported_count = 0
        for obj in objects:
            # Skip if already exported
            if obj.uuid in self.exported_objects:
                continue
            
            try:
                self._export_mesh(obj)
                self.exported_objects.add(obj.uuid)
                exported_count += 1
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to export mesh for {obj.uuid}: {e}"
                )
        
        self.node.get_logger().info(
            f"Exported {exported_count} new meshes "
            f"({len(self.exported_objects)} total)"
        )
    
    def _export_mesh(self, obj: BimObject) -> None:
        """
        Export mesh for a single BIM object.
        
        Args:
            obj: BimObject message
        """
        # Extract vertices and faces from geometry
        vertices = self._extract_vertices(obj)
        faces = self._extract_faces(obj)
        
        if not vertices or not faces:
            self.node.get_logger().warn(
                f"No geometry found for {obj.uuid}, skipping export"
            )
            return
        
        # Export based on format
        mesh_file = self.output_path / f"{obj.uuid}.{self.format}"
        
        if self.format == 'dae':
            self._export_collada(mesh_file, vertices, faces, obj)
        elif self.format == 'obj':
            self._export_obj(mesh_file, vertices, faces, obj)
        elif self.format == 'stl':
            self._export_stl(mesh_file, vertices, faces, obj)
        elif self.format == 'usd':
            self._export_usd(mesh_file, vertices, faces, obj)
        
        # Export metadata
        if self.export_metadata:
            self._export_metadata(obj)
        
        self.node.get_logger().debug(f"Exported: {mesh_file}")
    
    def _extract_vertices(self, obj: BimObject) -> List[List[float]]:
        """Extract vertex list from BimObject geometry"""
        # BimObject.geometry is a float32[] array
        # Format: [x1, y1, z1, x2, y2, z2, ...]
        geometry = obj.geometry
        
        if len(geometry) % 3 != 0:
            self.node.get_logger().warn(
                f"Invalid geometry length for {obj.uuid}: {len(geometry)}"
            )
            return []
        
        vertices = []
        for i in range(0, len(geometry), 3):
            vertices.append([geometry[i], geometry[i+1], geometry[i+2]])
        
        return vertices
    
    def _extract_faces(self, obj: BimObject) -> List[List[int]]:
        """
        Extract face indices from vertices.
        
        Note: BimObject currently stores flat vertex list.
        We assume triangular faces in sequence.
        """
        num_vertices = len(obj.geometry) // 3
        
        if num_vertices % 3 != 0:
            self.node.get_logger().warn(
                f"Vertex count not divisible by 3 for {obj.uuid}, assuming triangle soup"
            )
        
        # Create triangular faces
        faces = []
        for i in range(0, num_vertices, 3):
            if i + 2 < num_vertices:
                faces.append([i, i+1, i+2])
        
        return faces
    
    def _export_collada(self, filepath: Path, vertices: List, faces: List, 
                       obj: BimObject) -> None:
        """Export as Collada (.dae) format for Gazebo"""
        
        # Build vertex position string
        positions = " ".join([f"{v[0]} {v[1]} {v[2]}" for v in vertices])
        
        # Build face indices string
        indices = " ".join([f"{f[0]} {f[1]} {f[2]}" for f in faces])
        
        # Create Collada XML
        collada_xml = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <created>2025-12-22T00:00:00</created>
    <modified>2025-12-22T00:00:00</modified>
    <unit name="meter" meter="1.0"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  
  <library_geometries>
    <geometry id="{obj.uuid}-mesh" name="{obj.uuid}">
      <mesh>
        <source id="{obj.uuid}-positions">
          <float_array id="{obj.uuid}-positions-array" count="{len(vertices) * 3}">
            {positions}
          </float_array>
          <technique_common>
            <accessor source="#{obj.uuid}-positions-array" count="{len(vertices)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        
        <vertices id="{obj.uuid}-vertices">
          <input semantic="POSITION" source="#{obj.uuid}-positions"/>
        </vertices>
        
        <triangles count="{len(faces)}">
          <input semantic="VERTEX" source="#{obj.uuid}-vertices" offset="0"/>
          <p>{indices}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{obj.uuid}" name="{obj.uuid}">
        <instance_geometry url="#{obj.uuid}-mesh"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>"""
        
        with open(filepath, 'w') as f:
            f.write(collada_xml)
    
    def _export_obj(self, filepath: Path, vertices: List, faces: List,
                   obj: BimObject) -> None:
        """Export as Wavefront OBJ format"""
        
        with open(filepath, 'w') as f:
            f.write(f"# {obj.uuid}\n")
            f.write(f"# Category: {obj.category}\n\n")
            
            # Write vertices
            for v in vertices:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            
            f.write("\n")
            
            # Write faces (OBJ uses 1-based indexing)
            for face in faces:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    def _export_stl(self, filepath: Path, vertices: List, faces: List,
                   obj: BimObject) -> None:
        """Export as STL format (ASCII)"""
        
        with open(filepath, 'w') as f:
            f.write(f"solid {obj.uuid}\n")
            
            for face in faces:
                v0 = vertices[face[0]]
                v1 = vertices[face[1]]
                v2 = vertices[face[2]]
                
                # Compute normal (simplified - should be calculated properly)
                f.write("  facet normal 0 0 1\n")
                f.write("    outer loop\n")
                f.write(f"      vertex {v0[0]} {v0[1]} {v0[2]}\n")
                f.write(f"      vertex {v1[0]} {v1[1]} {v1[2]}\n")
                f.write(f"      vertex {v2[0]} {v2[1]} {v2[2]}\n")
                f.write("    endloop\n")
                f.write("  endfacet\n")
            
            f.write(f"endsolid {obj.uuid}\n")
    
    def _export_metadata(self, obj: BimObject) -> None:
        """Export object metadata as JSON"""
        
        metadata = {
            'id': obj.uuid,
            'category': obj.category,
            'element_type': obj.element_type,
            'pose': {
                'position': {
                    'x': obj.pose.position.x,
                    'y': obj.pose.position.y,
                    'z': obj.pose.position.z
                },
                'orientation': {
                    'x': obj.pose.orientation.x,
                    'y': obj.pose.orientation.y,
                    'z': obj.pose.orientation.z,
                    'w': obj.pose.orientation.w
                }
            },
            'properties': [
                {'key': p.key, 'value': p.value, 'type': p.type}
                for p in obj.parameters
            ]
        }
        
        json_file = self.output_path / f"{obj.uuid}.json"
        with open(json_file, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def _export_usd(self, filepath: Path, vertices: List, faces: List,
                   obj: BimObject) -> None:
        """Export as USD format for Isaac Sim (requires pxr package)"""
        try:
            from pxr import Usd, UsdGeom, Gf
            
            # Create USD stage
            stage = Usd.Stage.CreateNew(str(filepath))
            
            # Create mesh primitive
            mesh_path = f'/{obj.uuid}'
            mesh = UsdGeom.Mesh.Define(stage, mesh_path)
            
            # Set points (vertices)
            points = [Gf.Vec3f(v[0], v[1], v[2]) for v in vertices]
            mesh.GetPointsAttr().Set(points)
            
            # Set face vertex counts (all triangles)
            face_vertex_counts = [3] * len(faces)
            mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
            
            # Set face vertex indices
            indices = [idx for face in faces for idx in face]
            mesh.GetFaceVertexIndicesAttr().Set(indices)
            
            # Add metadata as custom attributes
            prim = stage.GetPrimAtPath(mesh_path)
            prim.SetCustomDataByKey('category', obj.category)
            prim.SetCustomDataByKey('element_type', obj.element_type)
            prim.SetCustomDataByKey('speckle_id', obj.uuid)
            
            # Save stage
            stage.Save()
            
        except ImportError:
            self.node.get_logger().error(
                "USD export requires 'pxr' package. Install with: pip install usd-core"
            )
            raise
    
    def shutdown(self) -> None:
        """Cleanup"""
        self.node.get_logger().info(
            f"Mesh exporter shutting down. Exported {len(self.exported_objects)} meshes."
        )
    
    @classmethod
    def get_plugin_name(cls) -> str:
        return "MeshExporter"
