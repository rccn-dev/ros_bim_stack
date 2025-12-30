#!/usr/bin/env python3
"""
Simple validation script to check plugin system syntax without ROS dependencies
"""

import ast
import sys
from pathlib import Path

def check_syntax(filepath):
    """Check Python file for syntax errors"""
    try:
        with open(filepath, 'r') as f:
            ast.parse(f.read())
        print(f"✓ {filepath.name}")
        return True
    except SyntaxError as e:
        print(f"✗ {filepath.name}: {e}")
        return False

def main():
    """Check all plugin files"""
    # Script is in scripts/, so project root is parent directory
    root = Path(__file__).parent.parent
    
    files_to_check = [
        root / "ros_speckle_bridge/ros_speckle_bridge/plugins/__init__.py",
        root / "ros_speckle_bridge/ros_speckle_bridge/plugins/base.py",
        root / "ros_speckle_bridge/ros_speckle_bridge/plugins/loader.py",
        root / "ros_speckle_bridge/ros_speckle_bridge/plugins/mesh_exporter.py",
        root / "examples/gazebo_integration/ros_speckle_gazebo/__init__.py",
        root / "examples/gazebo_integration/ros_speckle_gazebo/gazebo_spawner.py",
    ]
    
    print("Validating plugin system files...")
    print()
    
    all_valid = True
    for filepath in files_to_check:
        if not filepath.exists():
            print(f"✗ {filepath.name}: File not found")
            all_valid = False
        else:
            if not check_syntax(filepath):
                all_valid = False
    
    print()
    if all_valid:
        print("✅ All plugin files are syntactically valid!")
        return 0
    else:
        print("❌ Some files have errors")
        return 1

if __name__ == '__main__':
    sys.exit(main())
