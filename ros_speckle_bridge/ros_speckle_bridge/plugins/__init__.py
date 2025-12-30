"""
Plugin System for ROS Speckle Bridge
Enables extensible output handlers without adding dependencies to core package
"""

from .base import OutputPlugin

__all__ = ['OutputPlugin']
