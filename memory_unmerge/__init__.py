"""Hierarchical spatial scene graph for the Go2 lab map.

This package does not modify ``memory/``. It loads the places the navigation
stack already saved and keeps object instances, the place that contains each
one, and the stand pose the existing A* planner already drives to.
"""

from .scene_graph import SceneGraphManager

__all__ = ["SceneGraphManager"]
