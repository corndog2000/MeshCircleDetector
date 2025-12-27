"""
Mesh Circle Detector Library

This package contains utilities for detecting circular features on mesh bodies
in Fusion 360.
"""

from .circle_detector import (
    Point3D,
    Vector3D,
    DetectedCircle,
    CircleDetector,
    detect_coplanar_circle_features
)

from .enhanced_detector import EnhancedCircleDetector

from .mesh_utils import (
    get_mesh_data,
    get_all_mesh_bodies,
    point3d_to_fusion,
    fusion_to_point3d,
    vector3d_to_fusion,
    fusion_to_vector3d,
    get_view_ray_at_cursor,
    ray_plane_intersection,
    create_sketch_circle,
    find_nearest_mesh_point,
    get_mesh_bounding_box
)

from .graphics import (
    CircleGraphicsManager,
    ProgressIndicator
)

__all__ = [
    'Point3D',
    'Vector3D', 
    'DetectedCircle',
    'CircleDetector',
    'EnhancedCircleDetector',
    'detect_coplanar_circle_features',
    'get_mesh_data',
    'get_all_mesh_bodies',
    'point3d_to_fusion',
    'fusion_to_point3d',
    'vector3d_to_fusion',
    'fusion_to_vector3d',
    'get_view_ray_at_cursor',
    'ray_plane_intersection',
    'create_sketch_circle',
    'find_nearest_mesh_point',
    'get_mesh_bounding_box',
    'CircleGraphicsManager',
    'ProgressIndicator'
]
