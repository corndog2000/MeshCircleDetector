"""
Mesh Utilities for Fusion 360

This module provides utilities for extracting mesh data from Fusion 360
mesh bodies and converting between Fusion 360 types and our internal types.
"""

import adsk.core
import adsk.fusion
from typing import List, Tuple, Optional
from .circle_detector import Point3D, Vector3D, DetectedCircle


def get_mesh_data(mesh_body: adsk.fusion.MeshBody) -> Tuple[List[Tuple[float, float, float]], 
                                                             List[Tuple[int, int, int]]]:
    """
    Extract vertex and triangle data from a Fusion 360 MeshBody.
    
    Args:
        mesh_body: The Fusion 360 MeshBody to extract data from
        
    Returns:
        Tuple of (vertices, triangles) where:
            vertices: List of (x, y, z) coordinates in cm
            triangles: List of (v1, v2, v3) vertex indices
    """
    display_mesh = mesh_body.displayMesh
    
    # Get vertices
    node_coords = display_mesh.nodeCoordinatesAsFloat
    vertices = []
    for i in range(0, len(node_coords), 3):
        vertices.append((
            node_coords[i],      # x
            node_coords[i + 1],  # y
            node_coords[i + 2]   # z
        ))
    
    # Get triangles
    indices = display_mesh.nodeIndices
    triangles = []
    for i in range(0, len(indices), 3):
        triangles.append((
            indices[i],
            indices[i + 1],
            indices[i + 2]
        ))
    
    return vertices, triangles


def get_all_mesh_bodies(design: adsk.fusion.Design) -> List[adsk.fusion.MeshBody]:
    """
    Get all mesh bodies from the active design.
    
    Args:
        design: The active Fusion 360 design
        
    Returns:
        List of all MeshBody objects in the design
    """
    mesh_bodies = []
    
    # Check root component
    root = design.rootComponent
    for mesh_body in root.meshBodies:
        mesh_bodies.append(mesh_body)
    
    # Check all occurrences
    for occ in root.allOccurrences:
        for mesh_body in occ.component.meshBodies:
            mesh_bodies.append(mesh_body)
    
    return mesh_bodies


def point3d_to_fusion(point: Point3D) -> adsk.core.Point3D:
    """
    Convert our Point3D to a Fusion 360 Point3D.
    
    Note: Fusion 360 uses centimeters internally, ensure input is in cm.
    
    Args:
        point: Our Point3D object
        
    Returns:
        Fusion 360 Point3D
    """
    return adsk.core.Point3D.create(point.x, point.y, point.z)


def fusion_to_point3d(point: adsk.core.Point3D) -> Point3D:
    """
    Convert a Fusion 360 Point3D to our Point3D.
    
    Args:
        point: Fusion 360 Point3D
        
    Returns:
        Our Point3D object
    """
    return Point3D(point.x, point.y, point.z)


def vector3d_to_fusion(vector: Vector3D) -> adsk.core.Vector3D:
    """
    Convert our Vector3D to a Fusion 360 Vector3D.
    
    Args:
        vector: Our Vector3D object
        
    Returns:
        Fusion 360 Vector3D
    """
    return adsk.core.Vector3D.create(vector.x, vector.y, vector.z)


def fusion_to_vector3d(vector: adsk.core.Vector3D) -> Vector3D:
    """
    Convert a Fusion 360 Vector3D to our Vector3D.
    
    Args:
        vector: Fusion 360 Vector3D
        
    Returns:
        Our Vector3D object
    """
    return Vector3D(vector.x, vector.y, vector.z)


def get_view_ray_at_cursor(viewport: adsk.core.Viewport, 
                           screen_x: int, 
                           screen_y: int) -> Tuple[adsk.core.Point3D, adsk.core.Vector3D]:
    """
    Get the 3D ray from the camera through a screen point.
    
    Args:
        viewport: The active viewport
        screen_x: Screen X coordinate
        screen_y: Screen Y coordinate
        
    Returns:
        Tuple of (ray_origin, ray_direction)
    """
    camera = viewport.camera
    
    # Get viewport dimensions
    width = viewport.width
    height = viewport.height
    
    # Normalize screen coordinates to [-1, 1]
    ndc_x = (2.0 * screen_x / width) - 1.0
    ndc_y = 1.0 - (2.0 * screen_y / height)
    
    # Get camera parameters
    eye = camera.eye
    target = camera.target
    up_vector = camera.upVector
    
    # Calculate view direction
    view_dir = adsk.core.Vector3D.create(
        target.x - eye.x,
        target.y - eye.y,
        target.z - eye.z
    )
    view_dir.normalize()
    
    # Calculate right vector
    right = view_dir.crossProduct(up_vector)
    right.normalize()
    
    # Recalculate up to ensure orthogonality
    up = right.crossProduct(view_dir)
    up.normalize()
    
    # Calculate field of view scaling
    # This is approximate - Fusion doesn't expose exact FOV
    if camera.cameraType == adsk.core.CameraTypes.PerspectiveCameraType:
        fov_scale = camera.viewExtents / 10.0  # Approximate
    else:
        fov_scale = camera.viewExtents / 2.0
    
    # Calculate ray direction
    aspect = width / height
    ray_dir = adsk.core.Vector3D.create(
        view_dir.x + right.x * ndc_x * fov_scale * aspect + up.x * ndc_y * fov_scale,
        view_dir.y + right.y * ndc_x * fov_scale * aspect + up.y * ndc_y * fov_scale,
        view_dir.z + right.z * ndc_x * fov_scale * aspect + up.z * ndc_y * fov_scale
    )
    ray_dir.normalize()
    
    return eye, ray_dir


def ray_plane_intersection(ray_origin: adsk.core.Point3D,
                           ray_direction: adsk.core.Vector3D,
                           plane_point: adsk.core.Point3D,
                           plane_normal: adsk.core.Vector3D) -> Optional[adsk.core.Point3D]:
    """
    Calculate the intersection point of a ray and a plane.
    
    Args:
        ray_origin: Starting point of the ray
        ray_direction: Direction of the ray
        plane_point: A point on the plane
        plane_normal: Normal vector of the plane
        
    Returns:
        Intersection point, or None if ray is parallel to plane
    """
    denom = (plane_normal.x * ray_direction.x + 
             plane_normal.y * ray_direction.y + 
             plane_normal.z * ray_direction.z)
    
    if abs(denom) < 1e-10:
        return None
    
    diff = adsk.core.Vector3D.create(
        plane_point.x - ray_origin.x,
        plane_point.y - ray_origin.y,
        plane_point.z - ray_origin.z
    )
    
    t = (plane_normal.x * diff.x + 
         plane_normal.y * diff.y + 
         plane_normal.z * diff.z) / denom
    
    if t < 0:
        return None
    
    return adsk.core.Point3D.create(
        ray_origin.x + t * ray_direction.x,
        ray_origin.y + t * ray_direction.y,
        ray_origin.z + t * ray_direction.z
    )


def create_sketch_circle(sketch: adsk.fusion.Sketch,
                         circle: DetectedCircle,
                         include_center_point: bool = True) -> adsk.fusion.SketchCircle:
    """
    Create a sketch circle from a DetectedCircle.
    
    Args:
        sketch: The target sketch
        circle: The detected circle to create
        include_center_point: Whether to also create a construction point at center
        
    Returns:
        The created SketchCircle
    """
    # Convert center and normal to Fusion types
    center = point3d_to_fusion(circle.center)
    normal = vector3d_to_fusion(circle.normal)
    
    # Project center onto sketch plane
    sketch_plane = sketch.referencePlane
    
    # For best results, check if circle normal aligns with sketch normal
    sketch_normal = sketch_plane.geometry.normal if hasattr(sketch_plane, 'geometry') else None
    
    # Create the circle
    circles = sketch.sketchCurves.sketchCircles
    
    # Project the center point onto the sketch plane
    projected_center = sketch.modelToSketchSpace(center)
    
    # Create circle with projected center
    sketch_circle = circles.addByCenterRadius(projected_center, circle.radius)
    
    # Create center point if requested
    if include_center_point:
        sketch_points = sketch.sketchPoints
        center_point = sketch_points.add(projected_center)
        center_point.isConstruction = True
    
    return sketch_circle


def find_nearest_mesh_point(mesh_body: adsk.fusion.MeshBody,
                            ray_origin: adsk.core.Point3D,
                            ray_direction: adsk.core.Vector3D) -> Optional[adsk.core.Point3D]:
    """
    Find the nearest point on a mesh body along a ray.
    
    This is a simplified implementation that checks triangle intersections.
    
    Args:
        mesh_body: The mesh body to check
        ray_origin: Ray origin point
        ray_direction: Ray direction
        
    Returns:
        Nearest intersection point, or None if no intersection
    """
    vertices, triangles = get_mesh_data(mesh_body)
    
    min_t = float('inf')
    nearest_point = None
    
    for tri in triangles:
        # Get triangle vertices
        v0 = adsk.core.Point3D.create(*vertices[tri[0]])
        v1 = adsk.core.Point3D.create(*vertices[tri[1]])
        v2 = adsk.core.Point3D.create(*vertices[tri[2]])
        
        # Möller–Trumbore intersection algorithm
        edge1 = adsk.core.Vector3D.create(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z)
        edge2 = adsk.core.Vector3D.create(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z)
        
        h = ray_direction.crossProduct(edge2)
        a = edge1.dotProduct(h)
        
        if abs(a) < 1e-10:
            continue
        
        f = 1.0 / a
        s = adsk.core.Vector3D.create(
            ray_origin.x - v0.x,
            ray_origin.y - v0.y,
            ray_origin.z - v0.z
        )
        u = f * s.dotProduct(h)
        
        if u < 0.0 or u > 1.0:
            continue
        
        q = s.crossProduct(edge1)
        v = f * ray_direction.dotProduct(q)
        
        if v < 0.0 or u + v > 1.0:
            continue
        
        t = f * edge2.dotProduct(q)
        
        if t > 1e-10 and t < min_t:
            min_t = t
            nearest_point = adsk.core.Point3D.create(
                ray_origin.x + t * ray_direction.x,
                ray_origin.y + t * ray_direction.y,
                ray_origin.z + t * ray_direction.z
            )
    
    return nearest_point


def get_mesh_bounding_box(mesh_body: adsk.fusion.MeshBody) -> adsk.core.BoundingBox3D:
    """
    Get the bounding box of a mesh body.
    
    Args:
        mesh_body: The mesh body
        
    Returns:
        BoundingBox3D of the mesh
    """
    return mesh_body.boundingBox
