"""
Circle Detection Library for Mesh Bodies

This module provides algorithms to detect circular features (holes, bosses)
on mesh bodies by analyzing boundary edges and face geometry.
"""

import math
from typing import List, Tuple, Optional, NamedTuple
from collections import defaultdict

class Point3D(NamedTuple):
    """Simple 3D point representation"""
    x: float
    y: float
    z: float
    
    def distance_to(self, other: 'Point3D') -> float:
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2 +
            (self.z - other.z) ** 2
        )
    
    def __add__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Point3D':
        return Point3D(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __truediv__(self, scalar: float) -> 'Point3D':
        return Point3D(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def dot(self, other: 'Point3D') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other: 'Point3D') -> 'Point3D':
        return Point3D(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )
    
    def length(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def normalized(self) -> 'Point3D':
        length = self.length()
        if length < 1e-10:
            return Point3D(0, 0, 1)
        return self / length


class Vector3D(Point3D):
    """Alias for clarity when representing directions"""
    pass


class DetectedCircle(NamedTuple):
    """Represents a detected circular feature"""
    center: Point3D
    radius: float
    normal: Vector3D
    points: List[Point3D]  # Original points used for detection
    confidence: float  # 0-1 confidence score
    
    def distance_to_point(self, point: Point3D) -> float:
        """Calculate distance from a 3D point to this circle's center"""
        return self.center.distance_to(point)


class CircleDetector:
    """
    Detects circular features on mesh bodies.
    
    The algorithm works by:
    1. Finding boundary edges (edges belonging to only one triangle)
    2. Grouping connected boundary edges into loops
    3. Analyzing each loop for circularity
    4. Fitting circles to circular loops using least-squares
    """
    
    def __init__(self, 
                 min_points: int = 8,
                 circularity_threshold: float = 0.85,
                 min_radius: float = 0.5,  # mm
                 max_radius: float = 500.0):  # mm
        """
        Initialize the circle detector.
        
        Args:
            min_points: Minimum points to consider for circle fitting
            circularity_threshold: How circular a loop must be (0-1)
            min_radius: Minimum radius to detect (mm)
            max_radius: Maximum radius to detect (mm)
        """
        self.min_points = min_points
        self.circularity_threshold = circularity_threshold
        self.min_radius = min_radius
        self.max_radius = max_radius
    
    def detect_circles_from_mesh_data(self, 
                                       vertices: List[Tuple[float, float, float]],
                                       triangles: List[Tuple[int, int, int]]) -> List[DetectedCircle]:
        """
        Detect circles from raw mesh data.
        
        Args:
            vertices: List of (x, y, z) vertex coordinates
            triangles: List of (v1, v2, v3) vertex indices for each triangle
            
        Returns:
            List of detected circles
        """
        # Convert to Point3D
        points = [Point3D(*v) for v in vertices]
        
        # Find boundary edges
        boundary_loops = self._find_boundary_loops(points, triangles)
        
        # Analyze each loop for circularity
        detected_circles = []
        for loop in boundary_loops:
            if len(loop) >= self.min_points:
                circle = self._fit_circle_to_loop(loop)
                if circle and circle.confidence >= self.circularity_threshold:
                    if self.min_radius <= circle.radius <= self.max_radius:
                        detected_circles.append(circle)
        
        return detected_circles
    
    def _find_boundary_loops(self, 
                              points: List[Point3D],
                              triangles: List[Tuple[int, int, int]]) -> List[List[Point3D]]:
        """Find boundary edge loops in the mesh."""
        # Count edge occurrences
        edge_count = defaultdict(int)
        edge_to_vertices = {}
        
        for tri in triangles:
            edges = [
                tuple(sorted([tri[0], tri[1]])),
                tuple(sorted([tri[1], tri[2]])),
                tuple(sorted([tri[2], tri[0]]))
            ]
            for edge in edges:
                edge_count[edge] += 1
                edge_to_vertices[edge] = edge
        
        # Boundary edges appear only once
        boundary_edges = [edge for edge, count in edge_count.items() if count == 1]
        
        if not boundary_edges:
            return []
        
        # Build adjacency for boundary edges
        vertex_to_edges = defaultdict(list)
        for edge in boundary_edges:
            vertex_to_edges[edge[0]].append(edge)
            vertex_to_edges[edge[1]].append(edge)
        
        # Find connected loops
        loops = []
        used_edges = set()
        
        for start_edge in boundary_edges:
            if start_edge in used_edges:
                continue
            
            # Trace the loop
            loop_vertices = []
            current_edge = start_edge
            current_vertex = start_edge[0]
            
            while current_edge not in used_edges:
                used_edges.add(current_edge)
                loop_vertices.append(points[current_vertex])
                
                # Move to next vertex
                next_vertex = current_edge[1] if current_edge[0] == current_vertex else current_edge[0]
                
                # Find next edge
                next_edge = None
                for edge in vertex_to_edges[next_vertex]:
                    if edge not in used_edges:
                        next_edge = edge
                        break
                
                if next_edge is None:
                    break
                    
                current_vertex = next_vertex
                current_edge = next_edge
            
            if len(loop_vertices) >= self.min_points:
                loops.append(loop_vertices)
        
        return loops
    
    def _fit_circle_to_loop(self, points: List[Point3D]) -> Optional[DetectedCircle]:
        """
        Fit a circle to a loop of points.
        
        Uses a plane fitting followed by 2D circle fitting approach.
        """
        if len(points) < self.min_points:
            return None
        
        # Calculate centroid
        centroid = Point3D(
            sum(p.x for p in points) / len(points),
            sum(p.y for p in points) / len(points),
            sum(p.z for p in points) / len(points)
        )
        
        # Fit plane using PCA (simplified - use normal from cross products)
        normal = self._estimate_plane_normal(points, centroid)
        
        # Project points onto the plane and fit 2D circle
        projected_2d = self._project_to_plane(points, centroid, normal)
        
        # Fit 2D circle using algebraic method
        center_2d, radius = self._fit_circle_2d(projected_2d)
        
        if radius is None or radius < self.min_radius or radius > self.max_radius:
            return None
        
        # Transform center back to 3D
        center_3d = self._unproject_from_plane(center_2d, centroid, normal)
        
        # Calculate confidence based on how well points fit the circle
        confidence = self._calculate_confidence(points, center_3d, radius)
        
        return DetectedCircle(
            center=center_3d,
            radius=radius,
            normal=normal,
            points=points,
            confidence=confidence
        )
    
    def _estimate_plane_normal(self, points: List[Point3D], centroid: Point3D) -> Vector3D:
        """Estimate the plane normal using cross products of consecutive edges."""
        normals = []
        
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            p3 = points[(i + 2) % len(points)]
            
            v1 = p2 - p1
            v2 = p3 - p2
            
            n = v1.cross(v2)
            length = n.length()
            if length > 1e-10:
                normals.append(n / length)
        
        if not normals:
            return Vector3D(0, 0, 1)
        
        # Average normals
        avg_normal = Point3D(
            sum(n.x for n in normals) / len(normals),
            sum(n.y for n in normals) / len(normals),
            sum(n.z for n in normals) / len(normals)
        )
        
        return avg_normal.normalized()
    
    def _project_to_plane(self, 
                          points: List[Point3D], 
                          origin: Point3D, 
                          normal: Vector3D) -> List[Tuple[float, float]]:
        """Project 3D points onto a 2D plane."""
        # Create orthonormal basis for the plane
        if abs(normal.z) < 0.9:
            u = Vector3D(-normal.y, normal.x, 0).normalized()
        else:
            u = Vector3D(1, 0, 0)
        
        v = normal.cross(u).normalized()
        
        projected = []
        for p in points:
            diff = p - origin
            x = diff.dot(u)
            y = diff.dot(v)
            projected.append((x, y))
        
        return projected
    
    def _unproject_from_plane(self, 
                               point_2d: Tuple[float, float],
                               origin: Point3D,
                               normal: Vector3D) -> Point3D:
        """Transform a 2D point back to 3D on the plane."""
        # Recreate the same basis
        if abs(normal.z) < 0.9:
            u = Vector3D(-normal.y, normal.x, 0).normalized()
        else:
            u = Vector3D(1, 0, 0)
        
        v = normal.cross(u).normalized()
        
        return Point3D(
            origin.x + point_2d[0] * u.x + point_2d[1] * v.x,
            origin.y + point_2d[0] * u.y + point_2d[1] * v.y,
            origin.z + point_2d[0] * u.z + point_2d[1] * v.z
        )
    
    def _fit_circle_2d(self, points: List[Tuple[float, float]]) -> Tuple[Tuple[float, float], Optional[float]]:
        """
        Fit a circle to 2D points using the algebraic method.
        
        Solves the system: (x-a)^2 + (y-b)^2 = r^2
        Linearized: 2ax + 2by + c = x^2 + y^2 where c = r^2 - a^2 - b^2
        """
        n = len(points)
        if n < 3:
            return (0, 0), None
        
        # Build the system of equations
        sum_x = sum(p[0] for p in points)
        sum_y = sum(p[1] for p in points)
        sum_x2 = sum(p[0] ** 2 for p in points)
        sum_y2 = sum(p[1] ** 2 for p in points)
        sum_xy = sum(p[0] * p[1] for p in points)
        sum_x3 = sum(p[0] ** 3 for p in points)
        sum_y3 = sum(p[1] ** 3 for p in points)
        sum_x2y = sum(p[0] ** 2 * p[1] for p in points)
        sum_xy2 = sum(p[0] * p[1] ** 2 for p in points)
        
        # Matrix form: A * [a, b, c]^T = B
        A = [
            [sum_x2, sum_xy, sum_x],
            [sum_xy, sum_y2, sum_y],
            [sum_x, sum_y, n]
        ]
        
        B = [
            sum_x3 + sum_xy2,
            sum_x2y + sum_y3,
            sum_x2 + sum_y2
        ]
        
        # Solve using Cramer's rule (simple 3x3)
        det_A = self._det3x3(A)
        
        if abs(det_A) < 1e-10:
            # Fallback: use centroid and average distance
            cx = sum_x / n
            cy = sum_y / n
            radius = sum(math.sqrt((p[0] - cx) ** 2 + (p[1] - cy) ** 2) for p in points) / n
            return (cx, cy), radius
        
        # Solve for a, b, c
        A_a = [[B[0], A[0][1], A[0][2]],
               [B[1], A[1][1], A[1][2]],
               [B[2], A[2][1], A[2][2]]]
        
        A_b = [[A[0][0], B[0], A[0][2]],
               [A[1][0], B[1], A[1][2]],
               [A[2][0], B[2], A[2][2]]]
        
        A_c = [[A[0][0], A[0][1], B[0]],
               [A[1][0], A[1][1], B[1]],
               [A[2][0], A[2][1], B[2]]]
        
        a = self._det3x3(A_a) / det_A / 2
        b = self._det3x3(A_b) / det_A / 2
        c = self._det3x3(A_c) / det_A
        
        # r^2 = c + a^2 + b^2
        r_squared = c + a ** 2 + b ** 2
        
        if r_squared < 0:
            return (a, b), None
        
        return (a, b), math.sqrt(r_squared)
    
    def _det3x3(self, m: List[List[float]]) -> float:
        """Calculate determinant of a 3x3 matrix."""
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    
    def _calculate_confidence(self, 
                               points: List[Point3D], 
                               center: Point3D, 
                               radius: float) -> float:
        """
        Calculate confidence score based on how well points fit the circle.
        
        Uses the standard deviation of distances from points to the circle.
        """
        if radius < 1e-10:
            return 0.0
        
        distances = [center.distance_to(p) for p in points]
        mean_dist = sum(distances) / len(distances)
        
        if mean_dist < 1e-10:
            return 0.0
        
        # Calculate variance
        variance = sum((d - radius) ** 2 for d in distances) / len(distances)
        std_dev = math.sqrt(variance)
        
        # Normalize by radius - lower relative std_dev = higher confidence
        relative_error = std_dev / radius
        
        # Convert to confidence score (0-1)
        confidence = max(0, 1 - relative_error * 5)
        
        return confidence
    
    def find_circles_near_point(self, 
                                 circles: List[DetectedCircle],
                                 point: Point3D,
                                 max_distance: float) -> List[DetectedCircle]:
        """
        Find circles whose centers are within max_distance of a point.
        
        Args:
            circles: List of detected circles
            point: Reference point
            max_distance: Maximum distance to consider
            
        Returns:
            List of nearby circles, sorted by distance
        """
        nearby = []
        for circle in circles:
            dist = circle.center.distance_to(point)
            if dist <= max_distance:
                nearby.append((dist, circle))
        
        nearby.sort(key=lambda x: x[0])
        return [c for _, c in nearby]


def detect_coplanar_circle_features(vertices: List[Tuple[float, float, float]],
                                     triangles: List[Tuple[int, int, int]],
                                     normals: Optional[List[Tuple[float, float, float]]] = None,
                                     angle_tolerance: float = 5.0) -> List[DetectedCircle]:
    """
    Alternative detection method that looks for circles by finding
    groups of coplanar triangles that form circular patterns.
    
    This is useful for detecting circles on flat surfaces of meshes
    where boundary detection might not work well.
    
    Args:
        vertices: Mesh vertices
        triangles: Triangle indices
        normals: Optional face normals
        angle_tolerance: Angle tolerance for coplanarity (degrees)
        
    Returns:
        List of detected circles
    """
    points = [Point3D(*v) for v in vertices]
    
    # Calculate face normals if not provided
    if normals is None:
        normals = []
        for tri in triangles:
            p0 = points[tri[0]]
            p1 = points[tri[1]]
            p2 = points[tri[2]]
            
            v1 = p1 - p0
            v2 = p2 - p0
            n = v1.cross(v2).normalized()
            normals.append((n.x, n.y, n.z))
    
    # Group faces by normal direction
    cos_tolerance = math.cos(math.radians(angle_tolerance))
    groups = []
    group_normals = []
    
    for i, (tri, normal) in enumerate(zip(triangles, normals)):
        n = Vector3D(*normal)
        
        found_group = False
        for j, gn in enumerate(group_normals):
            if abs(n.dot(gn)) >= cos_tolerance:
                groups[j].append(i)
                found_group = True
                break
        
        if not found_group:
            groups.append([i])
            group_normals.append(n)
    
    # Analyze each group for circular patterns
    detector = CircleDetector()
    all_circles = []
    
    for group_indices, group_normal in zip(groups, group_normals):
        if len(group_indices) < 4:
            continue
        
        # Get all vertices in this group
        group_vertices = set()
        for idx in group_indices:
            tri = triangles[idx]
            group_vertices.update(tri)
        
        # Find boundary vertices within this group
        edge_count = defaultdict(int)
        for idx in group_indices:
            tri = triangles[idx]
            edges = [
                tuple(sorted([tri[0], tri[1]])),
                tuple(sorted([tri[1], tri[2]])),
                tuple(sorted([tri[2], tri[0]]))
            ]
            for edge in edges:
                edge_count[edge] += 1
        
        # Internal boundary edges (appear once within group)
        boundary_vertices = set()
        for edge, count in edge_count.items():
            if count == 1:
                boundary_vertices.update(edge)
        
        if len(boundary_vertices) >= detector.min_points:
            boundary_points = [points[v] for v in boundary_vertices]
            circle = detector._fit_circle_to_loop(boundary_points)
            if circle and circle.confidence >= detector.circularity_threshold:
                all_circles.append(circle)
    
    return all_circles
