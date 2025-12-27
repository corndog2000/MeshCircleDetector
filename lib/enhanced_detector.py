"""
Enhanced Circle Detection for Mesh Bodies

This module provides additional detection methods for finding circular features
on mesh bodies, including:
- Cylindrical surface detection
- Flat circular face detection
- Edge-based circle detection with RANSAC

These methods complement the basic boundary loop detection.
"""

import math
from typing import List, Tuple, Optional, Set
from collections import defaultdict
import random

from .circle_detector import Point3D, Vector3D, DetectedCircle


class EnhancedCircleDetector:
    """
    Enhanced circle detector with multiple detection strategies.
    
    This detector uses several approaches:
    1. Boundary loop detection (holes and edges)
    2. Cylindrical surface detection
    3. Flat circular region detection
    4. RANSAC-based detection for noisy data
    """
    
    def __init__(self,
                 min_points: int = 8,
                 circularity_threshold: float = 0.80,
                 min_radius: float = 0.5,
                 max_radius: float = 500.0,
                 ransac_iterations: int = 100,
                 ransac_threshold: float = 0.1):
        """
        Initialize the enhanced detector.
        
        Args:
            min_points: Minimum points for circle fitting
            circularity_threshold: Required circularity score
            min_radius: Minimum circle radius
            max_radius: Maximum circle radius
            ransac_iterations: RANSAC iterations for robust fitting
            ransac_threshold: Inlier threshold for RANSAC (as fraction of radius)
        """
        self.min_points = min_points
        self.circularity_threshold = circularity_threshold
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.ransac_iterations = ransac_iterations
        self.ransac_threshold = ransac_threshold
    
    def detect_all_circles(self,
                           vertices: List[Tuple[float, float, float]],
                           triangles: List[Tuple[int, int, int]]) -> List[DetectedCircle]:
        """
        Detect circles using all available methods.
        
        Args:
            vertices: Mesh vertices
            triangles: Mesh triangles
            
        Returns:
            List of unique detected circles
        """
        points = [Point3D(*v) for v in vertices]
        all_circles = []
        
        # Method 1: Boundary loop detection
        boundary_circles = self._detect_boundary_circles(points, triangles)
        all_circles.extend(boundary_circles)
        
        # Method 2: Cylindrical surface detection
        cylinder_circles = self._detect_cylinder_edges(points, triangles)
        all_circles.extend(cylinder_circles)
        
        # Method 3: Flat circular region detection
        flat_circles = self._detect_flat_circles(points, triangles)
        all_circles.extend(flat_circles)
        
        # Remove duplicates
        unique_circles = self._remove_duplicates(all_circles)
        
        return unique_circles
    
    def _detect_boundary_circles(self,
                                  points: List[Point3D],
                                  triangles: List[Tuple[int, int, int]]) -> List[DetectedCircle]:
        """Detect circles from boundary edge loops."""
        # Find boundary edges
        edge_count = defaultdict(int)
        for tri in triangles:
            for i in range(3):
                edge = tuple(sorted([tri[i], tri[(i + 1) % 3]]))
                edge_count[edge] += 1
        
        boundary_edges = [e for e, c in edge_count.items() if c == 1]
        
        if not boundary_edges:
            return []
        
        # Build adjacency
        vertex_edges = defaultdict(list)
        for edge in boundary_edges:
            vertex_edges[edge[0]].append(edge)
            vertex_edges[edge[1]].append(edge)
        
        # Trace loops
        loops = []
        used = set()
        
        for start_edge in boundary_edges:
            if start_edge in used:
                continue
            
            loop = []
            current_edge = start_edge
            current_vertex = start_edge[0]
            
            while current_edge not in used:
                used.add(current_edge)
                loop.append(points[current_vertex])
                
                next_vertex = current_edge[1] if current_edge[0] == current_vertex else current_edge[0]
                
                next_edge = None
                for edge in vertex_edges[next_vertex]:
                    if edge not in used:
                        next_edge = edge
                        break
                
                if next_edge is None:
                    break
                
                current_vertex = next_vertex
                current_edge = next_edge
            
            if len(loop) >= self.min_points:
                loops.append(loop)
        
        # Fit circles to loops
        circles = []
        for loop in loops:
            circle = self._fit_circle_to_points(loop)
            if circle and circle.confidence >= self.circularity_threshold:
                if self.min_radius <= circle.radius <= self.max_radius:
                    circles.append(circle)
        
        return circles
    
    def _detect_cylinder_edges(self,
                                points: List[Point3D],
                                triangles: List[Tuple[int, int, int]]) -> List[DetectedCircle]:
        """
        Detect circles at the ends of cylindrical surfaces.
        
        This method identifies cylindrical regions by finding faces with
        similar normals pointing outward from a common axis.
        """
        # Calculate face normals and centroids
        faces = []
        for tri in triangles:
            p0, p1, p2 = points[tri[0]], points[tri[1]], points[tri[2]]
            
            v1 = p1 - p0
            v2 = p2 - p0
            normal = v1.cross(v2)
            
            length = normal.length()
            if length < 1e-10:
                continue
            
            normal = normal / length
            centroid = (p0 + p1 + p2) / 3
            
            faces.append({
                'indices': tri,
                'normal': normal,
                'centroid': centroid
            })
        
        # Group faces by normal similarity (for cylindrical surfaces, 
        # normals should radiate from a line)
        # This is a simplified approach - full cylinder detection would use RANSAC
        
        # Find edges where face normals change significantly
        edge_faces = defaultdict(list)
        for i, face in enumerate(faces):
            for j in range(3):
                edge = tuple(sorted([face['indices'][j], face['indices'][(j + 1) % 3]]))
                edge_faces[edge].append(i)
        
        # Look for edges at cylinder ends (where cylindrical meets flat)
        potential_rim_edges = []
        for edge, face_indices in edge_faces.items():
            if len(face_indices) == 2:
                f1, f2 = faces[face_indices[0]], faces[face_indices[1]]
                
                # Check if normals are significantly different
                dot = f1['normal'].dot(f2['normal'])
                if dot < 0.7:  # ~45 degree threshold
                    potential_rim_edges.append(edge)
        
        if not potential_rim_edges:
            return []
        
        # Try to form circles from connected rim edges
        vertex_rim_edges = defaultdict(list)
        for edge in potential_rim_edges:
            vertex_rim_edges[edge[0]].append(edge)
            vertex_rim_edges[edge[1]].append(edge)
        
        # Trace connected rim edges
        rim_loops = []
        used = set()
        
        for start_edge in potential_rim_edges:
            if start_edge in used:
                continue
            
            loop = []
            current = start_edge
            current_v = start_edge[0]
            
            while current not in used:
                used.add(current)
                loop.append(points[current_v])
                
                next_v = current[1] if current[0] == current_v else current[0]
                
                next_edge = None
                for edge in vertex_rim_edges[next_v]:
                    if edge not in used:
                        next_edge = edge
                        break
                
                if next_edge is None:
                    break
                
                current_v = next_v
                current = next_edge
            
            if len(loop) >= self.min_points:
                rim_loops.append(loop)
        
        # Fit circles
        circles = []
        for loop in rim_loops:
            circle = self._fit_circle_to_points(loop)
            if circle and circle.confidence >= self.circularity_threshold * 0.9:  # Slightly relaxed
                if self.min_radius <= circle.radius <= self.max_radius:
                    circles.append(circle)
        
        return circles
    
    def _detect_flat_circles(self,
                              points: List[Point3D],
                              triangles: List[Tuple[int, int, int]]) -> List[DetectedCircle]:
        """
        Detect circular flat regions (e.g., screw head tops, flat cylinder ends).
        """
        # Group coplanar faces
        face_groups = self._group_coplanar_faces(points, triangles)
        
        circles = []
        for group in face_groups:
            if len(group['triangles']) < 3:
                continue
            
            # Get boundary of this group
            edge_count = defaultdict(int)
            for tri in group['triangles']:
                for i in range(3):
                    edge = tuple(sorted([tri[i], tri[(i + 1) % 3]]))
                    edge_count[edge] += 1
            
            boundary_edges = [e for e, c in edge_count.items() if c == 1]
            
            if len(boundary_edges) < self.min_points:
                continue
            
            # Get boundary points
            boundary_vertices = set()
            for edge in boundary_edges:
                boundary_vertices.update(edge)
            
            boundary_points = [points[v] for v in boundary_vertices]
            
            # Try to fit circle
            circle = self._fit_circle_to_points(boundary_points)
            if circle and circle.confidence >= self.circularity_threshold:
                if self.min_radius <= circle.radius <= self.max_radius:
                    circles.append(circle)
        
        return circles
    
    def _group_coplanar_faces(self,
                               points: List[Point3D],
                               triangles: List[Tuple[int, int, int]],
                               angle_tolerance: float = 5.0) -> List[dict]:
        """Group faces by their plane orientation."""
        cos_tol = math.cos(math.radians(angle_tolerance))
        
        # Calculate face data
        face_data = []
        for tri in triangles:
            p0, p1, p2 = points[tri[0]], points[tri[1]], points[tri[2]]
            v1 = p1 - p0
            v2 = p2 - p0
            normal = v1.cross(v2)
            
            length = normal.length()
            if length < 1e-10:
                continue
            
            normal = normal / length
            centroid = (p0 + p1 + p2) / 3
            
            face_data.append({
                'tri': tri,
                'normal': normal,
                'centroid': centroid
            })
        
        # Group by normal direction
        groups = []
        group_normals = []
        
        for face in face_data:
            found = False
            for i, gn in enumerate(group_normals):
                if abs(face['normal'].dot(gn)) >= cos_tol:
                    groups[i]['triangles'].append(face['tri'])
                    found = True
                    break
            
            if not found:
                groups.append({
                    'normal': face['normal'],
                    'triangles': [face['tri']]
                })
                group_normals.append(face['normal'])
        
        return groups
    
    def _fit_circle_to_points(self, points: List[Point3D]) -> Optional[DetectedCircle]:
        """Fit a circle to a set of 3D points."""
        if len(points) < self.min_points:
            return None
        
        # Use RANSAC for robust fitting
        best_circle = None
        best_inliers = 0
        
        for _ in range(self.ransac_iterations):
            # Sample 3 random points
            if len(points) < 3:
                break
            
            sample = random.sample(points, min(3, len(points)))
            
            # Fit plane to sample
            if len(sample) < 3:
                continue
            
            v1 = sample[1] - sample[0]
            v2 = sample[2] - sample[0]
            normal = v1.cross(v2)
            
            length = normal.length()
            if length < 1e-10:
                continue
            
            normal = normal / length
            
            # Fit circle using all points projected onto this plane
            centroid = Point3D(
                sum(p.x for p in points) / len(points),
                sum(p.y for p in points) / len(points),
                sum(p.z for p in points) / len(points)
            )
            
            # Project to 2D
            if abs(normal.z) < 0.9:
                u = Vector3D(-normal.y, normal.x, 0)
            else:
                u = Vector3D(1, 0, 0)
            
            u_len = u.length()
            if u_len > 1e-10:
                u = u / u_len
            
            v = normal.cross(u)
            v_len = v.length()
            if v_len > 1e-10:
                v = v / v_len
            
            projected = []
            for p in points:
                diff = p - centroid
                x = diff.dot(u)
                y = diff.dot(v)
                projected.append((x, y))
            
            # Fit 2D circle
            center_2d, radius = self._fit_circle_2d(projected)
            
            if radius is None or radius < self.min_radius or radius > self.max_radius:
                continue
            
            # Count inliers
            inliers = 0
            threshold = radius * self.ransac_threshold
            
            for px, py in projected:
                dist = math.sqrt((px - center_2d[0])**2 + (py - center_2d[1])**2)
                if abs(dist - radius) < threshold:
                    inliers += 1
            
            if inliers > best_inliers:
                best_inliers = inliers
                
                # Transform center back to 3D
                center_3d = Point3D(
                    centroid.x + center_2d[0] * u.x + center_2d[1] * v.x,
                    centroid.y + center_2d[0] * u.y + center_2d[1] * v.y,
                    centroid.z + center_2d[0] * u.z + center_2d[1] * v.z
                )
                
                confidence = inliers / len(points)
                
                best_circle = DetectedCircle(
                    center=center_3d,
                    radius=radius,
                    normal=Vector3D(normal.x, normal.y, normal.z),
                    points=points,
                    confidence=confidence
                )
        
        return best_circle
    
    def _fit_circle_2d(self, points: List[Tuple[float, float]]) -> Tuple[Tuple[float, float], Optional[float]]:
        """Fit a 2D circle using algebraic method."""
        n = len(points)
        if n < 3:
            return (0, 0), None
        
        sum_x = sum(p[0] for p in points)
        sum_y = sum(p[1] for p in points)
        sum_x2 = sum(p[0]**2 for p in points)
        sum_y2 = sum(p[1]**2 for p in points)
        sum_xy = sum(p[0] * p[1] for p in points)
        sum_x3 = sum(p[0]**3 for p in points)
        sum_y3 = sum(p[1]**3 for p in points)
        sum_x2y = sum(p[0]**2 * p[1] for p in points)
        sum_xy2 = sum(p[0] * p[1]**2 for p in points)
        
        A = [[sum_x2, sum_xy, sum_x],
             [sum_xy, sum_y2, sum_y],
             [sum_x, sum_y, n]]
        
        B = [sum_x3 + sum_xy2, sum_x2y + sum_y3, sum_x2 + sum_y2]
        
        det_A = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                 A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                 A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
        
        if abs(det_A) < 1e-10:
            cx = sum_x / n
            cy = sum_y / n
            radius = sum(math.sqrt((p[0] - cx)**2 + (p[1] - cy)**2) for p in points) / n
            return (cx, cy), radius
        
        # Cramer's rule
        def det3(m):
            return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
        
        A_a = [[B[0], A[0][1], A[0][2]],
               [B[1], A[1][1], A[1][2]],
               [B[2], A[2][1], A[2][2]]]
        
        A_b = [[A[0][0], B[0], A[0][2]],
               [A[1][0], B[1], A[1][2]],
               [A[2][0], B[2], A[2][2]]]
        
        A_c = [[A[0][0], A[0][1], B[0]],
               [A[1][0], A[1][1], B[1]],
               [A[2][0], A[2][1], B[2]]]
        
        a = det3(A_a) / det_A / 2
        b = det3(A_b) / det_A / 2
        c = det3(A_c) / det_A
        
        r_sq = c + a**2 + b**2
        if r_sq < 0:
            return (a, b), None
        
        return (a, b), math.sqrt(r_sq)
    
    def _remove_duplicates(self, circles: List[DetectedCircle],
                           center_tolerance: float = 0.5,
                           radius_tolerance: float = 0.1) -> List[DetectedCircle]:
        """Remove duplicate circles based on center and radius similarity."""
        if not circles:
            return []
        
        unique = []
        for circle in circles:
            is_duplicate = False
            for existing in unique:
                center_dist = circle.center.distance_to(existing.center)
                radius_diff = abs(circle.radius - existing.radius)
                
                if center_dist < center_tolerance and radius_diff < radius_tolerance * circle.radius:
                    # Keep the one with higher confidence
                    if circle.confidence > existing.confidence:
                        unique.remove(existing)
                        unique.append(circle)
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique.append(circle)
        
        return unique
