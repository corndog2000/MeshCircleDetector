"""
Custom Graphics for Mesh Circle Detector

This module provides utilities for rendering detected circles and their
center points as custom graphics in the Fusion 360 viewport.
"""

import adsk.core
import adsk.fusion
import math
from typing import List, Optional, Dict
from .circle_detector import DetectedCircle, Point3D, Vector3D
from .mesh_utils import point3d_to_fusion, vector3d_to_fusion


class CircleGraphicsManager:
    """
    Manages custom graphics for displaying detected circles.
    
    Creates and updates graphics showing:
    - Circle outlines
    - Center points
    - Normal direction indicators
    - Highlight effects for circles near cursor
    """
    
    # Color definitions (RGBA, 0-255)
    COLOR_CIRCLE_DEFAULT = (0, 150, 255, 200)      # Blue
    COLOR_CIRCLE_HIGHLIGHT = (255, 200, 0, 255)    # Yellow/Gold
    COLOR_CIRCLE_SELECTED = (0, 255, 100, 255)     # Green
    COLOR_CENTER_POINT = (255, 100, 0, 255)        # Orange
    COLOR_NORMAL_LINE = (150, 150, 150, 180)       # Gray
    
    def __init__(self, app: adsk.core.Application):
        """
        Initialize the graphics manager.
        
        Args:
            app: Fusion 360 Application object
        """
        self.app = app
        self.graphics_group: Optional[adsk.fusion.CustomGraphicsGroup] = None
        self.circle_graphics: Dict[int, adsk.fusion.CustomGraphicsEntity] = {}
        self.center_graphics: Dict[int, adsk.fusion.CustomGraphicsEntity] = {}
        self._highlighted_index: Optional[int] = None
        self._selected_indices: List[int] = []
    
    def initialize(self, design: adsk.fusion.Design) -> bool:
        """
        Initialize graphics for the given design.
        
        Args:
            design: The active Fusion 360 design
            
        Returns:
            True if initialization succeeded
        """
        try:
            root_comp = design.rootComponent
            self.graphics_group = root_comp.customGraphicsGroups.add()
            return True
        except Exception as e:
            self.app.log(f"Failed to initialize graphics: {e}")
            return False
    
    def clear_all(self):
        """Clear all graphics."""
        if self.graphics_group:
            try:
                self.graphics_group.deleteMe()
            except:
                pass
        self.graphics_group = None
        self.circle_graphics.clear()
        self.center_graphics.clear()
        self._highlighted_index = None
        self._selected_indices.clear()
    
    def display_circles(self, circles: List[DetectedCircle], 
                        show_centers: bool = True,
                        show_normals: bool = False,
                        circle_segments: int = 64):
        """
        Display detected circles as custom graphics.
        
        Args:
            circles: List of detected circles to display
            show_centers: Whether to show center points
            show_normals: Whether to show normal direction lines
            circle_segments: Number of line segments per circle
        """
        if not self.graphics_group:
            return
        
        # Clear existing graphics
        for entity in list(self.circle_graphics.values()):
            try:
                entity.deleteMe()
            except:
                pass
        self.circle_graphics.clear()
        
        for entity in list(self.center_graphics.values()):
            try:
                entity.deleteMe()
            except:
                pass
        self.center_graphics.clear()
        
        # Create graphics for each circle
        for i, circle in enumerate(circles):
            self._create_circle_graphics(i, circle, circle_segments, show_centers, show_normals)
    
    def _create_circle_graphics(self, index: int, circle: DetectedCircle,
                                 segments: int, show_center: bool, show_normal: bool):
        """Create graphics for a single circle."""
        if not self.graphics_group:
            return
        
        # Determine color based on state
        if index in self._selected_indices:
            color = self.COLOR_CIRCLE_SELECTED
        elif index == self._highlighted_index:
            color = self.COLOR_CIRCLE_HIGHLIGHT
        else:
            color = self.COLOR_CIRCLE_DEFAULT
        
        # Create coordinate system for the circle plane
        normal = circle.normal
        
        # Find perpendicular vectors
        if abs(normal.z) < 0.9:
            u = Vector3D(-normal.y, normal.x, 0)
        else:
            u = Vector3D(1, 0, 0)
        
        u_length = u.length()
        if u_length > 1e-10:
            u = Point3D(u.x / u_length, u.y / u_length, u.z / u_length)
        
        v = normal.cross(u)
        v_length = v.length()
        if v_length > 1e-10:
            v = Point3D(v.x / v_length, v.y / v_length, v.z / v_length)
        
        # Generate circle points
        coords = adsk.fusion.CustomGraphicsCoordinates.create()
        coord_list = []
        
        for i in range(segments + 1):
            angle = 2 * math.pi * i / segments
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            
            # Point on circle
            px = circle.center.x + circle.radius * (cos_a * u.x + sin_a * v.x)
            py = circle.center.y + circle.radius * (cos_a * u.y + sin_a * v.y)
            pz = circle.center.z + circle.radius * (cos_a * u.z + sin_a * v.z)
            
            coord_list.extend([px, py, pz])
        
        coords.setCoordinates(coord_list)
        
        # Create line strip for circle
        lines = self.graphics_group.addLines(coords, 
                                              [i for i in range(segments + 1)],
                                              True)  # isLineStrip
        
        # Set color
        color_effect = adsk.fusion.CustomGraphicsSolidColorEffect.create(
            adsk.core.Color.create(color[0], color[1], color[2], color[3])
        )
        lines.color = color_effect
        lines.weight = 3.0 if index == self._highlighted_index else 2.0
        
        self.circle_graphics[index] = lines
        
        # Create center point
        if show_center:
            center_coords = adsk.fusion.CustomGraphicsCoordinates.create()
            center_coords.setCoordinates([circle.center.x, circle.center.y, circle.center.z])
            
            # Create point cloud for center
            point_set = self.graphics_group.addPointSet(center_coords, 
                                                         [0],
                                                         adsk.fusion.CustomGraphicsPointTypes.UserDefinedCustomGraphicsPointType,
                                                         None)
            
            center_color = adsk.fusion.CustomGraphicsSolidColorEffect.create(
                adsk.core.Color.create(*self.COLOR_CENTER_POINT)
            )
            point_set.color = center_color
            point_set.pointSize = 8.0
            
            self.center_graphics[index] = point_set
        
        # Create normal line
        if show_normal:
            normal_length = circle.radius * 0.5
            normal_end = Point3D(
                circle.center.x + normal.x * normal_length,
                circle.center.y + normal.y * normal_length,
                circle.center.z + normal.z * normal_length
            )
            
            normal_coords = adsk.fusion.CustomGraphicsCoordinates.create()
            normal_coords.setCoordinates([
                circle.center.x, circle.center.y, circle.center.z,
                normal_end.x, normal_end.y, normal_end.z
            ])
            
            normal_line = self.graphics_group.addLines(normal_coords, [0, 1], False)
            normal_color = adsk.fusion.CustomGraphicsSolidColorEffect.create(
                adsk.core.Color.create(*self.COLOR_NORMAL_LINE)
            )
            normal_line.color = normal_color
            normal_line.weight = 1.0
    
    def highlight_circle(self, index: Optional[int], circles: List[DetectedCircle],
                         show_centers: bool = True):
        """
        Highlight a specific circle.
        
        Args:
            index: Index of circle to highlight, or None to clear highlight
            circles: List of all circles (needed for redraw)
            show_centers: Whether to show center points
        """
        if index != self._highlighted_index:
            self._highlighted_index = index
            self.display_circles(circles, show_centers)
    
    def toggle_selection(self, index: int, circles: List[DetectedCircle],
                         show_centers: bool = True):
        """
        Toggle selection state of a circle.
        
        Args:
            index: Index of circle to toggle
            circles: List of all circles
            show_centers: Whether to show center points
        """
        if index in self._selected_indices:
            self._selected_indices.remove(index)
        else:
            self._selected_indices.append(index)
        
        self.display_circles(circles, show_centers)
    
    def get_selected_indices(self) -> List[int]:
        """Get list of selected circle indices."""
        return self._selected_indices.copy()
    
    def clear_selection(self, circles: List[DetectedCircle], show_centers: bool = True):
        """Clear all selections."""
        self._selected_indices.clear()
        self.display_circles(circles, show_centers)
    
    def refresh(self):
        """Force a viewport refresh."""
        try:
            self.app.activeViewport.refresh()
        except:
            pass


class ProgressIndicator:
    """Simple progress indicator for long operations."""
    
    def __init__(self, app: adsk.core.Application, title: str, max_value: int):
        """
        Initialize progress indicator.
        
        Args:
            app: Fusion 360 Application
            title: Progress dialog title
            max_value: Maximum progress value
        """
        self.app = app
        self.progress = app.userInterface.createProgressDialog()
        self.progress.show(title, '', 0, max_value, 1)
    
    def update(self, value: int, message: str = ''):
        """Update progress value and optional message."""
        self.progress.progressValue = value
        if message:
            self.progress.message = message
        adsk.doEvents()
    
    def close(self):
        """Close the progress dialog."""
        self.progress.hide()
