# MeshCircleDetector - Fusion 360 Add-in
# Detects circular features on mesh bodies and projects to sketches
# Version 2.0 - List-based selection (more reliable)

import adsk
import adsk.core
import adsk.fusion
import traceback
import math
import os

# Global variables
_app = None
_ui = None
_handlers = []
_detected_circles = []
_selected_indices = []
_cmd_inputs = None  # Store reference to command inputs

# Global references to key inputs (set during command creation)
_input_refs = {}

# Debug logging
_debug_file = None

def log(msg):
    global _debug_file
    if _debug_file is None:
        try:
            addon_folder = os.path.dirname(os.path.realpath(__file__))
            _debug_file = open(os.path.join(addon_folder, 'debug.log'), 'w')
            _debug_file.write('=' * 50 + '\n')
        except:
            return
    try:
        _debug_file.write(str(msg) + '\n')
        _debug_file.flush()
    except:
        pass

def log_exception(context):
    log(f"EXCEPTION in {context}:")
    log(traceback.format_exc())


# ============================================================================
# Simple Geometry Classes
# ============================================================================

class Point3D:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
    
    def distance_to(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def __repr__(self):
        return f"Point3D({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"


class Vector3D:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
    
    def length(self):
        return math.sqrt(self.x*self.x + self.y*self.y + self.z*self.z)
    
    def normalize(self):
        length = self.length()
        if length > 1e-10:
            self.x /= length
            self.y /= length
            self.z /= length
        return self
    
    def dot(self, other):
        return self.x*other.x + self.y*other.y + self.z*other.z
    
    def cross(self, other):
        return Vector3D(
            self.y*other.z - self.z*other.y,
            self.z*other.x - self.x*other.z,
            self.x*other.y - self.y*other.x
        )


class DetectedCircle:
    def __init__(self, center, normal, radius, confidence=1.0):
        self.center = center
        self.normal = normal
        self.radius = radius
        self.confidence = confidence
    
    def __repr__(self):
        return f"Circle(r={self.radius:.3f}, center={self.center})"


# ============================================================================
# Mesh Data Extraction
# ============================================================================

def get_mesh_data(mesh_body):
    """Extract vertices and triangles from mesh body."""
    display_mesh = mesh_body.displayMesh
    
    mesh_coords = display_mesh.nodeCoordinatesAsFloat
    mesh_indices = display_mesh.nodeIndices
    
    vertices = []
    for i in range(0, len(mesh_coords), 3):
        vertices.append(Point3D(mesh_coords[i], mesh_coords[i+1], mesh_coords[i+2]))
    
    triangles = []
    for i in range(0, len(mesh_indices), 3):
        triangles.append((mesh_indices[i], mesh_indices[i+1], mesh_indices[i+2]))
    
    return vertices, triangles


# ============================================================================
# Circle Detection Algorithm
# ============================================================================

def find_boundary_edges(triangles):
    """Find edges that belong to only one triangle (boundary edges)."""
    edge_count = {}
    edge_triangles = {}
    
    for tri_idx, (v0, v1, v2) in enumerate(triangles):
        if tri_idx % 10000 == 0:  # Keep UI responsive
            adsk.doEvents()
        
        edges = [(v0, v1), (v1, v2), (v2, v0)]
        for e in edges:
            edge = tuple(sorted(e))
            edge_count[edge] = edge_count.get(edge, 0) + 1
            if edge not in edge_triangles:
                edge_triangles[edge] = []
            edge_triangles[edge].append(tri_idx)
    
    return [e for e, count in edge_count.items() if count == 1]


def trace_loops(boundary_edges):
    """Connect boundary edges into closed loops."""
    if not boundary_edges:
        return []
    
    adj = {}
    for v0, v1 in boundary_edges:
        if v0 not in adj:
            adj[v0] = []
        if v1 not in adj:
            adj[v1] = []
        adj[v0].append(v1)
        adj[v1].append(v0)
    
    used_edges = set()
    loops = []
    max_iterations = len(boundary_edges) * 2  # Safety limit
    edge_count = 0
    
    for start_edge in boundary_edges:
        edge_count += 1
        if edge_count % 1000 == 0:  # Keep UI responsive
            adsk.doEvents()
        
        edge_key = tuple(sorted(start_edge))
        if edge_key in used_edges:
            continue
        
        loop = [start_edge[0], start_edge[1]]
        used_edges.add(edge_key)
        
        iterations = 0
        while iterations < max_iterations:
            iterations += 1
            current = loop[-1]
            if current not in adj:
                break
            
            found_next = False
            for next_v in adj[current]:
                edge_key = tuple(sorted((current, next_v)))
                if edge_key not in used_edges:
                    used_edges.add(edge_key)  # Mark edge as used immediately
                    if next_v == loop[0] and len(loop) >= 3:
                        # Closed the loop
                        loops.append(loop)
                        found_next = True
                        break
                    loop.append(next_v)
                    found_next = True
                    break
            
            if not found_next:
                break
            
            # Check if we just closed a loop
            if loops and loops[-1] is loop:
                break
    
    return loops


def fit_plane(points):
    """Fit a plane to points using centroid and average normal."""
    n = len(points)
    if n < 3:
        return None, None
    
    cx = sum(p.x for p in points) / n
    cy = sum(p.y for p in points) / n
    cz = sum(p.z for p in points) / n
    centroid = Point3D(cx, cy, cz)
    
    normal = Vector3D(0, 0, 0)
    for i in range(n):
        p1 = points[i]
        p2 = points[(i + 1) % n]
        normal.x += (p1.y - cy) * (p2.z - cz) - (p1.z - cz) * (p2.y - cy)
        normal.y += (p1.z - cz) * (p2.x - cx) - (p1.x - cx) * (p2.z - cz)
        normal.z += (p1.x - cx) * (p2.y - cy) - (p1.y - cy) * (p2.x - cx)
    
    normal.normalize()
    return centroid, normal


def fit_circle_2d(points_2d):
    """Fit circle to 2D points using least squares."""
    n = len(points_2d)
    if n < 3:
        return None, None
    
    sum_x = sum(p[0] for p in points_2d)
    sum_y = sum(p[1] for p in points_2d)
    sum_x2 = sum(p[0]**2 for p in points_2d)
    sum_y2 = sum(p[1]**2 for p in points_2d)
    sum_xy = sum(p[0]*p[1] for p in points_2d)
    sum_x3 = sum(p[0]**3 for p in points_2d)
    sum_y3 = sum(p[1]**3 for p in points_2d)
    sum_x2y = sum(p[0]**2*p[1] for p in points_2d)
    sum_xy2 = sum(p[0]*p[1]**2 for p in points_2d)
    
    A = n * sum_x2 - sum_x**2
    B = n * sum_xy - sum_x * sum_y
    C = n * sum_y2 - sum_y**2
    D = 0.5 * (n * (sum_x3 + sum_xy2) - sum_x * (sum_x2 + sum_y2))
    E = 0.5 * (n * (sum_x2y + sum_y3) - sum_y * (sum_x2 + sum_y2))
    
    denom = A * C - B * B
    if abs(denom) < 1e-10:
        return None, None
    
    cx = (D * C - B * E) / denom
    cy = (A * E - B * D) / denom
    
    r2_sum = sum((p[0] - cx)**2 + (p[1] - cy)**2 for p in points_2d)
    radius = math.sqrt(r2_sum / n)
    
    return (cx, cy), radius


def project_to_plane(point, centroid, normal, u_axis, v_axis):
    """Project 3D point to 2D plane coordinates."""
    dx = point.x - centroid.x
    dy = point.y - centroid.y
    dz = point.z - centroid.z
    
    u = dx * u_axis.x + dy * u_axis.y + dz * u_axis.z
    v = dx * v_axis.x + dy * v_axis.y + dz * v_axis.z
    return (u, v)


def fit_circle_to_loop(vertices, loop_indices, min_points=8, circularity_threshold=0.75):
    """Fit a circle to a loop of vertices."""
    if len(loop_indices) < min_points:
        return None
    
    points = [vertices[i] for i in loop_indices]
    centroid, normal = fit_plane(points)
    
    if centroid is None:
        return None
    
    if abs(normal.z) < 0.9:
        u_axis = Vector3D(-normal.y, normal.x, 0)
    else:
        u_axis = Vector3D(1, 0, 0)
    u_axis.normalize()
    
    v_axis = normal.cross(u_axis)
    v_axis.normalize()
    
    points_2d = [project_to_plane(p, centroid, normal, u_axis, v_axis) for p in points]
    
    center_2d, radius = fit_circle_2d(points_2d)
    if center_2d is None or radius < 1e-6:
        return None
    
    errors = [abs(math.sqrt((p[0] - center_2d[0])**2 + (p[1] - center_2d[1])**2) - radius) for p in points_2d]
    max_error = max(errors)
    avg_error = sum(errors) / len(errors)
    
    circularity = 1.0 - (avg_error / radius) if radius > 0 else 0
    
    if circularity < circularity_threshold:
        return None
    
    center_3d = Point3D(
        centroid.x + center_2d[0] * u_axis.x + center_2d[1] * v_axis.x,
        centroid.y + center_2d[0] * u_axis.y + center_2d[1] * v_axis.y,
        centroid.z + center_2d[0] * u_axis.z + center_2d[1] * v_axis.z
    )
    
    return DetectedCircle(center_3d, normal, radius, circularity)


def detect_circles(vertices, triangles, min_points=8, circularity_threshold=0.75, min_radius=0.01, max_radius=1000.0):
    """Detect circles in mesh from boundary loops."""
    log(f"  Finding boundary edges from {len(triangles)} triangles...")
    boundary_edges = find_boundary_edges(triangles)
    log(f"  Found {len(boundary_edges)} boundary edges")
    
    # Keep UI responsive
    adsk.doEvents()
    
    log(f"  Tracing loops...")
    loops = trace_loops(boundary_edges)
    log(f"  Found {len(loops)} loops")
    
    # Keep UI responsive
    adsk.doEvents()
    
    circles = []
    log(f"  Fitting circles to loops...")
    for i, loop in enumerate(loops):
        if i % 50 == 0:  # Update every 50 loops
            adsk.doEvents()
        
        circle = fit_circle_to_loop(vertices, loop, min_points, circularity_threshold)
        if circle and min_radius <= circle.radius <= max_radius:
            circles.append(circle)
    
    log(f"  Fitted {len(circles)} circles")
    return circles


def remove_duplicates(circles, center_tol=0.1, radius_tol=0.05):
    """Remove duplicate circles based on center and radius similarity."""
    if not circles:
        return []
    
    unique = []
    for circle in circles:
        is_duplicate = False
        for existing in unique:
            center_dist = circle.center.distance_to(existing.center)
            radius_diff = abs(circle.radius - existing.radius)
            if center_dist < center_tol and radius_diff < radius_tol:
                is_duplicate = True
                if circle.confidence > existing.confidence:
                    unique.remove(existing)
                    unique.append(circle)
                break
        if not is_duplicate:
            unique.append(circle)
    
    return unique


# ============================================================================
# Detection Parameters
# ============================================================================

DETECTION_PARAMS = {
    'min_points': 8,
    'circularity_threshold': 0.75,
    'min_radius': 0.01,
    'max_radius': 1000.0
}


# ============================================================================
# Command Handlers
# ============================================================================

class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        global _input_refs
        try:
            log("CommandCreatedHandler.notify called")
            cmd = args.command
            inputs = cmd.commandInputs
            
            # Clear old refs
            _input_refs = {}
            
            # Detection group
            detect_group = inputs.addGroupCommandInput('detectGroup', 'Detection')
            detect_group.isExpanded = True
            detect_inputs = detect_group.children
            
            # Detect button
            detect_inputs.addBoolValueInput('detectButton', 'Detect Circles', False, '', True)
            
            # Status text
            status_text = detect_inputs.addTextBoxCommandInput('statusText', 'Status', 'Click Detect to scan mesh', 1, True)
            _input_refs['statusText'] = status_text
            
            # Settings group
            settings_group = inputs.addGroupCommandInput('settingsGroup', 'Settings')
            settings_group.isExpanded = False
            settings_inputs = settings_group.children
            
            # Circularity threshold
            circ_slider = settings_inputs.addFloatSliderCommandInput('circularitySlider', 'Circularity', '', 0.5, 1.0, False)
            circ_slider.valueOne = DETECTION_PARAMS['circularity_threshold']
            _input_refs['circularitySlider'] = circ_slider
            
            # Min radius
            min_rad_input = settings_inputs.addValueInput('minRadius', 'Min Radius', 'cm', 
                adsk.core.ValueInput.createByReal(DETECTION_PARAMS['min_radius']))
            _input_refs['minRadius'] = min_rad_input
            
            # Max radius
            max_rad_input = settings_inputs.addValueInput('maxRadius', 'Max Radius', 'cm',
                adsk.core.ValueInput.createByReal(DETECTION_PARAMS['max_radius']))
            _input_refs['maxRadius'] = max_rad_input
            
            # Selection group
            select_group = inputs.addGroupCommandInput('selectGroup', 'Selection')
            select_group.isExpanded = True
            select_inputs = select_group.children
            
            # Circle list dropdown
            circle_dropdown = select_inputs.addDropDownCommandInput('circleDropdown', 'Available Circles', 
                adsk.core.DropDownStyles.CheckBoxDropDownStyle)
            circle_dropdown.listItems.add('(Run detection first)', False, '')
            _input_refs['circleDropdown'] = circle_dropdown
            
            # Selection count
            selection_text = select_inputs.addTextBoxCommandInput('selectionText', 'Selected', '0 circles selected', 1, True)
            _input_refs['selectionText'] = selection_text
            
            # Select all / clear buttons
            select_inputs.addBoolValueInput('selectAllButton', 'Select All', False, '', True)
            select_inputs.addBoolValueInput('clearButton', 'Clear Selection', False, '', True)
            
            # Zoom to first selected circle
            select_inputs.addBoolValueInput('zoomToButton', 'Zoom To First Selected', False, '', True)
            
            # Projection group
            project_group = inputs.addGroupCommandInput('projectGroup', 'Projection')
            project_group.isExpanded = True
            project_inputs = project_group.children
            
            # Target sketch selection
            sketch_sel = project_inputs.addSelectionInput('sketchSel', 'Target Sketch', 'Select sketch to project circles')
            sketch_sel.addSelectionFilter('Sketches')
            sketch_sel.setSelectionLimits(0, 1)  # Make optional
            _input_refs['sketchSel'] = sketch_sel
            
            # Help text
            project_inputs.addTextBoxCommandInput('projectHelp', '', 
                'Select circles, choose sketch, then click OK to project', 1, True)
            
            # Add handlers
            on_input_changed = InputChangedHandler()
            cmd.inputChanged.add(on_input_changed)
            _handlers.append(on_input_changed)
            
            on_execute = CommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
            
            on_destroy = CommandDestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
            
            log("CommandCreatedHandler.notify completed")
            
        except:
            log_exception("CommandCreatedHandler")


def find_input(inputs, input_id):
    """Find an input by ID, using global refs or searching within groups."""
    global _input_refs
    
    # Try global reference first, but verify it's still valid
    if input_id in _input_refs:
        ref = _input_refs[input_id]
        if ref:
            try:
                # Try to access a property to verify the reference is still valid
                _ = ref.id
                log(f"find_input: Found '{input_id}' via global ref")
                return ref
            except:
                log(f"Global ref for '{input_id}' is stale, searching...")
    
    # Try direct lookup
    result = inputs.itemById(input_id)
    if result:
        log(f"find_input: Found '{input_id}' via direct lookup")
        return result
    
    log(f"find_input: Searching {inputs.count} groups for '{input_id}'")
    
    # Search within groups - iterate through all inputs
    for i in range(inputs.count):
        inp = inputs.item(i)
        log(f"  Group {i}: {inp.id}")
        # Try to cast to GroupCommandInput
        group_input = adsk.core.GroupCommandInput.cast(inp)
        if group_input:
            log(f"    Has {group_input.children.count} children")
            # Search in this group's children
            result = group_input.children.itemById(input_id)
            if result:
                log(f"find_input: Found '{input_id}' in group '{inp.id}'")
                return result
            # Also log what children exist
            for j in range(group_input.children.count):
                child = group_input.children.item(j)
                log(f"      Child: {child.id}")
    
    log(f"WARNING: Could not find input '{input_id}'")
    return None


class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        try:
            input_changed = args.input
            inputs = args.inputs
            log(f"InputChangedHandler: {input_changed.id}")
            
            if input_changed.id == 'detectButton':
                self._run_detection(inputs)
            
            elif input_changed.id == 'selectAllButton':
                self._select_all(inputs)
            
            elif input_changed.id == 'clearButton':
                self._clear_selection(inputs)
            
            elif input_changed.id == 'circleDropdown':
                self._update_selection_count(inputs)
            
            elif input_changed.id == 'zoomToButton':
                self._zoom_to_selected(inputs)
                
        except:
            log_exception("InputChangedHandler")
    
    def _run_detection(self, inputs):
        global _detected_circles, _selected_indices
        
        try:
            # Update parameters from inputs (search within groups)
            circ_slider = find_input(inputs, 'circularitySlider')
            min_rad = find_input(inputs, 'minRadius')
            max_rad = find_input(inputs, 'maxRadius')
            
            if circ_slider:
                DETECTION_PARAMS['circularity_threshold'] = circ_slider.valueOne
            if min_rad:
                DETECTION_PARAMS['min_radius'] = min_rad.value
            if max_rad:
                DETECTION_PARAMS['max_radius'] = max_rad.value
            
            design = adsk.fusion.Design.cast(_app.activeProduct)
            if not design:
                find_input(inputs, 'statusText').formattedText = 'No active design'
                return
            
            # Find mesh bodies
            mesh_bodies = []
            for mesh in design.rootComponent.meshBodies:
                mesh_bodies.append(mesh)
            for occ in design.rootComponent.allOccurrences:
                for mesh in occ.component.meshBodies:
                    mesh_bodies.append(mesh)
            
            log(f"Found {len(mesh_bodies)} mesh bodies")
            
            if not mesh_bodies:
                find_input(inputs, 'statusText').formattedText = 'No mesh bodies found'
                return
            
            find_input(inputs, 'statusText').formattedText = 'Detecting...'
            adsk.doEvents()
            
            # Detect circles
            _detected_circles = []
            _selected_indices = []
            
            for mesh in mesh_bodies:
                try:
                    log(f"Analyzing mesh: {mesh.name}")
                    vertices, triangles = get_mesh_data(mesh)
                    log(f"  Vertices: {len(vertices)}, Triangles: {len(triangles)}")
                    
                    circles = detect_circles(
                        vertices, triangles,
                        min_points=DETECTION_PARAMS['min_points'],
                        circularity_threshold=DETECTION_PARAMS['circularity_threshold'],
                        min_radius=DETECTION_PARAMS['min_radius'],
                        max_radius=DETECTION_PARAMS['max_radius']
                    )
                    log(f"  Found {len(circles)} circles before dedup")
                    
                    _detected_circles.extend(circles)
                except Exception as e:
                    log_exception(f"detect on mesh {mesh.name}")
            
            # Remove duplicates
            _detected_circles = remove_duplicates(_detected_circles)
            log(f"Total unique circles: {len(_detected_circles)}")
            
            # Sort by radius for easier selection (largest first)
            _detected_circles.sort(key=lambda c: c.radius, reverse=True)
            
            # Populate dropdown
            log(f"Looking for dropdown, _input_refs keys: {list(_input_refs.keys())}")
            dropdown = find_input(inputs, 'circleDropdown')
            log(f"Dropdown result: {dropdown}")
            
            if not dropdown:
                log("ERROR: Could not find circleDropdown!")
                _ui.messageBox(f'Found {len(_detected_circles)} circles but UI error.\nCheck debug.log for details.')
                return
            
            dropdown.listItems.clear()
            
            if _detected_circles:
                for i, circle in enumerate(_detected_circles):
                    # Format: "R=0.310 @ (2.9, 0.0, -0.8)"
                    label = f"R={circle.radius:.3f} @ ({circle.center.x:.2f}, {circle.center.y:.2f}, {circle.center.z:.2f})"
                    dropdown.listItems.add(label, False, '')
                
                status = find_input(inputs, 'statusText')
                if status:
                    status.formattedText = f'Found {len(_detected_circles)} unique circles'
            else:
                dropdown.listItems.add('(No circles found)', False, '')
                status = find_input(inputs, 'statusText')
                if status:
                    status.formattedText = 'No circles detected'
            
            sel_text = find_input(inputs, 'selectionText')
            if sel_text:
                sel_text.formattedText = '0 circles selected'
        
        except:
            log_exception("_run_detection")
            status = find_input(inputs, 'statusText')
            if status:
                status.formattedText = 'Detection failed'
    
    def _select_all(self, inputs):
        global _selected_indices
        
        dropdown = find_input(inputs, 'circleDropdown')
        count = 0
        for i in range(dropdown.listItems.count):
            item = dropdown.listItems.item(i)
            if not item.name.startswith('('):  # Skip placeholder items
                item.isSelected = True
                count += 1
        
        find_input(inputs, 'selectionText').formattedText = f'{count} circles selected'
    
    def _clear_selection(self, inputs):
        global _selected_indices
        
        dropdown = find_input(inputs, 'circleDropdown')
        for i in range(dropdown.listItems.count):
            dropdown.listItems.item(i).isSelected = False
        
        find_input(inputs, 'selectionText').formattedText = '0 circles selected'
    
    def _update_selection_count(self, inputs):
        dropdown = find_input(inputs, 'circleDropdown')
        count = sum(1 for i in range(dropdown.listItems.count) if dropdown.listItems.item(i).isSelected)
        find_input(inputs, 'selectionText').formattedText = f'{count} circles selected'
    
    def _zoom_to_selected(self, inputs):
        """Zoom the camera to center on the first selected circle."""
        global _detected_circles
        
        try:
            # Get selected circles
            dropdown = find_input(inputs, 'circleDropdown')
            selected_idx = None
            for i in range(dropdown.listItems.count):
                if dropdown.listItems.item(i).isSelected:
                    selected_idx = i
                    break  # Get first selected
            
            if selected_idx is None or selected_idx >= len(_detected_circles):
                _ui.messageBox('No circle selected. Select a circle from the dropdown first.')
                return
            
            circle = _detected_circles[selected_idx]
            log(f"Zooming to circle {selected_idx}: center=({circle.center.x:.2f}, {circle.center.y:.2f}, {circle.center.z:.2f}), radius={circle.radius:.3f}")
            
            # Get the camera
            viewport = _app.activeViewport
            camera = viewport.camera
            
            # Set the target point to the circle center
            target = adsk.core.Point3D.create(circle.center.x, circle.center.y, circle.center.z)
            
            # Calculate eye position - back off along the circle normal
            # Use a distance that shows the circle nicely (about 5x the radius)
            view_distance = max(circle.radius * 8, 2.0)  # At least 2cm away
            
            eye = adsk.core.Point3D.create(
                circle.center.x + circle.normal.x * view_distance,
                circle.center.y + circle.normal.y * view_distance,
                circle.center.z + circle.normal.z * view_distance
            )
            
            # Set up vector (try to keep "up" as Z or Y)
            if abs(circle.normal.z) > 0.9:
                up = adsk.core.Vector3D.create(0, 1, 0)
            else:
                up = adsk.core.Vector3D.create(0, 0, 1)
            
            # Apply camera changes
            camera.eye = eye
            camera.target = target
            camera.upVector = up
            camera.isSmoothTransition = True
            
            # Set view extents to show the circle
            camera.viewExtents = circle.radius * 4
            
            viewport.camera = camera
            viewport.refresh()
            
            log(f"Camera moved to view circle {selected_idx}")
            
        except:
            log_exception("_zoom_to_selected")
    


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        global _detected_circles
        log("CommandExecuteHandler.notify called")
        
        try:
            inputs = args.command.commandInputs
            
            # Get selected circles from dropdown
            dropdown = find_input(inputs, 'circleDropdown')
            if not dropdown:
                log("No dropdown found")
                return
            
            selected_indices = []
            for i in range(dropdown.listItems.count):
                if dropdown.listItems.item(i).isSelected:
                    selected_indices.append(i)
            
            if not selected_indices:
                log("No circles selected")
                return
            
            # Get target sketch
            sketch_sel = find_input(inputs, 'sketchSel')
            if not sketch_sel or sketch_sel.selectionCount == 0:
                log("No sketch selected")
                return
            
            sketch = adsk.fusion.Sketch.cast(sketch_sel.selection(0).entity)
            if not sketch:
                log("Invalid sketch")
                return
            
            log(f"Executing projection: {len(selected_indices)} circles to sketch '{sketch.name}'")
            log(f"Circles in sketch before: {sketch.sketchCurves.sketchCircles.count}")
            
            # Project circles
            projected = 0
            
            for idx in selected_indices:
                if idx >= len(_detected_circles):
                    continue
                    
                circle = _detected_circles[idx]
                try:
                    # Create 3D point for circle center
                    center_3d = adsk.core.Point3D.create(
                        circle.center.x, circle.center.y, circle.center.z
                    )
                    
                    log(f"Circle {idx}: center=({circle.center.x:.3f}, {circle.center.y:.3f}, {circle.center.z:.3f}), radius={circle.radius:.3f}")
                    
                    # Transform to sketch space
                    center_sketch = sketch.modelToSketchSpace(center_3d)
                    log(f"  sketch coords=({center_sketch.x:.3f}, {center_sketch.y:.3f}, {center_sketch.z:.3f})")
                    
                    # Create circle in sketch (use 2D point for sketch)
                    sketch_point = adsk.core.Point3D.create(center_sketch.x, center_sketch.y, 0)
                    
                    sketch_circles = sketch.sketchCurves.sketchCircles
                    new_circle = sketch_circles.addByCenterRadius(sketch_point, circle.radius)
                    log(f"  Created circle successfully")
                    
                    # Also add center point
                    sketch.sketchPoints.add(sketch_point)
                    
                    projected += 1
                except Exception as e:
                    log(f"Error projecting circle {idx}: {e}")
            
            log(f"Circles in sketch after: {sketch.sketchCurves.sketchCircles.count}")
            log(f"Successfully projected {projected} circles")
            
        except:
            log_exception("CommandExecuteHandler")


class CommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        global _detected_circles, _selected_indices, _debug_file
        log("CommandDestroyHandler.notify called")
        
        _detected_circles = []
        _selected_indices = []
        
        if _debug_file:
            try:
                _debug_file.close()
            except:
                pass
            _debug_file = None


# ============================================================================
# Add-in Entry Points
# ============================================================================

def run(context):
    global _app, _ui
    
    try:
        log("run() called")
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        log("Got app and UI")
        
        # Create command definition
        cmd_defs = _ui.commandDefinitions
        
        # Check for existing definition
        existing = cmd_defs.itemById('MeshCircleDetectorCmd')
        if existing:
            existing.deleteMe()
        
        cmd_def = cmd_defs.addButtonDefinition(
            'MeshCircleDetectorCmd',
            'Detect Mesh Circles',
            'Detect circular features on mesh bodies',
            ''
        )
        log("Created command definition")
        
        # Add handler
        on_command_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_command_created)
        _handlers.append(on_command_created)
        log("Added command created handler")
        
        # Add to UI
        inspect_panel = _ui.allToolbarPanels.itemById('InspectPanel')
        if inspect_panel:
            inspect_panel.controls.addCommand(cmd_def)
            log("Added to InspectPanel")
        
        log("run() completed successfully")
        
    except:
        log_exception("run")
        if _ui:
            _ui.messageBox(f'Failed to start add-in:\n{traceback.format_exc()}')


def stop(context):
    global _ui, _debug_file
    
    try:
        # Remove command
        cmd_def = _ui.commandDefinitions.itemById('MeshCircleDetectorCmd')
        if cmd_def:
            cmd_def.deleteMe()
        
        # Remove from panel
        inspect_panel = _ui.allToolbarPanels.itemById('InspectPanel')
        if inspect_panel:
            ctrl = inspect_panel.controls.itemById('MeshCircleDetectorCmd')
            if ctrl:
                ctrl.deleteMe()
        
        if _debug_file:
            _debug_file.close()
            _debug_file = None
            
    except:
        if _ui:
            _ui.messageBox(f'Failed to stop add-in:\n{traceback.format_exc()}')
