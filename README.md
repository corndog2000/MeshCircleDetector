# Mesh Circle Detector for Fusion 360

A Fusion 360 add-in that detects circular features (holes, bosses, etc.) on mesh bodies and allows you to project them to sketches. This provides functionality similar to the circle detection feature found in BambuStudio and OrcaSlicer.

<img width="1604" height="1026" alt="Example" src="https://github.com/corndog2000/MeshCircleDetector/blob/main/Example.png" />

## Features

- **Automatic Circle Detection**: Analyzes mesh bodies to find circular features by detecting boundary edge loops and fitting circles
- **Interactive Highlighting**: Circles near your mouse cursor are highlighted for easy identification
- **Center Point Display**: Shows the center point of each detected circle (like in slicer software)
- **Multi-Selection**: Click to select multiple circles for batch projection
- **Sketch Projection**: Project selected circles to any sketch, similar to the Project tool
- **Configurable Detection**: Adjust circularity threshold, min/max radius to tune detection

## Installation

### Method 1: Manual Installation

1. Download or clone this repository
2. Locate your Fusion 360 Add-Ins folder:
   - **Windows**: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns`
   - **Mac**: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns`
3. Copy the entire `MeshCircleDetector` folder to the Add-Ins folder
4. Restart Fusion 360 or use the Add-Ins manager to load it

### Method 2: Scripts and Add-Ins Dialog

1. In Fusion 360, go to **Tools > Add-Ins > Scripts and Add-Ins**
2. Click the **Add-Ins** tab
3. Click the green **+** button next to "My Add-Ins"
4. Navigate to the `MeshCircleDetector` folder and select it
5. Click **Run** to start the add-in

## Usage

### Basic Workflow

1. **Import a Mesh**: Insert a mesh body (STL, OBJ, 3MF, etc.) into your design
2. **Run the Command**: Find "Detect Mesh Circles" in the Inspect panel, or run it from Add-Ins
3. **Detect Circles**: Click the "Detect Circles" button to analyze all mesh bodies
4. **Hover to Highlight**: Move your mouse over the model - nearby circles will highlight in yellow
5. **Select Circles**: Click on circles to select them (they turn green when selected)
6. **Choose Target Sketch**: Select an existing sketch or create a new one
7. **Project**: Click "Project Selected" to add the circles to your sketch

### Detection Settings

- **Circularity Threshold** (0.5-1.0): How "perfect" a circle must be to be detected. Lower values find more circles but may include non-circular features.
- **Min Radius**: Minimum circle radius to detect (in mm)
- **Max Radius**: Maximum circle radius to detect (in mm)

### Display Options

- **Show Center Points**: Display orange center points for each detected circle
- **Show Normal Directions**: Display a line showing the circle's normal direction

### Tips

- For better detection results, ensure your mesh is clean and has reasonable resolution
- Circles on curved surfaces may not be detected as well as circles on flat surfaces
- If circles aren't being detected, try lowering the circularity threshold
- Use the "Clear Selection" button to deselect all circles

## How It Works

### Circle Detection Algorithm

1. **Boundary Edge Detection**: The algorithm first identifies boundary edges - edges that belong to only one triangle in the mesh. These typically indicate holes or the edges of features.

2. **Loop Tracing**: Connected boundary edges are traced to form closed loops.

3. **Plane Fitting**: For each loop, a best-fit plane is calculated using the loop's vertices.

4. **Circle Fitting**: The loop vertices are projected onto the fitted plane, and a least-squares circle fitting algorithm determines the best-fit circle.

5. **Confidence Scoring**: Each detected circle receives a confidence score based on how well the original points fit the computed circle.

### Limitations

- Works best with mesh bodies that have clean geometry
- Very small or very large circles may be missed depending on settings
- Circles on highly curved surfaces may have reduced detection accuracy
- Performance depends on mesh complexity (large meshes take longer to analyze)

## File Structure

```
MeshCircleDetector/
├── MeshCircleDetector.py        # Main add-in entry point
├── MeshCircleDetector.manifest  # Add-in manifest
├── README.md                    # This file
└── lib/
    ├── __init__.py              # Library module initialization
    ├── circle_detector.py       # Circle detection algorithms
    ├── mesh_utils.py            # Fusion 360 mesh utilities
    └── graphics.py              # Custom graphics for visualization
```

## API Reference

### CircleDetector Class

```python
from lib.circle_detector import CircleDetector

detector = CircleDetector(
    min_points=8,           # Minimum points for circle fitting
    circularity_threshold=0.85,  # Required circularity (0-1)
    min_radius=0.5,         # Minimum radius in model units
    max_radius=500.0        # Maximum radius in model units
)

# Detect circles from mesh data
circles = detector.detect_circles_from_mesh_data(vertices, triangles)
```

### DetectedCircle Properties

- `center`: Point3D - Center point of the circle
- `radius`: float - Radius of the circle
- `normal`: Vector3D - Normal direction of the circle plane
- `points`: List[Point3D] - Original points used for detection
- `confidence`: float - Detection confidence (0-1)

## Troubleshooting

### "No mesh bodies found"
Ensure you have inserted a mesh body into your design. Regular solid bodies are not supported.

### Circles not being detected
- Try lowering the circularity threshold
- Check that your circles fall within the min/max radius range
- Ensure the mesh has sufficient resolution

### Graphics not showing
- Try refreshing the view (middle-mouse button to orbit slightly)
- Ensure the add-in is running (check Add-Ins dialog)

### Performance issues
- Large meshes (>100k triangles) may take several seconds to analyze
- Consider using simpler meshes for initial testing

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This project is released under the MIT License. See LICENSE file for details.

## Acknowledgments

- Inspired by the circle detection feature in BambuStudio and OrcaSlicer
- Uses least-squares circle fitting algorithms
- Built using the Fusion 360 API
