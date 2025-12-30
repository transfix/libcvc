# VolRover3 Quick Start Guide

## What is VolRover3?

VolRover3 is a modern 3D visualization application for scientific data, built on top of the trans-cvc library. It combines:

- **Volume Rendering**: Visualize 3D scalar fields as semi-transparent volumes
- **Surface Meshes**: Display triangle meshes with lighting and colors
- **Interactive Navigation**: Quake-style first-person camera controls
- **Transfer Functions**: Map data values to colors and opacity
- **Reference Elements**: Toggleable grid and coordinate axes

## Quick Start

### 1. Build

```bash
./build_volrover3.sh
```

This will check dependencies and build the application.

### 2. Run

```bash
./build/bin/volrover3
```

### 3. Load Data

**Open a Surface Mesh**:
- File → Open Geometry
- Select a `.off`, `.raw`, or `.obj` file
- Mesh appears in the viewport

**Open Volume Data**:
- File → Open Volume
- Select a `.rawiv` or `.mrc` file
- Volume appears with current transfer function

### 4. Navigate

**Move**: W/A/S/D (forward/left/back/right)  
**Up/Down**: E or Space / Q or Ctrl  
**Look**: Click and drag mouse  
**Zoom**: Mouse wheel

### 5. Adjust Visualization

**Transfer Function**: Select a preset from the dropdown
- Grayscale: Simple black to white
- Rainbow: Full color spectrum
- Hot: Black → Red → Yellow → White
- Cool: Cyan to Magenta
- X-Ray: High contrast black/white

**View Options**:
- View → Show Grid (ground reference)
- View → Show Axis (XYZ coordinates)

## Dependencies

**Required**:
- Qt6 (Core, Widgets, OpenGL, OpenGLWidgets)
- VTK 9.0 or later
- OpenGL 3.3 or later

**Install on Ubuntu/Debian**:
```bash
sudo apt-get install qt6-base-dev qt6-opengl-dev libvtk9-dev
```

**Install on macOS**:
```bash
brew install qt@6 vtk
```

## Controls Reference

| Action | Key/Mouse |
|--------|-----------|
| Move Forward | W |
| Move Backward | S |
| Strafe Left | A |
| Strafe Right | D |
| Move Up | E or Space |
| Move Down | Q or Ctrl |
| Look Around | Mouse Drag (Left Button) |
| Zoom In/Out | Mouse Wheel |

## File Formats

**Geometry (Meshes)**:
- `.off` - Object File Format
- `.raw` - CVC raw format (triangles)
- `.rawn` - CVC raw with normals
- `.rawc` - CVC raw with colors
- `.obj` - Wavefront OBJ (via SDF)

**Volume (3D Data)**:
- `.rawiv` - RAWIV format
- `.mrc` - MRC/CCP4 format

## Example Datasets

If you have test data:
```bash
# Load the Stanford Bunny
./build/bin/volrover3
# File → Open Geometry → select bunny.off

# Load a volume dataset
# File → Open Volume → select volume.rawiv
```

## Troubleshooting

**Application doesn't build**:
- Check that Qt6 and VTK are installed
- Verify CMake finds the libraries: `cmake .. -DCVC_BUILD_VOLROVER3=ON`

**Black screen on startup**:
- Check OpenGL version: `glxinfo | grep "OpenGL version"`
- Ensure you have OpenGL 3.3 or later

**Volume doesn't appear**:
- Check transfer function settings
- Try different presets (X-Ray shows everything)
- Verify volume file loaded correctly (check status bar)

**Navigation feels too fast/slow**:
- Currently hard-coded, but configurable in code
- See `CameraController::setMovementSpeed()`

## Architecture

```
volrover3/
├── Main Window (Qt6)
│   ├── Menu Bar (File, View, Help)
│   ├── VTK Render Widget (OpenGL)
│   └── Transfer Function Dock
│
├── Scene Graph
│   ├── Geometry Node (Surface Meshes)
│   ├── Volume Node (3D Textures)
│   ├── Grid Node (Reference Grid)
│   └── Axis Node (XYZ Axes)
│
└── Camera Controller (Quake-style FPS)
```

## Tips

1. **Performance**: Smaller volumes render faster. Try downsampling large datasets.
2. **Visibility**: Use the grid and axis to understand spatial relationships.
3. **Transfer Functions**: Start with "X-Ray" to see the full data range, then adjust.
4. **Navigation**: Click in the viewport to capture mouse for smooth looking around.
5. **Multiple Datasets**: Currently supports one geometry + one volume at a time.

## Getting Help

- Check `VOLROVER3_IMPLEMENTATION.md` for technical details
- See `volrover3/README.md` for feature documentation
- Review VTK documentation for rendering details
- Consult libcvc API docs for data structure information

## Next Steps

Once you're comfortable with the basics:

1. **Experiment** with different transfer functions
2. **Load** your own scientific datasets
3. **Explore** the codebase to understand the implementation
4. **Extend** with new features (isosurfaces, measurements, etc.)
5. **Report** any issues or suggestions

Happy visualizing! 🎨📊🔬
