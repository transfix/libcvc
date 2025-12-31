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
- View → State Tree (inspect application state)
- View → Camera Settings (adjust camera parameters)

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

## State Management System

VolRover3 features a comprehensive **state tree** system that stores all application data in a hierarchical structure. This enables:

- **Data-Driven Rendering**: Graphics automatically update when state data changes
- **Metadata Tracking**: Computed properties like bounding boxes, vertex counts, extents
- **Inspection & Debugging**: View → State Tree to browse all application data
- **Documentation**: State objects include descriptive comments explaining their purpose

### State Tree Features

**Viewing States**:
- View → State Tree opens the state inspector
- Shows all state objects in hierarchical tree view
- Double-click to expand and view properties
- Read-only metadata marked with 🔒 icon
- Blue italic text shows state comments

**State Names**:
- Must follow C identifier rules (letters, digits, underscores)
- Cannot start with digits or contain special characters
- Filenames automatically sanitized: `my-model.obj` → `my_model`
- Invalid names show helpful suggestions with before/after preview

**Geometry State Structure**:
```
graphics/
  geometry/
    my_model/              # Sanitized from filename
      num_vertices         # Read-only: vertex count
      bbox_min_x/y/z       # Read-only: bounding box minimum
      bbox_max_x/y/z       # Read-only: bounding box maximum
      extent_x/y/z         # Read-only: dimensions
      center_x/y/z         # Read-only: geometric center
      type                 # Read-only: geometry type
```

**Read-Only Metadata**:
- Computed automatically when geometry loads
- Protected from user modification
- Always stays synchronized with actual geometry
- Displayed with lock emoji in state tree

### State Validation

When creating or loading states:
- **Valid**: `geometry_node`, `_private`, `mesh123`
- **Invalid**: `123start`, `my-mesh`, `file name.obj`
- **Auto-sanitized**: Files with dashes/spaces converted to underscores
- **User prompts**: Manual state creation offers sanitization suggestions

## Architecture

```
volrover3/
├── Main Window (Qt6)
│   ├── Menu Bar (File, View, Help)
│   ├── VTK Render Widget (OpenGL)
│   ├── Transfer Function Dock
│   └── State Tree Inspector
│
├── Scene Graph (Data-Driven)
│   ├── Geometry Node (Surface Meshes + State Sync)
│   ├── Volume Node (3D Textures)
│   ├── Grid Node (Reference Grid)
│   └── Axis Node (XYZ Axes)
│
├── State System
│   ├── Hierarchical state tree (cvc::state)
│   ├── Automatic metadata computation
│   ├── Read-only protection
│   └── Name validation & sanitization
│
└── Camera Controller (Quake-style FPS)
```

## Tips

1. **Performance**: Smaller volumes render faster. Try downsampling large datasets.
2. **Visibility**: Use the grid and axis to understand spatial relationships.
3. **Transfer Functions**: Start with "X-Ray" to see the full data range, then adjust.
4. **Navigation**: Click in the viewport to capture mouse for smooth looking around.
5. **State Inspection**: Use View → State Tree to examine geometry metadata and debugging info.
6. **File Naming**: Files with special characters auto-sanitize (`my-mesh.obj` → `my_mesh`).
7. **Camera Settings**: Adjust FOV, near/far planes, and movement speed in View → Camera Settings.
8. **Multiple Datasets**: Currently supports one geometry + one volume at a time.

## Getting Help

- Check `VOLROVER3_IMPLEMENTATION.md` for technical details
- See `VOLROVER3_INDEX.md` for complete feature documentation
- Review `docs/STATE_API.md` for state system documentation
- Consult VTK documentation for rendering details
- Review libcvc API docs for data structure information

## Next Steps

Once you're comfortable with the basics:

1. **Experiment** with different transfer functions
2. **Load** your own scientific datasets
3. **Inspect** the state tree to understand how data is organized
4. **Explore** the codebase to understand the implementation
5. **Customize** camera settings for your workflow
6. **Extend** with new features (isosurfaces, measurements, etc.)
7. **Report** any issues or suggestions

Happy visualizing! 🎨📊🔬
