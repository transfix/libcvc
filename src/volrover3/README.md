# VolRover3 - Volume Rover Version 3

A prototype visualization application built on libcvc for rendering volumetric data, surface meshes, and volumetric meshes.

## Features

- **Volume Rendering**: 3D texture-based volume rendering with GPU acceleration via VTK
- **Surface Mesh Visualization**: Triangle mesh rendering with normals and colors
- **Volumetric Mesh Support**: Tetrahedral and hexahedral mesh visualization
- **Transfer Functions**: Interactive color and opacity mapping for volume data
- **Quake-Style Camera**: First-person camera controls for intuitive navigation
- **Scene Elements**: Toggleable grid and coordinate axis for reference
- **File I/O Integration**: Support for CVC geometry and volume formats

## Building

VolRover3 requires:
- Qt6 (Core, Widgets, OpenGL, OpenGLWidgets)
- VTK (Visualization Toolkit) 9.0+
- libcvc (built from this project)

### Build Steps

```bash
mkdir build && cd build
cmake .. -DCVC_BUILD_VOLROVER3=ON
make volrover3
```

### Optional: Disable VolRover3 Build

If Qt6 or VTK are not available:

```bash
cmake .. -DCVC_BUILD_VOLROVER3=OFF
```

## Usage

### Launch

```bash
./bin/volrover3
```

### Controls

**Camera Movement (Quake-Style)**:
- `W` - Move forward
- `S` - Move backward
- `A` - Strafe left
- `D` - Strafe right
- `E` or `Space` - Move up
- `Q` or `Ctrl` - Move down
- `Mouse drag` (left button) - Look around
- `Mouse wheel` - Zoom in/out

### Menu Options

**File Menu**:
- `Open Geometry...` - Load surface meshes (.off, .raw, .obj, etc.)
- `Open Volume...` - Load volume data (.rawiv, .mrc, .ccp4)

**View Menu**:
- `Show Grid` - Toggle ground grid display
- `Show Axis` - Toggle coordinate axis display

## Supported File Formats

**Geometry**:
- `.off` - Object File Format
- `.raw`, `.rawn`, `.rawc`, `.rawnc` - CVC raw formats
- `.obj` - Wavefront OBJ (experimental via SDF)

**Volume**:
- `.rawiv` - RAWIV format
- `.mrc` - MRC/CCP4 format
- Other formats supported by libcvc

## Architecture

### Components

- **MainWindow**: Qt6 main application window with menus and docking
- **VTKRenderWidget**: VTK/OpenGL rendering widget with event handling
- **SceneGraph**: Scene management and traversal
- **SceneNode**: Base class for renderable objects
  - **GeometryNode**: Surface mesh rendering
  - **VolumeNode**: Volume rendering with transfer functions
  - **GridNode**: Reference grid
  - **AxisNode**: Coordinate axis
- **CameraController**: Quake-style first-person camera
- **TransferFunctionWidget**: Color and opacity mapping UI

### Rendering Pipeline

1. Load geometry/volume via libcvc file I/O
2. Convert to VTK data structures (vtkPolyData, vtkImageData)
3. Create appropriate mappers (vtkPolyDataMapper, vtkSmartVolumeMapper)
4. Add actors/volumes to VTK renderer
5. Scene graph manages visibility and updates
6. Camera controller handles user input
7. Transfer function widget controls volume appearance

## Future Enhancements

- [ ] Isosurface extraction and rendering
- [ ] Multiple geometry/volume layers
- [ ] Advanced transfer function editor with histogram
- [ ] Screenshot and animation export
- [ ] Property inspector for loaded data
- [ ] Clipping planes
- [ ] Lighting controls
- [ ] Material editor
- [ ] Measurements and annotations

## License

Copyright © 2025 CVC (Computational Visualization Center)

See main project LICENSE for details.
