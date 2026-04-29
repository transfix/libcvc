# VolRover3 Documentation Index

Welcome to VolRover3 - a modern 3D visualization application built on libcvc!

## Quick Links

### For Users
- 🚀 **[Quick Start Guide](VOLROVER3_QUICKSTART.md)** - Get up and running in 5 minutes
- 📖 **[User Manual](volrover3/README.md)** - Complete feature documentation
- 🔧 **[Build Script](build_volrover3.sh)** - Automated build with dependency checking

### For Developers
- 🏗️ **[Implementation Details](VOLROVER3_IMPLEMENTATION.md)** - Technical architecture and design
- ✅ **[Completion Summary](VOLROVER3_COMPLETE.md)** - What was built and current status
- 💻 **[Source Code](volrover3/)** - Headers and implementation files

## What is VolRover3?

VolRover3 is a prototype visualization application that provides:

- **Volume Rendering**: 3D scalar field visualization with transfer functions
- **Surface Meshes**: Triangle mesh rendering with lighting
- **Quake-Style Camera**: Intuitive first-person navigation
- **Scene Graph**: Modular rendering architecture
- **File I/O**: Integration with libcvc geometry and volume formats

## Getting Started

### Prerequisites
- Qt6 (Core, Widgets, OpenGL, OpenGLWidgets)
- VTK 9.0 or later
- OpenGL 3.3 or later

### Build
```bash
./build_volrover3.sh
```

### Run
```bash
./build/bin/volrover3
```

## Documentation Organization

### 1. Quick Start Guide (`VOLROVER3_QUICKSTART.md`)
**For**: New users who want to get started quickly  
**Contains**:
- Installation instructions
- Basic usage walkthrough
- Controls reference
- Troubleshooting tips

### 2. User Manual (`volrover3/README.md`)
**For**: Users who want to understand all features  
**Contains**:
- Complete feature list
- Detailed build instructions
- File format support
- Architecture overview
- Future enhancements

### 3. Implementation Details (`VOLROVER3_IMPLEMENTATION.md`)
**For**: Developers who want to understand the code  
**Contains**:
- Component architecture
- Technical implementation details
- Rendering pipeline explanation
- Performance considerations
- Known issues

### 4. Completion Summary (`VOLROVER3_COMPLETE.md`)
**For**: Project managers and reviewers  
**Contains**:
- What was implemented
- File structure
- Code statistics
- Success criteria
- Next steps

### 5. Build Script (`build_volrover3.sh`)
**For**: Anyone building the application  
**Contains**:
- Dependency checking
- Automated CMake configuration
- Parallel compilation

## Key Features

### Scene Graph System
```
SceneGraph
├── GeometryNode (Surface meshes)
├── VolumeNode (3D volumes with transfer functions)
├── GridNode (Reference grid)
└── AxisNode (Coordinate axes)
```

### Navigation Controls
- **WASD**: Move forward/left/back/right
- **E/Q** or **Space/Ctrl**: Move up/down
- **Mouse Drag**: Look around
- **Mouse Wheel**: Zoom

### Transfer Functions
- Grayscale
- Rainbow
- Hot
- Cool
- X-Ray

### File Formats
**Geometry**: .off, .raw, .rawn, .rawc, .obj  
**Volume**: .rawiv, .mrc, .ccp4

## Code Structure

```
volrover3/
├── include/               # Header files (10 files)
│   ├── MainWindow.h
│   ├── VTKRenderWidget.h
│   ├── SceneGraph.h
│   ├── SceneNode.h
│   ├── GeometryNode.h
│   ├── VolumeNode.h
│   ├── GridNode.h
│   ├── AxisNode.h
│   ├── CameraController.h
│   └── TransferFunctionWidget.h
│
├── src/                   # Source files (11 files)
│   ├── main.cpp
│   ├── MainWindow.cpp
│   ├── VTKRenderWidget.cpp
│   ├── SceneGraph.cpp
│   ├── SceneNode.cpp
│   ├── GeometryNode.cpp
│   ├── VolumeNode.cpp
│   ├── GridNode.cpp
│   ├── AxisNode.cpp
│   ├── CameraController.cpp
│   └── TransferFunctionWidget.cpp
│
├── CMakeLists.txt         # Build configuration
└── README.md              # User documentation
```

## Dependencies

### Build-time
- CMake 3.16+
- C++20 compiler (GCC 13+, Clang 17+, MSVC 19.29+)
- Qt6 development libraries
- VTK development libraries

### Runtime
- Qt6 (Core, Widgets, OpenGL, OpenGLWidgets)
- VTK 9.0+ shared libraries
- OpenGL 3.3+ capable graphics driver

## Building from Source

### Ubuntu/Debian
```bash
# Install dependencies
sudo apt-get install cmake qt6-base-dev qt6-opengl-dev libvtk9-dev

# Build
./build_volrover3.sh
```

### macOS
```bash
# Install dependencies
brew install cmake qt@6 vtk

# Build
./build_volrover3.sh
```

### Manual Build
```bash
mkdir build && cd build
cmake .. -DCVC_BUILD_VOLROVER3=ON
make volrover3 -j$(nproc)
```

## Testing

### Manual Testing Checklist
- [ ] Application builds without errors
- [ ] Application launches successfully
- [ ] Can open .off geometry file
- [ ] Can open .rawiv volume file
- [ ] Camera controls respond to WASD
- [ ] Mouse look controls work
- [ ] Grid toggles on/off
- [ ] Axis toggles on/off
- [ ] Transfer function presets work
- [ ] Volume updates when preset changes

### Example Test Data
If you have the Stanford Bunny or sample volumes:
```bash
./build/bin/volrover3
# File → Open Geometry → bunny.off
# File → Open Volume → sample.rawiv
```

## Troubleshooting

### Build Issues
- **Qt6 not found**: Install qt6-base-dev and qt6-opengl-dev
- **VTK not found**: Install libvtk9-dev or compile VTK from source
- **CMake too old**: Upgrade to CMake 3.16 or later

### Runtime Issues
- **Black screen**: Check OpenGL version (`glxinfo | grep "OpenGL version"`)
- **No volume visible**: Try "X-Ray" preset first
- **Slow navigation**: Reduce volume resolution or downsample

## Future Development

### Planned Features
1. Isosurface extraction and rendering
2. Interactive transfer function editing with histogram
3. Multiple geometry/volume layers
4. Screenshot and animation export
5. Property inspector panel
6. Clipping planes
7. Advanced lighting controls

### Extension Points
- Add new `SceneNode` subclasses for different renderables
- Implement custom transfer function editors
- Add file format support via libcvc extensions
- Create rendering modes (wireframe, points, etc.)

## Contributing

To extend VolRover3:

1. Study the existing scene node implementations
2. Follow the same pattern for new node types
3. Update the scene graph to manage new nodes
4. Add menu items or UI controls as needed
5. Document new features in README.md

## Support

- 📚 Read the documentation files
- 💻 Examine the well-commented source code
- 🔍 Check VTK documentation for rendering details
- 📖 Consult libcvc API documentation for data structures

## License

Copyright © 2025 CVC (Computational Visualization Center)

See main project [LICENSE](LICENSE) for details.

---

## Quick Reference

| Document | Purpose | Audience |
|----------|---------|----------|
| [VOLROVER3_QUICKSTART.md](VOLROVER3_QUICKSTART.md) | Get started quickly | New users |
| [volrover3/README.md](volrover3/README.md) | Complete features | All users |
| [VOLROVER3_IMPLEMENTATION.md](VOLROVER3_IMPLEMENTATION.md) | Technical details | Developers |
| [VOLROVER3_COMPLETE.md](VOLROVER3_COMPLETE.md) | Project summary | Reviewers |
| [build_volrover3.sh](build_volrover3.sh) | Build automation | Builders |

**Start here**: [VOLROVER3_QUICKSTART.md](VOLROVER3_QUICKSTART.md)

---

*Happy visualizing! 🎨📊🔬*
