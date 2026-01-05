# Geometry Render Modes Implementation

## Overview

Added support for multiple rendering modes for geometry objects in VolRover3, allowing users to visualize meshes as points, wireframes, solid surfaces, or volumetric elements.

## Render Modes

The `GeometryRenderMode` enum provides six rendering options:

```cpp
enum class GeometryRenderMode {
    POINTS,  // Render as point cloud
    LINES,   // Render as wireframe
    TRIS,    // Render triangles as solid surface
    QUADS,   // Render quads as solid surface
    TETS,    // Render tetrahedral mesh (placeholder - currently wireframe)
    HEXS     // Render hexahedral mesh (placeholder - currently wireframe)
};
```

## Implementation Details

### GeometryNode Changes

**Header (`inc/volrover3/GeometryNode.h`):**
- Added `GeometryRenderMode` enum
- Added `setRenderMode()` / `getRenderMode()` methods
- Added static helper methods: `renderModeToString()` and `stringToRenderMode()`
- Added `m_renderMode` member variable
- Added `m_renderModeConnection` for state tree synchronization

**Implementation (`src/volrover3/GeometryNode.cpp`):**
- Constructor initializes render mode to `TRIS` (default)
- `setRenderMode()` updates VTK actor properties and triggers re-rendering
- `updatePolyData()` completely rewritten to generate appropriate VTK cells based on mode
- State tree integration: render mode is saved/loaded and changes trigger updates

### VTK Rendering by Mode

#### POINTS Mode
- Renders geometry as a point cloud
- Uses `vtkCellArray` with vertex cells
- Sets `SetRepresentationToPoints()` with point size 3.0

#### LINES Mode
- Renders geometry as wireframe
- Extracts edges from triangles, quads, and explicit line arrays
- Uses `vtkCellArray` with line cells
- Sets `SetRepresentationToWireframe()` with line width 1.0

#### TRIS Mode
- Renders triangle meshes as solid surfaces
- Uses `vtkCellArray` with triangle polygons
- Sets `SetRepresentationToSurface()`

#### QUADS Mode
- Renders quad meshes as solid surfaces
- Uses `vtkCellArray` with quad polygons
- Sets `SetRepresentationToSurface()`

#### TETS Mode (Placeholder)
- Currently renders as wireframe edges
- Extracts 6 edges per tetrahedron: (0,1), (0,2), (0,3), (1,2), (1,3), (2,3)
- TODO: Implement proper volumetric rendering (volume slicing, iso-surface extraction)

#### HEXS Mode (Placeholder)
- Currently renders as wireframe edges
- Extracts 12 edges per hexahedron (4 bottom + 4 top + 4 vertical)
- TODO: Implement proper volumetric rendering

## State Tree Integration

### Integer Representation

Render modes are stored as integers in the state tree for efficiency:

```
0 = POINTS
1 = LINES
2 = TRIS
3 = QUADS
4 = TETS
5 = HEXS
```

### Automatic Mode Detection

When geometry is loaded, the render mode is automatically selected based on the geometry type:

- `SURFACE_TRI` → `TRIS` (mode 2)
- `SURFACE_QUAD` → `QUADS` (mode 3)
- `VOLUME_TET` → `TETS` (mode 4)
- `VOLUME_HEX` → `HEXS` (mode 5)
- `MIXED` → `TRIS` if triangles present, else `QUADS` if quads present, else `TETS`, else `HEXS`

### Saving to State (`syncToState`)

The render mode is stored as an integer in the state tree:

```cpp
myState("render_mode").value(std::to_string(static_cast<int>(m_renderMode)));
myState("render_mode").comment("Geometry rendering mode: 0=POINTS, 1=LINES, 2=TRIS, 3=QUADS, 4=TETS, 5=HEXS");
```

### Loading from State (`syncFromState`)

The render mode is loaded from an integer and a callback is registered to watch for changes:

```cpp
std::string renderModeStr = myState("render_mode").value();
setRenderMode(stringToRenderMode(renderModeStr)); // Parses integer

// Watch for changes - captures m_stateNode to access current value
m_renderModeConnection = myState("render_mode").valueChanged.connect([this]() {
    if (!m_stateNode) return;
    try {
        std::string newMode = (*m_stateNode)("render_mode").value();
        setRenderMode(stringToRenderMode(newMode));
    } catch (...) {}
});
```

**Important**: The callback doesn't receive the new value as a parameter, so it must re-query the state node to get the updated value.

## Usage Example

```cpp
// Create geometry node
auto geomNode = std::make_shared<GeometryNode>("bunny");

// Load geometry - render mode is automatically detected
cvc::geometry bunny = cvc::read_geometry("bunny.off");
bunny.set_geometry_type(cvc::geometry::SURFACE_TRI);
geomNode->setGeometry(bunny);
// Automatically uses TRIS mode (2) for SURFACE_TRI

// Manually change render mode
geomNode->setRenderMode(GeometryRenderMode::LINES);  // Wireframe (1)
geomNode->setRenderMode(GeometryRenderMode::POINTS); // Point cloud (0)
geomNode->setRenderMode(GeometryRenderMode::TRIS);   // Solid surface (2)

// Render mode is automatically saved to/loaded from state tree
cvc::state rootState = cvcstate();
geomNode->syncToState(rootState);

// Change via state tree (triggers automatic re-render)
cvcstate("bunny")("render_mode").value("1");  // Set to LINES mode
cvcstate("bunny")("render_mode").value("0");  // Set to POINTS mode
cvcstate("bunny")("render_mode").value("2");  // Set to TRIS mode
```

## Future Work

### Tetrahedral Mesh Rendering

Proper tetrahedral mesh rendering should include:
- Volume slicing with configurable slice planes
- Iso-surface extraction at specified values
- Semi-transparent volume rendering
- Interior wireframe with culling
- Vertex/cell data visualization with color mapping

### Hexahedral Mesh Rendering

Proper hexahedral mesh rendering should include:
- Same features as tetrahedral rendering
- Quad face extraction for boundary visualization
- Support for higher-order hexahedral elements
- Adaptive refinement visualization

### Performance Optimizations

- Edge deduplication for LINES mode (currently creates duplicate edges)
- Level-of-detail (LOD) for large meshes
- GPU-accelerated volume rendering for TETS/HEXS
- Frustum culling for better performance

## Files Modified

1. **inc/volrover3/GeometryNode.h**
   - Added `GeometryRenderMode` enum
   - Added render mode methods and member variables

2. **src/volrover3/GeometryNode.cpp**
   - Implemented render mode string conversion
   - Implemented `setRenderMode()` with VTK property updates
   - Rewrote `updatePolyData()` to support all render modes
   - Updated `syncToState()` to save render mode
   - Updated `syncFromState()` to load render mode and register callbacks

## Testing

To test the implementation:

1. Build VolRover3: `cmake --build build --target volrover3`
2. Load a geometry file
3. Change render mode via GUI or state tree
4. Verify visual updates occur automatically
5. Save/load state and verify render mode persists

## References

- **VTK Property Documentation**: https://vtk.org/doc/nightly/html/classvtkProperty.html
- **VTK Cell Arrays**: https://vtk.org/doc/nightly/html/classvtkCellArray.html
- **cvc::geometry API**: docs/GEOMETRY_API.md
- **State Tree API**: docs/STATE_API.md
