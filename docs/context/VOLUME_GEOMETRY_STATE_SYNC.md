# VolumeNode and GeometryNode State Synchronization

## Overview
Made all rendering attributes of VolumeNode and GeometryNode state-driven, with the state tree being authoritative for all values. Changes to the state tree automatically synchronize to VTK rendering properties.

## VolumeNode State Values

### Shading Properties
- `shading` (bool) - Enable/disable volume shading
- `ambient` (double) - Ambient lighting coefficient (0-1)
- `diffuse` (double) - Diffuse lighting coefficient (0-1)
- `specular` (double) - Specular lighting coefficient (0-1)
- `specular_power` (double) - Specular power/shininess

### Sampling Properties
- `scalar_opacity_unit_distance` (double) - Distance over which opacity is integrated
- `sample_distance` (double) - Distance between ray samples
- `auto_adjust_sample_distances` (bool) - Automatically adjust sampling based on volume size

### Data Range
- `data_min` (double) - Minimum data value in volume (updated when volume loaded)
- `data_max` (double) - Maximum data value in volume (updated when volume loaded)

### Transfer Functions
Transfer functions (color and opacity) are currently set via `setTransferFunction()` and `setDefaultTransferFunction()` methods. The functions use `data_min` and `data_max` from state for their ranges.

## GeometryNode State Values

### Render Mode
- `render_mode` (string) - Rendering mode: "0"=POINTS, "1"=LINES, "2"=TRIS, "3"=QUADS, "4"=TETS, "5"=HEXS

### Material Properties
- `color_r`, `color_g`, `color_b` (double 0-1) - Base material color
- `specular` (double 0-1) - Specular reflection coefficient
- `specular_power` (double) - Specular shininess/power
- `ambient` (double 0-1) - Ambient lighting coefficient
- `diffuse` (double 0-1) - Diffuse lighting coefficient  
- `opacity` (double 0-1) - Material opacity/transparency

### Point/Line Rendering
- `point_size` (double) - Size of points when in POINTS mode
- `line_width` (double) - Width of lines when in LINES/wireframe mode

## Implementation Details

### State Initialization

**VolumeNode** initializes with:
- Shading enabled, ambient=0.3, diffuse=0.6, specular=0.2, power=10.0
- Scalar opacity unit distance=1.0, sample distance=0.5
- Auto-adjust sample distances enabled
- Data range [0, 1] (updated when volume loaded)

**GeometryNode** initializes with:
- Render mode TRIS (solid triangles)
- Color (0.8, 0.8, 0.9) - light blue-gray
- Specular=0.3, power=20.0, ambient=0.0, diffuse=1.0, opacity=1.0
- Point size=3.0, line width=1.0

### Synchronization Pattern

All setter methods update the state tree instead of directly modifying VTK:

```cpp
// VolumeNode
void VolumeNode::setShading(bool enabled) {
    getState("shading").value(enabled ? 1 : 0);
}

// GeometryNode  
void GeometryNode::setColor(double r, double g, double b) {
    getState("color_r").value(r);
    getState("color_g").value(g);
    getState("color_b").value(b);
}
```

The `handleStateChanged()` method listens for state changes and updates VTK:

```cpp
// VolumeNode
if (childState == "shading") {
    m_shading = getState("shading").value<bool>();
    if (m_volumeProperty) {
        m_volumeProperty->SetShade(m_shading ? 1 : 0);
    }
}

// GeometryNode
else if (childState == "color_r" || childState == "color_g" || childState == "color_b") {
    double r = getState("color_r").value<double>();
    double g = getState("color_g").value<double>();
    double b = getState("color_b").value<double>();
    m_actor->GetProperty()->SetColor(r, g, b);
}
```

### GeometryNode Render Mode

The render mode synchronization uses a helper method `updateRenderModeVTK()` to separate VTK updates from state management. This allows `handleStateChanged()` to update the render mode without triggering a redundant state write.

When render mode changes:
1. State value updates
2. `handleStateChanged()` detects change
3. Calls `updateRenderModeVTK()` to update VTK representation
4. Updates polydata to match new mode (points/lines/surface)
5. Triggers render update if needed

### Main Thread Marshaling

Both `handleStateChanged()` implementations use `runOnMainThread()` to ensure all VTK operations execute on the main/GUI thread, preventing threading issues with Qt and VTK.

## Benefits

1. **State Tree Authority**: State tree is single source of truth for all rendering attributes
2. **Automatic Synchronization**: Changes propagate automatically via `handleStateChanged()`
3. **Persistence**: State values can be saved/restored via state tree serialization
4. **Remote Control**: State can be modified via XML-RPC or other interfaces
5. **Undo/Redo**: State changes can be tracked and reverted
6. **Consistency**: No possibility of state/rendering mismatch
7. **Thread Safety**: Main thread marshaling ensures safe VTK updates

## Testing

All volrover3 tests pass:
- AppStateTest ✓
- GraphicsNodeTest ✓
- VolumeNodeTest ✓
- NullGraphicNodeTest ✓

## Future Enhancements

### Transfer Function State Integration
Currently transfer functions are set via methods. Future work could:
- Store color/opacity tables as state arrays
- Sync transfer function changes bidirectionally
- Allow external control of transfer functions via state tree

### Additional GeometryNode Properties
Could add to state tree:
- Edge visibility
- Backface culling
- Texture coordinates
- Per-vertex colors/normals toggles
