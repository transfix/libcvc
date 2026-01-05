# GridNode and AxisNode State Synchronization

## Overview
Made all rendering attributes of GridNode and AxisNode state-driven, with the state tree being authoritative for all values. Changes to the state tree automatically synchronize to the VTK rendering properties.

## GridNode State Values

### Plane Visibility
- `yz_plane_visible` (bool) - YZ plane visibility at X=minX
- `xz_plane_visible` (bool) - XZ plane visibility at Y=minY  
- `xy_plane_visible` (bool) - XY plane visibility at Z=minZ

### Plane Colors (RGB 0-1)
- `yz_plane_color_r`, `yz_plane_color_g`, `yz_plane_color_b`
- `xz_plane_color_r`, `xz_plane_color_g`, `xz_plane_color_b`
- `xy_plane_color_r`, `xy_plane_color_g`, `xy_plane_color_b`

### Grid Divisions
- `divisions_x` (int) - Number of grid divisions along X axis
- `divisions_y` (int) - Number of grid divisions along Y axis
- `divisions_z` (int) - Number of grid divisions along Z axis

### Tick Properties
- `tick_interval_x` (int) - Show tick every N grid cells along X
- `tick_interval_y` (int) - Show tick every N grid cells along Y
- `tick_interval_z` (int) - Show tick every N grid cells along Z
- `tick_label_color_r`, `tick_label_color_g`, `tick_label_color_b` (double 0-1)
- `tick_label_font_size` (int)

## AxisNode State Values

### Axis Properties
- `axis_length` (double) - Length of all three axes
- `shaft_type_line` (bool) - true=line shaft, false=cylinder shaft
- `show_labels` (bool) - Show/hide axis labels

### Label Properties
- `label_font_size` (int) - Font size for all axis labels
- `x_label_color_r`, `x_label_color_g`, `x_label_color_b` (double 0-1)
- `y_label_color_r`, `y_label_color_g`, `y_label_color_b` (double 0-1)
- `z_label_color_r`, `z_label_color_g`, `z_label_color_b` (double 0-1)

## Implementation Details

### State Initialization
Both nodes initialize their state trees in constructors with default values:
- GridNode: gray grid planes (0.5, 0.5, 0.5), 64 divisions, tick intervals of 8
- AxisNode: 2.0 unit length, line shafts, red/green/blue labels, size 20

### Synchronization Pattern
All setter methods now update the state tree instead of directly modifying VTK properties:
```cpp
void GridNode::setYZPlaneVisible(bool visible) {
    getState("yz_plane_visible").value(visible ? 1 : 0);
}
```

The `handleStateChanged()` method listens for state changes and updates VTK properties:
```cpp
if (childState == "yz_plane_visible") {
    m_yzPlaneVisible = getState("yz_plane_visible").value<bool>();
    m_yzActor->SetVisibility(m_yzPlaneVisible);
}
```

### Benefits
1. **State Tree Authority**: State tree is single source of truth for all rendering attributes
2. **Automatic Synchronization**: Changes propagate automatically via `handleStateChanged()`
3. **Persistence**: State values can be saved/restored via state tree serialization
4. **Remote Control**: State can be modified via XML-RPC or other interfaces
5. **Undo/Redo**: State changes can be tracked and reverted
6. **Consistency**: No possibility of state/rendering mismatch

## Testing
All volrover3 tests pass:
- AppStateTest ✓
- GraphicsNodeTest ✓
- VolumeNodeTest ✓
- NullGraphicNodeTest ✓
