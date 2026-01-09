# Grid and Bounding Box Enhancements

## Summary

Implemented two major enhancements to the grid and bounding box visualization system:

1. **Automatic Grid Updates**: Grid now automatically updates when world bounding box changes
2. **World Coordinate Ticks**: Added toggle-able coordinate tick labels to the volume bounding box

## Features Implemented

### 1. Automatic Grid Updates

**Problem**: Previously, when loading a volume, the grid would remain at its original scale and not conform to the new data bounds.

**Solution**: The grid update mechanism was already in place through the `onWorldBoundsChanged()` callback in MainWindow.cpp. This callback triggers `updateGrid()` whenever:
- Geometry is loaded (`onGeometryChanged`)
- Volume is loaded (`onVolumeChanged`)
- World bounds are manually changed (`onWorldBoundsChanged`)

The grid automatically repositions to the minimum corner of the bounding box (i=0, j=0, k=0 position) and scales to match the data extents.

### 2. World Coordinate Tick Labels on Bounding Box

**New Feature**: Added configurable tick labels showing world coordinates along the edges of the volume bounding box.

**UI Controls** (in World Bounding Box dialog):
- **Show Ticks**: Checkbox to toggle tick visibility
- **Tick Interval**: Spacing between tick labels in world units (default: 1.0)
- **Label Color**: Color picker for tick label text (default: yellow)
- **Font Size**: Adjustable font size for labels (range: 6-72, default: 12)

**Tick Placement**:
- X-axis labels: Along bottom-front edge (minY, minZ)
- Y-axis labels: Along left-front edge (minX, minZ), starting after first tick
- Z-axis labels: Along left-bottom edge (minX, minY), starting after first tick

Labels show world coordinates with 2 decimal precision (e.g., "-5.00", "0.00", "5.00").

## Technical Implementation

### Modified Files

#### 1. BBoxNode (inc/volrover3/BBoxNode.h, src/volrover3/BBoxNode.cpp)
- Added tick label support with VTK 2D text actors
- New methods:
  - `setTicksVisible(bool)` - Toggle tick display
  - `setTickInterval(double)` - Control spacing
  - `setTickLabelColor(r, g, b)` - Customize color
  - `setTickLabelFontSize(int)` - Adjust size
- Override `addToRenderer()` / `removeFromRenderer()` to manage label actors
- `createTickLabels()` generates labels at intervals along bbox edges

#### 2. AppState (inc/volrover3/AppState.h, src/volrover3/AppState.cpp)
- Added state properties:
  - `volume_bbox_ticks_visible` (bool, default: false)
  - `volume_bbox_tick_interval` (double, default: 1.0)
  - `volume_bbox_tick_label_color` (RGB string, default: "1.0,1.0,0.0" - yellow)
  - `volume_bbox_tick_label_font_size` (int, default: 12)
- New methods: `volumeBBoxTicksVisible()`, `setVolumeBBoxTicksVisible()`, etc.
- New callback: `onVolumeBBoxTicksChanged()` for reactive updates

#### 3. BoundingBoxDialog (inc/volrover3/BoundingBoxDialog.h, src/volrover3/BoundingBoxDialog.cpp)
- Added "World Coordinate Ticks" section with controls
- New UI members:
  - `m_showTicksCheckbox` - QCheckBox
  - `m_tickIntervalSpinBox` - QDoubleSpinBox (0.01-1000.0 range)
  - `m_tickLabelColorButton` - QPushButton with color preview
  - `m_tickLabelFontSizeSpinBox` - QSpinBox (6-72 range)
- Methods:
  - `loadTickSettings()` - Load from AppState on dialog open
  - `saveTickSettings()` - Save to AppState on OK
  - `chooseTickLabelColor()` - QColorDialog integration
  - `updateColorButton()` - Show selected color in button background

#### 4. SceneGraph (inc/volrover3/SceneGraph.h, src/volrover3/SceneGraph.cpp)
- New method: `setVolumeBBoxTicks()` - Apply tick settings to volume bbox node
- Calls corresponding BBoxNode methods

#### 5. MainWindow (src/volrover3/MainWindow.cpp)
- Connected `onVolumeBBoxTicksChanged()` callback
- Updates SceneGraph when any tick property changes
- Triggers render update

## Usage

### Automatic Grid Updates
1. Load a volume or geometry
2. Grid automatically scales and repositions to match data bounds
3. Grid planes appear at i=0, j=0, k=0 corner
4. Grid tick labels show cell indices

### World Coordinate Ticks
1. Open **Settings → World Bounding Box** (or Ctrl+B)
2. Enable **"Show Ticks"** checkbox
3. Adjust **"Tick Interval"** for desired spacing
4. Click color button to customize label color
5. Adjust font size as needed
6. Click OK to apply

Tick labels display actual world coordinates, making it easy to identify spatial positions in your data.

## Grid vs BBox Ticks

**Grid Ticks** (on grid planes):
- Show **grid cell indices** (i, j, k)
- Help identify voxel positions
- Use configurable intervals (default: every 8 cells)
- Positioned on grid planes at minX, minY, minZ

**BBox Ticks** (on bounding box):
- Show **world coordinates** (x, y, z values)
- Help identify spatial positions in world units
- Use world-space intervals (e.g., every 1.0 units)
- Positioned along bbox edges

Both systems work together to provide complete spatial reference:
- Grid ticks → discrete voxel addressing
- BBox ticks → continuous world coordinates

## State Persistence

All settings are persisted through the `cvc::state` system:
- Tick visibility
- Tick interval
- Label color (RGB)
- Font size

Settings are preserved across sessions and automatically restored on application startup.

## Build Notes

Successfully compiled with:
- Qt6 (QCheckBox, QDoubleSpinBox, QSpinBox, QColorDialog)
- VTK 9.3 (vtkActor2D, vtkTextMapper for 2D text labels)
- Boost.Signals2 (reactive state updates)
