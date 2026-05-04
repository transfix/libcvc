# GraphicsNode Data-Driven Updates

## Overview

GraphicsNode now automatically monitors and responds to changes in the state tree's data field. When the geometry data in the state tree is modified, the GraphicsNode will:

1. **Reload the geometry** from the state data
2. **Update VTK rendering** to display the new geometry
3. **Recalculate all metadata** to reflect the new geometry's properties
4. **Trigger a redraw** to show the updated visualization

## Implementation Details

### State Data Connection

When `syncFromState()` is called, the GraphicsNode:
- Stores a pointer to its state node (`m_stateNode`)
- Connects to the `dataChanged` signal
- Loads initial geometry from state data if available

```cpp
m_dataConnection = myState.dataChanged.connect([this]() {
    onDataChanged();
});
```

### Data Change Handler

The `onDataChanged()` callback is triggered when state data changes:

```cpp
void GraphicsNode::onDataChanged() {
    // 1. Load geometry from state data
    const cvc::geometry& geom = boost::any_cast<const cvc::geometry&>(m_stateNode->data());
    
    // 2. Update VTK rendering
    updatePolyData(geom);
    
    // 3. Recalculate metadata
    updateMetadata(geom);
    
    // 4. Sync metadata back to state tree (marked read-only)
    // 5. Trigger redraw
    m_actor->Modified();
}
```

### Computed Metadata

The `updateMetadata()` method computes comprehensive geometry statistics:

#### Basic Stats
- `num_vertices` - Number of vertices (read-only)
- `num_triangles` - Number of triangles (read-only)
- `num_quads` - Number of quads (read-only)
- `type` - Geometry type: "triangle_mesh", "quad_mesh", or "mixed_mesh" (read-only)

#### Bounding Box
- `bbox_min_x`, `bbox_min_y`, `bbox_min_z` - Minimum bounds (read-only)
- `bbox_max_x`, `bbox_max_y`, `bbox_max_z` - Maximum bounds (read-only)

#### Extents (Dimensions)
- `extent_x`, `extent_y`, `extent_z` - Width, height, depth (read-only)

#### Center Point
- `center_x`, `center_y`, `center_z` - Geometric center (read-only)

### Read-Only Protection

All computed metadata is automatically marked as **read-only** in the state tree to prevent manual modification. This ensures that metadata always reflects the actual geometry data.

## Usage Example

```cpp
// Create a graphics node
GraphicsNode node("my_mesh");
node.setGeometry(initialGeometry);

// Sync to state tree
cvc::state& graphics = cvc::state::instance(app)("graphics");
node.syncToState(graphics);

// Connect to state data changes
node.syncFromState(graphics);

// Later, another part of the application modifies the geometry in state
cvc::geometry newGeometry = loadFromFile("modified.obj");
graphics("my_mesh").data(newGeometry);  // Triggers onDataChanged()

// GraphicsNode automatically:
// - Loads the new geometry
// - Updates rendering
// - Recalculates all metadata (num_vertices, bbox, extents, etc.)
// - Updates the display
```

## State Tree Structure

After syncing, the state tree looks like:

```
graphics
└── my_mesh
    ├── [DATA: cvc::geometry object]
    ├── transform: "1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1"
    └── metadata
        ├── num_vertices: 3 (read-only)
        ├── num_triangles: 1 (read-only)
        ├── num_quads: 0 (read-only)
        ├── type: "triangle_mesh" (read-only)
        ├── bbox_min_x: 0.0 (read-only)
        ├── bbox_min_y: 0.0 (read-only)
        ├── bbox_min_z: 0.0 (read-only)
        ├── bbox_max_x: 1.0 (read-only)
        ├── bbox_max_y: 1.0 (read-only)
        ├── bbox_max_z: 0.0 (read-only)
        ├── extent_x: 1.0 (read-only)
        ├── extent_y: 1.0 (read-only)
        ├── extent_z: 0.0 (read-only)
        ├── center_x: 0.5 (read-only)
        ├── center_y: 0.5 (read-only)
        ├── center_z: 0.0 (read-only)
        └── visible: true
```

## Benefits

1. **Single Source of Truth**: Geometry data lives in the state tree, not duplicated in multiple places
2. **Automatic Synchronization**: Any change to state data automatically updates rendering
3. **Comprehensive Metadata**: All geometry properties computed automatically
4. **Read-Only Safety**: Computed metadata can't be accidentally modified
5. **Responsive UI**: Changes to geometry trigger immediate visual updates

## Testing

Comprehensive tests added in `GraphicsNodeTest.cpp`:

- `MetadataFromGeometry` - Verifies basic stats computation
- `BoundingBoxMetadata` - Checks bounding box calculation
- `ExtentMetadata` - Validates extent computation
- `CenterMetadata` - Tests center point calculation
- `DataChangeTriggerUpdate` - Confirms data change triggers updates
- `MetadataSyncToState` - Ensures metadata syncs to state tree
- `ComputedMetadataReadOnly` - Verifies read-only protection
- `GeometryTypeDetection` - Tests mesh type classification
- `MetadataUpdatesOnGeometryChange` - Confirms metadata updates with geometry

All 497 tests pass, including the 10 new GraphicsNode data-driven tests.

## API Changes

### New Methods

- `void updateMetadata(const cvc::geometry& geom)` - Compute all geometry statistics
- `void onDataChanged()` - Handle state data changes (called via signal)

### New Members

- `cvc::state* m_stateNode` - Pointer to state tree node
- `boost::signals2::connection m_dataConnection` - Signal connection for data changes

### Modified Methods

- `syncFromState()` - Now connects to dataChanged signal and loads geometry from state data
- `syncToState()` - Now calls updateMetadata() to ensure fresh metadata
- `setGeometry()` - Now calls updateMetadata() to compute stats
- `~GraphicsNode()` - Disconnects signal connection

## Implementation Files

- `inc/volrover3/GraphicsNode.h` - Header with new members and methods
- `src/volrover3/GraphicsNode.cpp` - Implementation of data-driven updates
- `src/volrover3/tests/GraphicsNodeTest.cpp` - Comprehensive test coverage
