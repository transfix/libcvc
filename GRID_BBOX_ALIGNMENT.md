# Grid Bounding Box Alignment

## Overview
Updated the three-plane grid system to be positioned at the **minimum corner of the bounding box** rather than at world origin (0,0,0). This allows the grid to serve as a proper reference frame for indexing volumetric data.

## Changes Made

### Grid Plane Positioning
The three grid planes are now positioned at the minimum corner of the bounding box:

- **YZ Plane**: Moved from `X=0` to `X=minX`
- **XZ Plane**: Moved from `Y=0` to `Y=minY`  
- **XY Plane**: Moved from `Z=0` to `Z=minZ`

This creates a coordinate system aligned with the actual data bounds, making the grid indices correspond directly to voxel positions in loaded volumes.

### Tick Label Updates
The tick labels now show proper grid indices:

- **YZ Plane** (at X=minX):
  - Bottom edge: Shows `j=0, j=8, j=16, ...` (Y-axis indices)
  - Left edge: Shows `k=8, k=16, k=24, ...` (Z-axis indices)

- **XZ Plane** (at Y=minY):
  - Bottom edge: Shows `i=0, i=8, i=16, ...` (X-axis indices)
  - Left edge: Shows `k=8, k=16, k=24, ...` (Z-axis indices)

- **XY Plane** (at Z=minZ):
  - Bottom edge: Shows `i=0, i=8, i=16, ...` (X-axis indices)
  - Left edge: Shows `j=8, j=16, j=24, ...` (Y-axis indices)

The labels follow the convention:
- `i` = X-axis grid index
- `j` = Y-axis grid index
- `k` = Z-axis grid index

### Technical Details

**Modified File**: `src/volrover3/GridNode.cpp`

**Grid Plane Creation Functions**:
- `createYZPlane()`: Now uses `minX` for X coordinate
- `createXZPlane()`: Now uses `minY` for Y coordinate
- `createXYPlane()`: Now uses `minZ` for Z coordinate

**Tick Label Creation Functions**:
- `createYZTickLabels()`: Labels positioned at `minX`, showing j,k indices
- `createXZTickLabels()`: Labels positioned at `minY`, showing i,k indices
- `createXYTickLabels()`: Labels positioned at `minZ`, showing i,j indices

## Usage

The grid now acts as a proper reference frame for volumetric data:

1. **Load a volume** - The grid automatically aligns to the volume's bounding box
2. **Set grid divisions** - Controls how finely the volume is subdivided (e.g., 64 divisions)
3. **Set tick intervals** - Controls label spacing (e.g., every 8 cells)
4. **Read grid indices** - The i,j,k values on tick labels correspond to voxel grid positions

## Example

With a volume bounding box of `[-10, -10, -10, 10, 10, 10]`:
- Grid planes appear at X=-10, Y=-10, Z=-10 (minimum corner)
- With 64 divisions and tick interval 8:
  - Labels appear at indices: 0, 8, 16, 24, 32, 40, 48, 56, 64
  - Each grid cell represents 20/64 ≈ 0.3125 world units
  - Index `i=32` corresponds to world coordinate X=0 (center)

## Benefits

1. **Direct Voxel Indexing**: Grid indices directly correspond to voxel positions
2. **Alignment with Data**: Grid aligns with actual volume bounds, not arbitrary world origin
3. **Intuitive Navigation**: Easy to identify regions of interest by grid coordinates
4. **Consistent Reference**: All three planes meet at the (i=0, j=0, k=0) corner
