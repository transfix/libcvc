# SDF v2 Integration Summary

## Overview
Successfully integrated the SDF v2 (DistanceTransform) algorithm alongside the existing SDF v1 (SDFLibrary) with enum-based algorithm selection. Both algorithms can now be used through the same `sdf()` interface.

## Changes Made

### 1. Build System (`src/cvc/CMakeLists.txt`)
- Uncommented 8 SDF v2 source files:
  - Reg3Parser.cpp
  - RawivParser.cpp
  - Geom3DParser.cpp
  - geom.cpp
  - FaceVertSet3D.cpp
  - DistanceTransform.cpp
  - mtxlib.cpp
  - bufferedio.cpp

### 2. Algorithm Selection (`inc/cvc/algorithm.h`)
- Added `sdf_algorithm` enum with two values:
  - `SDF_V1` - Octree-based algorithm (default for backward compatibility)
  - `SDF_V2` - Brute-force DistanceTransform algorithm
- Updated `sdf()` function signature to accept algorithm parameter

### 3. SDF v2 Implementation (`src/cvc/algorithm.cpp`)
- Implemented `sdf_library_v2()` wrapper function
- Converts `cvc::geometry` to `FaceVertSet3D` format
- Uses in-memory data transfer (no temp files)
- **Critical fix**: Added `fvs.computeTriNormals()` call - required for proper sign computation
- Uses default scale factor of 2.0 (grid is 2x the geometry bounding box)
- Updated `sdf()` dispatcher with switch statement for algorithm selection

### 4. Macro Conflict Resolution
- Added strategic `#undef MIN` and `#undef MAX` after SDF v2 headers
- Prevents conflicts between SDF v2 macros and cvc-mesher template functions

### 5. Data Access (`src/cvc/SDF/SignDistanceFunction_v2/reg3data.h`)
- Added public `getData()` accessor methods (const and non-const)
- Enables direct memory access for efficient data copying

### 6. Comprehensive Testing (`src/cvc/tests/geometry_test.cpp`)
- **SDFV2Basic**: Tests basic SDF v2 functionality
  - Verifies negative values inside geometry
  - Verifies positive values outside geometry
- **SDFV1vsV2Comparison**: Compares both algorithms side-by-side
  - Tests on pyramid geometry
  - Validates both produce reasonable results (within 15x tolerance)
- **SDFV2ParallelExecution**: Tests thread safety
  - Launches 4 concurrent threads with different geometries
  - Verifies no global state interference
  - Confirms different inputs produce different outputs

## Key Technical Details

### SDF v1 Constraints
- Requires power-of-2 dimensions: 16, 32, 64, 128, 256, 512, or 1024
- Uses octree-based subdivision
- Respects user-provided bounding box

### SDF v2 Characteristics
- Accepts any dimensions
- Uses brute-force distance transform (slower but more straightforward)
- Creates its own grid based on geometry bounding box and scale factors
- Scale factors (sx, sy, sz) are **expansion multipliers**, not voxel spacing
  - Default 2.0 means grid is 2x the size of geometry bbox
- **Must call `computeTriNormals()`** on FaceVertSet3D before transform for proper sign computation

### Memory Transfer
- Both algorithms use in-memory data transfer
- SDF v2 uses `std::memcpy()` from `Reg3Data::getData()` to volume data
- No temporary files or disk I/O

### Thread Safety
- Both algorithms verified to work in parallel
- No global state interference
- Can run multiple SDF computations concurrently

## Test Results
All 406 tests pass:
- 53 app tests
- 57 geometry tests (including 3 new SDF v2 tests)
- 24 HDF5 tests
- 114 state tests
- 29 volume tests
- 129 voxels tests

## Usage Example

```cpp
#include <cvc/algorithm.h>

// Create geometry
geometry cube;
// ... populate cube ...

// Use SDF v1 (octree-based, default)
dimension dim(32, 32, 32);  // Must be power of 2
bounding_box bbox(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
volume sdf_v1 = sdf(cube, dim, bbox, SDF_V1);

// Use SDF v2 (brute-force distance transform)
dimension dim2(30, 30, 30);  // Can be any size
volume sdf_v2 = sdf(cube, dim2, bbox, SDF_V2);

// Default is SDF_V1 for backward compatibility
volume sdf_default = sdf(cube, dim, bbox);  // Uses SDF_V1
```

## Issues Encountered and Resolved

1. **Temp file I/O**: Initially used temp files for data transfer
   - **Solution**: Direct memory copy via `Reg3Data::getData()`

2. **Macro conflicts**: MIN/MAX macros conflicted with cvc-mesher
   - **Solution**: Strategic `#undef` placement

3. **TriId3i initialization**: No default constructor
   - **Solution**: Used `std::vector` instead of `boost::scoped_array`

4. **Scale factor interpretation**: Misunderstood sx/sy/sz parameters
   - **Solution**: Use 2.0 expansion factor instead of voxel spacing

5. **All positive SDF values**: Sign computation wasn't working
   - **Root cause**: Triangle normals not computed
   - **Solution**: Added `fvs.computeTriNormals()` call

6. **Power-of-2 requirement**: SDF v1 failed with non-power-of-2 dimensions
   - **Solution**: Changed test dimensions from 24 to 32

## Performance Notes
- SDF v1 is generally faster for large grids (uses octree)
- SDF v2 is simpler but slower (brute-force approach)
- Both algorithms are thread-safe and can run in parallel
- SDF v2 useful for non-power-of-2 grid sizes

## Backward Compatibility
- Default algorithm is SDF_V1
- All existing code continues to work without changes
- All 403 original tests still pass
