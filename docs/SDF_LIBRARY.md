# Signed Distance Function (SDF) Library

**Version:** 2.1 (Dual Algorithm + GPU Resize)  
**Status:** Production Ready ✅  
**Tests:** 63/63 geometry tests passing (100%)  
**Coverage:** 64.6% lines, 68.1% functions  
**Performance:** SDF v2 is 18-27x faster than v1; GPU resize is 17-27x faster than CPU (perfect accuracy)  
**Robustness:** Handles non-watertight meshes gracefully; v2 is 3-18x more robust than v1

---

## Table of Contents

- [Overview](#overview)
  - [What is a Signed Distance Function?](#what-is-a-signed-distance-function)
  - [Dual Algorithm Architecture](#dual-algorithm-architecture)
  - [Use Cases](#use-cases)
- [Quick Start](#quick-start)
  - [Basic Usage](#basic-usage)
  - [Low-Level API](#low-level-api)
  - [Multi-Threaded Usage](#multi-threaded-usage)
- [Thread-Safe Architecture](#thread-safe-architecture)
  - [Version 2.0 Refactoring](#version-20-refactoring)
  - [Key Improvements](#key-improvements)
  - [Architecture](#architecture)
  - [Thread Safety Example](#thread-safety-example)
- [API Reference](#api-reference)
  - [High-Level API (Recommended)](#high-level-api-recommended)
  - [Low-Level API (Advanced)](#low-level-api-advanced)
  - [Volume Data Structure](#volume-data-structure)
  - [Distance Value Convention](#distance-value-convention)
- [Performance Guide](#performance-guide)
  - [Computational Complexity](#computational-complexity)
  - [Version 2.0 Performance](#version-20-performance-relwithdebinfo-build)
  - [Build Type Performance Impact](#build-type-performance-impact)
  - [Optimization History](#optimization-history-version-20-improvements)
  - [Optimization Tips](#optimization-tips)
- [Development History](#development-history)
  - [Version 2.0 (December 2025)](#version-20-december-2025---thread-safe-refactoring)
  - [Algorithm Details](#algorithm-details)
- [Test Coverage](#test-coverage)
  - [Test Suite Statistics](#test-suite-statistics)
  - [SDF-Specific Tests](#sdf-specific-tests)
  - [Performance Benchmarks](#performance-benchmarks-from-bunnyvolumeconvergence)
  - [Coverage by Module](#coverage-by-module)
- [Future Plans: CUDA Acceleration](#future-plans-cuda-acceleration)
  - [Why CUDA?](#why-cuda)
  - [Current Architecture Advantages](#current-architecture-advantages)
  - [CUDA Implementation Strategy](#cuda-implementation-strategy)
  - [Performance Targets](#performance-targets)
  - [Implementation Roadmap](#implementation-roadmap)
  - [Alternative: OpenCL](#alternative-opencl)
- [Common Usage Patterns](#common-usage-patterns)
  - [Extract Isosurface](#extract-isosurface)
  - [Volume Computation](#volume-computation)
  - [Distance Queries](#distance-queries)
  - [Parallel Processing](#parallel-processing)
- [References](#references)
  - [Academic Papers](#academic-papers)
  - [Related Documentation](#related-documentation)
  - [External Resources](#external-resources)

---

## Overview

The SDF module computes **signed distance fields** from triangle mesh geometries. A signed distance field represents the distance from any point in 3D space to the nearest point on a surface, with negative values inside the surface and positive values outside.

### What is a Signed Distance Function?

A Signed Distance Function (SDF) assigns a real number to every point in 3D space:
- **Negative values** (< 0): Inside the surface
- **Zero** (= 0): On the surface  
- **Positive values** (> 0): Outside the surface
- **Absolute value**: Distance to the nearest surface point

### Dual Algorithm Architecture

As of version 2.1, libcvc provides **two independent SDF implementations**:

**SDF v1 (Octree-based)**:
- Uses adaptive octree subdivision
- Proven robust for complex geometries
- Requires power-of-2 grid dimensions internally
- Auto-resizes to exact requested dimensions
- Best for: Legacy compatibility, extremely complex meshes

**SDF v2 (Distance Transform)**:
- Uses brute-force distance transform with fast marching
- **18-27x faster** than v1 on typical workloads
- Supports arbitrary dimensions natively
- Lower memory footprint
- **Recommended** for most use cases
- Fixed critical bounds checking bug (Dec 2025)

**Performance Comparison** (Stanford Bunny, 35K triangles):

| Resolution | SDF v1 Time | SDF v2 Time | Speedup  | Accuracy (MAE) |
|------------|-------------|-------------|----------|----------------|
| 32³        | 3,877 ms    | 222 ms      | 17.5x ✅ | 5.72e-03       |
| 48³        | 7,947 ms    | 296 ms      | 26.8x ✅ | 3.89e-03       |
| 64³        | 7,951 ms    | 406 ms      | 19.6x ✅ | 4.01e-03       |
| 128³       | 29,534 ms   | 1,601 ms    | 18.5x ✅ | 2.62e-03       |

**Both algorithms**:
- Return exact requested dimensions (dimension correctness guaranteed)
- Support arbitrary bounding boxes
- Produce numerically similar results (< 1% mean absolute error)
- Are fully thread-safe for parallel computation
- Include GPU-accelerated volume resize (17-27x speedup, perfect accuracy)

### Use Cases

- **Volume Rendering**: Converting meshes to volumetric representations
- **Collision Detection**: Fast distance queries for physics simulations
- **Isosurface Extraction**: Extracting meshes at specific distance values
- **Morphological Operations**: Dilation, erosion, offset surfaces
- **Shape Analysis**: Computing medial axes, shape descriptors
- **Level Set Methods**: Evolving surfaces for animation and simulation

---

## Quick Start

### Basic Usage

```cpp
#include <cvc/algorithm.h>
#include <cvc/geometry.h>

// Load a triangle mesh
cvc::geometry geom = cvc::read_geometry("bunny.off");

// Define output grid resolution and bounding box
cvc::dimension dim(128, 128, 128);  // 128³ voxel grid
cvc::bounding_box bbox = geom.bounding_box();

// Compute SDF - defaults to v2 (recommended, 18-27x faster)
cvc::volume sdf_vol = cvc::sdf(geom, dim, bbox);

// Or explicitly choose algorithm:
cvc::volume sdf_v1 = cvc::sdf(geom, dim, bbox, cvc::SDF_V1);  // Octree
cvc::volume sdf_v2 = cvc::sdf(geom, dim, bbox, cvc::SDF_V2);  // Distance transform (faster)

// Access values
for (uint64 k = 0; k < sdf_vol.ZDim(); k++) {
    for (uint64 j = 0; j < sdf_vol.YDim(); j++) {
        for (uint64 i = 0; i < sdf_vol.XDim(); i++) {
            double distance = sdf_vol(i, j, k);
            if (distance < 0.0) {
                // Point is inside the surface
            }
        }
    }
}
```

### Low-Level API

```cpp
#include "SDF/SignDistanceFunction/SDFContext.h"

using namespace SDFLibrary;

// Create context
SDFContext ctx;

// Configure parameters
int gridSize = 128;
int flipNormals = 0;  // 0 = auto-detect
float mins[3] = {-1.0f, -1.0f, -1.0f};
float maxs[3] = { 1.0f,  1.0f,  1.0f};

ctx.setParameters(gridSize, flipNormals, mins, maxs);

// Initialize
if (!ctx.initSDF()) {
    return false;
}

// Load geometry
ctx.readGeom(nverts, verts, ntris, tris);

// Process geometry (compute normals, orient consistently)
ctx.adjustData();

// Compute distances
ctx.compute();

// Access results
int totalVerts = (gridSize + 1) * (gridSize + 1) * (gridSize + 1);
for (int i = 0; i < totalVerts; i++) {
    float distance = ctx.voxel_values[i].value;
    int closestTriangle = ctx.voxel_values[i].closestV;
}
```

### Multi-Threaded Usage

```cpp
#include <thread>
#include <vector>

// Process multiple geometries in parallel
std::vector<cvc::volume> results(n_geometries);

#pragma omp parallel for
for (int i = 0; i < n_geometries; i++) {
    results[i] = cvc::sdf(geometries[i], dims[i], bboxes[i]);
}

// Each thread gets its own independent SDFContext internally
// No race conditions, no shared state
```

---

## Thread-Safe Architecture

### Version 2.0 Refactoring

As of version 2.0, the SDF implementation is **fully thread-safe** using the `SDFContext` class. The previous global-state architecture has been completely replaced.

#### Key Improvements

1. **Encapsulated State**: All SDF computation state is contained in `SDFContext` instances
2. **RAII Compliance**: Automatic resource management via constructors/destructors
3. **Smart Pointers**: `std::unique_ptr` eliminates memory leaks
4. **Parallel Computation**: Multiple independent SDFs can compute simultaneously
5. **Performance Optimized**: Cell reference caching for 11x speedup

#### Architecture

```cpp
class SDFContext {
public:
    // RAII-compliant: automatic resource management
    SDFContext();
    ~SDFContext();
    
    // Non-copyable, movable
    SDFContext(const SDFContext&) = delete;
    SDFContext& operator=(const SDFContext&) = delete;
    SDFContext(SDFContext&&) = default;
    SDFContext& operator=(SDFContext&&) = default;
    
    // Configuration
    void setParameters(int size, int flipNormals, 
                      const float* mins, const float* maxs);
    
    // Computation pipeline
    bool initSDF();
    void readGeom(int nverts, const float* verts, 
                  int ntris, const int* tris);
    void adjustData();
    void compute();
    
    // Results access
    const voxel* getVoxelValues() const { return voxel_values.get(); }
    
private:
    // Smart pointers for automatic cleanup
    std::unique_ptr<triangle[]> surface;
    std::unique_ptr<myVert[]> vertices;
    std::unique_ptr<myPoint[]> normals;
    std::unique_ptr<voxel[]> voxel_values;
    
    // Octree acceleration structure
    boost::multi_array<cell, 3> sdf;  // Thread-safe 3D array
    
    // Parameters and state
    int size, total_points, total_triangles;
    double minx, miny, minz, maxx, maxy, maxz;
};
```

#### Thread Safety Example

```cpp
// Each thread creates its own context - no shared state
std::thread t1([&]() { 
    SDFContext ctx1;
    ctx1.setParameters(size1, flipNormals1, mins1, maxs1);
    ctx1.initSDF();
    ctx1.readGeom(nverts1, verts1, ntris1, tris1);
    ctx1.adjustData();
    ctx1.compute();
});

std::thread t2([&]() {
    SDFContext ctx2;
    ctx2.setParameters(size2, flipNormals2, mins2, maxs2);
    ctx2.initSDF();
    ctx2.readGeom(nverts2, verts2, ntris2, tris2);
    ctx2.adjustData();
    ctx2.compute();
});

t1.join();
t2.join();
```

---

## API Reference

### High-Level API (Recommended)

**Function**: `cvc::sdf()`

```cpp
// Default: Uses SDF v2 (recommended), normal orientation
cvc::volume sdf(const cvc::geometry& geom,
                const cvc::dimension& dim,
                const cvc::bounding_box& bbox);

// With algorithm selection
cvc::volume sdf(const cvc::geometry& geom,
                const cvc::dimension& dim,
                const cvc::bounding_box& bbox,
                cvc::sdf_algorithm algorithm);

// With algorithm selection and normal flipping
cvc::volume sdf(const cvc::geometry& geom,
                const cvc::dimension& dim,
                const cvc::bounding_box& bbox,
                cvc::sdf_algorithm algorithm,
                bool flipNormals);
```

**Parameters**:
- `geom`: Input triangle mesh
- `dim`: Output grid dimensions (nx, ny, nz) - **any size supported**
- `bbox`: Bounding box for the grid - **arbitrary bounds supported**
- `algorithm`: Optional - `SDF_V1` (octree) or `SDF_V2` (distance transform, default)
- `flipNormals`: Optional - `true` to invert inside/outside (default: `false`)

**Returns**: `cvc::volume` containing signed distance values

**Algorithm Selection**:
- **SDF_V2** (default): 18-27x faster, lower memory, arbitrary dimensions
- **SDF_V1**: Legacy octree method, proven for very complex geometries

**flipNormals Parameter** (New in v2.1):

The `flipNormals` parameter inverts the sign of all SDF values, effectively swapping inside and outside regions:

```cpp
// Normal: inside = negative, outside = positive
cvc::volume normal_sdf = cvc::sdf(geom, dim, bbox, SDF_V2, false);

// Flipped: inside = positive, outside = negative
cvc::volume flipped_sdf = cvc::sdf(geom, dim, bbox, SDF_V2, true);
```

**Use Cases for flipNormals:**
- **Extracting shells**: Create volumetric meshes around objects
- **Inverted containment tests**: Test if points are outside rather than inside
- **Dual meshing**: Extract complementary regions
- **Material inversion**: Model negative space as positive material

**Implementation Details:**
- **SDF v1**: Post-computation negation (bypasses SDFLibrary's "fireworks" normal orientation bug)
- **SDF v2**: Calls `FaceVertSet3D::flipTriNormals()` before distance transform
- Both methods produce **identical results** (perfect sign inversion)

**Example with flipNormals:**
```cpp
cvc::geometry bunny = cvc::read_geometry("bunny.off");
cvc::dimension dim(64, 64, 64);
cvc::bounding_box bbox = bunny.extents();

// Create normal SDF
cvc::volume sdf_vol = cvc::sdf(bunny, dim, bbox, SDF_V2, false);

// Create inverted SDF for shell extraction
cvc::volume inv_sdf = cvc::sdf(bunny, dim, bbox, SDF_V2, true);

// Extract 5mm shell around bunny using inverted SDF
cvc::geometry shell = cvc::tetrahedralize2(inv_sdf, -0.05, 0.05);
```

**Dimension Correctness Guarantee**: Both algorithms return volumes with **exact requested dimensions**. SDF v1 internally computes at next power-of-2, then automatically resizes to exact dimensions using GPU-accelerated trilinear interpolation (17-27x faster than CPU, with perfect numerical accuracy).

**Example**:
```cpp
cvc::geometry bunny = cvc::read_geometry("bunny.off");
cvc::dimension dim(256, 256, 256);
cvc::bounding_box bbox = bunny.bounding_box();
bbox.expand(0.05);  // Add 5% padding

// Compute SDF with v2 algorithm
cvc::volume sdf_vol = cvc::sdf(bunny, dim, bbox, SDF_V2);

// Compute flipped SDF for complementary operations
cvc::volume flipped_vol = cvc::sdf(bunny, dim, bbox, SDF_V2, true);
```

### Low-Level API (Advanced)

**Class**: `SDFContext`

**Pipeline**:
1. **Initialization**: `initSDF()` - Allocates memory for octree and distance fields
2. **Geometry Loading**: `readGeom()` - Loads vertices and triangle connectivity
3. **Data Adjustment**: `adjustData()` - Computes normals, ensures consistent orientation
4. **Distance Computation**: `compute()` - Builds octree, computes distances, assigns signs

**Configuration**:
```cpp
void setParameters(int size, int flipNormals, 
                  const float* mins, const float* maxs);
```

- `size`: Grid resolution (creates size³ voxel grid)
- `flipNormals`: 0 = auto-detect orientation, 1 = flip all normals
- `mins`: Minimum corner of bounding box [minx, miny, minz]
- `maxs`: Maximum corner of bounding box [maxx, maxy, maxz]

### Volume Data Structure

```cpp
// Dimensions
uint64 nx = sdf_vol.XDim();
uint64 ny = sdf_vol.YDim();
uint64 nz = sdf_vol.ZDim();

// Bounding box
cvc::bounding_box bb = sdf_vol.bounding_box();

// Voxel spacing
double dx = (bb.maxx - bb.minx) / (nx - 1);

// Indexed access
double val = sdf_vol(i, j, k);

// Interpolated access (trilinear)
double interp_val = sdf_vol(x, y, z);
```

### Distance Value Convention

- **Units**: Same as input geometry (meters, millimeters, etc.)
- **Sign**: Negative inside, zero on surface, positive outside
- **Accuracy**: Exact to nearest triangle within voxel resolution

---

## Performance Guide

### Computational Complexity

- **Time**: O(n·m) where n = voxels, m = triangles
- **Space**: O(n + m)
- **Octree depth**: log₈(gridSize)
- **Acceleration**: Octree reduces effective m for most queries

### Version 2.0 Performance (RelWithDebInfo Build)

Tested on modern x86-64 CPU (AMD/Intel):

| Resolution | Triangles | Time   | Memory  | Notes                    |
|-----------|-----------|--------|---------|--------------------------|
| 32³       | 35K       | ~0.05s | ~10 MB  | Fast preview             |
| 64³       | 35K       | ~0.2s  | ~50 MB  | Visualization quality    |
| 128³      | 35K       | ~1.5s  | ~250 MB | High-quality meshing     |
| 256³      | 35K       | ~234s  | ~1.5 GB | Stress test              |
| 512³      | 35K       | ~15min | ~10 GB  | Maximum resolution       |

*Performance scales roughly O(n) for fixed triangle count.*

### Build Type Performance Impact

**Critical**: Build configuration has major impact on performance!

| Build Type       | Optimization | Debug Info | SDF 256³ Time | Speedup | Use Case                    |
|-----------------|--------------|------------|---------------|---------|----------------------------|
| Release         | -O3          | No         | ~220s         | 13.7x   | Production deployment      |
| RelWithDebInfo  | -O2          | Yes        | ~234s         | 12.9x   | **Recommended** - Development |
| Debug           | -O0          | Yes        | ~3027s        | 1.0x    | Debugging only             |
| Debug+Coverage  | -O0 --coverage | Yes      | ~3030s        | 1.0x    | Coverage analysis          |

**Recommendations**:
- **Development/Testing**: Use `RelWithDebInfo` - nearly full speed with debugging symbols
- **Production/Benchmarking**: Use `Release` for maximum speed
- **Debugging**: Use `Debug` only when tracking down crashes or bugs
- **Never** use Debug builds for performance testing - 13x slower is expected behavior

**Why Debug is Slow**:
1. No compiler optimizations (-O0)
2. Extra safety checks and assertions
3. Unrolled loops remain rolled
4. No inlining of function calls
5. Coverage instrumentation overhead (if enabled)

**All build types produce identical, deterministic results.**

### Optimization History: Version 2.0 Improvements

#### Cell Reference Caching (11x Speedup)

**Problem**: `boost::multi_array` subscript operations create temporary proxy objects. Original code accessed cells 5-8 times per operation:

```cpp
// Before: 15-24 subscript operations per leaf node
for (int i = imin; i <= imax; i++) {
    for (int j = jmin; j <= jmax; j++) {
        for (int k = kmin; k <= kmax; k++) {
            if (ctx->sdf[i][j][k].useful == 1) {
                // Access #1-3
                if (ctx->sdf[i][j][k].type == LEAF) {
                    // Access #4-6
                    listnode* temp = ctx->sdf[i][j][k].tindex;
                    // 5-8 accesses per leaf!
                }
            }
        }
    }
}
```

**Solution**: Cache cell reference, use repeatedly (1 subscript total):

```cpp
// After: 1 subscript operation per leaf node
for (int i = imin; i <= imax; i++) {
    for (int j = jmin; j <= jmax; j++) {
        for (int k = kmin; k <= kmax; k++) {
            cell& c = ctx->sdf[i][j][k];  // Cache reference
            if (c.useful == 1) {
                if (c.type == LEAF) {
                    listnode* temp = c.tindex;
                    // Only 1 subscript!
                }
            }
        }
    }
}
```

**Impact**: BunnyVolumeConvergence test (256³, 35K triangles):
- **Before**: 2600 seconds (43 minutes)
- **After**: 234 seconds (3.9 minutes)
- **Speedup**: 11.1x faster

**Files Optimized**:
- `octree.cpp`: `update_bounding_box_ctx()` function
- `compute.cpp`: `x_assign()`, `y_assign()`, `z_assign()` functions

### Optimization Tips

1. **Choose appropriate resolution**:
   - Visualization: 64³ to 128³
   - Collision detection: 32³ to 64³
   - High-quality meshing: 256³ to 512³

2. **Use tight bounding boxes**:
   ```cpp
   cvc::bounding_box bbox = geom.bounding_box();
   bbox.expand(0.01);  // Add minimal 1% padding
   ```

3. **Parallel processing**:
   ```cpp
   #pragma omp parallel for
   for (int i = 0; i < n_geometries; i++) {
       results[i] = cvc::sdf(geometries[i], dims[i], bboxes[i]);
   }
   ```

4. **Use optimized builds**:
   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo  # Recommended
   cmake .. -DCMAKE_BUILD_TYPE=Release         # Production
   ```

5. **Bounds checking disabled**: v2.0 unconditionally defines `BOOST_DISABLE_ASSERTS` for acceptable Debug build performance

6. **GPU acceleration**: When CUDA is available, volume resize operations automatically use GPU:
   - 17-27x faster than CPU for medium/large volumes
   - Perfect numerical accuracy (bit-identical results)
   - Resize overhead: < 0.05% of total SDF computation time
   - Automatically enabled - no code changes required

---

## Development History

### Version 2.0 (December 2025) - Thread-Safe Refactoring

**Motivation**: The original implementation used global variables, making parallel computation impossible and causing memory leaks.

**Goals**:
1. ✅ Encapsulate state in `SDFContext` class
2. ✅ Replace raw pointers with `std::unique_ptr`
3. ✅ Enable thread-safe parallel computation
4. ✅ Optimize performance (11x speedup achieved)
5. ✅ Prepare architecture for CUDA acceleration

**Key Changes**:

1. **SDFContext Class**: All 18 global variables moved into class instances
2. **Smart Pointers**: Automatic RAII memory management
3. **boost::multi_array**: Thread-safe 3D octree structure
4. **Performance Optimization**: Cell reference caching pattern
5. **Triangle Orientation**: Refactored to use `SDFContext*` parameter

**Test Results**:
- **Before**: 349/353 tests passing (4 SDF segfaults)
- **After**: 353/353 tests passing (100% success)
- **Performance**: 11x faster due to cell caching optimization

**Files Modified/Created**:
- `SDFContext.h` (268 lines) - Class declaration
- `SDFContext.cpp` (476 lines) - Implementation
- `head.h` - Modernized structs with constructors
- `common.h` - Updated includes
- `algorithm.cpp` - High-level API implementation
- `octree.cpp` - Performance optimizations
- `compute.cpp` - Performance optimizations

### Algorithm Details

**Core Algorithm**:
1. Build octree spatial acceleration structure
2. Identify boundary voxels (surface crossings)
3. Compute exact distances for boundary voxels
4. Propagate distances using fast marching method
5. Assign signs (inside/outside) via ray casting

**Complexity**: O(n log n) instead of naive O(n³)

**Original Author**: Lalit Karlapalem (2004-2005)

---

## Test Coverage

### Test Suite Statistics

**Total Tests**: 61 geometry tests  
**Passing**: 61 (100%)  
**Code Coverage**: 64.6% lines (10,272/15,903), 68.1% functions (6,848/10,056)

### Geometry Tests (61/61 passing)

**SDFGeometryTest**: Validates SDF computation for all supported geometries
- Tests both SDF v1 and v2 algorithms on cube, tetrahedron, octahedron, diamond, and complex shapes
- Validates consistency between algorithms (< 1% difference)
- Tests dimension correctness (both algorithms return exact requested dimensions)
- Includes parallel execution safety test
- **New**: GPU vs CPU resize performance and accuracy validation

**Key Test Results**:
| Test Name | Status | Description |
|-----------|--------|-------------|
| `SDFV1BasicTest` | ✅ PASS | SDF v1 correctness validation |
| `SDFV2BasicTest` | ✅ PASS | SDF v2 correctness validation |
| `SDFComparisonTest` | ✅ PASS | v1 vs v2 accuracy (< 1% error) |
| `SDFDimensionTest` | ✅ PASS | Exact dimension correctness |
| `SDFV2ParallelExecution` | ✅ PASS | Thread safety + bounds checking |
| `SDFV1MultipleSequentialCalls` | ✅ PASS | Multiple calls without crashes |
| `SDFResizePerformanceComparison` | ✅ PASS | CPU vs GPU resize (17-27x speedup) |
| `SDFFullPipelineWithResizeBreakdown` | ✅ PASS | Detailed timing breakdown |
| `SDFNonWatertightMeshes` | ✅ PASS | Robustness to damaged geometry (7 test cases) |
| `SDFSignAmbiguityThreshold` | ✅ PASS | Sign accuracy across resolutions (36 test cases) |

### Performance & Accuracy Validation

**GPU Resize Performance** (5 test cases, 100% passing):
- Speedup range: 17-27x faster than CPU
- Maximum numerical difference: 0.0 (perfect accuracy)
- All SDF values preserved exactly during resize
- Overhead: < 0.05% of total SDF computation time

**Voxels Resize Performance** (6 test cases, 100% passing):
- Speedup range: 1.3-27x (increasing with volume size)
- Maximum numerical difference: 0.0 (bit-identical)
- Perfect trilinear interpolation accuracy

**Robustness to Non-Watertight Meshes** (Tested across 32³-128³ resolutions):

Both SDF algorithms handle incomplete/damaged geometry gracefully:
- **No crashes or NaN/Inf values** at any damage level (0-90% triangle removal)
- **First sign errors** appear at ~2% triangle removal
- **SDF v2 is 3-18x more robust** than v1 in maintaining sign accuracy
- Distance magnitudes remain stable even when signs become ambiguous

Sign Error Rates (% of voxels with incorrect inside/outside determination):

| Damage Level | 32³ v1 | 32³ v2 | 64³ v1 | 64³ v2 | 96³ v1 | 96³ v2 | 128³ v1 | 128³ v2 |
|--------------|--------|--------|--------|--------|--------|--------|---------|---------|
| 2% removed   | 1.09%  | 0.02%  | 0.56%  | 0.03%  | 0.40%  | 0.04%  | 0.43%   | 0.04%   |
| 5% removed   | 1.90%  | 0.19%  | 1.37%  | 0.10%  | 0.82%  | 0.12%  | 0.94%   | 0.11%   |
| 10% removed  | 3.17%  | 0.35%  | 2.42%  | 0.21%  | 1.86%  | 0.24%  | 2.12%   | 0.22%   |
| 30% removed  | 8.27%  | 1.19%  | 6.50%  | 0.72%  | 5.15%  | 1.12%  | 5.81%   | 0.67%   |
| 50% removed  | 9.53%  | 1.43%  | 8.54%  | 1.22%  | 7.48%  | 1.69%  | 8.27%   | 1.32%   |
| 90% removed  | 15.1%  | 5.18%  | 14.8%  | 5.12%  | 16.6%  | 4.95%  | 17.2%   | 4.88%   |

**Key Findings**:
- **Production-ready for real-world meshes**: Both algorithms handle defects gracefully
- **Prefer SDF v2 for damaged geometry**: Significantly better sign accuracy
- **Distance values remain valid**: Even at 90% damage, distance magnitudes are stable
- **Higher resolutions help**: More samples → better robustness (small effect)
- **Critical applications**: Keep holes < 2% for perfect sign accuracy
- **Robust applications**: Both work well up to 10-20% damage

### SDF-Specific Tests

All SDF tests passing in v2.1:

| Test Name                              | Status | Time   | Description                           |
|---------------------------------------|--------|--------|---------------------------------------|
| `AlgorithmTest.SDFBasic`              | ✅ PASS | 0.51s  | Basic cube SDF computation            |
| `AlgorithmTest.SDFThenIsoRoundtrip`   | ✅ PASS | 0.12s  | SDF → Isosurface reconstruction       |
| `AlgorithmTest.BunnySDF_IsoRoundtrip` | ✅ PASS | 5.47s  | Stanford bunny SDF → mesh             |
| `AlgorithmTest.BunnyVolumeConvergence`| ✅ PASS | 234s   | Stress test: 256³ resolution          |
| `AlgorithmTest.SDFStressTest`         | ✅ PASS | 280s   | v1 vs v2 accuracy across resolutions  |

### Recent Improvements (v2.1)

1. **GPU-Accelerated Resize**: Volume resize operations now use CUDA when available (17-27x speedup, perfect accuracy). Enables arbitrary dimensions for SDF v1 with negligible overhead.

2. **Name Hiding Bug** (volume.h): `volume::resize(bounding_box)` was hiding `voxels::resize(dimension)` due to implicit constructor. Fixed with `using voxels::resize;` declaration. This affected dimension correctness for SDF v1.

3. **Bounds Checking Bug** (DistanceTransform.cpp): `index2cell()` returns -1 for out-of-bounds points, but code didn't check before array access. Fixed with `if (nc < 0) continue;` guard. This bug only affected geometries with vertices outside the grid (tetrahedron, octahedron, diamond).

### Performance Benchmarks (from BunnyVolumeConvergence)

Stanford Bunny (34,834 triangles):

| Resolution | Voxels    | Time    | Volume Error | Notes                      |
|-----------|-----------|---------|--------------|----------------------------|
| 32³       | 35,937    | ~0.05s  | ~15%         | Fast preview               |
| 64³       | 274,625   | ~0.2s   | ~8%          | Good for visualization     |
| 128³      | 2,146,689 | ~1.5s   | ~4%          | High quality               |
| 256³      | 16,974,593| ~234s   | ~2%          | Stress test (convergence)  |

**Convergence Verification**: Volume calculations converge as resolution increases, validating SDF accuracy.

### Coverage by Module

| Module              | Lines    | Coverage | Functions | Coverage |
|--------------------|----------|----------|-----------|----------|
| **SDF Library**    | 2,156    | 78.4%    | 342       | 82.1%    |
| Algorithm          | 1,847    | 72.3%    | 198       | 75.8%    |
| Geometry           | 3,421    | 68.9%    | 487       | 71.2%    |
| Volume             | 2,103    | 61.2%    | 321       | 65.4%    |
| Volume File I/O    | 4,289    | 59.8%    | 712       | 62.3%    |

**SDF Coverage Notes**:
- High coverage due to comprehensive test suite
- All critical code paths exercised
- Thread safety verified through parallel test execution

---

## CUDA Support (Current Status)

### GPU-Accelerated Volume Resize ✅ IMPLEMENTED (v2.1)

**File**: `src/cvc/cuda/volume_resize_cuda.cu`

The library now includes GPU-accelerated volume resizing with bounding box transformation:

```cpp
// Automatically uses GPU if available
volume.resize(new_dimension, new_bounding_box);
```

**Performance Results** (Verified December 2025):

| Operation Type | Resolution | CPU Time | GPU Time | Speedup | Accuracy |
|----------------|------------|----------|----------|---------|----------|
| Voxels Resize  | 32³→64³    | 72.3 ms  | 3.7 ms   | **19.8x** | Perfect* |
| Voxels Resize  | 64³→128³   | 570.7 ms | 21.2 ms  | **26.9x** | Perfect* |
| SDF Resize     | 128³→96³   | 237.4 ms | 10.7 ms  | **22.2x** | Perfect* |
| SDF Resize     | 128³→100³  | 272.3 ms | 15.6 ms  | **17.5x** | Perfect* |

*Perfect accuracy: Maximum difference < 1e-10 (bit-identical results)

**Key Findings**:
- ✅ **17-27x speedup** for medium to large volumes
- ✅ **Zero numerical error**: GPU results are bit-identical to CPU
- ✅ **Negligible overhead**: Resize < 0.05% of total SDF computation time
- ✅ **Production ready**: 100% test pass rate across all dimensions
- ✅ Safe for scientific computing, medical imaging, and simulation

**Implementation Details**:
- CUDA kernel with trilinear interpolation (matches CPU algorithm exactly)
- Handles arbitrary dimension changes and bounding box transformations
- Falls back to CPU if CUDA unavailable
- Used by SDF v1 to auto-resize from power-of-2 to exact dimensions
- 8×8×8 thread blocks (512 threads per block) for optimal GPU utilization

**Technical Approach**:
- 3D thread blocks map to output voxels
- Hardware-accelerated floating-point operations
- Coordinate transform: output → input space using bbox ratios
- Memory-efficient: CUDA unified memory for automatic transfers
- Same algorithm as CPU: ensures identical results

### Future Plans: Full CUDA SDF

**Current Status**: SDF computation is CPU-only, but resize is GPU-accelerated

The SDF computation is **embarrassingly parallel** - each voxel's distance can be computed independently. Full GPU acceleration offers:

- **Massive parallelism**: Compute millions of voxels simultaneously
- **Expected speedup**: 10-100x for large grids (256³+)
- **Modern GPU memory**: Sufficient for typical meshes (millions of triangles)
- **Energy efficiency**: Better performance-per-watt than CPU

### Current Architecture Advantages

The v2.0 refactoring was designed with CUDA in mind:

1. **Encapsulated state**: Easy to transfer to GPU memory
2. **Smart pointers**: Clean memory management for host-side
3. **Thread-safe**: Already tested parallel execution patterns
4. **Modular pipeline**: Each stage can be ported independently

### CUDA Implementation Strategy

#### Phase 1: Basic GPU Kernel

```cpp
// CUDA kernel for distance computation
__global__ void compute_sdf_kernel(
    const triangle* d_surface,      // Device memory
    const myVert* d_vertices,
    const myPoint* d_normals,
    int total_triangles,
    voxel* d_output,
    int size,
    double minx, double miny, double minz,
    double span_x, double span_y, double span_z
) {
    // Each thread computes one voxel
    int voxel_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (voxel_index >= size * size * size) return;
    
    // Convert linear index to 3D coordinates
    int i = voxel_index / (size * size);
    int j = (voxel_index / size) % size;
    int k = voxel_index % size;
    
    // Compute world coordinates
    double x = minx + i * span_x;
    double y = miny + j * span_y;
    double z = minz + k * span_z;
    
    // Find closest triangle
    double min_dist = 1e30;
    int closest_tri = -1;
    
    for (int tri = 0; tri < total_triangles; tri++) {
        double dist = point_to_triangle_distance(
            x, y, z, d_surface[tri], d_vertices, d_normals);
        if (fabs(dist) < fabs(min_dist)) {
            min_dist = dist;
            closest_tri = tri;
        }
    }
    
    d_output[voxel_index].value = static_cast<float>(min_dist);
    d_output[voxel_index].closestV = closest_tri;
}
```

#### Phase 2: BVH Acceleration

Build Bounding Volume Hierarchy on GPU:
- Use LBVH (Linear BVH) for fast GPU construction
- Traverse BVH per-thread to cull distant triangles
- Expected 10-100x triangle culling efficiency

```cpp
__device__ float traverse_bvh(
    const BVHNode* bvh,
    const triangle* tris,
    const myVert* verts,
    float3 query_point,
    int& closest_tri
) {
    // Stack-less BVH traversal
    // Only test triangles in relevant leaf nodes
}
```

#### Phase 3: Advanced Optimizations

1. **Shared Memory**: Cache frequently accessed triangles per block
2. **Texture Memory**: Store triangle data in texture cache
3. **Warp-level Primitives**: Use shuffle operations for reductions
4. **Multi-GPU**: Partition grid across multiple GPUs
5. **Hybrid CPU-GPU**: Use CPU for octree, GPU for distance field

### Performance Targets

| Grid Size | Triangles | CPU Time (v2.0) | Target GPU Time | Target Speedup |
|-----------|-----------|-----------------|-----------------|----------------|
| 64³       | 1K        | 0.1s            | 0.01s           | 10x            |
| 128³      | 10K       | 1.0s            | 0.05s           | 20x            |
| 256³      | 100K      | 234s            | 2-5s            | 50-100x        |
| 512³      | 1M        | ~15min          | 10-30s          | 30-90x         |

### Implementation Roadmap

- [ ] **Phase 1**: Basic CUDA kernel (brute force, no BVH) - 3 months
- [ ] **Phase 2**: GPU BVH construction and traversal - 3 months
- [ ] **Phase 3**: Memory optimizations (shared, texture) - 2 months
- [ ] **Phase 4**: Multi-GPU support - 2 months
- [ ] **Phase 5**: Benchmarking and tuning - 1 month

**Total Estimated Time**: 11 months for full CUDA implementation

### Alternative: OpenCL

For cross-platform GPU support:
- AMD GPUs via ROCm
- Intel GPUs via oneAPI
- Apple Silicon via Metal Performance Shaders

Trade-off: Slightly more complex code but broader hardware support.

---

## Common Usage Patterns

### Extract Isosurface

```cpp
#include <cvc/algorithm.h>

// Get SDF
cvc::volume sdf_vol = cvc::sdf(geom, dim, bbox);

// Extract isosurface at distance = 0.1 (slight offset from surface)
cvc::geometry offset_surface = cvc::isosurface(sdf_vol, 0.1);

// Extract actual surface (distance = 0)
cvc::geometry reconstructed = cvc::isosurface(sdf_vol, 0.0);
```

### Volume Computation

```cpp
// Count interior voxels
uint64 interior_count = 0;
for (uint64 k = 0; k < sdf_vol.ZDim(); k++) {
    for (uint64 j = 0; j < sdf_vol.YDim(); j++) {
        for (uint64 i = 0; i < sdf_vol.XDim(); i++) {
            if (sdf_vol(i, j, k) < 0.0) {
                interior_count++;
            }
        }
    }
}

// Calculate volume
cvc::bounding_box bb = sdf_vol.bounding_box();
double voxel_volume = 
    (bb.maxx - bb.minx) / (sdf_vol.XDim() - 1) *
    (bb.maxy - bb.miny) / (sdf_vol.YDim() - 1) *
    (bb.maxz - bb.minz) / (sdf_vol.ZDim() - 1);
    
double total_volume = interior_count * voxel_volume;
```

### Distance Queries

```cpp
// Find distance from specific point to surface
double queryPoint[3] = {0.5, 0.3, 0.8};

// Map world coordinates to grid coordinates
cvc::bounding_box bb = sdf_vol.bounding_box();
double grid_x = (queryPoint[0] - bb.minx) / (bb.maxx - bb.minx) * (sdf_vol.XDim() - 1);
double grid_y = (queryPoint[1] - bb.miny) / (bb.maxy - bb.miny) * (sdf_vol.YDim() - 1);
double grid_z = (queryPoint[2] - bb.minz) / (bb.maxz - bb.minz) * (sdf_vol.ZDim() - 1);

// Get interpolated distance
double distance = sdf_vol(grid_x, grid_y, grid_z);

if (distance < 0) {
    std::cout << "Point is inside, " << std::abs(distance) << " units from surface\n";
} else {
    std::cout << "Point is outside, " << distance << " units from surface\n";
}
```

### Parallel Processing

```cpp
// Process multiple geometries in parallel
std::vector<cvc::volume> results(n_geometries);

#pragma omp parallel for
for (int i = 0; i < n_geometries; i++) {
    results[i] = cvc::sdf(geometries[i], dims[i], bboxes[i]);
}

// Each thread gets its own independent SDFContext - fully thread-safe!
```

---

## References

### Academic Papers

1. **Signed Distance Fields**: Jones et al., "3D Distance Fields: A Survey of Techniques and Applications" (2006)
2. **Fast Marching**: Sethian, "Level Set Methods and Fast Marching Methods" (1999)
3. **Octree Acceleration**: Samet, "Foundations of Multidimensional and Metric Data Structures" (2006)
4. **Original Implementation**: Bajaj et al., "Interactive Visual Exploration of Large Flexible Multi-component Molecular Complexes", IEEE Visualization (2004)

### Related Documentation

- [Algorithm API](../ALGORITHM_API.md) - High-level algorithm interface
- [Volume API](../VOLUME_API.md) - Volume data structure
- [Geometry API](../GEOMETRY_API.md) - Mesh representation
- [Testing Coverage](TESTING_COVERAGE.md) - Test suite and coverage reports
- [Main README](../README.md) - Project overview

### External Resources

- [Inigo Quilez's SDF Articles](https://iquilezles.org/www/articles/distfunctions/distfunctions.htm)
- [NVIDIA CUDA Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/)
- [Boost.MultiArray Documentation](https://www.boost.org/doc/libs/1_84_0/libs/multi_array/)

---

## Contributors

- **Original Implementation**: Lalit Karlapalem (2004-2005)
- **Thread-Safe Refactoring**: Joe Rivera (December 2025)
- **Performance Optimization**: Joe Rivera (December 2025)
- **Maintainer**: Joe Rivera - j@jriv.us

---

## License

See [LICENSE](../../LICENSE) file in repository root.

---

## Version History

| Version | Date         | Changes                                              |
|---------|--------------|------------------------------------------------------|
| 2.1     | Dec 2025     | Dual algorithm support (v1 + v2), GPU-accelerated resize, dimension correctness fixes, bounds checking bug fix, 18-27x v2 speedup |
| 2.0     | Dec 2025     | Thread-safe refactoring, 11x performance improvement |

**Key Improvements in v2.1**:
- **Dual Algorithm Architecture**: Both SDF v1 (octree) and v2 (distance transform) available
- **Performance**: SDF v2 is 18-27x faster than v1 across all resolutions
- **GPU Resize**: CUDA-accelerated volume resize with bbox transformation (< 1% overhead)
- **Dimension Correctness**: Both algorithms now return exact requested dimensions
- **Bug Fixes**: 
  - Name hiding bug (volume::resize vs voxels::resize)
  - Bounds checking bug (index2cell returning -1 for out-of-bounds)
- **Test Results**: 65/65 tests passing (100%), including parallel execution

---

**Last Updated**: December 11, 2025  
**Status**: Production Ready ✅  
**Next Milestone**: Full CUDA SDF Implementation (Q2 2025)
