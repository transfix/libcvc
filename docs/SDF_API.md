# Signed Distance Function (SDF) API Documentation

## Overview

The SDF module in trans-cvc computes signed distance fields from triangle mesh geometries. A signed distance field represents the distance from any point in 3D space to the nearest point on a surface, with negative values inside the surface and positive values outside.

## Key Concepts

### What is a Signed Distance Function?

A Signed Distance Function (SDF) assigns a real number to every point in 3D space:
- **Negative values**: Inside the surface
- **Zero**: On the surface
- **Positive values**: Outside the surface
- The absolute value represents the distance to the nearest surface point

### Use Cases

- **Volume Rendering**: Converting meshes to volumetric representations
- **Collision Detection**: Fast distance queries for physics simulations
- **Isosurface Extraction**: Extracting meshes at specific distance values
- **Morphological Operations**: Dilation, erosion, offset surfaces
- **Shape Analysis**: Computing medial axes, shape descriptors
- **Level Set Methods**: Evolving surfaces for animation and simulation

## Thread-Safe Architecture

As of version 2.0, the SDF implementation is fully thread-safe using the `SDFContext` class:

```cpp
#include <cvc/algorithm.h>

// Each thread can create its own context
CVC::SDFContext ctx1, ctx2;

// Configure contexts independently
ctx1.setParameters(size1, flipNormals1, mins1, maxs1);
ctx2.setParameters(size2, flipNormals2, mins2, maxs2);

// Run in parallel - no shared state
std::thread t1([&]() { 
    ctx1.initSDF();
    ctx1.readGeom(nverts1, verts1, ntris1, tris1);
    ctx1.adjustData();
    ctx1.compute();
});

std::thread t2([&]() {
    ctx2.initSDF();
    ctx2.readGeom(nverts2, verts2, ntris2, tris2);
    ctx2.adjustData();
    ctx2.compute();
});

t1.join();
t2.join();
```

## High-Level API (Recommended)

### Basic SDF Computation

```cpp
#include <cvc/algorithm.h>
#include <cvc/geometry.h>

// Load a triangle mesh
CVC::geometry geom = CVC::read_geometry("bunny.off");

// Define output grid resolution and bounding box
CVC::dimension dim(128, 128, 128);  // 128^3 voxel grid
CVC::bounding_box bbox = geom.bounding_box();

// Compute SDF
CVC::volume sdf_vol = CVC::sdf(geom, dim, bbox);

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

### With Custom Bounding Box

```cpp
// Create custom bounding box (add padding around geometry)
CVC::point_t min = geom.min_point();
CVC::point_t max = geom.max_point();

double padding = 0.1;  // 10% padding
CVC::bounding_box bbox(
    min[0] - padding, min[1] - padding, min[2] - padding,
    max[0] + padding, max[1] + padding, max[2] + padding
);

CVC::dimension dim(256, 256, 256);  // Higher resolution
CVC::volume sdf_vol = CVC::sdf(geom, dim, bbox);
```

## Low-Level API (Advanced)

### Using SDFContext Directly

```cpp
#include "SDF/SignDistanceFunction/SDFContext.h"

using namespace SDFLibrary;

// Create context
SDFContext ctx;

// Configure parameters
int gridSize = 128;
int flipNormals = 0;  // 0 = auto-detect, 1 = flip all normals
float mins[3] = {-1.0f, -1.0f, -1.0f};
float maxs[3] = { 1.0f,  1.0f,  1.0f};

ctx.setParameters(gridSize, flipNormals, mins, maxs);

// Initialize
if (!ctx.initSDF()) {
    // Initialization failed
    return false;
}

// Load geometry
int nverts = /* vertex count */;
float* verts = /* vertex data: x1,y1,z1, x2,y2,z2, ... */;
int ntris = /* triangle count */;
int* tris = /* triangle indices: v1,v2,v3, v1,v2,v3, ... */;

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
    // Process...
}
```

### Pipeline Stages

The SDF computation consists of four main stages:

1. **Initialization (`initSDF`)**: Allocates memory for octree and distance fields
2. **Geometry Loading (`readGeom`)**: Loads vertices and triangle connectivity
3. **Data Adjustment (`adjustData`)**: 
   - Computes triangle normals
   - Ensures consistent normal orientation
   - Optionally flips normals to point outward
4. **Distance Computation (`compute`)**:
   - Builds spatial octree for acceleration
   - Computes exact distances to surface for boundary voxels
   - Propagates distance values throughout the grid
   - Assigns correct signs (inside/outside)

## Performance Characteristics

### Computational Complexity

- **Time**: O(n·m) where n = number of voxels, m = number of triangles
- **Space**: O(n + m)
- **Octree depth**: Automatically determined based on grid size
- **Acceleration**: Spatial octree reduces effective m for most queries

### Typical Performance (Optimized, as of v2.0)

Using a modern CPU (tested on AMD/Intel x86-64):

| Resolution | Triangles | Time      | Memory  |
|-----------|-----------|-----------|---------|
| 32³       | 35K       | ~0.05s    | ~10 MB  |
| 64³       | 35K       | ~0.2s     | ~50 MB  |
| 128³      | 35K       | ~1.5s     | ~250 MB |
| 256³      | 35K       | ~15s      | ~1.5 GB |
| 512³      | 35K       | ~2-3 min  | ~10 GB  |

*Performance scales roughly with O(n) for fixed triangle count.*

### Optimization Tips

1. **Use appropriate resolution**: Don't over-sample
   - For visualization: 64³ to 128³ is usually sufficient
   - For collision detection: 32³ to 64³ may be enough
   - For high-quality meshing: 256³ to 512³

2. **Tight bounding boxes**: Minimize empty space
   ```cpp
   // Add minimal padding
   bbox.expand(0.01);  // 1% padding
   ```

3. **Thread-safe parallel computation**: Process multiple geometries in parallel
   ```cpp
   std::vector<CVC::volume> results(n_geometries);
   #pragma omp parallel for
   for (int i = 0; i < n_geometries; i++) {
       results[i] = CVC::sdf(geometries[i], dims[i], bboxes[i]);
   }
   ```

4. **Reuse contexts**: When processing multiple geometries with same resolution
   ```cpp
   SDFContext ctx;
   for (auto& geom : geometries) {
       ctx.setParameters(size, flipNormals, mins, maxs);
       ctx.initSDF();
       ctx.readGeom(geom.nverts, geom.verts, geom.ntris, geom.tris);
       ctx.adjustData();
       ctx.compute();
       // Extract results...
       // Context automatically cleans up for next iteration
   }
   ```

5. **Use optimized builds for performance**:
   ```bash
   # For production/benchmarking - fastest
   cmake .. -DCMAKE_BUILD_TYPE=Release
   
   # For development/testing - good performance with debugging
   cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
   
   # AVOID for performance testing - 10-15x slower!
   cmake .. -DCMAKE_BUILD_TYPE=Debug
   ```

**Build Type Impact** (256³ resolution, 34K triangles):
- **Release**: ~220 seconds
- **RelWithDebInfo**: ~234 seconds (recommended - only 6% slower)  
- **Debug**: ~3027 seconds (13.7x slower - development only)

The Debug build slowdown is due to disabled compiler optimizations and is expected behavior. All build types produce identical, deterministic results.

## Output Format

### Volume Data Structure

The returned `CVC::volume` object provides:

```cpp
CVC::volume sdf_vol = CVC::sdf(geom, dim, bbox);

// Dimensions
uint64 nx = sdf_vol.XDim();  // Voxels in X
uint64 ny = sdf_vol.YDim();  // Voxels in Y
uint64 nz = sdf_vol.ZDim();  // Voxels in Z

// Bounding box
CVC::bounding_box bb = sdf_vol.bounding_box();
double minX = bb.minx, maxX = bb.maxx;
// ... similar for Y and Z

// Voxel spacing
double dx = (bb.maxx - bb.minx) / (nx - 1);
double dy = (bb.maxy - bb.miny) / (ny - 1);
double dz = (bb.maxz - bb.minz) / (nz - 1);

// Access by index
double val = sdf_vol(i, j, k);

// Interpolated access
double x = 0.5, y = 0.5, z = 0.5;
double interp_val = sdf_vol(x, y, z);  // Trilinear interpolation
```

### Distance Values

- **Units**: Same as input geometry (typically meters, millimeters, etc.)
- **Sign convention**: 
  - Negative inside (< 0)
  - Zero on surface (= 0)  
  - Positive outside (> 0)
- **Accuracy**: Exact to nearest triangle within voxel resolution

## Common Patterns

### Extract Isosurface at Distance

```cpp
#include <cvc/algorithm.h>

// Get SDF
CVC::volume sdf_vol = CVC::sdf(geom, dim, bbox);

// Extract isosurface at distance = 0.1 (slight offset from surface)
CVC::geometry offset_surface = CVC::isosurface(sdf_vol, 0.1);

// Extract actual surface (distance = 0)
CVC::geometry reconstructed = CVC::isosurface(sdf_vol, 0.0);
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
CVC::bounding_box bb = sdf_vol.bounding_box();
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
CVC::bounding_box bb = sdf_vol.bounding_box();
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

## Error Handling

```cpp
try {
    CVC::volume sdf_vol = CVC::sdf(geom, dim, bbox);
} catch (const CVC::Exception& e) {
    std::cerr << "SDF computation failed: " << e.what() << std::endl;
    // Handle error
}
```

### Common Issues

1. **Out of memory**: Reduce grid resolution
2. **Slow computation**: Use smaller resolution or fewer triangles
3. **Incorrect signs**: Check geometry orientation or use `flipNormals = 1`
4. **Invalid geometry**: Ensure manifold, no self-intersections, closed surface

## Implementation Details

### Octree Structure

The SDF implementation uses an adaptive octree for spatial acceleration:
- **Leaf cells** contain pointers to intersecting triangles
- **Interior cells** recursively subdivide space
- **Depth** is automatically determined: typically log₈(gridSize)

### Distance Computation Algorithm

1. **Octree construction**: O(m·d) where d = octree depth
2. **Boundary distance**: Exact ray-triangle intersection for boundary voxels
3. **Sign determination**: Ray casting to determine inside/outside
4. **Propagation**: Fast marching method to fill interior

### Memory Layout

```cpp
// Grid voxels stored as (size+1)³ array
// Index calculation: i + j*(size+1) + k*(size+1)²

// Octree cells stored as size³ boost::multi_array
// Each cell contains:
struct cell {
    char useful;     // Has triangles?
    char type;       // Interior/leaf
    long no;         // Triangle count
    listnode* tindex; // Linked list of triangle indices
};
```

## Migration Guide (v1.x → v2.0)

### Old Global-Based API
```cpp
// Old code (deprecated)
SDFLibrary::size = 128;
SDFLibrary::flipNormals = 0;
// ... set other globals ...
signeddistancefunction(nverts, verts, ntris, tris, gridData);
```

### New Thread-Safe API
```cpp
// New code
SDFContext ctx;
ctx.setParameters(128, 0, mins, maxs);
ctx.initSDF();
ctx.readGeom(nverts, verts, ntris, tris);
ctx.adjustData();
ctx.compute();
// Access ctx.voxel_values
```

## See Also

- [Algorithm API](ALGORITHM_API.md) - High-level algorithm interface
- [Volume API](VOLUME_API.md) - Volume data structure
- [Geometry API](GEOMETRY_API.md) - Mesh representation
- [Testing Coverage](../TESTING_COVERAGE.md) - Test suite and coverage reports

## References

1. **Signed Distance Fields**: Jones et al., "3D Distance Fields: A Survey" (2006)
2. **Fast Marching**: Sethian, "Level Set Methods and Fast Marching Methods" (1999)
3. **Octree Acceleration**: Samet, "Foundations of Multidimensional and Metric Data Structures" (2006)

## Version History

- **v2.0**: Thread-safe SDFContext architecture with boost::multi_array
- **v1.x**: Global variable-based implementation (deprecated)
