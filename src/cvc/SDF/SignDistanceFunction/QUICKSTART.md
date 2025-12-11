# SDF Library Quick Start Guide

## For New Users

### Basic Usage (Thread-Safe API)

```cpp
#include "sdfLib.h"

// Your mesh data
float vertices[] = {
    -1.0f, -1.0f, -1.0f,  // vertex 0
     1.0f, -1.0f, -1.0f,  // vertex 1
     0.0f,  1.0f, -1.0f   // vertex 2
};

int triangles[] = {
    0, 1, 2  // triangle using vertices 0, 1, 2
};

// Bounding box for the SDF grid
float mins[] = {-2.0f, -2.0f, -2.0f};
float maxs[] = { 2.0f,  2.0f,  2.0f};

// Compute SDF on a 64x64x64 grid
int grid_size = 64;
int flip_normals = 0;  // 0 = don't flip, 1 = flip

auto sdf_grid = SDFLibrary::computeSDF_MT(
    3, vertices,      // 3 vertices
    1, triangles,     // 1 triangle
    grid_size,
    flip_normals,
    mins, maxs
);

// Access the distance at grid point (i, j, k):
float distance = sdf_grid[i * grid_size * grid_size + j * grid_size + k];

// Positive = outside the mesh
// Negative = inside the mesh
// Zero = on the surface
```

### Multi-Threaded Usage

```cpp
#include <thread>
#include <vector>

std::vector<Mesh> meshes = load_meshes();
std::vector<std::unique_ptr<float[]>> sdf_results(meshes.size());
std::vector<std::thread> threads;

// Process each mesh in parallel
for (size_t i = 0; i < meshes.size(); i++) {
    threads.emplace_back([&, i]() {
        sdf_results[i] = SDFLibrary::computeSDF_MT(
            meshes[i].num_verts, meshes[i].verts,
            meshes[i].num_tris, meshes[i].tris,
            64, 0, meshes[i].mins, meshes[i].maxs
        );
    });
}

// Wait for all threads
for (auto& t : threads) {
    t.join();
}

// Now sdf_results[i] contains the distance field for mesh i
```

## For Existing Code Migration

### Old API (Still Works)

```cpp
// Legacy code - still functional but not thread-safe
float* grid = SDFLibrary::computeSDF(
    num_verts, verts,
    num_tris, tris,
    grid_size, flip_normals,
    mins, maxs
);

// Use the grid...

// Must manually free!
delete[] grid;
```

### New API (Recommended)

```cpp
// Modern code - thread-safe with automatic cleanup
auto grid = SDFLibrary::computeSDF_MT(
    num_verts, verts,
    num_tris, tris,
    grid_size, flip_normals,
    mins, maxs
);

// Use the grid...
// No need to free - unique_ptr handles it automatically
```

## Advanced Usage

### Manual Context Management

For fine-grained control over the SDF computation pipeline:

```cpp
#include "SDFContext.h"

// Create a context
auto ctx = SDFLibrary::createContext();

// Set parameters
float mins[] = {-2, -2, -2};
float maxs[] = {2, 2, 2};
ctx->setParameters(64, 0, mins, maxs);

// Initialize
if (!ctx->initSDF()) {
    std::cerr << "Failed to initialize SDF\n";
    return;
}

// Load geometry
ctx->readGeom(num_verts, verts, num_tris, tris);

// Adjust bounding box and build octree
ctx->adjustData();

// Compute distance field
ctx->compute();

// Access results
const voxel* voxels = ctx->getVoxelValues();
for (int i = 0; i < 64 * 64 * 64; i++) {
    float distance = voxels[i].value;
    int closest_tri = voxels[i].closestV;
    // ... use data ...
}
```

## Common Pitfalls

### 1. Index Ordering
The grid is stored in **row-major order**: `[i][j][k]` → `i * size * size + j * size + k`

```cpp
// Correct:
float dist = sdf_grid[i * size * size + j * size + k];

// Wrong (column-major - don't do this):
float dist = sdf_grid[k * size * size + j * size + i];
```

### 2. Coordinate Systems
- Grid indices `(i, j, k)` are in the range `[0, size-1]`
- World coordinates are mapped linearly between `mins` and `maxs`
- To convert grid → world: `x = mins[0] + i * (maxs[0] - mins[0]) / (size - 1)`

### 3. Normal Orientation
- `flip_normals = 0`: Standard orientation (outward normals)
- `flip_normals = 1`: Flipped orientation (inward normals)
- If your mesh has inverted normals, use `flip_normals = 1`

### 4. Memory Management
```cpp
// Don't do this (memory leak with old API):
float* grid = SDFLibrary::computeSDF(...);
// ... forgot to delete[] grid ...

// Do this instead (automatic cleanup):
auto grid = SDFLibrary::computeSDF_MT(...);
// Automatic cleanup when grid goes out of scope
```

## Performance Tips

### Grid Size Selection
- Small meshes (< 1K triangles): 64³ grid is sufficient
- Medium meshes (1K-10K triangles): 128³ grid recommended
- Large meshes (> 10K triangles): 256³ or adaptive resolution

### Bounding Box
Tight bounding boxes improve accuracy:
```cpp
// Compute tight bounding box
float mins[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
float maxs[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

for (int i = 0; i < num_verts * 3; i += 3) {
    for (int j = 0; j < 3; j++) {
        mins[j] = std::min(mins[j], verts[i + j]);
        maxs[j] = std::max(maxs[j], verts[i + j]);
    }
}

// Add small padding (10%)
for (int j = 0; j < 3; j++) {
    float range = maxs[j] - mins[j];
    mins[j] -= range * 0.1f;
    maxs[j] += range * 0.1f;
}
```

### Parallel Processing
For multiple meshes, use thread pool to avoid oversubscription:
```cpp
#include <thread>

unsigned int num_threads = std::thread::hardware_concurrency();
// Process 'num_threads' meshes at a time
```

## Interpreting Results

### Distance Values
```cpp
float dist = sdf_grid[idx];

if (dist > 0) {
    // Point is OUTSIDE the mesh
    // dist = distance to nearest surface
}
else if (dist < 0) {
    // Point is INSIDE the mesh
    // |dist| = distance to nearest surface
}
else {
    // Point is ON the surface
}
```

### Closest Triangle
```cpp
const voxel* voxels = ctx->getVoxelValues();
int closest = voxels[idx].closestV;

// closest is the index of the nearest triangle
// Access it: triangles[closest * 3 + 0/1/2]
```

## Testing

Run the SDF tests to verify your installation:
```bash
cd build
ctest -R AlgorithmTest.SDF -V
```

Expected output:
```
Test #348: AlgorithmTest.SDFBasic ................   Passed    0.51 sec
Test #351: AlgorithmTest.SDFThenIsoRoundtrip .....   Passed    0.12 sec
100% tests passed
```

## Troubleshooting

### Build Errors
```bash
# Make sure SDFContext is included in CMakeLists.txt
grep SDFContext src/cvc/CMakeLists.txt

# Should see:
#   SDFContext.cpp
#   SDFContext.h
```

### Runtime Errors
```bash
# Run with sanitizers
cmake -DCMAKE_BUILD_TYPE=Debug -DSANITIZE=address ..
make
./your_program

# Check for memory leaks
valgrind --leak-check=full ./your_program
```

### Performance Issues
```bash
# Make sure you're using Release build
cmake -DCMAKE_BUILD_TYPE=Release ..

# Profile with perf
perf record -g ./your_program
perf report
```

## Examples

### Complete Example Program

```cpp
#include <iostream>
#include <vector>
#include <cmath>
#include "sdfLib.h"

int main() {
    // Create a simple triangle mesh (tetrahedron)
    std::vector<float> verts = {
         0.0f,  1.0f,  0.0f,  // top
        -1.0f, -1.0f, -1.0f,  // bottom corners
         1.0f, -1.0f, -1.0f,
         0.0f, -1.0f,  1.0f
    };
    
    std::vector<int> tris = {
        0, 1, 2,  // top-front face
        0, 2, 3,  // top-right face
        0, 3, 1,  // top-left face
        1, 3, 2   // bottom face
    };
    
    // Compute SDF
    float mins[] = {-2, -2, -2};
    float maxs[] = { 2,  2,  2};
    int grid_size = 64;
    
    auto sdf = SDFLibrary::computeSDF_MT(
        4, verts.data(),
        4, tris.data(),
        grid_size, 0,
        mins, maxs
    );
    
    // Find the maximum distance
    float max_dist = 0;
    for (int i = 0; i < grid_size * grid_size * grid_size; i++) {
        max_dist = std::max(max_dist, std::abs(sdf[i]));
    }
    
    std::cout << "Maximum distance: " << max_dist << "\n";
    
    // Sample at center
    int center = (grid_size/2) * grid_size * grid_size + 
                 (grid_size/2) * grid_size + 
                 (grid_size/2);
    std::cout << "Distance at center: " << sdf[center] << "\n";
    
    return 0;
}
```

Compile:
```bash
g++ -std=c++14 -O2 example.cpp -I/path/to/inc -L/path/to/lib -lcvc -o example
./example
```

## See Also

- `SDF_REFACTORING.md` - Complete technical documentation
- `REFACTORING_SUMMARY.md` - Summary of changes and test results
- Original paper: Bajaj et al., "Interactive Visual Exploration of Large Flexible Multi-component Molecular Complexes"

## Support

For issues or questions:
1. Check the full documentation in `SDF_REFACTORING.md`
2. Run the test suite: `ctest -R AlgorithmTest.SDF`
3. Review this quick start guide
4. Check the example code above

Happy computing!
