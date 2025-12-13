# Geometry API Reference

*Complete reference for the trans-cvc triangle mesh and geometry library*

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Core Concepts](#core-concepts)
  - [Data Types](#data-types)
  - [Memory Semantics](#memory-semantics)
  - [Array Types](#array-types)
  - [Boundary Vertices](#boundary-vertices)
- [Construction](#construction)
- [Data Access](#data-access)
  - [Points](#points)
  - [Triangles](#triangles)
  - [Quads](#quads)
  - [Lines](#lines)
  - [Normals](#normals)
  - [Colors](#colors)
  - [Boundary](#boundary)
- [Geometry Queries](#geometry-queries)
  - [Extents and Bounding Box](#extents-and-bounding-box)
  - [Counting Elements](#counting-elements)
  - [Testing Emptiness](#testing-emptiness)
- [Geometric Operations](#geometric-operations)
  - [Merging Geometries](#merging-geometries)
  - [Surface Extraction](#surface-extraction)
  - [Normal Computation](#normal-computation)
  - [Normal Inversion](#normal-inversion)
  - [Reorientation](#reorientation)
  - [Wireframe Generation](#wireframe-generation)
  - [Clearing Data](#clearing-data)
- [Mesh Processing](#mesh-processing)
  - [Projection](#projection)
  - [Smoothing](#smoothing)
  - [Quality Improvement](#quality-improvement)
- [File I/O](#file-io)
  - [Reading Geometries](#reading-geometries)
  - [Writing Geometries](#writing-geometries)
  - [Supported Formats](#supported-formats)
  - [File I/O Extension System](#file-io-extension-system)
- [Type Definitions](#type-definitions)
- [Complete Examples](#complete-examples)
- [Best Practices](#best-practices)
- [Performance Considerations](#performance-considerations)
- [Exception Handling](#exception-handling)

## Overview

The `cvc::geometry` class provides a versatile container for 3D triangle meshes, volumetric meshes (tetrahedra, hexahedra), and general geometric data. It serves as the standard geometry representation throughout trans-cvc for isosurfacing, mesh processing, rendering, and scientific visualization.

**Key Features:**
- **Triangle meshes**: Surface meshes with optional normals and colors
- **Quad meshes**: Quadrilateral elements for structured grids
- **Line meshes**: Wireframe and curve representations
- **Volumetric meshes**: Tetrahedral and hexahedral elements with boundary marking
- **Copy-on-write**: Efficient shallow copying with automatic data duplication on modification
- **Merge operations**: Combine multiple geometries with automatic index remapping
- **Surface extraction**: Extract boundary triangles from volumetric meshes
- **Normal computation**: Automatic surface normal calculation
- **File I/O**: OFF, RAW, RAWN, RAWC, RAWNC, OBJ (via SDF), and extensible I/O system
- **Mesh processing**: Smoothing, quality improvement, projection

**Use Cases:**
- Loading and saving triangle meshes
- Isosurface extraction from volumes
- Mesh generation and processing
- Scientific visualization
- 3D rendering preparation
- Geometry analysis and manipulation

**Thread Safety:** Not thread-safe by default. Use external synchronization for concurrent access.

## Quick Start

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>

using namespace CVC_NAMESPACE;

// Load mesh from file
geometry mesh = read_geometry("bunny.off");

std::cout << "Vertices: " << mesh.num_points() << "\n";
std::cout << "Triangles: " << mesh.num_tris() << "\n";

// Calculate surface normals
mesh.calculate_surf_normals();

// Access data
const auto& points = mesh.points();
const auto& tris = mesh.tris();
const auto& normals = mesh.normals();

// Process mesh
for (uint64_t i = 0; i < mesh.num_tris(); ++i) {
    const tri_t& tri = tris[i];
    const point_t& p0 = points[tri[0]];
    const point_t& p1 = points[tri[1]];
    const point_t& p2 = points[tri[2]];
    
    // Process triangle...
}

// Merge with another mesh
geometry mesh2 = read_geometry("dragon.off");
mesh.merge(mesh2);  // Indices automatically remapped

// Extract surface from volumetric mesh
geometry surface = mesh.tri_surface();

// Smooth mesh
mesh.smoothing(0.1f, false);  // delta=0.1, don't fix boundary

// Save result
mesh.write("output.raw");
```

## Core Concepts

### Data Types

The geometry class uses template-based types for flexibility:

```cpp
// Scalar type (coordinates, distances, etc.)
typedef double scalar_t;

// Index type (vertex/triangle indices)
typedef uint64_t index_t;

// 3D point (vertex coordinates)
typedef boost::array<scalar_t, 3> point_t;

// 3D vector (directions, displacements)
typedef boost::array<scalar_t, 3> vector_t;

// Normal vector (same as vector_t)
typedef vector_t normal_t;

// RGB color
typedef boost::array<scalar_t, 3> color_t;

// Connectivity types
typedef boost::array<index_t, 2> line_t;   // Line segment (2 vertices)
typedef boost::array<index_t, 3> tri_t;    // Triangle (3 vertices)
typedef boost::array<index_t, 4> quad_t;   // Quad (4 vertices)
```

**Array Containers:**

```cpp
// STL vectors for dynamic arrays
typedef std::vector<point_t>  points_t;
typedef std::vector<vector_t> normals_t;
typedef std::vector<color_t>  colors_t;
typedef std::vector<line_t>   lines_t;
typedef std::vector<tri_t>    tris_t;
typedef std::vector<quad_t>   quads_t;

// Boost dynamic bitset for boundary marking
typedef boost::dynamic_bitset<> boundary_t;
```

**Smart Pointers:**

```cpp
typedef boost::shared_ptr<points_t>   points_ptr_t;
typedef boost::shared_ptr<boundary_t> boundary_ptr_t;
typedef boost::shared_ptr<normals_t>  normals_ptr_t;
typedef boost::shared_ptr<colors_t>   colors_ptr_t;
typedef boost::shared_ptr<lines_t>    lines_ptr_t;
typedef boost::shared_ptr<tris_ptr_t> tris_ptr_t;
typedef boost::shared_ptr<quads_t>    quads_ptr_t;
```

### Memory Semantics

**Copy-on-Write (COW):**

The geometry class uses `boost::shared_ptr` to implement shallow copying with copy-on-write semantics:

```cpp
// Shallow copy - shares underlying data
geometry mesh1 = read_geometry("bunny.off");
geometry mesh2(mesh1);        // Copy constructor - shares data
geometry mesh3 = mesh1;       // Assignment - shares data
geometry mesh4;
mesh4.copy(mesh1);            // copy() method - shares data (default)

// All four geometries share the same underlying arrays
// (points, triangles, normals, etc.)

// First modification triggers copy-on-write
mesh2.points().push_back({{0, 0, 0}});  // Triggers COW
// Now mesh2 has independent data, others still shared
```

**Deep Copy:**

For true independent copies that don't use COW, use the `copy()` method with `deepCopy = true`:

```cpp
geometry mesh1 = read_geometry("bunny.off");

// Deep copy - immediately independent data
geometry mesh2;
mesh2.copy(mesh1, true);  // deepCopy = true

// Modifications to mesh2 do NOT affect mesh1
mesh2.points()[0][0] = 999.0;  // No COW needed, already independent
// mesh1.points()[0][0] is unchanged

// Shallow copy (default) for comparison
geometry mesh3;
mesh3.copy(mesh1);  // or mesh3.copy(mesh1, false);
// mesh3 shares data with mesh1 until modified
```

**When to Use Deep Copy:**

- When you need guaranteed independence from source geometry
- When building geometry that will be heavily modified
- When you want to avoid COW overhead for many modifications
- When working with multithreaded code (avoid shared_ptr reference counting)

**When to Use Shallow Copy (Default):**

- When creating temporary copies for read-only operations
- When you want to save memory by sharing unchanged data
- When modifications are infrequent
- For passing geometry around without duplication overhead

**When COW Triggers:**

Copy-on-write is triggered by the `pre_write()` method, which is called when you access non-const data:

```cpp
geometry mesh(bunny);

// These trigger COW if data is shared:
mesh.points().push_back({{0, 0, 0}});
mesh.tris().clear();
mesh.normals()[0] = {{1, 0, 0}};
mesh.boundary().set(10, true);

// These DO NOT trigger COW (const access):
uint64_t num = mesh.num_points();
bounding_box bbox = mesh.extents();
const auto& pts = mesh.const_points();  // Explicit const access
const auto& pts2 = mesh.points();       // Implicit const (const context)
```

**Independent Copies:**

Deep copies are now easily created using the `copy()` method:

```cpp
geometry mesh1 = read_geometry("bunny.off");

// Method 1: Deep copy (recommended)
geometry mesh2;
mesh2.copy(mesh1, true);  // Immediately independent

// Method 2: Shallow copy with forced COW
geometry mesh3(mesh1);
mesh3.points();  // Triggers COW, now independent

// Method 3: Manual deep copy
geometry mesh4;
mesh4.points() = mesh1.points();
mesh4.tris() = mesh1.tris();
mesh4.normals() = mesh1.normals();
// ... copy other arrays as needed
```

### Array Types

The geometry class organizes data into seven array types:

```cpp
enum ARRAY_TYPE {
    POINTS,    // Vertex coordinates
    BOUNDARY,  // Boundary vertex flags
    NORMALS,   // Vertex normals
    COLORS,    // Vertex colors
    LINES,     // Line connectivity
    TRIS,      // Triangle connectivity
    QUADS      // Quad connectivity
};
```

Each array type is managed independently with COW semantics.

### Boundary Vertices

The boundary bitset marks which vertices are on the surface of a volumetric mesh:

```cpp
geometry mesh;
// ... load tetrahedral mesh ...

// Mark boundary vertices
mesh.boundary().set(10, true);   // Vertex 10 is on boundary
mesh.boundary().set(20, false);  // Vertex 20 is interior

// Check boundary status
if (mesh.boundary().test(10)) {
    // Vertex 10 is on boundary
}

// Count boundary vertices
uint64_t boundary_count = mesh.boundary().count();
```

**Use in Surface Extraction:**

The `tri_surface()` method uses the boundary bitset to extract only boundary triangles from volumetric meshes.

## Construction

### Default Constructor

```cpp
geometry();
```

Creates an empty geometry with no vertices or elements.

```cpp
geometry mesh;
ASSERT_TRUE(mesh.empty());
ASSERT_EQ(mesh.num_points(), 0);
```

### Copy Constructor

```cpp
geometry(const geometry& geom);
```

Creates a shallow copy sharing data with the source geometry (COW semantics).

```cpp
geometry bunny = read_geometry("bunny.off");
geometry copy(bunny);  // Shares data with bunny

EXPECT_EQ(copy.num_points(), bunny.num_points());
```

### Copy Method

```cpp
void copy(const geometry& geom, bool deepCopy = false);
```

Copies data from another geometry. By default, performs a shallow copy with COW semantics. Set `deepCopy = true` for an immediate independent copy.

**Shallow Copy (default):**

```cpp
geometry mesh1 = read_geometry("bunny.off");
geometry mesh2;
mesh2.copy(mesh1);  // Shallow copy - shares data

// Modification triggers COW
mesh2.points()[0][0] = 1.0;  // Now mesh2 has independent data
```

**Deep Copy:**

```cpp
geometry mesh1 = read_geometry("bunny.off");
geometry mesh3;
mesh3.copy(mesh1, true);  // Deep copy - immediately independent

// No COW needed - already has independent data
mesh3.points()[0][0] = 1.0;  // mesh1 is unchanged
```

**Use Cases:**

- `deepCopy = false` (default): Memory-efficient sharing for read-mostly operations
- `deepCopy = true`: Guaranteed independence for heavy modification or multithreading

### Assignment Operator

```cpp
geometry& operator=(const geometry& geom);
```

Performs a shallow copy (equivalent to `copy(geom, false)`).

```cpp
geometry mesh1 = read_geometry("bunny.off");
geometry mesh2;
mesh2 = mesh1;  // Shallow copy - shares data
```

### File Constructor

```cpp
geometry(const std::string& filename);
```

Loads geometry from a file. File format determined by extension.

```cpp
geometry mesh("bunny.off");   // Load OFF file
geometry mesh2("model.raw");  // Load RAW file
```

**Supported Extensions:**
- `.off` - Object File Format
- `.raw`, `.rawn`, `.rawc`, `.rawnc` - CVC raw formats
- `.obj` - Wavefront OBJ (via SDF, experimental)
- `.bunny` - Embedded Stanford Bunny (for testing)

### Destructor

```cpp
~geometry();
```

Automatically cleans up resources. Shared data is deleted when the last reference is destroyed.

## Data Access

### Points

**Non-const Access (triggers COW):**

```cpp
points_t& points();
```

Returns mutable reference to vertex array.

```cpp
geometry mesh;
mesh.points().push_back({{0.0, 0.0, 0.0}});
mesh.points().push_back({{1.0, 0.0, 0.0}});
mesh.points().push_back({{0.0, 1.0, 0.0}});

// Modify existing point
mesh.points()[0] = {{0.5, 0.5, 0.5}};
```

**Const Access (no COW):**

```cpp
const points_t& points() const;
const points_t& const_points() const;
```

Returns const reference to vertex array.

```cpp
const geometry& mesh = get_mesh();
for (const auto& pt : mesh.points()) {
    std::cout << "Point: " << pt[0] << ", " << pt[1] << ", " << pt[2] << "\n";
}
```

**Individual Point Access:**

```cpp
point_t& pt = mesh.points()[index];
pt[0] = 1.0;  // x coordinate
pt[1] = 2.0;  // y coordinate
pt[2] = 3.0;  // z coordinate
```

### Triangles

**Non-const Access:**

```cpp
tris_t& tris();
```

Returns mutable reference to triangle array.

```cpp
geometry mesh;
// Assuming points 0, 1, 2 exist
mesh.tris().push_back({{0, 1, 2}});  // Add triangle

// Modify existing triangle
mesh.tris()[0] = {{2, 1, 0}};  // Reverse winding
```

**Const Access:**

```cpp
const tris_t& tris() const;
const tris_t& const_tris() const;
```

Returns const reference to triangle array.

```cpp
for (const auto& tri : mesh.tris()) {
    index_t i0 = tri[0];
    index_t i1 = tri[1];
    index_t i2 = tri[2];
    // Process triangle vertices
}
```

### Quads

**Non-const Access:**

```cpp
quads_t& quads();
```

Returns mutable reference to quad array.

```cpp
mesh.quads().push_back({{0, 1, 2, 3}});  // Add quad
```

**Const Access:**

```cpp
const quads_t& quads() const;
const quads_t& const_quads() const;
```

### Lines

**Non-const Access:**

```cpp
lines_t& lines();
```

Returns mutable reference to line array.

```cpp
mesh.lines().push_back({{0, 1}});  // Add line from vertex 0 to 1
mesh.lines().push_back({{1, 2}});  // Add line from vertex 1 to 2
```

**Const Access:**

```cpp
const lines_t& lines() const;
const lines_t& const_lines() const;
```

### Normals

**Non-const Access:**

```cpp
normals_t& normals();
```

Returns mutable reference to normal array.

```cpp
// Manually set normal for vertex 0
mesh.normals().resize(mesh.num_points());
mesh.normals()[0] = {{0.0, 0.0, 1.0}};  // Normal pointing up

// Or use automatic computation
mesh.calculate_surf_normals();
```

**Const Access:**

```cpp
const normals_t& normals() const;
const normals_t& const_normals() const;
```

### Colors

**Non-const Access:**

```cpp
colors_t& colors();
```

Returns mutable reference to color array.

```cpp
mesh.colors().resize(mesh.num_points());
mesh.colors()[0] = {{1.0, 0.0, 0.0}};  // Red
mesh.colors()[1] = {{0.0, 1.0, 0.0}};  // Green
mesh.colors()[2] = {{0.0, 0.0, 1.0}};  // Blue
```

**Const Access:**

```cpp
const colors_t& colors() const;
const colors_t& const_colors() const;
```

### Boundary

**Non-const Access:**

```cpp
boundary_t& boundary();
```

Returns mutable reference to boundary bitset.

```cpp
mesh.boundary().resize(mesh.num_points());
mesh.boundary().set(10, true);   // Mark vertex 10 as boundary
mesh.boundary().reset(20);       // Mark vertex 20 as interior
```

**Const Access:**

```cpp
const boundary_t& boundary() const;
const boundary_t& const_boundary() const;
```

**Boundary Operations:**

```cpp
// Set boundary for specific vertex
mesh.boundary().set(index, true);

// Clear boundary flag
mesh.boundary().set(index, false);
mesh.boundary().reset(index);

// Test boundary status
if (mesh.boundary().test(index)) {
    // Vertex is on boundary
}

// Count boundary vertices
uint64_t boundary_count = mesh.boundary().count();

// Resize bitset
mesh.boundary().resize(mesh.num_points());
```

## Geometry Queries

### Extents and Bounding Box

**Minimum Point:**

```cpp
point_t min_point() const;
```

Returns the minimum corner of the axis-aligned bounding box.

```cpp
point_t min = mesh.min_point();
std::cout << "Min: " << min[0] << ", " << min[1] << ", " << min[2] << "\n";
```

**Maximum Point:**

```cpp
point_t max_point() const;
```

Returns the maximum corner of the axis-aligned bounding box.

```cpp
point_t max = mesh.max_point();
std::cout << "Max: " << max[0] << ", " << max[1] << ", " << max[2] << "\n";
```

**Bounding Box:**

```cpp
bounding_box extents() const;
```

Returns the axis-aligned bounding box containing all vertices.

```cpp
bounding_box bbox = mesh.extents();
std::cout << "Bounds: [" << bbox.minx << ", " << bbox.maxx << "] × "
          << "[" << bbox.miny << ", " << bbox.maxy << "] × "
          << "[" << bbox.minz << ", " << bbox.maxz << "]\n";

// Calculate size
double width = bbox.maxx - bbox.minx;
double height = bbox.maxy - bbox.miny;
double depth = bbox.maxz - bbox.minz;
```

**Lazy Evaluation:**

Extents are computed on first access and cached. The cache is invalidated when points are modified.

### Counting Elements

```cpp
uint64_t num_points() const;
uint64_t num_lines() const;
uint64_t num_tris() const;
uint64_t num_quads() const;
```

Returns the number of elements in each array.

```cpp
geometry bunny = read_geometry("bunny.off");
std::cout << "Vertices: " << bunny.num_points() << "\n";   // 34,835
std::cout << "Triangles: " << bunny.num_tris() << "\n";    // 69,473
std::cout << "Lines: " << bunny.num_lines() << "\n";       // 0
std::cout << "Quads: " << bunny.num_quads() << "\n";       // 0
```

### Testing Emptiness

```cpp
bool empty() const;
```

Returns `true` if the geometry has no vertices.

```cpp
geometry mesh;
ASSERT_TRUE(mesh.empty());

mesh.points().push_back({{0, 0, 0}});
ASSERT_FALSE(mesh.empty());

mesh.clear();
ASSERT_TRUE(mesh.empty());
```

## Geometric Operations

### Merging Geometries

```cpp
geometry& merge(const geometry& geom);
```

Merges another geometry into this one. Automatically remaps indices to avoid conflicts.

```cpp
geometry mesh1 = read_geometry("bunny.off");
geometry mesh2 = read_geometry("dragon.off");

std::cout << "Before merge:\n";
std::cout << "  mesh1: " << mesh1.num_points() << " points, "
          << mesh1.num_tris() << " tris\n";
std::cout << "  mesh2: " << mesh2.num_points() << " points, "
          << mesh2.num_tris() << " tris\n";

mesh1.merge(mesh2);

std::cout << "After merge:\n";
std::cout << "  mesh1: " << mesh1.num_points() << " points, "
          << mesh1.num_tris() << " tris\n";
// mesh1 now contains both bunny and dragon
```

**Index Remapping:**

When merging, all indices in the source geometry are automatically adjusted:

```cpp
// Before merge:
//   mesh1 has points [0, ..., N1-1]
//   mesh2 has points [0, ..., N2-1] with triangles referencing them

// After merge:
//   mesh1 has points [0, ..., N1-1, N1, ..., N1+N2-1]
//   mesh2's triangles now reference points [N1, ..., N1+N2-1]
```

**Arrays Merged:**
- Points
- Triangles
- Quads
- Lines
- Normals (if both have them)
- Colors (if both have them)
- Boundary flags (if both have them)

**Example: Combining Multiple Meshes**

```cpp
std::vector<std::string> files = {"part1.off", "part2.off", "part3.off"};
geometry combined;

for (const auto& file : files) {
    geometry part = read_geometry(file);
    if (combined.empty()) {
        combined = part;
    } else {
        combined.merge(part);
    }
}

combined.write("combined.raw");
```

### Surface Extraction

```cpp
geometry tri_surface() const;
```

Extracts boundary surface from a volumetric mesh. Returns a new geometry containing only boundary triangles.

**For Triangle Meshes:**

If the mesh already consists only of triangles, returns a copy.

**For Volumetric Meshes:**

Extracts triangular faces where at least one vertex is marked as boundary. Quads are split into two triangles.

```cpp
// Load tetrahedral mesh
geometry tet_mesh = read_geometry("tets.raw");

// Mark boundary vertices (typically done during mesh generation)
for (uint64_t i = 0; i < tet_mesh.num_points(); ++i) {
    if (is_on_boundary(i)) {
        tet_mesh.boundary().set(i, true);
    }
}

// Extract surface triangles
geometry surface = tet_mesh.tri_surface();

std::cout << "Original: " << tet_mesh.num_points() << " vertices\n";
std::cout << "Surface: " << surface.num_tris() << " triangles\n";

surface.write("surface.off");
```

**Quad to Triangle Conversion:**

Quads are split along the diagonal:

```
Quad (v0, v1, v2, v3) -> Triangle (v0, v1, v2) + Triangle (v0, v2, v3)
```

### Normal Computation

```cpp
geometry& calculate_surf_normals();
```

Computes vertex normals based on adjacent triangle areas and angles. Modifies the geometry in-place.

**Algorithm:**

For each boundary vertex:
1. Find all adjacent triangles
2. Compute triangle normals weighted by area
3. Average normals at shared vertices
4. Normalize result

For interior vertices (non-boundary), normals are set to (0, 0, 0).

```cpp
geometry mesh = read_geometry("bunny.off");

// Calculate normals
mesh.calculate_surf_normals();

// Verify normals are unit length
for (const auto& normal : mesh.normals()) {
    double length = std::sqrt(normal[0]*normal[0] + 
                              normal[1]*normal[1] + 
                              normal[2]*normal[2]);
    EXPECT_NEAR(length, 1.0, 1e-6);  // Unit length
}

// Use normals for rendering
for (uint64_t i = 0; i < mesh.num_tris(); ++i) {
    const tri_t& tri = mesh.tris()[i];
    const normal_t& n0 = mesh.normals()[tri[0]];
    const normal_t& n1 = mesh.normals()[tri[1]];
    const normal_t& n2 = mesh.normals()[tri[2]];
    // Render with smooth shading
}
```

**Boundary Consideration:**

If boundary array is set, only boundary vertices get computed normals. Interior vertices get zero normals.

### Normal Inversion

```cpp
geometry& invert_normals();
```

Inverts all vertex normals. Useful for reversing mesh orientation.

```cpp
geometry mesh = read_geometry("model.off");
mesh.calculate_surf_normals();

// Check original orientation
normal_t original = mesh.normals()[0];

// Invert
mesh.invert_normals();

// Verify inversion
normal_t inverted = mesh.normals()[0];
EXPECT_DOUBLE_EQ(inverted[0], -original[0]);
EXPECT_DOUBLE_EQ(inverted[1], -original[1]);
EXPECT_DOUBLE_EQ(inverted[2], -original[2]);
```

**Preserves Length:**

Normal magnitude is preserved (typically unit length).

### Reorientation

```cpp
geometry& reorient();
```

Makes triangle orientations consistent across the mesh. Ensures all triangles follow a consistent winding order.

**Note:** Current implementation makes normals consistent but doesn't actually reorient triangle vertices. Future versions will properly flip triangle winding.

```cpp
geometry mesh = read_geometry("inconsistent.off");
mesh.reorient();  // Make orientations consistent
mesh.calculate_surf_normals();  // Compute consistent normals
```

### Wireframe Generation

```cpp
geometry generate_wire_interior() const;
```

Generates a wireframe representation of the interior structure of a volumetric mesh. Returns a new geometry with lines array filled.

**Use Case:**

Visualizing the internal structure of tetrahedral or hexahedral meshes.

```cpp
geometry tet_mesh = read_geometry("volume.raw");

// Generate wireframe of interior edges
geometry wireframe = tet_mesh.generate_wire_interior();

std::cout << "Wireframe has " << wireframe.num_lines() << " edges\n";

// Use for rendering
for (const auto& line : wireframe.lines()) {
    const point_t& p0 = wireframe.points()[line[0]];
    const point_t& p1 = wireframe.points()[line[1]];
    draw_line(p0, p1);
}
```

### Clearing Data

```cpp
geometry& clear();
```

Removes all data from the geometry, resetting it to an empty state.

```cpp
geometry mesh = read_geometry("bunny.off");
ASSERT_FALSE(mesh.empty());

mesh.clear();

ASSERT_TRUE(mesh.empty());
ASSERT_EQ(mesh.num_points(), 0);
ASSERT_EQ(mesh.num_tris(), 0);
ASSERT_EQ(mesh.num_lines(), 0);
ASSERT_EQ(mesh.num_quads(), 0);
```

**Arrays Cleared:**
- Points
- Triangles
- Quads
- Lines
- Normals
- Colors
- Boundary flags

## Mesh Processing

### Projection

```cpp
geometry& project(const geometry& target);
```

Projects boundary vertices of this geometry onto the target geometry surface. Useful for fitting meshes or enforcing surface constraints.

**Algorithm:**

For each boundary vertex in this geometry:
1. Find nearest point on target geometry surface
2. Move vertex to that location

```cpp
geometry coarse = read_geometry("coarse_bunny.off");
geometry fine = read_geometry("fine_bunny.off");

// Project coarse mesh onto fine mesh surface
coarse.project(fine);

// coarse vertices now lie on fine mesh surface
```

**Use Cases:**
- Mesh refinement while preserving original surface
- Fitting generated meshes to target surfaces
- Surface registration

### Smoothing

```cpp
geometry& smoothing(float delta = 0.1f, bool fix_boundary = false);
```

Applies Laplacian smoothing to the mesh vertices. Reduces mesh noise and irregularities.

**Parameters:**
- `delta`: Smoothing strength (0.0 = no smoothing, 1.0 = maximum). Default: 0.1
- `fix_boundary`: If `true`, boundary vertices remain fixed. Default: `false`

**Algorithm (Sangmin Park's method):**

For each vertex, move it toward the average position of its neighbors:

```
new_position = old_position + delta * (neighbor_average - old_position)
```

```cpp
geometry noisy = read_geometry("noisy_mesh.off");

// Light smoothing
noisy.smoothing(0.05f, false);

// Strong smoothing with fixed boundary
geometry smooth = read_geometry("mesh.off");
smooth.smoothing(0.5f, true);

// Iterate for stronger effect
for (int i = 0; i < 10; ++i) {
    smooth.smoothing(0.1f, false);
}
```

**Trade-offs:**
- **Low delta (0.01-0.1)**: Gentle smoothing, preserves features
- **High delta (0.3-0.9)**: Aggressive smoothing, may lose details
- **fix_boundary=true**: Preserves mesh outline, prevents shrinkage
- **fix_boundary=false**: Allows full smoothing, may cause shrinkage

### Quality Improvement

```cpp
geometry& quality_improve(int iterations, 
                         const std::string& improve_method = "geo_flow");
```

Improves mesh quality using LBIE (Level set, B-spline, Implicit surface, Extrapolation) methods.

**Parameters:**
- `iterations`: Number of improvement iterations
- `improve_method`: Quality improvement algorithm (default: "geo_flow")

**Methods:**
- `"geo_flow"`: Geometric flow-based improvement

```cpp
geometry poor_quality = read_geometry("poor_mesh.off");

// Improve mesh quality
poor_quality.quality_improve(10, "geo_flow");

// Quality metrics should improve:
// - Better triangle aspect ratios
// - More uniform edge lengths
// - Reduced skewness
```

**Use Cases:**
- Improving meshes generated from isosurfacing
- Preparing meshes for finite element analysis
- Reducing numerical errors in simulations

## File I/O

### Reading Geometries

**Method 1: Constructor**

```cpp
geometry(const std::string& filename);
```

```cpp
geometry mesh("bunny.off");
geometry mesh2("model.raw");
```

**Method 2: read() Member Function**

```cpp
geometry& read(const std::string& filename);
```

```cpp
geometry mesh;
mesh.read("bunny.off");

// Chain operations
mesh.read("input.off")
    .calculate_surf_normals()
    .smoothing(0.1f)
    .write("output.raw");
```

**Method 3: Free Function (Recommended)**

```cpp
geometry read_geometry(const std::string& filename);
```

```cpp
#include <cvc/geometry_file_io.h>

geometry mesh = read_geometry("bunny.off");
geometry mesh2 = read_geometry("model.raw");
```

**File Format Auto-Detection:**

Format is determined by file extension:

| Extension | Format | Description |
|-----------|--------|-------------|
| `.off` | Object File Format | Standard triangle mesh format |
| `.raw` | CVC Raw | Uncompressed binary geometry |
| `.rawn` | CVC Raw with Normals | Binary geometry with normals |
| `.rawc` | CVC Raw with Colors | Binary geometry with colors |
| `.rawnc` | CVC Raw with Normals+Colors | Binary with normals and colors |
| `.obj` | Wavefront OBJ | Via SDF (experimental) |
| `.bunny` | Stanford Bunny | Embedded test mesh |

### Writing Geometries

**Member Function:**

```cpp
void write(const std::string& filename) const;
```

```cpp
geometry mesh;
// ... build or load mesh ...
mesh.write("output.off");
mesh.write("output.raw");
```

**Format Selection:**

Output format determined by extension:

```cpp
mesh.write("mesh.off");    // OFF format
mesh.write("mesh.raw");    // CVC Raw format
mesh.write("mesh.rawn");   // CVC Raw with normals
mesh.write("mesh.rawc");   // CVC Raw with colors
mesh.write("mesh.rawnc");  // CVC Raw with normals and colors
```

### Supported Formats

#### OFF (Object File Format)

**Description:** ASCII or binary format for triangle meshes. Widely supported.

**Structure:**
```
OFF
<num_vertices> <num_triangles> 0
<x0> <y0> <z0>
<x1> <y1> <z1>
...
3 <i0> <i1> <i2>
3 <j0> <j1> <j2>
...
```

**Read:** ✅ Supported  
**Write:** ✅ Supported

#### RAW (CVC Raw Format)

**Description:** Uncompressed binary format optimized for fast loading.

**Variants:**
- `.raw` - Points and triangles only
- `.rawn` - Points, triangles, normals
- `.rawc` - Points, triangles, colors
- `.rawnc` - Points, triangles, normals, colors

**Read:** ✅ Supported  
**Write:** ✅ Supported

#### OBJ (Wavefront OBJ)

**Description:** Standard 3D object format. Support via SDF module (experimental).

**Read:** ⚠️ Experimental (via SDF)  
**Write:** ❌ Not supported

#### BUNNY (Embedded Stanford Bunny)

**Description:** Special file type that loads the embedded Stanford Bunny test mesh.

**Usage:**
```cpp
geometry bunny = read_geometry("test.bunny");
// Always loads the same 34,835 vertex bunny mesh
```

**Read:** ✅ Supported (embedded)  
**Write:** ❌ Not applicable

### File I/O Extension System

The file I/O system is extensible via the `geometry_file_io` interface:

```cpp
struct geometry_file_io {
    virtual const std::string& id() const = 0;
    virtual const extension_list& extensions() const = 0;
    virtual geometry read(const std::string& filename) const = 0;
    virtual void write(const geometry& geom, 
                      const std::string& filename) const = 0;
    virtual bounding_box extents(const std::string& filename);
};
```

**Registering Custom Handlers:**

```cpp
class MyGeometryIO : public geometry_file_io {
    // Implement interface...
};

// Register with system
geometry_file_io::insert_handler(
    boost::make_shared<MyGeometryIO>()
);
```

**Handler Map:**

The system maintains a map of file extensions to handler objects. When reading/writing, it:

1. Extracts file extension
2. Looks up handlers for that extension
3. Tries each handler until one succeeds

## Type Definitions

### Scalar and Index Types

```cpp
typedef double scalar_t;      // Floating-point precision
typedef uint64_t index_t;      // Index type (64-bit unsigned)
```

### Geometric Types

```cpp
typedef boost::array<scalar_t, 3> point_t;    // 3D point
typedef boost::array<scalar_t, 3> vector_t;   // 3D vector
typedef vector_t normal_t;                     // Normal vector
typedef boost::array<scalar_t, 3> color_t;    // RGB color
```

**Usage:**

```cpp
point_t p = {{1.0, 2.0, 3.0}};
vector_t v = {{0.0, 1.0, 0.0}};
color_t red = {{1.0, 0.0, 0.0}};

// Access components
double x = p[0];
double y = p[1];
double z = p[2];
```

### Connectivity Types

```cpp
typedef boost::array<index_t, 2> line_t;   // Line segment
typedef boost::array<index_t, 3> tri_t;    // Triangle
typedef boost::array<index_t, 4> quad_t;   // Quadrilateral
```

**Usage:**

```cpp
line_t edge = {{0, 1}};         // Line from vertex 0 to 1
tri_t triangle = {{0, 1, 2}};   // Triangle with vertices 0, 1, 2
quad_t quad = {{0, 1, 2, 3}};   // Quad with vertices 0, 1, 2, 3
```

### Container Types

```cpp
typedef std::vector<point_t>  points_t;
typedef std::vector<normal_t> normals_t;
typedef std::vector<color_t>  colors_t;
typedef std::vector<line_t>   lines_t;
typedef std::vector<tri_t>    tris_t;
typedef std::vector<quad_t>   quads_t;
typedef boost::dynamic_bitset<> boundary_t;
```

### Shared Pointer Types

```cpp
typedef boost::shared_ptr<points_t>   points_ptr_t;
typedef boost::shared_ptr<boundary_t> boundary_ptr_t;
typedef boost::shared_ptr<normals_t>  normals_ptr_t;
typedef boost::shared_ptr<colors_t>   colors_ptr_t;
typedef boost::shared_ptr<lines_t>    lines_ptr_t;
typedef boost::shared_ptr<tris_t>     tris_ptr_t;
typedef boost::shared_ptr<quads_t>    quads_ptr_t;
```

## Complete Examples

### Example 1: Load, Process, and Save

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <iostream>

using namespace CVC_NAMESPACE;

int main() {
    // Load mesh
    geometry mesh = read_geometry("bunny.off");
    
    std::cout << "Loaded mesh:\n";
    std::cout << "  Vertices: " << mesh.num_points() << "\n";
    std::cout << "  Triangles: " << mesh.num_tris() << "\n";
    
    // Calculate normals
    mesh.calculate_surf_normals();
    
    // Smooth the mesh
    mesh.smoothing(0.1f, true);  // delta=0.1, fix boundary
    
    // Get extents
    bounding_box bbox = mesh.extents();
    std::cout << "Bounding box: [" 
              << bbox.minx << ", " << bbox.maxx << "] × ["
              << bbox.miny << ", " << bbox.maxy << "] × ["
              << bbox.minz << ", " << bbox.maxz << "]\n";
    
    // Save result
    mesh.write("bunny_smooth.raw");
    
    return 0;
}
```

### Example 2: Combine Multiple Meshes

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <vector>
#include <string>

using namespace CVC_NAMESPACE;

geometry combine_meshes(const std::vector<std::string>& filenames) {
    geometry combined;
    
    for (const auto& filename : filenames) {
        geometry part = read_geometry(filename);
        
        if (combined.empty()) {
            combined = part;
        } else {
            combined.merge(part);
        }
    }
    
    return combined;
}

int main() {
    std::vector<std::string> parts = {
        "part1.off",
        "part2.off",
        "part3.off",
        "part4.off"
    };
    
    geometry assembled = combine_meshes(parts);
    
    std::cout << "Assembled mesh has " << assembled.num_points() 
              << " vertices and " << assembled.num_tris() 
              << " triangles\n";
    
    // Calculate normals for entire assembly
    assembled.calculate_surf_normals();
    
    assembled.write("assembled.raw");
    
    return 0;
}
```

### Example 3: Extract and Analyze Surface

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <cmath>

using namespace CVC_NAMESPACE;

// Calculate triangle area
double triangle_area(const point_t& p0, const point_t& p1, const point_t& p2) {
    // Edge vectors
    double e1x = p1[0] - p0[0];
    double e1y = p1[1] - p0[1];
    double e1z = p1[2] - p0[2];
    
    double e2x = p2[0] - p0[0];
    double e2y = p2[1] - p0[1];
    double e2z = p2[2] - p0[2];
    
    // Cross product
    double cx = e1y * e2z - e1z * e2y;
    double cy = e1z * e2x - e1x * e2z;
    double cz = e1x * e2y - e1y * e2x;
    
    // Area = 0.5 * |cross product|
    return 0.5 * std::sqrt(cx*cx + cy*cy + cz*cz);
}

void analyze_mesh(const geometry& mesh) {
    std::cout << "Mesh Analysis:\n";
    std::cout << "  Vertices: " << mesh.num_points() << "\n";
    std::cout << "  Triangles: " << mesh.num_tris() << "\n";
    
    // Calculate total surface area
    double total_area = 0.0;
    double min_area = std::numeric_limits<double>::max();
    double max_area = 0.0;
    
    const auto& points = mesh.points();
    const auto& tris = mesh.tris();
    
    for (const auto& tri : tris) {
        const point_t& p0 = points[tri[0]];
        const point_t& p1 = points[tri[1]];
        const point_t& p2 = points[tri[2]];
        
        double area = triangle_area(p0, p1, p2);
        total_area += area;
        min_area = std::min(min_area, area);
        max_area = std::max(max_area, area);
    }
    
    std::cout << "  Total surface area: " << total_area << "\n";
    std::cout << "  Average triangle area: " 
              << (total_area / mesh.num_tris()) << "\n";
    std::cout << "  Min triangle area: " << min_area << "\n";
    std::cout << "  Max triangle area: " << max_area << "\n";
    
    // Bounding box
    bounding_box bbox = mesh.extents();
    double width = bbox.maxx - bbox.minx;
    double height = bbox.maxy - bbox.miny;
    double depth = bbox.maxz - bbox.minz;
    
    std::cout << "  Bounding box size: " 
              << width << " × " << height << " × " << depth << "\n";
}

int main() {
    geometry bunny = read_geometry("bunny.off");
    analyze_mesh(bunny);
    
    // Extract surface (for demonstration)
    geometry surface = bunny.tri_surface();
    std::cout << "\nSurface extraction:\n";
    analyze_mesh(surface);
    
    return 0;
}
```

### Example 4: Generate Procedural Mesh

```cpp
#include <cvc/geometry.h>
#include <cmath>

using namespace CVC_NAMESPACE;

geometry create_icosahedron() {
    geometry icosa;
    
    // Golden ratio
    const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    const double scale = 1.0 / std::sqrt(phi * phi + 1.0);
    
    // 12 vertices of icosahedron
    icosa.points().push_back({{-1.0 * scale,  phi * scale,  0.0}});
    icosa.points().push_back({{ 1.0 * scale,  phi * scale,  0.0}});
    icosa.points().push_back({{-1.0 * scale, -phi * scale,  0.0}});
    icosa.points().push_back({{ 1.0 * scale, -phi * scale,  0.0}});
    
    icosa.points().push_back({{ 0.0, -1.0 * scale,  phi * scale}});
    icosa.points().push_back({{ 0.0,  1.0 * scale,  phi * scale}});
    icosa.points().push_back({{ 0.0, -1.0 * scale, -phi * scale}});
    icosa.points().push_back({{ 0.0,  1.0 * scale, -phi * scale}});
    
    icosa.points().push_back({{ phi * scale,  0.0, -1.0 * scale}});
    icosa.points().push_back({{ phi * scale,  0.0,  1.0 * scale}});
    icosa.points().push_back({{-phi * scale,  0.0, -1.0 * scale}});
    icosa.points().push_back({{-phi * scale,  0.0,  1.0 * scale}});
    
    // 20 triangular faces
    icosa.tris().push_back({{0, 11, 5}});
    icosa.tris().push_back({{0, 5, 1}});
    icosa.tris().push_back({{0, 1, 7}});
    icosa.tris().push_back({{0, 7, 10}});
    icosa.tris().push_back({{0, 10, 11}});
    
    icosa.tris().push_back({{1, 5, 9}});
    icosa.tris().push_back({{5, 11, 4}});
    icosa.tris().push_back({{11, 10, 2}});
    icosa.tris().push_back({{10, 7, 6}});
    icosa.tris().push_back({{7, 1, 8}});
    
    icosa.tris().push_back({{3, 9, 4}});
    icosa.tris().push_back({{3, 4, 2}});
    icosa.tris().push_back({{3, 2, 6}});
    icosa.tris().push_back({{3, 6, 8}});
    icosa.tris().push_back({{3, 8, 9}});
    
    icosa.tris().push_back({{4, 9, 5}});
    icosa.tris().push_back({{2, 4, 11}});
    icosa.tris().push_back({{6, 2, 10}});
    icosa.tris().push_back({{8, 6, 7}});
    icosa.tris().push_back({{9, 8, 1}});
    
    // Calculate normals
    icosa.calculate_surf_normals();
    
    return icosa;
}

int main() {
    geometry icosa = create_icosahedron();
    icosa.write("icosahedron.off");
    
    std::cout << "Created icosahedron with " 
              << icosa.num_points() << " vertices and "
              << icosa.num_tris() << " triangles\n";
    
    return 0;
}
```

### Example 5: Mesh Quality Analysis and Improvement

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <cmath>
#include <limits>

using namespace CVC_NAMESPACE;

// Calculate triangle aspect ratio (quality metric)
double triangle_aspect_ratio(const point_t& p0, 
                             const point_t& p1, 
                             const point_t& p2) {
    // Edge lengths
    auto dist = [](const point_t& a, const point_t& b) {
        double dx = b[0] - a[0];
        double dy = b[1] - a[1];
        double dz = b[2] - a[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };
    
    double e0 = dist(p0, p1);
    double e1 = dist(p1, p2);
    double e2 = dist(p2, p0);
    
    double longest = std::max({e0, e1, e2});
    double shortest = std::min({e0, e1, e2});
    
    return shortest / longest;  // 1.0 = perfect equilateral
}

void analyze_quality(const geometry& mesh, const std::string& label) {
    const auto& points = mesh.points();
    const auto& tris = mesh.tris();
    
    double total_quality = 0.0;
    double min_quality = 1.0;
    
    for (const auto& tri : tris) {
        double quality = triangle_aspect_ratio(
            points[tri[0]], points[tri[1]], points[tri[2]]
        );
        total_quality += quality;
        min_quality = std::min(min_quality, quality);
    }
    
    double avg_quality = total_quality / mesh.num_tris();
    
    std::cout << label << ":\n";
    std::cout << "  Average quality: " << avg_quality << "\n";
    std::cout << "  Minimum quality: " << min_quality << "\n";
}

int main() {
    geometry mesh = read_geometry("poor_quality.off");
    
    analyze_quality(mesh, "Original mesh");
    
    // Improve quality
    mesh.quality_improve(10, "geo_flow");
    
    analyze_quality(mesh, "After quality improvement");
    
    // Additional smoothing
    mesh.smoothing(0.05f, true);
    
    analyze_quality(mesh, "After smoothing");
    
    mesh.write("improved.raw");
    
    return 0;
}
```

## Best Practices

### Memory Management

**1. Understand Copy-on-Write:**

```cpp
// GOOD: Efficient passing to read-only functions
void analyzeGeometry(const geometry& mesh) {
    // No copy, no COW trigger
    std::cout << "Vertices: " << mesh.num_points() << "\n";
}

geometry bunny = read_geometry("bunny.off");
analyzeGeometry(bunny);  // No overhead

// GOOD: Intentional sharing
geometry original = read_geometry("mesh.off");
geometry backup = original;  // Share data

// Modify only one
original.smoothing(0.1f);  // Triggers COW, backup unchanged
```

**2. Avoid Unintended Sharing:**

```cpp
// BAD: Unintended sharing
geometry mesh1 = read_geometry("mesh.off");
geometry mesh2 = mesh1;  // Shares data!

mesh1.points()[0] = {{0, 0, 0}};  // Triggers COW
mesh2.points()[0] = {{1, 1, 1}};  // Triggers another COW

// GOOD: Explicit independent copy
geometry mesh1 = read_geometry("mesh.off");
geometry mesh2 = mesh1;
mesh2.points();  // Force COW immediately
```

### Performance

**1. Minimize Copies:**

```cpp
// BAD: Unnecessary copies
geometry process(geometry mesh) {  // Copy on call
    mesh.smoothing(0.1f);
    return mesh;  // Copy on return
}

// GOOD: Use references
void process(geometry& mesh) {
    mesh.smoothing(0.1f);
}

// OR: Use move semantics (C++11)
geometry process(geometry&& mesh) {
    mesh.smoothing(0.1f);
    return std::move(mesh);
}
```

**2. Reserve Capacity:**

```cpp
// GOOD: Reserve space when size is known
geometry mesh;
mesh.points().reserve(10000);
mesh.tris().reserve(20000);

for (int i = 0; i < 10000; ++i) {
    mesh.points().push_back(generate_point(i));
}
```

**3. Batch Operations:**

```cpp
// BAD: Incremental normal calculation
for (auto& normal : mesh.normals()) {
    // Recalculate each time
}

// GOOD: Single bulk operation
mesh.calculate_surf_normals();  // Processes all at once
```

### Data Integrity

**1. Validate Indices:**

```cpp
// GOOD: Check triangle indices
void validate_mesh(const geometry& mesh) {
    uint64_t num_pts = mesh.num_points();
    
    for (const auto& tri : mesh.tris()) {
        if (tri[0] >= num_pts || tri[1] >= num_pts || tri[2] >= num_pts) {
            throw std::runtime_error("Invalid triangle index");
        }
    }
}
```

**2. Synchronize Arrays:**

```cpp
// GOOD: Keep arrays synchronized
void add_vertex_with_normal(geometry& mesh, 
                           const point_t& pt,
                           const normal_t& n) {
    mesh.points().push_back(pt);
    
    // Ensure normals array is same size
    if (mesh.normals().size() < mesh.num_points()) {
        mesh.normals().resize(mesh.num_points());
    }
    mesh.normals().back() = n;
}
```

### File I/O

**1. Use Appropriate Formats:**

```cpp
// RAW for fast loading (binary)
mesh.write("data.raw");

// OFF for portability (ASCII)
mesh.write("data.off");

// RAWN for normals
mesh.calculate_surf_normals();
mesh.write("data.rawn");
```

**2. Handle Errors:**

```cpp
// GOOD: Handle file errors
try {
    geometry mesh = read_geometry("file.off");
} catch (const std::exception& e) {
    std::cerr << "Failed to load: " << e.what() << "\n";
    // Handle error
}
```

## Performance Considerations

### Memory Usage

```cpp
// Approximate memory per geometry:

sizeof(point_t) = 3 * sizeof(double) = 24 bytes
sizeof(tri_t) = 3 * sizeof(uint64_t) = 24 bytes
sizeof(normal_t) = 3 * sizeof(double) = 24 bytes

// Stanford Bunny (34,835 vertices, 69,473 triangles):
points:   34,835 * 24 = 836 KB
tris:     69,473 * 24 = 1,667 KB
normals:  34,835 * 24 = 836 KB
Total:    ~3.3 MB

// Large mesh (1M vertices, 2M triangles):
points:   1M * 24 = 24 MB
tris:     2M * 24 = 48 MB  
normals:  1M * 24 = 24 MB
Total:    ~96 MB
```

### Copy-on-Write Overhead

```cpp
// Shallow copy is O(1)
geometry mesh1 = read_geometry("bunny.off");
geometry mesh2 = mesh1;  // Instant (just pointer copy)

// First write triggers full copy: O(n)
mesh2.points().push_back({{0, 0, 0}});  
// Copies all 3.3 MB on first modification

// Subsequent writes are O(1)
mesh2.points().push_back({{1, 1, 1}});  // No copy
```

### Operation Complexity

| Operation | Complexity | Notes |
|-----------|------------|-------|
| `num_points()` | O(1) | Vector size |
| `num_tris()` | O(1) | Vector size |
| `extents()` | O(n) | Cached after first call |
| `empty()` | O(1) | Checks point count |
| `merge()` | O(n + m) | n = this size, m = other size |
| `tri_surface()` | O(t) | t = triangle count |
| `calculate_surf_normals()` | O(t) | t = triangle count |
| `smoothing()` | O(v * i) | v = vertices, i = iterations |
| `quality_improve()` | O(t * i) | t = triangles, i = iterations |

### Optimization Tips

**1. Minimize Extents Recalculation:**

```cpp
// BAD: Recalculates extents each time
for (int i = 0; i < 1000; ++i) {
    mesh.points().push_back(pt);
    bounding_box bbox = mesh.extents();  // O(n) each time!
}

// GOOD: Calculate once
for (int i = 0; i < 1000; ++i) {
    mesh.points().push_back(pt);
}
bounding_box bbox = mesh.extents();  // O(n) once
```

**2. Use Const Access When Possible:**

```cpp
// GOOD: Const access avoids COW triggers
void render(const geometry& mesh) {
    const auto& points = mesh.points();  // No COW
    const auto& tris = mesh.tris();      // No COW
    
    // Render...
}
```

## Exception Handling

**Common Exceptions:**

```cpp
#include <cvc/exception.h>

// Unsupported file format
try {
    geometry mesh = read_geometry("file.unknown");
} catch (const unsupported_geometry_file_type& e) {
    std::cerr << "Unsupported file type: " << e.what() << "\n";
}

// File not found
try {
    geometry mesh = read_geometry("nonexistent.off");
} catch (const std::exception& e) {
    std::cerr << "Cannot read file: " << e.what() << "\n";
}

// Invalid mesh operations
try {
    geometry empty;
    bounding_box bbox = empty.extents();  // May throw for empty mesh
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
}
```

**Exception-Safe Code:**

```cpp
// RAII ensures cleanup
void processGeometry(const std::string& input, 
                    const std::string& output) {
    geometry mesh = read_geometry(input);  // May throw
    
    mesh.calculate_surf_normals();  // May throw
    mesh.smoothing(0.1f);           // May throw
    
    mesh.write(output);  // May throw
    
    // mesh destructor called automatically even if exception thrown
}
```

---

*For additional documentation, see:*
- [VOLUME_API.md](VOLUME_API.md) - Volume and voxels data structures
- [APP_API.md](APP_API.md) - Application framework
- [STATE_API.md](STATE_API.md) - State tree management
- [SDF_API.md](SDF_API.md) - Signed distance field operations
- [SDF_LIBRARY.md](SDF_LIBRARY.md) - SDF computation library
