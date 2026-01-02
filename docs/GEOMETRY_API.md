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
- [Volumetric Meshing](#volumetric-meshing)
  - [Signed Distance Function (SDF)](#signed-distance-function-sdf)
  - [Isosurface Extraction](#isosurface-extraction)
  - [Tetrahedral Meshing](#tetrahedral-meshing)
  - [Hexahedral Meshing](#hexahedral-meshing)
  - [Interval/Layer Meshing (Tet2)](#intervallayer-meshing-tet2)
  - [Complete Volumetric Mesh Example](#complete-volumetric-mesh-example)
- [Volumetric Mesh Utilities](#volumetric-mesh-utilities)
  - [Property Interpolation](#property-interpolation)
  - [Mesh Encoding and Decoding](#mesh-encoding-and-decoding)
  - [Surface Extraction from Volumetric Meshes](#surface-extraction-from-volumetric-meshes)
  - [Quality Metrics](#quality-metrics)
  - [Quality Analysis and Filtering](#quality-analysis-and-filtering)
  - [Point Location](#point-location)
  - [Mesh Analysis](#mesh-analysis)
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
- **Property interpolation**: Barycentric and trilinear interpolation for scalar/vector fields
- **Quality metrics**: Comprehensive element quality analysis (aspect ratio, Jacobian, angles)
- **Quality filtering**: Statistical analysis and mesh filtering by quality thresholds
- **Point location**: Find elements containing arbitrary query points
- **Mesh conversion**: Encoding/decoding between tet/hex and surface representations
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
- Volumetric mesh analysis and quality assessment
- Property interpolation for FEM and scientific computing
- Point location queries for ray tracing and collision detection
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
                         improvement_method method = IMPROVE_GEO_FLOW);
```

Improves mesh quality using LBIE (Level set, B-spline, Implicit surface, Extrapolation) methods.

**Parameters:**
- `iterations`: Number of improvement iterations
- `method`: Quality improvement algorithm enum (default: `IMPROVE_GEO_FLOW`)

**Available Methods:**
- `IMPROVE_NO_IMPROVE`: No improvement (pass-through)
- `IMPROVE_GEO_FLOW`: Geometric flow-based improvement (default, recommended)
- `IMPROVE_EDGE_CONTRACT`: Edge contraction
- `IMPROVE_JOE_LIU`: Joe-Liu algorithm
- `IMPROVE_MINIMAL_VOL`: Minimal volume optimization
- `IMPROVE_OPTIMIZATION`: General optimization

```cpp
#include <cvc/algorithm.h>

geometry poor_quality = read_geometry("poor_mesh.off");

// Improve mesh quality with geometric flow (default)
poor_quality.quality_improve(10, IMPROVE_GEO_FLOW);

// Try different methods
poor_quality.quality_improve(5, IMPROVE_EDGE_CONTRACT);
poor_quality.quality_improve(10, IMPROVE_OPTIMIZATION);

// Quality metrics should improve:
// - Better triangle aspect ratios
// - More uniform edge lengths
// - Reduced skewness
```

**Use Cases:**
- Improving meshes generated from isosurfacing
- Preparing meshes for finite element analysis
- Reducing numerical errors in simulations
- Post-processing volumetric meshes

## Volumetric Meshing

The geometry API supports creating volumetric meshes (tetrahedral and hexahedral) from signed distance functions. These are useful for finite element analysis, material simulation, and advanced visualization.

### Signed Distance Function (SDF)

```cpp
volume sdf(const geometry& geom,
           const dimension& dim,
           const bounding_box& bbox,
           sdf_algorithm algorithm = SDF_V2,
           bool flipNormals = false);
```

Computes the signed distance function for a geometry, creating a volume where each voxel stores the signed distance to the nearest surface point.

**Parameters:**
- `geom`: Input triangle mesh (must be watertight for correct sign)
- `dim`: Volume dimensions (e.g., `dimension(64, 64, 64)`)
- `bbox`: Bounding box for the volume (use `geom.extents()` to match geometry)
- `algorithm`: SDF algorithm selection (default: `SDF_V2`)
- `flipNormals`: Invert inside/outside regions (default: `false`)

**Algorithms:**
- `SDF_V1`: Legacy SDFLibrary (power-of-2 only, slower, single-threaded)
- `SDF_V2`: Modern DistanceTransform (11x faster, thread-safe, arbitrary dimensions)

**Sign Convention:**
- **Negative values**: Inside the surface
- **Zero**: On the surface
- **Positive values**: Outside the surface

```cpp
#include <cvc/algorithm.h>

geometry bunny = read_geometry("bunny.off");

// Create SDF with v2 algorithm (recommended)
dimension sdf_dim(64, 64, 64);
volume sdf_vol = sdf(bunny, sdf_dim, bunny.extents(), SDF_V2);

std::cout << "SDF range: [" << sdf_vol.min() << ", " << sdf_vol.max() << "]\n";

// Flip normals to invert inside/outside
volume inverted_sdf = sdf(bunny, sdf_dim, bunny.extents(), SDF_V2, true);
```

**flipNormals Feature** (New in v2.0):

The `flipNormals` parameter inverts the sign of all SDF values, effectively swapping inside and outside regions:

```cpp
// Normal: inside = negative, outside = positive
volume normal_sdf = sdf(geom, dim, bbox, SDF_V2, false);

// Flipped: inside = positive, outside = negative  
volume flipped_sdf = sdf(geom, dim, bbox, SDF_V2, true);

// Use case: extracting the "shell" around an object
volume shell_sdf = sdf(geom, dim, bbox, SDF_V2, true);
geometry shell = tetrahedralize2(shell_sdf, -0.05, 0.05);
```

**Implementation Details:**
- **SDF v1**: Post-computation negation (bypasses SDFLibrary's "fireworks" normal orientation bug)
- **SDF v2**: Calls `FaceVertSet3D::flipTriNormals()` before distance transform

### Isosurface Extraction

```cpp
geometry iso(const volume& vol, 
             double isovalue,
             extraction_method method = EXTRACT_DUALLIB,
             int improve_iterations = 1);
```

Extracts a triangle mesh representing a specific isovalue from a volume (typically an SDF).

**Parameters:**
- `vol`: Input volume (e.g., from `sdf()`)
- `isovalue`: Surface value to extract (0.0 for exact surface in SDF)
- `method`: Extraction algorithm (default: `EXTRACT_DUALLIB`)
- `improve_iterations`: Number of quality improvement passes (default: 1)

**Extraction Methods:**
- `EXTRACT_FASTCONTOURING`: Fast marching cubes variant
- `EXTRACT_LIBISOCONTOUR`: Library-based isocontour extraction
- `EXTRACT_DUALLIB`: Dual contouring (better quality, recommended)

```cpp
geometry bunny = read_geometry("bunny.off");
volume sdf_vol = sdf(bunny, dimension(64, 64, 64), bunny.extents());

// Extract surface at isovalue 0.0 (exact surface)
geometry surface = iso(sdf_vol, 0.0, EXTRACT_DUALLIB, 1);

// Extract slightly inside/outside surface
geometry inner = iso(sdf_vol, -0.01);  // Inside
geometry outer = iso(sdf_vol, 0.01);   // Outside
```

### Tetrahedral Meshing

```cpp
geometry tetrahedralize(const volume& vol,
                       double isovalue,
                       extraction_method method = EXTRACT_DUALLIB,
                       improvement_method improve_method = IMPROVE_GEO_FLOW,
                       int improve_iterations = 1);
```

Creates a tetrahedral volumetric mesh from a volume. Useful for finite element analysis and volumetric rendering.

**Parameters:**
- `vol`: Input volume (e.g., from `sdf()`)
- `isovalue`: Boundary isovalue
- `method`: Extraction algorithm
- `improve_method`: Quality improvement method
- `improve_iterations`: Number of improvement passes

```cpp
geometry bunny = read_geometry("bunny.off");
volume sdf_vol = sdf(bunny, dimension(64, 64, 64), bunny.extents());

// Create tetrahedral mesh
geometry tet_mesh = tetrahedralize(sdf_vol, 0.0, EXTRACT_DUALLIB, IMPROVE_GEO_FLOW, 3);

std::cout << "Tetrahedral mesh:\n";
std::cout << "  Vertices: " << tet_mesh.num_points() << "\n";
std::cout << "  Triangles: " << tet_mesh.num_tris() << "\n";

// Use for FEM analysis, volume rendering, etc.
```

### Hexahedral Meshing

```cpp
geometry hexahedralize(const volume& vol,
                      double isovalue,
                      extraction_method method = EXTRACT_DUALLIB,
                      improvement_method improve_method = IMPROVE_GEO_FLOW,
                      int improve_iterations = 1);
```

Creates a hexahedral (brick) volumetric mesh from a volume. Hexahedral meshes often perform better in FEM simulations.

**Parameters:** Same as `tetrahedralize()`

```cpp
geometry bunny = read_geometry("bunny.off");
volume sdf_vol = sdf(bunny, dimension(64, 64, 64), bunny.extents());

// Create hexahedral mesh
geometry hex_mesh = hexahedralize(sdf_vol, 0.0);

std::cout << "Hexahedral mesh:\n";
std::cout << "  Vertices: " << hex_mesh.num_points() << "\n";
std::cout << "  Quads: " << hex_mesh.num_quads() << "\n";
```

### Interval/Layer Meshing (Tet2)

```cpp
geometry tetrahedralize2(const volume& vol,
                        double isovalue_outer,
                        double isovalue_inner,
                        extraction_method method = EXTRACT_DUALLIB,
                        improvement_method improve_method = IMPROVE_GEO_FLOW,
                        int improve_iterations = 1);
```

Creates a tetrahedral mesh of the **layer/shell** between two isosurfaces. This is a dual-isovalue interval meshing technique unique to the LBIE mesher.

**Parameters:**
- `vol`: Input volume (SDF)
- `isovalue_outer`: Outer surface isovalue (more positive = further outside)
- `isovalue_inner`: Inner surface isovalue (more negative = further inside)
- `method`: Extraction algorithm
- `improve_method`: Quality improvement method
- `improve_iterations`: Number of improvement passes

**Use Cases:**
- Modeling material layers (skin, coatings, shells)
- Dual contouring between surfaces
- Volumetric region extraction
- Creating shells of specific thickness
- Multi-material simulations

```cpp
#include <cvc/algorithm.h>

geometry bunny = read_geometry("bunny.off");
volume sdf_vol = sdf(bunny, dimension(64, 64, 64), bunny.extents(), SDF_V2);

// Create a shell layer between two surfaces
// Remember: negative = inside, positive = outside in SDF
double outer_iso = 0.03;   // Outer boundary (outside surface)
double inner_iso = -0.03;  // Inner boundary (inside surface)

geometry shell = tetrahedralize2(sdf_vol, outer_iso, inner_iso);

std::cout << "Shell mesh between isovalues " << outer_iso 
          << " and " << inner_iso << ":\n";
std::cout << "  Vertices: " << shell.num_points() << "\n";
std::cout << "  Triangles: " << shell.num_tris() << "\n";

// Example: Create a 5mm thick shell around an object
double shell_thickness = 0.005;  // 5mm
geometry thick_shell = tetrahedralize2(sdf_vol, 
                                       0.0,              // Outer surface
                                       -shell_thickness); // Inner surface
```

**Implementation Notes:**

The tet2 interval meshing implementation had several bugs that were fixed in December 2025:

1. **Skip Condition Bug** (Fixed): `traverse_qef_interval()` had backwards logic for filtering octree cells. It was keeping only cells that fully spanned both isovalues (extremely restrictive), rather than cells that overlapped the interval.
   
2. **Infinite Refinement Bug** (Fixed): Missing octree depth check caused infinite subdivision when cells reached maximum depth.
   
3. **Intersection Handling Bug** (Fixed): `tetrahedralize_interval()` only processed intersection types ±1 (edge crosses one isovalue) but ignored type ±3 (edge crosses through entire interval), which is the most common case for interval meshing.

**Current Status:** Fully functional as of December 27, 2025. Generates high-quality volumetric meshes of layers between isosurfaces.

### Complete Volumetric Mesh Example

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <cvc/algorithm.h>

using namespace CVC_NAMESPACE;

int main() {
    // Load input geometry
    geometry bunny = read_geometry("bunny.off");
    
    // Create signed distance function
    dimension sdf_dim(128, 128, 128);  // Higher resolution = better quality
    volume sdf_vol = sdf(bunny, sdf_dim, bunny.extents(), SDF_V2);
    
    std::cout << "SDF computed. Range: [" 
              << sdf_vol.min() << ", " << sdf_vol.max() << "]\n";
    
    // Extract isosurface at zero level (exact surface)
    geometry surface = iso(sdf_vol, 0.0, EXTRACT_DUALLIB, 3);
    surface.write("bunny_surface.raw");
    
    // Create tetrahedral volumetric mesh
    geometry tet_vol = tetrahedralize(sdf_vol, 0.0, EXTRACT_DUALLIB, 
                                     IMPROVE_GEO_FLOW, 3);
    tet_vol.write("bunny_tets.raw");
    
    // Create hexahedral mesh
    geometry hex_vol = hexahedralize(sdf_vol, 0.0, EXTRACT_DUALLIB,
                                    IMPROVE_GEO_FLOW, 3);
    hex_vol.write("bunny_hexes.raw");
    
    // Create a 3mm shell around the bunny
    double shell_thickness = 0.003;
    geometry shell = tetrahedralize2(sdf_vol, 0.0, -shell_thickness,
                                    EXTRACT_DUALLIB, IMPROVE_GEO_FLOW, 3);
    shell.write("bunny_shell.raw");
    
    // Create a hollow layer between 2mm and 5mm from surface
    geometry hollow_layer = tetrahedralize2(sdf_vol, 0.005, 0.002);
    hollow_layer.write("bunny_hollow_layer.raw");
    
    std::cout << "All meshes generated successfully!\n";
    std::cout << "  Surface: " << surface.num_tris() << " triangles\n";
    std::cout << "  Tets: " << tet_vol.num_points() << " vertices\n";
    std::cout << "  Hexes: " << hex_vol.num_quads() << " quads\n";
    std::cout << "  Shell: " << shell.num_points() << " vertices, "
              << shell.num_tris() << " triangles\n";
    std::cout << "  Hollow: " << hollow_layer.num_points() << " vertices\n";
    
    return 0;
}
```

**Compilation:**

```bash
g++ -std=c++14 -O3 volumetric_mesh.cpp -lcvc -lboost_system -lboost_thread -o volumetric_mesh
./volumetric_mesh
```

## Volumetric Mesh Utilities

The geometry API provides comprehensive utilities for working with volumetric meshes (tetrahedra and hexahedra), including property interpolation, mesh conversion, quality analysis, and geometric queries.

### Property Interpolation

Property interpolation allows you to evaluate scalar or vector fields at arbitrary points within volumetric mesh elements using proper basis functions.

**Tetrahedral Interpolation:**

```cpp
double interpolate_in_tet(const geometry::point_t& p,
                         const geometry::point_t tet_verts[4],
                         const double tet_values[4]);
```

Interpolates using barycentric coordinates within a tetrahedron.

**Hexahedral Interpolation:**

```cpp
double interpolate_in_hex(const geometry::point_t& p,
                         const geometry::point_t hex_verts[8],
                         const double hex_values[8]);
```

Interpolates using trilinear basis functions within a hexahedron.

**Example:**

```cpp
#include <cvc/algorithm.h>

using namespace CVC_NAMESPACE;

// Tetrahedral mesh with temperature field
geometry tet_mesh = tetrahedralize(sdf_vol, 0.0);

// Define property values at vertices (e.g., temperature)
std::vector<double> temperature(tet_mesh.num_points());
for(size_t i = 0; i < temperature.size(); ++i) {
    temperature[i] = 273.15 + i * 0.1;  // Temperature in Kelvin
}

// Interpolate at a specific point
geometry::point_t query_point = {{0.5, 0.5, 0.5}};

// Find which tet contains the point
auto containing_tets = find_tets_containing_point(query_point, 
                                                   tet_mesh.const_tets(),
                                                   tet_mesh.points());

if(!containing_tets.empty()) {
    size_t tet_idx = containing_tets[0];
    const auto& tet = tet_mesh.const_tets()[tet_idx];
    
    // Gather vertex coordinates and values
    geometry::point_t tet_verts[4];
    double tet_temps[4];
    for(int i = 0; i < 4; ++i) {
        tet_verts[i] = tet_mesh.points()[tet[i]];
        tet_temps[i] = temperature[tet[i]];
    }
    
    // Interpolate temperature at query point
    double temp_at_point = interpolate_in_tet(query_point, tet_verts, tet_temps);
    
    std::cout << "Temperature at (" << query_point[0] << ", " 
              << query_point[1] << ", " << query_point[2] << "): "
              << temp_at_point << " K\n";
}
```

**Property Interpolation in Mesh Generation:**

```cpp
// Enable property interpolation during meshing
geometry iso_with_props = iso(sdf_vol, 0.0, EXTRACT_DUALLIB, 1, 
                              true);  // interpolate_property = true

geometry tet_with_props = tetrahedralize(sdf_vol, 0.0, EXTRACT_DUALLIB,
                                        IMPROVE_GEO_FLOW, 1, 
                                        true);  // interpolate_property = true

geometry hex_with_props = hexahedralize(sdf_vol, 0.0, EXTRACT_DUALLIB,
                                       IMPROVE_GEO_FLOW, 1,
                                       true);  // interpolate_property = true
```

When enabled, vertex properties (stored in `geometry.functions()`) are interpolated from the source volume's voxel values using the appropriate basis functions.

### Mesh Encoding and Decoding

Utilities for converting between different representations of volumetric meshes.

**Tetrahedral Mesh Encoding:**

```cpp
// Encode tets to quads (3 quads per tet, representing 3 of 4 faces)
geometry::quads_t encode_tets_to_quads(const geometry::tets_t& tets);

// Encode tets to triangles (4 triangles per tet)
geometry::tris_t encode_tets_to_tris(const geometry::tets_t& tets);

// Encode single tet
geometry::quad_t encode_tet_to_quad(const geometry::tet_t& tet, int face_idx);
```

**Tetrahedral Mesh Decoding:**

```cpp
// Decode quads back to tets
geometry::tets_t decode_quads_to_tets(const geometry::quads_t& quads);

// Decode triangles to tets
geometry::tets_t decode_tris_to_tets(const geometry::tris_t& tris);

// Decode single quad to tet
geometry::tet_t decode_quad_to_tet(const geometry::quad_t& quad);
```

**Hexahedral Mesh Encoding:**

```cpp
// Encode hexs to surface quads (6 quads per hex)
geometry::quads_t encode_hexs_to_quads(const geometry::hexs_t& hexs);

// Encode hexs to triangulated surface (12 triangles per hex)
geometry::tris_t encode_hexs_to_tris(const geometry::hexs_t& hexs);

// Encode single hex
geometry::quad_t encode_hex_to_quad(const geometry::hex_t& hex, int face_idx);
```

**Example:**

```cpp
using namespace CVC_NAMESPACE;

geometry tet_mesh = tetrahedralize(sdf_vol, 0.0);

// Convert tets to triangular surface for rendering
geometry::tris_t surface_tris = encode_tets_to_tris(tet_mesh.const_tets());

geometry surface_mesh;
surface_mesh.points() = tet_mesh.points();
surface_mesh.tris() = surface_tris;
surface_mesh.write("tet_surface.raw");

// Convert back if needed
geometry::tets_t recovered_tets = decode_tris_to_tets(surface_tris);
```

### Surface Extraction from Volumetric Meshes

Extract the boundary surface from tetrahedral or hexahedral meshes.

```cpp
geometry extract_surface(const geometry& geom);
```

Creates a new geometry containing only the boundary triangles/quads of a volumetric mesh. Automatically detects mesh type and extracts appropriate surface elements.

**Algorithm:**
- Builds a map of all element faces
- Identifies faces that appear only once (boundary faces)
- Creates surface mesh with proper vertex mapping

**Example:**

```cpp
using namespace CVC_NAMESPACE;

// Create volumetric mesh
geometry tet_mesh = tetrahedralize(sdf_vol, 0.0);

std::cout << "Volumetric mesh: " << tet_mesh.num_points() << " vertices, "
          << tet_mesh.num_tets() << " tets\n";

// Extract boundary surface
geometry surface = extract_surface(tet_mesh);

std::cout << "Boundary surface: " << surface.num_points() << " vertices, "
          << surface.num_tris() << " triangles\n";

// Surface can be processed separately
surface.calculate_surf_normals();
surface.write("boundary.raw");

// Works for hexahedral meshes too
geometry hex_mesh = hexahedralize(sdf_vol, 0.0);
geometry hex_surface = extract_surface(hex_mesh);  // Extracts quad faces
```

### Quality Metrics

Compute geometric quality metrics for individual volumetric mesh elements.

**Quality Metric Types:**

```cpp
enum quality_metric
{
  // Tetrahedral mesh metrics
  TET_VOLUME = 0,         // Volume of tetrahedron
  TET_ASPECT_RATIO = 1,   // Aspect ratio (lower is better)
  TET_MIN_ANGLE = 2,      // Minimum dihedral angle (higher is better)
  
  // Hexahedral mesh metrics
  HEX_VOLUME = 3,         // Volume of hexahedron
  HEX_JACOBIAN = 4,       // Jacobian determinant (positive is valid)
  HEX_SCALED_JACOBIAN = 5 // Scaled Jacobian quality [-1, 1]
};
```

**Tetrahedral Quality Functions:**

```cpp
// Signed volume of tetrahedron
double tet_volume(const geometry::point_t& v0,
                 const geometry::point_t& v1,
                 const geometry::point_t& v2,
                 const geometry::point_t& v3);

// Aspect ratio (ratio of longest edge to inradius)
// Lower is better; ideal tetrahedron has aspect ratio ≈ 2.04
double tet_aspect_ratio(const geometry::point_t& v0,
                       const geometry::point_t& v1,
                       const geometry::point_t& v2,
                       const geometry::point_t& v3);

// Minimum dihedral angle in degrees [0°, 180°]
// Higher is better; ideal tetrahedron has all angles ≈ 70.53°
double tet_min_dihedral_angle(const geometry::point_t& v0,
                              const geometry::point_t& v1,
                              const geometry::point_t& v2,
                              const geometry::point_t& v3);
```

**Hexahedral Quality Functions:**

```cpp
// Volume via decomposition to 5 tetrahedra
double hex_volume(const geometry::point_t hex_verts[8]);

// Jacobian determinant at hex center
// Positive values indicate valid (non-inverted) element
double hex_jacobian(const geometry::point_t hex_verts[8]);

// Scaled Jacobian quality metric in range [-1, 1]
// Values > 0.2 are generally acceptable
// Values near 1.0 are ideal
double hex_scaled_jacobian(const geometry::point_t hex_verts[8]);
```

**Example:**

```cpp
using namespace CVC_NAMESPACE;

geometry tet_mesh = tetrahedralize(sdf_vol, 0.0);

// Analyze quality of each tet
for(const auto& tet : tet_mesh.const_tets()) {
    const auto& v0 = tet_mesh.points()[tet[0]];
    const auto& v1 = tet_mesh.points()[tet[1]];
    const auto& v2 = tet_mesh.points()[tet[2]];
    const auto& v3 = tet_mesh.points()[tet[3]];
    
    double volume = tet_volume(v0, v1, v2, v3);
    double aspect = tet_aspect_ratio(v0, v1, v2, v3);
    double min_angle = tet_min_dihedral_angle(v0, v1, v2, v3);
    
    if(aspect > 10.0) {
        std::cout << "Warning: Poor quality tet with aspect ratio " 
                  << aspect << "\n";
    }
}

// Analyze hex quality
geometry hex_mesh = hexahedralize(sdf_vol, 0.0);
for(const auto& hex : hex_mesh.const_hexs()) {
    geometry::point_t verts[8];
    for(int i = 0; i < 8; ++i) {
        verts[i] = hex_mesh.points()[hex[i]];
    }
    
    double scaled_jac = hex_scaled_jacobian(verts);
    if(scaled_jac < 0.2) {
        std::cout << "Warning: Poor quality hex with scaled Jacobian " 
                  << scaled_jac << "\n";
    }
}
```

### Quality Analysis and Filtering

Compute statistics and filter meshes based on quality metrics.

**Quality Statistics Structure:**

```cpp
struct quality_stats {
    double min;      // Minimum quality value
    double max;      // Maximum quality value
    double mean;     // Average quality
    double std_dev;  // Standard deviation
};
```

**Statistical Analysis:**

```cpp
// Compute quality statistics for tetrahedral mesh
quality_stats compute_tet_quality_stats(const geometry::tets_t& tets,
                                        const geometry::points_t& vertices,
                                        quality_metric metric = TET_ASPECT_RATIO);

// Compute quality statistics for hexahedral mesh
quality_stats compute_hex_quality_stats(const geometry::hexs_t& hexs,
                                        const geometry::points_t& vertices,
                                        quality_metric metric = HEX_SCALED_JACOBIAN);
```

**Quality Filtering:**

```cpp
// Filter tets by quality threshold
// Returns indices of tets that meet quality criteria
std::vector<size_t> filter_tets_by_quality(const geometry::tets_t& tets,
                                           const geometry::points_t& vertices,
                                           double threshold = 10.0,
                                           quality_metric metric = TET_ASPECT_RATIO);

// Filter hexs by quality threshold
std::vector<size_t> filter_hexs_by_quality(const geometry::hexs_t& hexs,
                                           const geometry::points_t& vertices,
                                           double threshold = 0.2,
                                           quality_metric metric = HEX_SCALED_JACOBIAN);

// Extract high-quality elements to new geometry
geometry extract_quality_elements(const geometry& geom,
                                 double threshold,
                                 quality_metric metric = TET_ASPECT_RATIO);
```

**Threshold Interpretation:**
- `TET_ASPECT_RATIO`: Keep tets with aspect ratio **< threshold** (lower is better)
- `TET_VOLUME`: Keep tets with volume **> threshold**
- `TET_MIN_ANGLE`: Keep tets with min angle **> threshold** degrees
- `HEX_SCALED_JACOBIAN`: Keep hexs with scaled Jacobian **> threshold**
- `HEX_VOLUME`: Keep hexs with volume **> threshold**
- `HEX_JACOBIAN`: Keep hexs with Jacobian **> threshold**

**Example:**

```cpp
using namespace CVC_NAMESPACE;

geometry tet_mesh = tetrahedralize(sdf_vol, 0.0);

// Compute aspect ratio statistics
auto ar_stats = compute_tet_quality_stats(tet_mesh.const_tets(),
                                          tet_mesh.points(),
                                          TET_ASPECT_RATIO);

std::cout << "Aspect Ratio Statistics:\n";
std::cout << "  Min: " << ar_stats.min << "\n";
std::cout << "  Max: " << ar_stats.max << "\n";
std::cout << "  Mean: " << ar_stats.mean << "\n";
std::cout << "  Std Dev: " << ar_stats.std_dev << "\n";

// Filter out poor quality tets (aspect ratio > 5.0)
auto good_tets = filter_tets_by_quality(tet_mesh.const_tets(),
                                       tet_mesh.points(),
                                       5.0,
                                       TET_ASPECT_RATIO);

std::cout << "Good quality tets: " << good_tets.size() 
          << " / " << tet_mesh.num_tets() << "\n";

// Create mesh with only high-quality elements
geometry quality_mesh = extract_quality_elements(tet_mesh, 5.0, TET_ASPECT_RATIO);

std::cout << "Quality mesh: " << quality_mesh.num_tets() << " tets\n";
quality_mesh.write("quality_tets.raw");

// Analyze hexahedral mesh
geometry hex_mesh = hexahedralize(sdf_vol, 0.0);
auto sj_stats = compute_hex_quality_stats(hex_mesh.const_hexs(),
                                          hex_mesh.points(),
                                          HEX_SCALED_JACOBIAN);

std::cout << "Scaled Jacobian Statistics:\n";
std::cout << "  Min: " << sj_stats.min << "\n";
std::cout << "  Max: " << sj_stats.max << "\n";
std::cout << "  Mean: " << sj_stats.mean << "\n";

// Extract hexs with scaled Jacobian > 0.3
geometry quality_hex = extract_quality_elements(hex_mesh, 0.3, HEX_SCALED_JACOBIAN);
```

### Point Location

Find volumetric mesh elements containing a given point.

```cpp
// Find all tets containing a point
std::vector<size_t> find_tets_containing_point(const geometry::point_t& p,
                                               const geometry::tets_t& tets,
                                               const geometry::points_t& vertices);

// Find all hexs containing a point
std::vector<size_t> find_hexs_containing_point(const geometry::point_t& p,
                                               const geometry::hexs_t& hexs,
                                               const geometry::points_t& vertices);
```

Returns indices of all elements that contain the query point. 

**Algorithm:**
- **Tet location**: Uses barycentric coordinates for precise containment testing
- **Hex location**: Uses trilinear weights for precise containment testing
- **Multi-tier acceleration strategy** (adaptive based on mesh size):
  - **Small meshes** (<100 elements): Direct barycentric/trilinear tests, minimal overhead
  - **Medium meshes** (100-1000 elements): Manual bounding box pre-filtering
  - **Large meshes** (1000-10000 elements): Pre-computed `bounding_box` structures for fast spatial filtering
  - **Very large meshes** (>10000 elements with CGAL): CGAL AABB tree with surface triangles for O(log n) inside/outside test + bbox acceleration
  - **CGAL disabled**: Always uses `bounding_box`-accelerated linear search
- **Performance**:
  - Small meshes: O(n), minimal overhead
  - Medium meshes: O(n) with 5-10x speedup via bbox rejection
  - Large meshes: O(n) with 10-50x speedup via pre-computed `bounding_box` structures
  - Very large meshes with CGAL AABB tree: O(log n) surface test + O(n) bbox search with high rejection rate

**Implementation Details:**

1. **Bounding Box Acceleration** (all sizes):
   - Uses the existing `CVC::bounding_box` class from `cvc/bounding_box.h`
   - No external dependencies - works with or without CGAL
   - Pre-computes element bboxes once for large meshes (amortized cost)
   - Fast `contains()` test (6 comparisons) eliminates most elements

2. **CGAL AABB Tree** (very large meshes only, >10000 elements):
   - Extracts surface triangles using `extract_surface()`
   - Builds CGAL AABB tree for O(log n) spatial queries
   - Uses `do_intersect()` for quick inside/outside determination
   - Falls back to bbox-accelerated search only for nearby elements
   - Provides 100x+ speedup on meshes with millions of elements

**Memory Usage:**
- Small/medium: No additional storage
- Large (1000-10000): ~64 bytes per element (pre-computed bbox + index)
- Very large with AABB tree: Surface mesh memory + tree structure (~100-200 bytes/element)

**Example:**

```cpp
using namespace CVC_NAMESPACE;

geometry tet_mesh = tetrahedralize(sdf_vol, 0.0);

// Query point
geometry::point_t query = {{0.25, 0.25, 0.25}};

// Find containing tets
auto containing = find_tets_containing_point(query,
                                             tet_mesh.const_tets(),
                                             tet_mesh.points());

if(containing.empty()) {
    std::cout << "Point is outside the mesh\n";
} else {
    std::cout << "Point is in " << containing.size() << " tet(s)\n";
    
    // Use first containing tet for interpolation
    size_t tet_idx = containing[0];
    // ... perform interpolation ...
}
```

### Mesh Analysis

Utilities for analyzing mesh properties.

**Bounding Box Computation:**

```cpp
// Compute axis-aligned bounding box
std::array<double, 6> compute_mesh_bounds(const geometry& geom);
```

Returns `{min_x, min_y, min_z, max_x, max_y, max_z}`.

**Example:**

```cpp
using namespace CVC_NAMESPACE;

geometry mesh = tetrahedralize(sdf_vol, 0.0);

auto bounds = compute_mesh_bounds(mesh);

std::cout << "Mesh bounds:\n";
std::cout << "  X: [" << bounds[0] << ", " << bounds[3] << "]\n";
std::cout << "  Y: [" << bounds[1] << ", " << bounds[4] << "]\n";
std::cout << "  Z: [" << bounds[2] << ", " << bounds[5] << "]\n";

double width = bounds[3] - bounds[0];
double height = bounds[4] - bounds[1];
double depth = bounds[5] - bounds[2];

std::cout << "Dimensions: " << width << " × " << height << " × " << depth << "\n";
```

**Complete Workflow Example:**

```cpp
#include <cvc/geometry.h>
#include <cvc/geometry_file_io.h>
#include <cvc/algorithm.h>

using namespace CVC_NAMESPACE;

int main() {
    // Create volumetric mesh
    geometry input = read_geometry("model.off");
    volume sdf_vol = sdf(input, dimension(64, 64, 64), input.extents(), SDF_V2);
    geometry tet_mesh = tetrahedralize(sdf_vol, 0.0, EXTRACT_DUALLIB, IMPROVE_GEO_FLOW, 3);
    
    // Analyze mesh quality
    std::cout << "\n=== Quality Analysis ===\n";
    auto ar_stats = compute_tet_quality_stats(tet_mesh.const_tets(),
                                              tet_mesh.points(),
                                              TET_ASPECT_RATIO);
    auto vol_stats = compute_tet_quality_stats(tet_mesh.const_tets(),
                                               tet_mesh.points(),
                                               TET_VOLUME);
    
    std::cout << "Aspect Ratio: [" << ar_stats.min << ", " << ar_stats.max 
              << "], mean=" << ar_stats.mean << "\n";
    std::cout << "Volume: [" << vol_stats.min << ", " << vol_stats.max 
              << "], mean=" << vol_stats.mean << "\n";
    
    // Filter poor quality elements
    geometry quality_mesh = extract_quality_elements(tet_mesh, 8.0, TET_ASPECT_RATIO);
    std::cout << "\nFiltered: " << tet_mesh.num_tets() << " → " 
              << quality_mesh.num_tets() << " tets\n";
    
    // Extract boundary surface
    geometry surface = extract_surface(quality_mesh);
    std::cout << "Surface: " << surface.num_tris() << " triangles\n";
    
    // Compute bounds
    auto bounds = compute_mesh_bounds(quality_mesh);
    std::cout << "Bounds: [" << bounds[0] << "," << bounds[3] << "] × ["
              << bounds[1] << "," << bounds[4] << "] × ["
              << bounds[2] << "," << bounds[5] << "]\n";
    
    // Save results
    quality_mesh.write("output_volume.raw");
    surface.write("output_surface.raw");
    
    return 0;
}
```

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

### Volumetric Mesh Types

```cpp
typedef boost::array<index_t, 4> tet_t;    // Tetrahedron (4 vertices)
typedef boost::array<index_t, 8> hex_t;    // Hexahedron (8 vertices)

typedef std::vector<tet_t> tets_t;         // Tetrahedral mesh
typedef std::vector<hex_t> hexs_t;         // Hexahedral mesh

typedef boost::shared_ptr<tets_t> tets_ptr_t;
typedef boost::shared_ptr<hexs_t> hexs_ptr_t;
```

**Usage:**

```cpp
tet_t tetrahedron = {{0, 1, 2, 3}};  // Tet with vertices 0, 1, 2, 3
hex_t hexahedron = {{0, 1, 2, 3, 4, 5, 6, 7}};  // Hex with 8 vertices

tets_t tet_mesh;
tet_mesh.push_back(tetrahedron);
```

### Enumeration Types

**Quality Metrics:**

```cpp
enum quality_metric
{
  // Tetrahedral mesh metrics
  TET_VOLUME = 0,         // Volume of tetrahedron
  TET_ASPECT_RATIO = 1,   // Aspect ratio (lower is better)
  TET_MIN_ANGLE = 2,      // Minimum dihedral angle in degrees
  
  // Hexahedral mesh metrics
  HEX_VOLUME = 3,         // Volume of hexahedron
  HEX_JACOBIAN = 4,       // Jacobian determinant at center
  HEX_SCALED_JACOBIAN = 5 // Scaled Jacobian quality [-1, 1]
};
```

**Extraction Methods:**

```cpp
enum extraction_method
{
  DUALLIB = 0,          // Dual contouring (recommended)
  FASTCONTOURING = 1,   // Fast marching cubes
  LIBISOCONTOUR = 2     // Library-based extraction
};
```

**Improvement Methods:**

```cpp
enum improvement_method
{
  NO_IMPROVE = 0,      // No quality improvement
  GEO_FLOW = 1,        // Geometric flow smoothing
  EDGE_CONTRACT = 2,   // Edge contraction
  JOE_LIU = 3,         // Joe-Liu algorithm
  MINIMAL_VOL = 4,     // Minimal volume optimization
  OPTIMIZATION = 5     // General optimization
};
```

**Mesh Types:**

```cpp
enum mesh_type
{
  SURFACE_MESH = 0,   // Triangle surface mesh
  TETRAHEDRAL = 1,    // Tetrahedral volume mesh
  QUAD_MESH = 2,      // Quad surface mesh
  HEXAHEDRAL = 3,     // Hexahedral volume mesh
  DUAL_SURFACE = 4,   // Dual surface mesh
  TETRAHEDRAL2 = 5    // Interval tetrahedral mesh
};
```

**SDF Algorithms:**

```cpp
enum sdf_algorithm
{
  SDF_V1,  // Legacy SDFLibrary (single-threaded, power-of-2 only)
  SDF_V2   // Modern DistanceTransform (11x faster, thread-safe)
};
```

### Quality Statistics Structure

```cpp
struct quality_stats {
    double min;      // Minimum quality value in mesh
    double max;      // Maximum quality value in mesh
    double mean;     // Average quality across all elements
    double std_dev;  // Standard deviation of quality
};
```

**Usage:**

```cpp
quality_stats stats = compute_tet_quality_stats(tets, vertices, TET_ASPECT_RATIO);
std::cout << "Aspect ratio range: [" << stats.min << ", " << stats.max << "]\n";
std::cout << "Mean: " << stats.mean << " ± " << stats.std_dev << "\n";
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
