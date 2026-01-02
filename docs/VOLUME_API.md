# Volume and Voxels API Reference (VolMagick)

*Complete reference for the libcvc/VolMagick volume data library*

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Core Concepts](#core-concepts)
  - [Voxels vs Volume](#voxels-vs-volume)
  - [Memory Semantics](#memory-semantics)
  - [Data Types](#data-types)
  - [Coordinate Systems](#coordinate-systems)
- [Voxels Class](#voxels-class)
  - [Construction](#voxels-construction)
  - [Dimensions and Types](#dimensions-and-types)
  - [Data Access](#data-access)
  - [Copy Operations](#copy-operations)
  - [Subvolume Extraction](#subvolume-extraction)
  - [Resizing and Interpolation](#resizing-and-interpolation)
  - [Data Manipulation](#data-manipulation)
  - [Image Processing Filters](#image-processing-filters)
  - [Statistics](#statistics)
  - [Compositing](#compositing)
  - [CUDA Acceleration](#cuda-acceleration)
- [Volume Class](#volume-class)
  - [Construction](#volume-construction)
  - [Bounding Box Operations](#bounding-box-operations)
  - [Spatial Interpolation](#spatial-interpolation)
  - [Subvolume by Bounding Box](#subvolume-by-bounding-box)
  - [Volume Combination](#volume-combination)
  - [File I/O](#file-io)
- [Supporting Types](#supporting-types)
  - [dimension](#dimension)
  - [bounding_box](#bounding_box)
  - [data_type](#data_type)
  - [composite_function](#composite_function)
- [Complete Examples](#complete-examples)
- [Best Practices](#best-practices)
- [Performance Considerations](#performance-considerations)
- [Exception Handling](#exception-handling)

## Overview

The **VolMagick** library provides the `voxels` and `volume` classes for managing 3D volumetric data. These classes form the foundation for scientific visualization, medical imaging, and computational geometry in libcvc.

**Key Features:**
- **Type-safe voxel data**: Support for UChar, UShort, UInt, Float, Double, UInt64
- **Efficient memory management**: Shallow copy by default with optional deep copy
- **Spatial coordinate system**: Object-space bounding boxes with trilinear interpolation
- **Image processing**: Bilateral filter, contrast enhancement, anisotropic diffusion
- **File I/O**: RAWIV, MRC, HDF5, and other scientific formats
- **CUDA acceleration**: GPU-accelerated operations with unified memory
- **Copy-on-write**: Automatic data duplication on modification

**Class Hierarchy:**
```
voxels                  (Multi-dimensional voxel container)
  └── volume           (Voxels + spatial coordinate system)
```

**Thread Safety:** Not thread-safe by default. Use external synchronization for concurrent access.

## Quick Start

```cpp
#include <cvc/volume.h>
#include <cvc/voxels.h>

using namespace CVC_NAMESPACE;

// Create a 128³ volume with float data
dimension dim(128, 128, 128);
bounding_box bbox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
volume vol(dim, Float, bbox);

// Fill with gradient
for (uint64 k = 0; k < vol.ZDim(); ++k) {
    for (uint64 j = 0; j < vol.YDim(); ++j) {
        for (uint64 i = 0; i < vol.XDim(); ++i) {
            vol(i, j, k, i + j + k);
        }
    }
}

// Interpolate at arbitrary spatial coordinates
double value = vol.interpolate(5.0, 5.0, 5.0);

// Extract subvolume centered at (5,5,5)
bounding_box sub_bbox(4.0, 4.0, 4.0, 6.0, 6.0, 6.0);
volume sub = vol.sub(sub_bbox);

// Apply bilateral filter
vol.bilateralFilter(200.0, 1.5, 2);

// Save to file
vol.write("output.rawiv");

// Load from file
volume loaded("input.rawiv");
```

## Core Concepts

### Voxels vs Volume

**voxels:**
- Multi-dimensional array of typed data (UChar, Float, etc.)
- Indexed by integer coordinates: `(i, j, k)` where `0 <= i < XDim()`, etc.
- No spatial information - pure discrete grid
- Base class for volume

**volume:**
- Extends voxels with spatial coordinate system
- Bounding box maps voxel indices to object space
- Supports trilinear interpolation at arbitrary spatial coordinates
- Enables spatial operations (combine volumes, extract by spatial bounds)

### Memory Semantics

**CRITICAL - Shallow Copy by Default:**

Both `voxels` and `volume` use **shallow copy semantics** via `boost::shared_array`:

```cpp
// Shallow copy - SHARES underlying data
volume vol1(dimension(64, 64, 64), Float);
volume vol2(vol1);        // Copy constructor - SHARES data
volume vol3 = vol1;       // Assignment - SHARES data
vol3.copy(vol1);          // copy() default - SHARES data
vol3.copy(vol1, false);   // copy() explicit - SHARES data

// All four volumes share the same voxel data array!
vol2(10, 10, 10, 42.0);   // Modifies vol1, vol2, vol3 all!
```

**Copy-on-Write Protection:**

Shallow copies are protected by copy-on-write (COW):

```cpp
volume vol1(dimension(64, 64, 64), Float);
volume vol2(vol1);        // Shares data initially

vol2(10, 10, 10, 42.0);   // Triggers COW - vol2 gets its own copy
// Now vol1 and vol2 have independent data
```

**Deep Copy (Independent Data):**

```cpp
// Method 1: Use copy() with deep copy flag
volume vol2;
vol2.copy(vol1, true);    // Creates independent data

// Method 2: Use sub() to extract full volume
volume vol2(vol1);
vol2.sub(0, 0, 0, vol1.voxel_dimensions());  // Creates new array

// Method 3: Manual element-by-element copy
volume vol2(vol1.voxel_dimensions(), vol1.voxelType(), vol1.boundingBox());
for (uint64 i = 0; i < vol1.voxel_dimensions().size(); ++i) {
    vol2(i, vol1(i));
}
vol2.desc(vol1.desc());
```

**When to Use Shallow vs Deep Copy:**

- **Shallow (default)**: Passing volumes to functions, temporary references, read-only access
- **Deep (explicit)**: Independent modifications, creating variants, preserving original data

### Data Types

```cpp
enum data_type {
    UChar,     // unsigned char (8-bit)   - range [0, 255]
    UShort,    // unsigned short (16-bit) - range [0, 65535]
    UInt,      // unsigned int (32-bit)   - range [0, 4294967295]
    Float,     // float (32-bit)          - IEEE 754 single precision
    Double,    // double (64-bit)         - IEEE 754 double precision
    UInt64     // uint64 (64-bit)         - range [0, 2^64-1]
};

// Type sizes in bytes
uint64 data_type_sizes[] = { 1, 2, 4, 4, 8, 8 };

// Type names
const char* data_type_strings[] = {
    "unsigned char", "unsigned short", "unsigned int",
    "float", "double", "uint64"
};

// Access type info
voxels v(dimension(10, 10, 10), Float);
uint64 bytes_per_voxel = v.voxelSize();  // 4
const char* type_name = v.voxelTypeStr();  // "float"
```

**Type Conversion:**

Voxel values are always accessed as `double`, with automatic conversion:

```cpp
voxels v(dimension(10, 10, 10), UChar);
v(5, 5, 5, 255.0);        // Stored as UChar: 255
double val = v(5, 5, 5);  // Read as double: 255.0

// Changing voxel type converts all data
v.voxelType(Float);       // Converts all 255 -> 255.0f
```

### Coordinate Systems

**Voxel Coordinates (Integer Grid):**
```
Voxels: (i, j, k) where 0 <= i < XDim(), 0 <= j < YDim(), 0 <= k < ZDim()
```

**Object Coordinates (Spatial):**
```
Object space: (x, y, z) in bounding box [minx, maxx] × [miny, maxy] × [minz, maxz]
```

**Coordinate Transformation:**

```cpp
volume vol(dimension(100, 100, 100), Float, 
           bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

// Voxel spacing (span between adjacent voxels)
double x_span = vol.XSpan();  // (10.0 - 0.0) / (100 - 1) ≈ 0.101
double y_span = vol.YSpan();  // (10.0 - 0.0) / (100 - 1) ≈ 0.101
double z_span = vol.ZSpan();  // (10.0 - 0.0) / (100 - 1) ≈ 0.101

// Voxel (i, j, k) -> Object (x, y, z)
x = vol.XMin() + i * vol.XSpan();
y = vol.YMin() + j * vol.YSpan();
z = vol.ZMin() + k * vol.ZSpan();

// Object (x, y, z) -> Voxel (i, j, k) [approximate]
i = round((x - vol.XMin()) / vol.XSpan());
j = round((y - vol.YMin()) / vol.YSpan());
k = round((z - vol.ZMin()) / vol.ZSpan());
```

**Special Case - Single Voxel Dimension:**
```cpp
volume vol(dimension(1, 100, 100), Float, bbox);
vol.XSpan();  // Returns 0.0 (no spacing with single voxel)
```

## Voxels Class

### Voxels Construction

```cpp
// Default: 4×4×4 UChar volume
voxels v1;

// Specify dimensions and type
voxels v2(dimension(128, 128, 128), Float);

// From existing data (copies data into shared_array)
float data[64 * 64 * 64];
voxels v3(data, dimension(64, 64, 64), Float);

// Copy constructor (shallow - shares data)
voxels v4(v3);

// Destructor automatically manages shared_array
```

### Dimensions and Types

```cpp
voxels v(dimension(100, 200, 300), Float);

// Get dimensions
dimension& dim = v.voxel_dimensions();
uint64 nx = v.XDim();  // 100
uint64 ny = v.YDim();  // 200
uint64 nz = v.ZDim();  // 300
uint64 total = dim.size();  // 100 * 200 * 300 = 6,000,000

// Change dimensions (reallocates data)
v.voxel_dimensions(dimension(128, 128, 128));

// Get/set voxel type
data_type type = v.voxelType();  // Float
v.voxelType(Double);             // Converts all data to double

// Type information
uint64 bytes = v.voxelSize();         // 8 (for Double)
const char* name = v.voxelTypeStr();  // "double"
```

### Data Access

**3D Indexing:**

```cpp
voxels v(dimension(100, 100, 100), Float);

// Write voxel
v(10, 20, 30, 42.5);

// Read voxel
double value = v(10, 20, 30);  // 42.5

// Out of bounds throws exception
try {
    v(200, 0, 0, 1.0);  // Throws index_out_of_bounds
} catch (index_out_of_bounds& e) {
    // Handle error
}
```

**Linear Indexing:**

```cpp
// Linear index i = x + y*XDim + z*XDim*YDim
voxels v(dimension(10, 10, 10), Float);

// Write using linear index
v(542, 99.9);  // Sets voxel at position (2, 4, 5)

// Read using linear index
double val = v(542);  // 99.9

// Convert between 3D and linear
uint64 i = 2, j = 4, k = 5;
uint64 linear = i + j * v.XDim() + k * v.XDim() * v.YDim();  // 542
```

**Raw Data Access:**

```cpp
voxels v(dimension(64, 64, 64), UChar);

// Get raw data pointer (triggers copy-on-write)
unsigned char* data = *v;
data[0] = 255;

// Const access (no copy-on-write)
const unsigned char* const_data = *v;

// Type-specific access
voxels vf(dimension(64, 64, 64), Float);
float* float_data = reinterpret_cast<float*>(*vf);

// Legacy shared_array access
boost::shared_array<unsigned char> shared = v.data_as_shared_array();
```

### Copy Operations

**Shallow Copy (Default):**

```cpp
voxels v1(dimension(64, 64, 64), Float);
v1.fill(42.0);

// All share same underlying data
voxels v2(v1);           // Copy constructor
voxels v3 = v1;          // Assignment
voxels v4;
v4.copy(v1);             // copy() default
v4.copy(v1, false);      // copy() explicit shallow

// Modify one, affects all (until COW triggers)
v2(10, 10, 10, 99.0);
```

**Deep Copy (Independent Data):**

```cpp
voxels v1(dimension(64, 64, 64), Float);
v1.fill(42.0);

// Method 1: copy() with deep flag
voxels v2;
v2.copy(v1, true);       // Creates independent copy

// Method 2: copy() self (deep copy idiom)
voxels v3(v1);
v3.copy();               // Turns itself into a deep copy

// Now modifications are independent
v2(10, 10, 10, 99.0);    // Only v2 changes
```

**Equality Testing:**

```cpp
voxels v1(dimension(10, 10, 10), Float);
voxels v2(v1);  // Shares data
voxels v3;
v3.copy(v1, true);  // Deep copy

// Equality compares dimensions, type, and data
bool same1 = (v1 == v2);  // true (same dimensions, type, shared data)
bool same2 = (v1 == v3);  // true (same dimensions, type, data values)
bool diff = (v1 != v3);   // false
```

### Subvolume Extraction

```cpp
voxels v(dimension(100, 100, 100), Float);
// Fill with data...

// Extract subvolume (creates new independent data)
// Offset (10, 20, 30), size (50×50×50)
v.sub(10, 20, 30, dimension(50, 50, 50));

// Now v is 50×50×50 containing original data [10:60, 20:70, 30:80]
EXPECT_EQ(v.XDim(), 50);
EXPECT_EQ(v.YDim(), 50);
EXPECT_EQ(v.ZDim(), 50);

// Out of bounds throws exception
try {
    v.sub(90, 90, 90, dimension(50, 50, 50));  // Goes beyond [100,100,100]
    // Throws sub_volume_out_of_bounds
} catch (sub_volume_out_of_bounds& e) {
    // Handle error
}
```

**Deep Copy Using sub():**

```cpp
voxels v1(dimension(64, 64, 64), Float);
v1.fill(42.0);

// Deep copy idiom: extract full volume
voxels v2(v1);
v2.sub(0, 0, 0, v1.voxel_dimensions());

// v2 is now independent
```

### Resizing and Interpolation

```cpp
voxels v(dimension(64, 64, 64), Float);
// Fill with gradient
for (uint64 k = 0; k < 64; ++k) {
    for (uint64 j = 0; j < 64; ++j) {
        for (uint64 i = 0; i < 64; ++i) {
            v(i, j, k, i + j + k);
        }
    }
}

// Resize using trilinear interpolation
v.resize(dimension(128, 128, 128));

// Now v is 128³ with interpolated values
EXPECT_EQ(v.XDim(), 128);

// Resize can upsample or downsample
v.resize(dimension(32, 32, 32));  // Downsample
```

**CUDA-Accelerated Resize:**

If built with CUDA support and CUDA is enabled:
```cpp
#ifdef CVC_USING_CUDA
v.enableCUDA();  // Enable GPU acceleration
v.resize(dimension(256, 256, 256));  // GPU trilinear interpolation
#endif
```

### Data Manipulation

**Fill with Constant:**

```cpp
voxels v(dimension(100, 100, 100), Float);

// Fill entire volume
v.fill(42.0);

// Fill subvolume
// Offset (10, 10, 10), size (20×20×20)
v.fillsub(10, 10, 10, dimension(20, 20, 20), 99.0);
```

**Map Value Range:**

```cpp
voxels v(dimension(100, 100, 100), Float);
// v has values in range [50.0, 200.0]

// Map to [0.0, 1.0]
v.map(0.0, 1.0);

// Now all values scaled: (val - min) / (max - min) * (new_max - new_min) + new_min
```

### Image Processing Filters

**Bilateral Filter:**

Smooths while preserving edges.

```cpp
voxels v(dimension(128, 128, 128), Float);
// Load noisy data...

// Apply bilateral filter
// radiometricSigma: intensity difference tolerance (default 200.0)
// spatialSigma: spatial distance weight (default 1.5)
// filterRadius: neighborhood size in voxels (default 2)
v.bilateralFilter(200.0, 1.5, 2);
```

**Contrast Enhancement:**

Enhances contrast using anisotropic diffusion resistor network.

```cpp
voxels v(dimension(128, 128, 128), Float);

// Enhance contrast
// resistor: value in [0.0, 1.0] controlling enhancement strength (default 0.95)
v.contrastEnhancement(0.95);

// WARNING: Requires memory for original + 6× volume size in Float
```

**Anisotropic Diffusion:**

Noise reduction while preserving edges better than bilateral filter.

```cpp
voxels v(dimension(128, 128, 128), Float);

// Apply anisotropic diffusion
// iterations: number of diffusion steps (default 20)
v.anisotropicDiffusion(20);
```

**GDTV Filter:**

Gradient-Domain Total Variation filter for noise reduction and feature preservation.

```cpp
voxels v(dimension(128, 128, 128), Float);

// Apply GDTV filter
// parameterq: regularization parameter (default 1.0)
// lambda: fidelity weight (default 0.1)
// iteration: number of iterations (default 50)
// neighbour: neighborhood type - 1 for 6-neighbor, 2 for 26-neighbor (default 1)
v.gdtvFilter(1.0, 0.1, 50, 1);
```

**Algorithm:**
- Computes gradient magnitude using finite differences
- Applies phi function: φ(x) = (2-q) * x^(1-q)
- Iterative weighted averaging based on gradient ratios
- Uses Jacobi iteration pattern (fully parallelizable)

**CUDA Acceleration:**

GDTV filtering is one of the **best candidates for GPU acceleration** due to its iterative nature:

```cpp
#ifdef CVC_USING_CUDA
voxels v(dimension(128, 128, 128), Float);
// Load data...

if (voxels::cuda_available()) {
    v.enableCUDA();
    
    // GPU-accelerated GDTV filter
    v.gdtvFilter(1.0, 0.1, 20, 1);
    // Both gradient computation and filtering run on GPU
}
#endif
```

**CUDA Performance Gains:**

| Volume Size | Iterations | CPU Time | GPU Time | Speedup |
|-------------|------------|----------|----------|----------|
| 32³         | 5          | 230ms    | 13ms     | **18x**  |
| 64³         | 5          | 917ms    | 15ms     | **61x**  |
| 64³         | 20         | 3056ms   | 36ms     | **85x**  |
| 128³        | 10         | 18741ms  | 218ms    | **86x**  |

**Why GDTV Shows Excellent Speedup:**
1. Both gradient and filtering phases are GPU-accelerated
2. Each iteration is fully parallelizable (Jacobi pattern)
3. More iterations = higher speedup (work multiplies, overhead doesn't)
4. Thousands of independent voxel computations per iteration
5. Memory-bound operation benefits from GPU memory bandwidth

**Numerical Accuracy:** GPU and CPU results agree within 3×10⁻⁵ (effectively identical).

### Statistics

**Min/Max Values:**

```cpp
voxels v(dimension(100, 100, 100), Float);
// Fill with data in range [10.0, 90.0]

// Auto-calculate min/max (lazy evaluation)
double min_val = v.min();  // 10.0
double max_val = v.max();  // 90.0

// Manual set (overrides auto-calculation)
v.min(0.0);
v.max(100.0);

// Check if set
bool min_set = v.minIsSet();  // true
bool max_set = v.maxIsSet();  // true

// Clear manual values (next access recalculates)
v.unsetMinMax();

// Subvolume min/max
double sub_min = v.min(10, 10, 10, dimension(50, 50, 50));
double sub_max = v.max(10, 10, 10, dimension(50, 50, 50));
```

**Histogram:**

```cpp
voxels v(dimension(128, 128, 128), Float);
// Fill with data...

// Calculate histogram with 256 bins
auto hist_data = v.histogram(256);
const uint64* bins = hist_data.get<0>();  // Array of bin counts
uint64 num_bins = hist_data.get<1>();     // 256

// Access bin counts
for (uint64 i = 0; i < num_bins; ++i) {
    std::cout << "Bin " << i << ": " << bins[i] << " voxels\n";
}
```

### Compositing

Combine two voxel volumes using a composite function.

```cpp
// Define composite function
class AddComposite : public composite_function {
public:
    virtual double operator()(double base, double overlay) const {
        return base + overlay;
    }
};

voxels v1(dimension(100, 100, 100), Float);
voxels v2(dimension(50, 50, 50), Float);
v1.fill(10.0);
v2.fill(5.0);

// Composite v2 into v1 at offset (25, 25, 25)
AddComposite add_func;
v1.composite(v2, 25, 25, 25, add_func);

// Now v1[25:75, 25:75, 25:75] contains 15.0 (10 + 5)

// Negative offsets supported (only overlapping region composited)
v1.composite(v2, -10, -10, -10, add_func);
```

**Built-in Composite Functions:**

See `composite_function` class hierarchy for predefined operations:
- Addition
- Maximum
- Minimum
- Alpha blending
- Overwrite

### CUDA Acceleration

**Enable CUDA:**

```cpp
#ifdef CVC_USING_CUDA
voxels v(dimension(256, 256, 256), Float);

// Check CUDA availability
if (voxels::cuda_available()) {
    // Get GPU info
    int num_gpus = voxels::cuda_device_count();
    auto gpu_info = voxels::get_gpu_info();
    
    for (const auto& info : gpu_info) {
        std::cout << "GPU " << info.device_id << ": " 
                  << info.name << " (" << info.memory_mb << " MB)\n";
    }
    
    // Enable CUDA on GPU 0
    v.enableCUDA(0);
    
    // Operations now GPU-accelerated
    v.resize(dimension(512, 512, 512));  // GPU trilinear interpolation
}
#endif
```

**GPU Management:**

```cpp
#ifdef CVC_USING_CUDA
voxels v;
v.enableCUDA(0);  // Enable on GPU 0

// Check CUDA state
bool using_gpu = v.using_cuda();  // true
int gpu_id = v.cuda_device();     // 0

// Switch to different GPU (peer-to-peer copy)
v.switchGPU(1);

// Disable CUDA (migrates data back to CPU)
v.disableCUDA();
#endif
```

**Best CUDA-Accelerated Operations:**

| Operation | Speedup | Notes |
|-----------|---------|-------|
| GDTV Filter | **18-86x** | Best candidate - iterative, fully parallel |
| Bilateral Filter | **17-25x** | Excellent for edge-preserving smoothing |
| Trilinear Resize | **10-100x** | Scales with output size |
| Min/Max Calculation | **5-15x** | Simple reduction operation |

**Note:** Contrast enhancement is NOT CUDA-accelerated because 2/3 of the algorithm (propagation) is inherently sequential.

**Unified Memory:**

When CUDA is enabled, voxels uses CUDA unified memory:
- Automatic migration between CPU and GPU
- Reference counted via `std::shared_ptr` with `CudaManagedDeleter`
- Transparent access from both CPU and GPU code

## Volume Class

The `volume` class extends `voxels` with spatial coordinate system.

### Volume Construction

```cpp
// Default: 4×4×4 UChar in [-0.5, 0.5]³
volume vol1;

// Custom dimensions, type, and bounding box
dimension dim(128, 128, 128);
bounding_box bbox(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
volume vol2(dim, Float, bbox);

// From voxels + bounding box
voxels vox(dimension(64, 64, 64), Float);
volume vol3(vox, bbox);

// Copy constructor (shallow - shares data)
volume vol4(vol3);

// From file
volume vol5("data.rawiv");
```

### Bounding Box Operations

```cpp
volume vol(dimension(100, 100, 100), Float,
           bounding_box(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0));

// Get/set bounding box
bounding_box& bbox = vol.boundingBox();
vol.boundingBox(bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

// Access bounds
double xmin = vol.XMin();  // -5.0
double xmax = vol.XMax();  //  5.0
double ymin = vol.YMin();  // -5.0
double ymax = vol.YMax();  //  5.0
double zmin = vol.ZMin();  // -5.0
double zmax = vol.ZMax();  //  5.0

// Voxel spacing
double x_span = vol.XSpan();  // (5 - (-5)) / (100 - 1) ≈ 0.101
double y_span = vol.YSpan();  // (5 - (-5)) / (100 - 1) ≈ 0.101
double z_span = vol.ZSpan();  // (5 - (-5)) / (100 - 1) ≈ 0.101
```

### Spatial Interpolation

**Trilinear Interpolation:**

```cpp
volume vol(dimension(100, 100, 100), Float,
           bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

// Fill with gradient
for (uint64 k = 0; k < vol.ZDim(); ++k) {
    for (uint64 j = 0; j < vol.YDim(); ++j) {
        for (uint64 i = 0; i < vol.XDim(); ++i) {
            double x = vol.XMin() + i * vol.XSpan();
            double y = vol.YMin() + j * vol.YSpan();
            double z = vol.ZMin() + k * vol.ZSpan();
            vol(i, j, k, x + y + z);
        }
    }
}

// Interpolate at arbitrary spatial coordinates
double val1 = vol.interpolate(5.0, 5.0, 5.0);     // 15.0 (center)
double val2 = vol.interpolate(2.5, 3.7, 8.1);     // Interpolated value
double val3 = vol.interpolate(0.0, 0.0, 0.0);     // 0.0 (corner)

// Out of bounds throws exception
try {
    vol.interpolate(15.0, 5.0, 5.0);  // Outside [0, 10]
    // Throws invalid_bounding_box or similar
} catch (...) {
    // Handle error
}
```

**Interpolation at Voxel Centers:**

```cpp
// Voxel (i, j, k) is centered at object coordinates:
double x = vol.XMin() + i * vol.XSpan();
double y = vol.YMin() + j * vol.YSpan();
double z = vol.ZMin() + k * vol.ZSpan();

// Interpolation at voxel center matches voxel value
double voxel_val = vol(i, j, k);
double interp_val = vol.interpolate(x, y, z);
// voxel_val ≈ interp_val (may differ slightly due to FP precision)
```

### Subvolume by Bounding Box

**Extract Subvolume by Spatial Bounds:**

```cpp
volume vol(dimension(200, 200, 200), Float,
           bounding_box(0.0, 0.0, 0.0, 20.0, 20.0, 20.0));

// Extract subvolume in spatial region [5, 15]³
bounding_box sub_bbox(5.0, 5.0, 5.0, 15.0, 15.0, 15.0);
vol.sub(sub_bbox);

// Volume now covers [5, 15]³ with original span preserved
EXPECT_EQ(vol.boundingBox(), sub_bbox);
// Dimensions adjusted to maintain span
```

**Extract with Custom Resolution:**

```cpp
volume vol(dimension(200, 200, 200), Float,
           bounding_box(0.0, 0.0, 0.0, 20.0, 20.0, 20.0));

// Extract region [5, 15]³ at 64³ resolution
bounding_box sub_bbox(5.0, 5.0, 5.0, 15.0, 15.0, 15.0);
dimension sub_dim(64, 64, 64);
vol.sub(sub_bbox, sub_dim);

// Volume is now 64³ covering [5, 15]³
EXPECT_EQ(vol.XDim(), 64);
EXPECT_EQ(vol.boundingBox(), sub_bbox);
```

**Extract by Voxel Offset:**

```cpp
// Volume inherits voxels::sub() for voxel-space extraction
volume vol(dimension(100, 100, 100), Float,
           bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

// Extract 50³ region starting at voxel (25, 25, 25)
vol.sub(25, 25, 25, dimension(50, 50, 50));

// Bounding box automatically adjusted to match extracted region
```

### Volume Combination

**Combine Two Volumes:**

```cpp
volume vol1(dimension(100, 100, 100), Float,
            bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

volume vol2(dimension(50, 50, 50), Float,
            bounding_box(8.0, 8.0, 8.0, 12.0, 12.0, 12.0));

// Combine: vol1 expands to contain both bounding boxes
vol1.combineWith(vol2);

// vol1 now covers [0, 12]×[0, 12]×[0, 12]
EXPECT_DOUBLE_EQ(vol1.XMin(), 0.0);
EXPECT_DOUBLE_EQ(vol1.XMax(), 12.0);
```

**Combine with Custom Dimensions:**

```cpp
volume vol1(dimension(100, 100, 100), Float,
            bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));

volume vol2(dimension(50, 50, 50), Float,
            bounding_box(8.0, 8.0, 8.0, 12.0, 12.0, 12.0));

// Combine at specific resolution
dimension combined_dim(256, 256, 256);
vol1.combineWith(vol2, combined_dim);

// vol1 is now 256³ covering [0, 12]³
EXPECT_EQ(vol1.XDim(), 256);
```

### File I/O

**Read Volume from File:**

```cpp
// Read entire volume
volume vol("data.rawiv");

// Read specific variable and timestep (for multi-var/time formats)
volume vol2;
vol2.read("simulation.h5", 0, 0);  // var=0, time=0

// Read subvolume by bounding box
bounding_box region(5.0, 5.0, 5.0, 15.0, 15.0, 15.0);
volume vol3;
vol3.read("data.rawiv", 0, 0, region);
```

**Write Volume to File:**

```cpp
volume vol(dimension(128, 128, 128), Float,
           bounding_box(0.0, 0.0, 0.0, 10.0, 10.0, 10.0));
vol.desc("Simulation Output");
// Fill with data...

// Write (format determined by extension)
vol.write("output.rawiv");   // RAWIV format
vol.write("output.mrc");     // MRC format
vol.write("output.h5");      // HDF5 format
```

**Supported Formats:**

- **RAWIV**: Simple binary format with header
- **MRC**: Medical Research Council format
- **HDF5**: Hierarchical Data Format 5
- Others depending on build configuration

**Volume Description:**

```cpp
volume vol;
vol.desc("Electron Density Map");  // Set description
std::string desc = vol.desc();      // Get description

// Description saved with volume (format-dependent)
vol.write("output.mrc");  // MRC stores description
```

## Supporting Types

### dimension

```cpp
#include <cvc/dimension.h>

// 3D dimensions
dimension dim(100, 200, 300);

// Access
uint64 x = dim.xdim;  // 100
uint64 y = dim.ydim;  // 200
uint64 z = dim.zdim;  // 300

// Total size
uint64 total = dim.size();  // 100 * 200 * 300 = 6,000,000

// Equality
dimension dim2(100, 200, 300);
bool same = (dim == dim2);  // true
bool diff = (dim != dim2);  // false
```

### bounding_box

```cpp
#include <cvc/bounding_box.h>

// Generic bounding box (usually double)
typedef generic_bounding_box<double> bounding_box;

// Construction
bounding_box bbox1;  // Default: [0,0,0,0,0,0]
bounding_box bbox2(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);

// Access
double xmin = bbox2.minx;  // 0.0
double xmax = bbox2.maxx;  // 10.0
double ymin = bbox2.miny;  // 0.0
double ymax = bbox2.maxy;  // 10.0
double zmin = bbox2.minz;  // 0.0
double zmax = bbox2.maxz;  // 10.0

// Array access
double min_x = bbox2.min_[0];  // 0.0
double max_z = bbox2.max_[2];  // 10.0

// Set bounds
bbox2.setMin(1.0, 2.0, 3.0);
bbox2.setMax(11.0, 12.0, 13.0);

// Equality
bounding_box bbox3(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
bool same = (bbox2 == bbox3);  // false (different bounds)

// Validity checking
bbox2.checkBounds();  // Throws if min > max
```

### data_type

```cpp
// Enumeration of supported voxel types
enum data_type {
    UChar,   // unsigned char (8-bit)
    UShort,  // unsigned short (16-bit)
    UInt,    // unsigned int (32-bit)
    Float,   // float (32-bit)
    Double,  // double (64-bit)
    UInt64   // uint64 (64-bit)
};

// Type information arrays
extern uint64 data_type_sizes[6];        // Bytes per voxel
extern const char* data_type_strings[6]; // Type names

// Usage
data_type dt = Float;
uint64 bytes = data_type_sizes[dt];       // 4
const char* name = data_type_strings[dt]; // "float"
```

### composite_function

```cpp
#include <cvc/composite_function.h>

// Abstract base class for voxel compositing
class composite_function {
public:
    virtual ~composite_function() {}
    
    // Override to define composite operation
    // base: existing voxel value
    // overlay: value being composited
    // Returns: new voxel value
    virtual double operator()(double base, double overlay) const = 0;
};

// Example: Addition composite
class AddComposite : public composite_function {
public:
    virtual double operator()(double base, double overlay) const {
        return base + overlay;
    }
};

// Example: Maximum composite
class MaxComposite : public composite_function {
public:
    virtual double operator()(double base, double overlay) const {
        return std::max(base, overlay);
    }
};

// Example: Alpha blending
class AlphaComposite : public composite_function {
    double _alpha;
public:
    AlphaComposite(double alpha) : _alpha(alpha) {}
    
    virtual double operator()(double base, double overlay) const {
        return (1.0 - _alpha) * base + _alpha * overlay;
    }
};

// Usage
voxels v1, v2;
// ...
AlphaComposite blend(0.5);
v1.composite(v2, 0, 0, 0, blend);
```

## Complete Examples

### Example 1: Load, Filter, and Save

```cpp
#include <cvc/volume.h>

using namespace CVC_NAMESPACE;

int main() {
    try {
        // Load volume from file
        volume vol("input.rawiv");
        
        std::cout << "Loaded " << vol.XDim() << "×" 
                  << vol.YDim() << "×" << vol.ZDim() 
                  << " volume\n";
        
        // Apply bilateral filter for noise reduction
        vol.bilateralFilter(200.0, 1.5, 2);
        
        // Enhance contrast
        vol.contrastEnhancement(0.95);
        
        // Map to [0, 255] range
        vol.map(0.0, 255.0);
        
        // Convert to UChar for compact storage
        vol.voxelType(UChar);
        
        // Save result
        vol.desc("Filtered Volume");
        vol.write("output.rawiv");
        
        std::cout << "Filtered volume saved\n";
        
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
```

### Example 2: Generate Synthetic Volume

```cpp
#include <cvc/volume.h>
#include <cmath>

using namespace CVC_NAMESPACE;

volume generateSphere(double radius, uint64 resolution) {
    dimension dim(resolution, resolution, resolution);
    bounding_box bbox(-radius, -radius, -radius, radius, radius, radius);
    volume vol(dim, Float, bbox);
    
    // Generate sphere: distance from center
    for (uint64 k = 0; k < vol.ZDim(); ++k) {
        for (uint64 j = 0; j < vol.YDim(); ++j) {
            for (uint64 i = 0; i < vol.XDim(); ++i) {
                double x = vol.XMin() + i * vol.XSpan();
                double y = vol.YMin() + j * vol.YSpan();
                double z = vol.ZMin() + k * vol.ZSpan();
                
                double dist = std::sqrt(x*x + y*y + z*z);
                
                // Smooth sphere with falloff
                double value = 0.0;
                if (dist < radius) {
                    value = 255.0 * (1.0 - dist / radius);
                }
                
                vol(i, j, k, value);
            }
        }
    }
    
    return vol;
}

int main() {
    volume sphere = generateSphere(5.0, 128);
    sphere.desc("Synthetic Sphere");
    sphere.write("sphere.rawiv");
    return 0;
}
```

### Example 3: Subvolume Extraction and Analysis

```cpp
#include <cvc/volume.h>
#include <iostream>

using namespace CVC_NAMESPACE;

void analyzeRegion(const volume& vol, const bounding_box& region) {
    // Extract subvolume
    volume subvol(vol);
    subvol.sub(region);
    
    // Calculate statistics
    double min_val = subvol.min();
    double max_val = subvol.max();
    
    std::cout << "Region " << region << ":\n";
    std::cout << "  Dimensions: " << subvol.XDim() << "×" 
              << subvol.YDim() << "×" << subvol.ZDim() << "\n";
    std::cout << "  Min value: " << min_val << "\n";
    std::cout << "  Max value: " << max_val << "\n";
    std::cout << "  Range: " << (max_val - min_val) << "\n";
    
    // Histogram
    auto hist = subvol.histogram(256);
    const uint64* bins = hist.get<0>();
    uint64 num_bins = hist.get<1>();
    
    // Find peak bin
    uint64 max_bin = 0;
    uint64 max_count = 0;
    for (uint64 i = 0; i < num_bins; ++i) {
        if (bins[i] > max_count) {
            max_count = bins[i];
            max_bin = i;
        }
    }
    
    double peak_value = min_val + max_bin * (max_val - min_val) / num_bins;
    std::cout << "  Peak value: " << peak_value 
              << " (" << max_count << " voxels)\n";
}

int main() {
    volume vol("data.rawiv");
    
    // Analyze several regions
    analyzeRegion(vol, bounding_box(0, 0, 0, 5, 5, 5));
    analyzeRegion(vol, bounding_box(5, 5, 5, 10, 10, 10));
    
    return 0;
}
```

### Example 4: Volume Interpolation and Resampling

```cpp
#include <cvc/volume.h>
#include <vector>

using namespace CVC_NAMESPACE;

// Sample volume along a line
std::vector<double> sampleLine(const volume& vol,
                                double x0, double y0, double z0,
                                double x1, double y1, double z1,
                                uint64 num_samples) {
    std::vector<double> samples;
    samples.reserve(num_samples);
    
    for (uint64 i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double x = x0 + t * (x1 - x0);
        double y = y0 + t * (y1 - y0);
        double z = z0 + t * (z1 - z0);
        
        try {
            samples.push_back(vol.interpolate(x, y, z));
        } catch (...) {
            samples.push_back(0.0);  // Outside bounds
        }
    }
    
    return samples;
}

// Resample volume to different resolution
volume resampleVolume(const volume& vol, const dimension& new_dim) {
    volume resampled(new_dim, vol.voxelType(), vol.boundingBox());
    
    // Interpolate at new voxel centers
    for (uint64 k = 0; k < new_dim.zdim; ++k) {
        for (uint64 j = 0; j < new_dim.ydim; ++j) {
            for (uint64 i = 0; i < new_dim.xdim; ++i) {
                double x = resampled.XMin() + i * resampled.XSpan();
                double y = resampled.YMin() + j * resampled.YSpan();
                double z = resampled.ZMin() + k * resampled.ZSpan();
                
                double value = vol.interpolate(x, y, z);
                resampled(i, j, k, value);
            }
        }
    }
    
    return resampled;
}

int main() {
    volume vol("input.rawiv");
    
    // Sample diagonal line
    auto samples = sampleLine(vol,
                              vol.XMin(), vol.YMin(), vol.ZMin(),
                              vol.XMax(), vol.YMax(), vol.ZMax(),
                              100);
    
    // Resample to higher resolution
    volume hires = resampleVolume(vol, dimension(256, 256, 256));
    hires.write("hires.rawiv");
    
    return 0;
}
```

### Example 5: CUDA-Accelerated Processing

```cpp
#include <cvc/volume.h>
#include <iostream>

using namespace CVC_NAMESPACE;

int main() {
#ifdef CVC_USING_CUDA
    // Check GPU availability
    if (!voxels::cuda_available()) {
        std::cerr << "CUDA not available\n";
        return 1;
    }
    
    // List GPUs
    auto gpu_info = voxels::get_gpu_info();
    std::cout << "Available GPUs:\n";
    for (const auto& info : gpu_info) {
        std::cout << "  GPU " << info.device_id << ": " 
                  << info.name << " (" << info.memory_mb << " MB)\n";
    }
    
    // Load large volume
    volume vol("large_data.rawiv");
    
    // Enable GPU acceleration on best GPU
    vol.enableCUDA(0);
    std::cout << "Using GPU " << vol.cuda_device() << "\n";
    
    // GPU-accelerated resize (trilinear interpolation)
    auto start = std::chrono::high_resolution_clock::now();
    vol.resize(dimension(512, 512, 512));
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Resize took " << duration.count() << " ms on GPU\n";
    
    // Disable CUDA (migrates back to CPU)
    vol.disableCUDA();
    
    vol.write("resized.rawiv");
    
#else
    std::cout << "Built without CUDA support\n";
#endif
    
    return 0;
}
```

## Best Practices

### Memory Management

**1. Understand Shallow vs Deep Copy:**

```cpp
// WRONG: Expecting independent copies
volume vol1(dimension(128, 128, 128), Float);
volume vol2(vol1);  // Shares data!
vol2(10, 10, 10, 42.0);  // Triggers COW, NOW independent

// CORRECT: Explicit deep copy when needed
volume vol1(dimension(128, 128, 128), Float);
volume vol2;
vol2.copy(vol1, true);  // Independent from start
vol2(10, 10, 10, 42.0);  // Only vol2 modified
```

**2. Use Shallow Copy for Read-Only Access:**

```cpp
// Efficient: no data duplication
void analyzeVolume(const volume& vol) {
    double min_val = vol.min();
    double max_val = vol.max();
    // Read-only operations - shallow copy is perfect
}

volume vol("data.rawiv");
analyzeVolume(vol);  // No copy overhead
```

**3. Use Deep Copy for Independent Modifications:**

```cpp
// Create variants
volume original("data.rawiv");

volume filtered;
filtered.copy(original, true);
filtered.bilateralFilter();

volume enhanced;
enhanced.copy(original, true);
enhanced.contrastEnhancement();

// Now have 3 independent volumes
```

### Performance

**1. Prefer Batch Operations:**

```cpp
// WRONG: Individual voxel operations
for (uint64 i = 0; i < vol.voxel_dimensions().size(); ++i) {
    vol(i, vol(i) * 2.0);  // Slow
}

// CORRECT: Use map()
vol.map(vol.min() * 2.0, vol.max() * 2.0);

// OR: Raw data access for custom operations
float* data = reinterpret_cast<float*>(*vol);
uint64 size = vol.voxel_dimensions().size();
for (uint64 i = 0; i < size; ++i) {
    data[i] *= 2.0;
}
vol.unsetMinMax();  // Invalidate cached min/max
```

**2. Use CUDA for Large Volumes:**

```cpp
#ifdef CVC_USING_CUDA
if (vol.voxel_dimensions().size() > 1000000 && voxels::cuda_available()) {
    vol.enableCUDA();  // GPU acceleration for large data
    vol.resize(dimension(512, 512, 512));  // Much faster on GPU
    vol.disableCUDA();  // Back to CPU when done
}
#endif
```

**3. Minimize Type Conversions:**

```cpp
// WRONG: Type conversion overhead
voxels v(dimension(256, 256, 256), UChar);
// ... process ...
v.voxelType(Float);   // Converts all 16M voxels
// ... more processing ...
v.voxelType(Double);  // Converts all 16M voxels again

// CORRECT: Choose appropriate type from start
voxels v(dimension(256, 256, 256), Float);
// ... all processing at Float precision ...
v.voxelType(UChar);   // Single conversion at end for storage
```

### Data Integrity

**1. Validate Bounds:**

```cpp
// Always check interpolation bounds
if (x >= vol.XMin() && x <= vol.XMax() &&
    y >= vol.YMin() && y <= vol.YMax() &&
    z >= vol.ZMin() && z <= vol.ZMax()) {
    double value = vol.interpolate(x, y, z);
} else {
    // Handle out-of-bounds
}

// Or use try-catch
try {
    double value = vol.interpolate(x, y, z);
} catch (...) {
    // Out of bounds
}
```

**2. Preserve Metadata:**

```cpp
// When creating derived volumes, preserve description
volume original("data.rawiv");
volume processed;
processed.copy(original, true);
processed.bilateralFilter();

// Preserve or update description
processed.desc(original.desc() + " (filtered)");
processed.write("filtered.rawiv");
```

**3. Check Min/Max After Modifications:**

```cpp
volume vol("data.rawiv");
vol.unsetMinMax();  // Clear cached values

// Modify data
float* data = reinterpret_cast<float*>(*vol);
for (uint64 i = 0; i < vol.voxel_dimensions().size(); ++i) {
    data[i] = std::sqrt(data[i]);
}

// Recalculate statistics
double new_min = vol.min();  // Auto-calculates
double new_max = vol.max();
```

### File I/O

**1. Use Appropriate Formats:**

```cpp
// RAWIV: Simple, fast, no compression
vol.write("data.rawiv");

// HDF5: Compressed, multi-variable, metadata
vol.write("data.h5");

// MRC: Medical imaging standard
vol.write("data.mrc");
```

**2. Read Subvolumes for Large Files:**

```cpp
// Don't load entire 10GB volume
// volume vol("huge_data.rawiv");  // Memory exhausted!

// Load only region of interest
bounding_box roi(10, 10, 10, 20, 20, 20);
volume vol;
vol.read("huge_data.rawiv", 0, 0, roi);  // Only loads subvolume
```

**3. Set Descriptions:**

```cpp
volume vol(dimension(128, 128, 128), Float);
// ... generate data ...

vol.desc("Simulation: timestep=100, parameter=0.5");
vol.write("sim_t100.rawiv");  // Description saved in file
```

## Performance Considerations

### Memory Usage

```cpp
// Memory = XDim * YDim * ZDim * voxelSize()

dimension dim(512, 512, 512);
voxels v_uchar(dim, UChar);   // 512³ * 1 = 128 MB
voxels v_float(dim, Float);   // 512³ * 4 = 512 MB
voxels v_double(dim, Double); // 512³ * 8 = 1 GB
```

**Shared Memory:**

```cpp
// 10 shallow copies share single 512 MB allocation
volume vol1(dimension(512, 512, 512), Float);
volume copies[10];
for (int i = 0; i < 10; ++i) {
    copies[i] = vol1;  // Shallow copy
}
// Total memory: ~512 MB

// 10 deep copies require 5.12 GB
for (int i = 0; i < 10; ++i) {
    copies[i].copy(vol1, true);  // Deep copy
}
// Total memory: ~5.12 GB
```

### Copy-on-Write Overhead

```cpp
// Shallow copy is O(1)
volume vol1(dimension(512, 512, 512), Float);
volume vol2(vol1);  // Instant (just pointer copy)

// First write triggers COW: O(n) where n = volume size
vol2(10, 10, 10, 42.0);  // Copies all 512³ voxels first time

// Subsequent writes are O(1)
vol2(20, 20, 20, 99.0);  // No copy needed
```

### Resize Performance

**CPU Trilinear Resize:**
- O(n_out * 8) where n_out = output voxel count
- Each output voxel samples 8 neighbors

```cpp
// 64³ -> 512³: ~1 second CPU
// 512³ -> 1024³: ~1 minute CPU (8× more output voxels)
```

**GPU Trilinear Resize:**
- O(n_out) with massive parallelism
- 10-100× faster than CPU

```cpp
#ifdef CVC_USING_CUDA
vol.enableCUDA();
vol.resize(dimension(1024, 1024, 1024));  // ~seconds on GPU
#endif
```

### Filter Performance

**Bilateral Filter:**
- O(n * r³) where n = voxel count, r = filter radius
- Radius 2 → 5³ = 125 samples per voxel
- Very expensive for large volumes

```cpp
// 128³ volume, radius 2: ~10 seconds
// 256³ volume, radius 2: ~80 seconds (8× volume, 8× time)
```

**Optimization: Reduce Radius or Resolution:**

```cpp
// Option 1: Smaller radius
vol.bilateralFilter(200.0, 1.5, 1);  // Radius 1: 3³ = 27 samples

// Option 2: Downsample, filter, upsample
volume downsampled(vol);
downsampled.resize(dimension(64, 64, 64));  // Faster
downsampled.bilateralFilter(200.0, 1.5, 2);
downsampled.resize(vol.voxel_dimensions());  // Restore resolution
```

## Exception Handling

**Common Exceptions:**

```cpp
#include <cvc/exception.h>

try {
    volume vol(dimension(1000, 1000, 1000), Double);
} catch (memory_allocation_error& e) {
    // Insufficient memory for 8 GB volume
    std::cerr << "Out of memory: " << e.what() << "\n";
}

try {
    voxels v(dimension(100, 100, 100), Float);
    v(200, 0, 0, 1.0);  // Out of bounds
} catch (index_out_of_bounds& e) {
    std::cerr << "Index error: " << e.what() << "\n";
}

try {
    volume vol(dimension(100, 100, 100), Float,
               bounding_box(0, 0, 0, 10, 10, 10));
    vol.sub(90, 90, 90, dimension(50, 50, 50));  // Extends beyond volume
} catch (sub_volume_out_of_bounds& e) {
    std::cerr << "Subvolume error: " << e.what() << "\n";
}

try {
    bounding_box bbox(10, 10, 10, 0, 0, 0);  // min > max
    bbox.checkBounds();
} catch (invalid_bounding_box& e) {
    std::cerr << "Invalid bbox: " << e.what() << "\n";
}

try {
    volume vol;
    vol.read("nonexistent.rawiv");
} catch (std::exception& e) {
    std::cerr << "File error: " << e.what() << "\n";
}
```

**Exception-Safe Code:**

```cpp
// RAII ensures cleanup even if exceptions thrown
void processVolume(const std::string& filename) {
    volume vol(filename);  // Constructor may throw
    
    vol.bilateralFilter();  // May throw bad_alloc
    
    vol.write("output.rawiv");  // May throw file error
    
    // vol destructor called automatically even if exception thrown
}
```

---

*For additional documentation, see:*
- [APP_API.md](APP_API.md) - Application framework
- [STATE_API.md](STATE_API.md) - State tree management
- [SDF_API.md](SDF_API.md) - Signed distance field operations
- [CUDA_GUIDE.md](../CUDA_GUIDE.md) - CUDA acceleration details
- [IMAGE_PROCESSING_ALGORITHMS.md](IMAGE_PROCESSING_ALGORITHMS.md) - Filter algorithms
