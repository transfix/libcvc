# libcvc

[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-17%2B-orange.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-Check%20LICENSE-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-577%20passing-brightgreen.svg)](#testing)
[![Coverage](https://img.shields.io/badge/coverage-64.6%25%20lines%20%7C%2068.1%25%20functions-yellow.svg)](docs/TESTING.md)

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Quick Start](#quick-start)
  - [Prerequisites](#prerequisites)
  - [Installation (Ubuntu/Debian)](#installation-ubuntudebian)
  - [Build Instructions](#build-instructions)
  - [Quick Build Verification](#quick-build-verification)
- [Build Options](#build-options)
- [Testing](#testing)
- [Usage Example](#usage-example)
- [Supported File Formats](#supported-file-formats)
- [Documentation](#documentation)
- [Project Structure](#project-structure)
- [Version History](#version-history)
- [Contributing](#contributing)
- [Known Issues](#known-issues)
- [License](#license)
- [Credits](#credits)
- [References](#references)
- [Contact](#contact)

## Overview

A comprehensive computational visualization library from the Computational Visualization Center at UT Austin. libcvc provides the computational core functionality of the VolumeRover package, including volume processing, geometry manipulation, isosurfacing, and signed distance function calculations.

**Maintainer:** Joe Rivera - j@jriv.us

## Features

- 🎨 **Volume Processing**: Multiple volume file format support (RAWIV, MRC, Spider, HDF5, VTK)
- 🔺 **Geometry Handling**: Read/write various geometry formats (OFF, OBJ, RAW variants)
- 🎯 **Meshing & Isosurfacing**: Marching cubes, LBIE meshing, fast contouring
- 📐 **Signed Distance Functions v2.0**: Thread-safe, 11x faster, GPU-ready architecture
- 🌐 **Network Support**: Optional XMLRPC for distributed state sharing
- 🔬 **Image Filtering**: Bilateral filter, anisotropic diffusion, GDTV, contrast enhancement
- 🧮 **Scientific Computing**: Integration with FFTW, GSL, CGAL, Boost

## Quick Start

### Prerequisites

**Required:**
- CMake 3.15 or higher
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+); C++14/20/23 selectable via `-DCMAKE_CXX_STANDARD=`
- Boost libraries (>= 1.58): thread, date_time, regex, filesystem, system

**Optional:**
- HDF5 (for .cvc file format)
- CGAL (for advanced geometry operations)
- FFTW (for frequency domain operations)
- GSL (for scientific computing)
- Log4cplus (for advanced logging)

### Installation (Ubuntu/Debian)

```bash
# Install required dependencies
sudo apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev

# Install optional dependencies
sudo apt-get install -y \
    libhdf5-dev \
    libcgal-dev \
    libfftw3-dev \
    libgsl-dev \
    liblog4cplus-dev
```

### Build Instructions

```bash
# Clone or navigate to the repository
cd /path/to/libcvc

# Create and enter build directory
mkdir build && cd build

# Configure
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build (using all available cores)
cmake --build . -j$(nproc)

# Optional: Install system-wide
sudo cmake --install .
```

### Quick Build Verification

Use the provided script to verify your build environment:

```bash
./build_verify.sh
```

## Build Options

Common CMake configuration options:

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \          # Build type (Debug/Release/RelWithDebInfo)
  -DCMAKE_CXX_STANDARD=17 \             # C++ standard (14/17/20/23)
  -DBUILD_SHARED_LIBS=ON \              # Build shared libraries
  -DCVC_BUILD_TESTS=ON \                # Enable unit tests (ON by default)
  -DCVC_USING_HDF5=ON \                 # Enable HDF5 support
  -DCVC_ENABLE_MESHER=ON \              # Enable meshing features
  -DCVC_ENABLE_SDF=ON \                 # Enable SDF calculations
  -DCVC_USING_XMLRPC=OFF \              # Enable network sharing
  -DDISABLE_CGAL=OFF                    # Enable CGAL support
```

### Build Type Performance Impact

The build type significantly affects performance, especially for computationally intensive operations like SDF computation:

| Build Type | Optimization | Debug Info | Use Case | SDF 256³ Performance* |
|------------|-------------|------------|----------|----------------------|
| **Release** | `-O3 -DNDEBUG` | No | Production | ~220s (fastest) |
| **RelWithDebInfo** | `-O2 -g -DNDEBUG` | Yes | **Recommended** | ~234s (1.06x) |
| **Debug** | `-g` (no opt) | Yes | Development only | ~3027s (13.7x slower) |
| **Debug + Coverage** | `-g --coverage` | Yes | Coverage analysis | ~3030s (13.8x slower) |

*Tested with Stanford Bunny (34,834 triangles) at 256³ resolution

**Recommendations**:
- **Development/Testing**: Use `RelWithDebInfo` for reasonable performance with debugging capability
- **Production**: Use `Release` for maximum performance
- **Debugging**: Use `Debug` only when you need to step through unoptimized code
- **Coverage Analysis**: Use `Debug` with coverage flags, but expect 10-15x slower execution

The dramatic slowdown in Debug builds is due to:
- No compiler optimizations (loop unrolling, inlining, dead code elimination)
- Coverage instrumentation overhead (if enabled)
- Full bounds checking on all array accesses
- No constant propagation or strength reduction

**Note**: All build types produce identical, deterministic results. The difference is purely execution speed.

## Testing

libcvc includes comprehensive unit tests using Google Test. Tests are **enabled by default**.

### Test Suite: 577 Tests (100% Passing)

- **72 App Tests** — Core application framework, data/property management, threading
- **158 State Tests** — State tree, hierarchies, signals, async operations, futures API, state_object pattern
- **133 Voxels Tests** — Volume data operations, algorithms, CUDA GPU acceleration
- **29 Volume Tests** — Spatial coordinates, interpolation, subvolumes, bounding boxes
- **119 Geometry Tests** — Mesh operations, normals, I/O, SDF computation, isosurface extraction
- **42 Volume-Ops Tests** — Filtering, contrast enhancement, anisotropic diffusion, GDTV
- **24 HDF5 Tests** — .cvc file format round-trip and metadata
- **Procedural Geometry Tests** — Templated/parameterized fixtures over generated meshes

### Code Coverage: 90.5% on Core Components

The actively tested core modules (app, state, voxels, volume, geometry) achieve excellent coverage:

- **state.cpp**: 92.75% (243/262 lines)
- **voxels.cpp**: 92.17% (306/332 lines)  ⚡ Includes CUDA paths
- **volume.cpp**: 91.18% (124/136 lines)
- **app.cpp**: 89.82% (441/491 lines)
- **geometry.cpp**: 79.00% (252/319 lines)
- **Functions**: 94.7% (355/375 functions)

See [docs/TESTING.md](docs/TESTING.md) for comprehensive testing documentation including coverage analysis

### Quick Test Commands

```bash
# Build with tests (default)
cmake --build build

# Run all tests
cd build && ctest --output-on-failure

# Or use the convenience target
cmake --build build --target check

# Run specific test suites
./build/bin/app_test    # Test cvc::app functionality
./build/bin/state_test  # Test cvc::state functionality (includes futures & threading)

# Stress/Performance Tests
# By default, long-running stress tests are disabled for faster development
# To run them, use the --enable-stress-tests flag:
./build/bin/geometry_test --enable-stress-tests
./build/bin/state_test --enable-stress-tests

# See all stress tests with --help-stress:
./build/bin/geometry_test --help-stress
./build/bin/state_test --help-stress
```

### Advanced Features Tested

- ✅ **Multithreaded Operations** - Concurrent access with thread safety
- ✅ **Futures API** - Async value retrieval with blocking/callbacks
- ✅ **CUDA GPU Acceleration** - 17 comprehensive tests (device selection, memory migration, operations)
- ✅ **Spatial Interpolation** - Trilinear interpolation and gradients
- ✅ **Subvolume Operations** - Coordinate system transformations
- ✅ **Edge Cases** - Boundary conditions, tiny/large volumes
- ✅ **Memory Semantics** - Shallow copy by default (ref-counted CUDA), deep copy available

### Documentation

See **[docs/TESTING.md](docs/TESTING.md)** for comprehensive testing documentation:
- 577 tests with 100% pass rate (72 app, 158 state, 133 voxels, 29 volume, 119 geometry, 42 volume-ops, 24 hdf5, plus procedural-geometry typed fixtures)
- 90.5% coverage on core components
- How to run tests (CTest, Google Test)
- Test coverage details and analysis
- Multithreaded testing (12 tests)
- Futures API testing (11 tests)
- State object pattern tests (11 tests)
- CUDA GPU tests (20 tests)
- Adding new tests
- CI/CD integration
- Performance benchmarks
- Troubleshooting

See **[docs/STATE_API.md](docs/STATE_API.md)** for state system and futures API:
- Complete state tree documentation
- Async programming patterns with futures
- State object pattern (CRTP-based)
- Blocking waits for values
- Callbacks for state changes
- Producer-consumer examples
- Timeout handling

## Core APIs

### Volume Processing

```cpp
#include <cvc/volume.h>

// Load a volume file
cvc::volume vol("data.rawiv");

// Access voxel data
double value = vol(x, y, z);  // Trilinear interpolation

// Create subvolume with coordinate mapping
cvc::bounding_box region(x0, y0, z0, x1, y1, z1);
cvc::volume subvol = vol.sub(region);

// Apply image processing filters
vol.bilateral_filter(sigma_space, sigma_range);
vol.anisotropic_diffusion(iterations, k, lambda);
vol.contrast_enhancement(min_val, max_val);

// Save processed volume
vol.write("output.rawiv");

// Copy semantics - efficient data sharing
cvc::voxels v1(cvc::dimension(100, 100, 100), cvc::Float);
v1.fill(42.0);

// Shallow copy (default) - shares data via boost::shared_array
cvc::voxels v2;
v2.copy(v1);              // Fast, no memory duplication
// or: cvc::voxels v2(v1); // Copy constructor also shallow

// Deep copy - creates independent data allocation
cvc::voxels v3;
v3.copy(v1, true);        // Full memory duplication
v3(0, 0, 0, 99.0);        // Modify v3 without affecting v1
```

### Geometry Processing

```cpp
#include <cvc/geometry.h>

// Load triangle mesh
cvc::geometry mesh("bunny.off");

// Access mesh data
std::cout << "Vertices: " << mesh.num_points() << std::endl;
std::cout << "Triangles: " << mesh.num_tris() << std::endl;

// Calculate surface normals
mesh.calculate_surf_normals();

// Merge multiple geometries
cvc::geometry combined = mesh1;
combined.merge(mesh2);  // Indices automatically remapped

// Extract boundary surface
cvc::geometry surface = mesh.tri_surface();

// Get bounding box
cvc::bounding_box bbox = mesh.extents();

// Save mesh
mesh.write("output.raw");
```

### Signed Distance Functions (v2.0)

```cpp
#include <cvc/algorithm.h>
#include <cvc/geometry.h>

// Load triangle mesh
cvc::geometry bunny = cvc::read_geometry("bunny.off");

// Define output grid and bounding box
cvc::dimension dim(128, 128, 128);  // 128³ voxels
cvc::bounding_box bbox = bunny.bounding_box();
bbox.expand(0.05);  // Add 5% padding

// Compute signed distance field (thread-safe, optimized)
cvc::volume sdf_vol = cvc::sdf(bunny, dim, bbox);

// Access distances
for (uint64 k = 0; k < sdf_vol.ZDim(); k++) {
    for (uint64 j = 0; j < sdf_vol.YDim(); j++) {
        for (uint64 i = 0; i < sdf_vol.XDim(); i++) {
            double dist = sdf_vol(i, j, k);
            if (dist < 0.0) {
                // Inside the surface
            }
        }
    }
}

// Extract isosurface at offset distance
cvc::geometry offset_surface = cvc::isosurface(sdf_vol, 0.1);

// Thread-safe: compute multiple SDFs in parallel
#pragma omp parallel for
for (int i = 0; i < n_geometries; i++) {
    results[i] = cvc::sdf(geometries[i], dims[i], bboxes[i]);
}
```

**SDF Performance** (256³ resolution, 35K triangles):
- **Release build**: ~220 seconds
- **RelWithDebInfo build**: ~234 seconds (recommended)
- **Debug build**: ~3027 seconds (development only)

See [docs/SDF_LIBRARY.md](docs/SDF_LIBRARY.md) for complete documentation.

## Supported File Formats

### Volume Formats
- RAWIV, RAWV (Raw image volumes)
- MRC (Medical Research Council via IMOD)
- Spider (SPIDER image format)
- CVC (Custom format with HDF5)
- VTK (VTK legacy format)
- DX (OpenDX format)

### Geometry Formats
- OFF (Object File Format) - Read/Write
- RAW/RAWN/RAWC/RAWNC (Raw geometry variants) - Read/Write
- BUNNY (Stanford Bunny for testing/demos) - Read-only
- OBJ (Wavefront OBJ via SDF) - Experimental

## Documentation

### Core Documentation

- **[USAGE.md](USAGE.md)** - Consumer guide for `find_package(cvc)` from a CMake project
- **[docs/APP_API.md](docs/APP_API.md)** - `cvc::app` runtime context API
- **[docs/STATE_API.md](docs/STATE_API.md)** - State tree / property bag API
- **[docs/VOLUME_API.md](docs/VOLUME_API.md)** - Volume data structures
- **[docs/GEOMETRY_API.md](docs/GEOMETRY_API.md)** - Geometry data structures
- **[docs/THREAD_POOL.md](docs/THREAD_POOL.md)** - Thread pool overview, examples, and usage guide
- **[docs/THREAD_POOL_API.md](docs/THREAD_POOL_API.md)** - Thread pool API reference
- **[docs/CUDA_GUIDE.md](docs/CUDA_GUIDE.md)** - CUDA usage guide

### Testing Documentation

- **[docs/TESTING.md](docs/TESTING.md)** - Comprehensive testing guide
  - **577 tests with 100% pass rate** (72 app, 158 state, 133 voxels, 29 volume, 119 geometry, 42 volume-ops, 24 hdf5, plus procedural-geometry typed fixtures)
  - **90.5% coverage on core components** (1,366/1,509 lines)
  - Running tests (CTest, Google Test, custom check target)
  - Test organization and naming conventions
  - Detailed coverage analysis with lcov/gcov
  - Advanced features: multithreaded tests, futures API, state_object pattern, CUDA GPU
  - Performance benchmarks (SDF v2.0: 11x speedup)
  - Adding new tests with examples
  - Common testing patterns (fixtures, parameterized tests, multithreading)
  - CI/CD integration examples
  - Troubleshooting guide
  - Best practices and future improvements

### SDF Library

- **[docs/SDF_LIBRARY.md](docs/SDF_LIBRARY.md)** - **Comprehensive SDF documentation (RECOMMENDED)**
  - **Version 2.0**: Complete refactoring to thread-safe architecture
  - **11x performance improvement** via cell reference caching optimization
  - Quick start guide and complete API reference
  - Performance benchmarks and build type impact analysis
  - Development history: v1.x (global state) → v2.0 (thread-safe)
  - Migration guide from deprecated v1.x API
  - **Future plans**: CUDA GPU acceleration roadmap (10-100x speedup target)
  - Thread-safe parallel computation examples
  - Consolidates all previous SDF documentation

### Image Processing Algorithms

- **[docs/IMAGE_PROCESSING_ALGORITHMS.md](docs/IMAGE_PROCESSING_ALGORITHMS.md)** - Comprehensive API reference
  - **Anisotropic Diffusion**: Edge-preserving noise reduction (Perona-Malik model)
  - **Bilateral Filter**: Non-linear smoothing with edge preservation
  - **Contrast Enhancement**: Adaptive histogram equalization with resistor propagation
  - **GDTV Filter**: Gradient-dependent total variation regularization
  - Complete algorithm details, parameters, usage examples, and best practices

### API Documentation

- **[docs/APP_API.md](docs/APP_API.md)** - Complete application framework API
  - Singleton application object (`cvcapp`)
  - Data management with `boost::any` type-safe storage
  - Property system for configuration
  - Thread management with progress tracking
  - Named mutex system for resource synchronization
  - Type registration for human-readable names
  - Signals for change notifications
  - Complete examples and design patterns

- **[docs/STATE_API.md](docs/STATE_API.md)** - Complete state tree API reference
  - Hierarchical key-value store
  - Value and data operations
  - Property system
  - Tree navigation
  - Futures API for async/await patterns
  - State object pattern (CRTP-based)
  - Producer-consumer patterns
  - Callbacks and signals with safety patterns
  - Avoiding infinite loops and stack overflow
  - Complete examples and best practices

- **[docs/VOLUME_API.md](docs/VOLUME_API.md)** - Complete volume/voxels API reference (VolMagick)
  - Multi-dimensional voxel container (`voxels`)
  - Spatial volume with coordinate system (`volume`)
  - Memory semantics: shallow vs deep copy
  - Data types: UChar, UShort, UInt, Float, Double, UInt64
  - Image processing filters: bilateral, anisotropic diffusion, GDTV, contrast enhancement
  - Trilinear interpolation and resampling
  - File I/O: RAWIV, MRC, HDF5, VTK formats
  - CUDA GPU acceleration with unified memory
  - Subvolume extraction and compositing
  - Complete examples and best practices

- **[docs/GEOMETRY_API.md](docs/GEOMETRY_API.md)** - Complete geometry/mesh API reference
  - Triangle mesh container (`geometry`)
  - Copy-on-write memory semantics
  - Volumetric mesh support (tetrahedral, hexahedral)
  - Mesh operations: merge, surface extraction, normal computation
  - Mesh processing: smoothing, quality improvement, projection
  - File I/O: OFF, RAW, RAWN, RAWC, RAWNC formats
  - Extensible file I/O system
  - Boundary vertex marking
  - Complete examples and best practices

- **[docs/CUDA_GUIDE.md](docs/CUDA_GUIDE.md)** - CUDA development guide
  - Modern CMake CUDA integration
  - CUDA architecture configuration
  - Example CUDA code patterns

- **[docs/CUDA_MODERNIZATION.md](docs/CUDA_MODERNIZATION.md)** - CUDA migration
  - Legacy to modern CMake CUDA
  - Before/after comparisons
  - Build examples

### Development Guidelines

- **[docs/CONTRIBUTING.md](docs/CONTRIBUTING.md)** - Development guidelines
  - Code style and conventions
  - Build system best practices
  - Git workflow recommendations

## Project Structure

```
libcvc/
├── CMakeLists.txt          # Root build configuration
├── CMake/                  # CMake helper modules
├── inc/                    # Public headers
│   ├── cvc/               # Main library headers
│   └── xmlrpc/            # XMLRPC headers
├── src/                    # Implementation
│   ├── cvc/               # Main library
│   │   ├── libiimod/      # MRC file support
│   │   ├── cvc-mesher/    # Meshing algorithms
│   │   └── SDF/           # Distance functions
│   └── xmlrpc/            # XMLRPC implementation
└── build_verify.sh         # Build verification script
```

## Version History

- **3.0.0** (2026) - **Production-grade packaging & CI/CD release**
  - Cross-platform release artifacts: Linux (.tar.gz, .deb, AppImage),
    macOS (.zip, .dmg with VolumeRover3.app bundle), Windows (.zip,
    NSIS installer with desktop/start-menu shortcuts)
  - Separate libcvc SDK (Debug + Release) and volrover3 user-app
    components, packaged independently per platform
  - Portable `find_package(cvc)` with full transitive dependency
    discovery (Boost, CGAL, HDF5, OpenMP, FFTW, CUDAToolkit, etc.)
  - Qt deployment integration (windeployqt, macdeployqt, linuxdeploy)
  - Multi-platform CI matrix (Ubuntu, macOS, Windows) with vcpkg
    binary caching
- **2.0.0** (2025) - **Major modernization release**
  - Thread-safe SDF library with 11x performance improvement
  - Modernized CMake build system with proper CUDA integration
  - C++14+ support with smart pointers and RAII
  - Comprehensive test suite (353 tests, 100% passing)
  - 64.6% code coverage on core modules
  - Complete refactoring of global-state architecture to thread-safe contexts
- **1.0.0** (Legacy) - Original release with global-state architecture

## Contributing

This is a modernization of legacy research software. Contributions welcome:

1. Bug fixes and improvements
2. Additional file format support
3. Test coverage for I/O and legacy modules
4. Documentation enhancements
5. Performance optimizations

## Known Issues

- Duplicate VolMagick code in mesher component (historical TODO)
- Coverage race conditions in multithreaded tests (use `--ignore-errors negative` with lcov)
- ✅ **Algorithm Tests** - 6 SDF/isosurface tests including 256³ stress test
- ✅ **Volume & Voxels** - 157 combined tests for volumetric data structures

## License

See [LICENSE](LICENSE) file for details.

## Credits

Based on research and software developed at the **Computational Visualization Center, University of Texas at Austin**.

Original VolumeRover package contributors and the CVC research group.

## References

- [VolumeRover Project](https://www.cs.utexas.edu/~bajaj/cvc/)
- [Computational Visualization Center](https://www.ices.utexas.edu/)

## Contact

For questions or issues, please open an issue on the project repository or contact:
- Joe Rivera (j@jriv.us)

