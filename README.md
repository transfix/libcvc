# trans-cvc

[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-14%2F17%2F20-orange.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-Check%20LICENSE-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-320%20passing%20%7C%2019%20CUDA-brightgreen.svg)](#testing)
[![Coverage](https://img.shields.io/badge/core%20coverage-89.8%25-brightgreen.svg)](TESTING_COVERAGE.md)

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

A comprehensive computational visualization library from the Computational Visualization Center at UT Austin. Trans-cvc provides the computational core functionality of the VolumeRover package, including volume processing, geometry manipulation, isosurfacing, and signed distance function calculations.

**Author:** Joe R - transfix@sublevels.net

## Features

- 🎨 **Volume Processing**: Multiple volume file format support (RAWIV, MRC, Spider, HDF5, VTK)
- 🔺 **Geometry Handling**: Read/write various geometry formats (OFF, OBJ, RAW variants)
- 🎯 **Meshing & Isosurfacing**: Marching cubes, LBIE meshing, fast contouring
- 📐 **Signed Distance Functions**: Calculate distance fields from geometries
- 🌐 **Network Support**: Optional XMLRPC for distributed state sharing
- 🔬 **Image Filtering**: Bilateral filter, anisotropic diffusion, GDTV, contrast enhancement
- 🧮 **Scientific Computing**: Integration with FFTW, GSL, CGAL, Boost

## Quick Start

### Prerequisites

**Required:**
- CMake 3.15 or higher
- C++14 compatible compiler (GCC 5+, Clang 3.8+, MSVC 2017+)
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
cd /path/to/trans-cvc

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
  -DCMAKE_BUILD_TYPE=Release \          # Build type (Debug/Release)
  -DCMAKE_CXX_STANDARD=17 \             # C++ standard (14/17/20/23)
  -DBUILD_SHARED_LIBS=ON \              # Build shared libraries
  -DCVC_BUILD_TESTS=ON \                # Enable unit tests (ON by default)
  -DCVC_USING_HDF5=ON \                 # Enable HDF5 support
  -DCVC_ENABLE_MESHER=ON \              # Enable meshing features
  -DCVC_ENABLE_SDF=ON \                 # Enable SDF calculations
  -DCVC_USING_XMLRPC=OFF \              # Enable network sharing
  -DDISABLE_CGAL=OFF                    # Enable CGAL support
```

See `PROJECT_REPORT.md` for a complete list of build options.

## Testing

Trans-cvc includes comprehensive unit tests using Google Test. Tests are **enabled by default**.

### Test Suite: 320 Tests (100% Passing)

- **114 App Tests** - Core application framework, data/property management, threading
- **128 State Tests** - State tree, hierarchies, serialization, signals, async operations
- **127 Voxels Tests** - Volume data operations, algorithms, **19 CUDA tests** (GPU acceleration, multithreading)
- **29 Volume Tests** - Spatial coordinates, interpolation, subvolumes, bounding boxes
- **37 Geometry Tests** - Mesh operations, normals, I/O using Stanford Bunny

### Code Coverage: 90.5% on Core Components

The actively tested core modules (app, state, voxels, volume, geometry) achieve excellent coverage:

- **state.cpp**: 92.75% (243/262 lines)
- **voxels.cpp**: 92.17% (306/332 lines)  ⚡ Includes CUDA paths
- **volume.cpp**: 91.18% (124/136 lines)
- **app.cpp**: 89.82% (441/491 lines)
- **geometry.cpp**: 79.00% (252/319 lines)
- **Functions**: 94.7% (355/375 functions)

See [TESTING_COVERAGE.md](TESTING_COVERAGE.md) for detailed coverage analysis

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

See **[TESTING.md](TESTING.md)** for detailed testing documentation:
- How to run tests
- Test coverage details
- Adding new tests
- CI/CD integration
- Troubleshooting

See **[FUTURES_API.md](FUTURES_API.md)** for async programming patterns:
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

- **[PROJECT_REPORT.md](PROJECT_REPORT.md)** - Comprehensive project documentation
  - Full dependency list and installation instructions
  - Detailed build options (all 20+ options explained)
  - Architecture overview
  - Migration guide from version 1.x to 2.0
  - Platform support (Linux, Windows, macOS, BSD)

### Testing Documentation

- **[TESTING.md](TESTING.md)** - Unit testing guide
  - Running tests with CTest and Google Test
  - Test organization and coverage (308 tests)
  - Adding new tests
  - CI/CD integration examples
  - Code coverage with lcov/genhtml

- **[TESTING_COVERAGE.md](TESTING_COVERAGE.md)** - Coverage analysis
  - 308 tests across app, state, voxels, volume, and geometry components
  - **89.6% coverage on core components** (1,740/1,941 lines)
  - Detailed per-file coverage metrics including geometry API
  - Bug fixes and API improvements from testing
  - Coverage report generation with lcov/genhtml/gcov

- **[TESTING_IMPLEMENTATION.md](TESTING_IMPLEMENTATION.md)** - Implementation details
  - Google Test infrastructure setup
  - Test file structure
  - Success metrics and verification

### Image Processing Algorithms

- **[IMAGE_PROCESSING_ALGORITHMS.md](docs/IMAGE_PROCESSING_ALGORITHMS.md)** - Comprehensive API reference
  - **Anisotropic Diffusion**: Edge-preserving noise reduction (Perona-Malik model)
  - **Bilateral Filter**: Non-linear smoothing with edge preservation
  - **Contrast Enhancement**: Adaptive histogram equalization with resistor propagation
  - **GDTV Filter**: Gradient-dependent total variation regularization
  - Complete algorithm details, parameters, usage examples, and best practices
  - Based on 271-test validation suite

### Advanced Features

- **[FUTURES_API.md](FUTURES_API.md)** - Async state programming
  - Blocking waits for values/data
  - Callback registration
  - Producer-consumer patterns
  - Timeout handling
  - 11 futures tests

- **[CUDA_GUIDE.md](CUDA_GUIDE.md)** - CUDA development guide
  - Modern CMake CUDA integration
  - CUDA architecture configuration
  - Example CUDA code patterns

- **[CUDA_MODERNIZATION.md](CUDA_MODERNIZATION.md)** - CUDA migration
  - Legacy to modern CMake CUDA
  - Before/after comparisons
  - Build examples

### Development Guidelines

- **[CONTRIBUTING.md](CONTRIBUTING.md)** - Development guidelines
  - Code style and conventions
  - Build system best practices
  - Git workflow recommendations

## Project Structure

```
trans-cvc/
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

- **2.0.0** (2025) - Modernized CMake build system, C++14+ support
- **1.0.0** (Legacy) - Original release

## Contributing

This is a modernization of legacy research software. Contributions welcome:

1. Bug fixes and improvements
2. Additional file format support
3. Test coverage for I/O and legacy modules
4. Documentation enhancements
5. Performance optimizations

## Known Issues

- Duplicate VolMagick code in mesher component (historical TODO)
- No CUDA source files yet (infrastructure ready - see CUDA_GUIDE.md)
- Coverage race conditions in multithreaded tests (use `--ignore-errors negative` with lcov)

## Recent Additions (December 2025)

- ✅ **Geometry Class Tests** - 37 comprehensive tests using Stanford Bunny mesh
- ✅ **308 Total Tests** - 100% passing with 89.6% coverage on core components
- ✅ **Geometry API Coverage** - 80.80% coverage (282/349 lines) for mesh operations
- ✅ **Volume Class Tests** - 29 tests for spatial coordinates and interpolation
- ✅ **Bug Fixes** - Fixed volume::sub() bounding box and voxels::operator== byte comparison
- ✅ **API Documentation** - Documented shallow copy semantics and deep copy alternatives
- ✅ **Coverage Analysis** - Detailed per-file metrics with lcov/genhtml/gcov

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

