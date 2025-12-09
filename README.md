# trans-cvc

[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-14%2F17%2F20-orange.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-Check%20LICENSE-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-271%20passing-brightgreen.svg)](#testing)

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

### Test Suite: 145 Tests (100% Passing)

- **53 App Tests** - Core application framework, data/property management, threading
- **81 State Tests** - State tree, hierarchies, serialization, signals
- **11 Futures Tests** - Async value retrieval, blocking waits, callbacks

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

- ✅ **Multithreaded Operations** - 12 concurrent access tests
- ✅ **Futures API** - Async value retrieval with blocking/callbacks
- ✅ **Deadlock Detection** - Signal handler reentrancy tests
- ✅ **Producer-Consumer Patterns** - Thread coordination tests
- ✅ **Stress Testing** - 1000+ operations without deadlock

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

## Usage Example

```cpp
#include <cvc/volume.h>
#include <cvc/geometry.h>

// Load a volume file
cvc::volume vol("data.rawiv");

// Apply filtering
vol.bilateral_filter(sigma_space, sigma_range);

// Extract isosurface
cvc::geometry mesh = vol.isosurface(isovalue);

// Save result
mesh.save("output.off");
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
- OFF (Object File Format)
- OBJ (Wavefront OBJ via SDF)
- RAW/RAWN/RAWC/RAWNC (Raw geometry variants)

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
  - Test organization and coverage (145 tests)
  - Adding new tests
  - CI/CD integration examples
  - Code coverage with lcov/genhtml

- **[TESTING_COVERAGE.md](TESTING_COVERAGE.md)** - Coverage analysis
  - 122 core tests detailed breakdown
  - Coverage metrics (78%+ on critical components)
  - Testing strategy and best practices
  - Improving coverage guidelines

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
3. Unit test coverage (currently needed!)
4. Documentation enhancements
5. Performance optimizations

## Known Issues

- Duplicate VolMagick code in mesher component (historical TODO)
- No CUDA source files yet (infrastructure ready - see CUDA_GUIDE.md)
- Coverage race conditions in multithreaded tests (use `--ignore-errors negative` with lcov)

## Recent Additions (December 2025)

- ✅ **Futures API** - Async value retrieval with blocking waits and callbacks
- ✅ **Multithreaded Tests** - 12 comprehensive concurrency tests
- ✅ **145 Total Tests** - 100% passing with excellent coverage
- ✅ **Thread Safety Validation** - Deadlock detection and stress testing
- ✅ **Comprehensive Documentation** - 7 detailed markdown documents

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
- Joe R (transfix@sublevels.net)

