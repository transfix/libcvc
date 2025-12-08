# trans-cvc

[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-14%2F17%2F20-orange.svg)](https://isocpp.org/)
[![License](https://img.shields.io/badge/license-Check%20LICENSE-green.svg)](LICENSE)

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
  -DCVC_USING_HDF5=ON \                 # Enable HDF5 support
  -DCVC_ENABLE_MESHER=ON \              # Enable meshing features
  -DCVC_ENABLE_SDF=ON \                 # Enable SDF calculations
  -DCVC_USING_XMLRPC=OFF \              # Enable network sharing
  -DDISABLE_CGAL=OFF                    # Enable CGAL support
```

See `PROJECT_REPORT.md` for a complete list of build options.

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

- **[PROJECT_REPORT.md](PROJECT_REPORT.md)** - Comprehensive project documentation
  - Full dependency list and installation instructions
  - Detailed build options
  - Architecture overview
  - Migration guide from version 1.x to 2.0

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
- Unit tests not yet implemented

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

