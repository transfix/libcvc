# libcvc

[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-20-orange.svg)](https://isocpp.org/)
[![License: GPL v2](https://img.shields.io/badge/license-GPL%20v2-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-2662%20passing-brightgreen.svg)](#testing)
[![Coverage](https://img.shields.io/badge/coverage-83.8%25%20lines%20%7C%2091.4%25%20functions-brightgreen.svg)](docs/TESTING.md)

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
- [Core APIs](#core-apis)
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

A comprehensive computational visualization library from the Computational Visualization Center at UT Austin. libcvc provides the computational core functionality of the VolumeRover package, including volume processing, geometry manipulation, isosurfacing, and signed distance function calculations. Beyond the volume/geometry core it now also hosts a torch-free real-time **reactive swarm navigation** runtime (`cvc::nav`), a VTK-backed **3D scene graph + renderer** (`cvc::gl` / cvcGL), native **image and mesh/asset** loaders (`cvc::image`, `cvc::model`), a fixed-step **simulation clock** (`cvc::world_clock`), and a **federated distributed-state** replication layer.

**Maintainer:** Joe Rivera - j@jriv.us

## Features

- 🎨 **Volume Processing**: Multiple volume file format support (RAWIV, MRC, Spider, HDF5, VTK)
- 🔺 **Geometry Handling**: Read/write various geometry formats (OFF, OBJ, RAW variants)
- 🎯 **Meshing & Isosurfacing**: Marching cubes, LBIE meshing, fast contouring
- 📐 **Signed Distance Functions v2.0**: Thread-safe, 11x faster, GPU-ready architecture
- 🌐 **Distributed state**: a federated `cvc::state` replication layer —
  peer-to-peer mutation journal, inproc/IPC/gRPC transports, leased subtree
  delegation, content-addressed blobs. See
  [`docs/roadmap/DISTRIBUTED_STATE_ROADMAP.md`](docs/roadmap/DISTRIBUTED_STATE_ROADMAP.md)
  and `USAGE.md` §6. (The legacy `CVC_USING_XMLRPC` option is unrelated to this
  and is off by default.)
- 🔬 **Image Filtering**: Bilateral filter, anisotropic diffusion, GDTV, contrast enhancement
- 🧭 **Reactive Swarm Navigation** (`cvc::nav`): a torch-free, Python-free real-time reactive swarm runtime ported from GRL-SNAM — bit-identical grid kernels (exact EDT, 8-connected A*, footprint→SDF), a fused per-agent drive (`coef_feats` → `CoefMLP` → kinematic-bicycle rollout), a `sim_world` swarm with shared/grouped/private belief planes plus a device-resident CUDA twin, lock-free off-thread stepping, and a self-supervised policy trainer (CPU + CUDA, portable `.cvcnav` weights). Terrain SEMANTICS ride along: per-cell material risk + hard hazards with a feasibility-witness gate ([`docs/NAV_MATERIAL.md`](docs/NAV_MATERIAL.md)). See [`docs/NAV_TRAINING.md`](docs/NAV_TRAINING.md).
- 🖼️ **3D Scene Graph & Assets**: a VTK-backed scene graph + persistent renderer (`cvc::gl` / cvcGL — geometry/volume/grid nodes, scene-owned lighting & shadows, offscreen/onscreen capture), a standalone 2D raster + codecs container (`cvc::image` — PNG/JPEG/WebP), and a PBR multi-mesh scene loader (`cvc::model` — OBJ/glTF/GLB/FBX/DAE/PLY via assimp)
- ⏱️ **Simulation Clock** (`cvc::world_clock`): an authoritative fixed-step clock separating world time from wall time and render cadence — banks `advance(wall_dt)` into whole quanta and returns `{steps, alpha}` for interpolated rendering, with deterministic live/replay/paused modes
- 🔭 **Level-of-Detail selection** (`cvc::lod`): single-process, allocation-free LOD math shared by the cvcGL nav demos and the L-System Laboratory — hysteretic rung selection by screen-space error, a width-based mesh↔impostor switch, and a greedy triangle/prop/memory budget solver that is a pure (headless-exact) function of its inputs. See [`docs/LOD_API.md`](docs/LOD_API.md).
- 🧮 **Scientific Computing**: Integration with FFTW, GSL, CGAL, Boost

## Quick Start

### Prerequisites

**Required:**
- CMake 3.15 or higher
- C++20 compatible compiler (GCC 13+, Clang 17+, MSVC 19.29+); C++23 selectable via `-DCMAKE_CXX_STANDARD=23`
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

### Dependencies via cvcpkg (reproducible — matches CI)

Instead of system packages, you can install libcvc's **exact, pinned dependency
set** — the same bundles CI builds against — with [cvcpkg](https://cvcpkg.org).
The dependency list lives in exactly one place: libcvc's in-tree recipe
`cvcpkg/recipes/libcvc`. `cvcpkg install-deps` reads its declared deps straight
from there (there is no separate requirements file):

```bash
# Install libcvc's dependency closure (boost, hdf5, cgal, fftw3, gsl,
# imagemagick, libiimod, openblas, protobuf/grpc, …) into ./deps.
cvcpkg install-deps cvcpkg/recipes/libcvc --prefix deps --config release
```

Then build libcvc against that prefix — either drive CMake yourself:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/deps"
cmake --build build -j
```

…or build through the recipe, **reusing the deps you just installed** — `--no-deps`
skips re-resolving them. It builds *this* checkout (the recipe's source is the
repo root) and installs into the same prefix:

```bash
cvcpkg build libcvc --recipes-dir cvcpkg/recipes --no-deps --prefix deps --config release
```

Notes:

- Host build tools (CMake, Ninja) come from your system `PATH` —
  `install-deps` installs only the library dependencies. Add
  `--include-host-tools` to install cmake/ninja from cvcpkg as well.
- For a debug dependency set + build, pass `--config debug` to **both** commands.

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

### Test Suite: 2,662 Google Test cases across 86+ suites (100% passing)

A default Linux + CUDA build compiles ~2,216 of them; the gRPC-transport suite and a
few POSIX-only distributed-state suites are opt-in. The major families:

- **Core framework** — `app_test` (72: context, data/property store, threading, signals), `state_test` + `state_list_test` (state tree, hierarchies, futures, `state_object` pattern)
- **State execution DSL** (`cvc::state_exec`) — the largest family: parser/AST, tree-walking + stackless + async evaluators, scheduler, coordinator, builtins, codec (`state_exec_*`, 800+ cases)
- **Distributed state replication** — transports (inproc/IPC/gRPC), change journal, delegation, cluster membership/sharding, blob store, links & telemetry (dozens of `state_*` suites)
- **Volume / voxels** — `voxels_test` (136, incl. CUDA paths), `volume_test` (29), `volume_io_test` (25), `volume_ops_test` (42), `hdf5_test` (24)
- **Geometry** — `geometry_test` (119), `geometry_attributes_test` (10), `algorithm_test` (10: SDF / isosurface, incl. a 256³ stress test)
- **Navigation** (`cvc::nav`) — `nav_test` (41: kernels, drive, `sim_world` shared/grouped/private belief, `sim_thread`, CUDA twin) + `nav_coef_train_test` (10: torch-free trainer gradcheck; the two full train-then-drive convergence runs are opt-in, see `NavCoefTrainConvergence`)
- **Simulation clock** — `world_clock_test` (33)
- **Assets** — `image_test` (19), `model_test` (9)
- **Mesher / SDF internals** — `lbie_mesher_test` (LBIE octree subdivision, quad/interval/tetra2 mesh types, quality-improve methods incl. `OPTIMIZATION`), `fastcontouring_math_test` (Quaternion/Matrix/Vector/Ray/ContourGeometry), `mtxlib_test` (SDF V2's vector/matrix library, `DistanceTransform` predicates)
- **Volume I/O depth** — `hdf5_volume_test` (the HDF5 volume backend: multi-variable/timestep, `|object` addressing, subvolume reads), `volume_io_extra_test` (MRC/RAWV/RAWIV/VTK/Spider/cvcraw error paths and format edge cases)
- **State execution gaps** — `state_exec_gaps_test` (evaluator/stackless-evaluator/async-scheduler error paths and object-model edge cases)

### Code Coverage

CI enforces an **80% line-coverage gate** on the supported surface (`src/` + `inc/`,
minus tests and two vendored legacy trees) — see
[`.github/workflows/ci.yml`](.github/workflows/ci.yml) and
[docs/TESTING.md](docs/TESTING.md#code-coverage-analysis) for the exact filter. Coverage
is generated with `lcov`/`gcov` from a `Debug + Coverage` build; run
`./generate_coverage.sh` locally to regenerate the HTML report.

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
- ✅ **CUDA GPU Acceleration** - device selection, memory migration, and GPU operations (voxels CUDA-guarded cases, plus the `cvc::nav` `drive.cu` / `sim_world_cuda` device path)
- ✅ **Spatial Interpolation** - Trilinear interpolation and gradients
- ✅ **Subvolume Operations** - Coordinate system transformations
- ✅ **Edge Cases** - Boundary conditions, tiny/large volumes
- ✅ **Memory Semantics** - Shallow copy by default (ref-counted CUDA), deep copy available

### Documentation

See **[docs/TESTING.md](docs/TESTING.md)** for comprehensive testing documentation:
- 2,240 Google Test cases across 86 suites (100% passing), with the full per-suite breakdown
- Coverage details and how to regenerate the report
- How to run tests (CTest, Google Test)
- Multithreaded, futures API, and `state_object` pattern testing
- CUDA GPU tests
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
#include <cvc/volume/volume.h>

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
#include <cvc/geometry/geometry.h>

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
#include <cvc/utility/algorithm.h>
#include <cvc/geometry/geometry.h>

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

### Reactive Swarm Navigation (`cvc::nav`)

Drop thousands of vehicles that react to a map into a pure-C++ host — no libtorch, no
Python. `sim_world::from_occupancy` scatters agents onto free cells; each `step()` runs
sense → field rebuild → carrot FSM → fused drive, threaded across agents, and
`snapshot()` hands world-space poses straight to a renderer. Belief is M planes selected
per agent (shared / grouped / private); a device-resident `sim_world_cuda` twin runs the
same math on the GPU when N outgrows the CPU.

```cpp
#include <cvc/nav/sim_world.h>
using namespace cvc::nav;

// occ: row-major rows*cols uint8 (0 = free cell, nonzero = wall/building)
sim_world::config cfg;
cfg.rows = R;  cfg.cols = C;
cfg.min_x = -400; cfg.min_y = -400; cfg.max_x = 400; cfg.max_y = 400;
cfg.scale = 0.02;          // world metres -> normalized
cfg.freeze_sense = true;   // static known map; set false for discover-as-you-go fog

// Scatter 2000 agents on free cells with the zero-setup bias policy (no .cvcnav
// file needed). Swap in coef_mlp::load(coef_mlp::default_weights_path()) for a
// trained policy. `mode` defaults to shared belief (M = 1).
const int N = 2000;
sim_world world = sim_world::from_occupancy(cfg, occ.data(),
                                            coef_mlp::default_biased(), N, /*seed=*/42);

std::vector<float> pos(2 * N), heading(N), speed(N);
std::vector<int> mode(N);
std::vector<std::uint8_t> reached(N);
for (int t = 0; t < 400; ++t) {
  world.step();  // sense -> rebuild -> carrot FSM -> fused drive; threaded internally
  world.snapshot(pos.data(), heading.data(), speed.data(), mode.data(), reached.data());
  // pos[] is WORLD metres, heading[] rad -> hand straight to a renderer
}
```

The policy is trained torch-free (self-supervised, differentiable rollout with a
finite-difference-checked adjoint) via `coef_train` / the `nav_train_demo` CLI, on CPU or
CUDA, exporting the portable `.cvcnav` weight blob. See
[docs/NAV_TRAINING.md](docs/NAV_TRAINING.md).

### Level of Detail (`cvc::lod`)

Header-only-to-call selection math for keeping a large scene inside a frame
budget — used by the cvcGL nav demos and the L-System Laboratory. No VTK, no GL,
no allocation on the hot path.

```cpp
#include <cvc/lod/select.h>
using namespace cvc::lod;

view_params view = preset_view(quality_preset::balanced);   // 2.0 px error budget
view.eye[0] = cam.x; view.eye[1] = cam.y; view.eye[2] = cam.z;

// Per visible group: nearest-bound distance -> hysteretic rung (0 = finest).
double dist = bound_distance_m(tile.centre, tile.radius_m, view);
tile.rung   = select_rung(dist, tile.world_error_m, tile.nrungs, tile.rung, view);

// Fit the whole grid to a triangle / prop / memory budget (headless-exact).
plan p = solve(candidates, preset_budget(budget_profile::desktop_default));
```

Measured 3.7–6.0× fewer triangles on the Austin bundle; `solve()` for a
1024-tile grid runs in ~3.5 µs/frame. Full reference, presets, and the
`lab.lod.*` state-tree knobs: **[docs/LOD_API.md](docs/LOD_API.md)**.

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

### Mesh / Model Formats (`cvc::model`, assimp-backed)
- OBJ, glTF, GLB, FBX, DAE (COLLADA), PLY — multi-mesh scenes with glTF-style PBR materials (base color, metallic/roughness/emissive, textures)

### Image Formats (`cvc::image`)
- PNG, JPEG, WebP — 2D rasters (GRAY / GRAY_ALPHA / RGB / RGBA × u8 / u16 / f32), registry-dispatched load/save

## Documentation

### Core Documentation

- **[docs/CLI_GUIDE.md](docs/CLI_GUIDE.md)** - Unified `cvc` CLI guide with examples for volume processing, geometry meshing, distributed state, and script execution
- **[USAGE.md](USAGE.md)** - Consumer guide for `find_package(cvc)` from a CMake project
- **[docs/APP_API.md](docs/APP_API.md)** - `cvc::app` runtime context API
- **[docs/STATE_API.md](docs/STATE_API.md)** - State tree / property bag API
- **[docs/VOLUME_API.md](docs/VOLUME_API.md)** - Volume data structures
- **[docs/GEOMETRY_API.md](docs/GEOMETRY_API.md)** - Geometry data structures
- **[docs/THREAD_POOL.md](docs/THREAD_POOL.md)** - Thread pool overview, examples, and usage guide
- **[docs/THREAD_POOL_API.md](docs/THREAD_POOL_API.md)** - Thread pool API reference
- **[docs/CUDA_GUIDE.md](docs/CUDA_GUIDE.md)** - CUDA usage guide
- **[docs/NAV_TRAINING.md](docs/NAV_TRAINING.md)** - `cvc::nav` self-supervised policy training (torch-free, CPU + CUDA; surrogate vs bicycle rollout)
- **[docs/LOD_API.md](docs/LOD_API.md)** - `cvc::lod` level-of-detail selection math: rung selection, budget solver, presets, and the user-facing knobs

### Testing Documentation

- **[docs/TESTING.md](docs/TESTING.md)** — testing guide
  - Running the test suite (CTest, Google Test, custom `check` target)
  - Test organization, naming conventions, and fixtures
  - Coverage analysis with `lcov`/`gcov`
  - Multithreaded tests, futures API, `state_object` pattern, CUDA-enabled tests
  - Adding new tests with examples
  - CI/CD integration and troubleshooting

### SDF Library

- **[docs/SDF_LIBRARY.md](docs/SDF_LIBRARY.md)** — full SDF library reference
  - What a signed distance field is and the sign / layout conventions used
  - Complete public API (`cvc::sdf`, `SDFLibrary::computeSDF_MT`, `SDFContext`)
  - Octree + propagation algorithm walkthrough
  - Thread-safe parallel computation examples
  - Performance guide and grid-sizing tips
- **[src/cvc/SDF/SignDistanceFunction/README.md](src/cvc/SDF/SignDistanceFunction/README.md)** —
  short, header-level overview of the SDF backend living next to the code

### Image Processing Algorithms

- **[docs/IMAGE_PROCESSING_ALGORITHMS.md](docs/IMAGE_PROCESSING_ALGORITHMS.md)** - Comprehensive API reference
  - **Anisotropic Diffusion**: Edge-preserving noise reduction (Perona-Malik model)
  - **Bilateral Filter**: Non-linear smoothing with edge preservation
  - **Contrast Enhancement**: Adaptive histogram equalization with resistor propagation
  - **GDTV Filter**: Gradient-dependent total variation regularization
  - Complete algorithm details, parameters, usage examples, and best practices

### API Documentation

- **[docs/APP_API.md](docs/APP_API.md)** - Complete application framework API
  - Per-app context object (`cvc::app`) — explicit instances, no singleton
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
├── inc/cvc/                # Public headers, one directory per module:
│   ├── core/              #   cvc::app / cvc::state, state_exec DSL, world_clock, distributed state
│   ├── volume/            #   VolMagick voxels/volume + I/O + filters
│   ├── geometry/          #   triangle/volumetric meshes + I/O
│   ├── utility/           #   algorithm.h (cvc::sdf, isosurface), CUDA utils
│   ├── nav/               #   cvc::nav reactive swarm navigation + trainer
│   ├── gl/                #   cvcGL VTK scene graph + renderer
│   ├── image/             #   cvc::image raster + codecs
│   └── model/             #   cvc::model PBR meshes (assimp)
├── src/                    # Implementation (mirrors inc/cvc, + cvc/tests, cvc/SDF, xmlrpc)
└── build_verify.sh         # Build verification script
```

## Version History

- **3.3.0** (2026) - **Real-time navigation release**
  - `cvc::nav` — a torch-free, Python-free reactive swarm-navigation
    subsystem ported from GRL-SNAM: bit-identical grid kernels (EDT /
    A* / SDF) with threaded batch variants, a fused per-agent drive,
    a `sim_world` swarm with shared/grouped/private belief planes, a
    device-resident `sim_world_cuda` GPU twin, and lock-free off-thread
    stepping (`sim_thread`).
  - A self-supervised, torch-free `CoefMLP` policy trainer (`coef_train`,
    CPU + CUDA) with a finite-difference-checked differentiable rollout,
    a surrogate/bicycle switch, the portable `.cvcnav` weight format, a
    `nav_train_demo` CLI, and a `pycvc` binding.
  - cvcGL extracted as a standalone VTK scene-graph library (persistent
    `SceneRenderer`, scene-owned lighting/shadows); native `cvc::image`
    (PNG/JPEG/WebP) and `cvc::model` (OBJ/glTF/GLB/FBX/DAE/PLY via assimp).
  - `cvc::world_clock` fixed-quantum simulation clock; direct-wrap `pycvc`
    Python bindings; module reorg onto the plain `cvc` namespace.
- **3.2.4** (2026) - **Federated distributed-state subsystem**
  - `cvc::state` replication Phases 7–10: writable transparent links,
    expiring state, delta codec, per-node telemetry with a cluster
    aggregator and routing feedback, and automatic cluster membership.
  - File-backed blob store, volume/geometry streaming via brick manifests
    with lazy hydration, and a `distributed_state_session` API.
  - Cross-platform release-archive fixes (macOS dylib/zstd, MSVC
    `_BitScanReverse64`, portable cmake exports).
- **3.2.0–3.2.3** (2026) - **Self-contained release archives**
  - Bundle all dependencies into the release archives (Linux RPATH,
    Windows PDBs); enable SDF / Mesher / CGAL; static builds and nightly
    artifacts; consume the `libcvc-deps` archive (dropping vendored
    libiimod). First coverage-expansion pass (volume I/O + utility tests).
- **3.1.1** (2026) - Bundle + verify the Windows runtime DLLs
  (CUDA/OpenMP/FFTW/zlib) in CI.
- **3.1.0** (2026) - **Singleton-less API release**
  - Removed the global `cvcapp` / `cvcstate` macros and the
    `cvc::app::instance()` / `cvc::state::instance()` zero-arg
    singletons; every caller now reaches the application context
    and state tree through an explicit `cvc::app&`.
  - Added `cvc::app::root()` member shorthand, equivalent to
    `cvc::state::instance(app)`, for ergonomic per-app state access.
  - Per-app state caching keyed on the owning `cvc::app` instance.
  - Documentation refresh across `README.md`, `docs/APP_API.md`,
    `docs/STATE_API.md`, `docs/TESTING.md`, `docs/THREAD_POOL*.md`,
    `docs/IMAGE_PROCESSING_ALGORITHMS.md`, and
    `docs/GRAPHICS_DATA_DRIVEN_UPDATES.md`.
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

