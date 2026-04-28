# CUDA Modernization Summary

## Table of Contents

- [What Changed](#what-changed)
- [Key Changes](#key-changes)
- [Before and After](#before-and-after)
- [Usage Example](#usage-example)
  - [Adding CUDA Source Files](#adding-cuda-source-files)
- [Documentation](#documentation)
- [Build Examples](#build-examples)
- [Testing CUDA Setup](#testing-cuda-setup)
- [Migration Notes](#migration-notes)
- [Requirements](#requirements)
- [Compiler Support](#compiler-support)
- [Performance Notes](#performance-notes)
- [Future Enhancements](#future-enhancements)
- [Summary](#summary)

## What Changed

The libcvc project's CUDA support has been completely modernized from the legacy FindCUDA module (pre-CMake 3.17) to CMake's native CUDA language support.

## Key Changes

### 1. **CMake Files Updated**

- **CMakeLists.txt** - Added CUDA language to project, configuration options
- **src/CMakeLists.txt** - Removed legacy `cuda_build_clean_target()`
- **src/cvc/CMakeLists.txt** - Added CUDA source file handling and properties
- **CMake/SetupCUDA.cmake** - New helper function for CUDA configuration

### 2. **New Build Option**

```bash
# Enable CUDA support
cmake -B build -DCVC_ENABLE_CUDA=ON

# Specify target GPU architectures
cmake -B build \
  -DCVC_ENABLE_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES="75;80;86"
```

### 3. **Automatic Features**

When `CVC_ENABLE_CUDA=ON`:
- ✅ CUDA language added to project
- ✅ CMake 3.17+ automatically enforced
- ✅ `.cu` files automatically compiled with nvcc
- ✅ CUDA runtime automatically linked
- ✅ `CVC_USING_CUDA` definition added
- ✅ Proper device code linking and separation

### 4. **Target GPU Architectures**

Default architectures (customizable):
- **50** - Maxwell (GTX 900 series)
- **60** - Pascal (GTX 10xx)
- **70** - Volta (Tesla V100)
- **75** - Turing (RTX 20xx)
- **80** - Ampere (RTX 30xx, A100)

### 5. **CMake Presets Added**

```bash
# Release build with CUDA
cmake --preset cuda

# Debug build with CUDA
cmake --preset cuda-debug
```

## Before and After

### Before (Legacy FindCUDA)

```cmake
find_package(CUDA)
if(CUDA_FOUND)
  cuda_add_library(mylib source.cpp)
  cuda_add_executable(myapp main.cpp)
  
  # Manual configuration needed
  set(CUDA_NVCC_FLAGS "-arch=sm_50")
  cuda_build_clean_target()
endif()
```

**Problems:**
- Manual configuration complex
- Separate functions for CUDA targets
- Poor IDE integration
- Deprecated since CMake 3.17

### After (Modern CMake CUDA)

```cmake
option(CVC_ENABLE_CUDA "Enable CUDA" OFF)

if(CVC_ENABLE_CUDA)
  project(libcvc LANGUAGES C CXX CUDA)
endif()

add_library(cvc source.cpp kernel.cu)  # Mixed sources work!

if(CVC_ENABLE_CUDA)
  set_target_properties(cvc PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
  )
endif()
```

**Benefits:**
- ✅ CUDA is a first-class language
- ✅ Standard CMake targets
- ✅ Automatic dependency handling
- ✅ Better IDE support (VS Code, CLion)
- ✅ Generator expressions for CUDA-specific options

## Usage Example

### Adding CUDA Source Files

**1. Create CUDA source file:**

```cuda
// src/cvc/cuda/volume_kernels.cu
#include <cuda_runtime.h>

__global__ void process_kernel(float* data, int size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        data[idx] *= 2.0f;
    }
}

void process_volume_gpu(float* data, int size) {
    float* d_data;
    cudaMalloc(&d_data, size * sizeof(float));
    cudaMemcpy(d_data, data, size * sizeof(float), cudaMemcpyHostToDevice);
    
    int threads = 256;
    int blocks = (size + threads - 1) / threads;
    process_kernel<<<blocks, threads>>>(d_data, size);
    
    cudaMemcpy(data, d_data, size * sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(d_data);
}
```

**2. Update CMakeLists.txt:**

```cmake
# In src/cvc/CMakeLists.txt
if(CVC_ENABLE_CUDA)
  list(APPEND SOURCE_FILES
    cuda/volume_kernels.cu
  )
endif()
```

**3. Use in C++ code:**

```cpp
// volume.cpp
#ifdef CVC_USING_CUDA
extern void process_volume_gpu(float* data, int size);
#endif

void volume::process() {
#ifdef CVC_USING_CUDA
    if (use_gpu) {
        process_volume_gpu(data_, size_);
        return;
    }
#endif
    // CPU fallback
    process_cpu();
}
```

## Documentation

Complete documentation available in:
- **CUDA_GUIDE.md** - Comprehensive CUDA usage guide
  - Requirements and setup
  - Adding CUDA source files
  - Performance optimization
  - Troubleshooting
  - Code examples

## Build Examples

### Basic CUDA Build
```bash
cmake -B build -DCVC_ENABLE_CUDA=ON
cmake --build build -j$(nproc)
```

### CUDA with Specific GPU
```bash
# For RTX 3080 (Ampere, compute capability 8.6)
cmake -B build \
  -DCVC_ENABLE_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES="86"
```

### Full Featured with CUDA
```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCVC_ENABLE_CUDA=ON \
  -DCVC_USING_HDF5=ON \
  -DCVC_ENABLE_MESHER=ON \
  -DCVC_ENABLE_SDF=ON
```

## Testing CUDA Setup

```bash
# Check CUDA availability
nvidia-smi

# Check compute capability
nvidia-smi --query-gpu=compute_cap --format=csv

# Build with CUDA
cmake --preset cuda
cmake --build --preset cuda

# Verify CUDA in binary
strings build/lib/libcvc.so | grep -i cuda
```

## Migration Notes

### For Existing Code

No changes needed if you weren't using CUDA:
- Default is `CVC_ENABLE_CUDA=OFF`
- No impact on existing builds
- CUDA is opt-in only

### For New CUDA Development

1. Enable CUDA: `-DCVC_ENABLE_CUDA=ON`
2. Add `.cu` files to `SOURCE_FILES`
3. Use `#ifdef CVC_USING_CUDA` for conditional compilation
4. Always provide CPU fallback

## Requirements

### When CUDA Disabled (Default)
- CMake 3.15+
- No CUDA toolkit needed

### When CUDA Enabled
- CMake 3.17+ (automatically enforced)
- CUDA Toolkit 10.0+ (11.0+ recommended)
- Compatible GPU
- Compatible host compiler (check CUDA docs)

## Compiler Support

| CUDA Version | GCC | Clang | MSVC |
|--------------|-----|-------|------|
| 10.0 | ≤7 | ≤6 | 2017 |
| 11.0 | ≤9 | ≤10 | 2019 |
| 11.5 | ≤11 | ≤12 | 2019 |
| 12.0 | ≤12 | ≤14 | 2022 |

## Performance Notes

- Mixed C++/CUDA compilation is seamless
- Proper device code separation enabled
- Debug symbols available with `-G` flag
- Fast math optimizations in release mode
- Separable compilation for better modularity

## Future Enhancements

Potential areas for GPU acceleration:
1. **Volume filtering** - Bilateral, anisotropic diffusion
2. **Isosurfacing** - GPU-accelerated marching cubes
3. **SDF calculation** - Parallel distance field computation
4. **Image processing** - Contrast enhancement, smoothing
5. **Geometry operations** - Point cloud processing

All infrastructure is ready - just add `.cu` files!

## Summary

✅ **Completed:**
- Modern CMake 3.17+ CUDA support
- Flexible architecture targeting
- Proper compile/link separation
- Comprehensive documentation
- CMake presets for CUDA builds
- Helper functions for configuration

🎯 **Ready for:**
- Adding GPU-accelerated algorithms
- CUDA kernel implementation
- Performance optimization
- Multi-GPU support

📚 **Resources:**
- See **CUDA_GUIDE.md** for detailed usage
- Check CMakePresets.json for build configurations

The CUDA infrastructure is now production-ready and follows modern CMake best practices!
