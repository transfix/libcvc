# CUDA Support Guide

## Overview

libcvc now uses modern CMake 3.17+ native CUDA language support instead of the legacy FindCUDA module. This provides better integration, simpler syntax, and improved build performance.

## Requirements

- **CMake 3.17 or higher** (when CUDA is enabled)
- **CUDA Toolkit** (tested with CUDA 10.0+, recommended 11.0+)
- **Compatible GPU** (see supported architectures below)
- **Compatible C++ compiler** for host code

## Enabling CUDA Support

### Build Configuration

Enable CUDA during CMake configuration:

```bash
cmake -B build \
  -DCVC_ENABLE_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES="75;80;86"
```

### Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `CVC_ENABLE_CUDA` | OFF | Enable CUDA support |
| `CMAKE_CUDA_STANDARD` | (matches C++ std) | CUDA C++ standard version |
| `CMAKE_CUDA_ARCHITECTURES` | "50;60;70;75;80" | Target GPU architectures |
| `CMAKE_CUDA_COMPILER` | (auto-detected) | Path to nvcc compiler |

## Supported GPU Architectures

The default configuration targets:

| Compute Capability | Architecture | Example GPUs |
|-------------------|--------------|--------------|
| 50 | Maxwell | GTX 900 series |
| 60 | Pascal | GTX 10xx series |
| 70 | Volta | Tesla V100 |
| 75 | Turing | RTX 20xx series |
| 80 | Ampere | RTX 30xx series, A100 |
| 86 | Ampere | RTX 30xx Mobile |
| 89 | Ada Lovelace | RTX 40xx series |
| 90 | Hopper | H100 |

### Customizing Target Architectures

For specific GPUs only:

```bash
# Target only RTX 3080 (Ampere, compute capability 8.6)
cmake -B build -DCVC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES="86"

# Target multiple generations
cmake -B build -DCVC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES="75;80;86;89"
```

For maximum compatibility (slower compilation):

```bash
cmake -B build -DCVC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES="50;60;70;75;80;86"
```

## Adding CUDA Source Files

### File Organization

Place CUDA source files (.cu) in the appropriate directory:

```
src/cvc/
├── algorithm.cpp
├── volume.cpp
├── cuda/
│   ├── volume_kernels.cu    # CUDA kernels
│   └── filters.cu            # GPU-accelerated filters
```

### CMakeLists.txt Integration

In `src/cvc/CMakeLists.txt`, CUDA files are automatically handled:

```cmake
# CUDA files are automatically compiled when added to SOURCE_FILES
if(CVC_ENABLE_CUDA)
  list(APPEND SOURCE_FILES
    cuda/volume_kernels.cu
    cuda/filters.cu
  )
endif()
```

### Example CUDA Source File

**cuda/volume_kernels.cu:**

```cpp
#include <cvc/volume.h>
#include <cuda_runtime.h>

namespace cvc {
namespace cuda {

// CUDA kernel
__global__ void bilateral_filter_kernel(
    float* output,
    const float* input,
    int width, int height, int depth,
    float sigma_space, float sigma_range)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;
    
    if (x >= width || y >= height || z >= depth) return;
    
    // Kernel implementation...
}

// Host function
void bilateral_filter_gpu(
    float* output,
    const float* input,
    int width, int height, int depth,
    float sigma_space, float sigma_range)
{
    // Allocate device memory
    float *d_input, *d_output;
    size_t size = width * height * depth * sizeof(float);
    
    cudaMalloc(&d_input, size);
    cudaMalloc(&d_output, size);
    
    // Copy data to device
    cudaMemcpy(d_input, input, size, cudaMemcpyHostToDevice);
    
    // Launch kernel
    dim3 blockSize(8, 8, 8);
    dim3 gridSize(
        (width + blockSize.x - 1) / blockSize.x,
        (height + blockSize.y - 1) / blockSize.y,
        (depth + blockSize.z - 1) / blockSize.z
    );
    
    bilateral_filter_kernel<<<gridSize, blockSize>>>(
        d_output, d_input,
        width, height, depth,
        sigma_space, sigma_range
    );
    
    // Copy result back
    cudaMemcpy(output, d_output, size, cudaMemcpyDeviceToHost);
    
    // Cleanup
    cudaFree(d_input);
    cudaFree(d_output);
}

} // namespace cuda
} // namespace cvc
```

### Header File Integration

**inc/cvc/volume.h:**

```cpp
#ifndef CVC_VOLUME_H
#define CVC_VOLUME_H

#ifdef CVC_USING_CUDA
namespace cvc {
namespace cuda {
  // CUDA function declarations
  void bilateral_filter_gpu(
      float* output,
      const float* input,
      int width, int height, int depth,
      float sigma_space, float sigma_range);
} // namespace cuda
} // namespace cvc
#endif // CVC_USING_CUDA

namespace cvc {
  class volume {
  public:
    void bilateral_filter(float sigma_space, float sigma_range) {
#ifdef CVC_USING_CUDA
      if (use_gpu_) {
        cuda::bilateral_filter_gpu(/* ... */);
        return;
      }
#endif
      // CPU fallback implementation
      bilateral_filter_cpu(sigma_space, sigma_range);
    }
  };
}

#endif // CVC_VOLUME_H
```

## Compile Options

### Language-Specific Options

CMake automatically handles different compile options for different languages:

```cmake
target_compile_options(cvc PRIVATE
  # C++ options
  $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra>
  
  # CUDA options
  $<$<COMPILE_LANGUAGE:CUDA>:
    --expt-relaxed-constexpr
    --expt-extended-lambda
  >
)
```

### Debug vs Release

Different optimization levels for debug and release:

```cmake
target_compile_options(cvc PRIVATE
  $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Debug>>:
    -G              # Enable device debugging
    -lineinfo       # Line number info
  >
  $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Release>>:
    --use_fast_math # Fast math optimizations
    -O3             # Maximum optimization
  >
)
```

## Linking CUDA Libraries

### Standard CUDA Runtime

Automatically linked when CUDA is enabled:

```cmake
target_link_libraries(cvc PUBLIC CUDA::cudart)
```

### Additional CUDA Libraries

For cuBLAS, cuFFT, etc.:

```cmake
if(CVC_ENABLE_CUDA)
  target_link_libraries(cvc PUBLIC
    CUDA::cublas
    CUDA::cufft
    CUDA::cusparse
  )
endif()
```

## Runtime Detection

### Checking for GPU Availability

```cpp
#include <cuda_runtime.h>

bool has_cuda_device() {
#ifdef CVC_USING_CUDA
    int device_count = 0;
    cudaError_t error = cudaGetDeviceCount(&device_count);
    return error == cudaSuccess && device_count > 0;
#else
    return false;
#endif
}
```

### Fallback to CPU

Always provide CPU fallback implementations:

```cpp
void volume::process() {
#ifdef CVC_USING_CUDA
    if (has_cuda_device() && prefer_gpu_) {
        process_gpu();
        return;
    }
#endif
    process_cpu();  // CPU fallback
}
```

## Performance Tips

### Memory Management

1. **Minimize Host-Device Transfers**
   ```cpp
   // Bad: Multiple transfers
   cudaMemcpy(d_data, h_data, size, cudaMemcpyHostToDevice);
   kernel1<<<grid, block>>>(d_data);
   cudaMemcpy(h_data, d_data, size, cudaMemcpyDeviceToHost);
   cudaMemcpy(d_data, h_data, size, cudaMemcpyHostToDevice);
   kernel2<<<grid, block>>>(d_data);
   
   // Good: Keep data on device
   cudaMemcpy(d_data, h_data, size, cudaMemcpyHostToDevice);
   kernel1<<<grid, block>>>(d_data);
   kernel2<<<grid, block>>>(d_data);
   cudaMemcpy(h_data, d_data, size, cudaMemcpyDeviceToHost);
   ```

2. **Use Pinned Memory**
   ```cpp
   float* h_data;
   cudaMallocHost(&h_data, size);  // Faster transfers
   // ... use data ...
   cudaFreeHost(h_data);
   ```

3. **Async Operations**
   ```cpp
   cudaMemcpyAsync(d_data, h_data, size, 
                   cudaMemcpyHostToDevice, stream);
   kernel<<<grid, block, 0, stream>>>(d_data);
   ```

### Kernel Optimization

1. **Coalesced Memory Access**
2. **Shared Memory Usage**
3. **Occupancy Optimization**
4. **Stream Parallelism**

## Troubleshooting

### CUDA Not Found

```bash
# Set CUDA toolkit path
export CUDA_HOME=/usr/local/cuda
export CUDACXX=/usr/local/cuda/bin/nvcc
cmake -B build -DCVC_ENABLE_CUDA=ON
```

### Compute Capability Mismatch

```bash
# Check your GPU
nvidia-smi --query-gpu=compute_cap --format=csv

# Set matching architecture
cmake -B build -DCVC_ENABLE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES="80"
```

### Compiler Incompatibility

CUDA has specific host compiler requirements. Check CUDA documentation for supported versions.

### Out of Memory

```cpp
// Check available memory
size_t free_mem, total_mem;
cudaMemGetInfo(&free_mem, &total_mem);
std::cout << "Free GPU memory: " << free_mem / (1024*1024) << " MB\n";
```

## Testing CUDA Build

### Build Test

```bash
# Configure with CUDA
cmake -B build -DCVC_ENABLE_CUDA=ON

# Build
cmake --build build -j$(nproc)

# Verify CUDA compilation
file build/lib/libcvc.so | grep -i cuda
```

### Runtime Test

```cpp
#include <iostream>
#include <cuda_runtime.h>

int main() {
    int device_count;
    cudaGetDeviceCount(&device_count);
    
    std::cout << "CUDA devices: " << device_count << "\n";
    
    for (int i = 0; i < device_count; ++i) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        std::cout << "Device " << i << ": " << prop.name << "\n";
        std::cout << "  Compute: " << prop.major << "." << prop.minor << "\n";
        std::cout << "  Memory: " << prop.totalGlobalMem / (1024*1024) << " MB\n";
    }
    
    return 0;
}
```

## Migration from Legacy FindCUDA

### Old Way (Legacy FindCUDA)

```cmake
find_package(CUDA)
cuda_add_library(mylib source.cpp kernel.cu)
cuda_add_executable(myapp main.cpp)
```

### New Way (CMake 3.17+)

```cmake
project(myproject LANGUAGES CXX CUDA)
add_library(mylib source.cpp kernel.cu)
add_executable(myapp main.cpp)
```

Key differences:
- CUDA is now a first-class language
- No special `cuda_add_*` functions needed
- Better IDE integration
- Simpler syntax
- Automatic dependency handling

## Resources

- [CMake CUDA Documentation](https://cmake.org/cmake/help/latest/manual/cmake-language.7.html#cuda)
- [NVIDIA CUDA Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/)
- [CMake CUDA Support Notes](https://cmake.org/cmake/help/latest/prop_tgt/CUDA_ARCHITECTURES.html)

## Summary

Modern CUDA support in libcvc:
- ✅ Uses CMake 3.17+ native CUDA language
- ✅ Automatic nvcc integration
- ✅ Target-specific architecture compilation
- ✅ Mixed C++/CUDA source files in same target
- ✅ Proper device code linking
- ✅ Generator expressions for compile options
- ✅ Standard CMake target properties

The legacy FindCUDA module has been completely replaced with modern CMake CUDA language support.
