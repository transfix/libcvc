# SDF Library Refactoring - Thread-Safe and Memory-Safe Design

> **⚠️ DEPRECATED**: This refactoring document has been consolidated.
> 
> **Please see**: [docs/SDF_LIBRARY.md](../../../../docs/SDF_LIBRARY.md)
> 
> The comprehensive SDF documentation includes:
> - Complete refactoring history and technical details
> - Migration guide from v1.x to v2.0
> - Performance improvements (11x speedup)
> - Thread-safe architecture explanation
> - Test results and coverage
> - Future CUDA implementation plans
>
> This standalone refactoring document is no longer maintained.

---

## Overview

The SDF library has been refactored to eliminate global state and use modern C++ memory management. This makes the library:
1. **Thread-safe**: Multiple SDF computations can run in parallel
2. **Memory-safe**: Smart pointers eliminate memory leaks
3. **RAII-compliant**: Automatic resource cleanup
4. **Maintainable**: Clearer ownership semantics

## Key Changes

### 1. SDFContext Class

All global variables have been moved into the `SDFContext` class:

**Before (global state)**:
```cpp
namespace SDFLibrary {
    extern triangle* surface;
    extern myVert* vertices;
    extern myPoint* normals;
    extern cell*** sdf;
    extern voxel* values;
    extern int size, total_points, total_triangles;
    // ... many more globals
}
```

**After (encapsulated state)**:
```cpp
class SDFContext {
public:
    std::unique_ptr<triangle[]> surface;
    std::unique_ptr<myVert[]> vertices;
    std::unique_ptr<myPoint[]> normals;
    std::unique_ptr<cell***, OctreeDeleter> sdf;
    std::unique_ptr<voxel[]> voxel_values;
    int size, total_points, total_triangles;
    // ... all state is instance-specific
};
```

### 2. Smart Pointers

Replaced raw pointers with smart pointers:
- `std::unique_ptr<T[]>` for arrays
- Custom deleters for complex structures (octree)
- Automatic memory management (no manual `free()` or `delete[]`)

### 3. New Thread-Safe API

**Recommended for new code**:
```cpp
std::unique_ptr<float[]> computeSDF_MT(
    int nverts, const float* verts,
    int ntris, const int* tris,
    int size, int isNormalFlip,
    const float* mins, const float* maxs);
```

**Manual control**:
```cpp
auto ctx = SDFLibrary::createContext();
ctx->setParameters(size, flipNormals, mins, maxs);
ctx->initSDF();
ctx->readGeom(nverts, verts, ntris, tris);
ctx->adjustData();
ctx->compute();
float* result = ctx->getValues();
```

### 4. Backward Compatibility

The old global API is maintained for backward compatibility but marked as deprecated:
```cpp
// Still works, but uses thread-local storage
SDFLibrary::setParameters(size, flipNormals, mins, maxs);
float* values = SDFLibrary::computeSDF(nverts, verts, ntris, tris);
```

## Migration Guide

### For Simple Use Cases

**Before**:
```cpp
SDFLibrary::setParameters(size, flipNormals, mins, maxs);
float* values = SDFLibrary::computeSDF(nverts, verts, ntris, tris);
// ... use values ...
delete[] values;  // Manual cleanup
```

**After**:
```cpp
auto values = SDFLibrary::computeSDF_MT(nverts, verts, ntris, tris,
                                         size, flipNormals, mins, maxs);
// ... use values ...
// Automatic cleanup when values goes out of scope
```

### For Multi-threaded Use

**Before** (NOT thread-safe):
```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
    SDFLibrary::setParameters(...);  // RACE CONDITION!
    float* v = SDFLibrary::computeSDF(...);
    delete[] v;
}
```

**After** (thread-safe):
```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
    auto v = SDFLibrary::computeSDF_MT(...);  // Each thread has own context
    // No race conditions!
}
```

## Memory Leak Fixes

### Issues Found

1. **Octree linked lists**: Not properly freed in all code paths
2. **Global arrays**: Leaked if computation failed midway
3. **myVert destructor**: std::vector members not properly destroyed with `free()`

### Solutions

1. **Smart pointers**: Automatic cleanup even on exceptions
2. **RAII**: Resources tied to object lifetime
3. **Proper destructors**: `delete[]` for C++ objects with destructors

## Testing Recommendations

1. Run existing tests to ensure backward compatibility
2. Add multi-threaded tests using `computeSDF_MT()`
3. Use memory leak detection tools (valgrind, AddressSanitizer)
4. Benchmark to ensure no performance regression

## Future Improvements

1. GPU/CUDA support can use same context pattern
2. Add progress callbacks to SDFContext
3. Consider std::span for input arrays (C++20)
4. Move to std::array for fixed-size arrays
5. Add move semantics for zero-copy result transfer

## Files Modified

- `SDFContext.h` (new): Context class definition
- `SDFContext.cpp` (new): Context implementation
- `head.h`: Modernized structs, added constructors
- `common.h`: Added SDFContext include, marked globals deprecated
- `common.cpp`: Updated utility functions
- `sdfLib.h`: New thread-safe API
- `main.cpp`: Implementation of new API
- All `.cpp` files: Updated to use context (ongoing)

## Compilation

The refactoring maintains ABI compatibility with existing code. No changes needed to build system.
