# Week 5: CUDA Unified Memory Integration

## Summary

Week 5 extends the Week 4 geoframe_adapter architecture to achieve **true zero-copy** data sharing between CPU (LBIE algorithms) and GPU (CUDA kernels) using CUDA unified memory. This leverages the existing unified memory implementation from `voxels.cpp` to eliminate all data copying overhead.

## Goals

1. **Zero-copy geometry access** - GPU and CPU share same memory
2. **GPU-accelerated algorithms** - Point location, quality metrics, mesh refinement
3. **Transparent migration** - Existing code works without modification
4. **Flexible execution** - Per-operation CPU/GPU selection

## Architecture Overview

### Week 4 (Current State)
```
CVC API (geometry) → to_geoframe() → LBIE (geoframe) → to_geometry() → CVC API
                     ↑ COPY ONCE ↑                     ↑ COPY ONCE ↑
                     
Performance: 50% reduction vs Weeks 1-3 (1 copy instead of 2)
Memory: 33% reduction (2 allocations instead of 3)
```

### Week 5 (Target State)
```
CVC API (geometry) → geoframe_adapter → LBIE (geoframe) → geoframe_adapter → CVC API
                     ↑ ZERO COPY   ↑                      ↑ ZERO COPY   ↑
                     
Performance: 100% reduction (0 copies)
Memory: CUDA unified memory shared between CPU/GPU
GPU Access: Same data accessible from CUDA kernels
```

## Key Components

### 1. CUDA Unified Memory Foundation

**Inspiration:** Existing `voxels` class implementation

**Pattern from voxels.h/cpp:**
```cpp
class voxels {
private:
    // CPU memory (default)
    boost::shared_array<unsigned char> _voxels;
    
    // CUDA unified memory (when enabled)
    bool _using_cuda;
    int _cuda_device_id;
    std::shared_ptr<void> _cuda_unified_ptr;  // ← Reference counted!
    
    // Custom deleter for automatic cleanup
    struct CudaManagedDeleter {
        void operator()(void* ptr) const {
            if (ptr) cudaFree(ptr);  // Automatic when refcount hits 0
        }
    };
    
    // Unified data access
    byte* get_data_ptr() {
#ifdef CVC_USING_CUDA
        if (_using_cuda && _cuda_unified_ptr) {
            return reinterpret_cast<byte*>(_cuda_unified_ptr.get());
        }
#endif
        return _voxels.get();
    }
};
```

**Key Insights:**
- ✅ `std::shared_ptr` with custom deleter for automatic CUDA memory management
- ✅ Reference counting enables safe shallow copying (multiple owners)
- ✅ Transparent CPU/GPU access via `get_data_ptr()`
- ✅ Automatic cleanup when last reference is destroyed

### 2. Geometry with CUDA Unified Memory

**Goal:** Make geometry vectors use CUDA unified memory with zero code changes for users.

#### 2.1 Custom Allocator

```cpp
// inc/cvc/cuda_allocator.h
namespace CVC_NAMESPACE {

#ifdef CVC_USING_CUDA

template<typename T>
class cuda_unified_allocator {
public:
    typedef T value_type;
    
    cuda_unified_allocator() = default;
    
    template<typename U>
    cuda_unified_allocator(const cuda_unified_allocator<U>&) {}
    
    T* allocate(std::size_t n) {
        T* ptr = nullptr;
        cudaError_t err = cudaMallocManaged(&ptr, n * sizeof(T));
        if (err != cudaSuccess) {
            throw cuda_error("cudaMallocManaged failed: " + 
                           std::string(cudaGetErrorString(err)));
        }
        return ptr;
    }
    
    void deallocate(T* ptr, std::size_t) {
        if (ptr) {
            cudaFree(ptr);
        }
    }
    
    template<typename U>
    bool operator==(const cuda_unified_allocator<U>&) const { return true; }
    
    template<typename U>
    bool operator!=(const cuda_unified_allocator<U>&) const { return false; }
};

#endif // CVC_USING_CUDA

} // namespace CVC_NAMESPACE
```

#### 2.2 Geometry CUDA Support

**Pattern:** Follow voxels' dual-mode approach (CPU memory by default, CUDA on demand)

```cpp
// inc/cvc/geometry.h (additions)
class geometry {
public:
    // Existing typedefs (unchanged)
    typedef std::vector<point_t>    points_t;
    typedef std::vector<tet_t>      tets_t;
    typedef std::vector<hex_t>      hexs_t;
    // ... etc

    // CUDA unified memory support (NEW)
    
    // Enable CUDA unified memory for this geometry
    void enableCUDA(int device_id = -1);
    
    // Disable CUDA and migrate back to system RAM
    void disableCUDA();
    
    // Check if currently using CUDA unified memory
    bool using_cuda() const { return _using_cuda; }
    int cuda_device_id() const { return _cuda_device_id; }
    
    // Get raw pointers for GPU kernel access
    point_t* points_data() { 
        return _cuda_points_ptr ? 
            reinterpret_cast<point_t*>(_cuda_points_ptr.get()) : 
            _points->data(); 
    }
    
    tet_t* tets_data() { 
        return _cuda_tets_ptr ? 
            reinterpret_cast<tet_t*>(_cuda_tets_ptr.get()) : 
            _tets->data(); 
    }
    
    // ... similar for other arrays

private:
    // Existing members (unchanged)
    points_ptr_t _points;
    tets_ptr_t _tets;
    hexs_ptr_t _hexs;
    // ... etc
    
    // CUDA unified memory state (NEW - following voxels pattern)
#ifdef CVC_USING_CUDA
    bool _using_cuda;
    int _cuda_device_id;
    
    // Reference-counted unified memory pointers
    std::shared_ptr<void> _cuda_points_ptr;
    std::shared_ptr<void> _cuda_tets_ptr;
    std::shared_ptr<void> _cuda_hexs_ptr;
    // ... etc for all arrays
    
    // Custom deleter (same pattern as voxels)
    struct CudaManagedDeleter {
        void operator()(void* ptr) const {
            if (ptr) cudaFree(ptr);
        }
    };
    
    // Helper methods
    void allocate_cuda_arrays();
    void migrate_to_cuda(int device_id);
    void migrate_from_cuda();
#endif
};
```

#### 2.3 Migration Implementation

**Pattern from voxels.cpp:**
```cpp
void geometry::enableCUDA(int device_id) {
#ifdef CVC_USING_CUDA
    if (_using_cuda) {
        // Already using CUDA - maybe switch devices?
        if (device_id >= 0 && device_id != _cuda_device_id) {
            switchGPU(device_id);
        }
        return;
    }
    
    if (!cuda_device_manager::cuda_available()) {
        throw cuda_not_available("CUDA not available on this system");
    }
    
    // Use current device if not specified
    if (device_id < 0) {
        device_id = cuda_device_manager::get_current_device();
        if (device_id < 0) device_id = 0;
    }
    
    // Validate device
    if (device_id >= cuda_device_manager::device_count()) {
        throw cuda_error("Invalid CUDA device ID");
    }
    
    if (!cuda_device_manager::supports_unified_memory(device_id)) {
        throw cuda_error("Device does not support unified memory");
    }
    
    // Migrate data to CUDA unified memory
    migrate_to_cuda(device_id);
    
    _using_cuda = true;
    _cuda_device_id = device_id;
    
    cvcapp.log(3, "CUDA unified memory enabled for geometry on device " + 
               std::to_string(device_id));
#else
    throw cuda_not_available("CVC was not compiled with CUDA support");
#endif
}

void geometry::migrate_to_cuda(int device_id) {
#ifdef CVC_USING_CUDA
    // Set device
    int old_device = cuda_device_manager::get_current_device();
    cuda_device_manager::set_current_device(device_id);
    
    // Migrate points
    if (_points && !_points->empty()) {
        size_t byte_size = _points->size() * sizeof(point_t);
        void* ptr = nullptr;
        CUDA_CHECK(cudaMallocManaged(&ptr, byte_size));
        std::memcpy(ptr, _points->data(), byte_size);
        _cuda_points_ptr = std::shared_ptr<void>(ptr, CudaManagedDeleter());
    }
    
    // Migrate tets
    if (_tets && !_tets->empty()) {
        size_t byte_size = _tets->size() * sizeof(tet_t);
        void* ptr = nullptr;
        CUDA_CHECK(cudaMallocManaged(&ptr, byte_size));
        std::memcpy(ptr, _tets->data(), byte_size);
        _cuda_tets_ptr = std::shared_ptr<void>(ptr, CudaManagedDeleter());
    }
    
    // ... similar for hexs, normals, colors, etc.
    
    // Synchronize to ensure data is uploaded
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // Restore device
    cuda_device_manager::set_current_device(old_device);
#endif
}

void geometry::disableCUDA() {
#ifdef CVC_USING_CUDA
    if (!_using_cuda) return;
    
    migrate_from_cuda();
    
    _using_cuda = false;
    _cuda_device_id = -1;
    
    cvcapp.log(3, "CUDA unified memory disabled for geometry");
#endif
}

void geometry::migrate_from_cuda() {
#ifdef CVC_USING_CUDA
    if (!_using_cuda) return;
    
    // Ensure data is synchronized to host
    if (_cuda_device_id >= 0) {
        int old_device = cuda_device_manager::get_current_device();
        cuda_device_manager::set_current_device(_cuda_device_id);
        CUDA_CHECK(cudaDeviceSynchronize());
        cuda_device_manager::set_current_device(old_device);
    }
    
    // Copy CUDA data back to std::vectors
    if (_cuda_points_ptr && _points) {
        std::memcpy(_points->data(), _cuda_points_ptr.get(), 
                   _points->size() * sizeof(point_t));
        _cuda_points_ptr.reset();  // Release CUDA memory
    }
    
    if (_cuda_tets_ptr && _tets) {
        std::memcpy(_tets->data(), _cuda_tets_ptr.get(), 
                   _tets->size() * sizeof(tet_t));
        _cuda_tets_ptr.reset();
    }
    
    // ... similar for other arrays
#endif
}
```

### 3. Zero-Copy geoframe_adapter

**Current Week 4 Implementation:**
```cpp
void geoframe_adapter::sync_from_geometry() {
    // Week 4: Copy data from geometry to geoframe
    numverts = points.size();
    verts.resize(numverts);
    for (size_t i = 0; i < points.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            verts[i][j] = static_cast<float>(points[i][j]);  // COPY!
        }
    }
    // ... more copying for tris, quads, normals, etc.
}
```

**Week 5 Zero-Copy Implementation:**

#### 3.1 geoframe with Raw Pointers

```cpp
// LBIE/geoframe_unified.h (new variant)
namespace LBIE {

// Zero-copy geoframe variant using raw pointers to unified memory
class geoframe_unified : public geoframe_base {
public:
    geoframe_unified() 
        : verts_ptr(nullptr), verts_size(0),
          tris_ptr(nullptr), tris_size(0),
          owns_data(false) {}
    
    virtual ~geoframe_unified() {
        // Don't free - we don't own the data
        // Ownership remains with geometry's shared_ptr
    }
    
    // Access methods (interface compatible with geoframe)
    size_t numverts() const { return verts_size; }
    size_t numtriangles() const { return tris_size; }
    
    // Direct pointer access for LBIE algorithms
    float* verts_data() { return reinterpret_cast<float*>(verts_ptr); }
    unsigned int* tris_data() { return reinterpret_cast<unsigned int*>(tris_ptr); }
    
    // Indexing operators (compatible with geoframe)
    float_3& vert(size_t i) { 
        assert(i < verts_size);
        return verts_ptr[i]; 
    }
    
    tri_uint& triangle(size_t i) { 
        assert(i < tris_size);
        return tris_ptr[i]; 
    }
    
private:
    // Raw pointers to CUDA unified memory (NOT owned)
    float_3* verts_ptr;
    size_t verts_size;
    
    tri_uint* tris_ptr;
    size_t tris_size;
    
    // ... similar for quads, normals, colors, etc.
    
    bool owns_data;  // Always false for unified memory
    
    friend class geoframe_adapter;
};

} // namespace LBIE
```

#### 3.2 Zero-Copy Adapter

```cpp
// LBIE/geoframe_adapter.h (Week 5 updates)
namespace LBIE {

class geoframe_adapter {
public:
    explicit geoframe_adapter(CVC_NAMESPACE::geometry& geom) 
        : _geom(geom), _dirty(false) {
        
#ifdef CVC_USING_CUDA
        // If geometry is using CUDA, use zero-copy mode
        if (geom.using_cuda()) {
            sync_from_geometry_zerocopy();
            return;
        }
#endif
        // Fallback: copy mode for non-CUDA geometry
        sync_from_geometry_copy();
    }
    
    ~geoframe_adapter() {
#ifdef CVC_USING_CUDA
        if (_geom.using_cuda()) {
            sync_to_geometry_zerocopy();
            return;
        }
#endif
        sync_to_geometry_copy();
    }
    
private:
#ifdef CVC_USING_CUDA
    void sync_from_geometry_zerocopy() {
        // Week 5: Zero-copy - just assign pointers!
        
        // Points → verts (NO COPY)
        // Challenge: geometry uses double[3], geoframe uses float[3]
        // Solution: Use CUDA kernel to create float view in-place OR
        //           modify geoframe to accept double* directly
        
        // For now: assume we've modified geoframe to use double internally
        // OR: we use a thin float wrapper over double data
        
        _unified_geoframe.verts_ptr = 
            reinterpret_cast<float_3*>(_geom.points_data());
        _unified_geoframe.verts_size = _geom.num_points();
        
        // Tets/Hexs → surface tris/quads
        // Challenge: Need to encode volume elements as surface faces
        // Solution: Build face index array in unified memory
        
        extract_surface_to_unified_memory();
        
        _unified_geoframe.owns_data = false;
        _dirty = false;
    }
    
    void sync_to_geometry_zerocopy() {
        // Week 5: Zero-copy - data already modified in-place!
        
        if (!_dirty) return;
        
        // Geometry's unified memory was modified directly by LBIE
        // No copy needed - changes are already visible
        
        // Only need to update metadata (vertex count, etc.)
        // if LBIE resized arrays
        
        _dirty = false;
    }
#endif
    
    void sync_from_geometry_copy() {
        // Week 4 implementation (fallback for CPU-only mode)
        // ... existing copy code ...
    }
    
    void sync_to_geometry_copy() {
        // Week 4 implementation (fallback)
        // ... existing copy code ...
    }
    
    void extract_surface_to_unified_memory() {
        // Extract boundary faces from tets/hexs
        // Allocate face index array in CUDA unified memory
        // Build face list (can use GPU kernel for this!)
        
#ifdef CVC_USING_CUDA
        if (_geom.get_geometry_type() == CVC_NAMESPACE::geometry::VOLUME_TET) {
            // Use CUDA kernel to extract tet faces
            extract_tet_faces_cuda(
                _geom.tets_data(), _geom.num_tets(),
                _geom.boundary().data(),
                &_unified_geoframe.tris_ptr,  // output
                &_unified_geoframe.tris_size
            );
        } else if (_geom.get_geometry_type() == CVC_NAMESPACE::geometry::VOLUME_HEX) {
            // Use CUDA kernel to extract hex faces
            extract_hex_faces_cuda(
                _geom.hexs_data(), _geom.num_hexs(),
                _geom.boundary().data(),
                &_unified_geoframe.quads_ptr,  // output
                &_unified_geoframe.quads_size
            );
        }
#endif
    }
    
    CVC_NAMESPACE::geometry& _geom;
    geoframe_unified _unified_geoframe;  // Zero-copy variant
    bool _dirty;
};

} // namespace LBIE
```

### 4. CUDA Kernel Examples

#### 4.1 Extract Tet Boundary Faces

```cpp
// LBIE/geometry_kernels.cu (new file)
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/unique.h>
#include <thrust/sort.h>

namespace LBIE {

// Each tet has 4 faces: (0,1,2), (0,1,3), (0,2,3), (1,2,3)
__global__ void extract_tet_faces_kernel(
    const uint64_t* tets,         // [num_tets * 4]
    const bool* boundary_flags,   // [num_vertices]
    uint64_t num_tets,
    unsigned int* face_buffer,    // [num_tets * 4 * 3] (output)
    int* face_valid               // [num_tets * 4] (output)
) {
    uint64_t tet_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tet_idx >= num_tets) return;
    
    const uint64_t* tet = &tets[tet_idx * 4];
    
    // Four faces per tet
    const int faces[4][3] = {
        {0, 1, 2},
        {0, 1, 3},
        {0, 2, 3},
        {1, 2, 3}
    };
    
    for (int f = 0; f < 4; ++f) {
        uint64_t v0 = tet[faces[f][0]];
        uint64_t v1 = tet[faces[f][1]];
        uint64_t v2 = tet[faces[f][2]];
        
        // Check if all vertices are on boundary
        bool on_boundary = boundary_flags[v0] && 
                          boundary_flags[v1] && 
                          boundary_flags[v2];
        
        uint64_t out_idx = tet_idx * 4 + f;
        
        if (on_boundary) {
            face_buffer[out_idx * 3 + 0] = static_cast<unsigned int>(v0);
            face_buffer[out_idx * 3 + 1] = static_cast<unsigned int>(v1);
            face_buffer[out_idx * 3 + 2] = static_cast<unsigned int>(v2);
            face_valid[out_idx] = 1;
        } else {
            face_valid[out_idx] = 0;
        }
    }
}

void extract_tet_faces_cuda(
    const void* tets_data,
    size_t num_tets,
    const bool* boundary_flags,
    unsigned int** output_faces,
    size_t* output_count
) {
    // Allocate temporary buffers
    size_t max_faces = num_tets * 4;
    
    unsigned int* face_buffer;
    int* face_valid;
    cudaMallocManaged(&face_buffer, max_faces * 3 * sizeof(unsigned int));
    cudaMallocManaged(&face_valid, max_faces * sizeof(int));
    
    // Launch kernel
    int threads = 256;
    int blocks = (num_tets + threads - 1) / threads;
    
    extract_tet_faces_kernel<<<blocks, threads>>>(
        reinterpret_cast<const uint64_t*>(tets_data),
        boundary_flags,
        num_tets,
        face_buffer,
        face_valid
    );
    
    cudaDeviceSynchronize();
    
    // Compact valid faces using thrust
    // (Remove invalid faces where face_valid[i] == 0)
    // ... thrust::remove_if ...
    
    *output_faces = face_buffer;
    *output_count = /* compacted count */;
}

} // namespace LBIE
```

#### 4.2 GPU-Accelerated Point Location

```cpp
// LBIE/point_location_kernels.cu (new file)
namespace LBIE {

__device__ bool point_in_tet(
    const double* point,
    const double* v0,
    const double* v1,
    const double* v2,
    const double* v3
) {
    // Barycentric coordinate test
    // ... compute determinants ...
    // Return true if all barycentric coords >= 0
}

__global__ void find_containing_tets_kernel(
    const double* points,          // [num_points * 3]
    uint64_t num_points,
    const double* vertices,        // [num_verts * 3]
    const uint64_t* tets,          // [num_tets * 4]
    uint64_t num_tets,
    int* results                   // [num_points] (output: tet index or -1)
) {
    uint64_t point_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (point_idx >= num_points) return;
    
    const double* p = &points[point_idx * 3];
    
    // Brute-force search (can optimize with BVH/octree later)
    for (uint64_t tet_idx = 0; tet_idx < num_tets; ++tet_idx) {
        const uint64_t* tet = &tets[tet_idx * 4];
        
        const double* v0 = &vertices[tet[0] * 3];
        const double* v1 = &vertices[tet[1] * 3];
        const double* v2 = &vertices[tet[2] * 3];
        const double* v3 = &vertices[tet[3] * 3];
        
        if (point_in_tet(p, v0, v1, v2, v3)) {
            results[point_idx] = tet_idx;
            return;
        }
    }
    
    results[point_idx] = -1;  // Not found
}

} // namespace LBIE
```

#### 4.3 GPU Quality Metrics

```cpp
// LBIE/quality_kernels.cu (new file)
namespace LBIE {

__device__ double tet_quality_aspect_ratio(
    const double* v0,
    const double* v1,
    const double* v2,
    const double* v3
) {
    // Compute edge lengths
    double edges[6];
    // ... compute distances ...
    
    // Compute volume
    double volume;
    // ... determinant ...
    
    // Aspect ratio = (longest edge)^3 / volume
    double max_edge = /* ... */;
    return (max_edge * max_edge * max_edge) / volume;
}

__global__ void compute_tet_quality_kernel(
    const double* vertices,
    const uint64_t* tets,
    uint64_t num_tets,
    double* qualities  // [num_tets] (output)
) {
    uint64_t tet_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (tet_idx >= num_tets) return;
    
    const uint64_t* tet = &tets[tet_idx * 4];
    
    const double* v0 = &vertices[tet[0] * 3];
    const double* v1 = &vertices[tet[1] * 3];
    const double* v2 = &vertices[tet[2] * 3];
    const double* v3 = &vertices[tet[3] * 3];
    
    qualities[tet_idx] = tet_quality_aspect_ratio(v0, v1, v2, v3);
}

// Host interface
std::vector<double> compute_tet_qualities_cuda(
    const CVC_NAMESPACE::geometry& geom
) {
    if (!geom.using_cuda()) {
        throw std::runtime_error("Geometry must be using CUDA unified memory");
    }
    
    size_t num_tets = geom.num_tets();
    
    double* qualities;
    cudaMallocManaged(&qualities, num_tets * sizeof(double));
    
    int threads = 256;
    int blocks = (num_tets + threads - 1) / threads;
    
    compute_tet_quality_kernel<<<blocks, threads>>>(
        reinterpret_cast<const double*>(geom.points_data()),
        reinterpret_cast<const uint64_t*>(geom.tets_data()),
        num_tets,
        qualities
    );
    
    cudaDeviceSynchronize();
    
    std::vector<double> result(qualities, qualities + num_tets);
    cudaFree(qualities);
    
    return result;
}

} // namespace LBIE
```

### 5. Type Conversion Strategies

**Challenge:** geometry uses `double`, geoframe uses `float`

**Option 1: In-Place Conversion (Temporary Buffer)**
```cpp
// Create float view of double data in unified memory
template<typename T>
struct unified_view {
    std::shared_ptr<void> _storage;  // Actual storage
    size_t _size;
    
    // Convert double* to float* on-demand
    static unified_view<float> from_double_array(
        std::shared_ptr<void> double_ptr, 
        size_t count
    ) {
        float* float_buffer;
        cudaMallocManaged(&float_buffer, count * sizeof(float));
        
        // GPU kernel to convert double→float
        convert_double_to_float<<<...>>>(
            reinterpret_cast<const double*>(double_ptr.get()),
            float_buffer,
            count
        );
        
        return unified_view<float>(float_buffer, count);
    }
};
```

**Option 2: Modify geoframe to Support Double**
```cpp
// LBIE/geoframe_unified.h
template<typename FloatType = float>
class geoframe_unified_t {
    FloatType* verts_ptr;  // Can be float or double
    // ...
};

using geoframe_unified = geoframe_unified_t<float>;   // Default
using geoframe_unified_d = geoframe_unified_t<double>; // For geometry

// LBIE algorithms templated on FloatType
template<typename FloatType>
void Octree::mesh_extract(geoframe_unified_t<FloatType>& gf, ...) {
    // Works with both float and double
}
```

**Recommendation: Option 2** (cleaner, more flexible, future-proof)

### 6. API Usage Examples

#### 6.1 Basic Usage (CPU Mode - Backward Compatible)

```cpp
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <mesher.h>

// Works exactly like Week 4 (one-copy mode)
CVC::volume vol("input.vol");
CVC::geometry mesh = LBIE::do_mesh_geometry(
    vol, 128.0, 255.0, 0.01, 1.0,
    CVC::geometry::VOLUME_TET,
    LBIE::Mesher::GEO_FLOW,
    "bspline_convolution",
    LBIE::Mesher::DUALLIB,
    5,
    false, false
);

// Result: 1 copy (same as Week 4)
```

#### 6.2 Zero-Copy GPU Mode

```cpp
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <mesher.h>

// Load volume
CVC::volume vol("input.vol");

// Enable CUDA for volume data
vol.enableCUDA(0);  // Use GPU 0

// Mesh in GPU mode (zero-copy!)
CVC::geometry mesh = LBIE::do_mesh_geometry(
    vol, 128.0, 255.0, 0.01, 1.0,
    CVC::geometry::VOLUME_TET,
    LBIE::Mesher::GEO_FLOW,
    "bspline_convolution",
    LBIE::Mesher::DUALLIB,
    5,
    false, false
);

// mesh automatically uses CUDA unified memory
assert(mesh.using_cuda() == true);

// Result: 0 copies! Volume and mesh share GPU memory
```

#### 6.3 GPU-Accelerated Quality Improvement

```cpp
#include <cvc/geometry.h>
#include <mesher.h>

// Load existing mesh
CVC::geometry mesh("input.mesh");

// Enable CUDA
mesh.enableCUDA(0);

// GPU quality metrics (parallel)
auto qualities = LBIE::compute_tet_qualities_cuda(mesh);

// GPU point location (parallel)
std::vector<CVC::geometry::point_t> query_points = {...};
auto containing_tets = LBIE::find_containing_tets_cuda(mesh, query_points);

// GPU quality improvement (zero-copy throughout)
CVC::geometry improved = LBIE::quality_improve_geometry(
    mesh,
    LBIE::Mesher::GEO_FLOW,
    10  // iterations
);

// All operations shared the same GPU memory - zero copies!
```

#### 6.4 Multi-GPU Support

```cpp
#include <cvc/geometry.h>
#include <cvc/cuda_utils.h>

// Query available GPUs
auto devices = CVC::cuda_device_manager::get_device_info();
for (const auto& dev : devices) {
    std::cout << "GPU " << dev.device_id << ": " << dev.name 
              << " (" << dev.total_memory / (1024*1024) << " MB)\n";
}

// Create meshes on different GPUs
CVC::volume vol1("vol1.vol"), vol2("vol2.vol");

vol1.enableCUDA(0);  // GPU 0
vol2.enableCUDA(1);  // GPU 1

CVC::geometry mesh1 = LBIE::do_mesh_geometry(vol1, ...);  // Uses GPU 0
CVC::geometry mesh2 = LBIE::do_mesh_geometry(vol2, ...);  // Uses GPU 1

// Meshes can be on different GPUs simultaneously
assert(mesh1.cuda_device_id() == 0);
assert(mesh2.cuda_device_id() == 1);
```

## Implementation Plan

### Phase 1: Foundation (Days 1-2)
- [x] Review voxels.cpp CUDA implementation
- [ ] Create `cuda_allocator.h` with `cuda_unified_allocator<T>`
- [ ] Add CUDA support to geometry class
  - [ ] Add `_using_cuda`, `_cuda_device_id` members
  - [ ] Add `_cuda_*_ptr` shared_ptr members for each array
  - [ ] Implement `enableCUDA()`, `disableCUDA()`, `migrate_to_cuda()`
  - [ ] Add `*_data()` raw pointer accessors
- [ ] Unit tests for geometry CUDA migration

### Phase 2: Zero-Copy Adapter (Days 3-4)
- [ ] Create `geoframe_unified.h` with pointer-based geoframe
- [ ] Implement `geoframe_adapter::sync_from_geometry_zerocopy()`
- [ ] Implement `geoframe_adapter::sync_to_geometry_zerocopy()`
- [ ] Handle type conversions (double ↔ float)
  - [ ] Option 1: Temporary float buffer in unified memory
  - [ ] Option 2: Template geoframe on FloatType
- [ ] Unit tests for zero-copy adapter

### Phase 3: CUDA Kernels (Days 5-7)
- [ ] Create `geometry_kernels.cu`
  - [ ] `extract_tet_faces_kernel()`
  - [ ] `extract_hex_faces_kernel()`
- [ ] Create `point_location_kernels.cu`
  - [ ] `find_containing_tets_kernel()`
  - [ ] BVH/octree acceleration structure (optional)
- [ ] Create `quality_kernels.cu`
  - [ ] `compute_tet_quality_kernel()`
  - [ ] `compute_hex_quality_kernel()`
  - [ ] Various quality metrics (aspect ratio, Jacobian, etc.)
- [ ] Unit tests for each kernel

### Phase 4: Integration (Days 8-9)
- [ ] Update `do_mesh_geometry()` to detect CUDA mode
- [ ] Update `quality_improve_geometry()` to use GPU kernels
- [ ] Add automatic CUDA enablement for large meshes
- [ ] Integration tests with full pipeline

### Phase 5: Optimization (Day 10)
- [ ] Profile GPU vs CPU performance
- [ ] Optimize kernel launch parameters
- [ ] Add GPU memory prefetching hints
- [ ] Benchmark suite for performance validation

### Phase 6: Documentation (Day 11)
- [ ] Update API documentation
- [ ] Create CUDA usage guide
- [ ] Performance benchmarks
- [ ] Migration guide from Week 4

## Performance Targets

### Memory Reduction
- **Week 3:** 3 allocations (geometry → geoframe → geometry)
- **Week 4:** 2 allocations (50% reduction)
- **Week 5:** 1 allocation (100% reduction from baseline, 50% from Week 4)

### GPU Acceleration
- **Point location:** 10-100x speedup (depends on query count)
- **Quality metrics:** 20-50x speedup (massively parallel)
- **Surface extraction:** 5-10x speedup

### Mesh Size Scalability
- **Small meshes (<10K tets):** Week 4 copy mode may be faster (low overhead)
- **Medium meshes (10K-1M tets):** Zero-copy provides 2-5x overall speedup
- **Large meshes (>1M tets):** Zero-copy essential (avoids OOM on 2x copy)

## Testing Strategy

### Unit Tests
```cpp
TEST(GeometryTest, CUDAMigration) {
    geometry geom;
    geom.points().push_back({1.0, 2.0, 3.0});
    
    EXPECT_FALSE(geom.using_cuda());
    
    geom.enableCUDA(0);
    EXPECT_TRUE(geom.using_cuda());
    EXPECT_EQ(geom.cuda_device_id(), 0);
    
    // Verify data integrity
    EXPECT_DOUBLE_EQ(geom.points_data()[0][0], 1.0);
    
    geom.disableCUDA();
    EXPECT_FALSE(geom.using_cuda());
    EXPECT_DOUBLE_EQ(geom.points()[0][0], 1.0);  // Data preserved
}

TEST(GeometryTest, ZeroCopyAdapter) {
    volume vol("sphere_64.vol");
    vol.enableCUDA(0);
    
    geometry mesh = do_mesh_geometry(vol, 128.0, 255.0, 0.01, 1.0,
                                    geometry::VOLUME_TET, ...);
    
    EXPECT_TRUE(mesh.using_cuda());
    EXPECT_GT(mesh.num_tets(), 0);
    
    // Verify no copies occurred
    // (Check memory allocation counts via instrumentation)
}

TEST(CUDAKernelsTest, TetQuality) {
    geometry geom;
    // Create simple tet mesh
    geom.points().push_back({0, 0, 0});
    geom.points().push_back({1, 0, 0});
    geom.points().push_back({0, 1, 0});
    geom.points().push_back({0, 0, 1});
    geom.tets().push_back({0, 1, 2, 3});
    geom.set_geometry_type(geometry::VOLUME_TET);
    
    geom.enableCUDA(0);
    
    auto qualities = compute_tet_qualities_cuda(geom);
    
    ASSERT_EQ(qualities.size(), 1);
    EXPECT_GT(qualities[0], 0.0);  // Valid quality metric
}
```

### Integration Tests
```cpp
TEST(Week5Integration, EndToEndZeroCopy) {
    // Full pipeline: Volume → Mesh → Quality → Point Location
    volume vol("bunny_128.vol");
    vol.enableCUDA(0);
    
    // Mesh generation (zero-copy)
    geometry mesh = do_mesh_geometry(vol, ...);
    EXPECT_TRUE(mesh.using_cuda());
    
    // Quality metrics (GPU)
    auto qualities = compute_tet_qualities_cuda(mesh);
    EXPECT_EQ(qualities.size(), mesh.num_tets());
    
    // Quality improvement (zero-copy)
    geometry improved = quality_improve_geometry(mesh, ...);
    EXPECT_TRUE(improved.using_cuda());
    
    // Point location (GPU)
    std::vector<geometry::point_t> queries = {...};
    auto tets = find_containing_tets_cuda(improved, queries);
    EXPECT_EQ(tets.size(), queries.size());
    
    // Verify no data corruption
    EXPECT_TRUE(improved.num_tets() > 0);
}
```

### Performance Benchmarks
```cpp
void BM_MeshGeneration_CPU(benchmark::State& state) {
    volume vol("sphere_512.vol");
    for (auto _ : state) {
        geometry mesh = do_mesh_geometry(vol, ...);
    }
}

void BM_MeshGeneration_GPU(benchmark::State& state) {
    volume vol("sphere_512.vol");
    vol.enableCUDA(0);
    for (auto _ : state) {
        geometry mesh = do_mesh_geometry(vol, ...);
    }
}

void BM_PointLocation_CPU(benchmark::State& state) {
    geometry mesh = load_mesh("bunny.mesh");
    std::vector<point_t> queries = generate_random_points(10000);
    for (auto _ : state) {
        find_containing_tets_cpu(mesh, queries);
    }
}

void BM_PointLocation_GPU(benchmark::State& state) {
    geometry mesh = load_mesh("bunny.mesh");
    mesh.enableCUDA(0);
    std::vector<point_t> queries = generate_random_points(10000);
    for (auto _ : state) {
        find_containing_tets_cuda(mesh, queries);
    }
}
```

## Risk Mitigation

### Type Conversion Issues
**Risk:** Double ↔ float conversion may lose precision  
**Mitigation:** 
- Use double-precision throughout where possible
- Template geoframe on FloatType
- Validate precision in unit tests
- Document precision guarantees

### CUDA Availability
**Risk:** Code must work on systems without CUDA  
**Mitigation:**
- All CUDA code in `#ifdef CVC_USING_CUDA` blocks
- Automatic fallback to Week 4 copy mode
- CPU implementations always available
- Unit tests run in both modes

### Memory Leaks
**Risk:** Unified memory not properly freed  
**Mitigation:**
- Use `std::shared_ptr` with `CudaManagedDeleter` (proven pattern from voxels)
- Reference counting ensures automatic cleanup
- Valgrind testing
- CUDA memory profiling tools

### Performance Regression
**Risk:** GPU overhead may hurt small meshes  
**Mitigation:**
- Automatic mode selection based on mesh size
- Benchmarks for various mesh sizes
- Fallback to CPU for small workloads
- User override via API flags

## Success Criteria

Week 5 is successful when:

1. ✅ Zero memory copies for geometry with CUDA enabled
2. ✅ All 106+ tests pass in both CPU and GPU modes
3. ✅ GPU point location ≥10x faster than CPU (10K+ queries)
4. ✅ GPU quality metrics ≥20x faster than CPU
5. ✅ No memory leaks (Valgrind clean)
6. ✅ Backward compatible (Week 4 CPU mode still works)
7. ✅ Multi-GPU support functional
8. ✅ Documentation complete

## Conclusion

Week 5 builds on Week 4's architectural foundation to achieve true zero-copy data sharing using CUDA unified memory. By leveraging the proven patterns from `voxels.cpp` (reference-counted shared_ptr with custom deleters), we can eliminate all conversion overhead while maintaining backward compatibility and code safety.

The key insights from the voxels implementation:
1. `std::shared_ptr<void>` with `CudaManagedDeleter` for automatic memory management
2. Shallow copy semantics via reference counting
3. Transparent CPU/GPU access via `get_data_ptr()`
4. Graceful fallback to CPU mode when CUDA unavailable

This approach extends naturally to geometry, enabling GPU-accelerated algorithms while preserving the clean Week 4 API.

**Estimated Timeline:** 11 days  
**Dependencies:** Week 4 complete, CUDA 11.0+, GPU with compute capability ≥6.0  
**Risk Level:** Medium (type conversion complexity, new CUDA kernels)  
**Reward:** High (zero-copy, GPU acceleration, future-proof architecture)

Next: **Implementation Phase 1** - CUDA allocator and geometry migration 🚀
