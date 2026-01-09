# SDF v1 Multiple Call Bug Investigation

**Date**: December 24, 2025  
**Status**: INVESTIGATION IN PROGRESS 🔍  
**Bug**: Stack smashing when calling SDF v1 multiple times with complex geometries

## Summary

CPU vs GPU resize performance tests added successfully (✅ **27x speedup** achieved).  
However, discovered SDF v1 crashes when called multiple times with complex geometries (Stanford Bunny).

## Test Added

**File**: `src/cvc/tests/geometry_test.cpp`  
**Test**: `GeometryTest.SDFV1MultipleSequentialCalls`

```cpp
// Tests calling SDF v1 multiple times with Bunny geometry
// Calls with dimensions: 30³, 32³, 40³, 48³, 50³
```

## Bug Symptoms

- **Simple geometries (cube)**: ✅ Multiple calls work fine
- **Complex geometries (Bunny, 69K triangles)**: ❌ **Stack smashing** on 1st or 2nd call  
- **Error**: `*** stack smashing detected ***: terminated`

## Investigation Findings

### 1. Thread-Safe Refactoring Status
- ✅ SDFContext encapsulates most state
- ✅ `computeSDF_MT()` creates fresh context per call
- ❌ **Global variables still exist** in `head.cpp` (legacy code not fully removed)

### 2. Memory Management Issues Explored
- **boost::multi_array with unique_ptr**: Investigated copy/move semantics
- **cell::tindex**: unique_ptr to listnode (auto-cleanup)
- **Attempted fixes**:
  - Manual cleanup before resize → broke single calls
  - std::swap on multi_array → segfault in copy assignment
  - Modified copy assignment operator → broke initialization

### 3. GDB Backtrace Analysis
```
Program received signal SIGSEGV
at cell::operator=(const cell& other)
during boost::multi_array::operator= → std::copy
when SDFContext::initSDF() calls:
  sdf = boost::multi_array<cell, 3>(boost::extents[size][size][size])
```

**Root cause hypothesis**: boost::multi_array doesn't properly handle objects with unique_ptr members during copy/assignment operations.

### 4. Suspected Issues

1. **Global Variables** (head.cpp lines 55-77):
   ```cpp
   namespace SDFLibrary {
       double MAX_DIST;
       int size;
       triangle* surface;  // OLD GLOBALS - should not exist!
       myVert* vertices;
       ...
   }
   ```
   These may interfere with SDFContext-based code.

2. **boost::multi_array limitations**:
   - Doesn't play well with move-only types (unique_ptr)
   - Copy operations during resize may cause use-after-free
   - Assignment operator invokes deep copy with potential issues

3. **Stack overflow possibility**:
   - Large geometries (69K triangles) might trigger deeper recursion
   - Octree depth increases with geometry complexity
   - Some functions may use stack-allocated temporary buffers

## Current Code State

### What Works ✅
- Single SDF v1 call with any geometry
- Multiple SDF v1 calls with simple geometries (< 100 triangles)
- All SDF v2 calls (both single and multiple, any geometry)
- CPU vs GPU resize performance tests (added successfully)

### What Fails ❌
- Multiple SDF v1 calls with complex geometries (> 1000 triangles)
- Occurs during `initSDF()` → `sdf.resize()` or assignment

## Attempted Fixes (All Reverted)

###  1. Manual Cleanup Before Resize
```cpp
if (sdf.num_elements() > 0) {
    auto shape = sdf.shape();
    for (size_t i = 0; i < shape[0]; i++)
        for (size_t j = 0; j < shape[1]; j++)
            for (size_t k = 0; k < shape[2]; k++)
                sdf[i][j][k].tindex.reset();
}
sdf = boost::multi_array<cell, 3>(boost::extents[size][size][size]);
```
**Result**: Broke single calls (segfault)

### 2. std::swap Approach
```cpp
boost::multi_array<cell, 3> new_sdf(boost::extents[size][size][size]);
std::swap(sdf, new_sdf);
```
**Result**: Segfault in copy assignment operator during swap

### 3. Modified cell::operator=
```cpp
cell& operator=(const cell& other) {
    ...
    tindex.reset();  // ADDED: Reset before assignment
    if (other.tindex) {
        tindex = std::make_unique<listnode>(*other.tindex);
    }
    ...
}
```
**Result**: Broke initialization (crashes on first call)

## Recommended Next Steps

### Option A: Remove boost::multi_array  
Replace `boost::multi_array<cell, 3> sdf` with:
```cpp
std::vector<std::vector<std::vector<cell>>> sdf;
```
or
```cpp
std::unique_ptr<cell[]> sdf;  // with manual 3D indexing
```

**Pros**: Full control over construction/destruction  
**Cons**: Requires refactoring all sdf[i][j][k] accesses

### Option B: Make tindex non-copyable, use raw pointer
```cpp
struct cell {
    ...
    listnode* tindex;  // Manual management
    ~cell() { delete tindex; }
    cell& operator=(const cell& other) {
        delete tindex;
        tindex = other.tindex ? new listnode(*other.tindex) : nullptr;
    }
};
```

**Pros**: boost::multi_array works fine with raw pointers  
**Cons**: Manual memory management (error-prone)

### Option C: Remove global variables completely
1. Search for all references to `SDFLibrary::surface`, `SDFLibrary::vertices`, etc.
2. Ensure all code paths use SDFContext members only
3. Delete global variable declarations in head.cpp

**Pros**: Fixes potential global state corruption  
**Cons**: May require extensive code audit

### Option D: Use SDF v2 for all production code
- SDF v2 is 18-27x faster
- SDF v2 has no multiple-call bugs
- Mark SDF v1 as deprecated/legacy

**Pros**: Pragmatic workaround  
**Cons**: Doesn't fix the underlying bug

## GPU Resize Performance (Successfully Completed ✅)

### Test Results
| Resolution | CPU Time | GPU Time | **Speedup** |
|------------|----------|----------|-------------|
| 16³ → 32³  | 8.9 ms   | 6.1 ms   | 1.5x        |
| 32³ → 64³  | 68.9 ms  | 3.5 ms   | **19.5x**   |
| 64³ → 128³ | 560.2 ms | 20.4 ms  | **27.5x**   |
| 32³ → 48³  | 29.1 ms  | 2.1 ms   | 14.0x       |

### Documentation Created
- **GPU_RESIZE_PERFORMANCE.md**: Comprehensive performance analysis
- **VoxelsCUDATest.ResizePerformanceComparison**: Automated test in CI

## Files Modified (This Session)

### Tests Added
- `src/cvc/tests/geometry_test.cpp`:
  - `GeometryTest.SDFV1MultipleSequentialCalls` (exposes bug)
  - `GeometryTest.SDFResizePerformanceComparison` (resize perf)
  - `GeometryTest.SDFFullPipelineWithResizeBreakdown` (pipeline analysis)

- `src/cvc/tests/voxels_test.cpp`:
  - `VoxelsCUDATest.ResizePerformanceComparison` (✅ passing)

### SDF v1 Code Investigated
- `src/cvc/SDF/SignDistanceFunction/SDFContext.cpp` (initSDF function)
- `src/cvc/SDF/SignDistanceFunction/head.h` (cell structure)
- `src/cvc/SDF/SignDistanceFunction/head.cpp` (global variables - legacy)

### Documentation
- `GPU_RESIZE_PERFORMANCE.md` (✅ created)
- `SDF_V1_MULTIPLE_CALL_BUG.md` (this file)

## Conclusion

**GPU resize performance**: ✅ **Mission accomplished** - 27x speedup documented and tested  
**SDF v1 multiple calls**: ⚠️ **Bug identified but not fixed** - requires architectural decision

**Recommendation**: 
1. Use SDF v2 for all new code (18-27x faster, no bugs)
2. Mark SDF v1 as legacy/deprecated
3. If SDF v1 fix is required, implement Option A (remove boost::multi_array) or Option B (raw pointers)

The GPU resize work is production-ready and provides substantial performance improvements. The SDF v1 bug is a known issue that can be worked around by using SDF v2.

---

**For Future Debugging**:
Run test with GDB:
```bash
cd build
gdb --args ./bin/geometry_test --gtest_filter="GeometryTest.SDFV1MultipleSequentialCalls"
(gdb) run
(gdb) bt  # when it crashes
```

Check for stack overflow:
```bash
ulimit -s unlimited  # Remove stack limit
./bin/geometry_test --gtest_filter="GeometryTest.SDFV1MultipleSequentialCalls"
```
