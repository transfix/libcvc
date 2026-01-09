# SDF v2 Critical Bug Fix - Out-of-Bounds Array Access

## Executive Summary
Fixed a critical bug in SDF v2 that caused crashes on geometries with vertices extending outside the grid bounding box. The bug was a pre-existing issue (present since before the bbox resize work) that manifested as segfaults when processing certain geometries like tetrahedrons, octahedrons, and diamonds.

## Root Cause Analysis

### The Bug
File: `src/cvc/SDF/SignDistanceFunction_v2/DistanceTransform.cpp`  
Function: `DistanceTransform::init()`  
Line: ~74

**Problem**: When building the triangle list for cells, the code called `index2cell(i, j, k)` which returns `-1` for out-of-bounds indices, but then used this value directly as an array index without checking:

```cpp
int nc = p_Data->index2cell(i, j, k);
assert(nc < p_Data->getNCells() && nc >= 0);  // Assert compiled out in release!
p_Cells[nc].triList.insert(nt);  // CRASH when nc == -1
```

**Why it happens**:
1. Triangle bounding box extends outside grid bounds
2. `minId`/`maxId` calculated from triangle vertices can be negative or >= dim-1
3. `index2cell()` correctly returns -1 for out-of-bounds
4. Code didn't check for -1 before array access
5. `assert()` statements are compiled out in release builds (`NDEBUG`)

### Manifestation
- **Cube**: Works fine (vertices within bounds)
- **Tetrahedron**: CRASHES (vertices extend outside)
- **Octahedron**: CRASHES (vertices extend outside)
- **Diamond**: CRASHES (vertices extend outside)

### Debug Output
```
DEBUG: Inserting tri 0 into cell 3580 (cells allocated: 6859)
DEBUG: Inserting tri 0 into cell 3581 (cells allocated: 6859)
DEBUG: Inserting tri 0 into cell 3582 (cells allocated: 6859)
DEBUG: Inserting tri 0 into cell -1 (cells allocated: 6859)  ← CRASH HERE
timeout: the monitored command dumped core
Segmentation fault
```

## The Fix

### Code Change
```cpp
// BEFORE (buggy):
int nc = p_Data->index2cell(i, j, k);
assert(nc < p_Data->getNCells() && nc >= 0);
Vector3f norm;
p_Surf->getTriNormal(nt, norm);
if(intersectCell(v0, norm, i, j, k)) {
    // ... triangle-cube intersection check
    p_Cells[nc].triList.insert(nt);  // CRASH when nc == -1
}

// AFTER (fixed):
int nc = p_Data->index2cell(i, j, k);
if (nc < 0) continue;  // Skip out-of-bounds cells ✅
assert(nc < p_Data->getNCells());
Vector3f norm;
p_Surf->getTriNormal(nt, norm);
if(intersectCell(v0, norm, i, j, k)) {
    // ... triangle-cube intersection check
    p_Cells[nc].triList.insert(nt);  // Safe: nc is valid
}
```

### Why This Fix is Correct
1. **Semantically sound**: Out-of-bounds cells don't exist, so skipping them is correct
2. **Matches existing pattern**: Lines 391 and 767 already do bounds checking before calling `index2cell()`
3. **Minimal change**: Single line addition, no algorithm changes
4. **Performance**: Negligible impact (check is cheap, out-of-bounds cases are rare)

## Test Results

### Before Fix
```bash
$ ./bin/geometry_test --gtest_filter="*SDF*"
[==========] Running 6 tests from 1 test suite.
[  PASSED  ] 6 tests.
  YOU HAVE 1 DISABLED TEST  # Parallel test disabled
```

### After Fix
```bash
$ ./bin/geometry_test --gtest_filter="*SDF*"
[==========] Running 7 tests from 1 test suite.
[       OK ] AlgorithmTest.SDFBasic (243 ms)
[       OK ] AlgorithmTest.SDFThenIsoRoundtrip (40 ms)
[       OK ] AlgorithmTest.BunnySDF_IsoRoundtrip (5530 ms)
[       OK ] AlgorithmTest.SDFV2Basic (8 ms)
[       OK ] AlgorithmTest.SDFV1vsV2Comparison (395 ms)
[       OK ] AlgorithmTest.SDFV2ParallelExecution (6 ms)  ← NOW PASSING! ✅
[       OK ] AlgorithmTest.SDFStressTest (51240 ms)
[==========] 7 tests from 1 test suite ran. (57466 ms total)
[  PASSED  ] 7 tests.  ← ALL TESTS PASSING! ✅
```

## Investigation Journey

### Initial Hypothesis (WRONG)
"This is a race condition in `dynamic_array::insert()`"
- Evidence: Parallel test crashed
- Reasoning: `dynamic_array` is not thread-safe
- Attempted fix: Added mutex to serialize SDF calls
- Result: **STILL CRASHED** (even with exclusive lock!)

### Key Insight
Thread 1 crashed WHILE HOLDING EXCLUSIVE LOCK:
```
Thread 0 completed!
Thread 1 acquiring lock...
Thread 1 has lock, calling sdf()...
scaling factors: 1.154734...
[TIMEOUT CORE DUMP]  ← No other thread running!
```

**Conclusion**: NOT a concurrency bug - it's geometry-specific!

### Breakthrough
Tested geometries individually:
- Cube alone: ✅ SUCCESS
- Tetrahedron alone: ❌ CRASH

Added debug output:
```cpp
std::cout << "DEBUG: Inserting tri " << nt << " into cell " << nc 
          << " (cells allocated: " << p_Data->getNCells() << ")" << std::endl;
```

Output showed `nc = -1` → **OUT-OF-BOUNDS ARRAY ACCESS**

## Files Modified

### Core Fix
- `src/cvc/SDF/SignDistanceFunction_v2/DistanceTransform.cpp`
  - Added `#include <iostream>` for debug output
  - Added bounds check: `if (nc < 0) continue;`

### Test Updates  
- `src/cvc/tests/geometry_test.cpp`
  - Re-enabled parallel test
  - Removed unnecessary mutex serialization
  - Updated comments to reflect the fix

## Impact

### What This Fixes
✅ Tetrahedron geometry now works  
✅ Octahedron geometry now works  
✅ Diamond geometry now works  
✅ Parallel test now passes  
✅ Any geometry with vertices outside grid bounds  

### Backward Compatibility
✅ No API changes  
✅ No algorithm changes  
✅ All existing tests still pass  
✅ Performance unchanged (bounds check is trivial)  

## Lessons Learned

1. **Don't rely on `assert()` for runtime safety** - they're compiled out in release builds
2. **Negative array indices are valid C++ but crash at runtime** - always bounds check!
3. **Concurrency bugs and geometry bugs can look similar** - test in isolation to distinguish
4. **Simple geometries make better test cases** - tetrahedron revealed the bug, bunny didn't

## Verification

To verify the fix yourself:
```bash
# Test tetrahedron alone (previously crashed)
cd build
g++ -o test_tetra test_tetra.cpp -I./inc -I../inc -L./lib -lcvc -Wl,-rpath,./lib -std=c++11
./test_tetra  # Should succeed without crashing

# Run parallel test (previously disabled)
./bin/geometry_test --gtest_filter="*Parallel*"
# Should show: [  PASSED  ] 1 test.

# Run all SDF tests
./bin/geometry_test --gtest_filter="*SDF*"
# Should show: [  PASSED  ] 7 tests.
```

## Conclusion

A single missing bounds check caused crashes on valid geometries. The fix is minimal (one line), semantically correct, and enables SDF v2 to handle all geometry types including those with vertices extending outside the grid bounding box.

**Status**: ✅ FIXED - All 7 SDF tests passing including parallel execution!
