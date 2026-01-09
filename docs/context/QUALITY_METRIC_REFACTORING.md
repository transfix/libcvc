# Quality Metric Enum Refactoring

## Overview

Refactored quality metric selection from string-based parameters to type-safe enum values across all quality-related functions in the CVC library.

## Motivation

- **Type Safety**: Enum provides compile-time type checking, preventing typos and invalid metric names
- **IDE Support**: Better autocomplete and code navigation
- **Consistency**: Matches existing CVC API patterns (extraction_method, improvement_method, mesh_type)
- **Performance**: Switch statements may optimize better than string comparisons
- **Maintainability**: Easier to add new metrics and refactor code

## Changes

### 1. New Enum Definition (inc/cvc/types.h)

Added `quality_metric` enum with 6 values:

```cpp
enum quality_metric
{
  // Tetrahedral mesh metrics
  TET_VOLUME = 0,         // Volume of tetrahedron
  TET_ASPECT_RATIO = 1,   // Aspect ratio (lower is better)
  TET_MIN_ANGLE = 2,      // Minimum dihedral angle (higher is better)
  
  // Hexahedral mesh metrics
  HEX_VOLUME = 3,         // Volume of hexahedron
  HEX_JACOBIAN = 4,       // Jacobian determinant (positive is valid)
  HEX_SCALED_JACOBIAN = 5 // Scaled Jacobian quality [-1, 1]
};
```

### 2. Function Signature Updates (inc/cvc/algorithm.h)

Changed 5 function signatures from `const std::string& metric` to `quality_metric metric`:

```cpp
// Before:
quality_stats compute_tet_quality_stats(const geometry::tets_t& tets,
                                        const geometry::points_t& vertices,
                                        const std::string& metric = "aspect_ratio");

// After:
quality_stats compute_tet_quality_stats(const geometry::tets_t& tets,
                                        const geometry::points_t& vertices,
                                        quality_metric metric = TET_ASPECT_RATIO);
```

**Updated Functions:**
- `compute_tet_quality_stats()` - default: TET_ASPECT_RATIO
- `compute_hex_quality_stats()` - default: HEX_SCALED_JACOBIAN
- `filter_tets_by_quality()` - default: TET_ASPECT_RATIO
- `filter_hexs_by_quality()` - default: HEX_SCALED_JACOBIAN
- `extract_quality_elements()` - default: TET_ASPECT_RATIO

### 3. Implementation Updates (src/cvc/algorithm.cpp)

Replaced string comparisons with switch statements:

```cpp
// Before:
if(metric == "volume") {
  val = std::abs(tet_volume(...));
} else if(metric == "aspect_ratio") {
  val = tet_aspect_ratio(...);
} else if(metric == "min_angle") {
  val = tet_min_dihedral_angle(...);
}

// After:
switch(metric) {
  case TET_VOLUME:
    val = std::abs(tet_volume(...));
    break;
  case TET_ASPECT_RATIO:
    val = tet_aspect_ratio(...);
    break;
  case TET_MIN_ANGLE:
    val = tet_min_dihedral_angle(...);
    break;
  default:
    val = tet_aspect_ratio(...);
    break;
}
```

### 4. Test Updates (src/cvc/tests/geometry_test.cpp)

Updated test calls to use enum values:

```cpp
// Before:
auto stats = compute_tet_quality_stats(geom.const_tets(), geom.points(), "volume");
auto good_tets = filter_tets_by_quality(geom.const_tets(), geom.points(), 5.0, "aspect_ratio");
geometry quality_mesh = extract_quality_elements(geom, 5.0, "aspect_ratio");

// After:
auto stats = compute_tet_quality_stats(geom.const_tets(), geom.points(), TET_VOLUME);
auto good_tets = filter_tets_by_quality(geom.const_tets(), geom.points(), 5.0, TET_ASPECT_RATIO);
geometry quality_mesh = extract_quality_elements(geom, 5.0, TET_ASPECT_RATIO);
```

## Usage Examples

### Tetrahedral Mesh Quality Analysis

```cpp
using namespace CVC_NAMESPACE;

geometry tet_mesh = /* ... */;

// Compute volume statistics
auto vol_stats = compute_tet_quality_stats(tet_mesh.const_tets(), 
                                           tet_mesh.points(), 
                                           TET_VOLUME);

// Compute aspect ratio statistics
auto ar_stats = compute_tet_quality_stats(tet_mesh.const_tets(), 
                                          tet_mesh.points(), 
                                          TET_ASPECT_RATIO);

// Filter tets with aspect ratio < 5.0
auto good_tets = filter_tets_by_quality(tet_mesh.const_tets(), 
                                       tet_mesh.points(), 
                                       5.0, 
                                       TET_ASPECT_RATIO);

// Extract elements with min dihedral angle > 15 degrees
geometry quality_mesh = extract_quality_elements(tet_mesh, 15.0, TET_MIN_ANGLE);
```

### Hexahedral Mesh Quality Analysis

```cpp
geometry hex_mesh = /* ... */;

// Compute scaled Jacobian statistics
auto sj_stats = compute_hex_quality_stats(hex_mesh.const_hexs(), 
                                          hex_mesh.points(), 
                                          HEX_SCALED_JACOBIAN);

// Filter hexs with scaled Jacobian > 0.3
auto good_hexs = filter_hexs_by_quality(hex_mesh.const_hexs(), 
                                       hex_mesh.points(), 
                                       0.3, 
                                       HEX_SCALED_JACOBIAN);

// Extract elements by volume threshold
geometry quality_mesh = extract_quality_elements(hex_mesh, 0.01, HEX_VOLUME);
```

## Backward Compatibility

**Breaking Change**: This is an API-breaking change. Code using string-based metrics must be updated to use enum values.

### Migration Guide

| Old String Value | New Enum Value |
|-----------------|----------------|
| "volume" (tet) | TET_VOLUME |
| "aspect_ratio" | TET_ASPECT_RATIO |
| "min_angle" | TET_MIN_ANGLE |
| "volume" (hex) | HEX_VOLUME |
| "jacobian" | HEX_JACOBIAN |
| "scaled_jacobian" | HEX_SCALED_JACOBIAN |

## Testing

All 112 tests pass (104 active, 8 skipped stress tests):
- 69 tests from GeometryTest
- 34 tests from AlgorithmTest (includes quality metric tests)
- 9 tests from GeometryWeek1Test

**Quality-specific tests:**
- TetMeshQualityStatistics ✅
- FilterTetsByQuality ✅
- ExtractQualityElements ✅
- All existing quality improvement tests ✅

## Benefits

1. **Compile-Time Safety**: Invalid metrics caught at compile time rather than runtime
2. **Better IDE Support**: Autocomplete shows available metrics with documentation
3. **Cleaner Code**: Switch statements are more readable than string comparisons
4. **Consistency**: Follows established CVC patterns for enum-based API selection
5. **Performance**: Potential compiler optimizations with switch statements
6. **Maintainability**: Adding new metrics requires enum update, no string typo risks

## Date

December 28, 2025
