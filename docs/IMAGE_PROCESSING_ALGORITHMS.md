# Image Processing Algorithms - API Reference

This document provides detailed technical documentation for the image processing algorithms implemented in the libcvc library. These algorithms operate on volumetric data through the `voxels` class interface.

## Table of Contents
- [Anisotropic Diffusion](#anisotropic-diffusion)
- [Bilateral Filter](#bilateral-filter)
- [Contrast Enhancement](#contrast-enhancement)
- [GDTV Filter](#gdtv-filter-gradient-dependent-total-variation)

---

## Anisotropic Diffusion

### Overview
Anisotropic diffusion is an edge-preserving smoothing technique based on the Perona-Malik diffusion model. It reduces noise while preserving and even enhancing edges by using gradient-dependent diffusion coefficients.

### Method Signature
```cpp
voxels& voxels::anisotropicDiffusion(unsigned int iterations = 1);
```

### Parameters
- **iterations** (unsigned int, default=1): Number of diffusion iterations to perform. More iterations produce stronger smoothing effects.

### Algorithm Details

**Diffusion Model:**
- Uses the Perona-Malik model with exponential diffusion coefficient
- Diffusion coefficient: `c(∇I) = 1 / (1 + (∇I/K)²)`
- Fixed parameters:
  - **K** = 3.0 (edge threshold parameter)
  - **λ** = 0.16 (diffusion rate parameter)

**Update Equation:**
For each voxel at position (i,j,k):
```
I_new = I + λ * Σ(c_n * ∇_n)
```
where the sum is over 6-connected neighbors (north, south, east, west, up, down).

**Neighbor Differences:**
- δ_north = I(i, j-1, k) - I(i, j, k)
- δ_south = I(i, j+1, k) - I(i, j, k)
- δ_east = I(i+1, j, k) - I(i, j, k)
- δ_west = I(i-1, j, k) - I(i, j, k)
- δ_up = I(i, j, k+1) - I(i, j, k)
- δ_down = I(i, j, k-1) - I(i, j, k)

### Usage Example
```cpp
#include <cvc/volmagick.h>

voxels volume = /* load your volume */;

// Light smoothing (1 iteration)
volume.anisotropicDiffusion(1);

// Moderate smoothing (5 iterations)
volume.anisotropicDiffusion(5);

// Strong smoothing (20 iterations)
volume.anisotropicDiffusion(20);
```

### Behavior & Properties

**Edge Preservation:**
- Small gradients (< K): High diffusion coefficient → strong smoothing
- Large gradients (> K): Low diffusion coefficient → weak smoothing (edge preservation)
- Edges with contrast > 3.0 are strongly preserved

**Iteration Effects:**
- Each iteration applies one diffusion step
- Multiple iterations compound the smoothing effect
- Typical range: 1-50 iterations depending on noise level

**Computational Complexity:**
- Time: O(iterations × width × height × depth)
- Space: O(width × height × depth) for temporary buffer

### Best Practices

✓ **Use when:**
- You need to remove noise while preserving edges
- Working with medical imaging data (CT, MRI)
- Preprocessing for segmentation or feature extraction

✗ **Avoid when:**
- You need uniform smoothing across the entire volume
- Edges should be smoothed (use Gaussian blur instead)
- Real-time performance is critical (relatively slow)

### Performance Notes
- In-place modification with one temporary buffer
- Boundary voxels use zero-gradient assumption
- Thread-safe with progress reporting via `app.threadProgress()`

---

## Bilateral Filter

### Overview
The bilateral filter is a non-linear, edge-preserving smoothing filter that combines spatial proximity and radiometric (intensity) similarity. It smooths images while keeping edges sharp by weighting neighbors based on both distance and intensity difference.

### Method Signature
```cpp
voxels& voxels::bilateralFilter(
    double radiometricSigma = 50.0,
    double spatialSigma = 1.5,
    unsigned int filterRadius = 1
);
```

### Parameters

- **radiometricSigma** (double, default=50.0): Controls sensitivity to intensity differences
  - Smaller values: Only similar intensities are averaged (stronger edge preservation)
  - Larger values: More tolerance for intensity differences (more smoothing across edges)
  - Typical range: 10.0 - 100.0

- **spatialSigma** (double, default=1.5): Controls spatial extent of the Gaussian kernel
  - Smaller values: Only nearby neighbors contribute (local smoothing)
  - Larger values: Distant neighbors contribute (wider smoothing)
  - Typical range: 0.5 - 3.0

- **filterRadius** (unsigned int, default=1): Neighborhood radius in voxels
  - Defines the cubic neighborhood size: (2×radius+1)³
  - radius=1 → 3×3×3 = 27 voxels
  - radius=2 → 5×5×5 = 125 voxels
  - Typical range: 1-3

### Algorithm Details

**Two-Component Weighting:**

1. **Spatial Weight** (Gaussian distance):
```
w_spatial(x,y,z) = exp(-(x² + y² + z²) / (2 × spatialSigma²))
```

2. **Radiometric Weight** (intensity similarity):
```
w_radiometric(ΔI) = exp(-(ΔI²) / (2 × radiometricSigma²))
```

**Combined Weight:**
```
w_total = w_spatial × w_radiometric
```

**Filtered Output:**
```
I_filtered(p) = Σ(I(q) × w_total(p,q)) / Σ(w_total(p,q))
```

**Optimization:**
- Pre-computed spatial mask for the entire filter kernel
- Pre-computed radiometric lookup table (256 entries) for intensity differences
- Values normalized to [0, 255] range for table lookup

### Usage Example
```cpp
#include <cvc/volmagick.h>

voxels volume = /* load your volume */;

// Default parameters (balanced smoothing)
volume.bilateralFilter();

// Strong edge preservation (small radiometric sigma)
volume.bilateralFilter(20.0, 1.5, 1);

// More aggressive smoothing (large radiometric sigma)
volume.bilateralFilter(80.0, 2.0, 2);

// Wide neighborhood (large spatial sigma and radius)
volume.bilateralFilter(50.0, 3.0, 3);
```

### Behavior & Properties

**Edge Preservation:**
- Edges with contrast > 35 (relative to value range) are strongly preserved
- Edge sharpness is maintained while surrounding regions are smoothed
- No gradient reversal or edge shifting

**Noise Reduction:**
- Effectively reduces Gaussian noise in homogeneous regions
- Quantitative noise variance reduction: typically 40-70% in uniform areas
- Minimal noise reduction near edges (by design)

**Parameter Interactions:**
- **Small radiometricSigma + Small spatialSigma**: Minimal smoothing, strong edge preservation
- **Large radiometricSigma + Large spatialSigma**: Strong smoothing, moderate edge preservation
- **Small radiometricSigma + Large spatialSigma**: Interesting effect - smooths textures but preserves major edges
- **Large radiometricSigma + Small spatialSigma**: Localized averaging

### Implementation Details

**Min/Max Caching:**
- Original min/max values are cached before filtering
- Ensures consistent radiometric normalization throughout processing
- Critical for correct behavior with in-place modification

**Boundary Handling:**
- Only processes voxels where the full kernel fits within volume bounds
- Boundary voxels maintain original values
- Effective processing region: [radius, dim-radius) in each dimension

**Computational Complexity:**
- Time: O(width × height × depth × (2×radius+1)³)
- Space: O((2×radius+1)³) for spatial mask
- Memory: O(width × height × depth) + 256 bytes for lookup table

### Best Practices

✓ **Use when:**
- Noise reduction with sharp edge preservation is critical
- Processing photographic or medical imaging data
- Preparing data for edge-based segmentation
- Denoising without blurring important features

✗ **Avoid when:**
- Real-time performance is required (computationally expensive)
- You need uniform smoothing throughout (use Gaussian instead)
- Working with very low-contrast data (edges may not be detected)

### Performance Tips
- Start with default parameters and adjust based on visual inspection
- Increase filterRadius cautiously (computational cost increases cubically)
- Use smaller radiometricSigma for better edge preservation
- For large volumes, consider processing in chunks or using GPU acceleration

---

## Contrast Enhancement

### Overview
Adaptive contrast enhancement based on resistor-based propagation for local min/max estimation. This algorithm performs histogram equalization with spatially-varying local contrast stretching, preserving the original value range while redistributing intensities for better visual contrast.

### Method Signature
```cpp
voxels& voxels::contrastEnhancement(double resistor = 0.5);
```

### Parameters

- **resistor** (double, default=0.5, range=[0.0, 1.0]): Controls spatial influence strength
  - **0.0**: No spatial propagation (purely local contrast)
  - **0.5**: Moderate spatial coupling (balanced)
  - **1.0**: Strong spatial propagation (highly adaptive)
  - Values automatically clamped to [0.0, 1.0]

### Algorithm Details

**Three-Phase Process:**

**Phase 1: Internal Normalization**
```
Map volume to [0, 255] range for processing
```

**Phase 2: Local Min/Max Estimation via Resistor Network**

Estimates local minimum and maximum through bi-directional propagation:

*Bottom-Up Propagation:*
- Propagates from slice 0 to slice Z-1
- Within each slice: multiple passes (left→right, right→left, top→bottom, bottom→top)
- Update rule: `value_new = value + resistor × (neighbor_value - value)`

*Top-Down Propagation:*
- Propagates from slice Z-1 to slice 0
- Similar multi-pass strategy within each slice

*Min/Max Update:*
- `min_new = min + resistor × (neighbor_min - min)` if neighbor_min < min
- `max_new = max + resistor × (neighbor_max - max)` if neighbor_max > max

**Phase 3: Adaptive Contrast Stretching**

For each voxel:
1. Compute local min/max from bi-directional estimates
2. Calculate window: `w = sqrt((lmax - lmin) × (510 - (lmax - lmin)))`
3. Normalize: `img_norm = w × (img - lmin) / (lmax - lmin)`
4. Apply quadratic transformation for histogram spreading
5. Restore original value range: map back to [origmin, origmax]

**Mathematical Model:**

The quadratic transformation:
```
α = (avg - img) / (181.019 × window)

If α ≠ 0:
    a = 0.707 × α
    b = 1.414 × α × (img - window) - 1
    c = 0.707 × α × img × (img - 2×window) + img
    result = lmin + (-b - sqrt(b² - 4ac)) / (2a)
Else:
    result = img + lmin
```

### Usage Example
```cpp
#include <cvc/volmagick.h>

voxels volume = /* load your volume */;

// Store original range for reference
double orig_min = volume.min();
double orig_max = volume.max();

// Light enhancement (minimal spatial influence)
volume.contrastEnhancement(0.2);

// Moderate enhancement (balanced)
volume.contrastEnhancement(0.5);

// Strong enhancement (high spatial coupling)
volume.contrastEnhancement(0.9);

// Verify range is preserved
assert(std::abs(volume.min() - orig_min) < 1.0);
assert(std::abs(volume.max() - orig_max) < 1.0);
```

### Behavior & Properties

**Range Preservation:**
- **Critical property**: Original [min, max] range is always restored
- Internal processing uses [0, 255] normalization
- Final output mapped back to original range
- Tolerance: typically within ±0.1 of original min/max

**Contrast Redistribution:**
- Values are redistributed within the original range
- Low-contrast regions get expanded
- High-contrast regions may be compressed slightly
- Overall histogram is spread more evenly

**Local Adaptivity:**
- Each voxel's enhancement depends on its local neighborhood
- Resistor parameter controls neighborhood influence radius
- Higher resistor → larger spatial influence → more global behavior
- Lower resistor → smaller influence → more local adaptivity

**Edge Behavior:**
- Edges are generally preserved (contrast ≥ 80% of original)
- No edge reversal or significant edge shifting
- May enhance edges in some cases

**Ordering:**
- General monotonic trends are preserved
- Strict point-by-point monotonicity may be violated (nonlinear transformation)
- Tolerance: typically ≤ 2 ordering violations in smooth gradients

### Implementation Details

**Multi-Pass Propagation:**
Each slice undergoes 8 directional passes:
1. Bottom-up: left→right, right→left
2. Row-by-row: left→right, right→left (for each row)
3. Top-down: left→right, right→left
4. Row-by-row reverse: left→right, right→left (for each row)

**Progress Reporting:**
- Total steps: 3 × depth (bottom-up + top-down + stretching)
- Progress reported via `app.threadProgress()`

**Computational Complexity:**
- Time: O(depth × width × height × passes)
- Space: O(4 × width × height × depth) for temporary volumes
- Memory-intensive due to multiple full-volume copies

### Best Practices

✓ **Use when:**
- Need to improve visibility in low-contrast images
- Preparing data for visualization
- Enhancing features for visual inspection
- Original value range must be preserved (e.g., for quantitative analysis)

✗ **Avoid when:**
- Absolute intensity values are critical (use histogram equalization instead)
- Real-time processing is needed (computationally expensive)
- Data is already high-contrast
- You need guaranteed monotonicity preservation

### Parameter Selection Guide

**Resistor Value Guidelines:**
- **0.1 - 0.3**: Very local enhancement, good for texture enhancement
- **0.3 - 0.5**: Balanced local/global, general-purpose
- **0.5 - 0.7**: More global behavior, good for large features
- **0.7 - 0.9**: Very global, uniform enhancement across volume
- **< 0.5**: Recommended for most use cases

**Testing Strategy:**
1. Start with default (0.5)
2. If enhancement is too aggressive: reduce resistor
3. If enhancement is too subtle: increase resistor
4. Visually inspect edges and important features
5. Verify value range is preserved

### Known Limitations
- May introduce small non-monotonic variations in smooth gradients
- Computationally expensive for large volumes
- Enhancement effect depends heavily on local intensity distribution
- Not suitable when exact intensity preservation is critical

---

## GDTV Filter (Gradient-Dependent Total Variation)

### Overview
GDTV is an advanced edge-preserving smoothing filter that uses gradient-dependent weighting for adaptive noise reduction. It combines total variation regularization with a nonlinear edge detection function, providing strong smoothing in homogeneous regions while preserving edges.

### Method Signature
```cpp
voxels& voxels::gdtvFilter(
    double parameterq = 1.5,
    double lambda = 0.5,
    unsigned int iteration = 1,
    unsigned int neigbour = 0
);
```

### Parameters

- **parameterq** (double, default=1.5): Nonlinearity parameter for phi function
  - Range: typically [1.0, 2.0]
  - **q < 1.5**: Less nonlinear, smoother edges
  - **q = 1.5**: Balanced (recommended)
  - **q > 1.5**: More nonlinear, sharper edge preservation
  - Controls the edge detection sensitivity

- **lambda** (double, default=0.5): Data fidelity weight
  - Range: [0.0, 1.0]
  - **λ → 0**: More smoothing, less data preservation
  - **λ = 0.5**: Balanced (recommended)
  - **λ → 1**: Less smoothing, more original data retained
  - Higher values preserve original data better

- **iteration** (unsigned int, default=1): Number of filtering iterations
  - Range: typically [1, 20]
  - Each iteration applies one filtering pass
  - More iterations → more smoothing
  - Note: Gradient is computed once, then reused for all iterations

- **neigbour** (unsigned int, default=0): Neighborhood connectivity mode
  - **0**: 6-neighbor connectivity (face neighbors only)
  - **non-zero**: 26-neighbor connectivity (face, edge, and corner neighbors)
  - 26-neighbor mode is more computationally expensive but smoother

### Algorithm Details

**Core Function:**
```
φ(x, q) = (2 - q) × x^(1-q)
```

**Gradient Computation:**

*6-Neighbor Mode (neigbour=0):*
```
∇I(i,j,k) = sqrt(Σ(δ_n²))
```
where δ_n are differences to 6 face neighbors

*26-Neighbor Mode (neigbour≠0):*
```
∇I(i,j,k) = sqrt(Σ(w_n × δ_n²))
```
Weights:
- Face neighbors (6): w = 1.0
- Edge neighbors (12): w = 0.5
- Corner neighbors (8): w = 0.333

**Filtering Process:**

For each voxel and each iteration:

1. **Compute Weights:**
```
If grad(p) ≠ 0:
    weight_n = φ(grad(p), q)/grad(p) + φ(grad(n), q)/grad(n)
Else:
    weight_n = φ(ε, q)/ε  (ε = 0.0001)
```

2. **Normalize Weights:**
```
weights_normalized = weights / (Σweights + λ)
```

3. **Weighted Average:**
```
I_new(p) = Σ(I(n) × weight_normalized_n) + I_original(p) × (λ / total)
```

**Iteration Strategy:**
- Gradient computed once before iteration loop
- Same gradient used for all iterations
- Each iteration smooths based on the fixed gradient field
- This approach is fast but gradient-independent after first computation

### Usage Example
```cpp
#include <cvc/volmagick.h>

voxels volume = /* load your volume */;

// Default parameters (balanced smoothing)
volume.gdtvFilter();

// Strong edge preservation (high q, high lambda)
volume.gdtvFilter(1.8, 0.7, 3, 0);

// Aggressive smoothing (low q, low lambda)
volume.gdtvFilter(1.2, 0.2, 10, 0);

// 26-neighbor mode for smoother results
volume.gdtvFilter(1.5, 0.5, 5, 26);

// Light denoising (minimal iterations)
volume.gdtvFilter(1.5, 0.4, 1, 0);

// Strong denoising (many iterations)
volume.gdtvFilter(1.5, 0.3, 15, 0);
```

### Behavior & Properties

**Edge Preservation:**
- High gradient regions receive low smoothing weights
- Edges with contrast > 30 are strongly preserved
- No edge reversal or significant shifting
- Better preservation than Gaussian or median filters

**Noise Reduction:**
- Effective in homogeneous regions (low gradient)
- Variance reduction typically 30-60% in uniform areas
- Preserves structure while removing noise
- Handles Gaussian and some impulse noise

**Gradient Sensitivity:**
- Algorithm behavior strongly depends on gradient magnitude
- Low gradients → high smoothing weights → strong smoothing
- High gradients → low smoothing weights → preservation
- Threshold controlled by q parameter

**3D Behavior:**
- Naturally extends to 3D volumes
- Gradient computed in all three dimensions
- Isotropic smoothing (equal in all directions)
- Corner and edge voxels handled correctly

### Parameter Interactions

**q and λ:**
- **Low q + Low λ**: Maximum smoothing, minimal edge preservation
- **High q + Low λ**: Strong smoothing with sharp edge preservation
- **Low q + High λ**: Minimal smoothing, soft edges
- **High q + High λ**: Minimal smoothing, sharp edge preservation

**iterations and λ:**
- More iterations amplify the effect of λ
- High λ + many iterations: converges to original image
- Low λ + many iterations: strong smoothing even at edges

**neigbour connectivity:**
- 6-neighbor: Faster, more directional artifacts possible
- 26-neighbor: Slower, smoother, more isotropic

### Implementation Details

**Computational Complexity:**

*6-Neighbor Mode:*
- Gradient: O(width × height × depth)
- Filtering: O(iterations × width × height × depth × 7)
- Memory: O(3 × width × height × depth)

*26-Neighbor Mode:*
- Gradient: O(width × height × depth × 26)
- Filtering: O(iterations × width × height × depth × 27)
- Memory: O(3 × width × height × depth)

**Boundary Handling:**
- Gradient uses zero-difference for out-of-bounds neighbors
- No padding or mirroring
- Boundary voxels may have different behavior

**Numerical Stability:**
- Epsilon (ε = 0.0001) prevents division by zero
- Phi function always positive for x > 0
- Weights normalized to prevent overflow

### Best Practices

✓ **Use when:**
- Need strong edge preservation with effective noise reduction
- Processing medical imaging, scientific data
- Preparing for segmentation or feature extraction
- Gradient information is reliable

✗ **Avoid when:**
- Real-time performance is critical (relatively slow)
- Data is very noisy (may preserve noise structures)
- Uniform smoothing is desired
- Edges should be smoothed (use Gaussian instead)

### Parameter Selection Guide

**General-Purpose Settings:**
```cpp
volume.gdtvFilter(1.5, 0.3, 5, 0);  // Good starting point
```

**Aggressive Denoising:**
```cpp
volume.gdtvFilter(1.2, 0.2, 15, 26);  // Maximum smoothing
```

**Maximum Edge Preservation:**
```cpp
volume.gdtvFilter(1.8, 0.7, 2, 0);  // Minimal smoothing at edges
```

**Testing Strategy:**
1. Start with default parameters
2. Adjust q first (1.2 - 1.8) based on edge quality
3. Adjust λ (0.2 - 0.7) based on smoothing strength desired
4. Increase iterations if more smoothing needed
5. Try 26-neighbor mode if directional artifacts appear

### Performance Tips
- Use 6-neighbor mode unless smoothness is critical
- Keep iterations ≤ 10 for reasonable performance
- Gradient computation is one-time cost (not per iteration)
- Consider downsampling large volumes before filtering

### Comparison with Other Filters

**vs. Bilateral Filter:**
- GDTV: Better theoretical foundation, more consistent edge preservation
- Bilateral: Faster for small kernels, more intuitive parameters

**vs. Anisotropic Diffusion:**
- GDTV: More aggressive smoothing in uniform regions
- Anisotropic: Smoother iteration-to-iteration changes

**vs. Total Variation:**
- GDTV: Gradient-dependent (adaptive)
- TV: Constant regularization weight (non-adaptive)

---

## General Notes

### Thread Safety
All algorithms use `thread_info` for context management and are thread-safe at the algorithm level. However, simultaneous operations on the same voxels object are not safe.

### Progress Reporting
All algorithms report progress through `app.threadProgress()`:
- Values range from 0.0 to 1.0
- Called periodically during processing
- Can be used to update progress bars or cancel operations

### Error Handling
- Algorithms assume valid input volumes
- Boundary conditions handled implicitly (zero-gradient or clamping)
- No exceptions thrown for invalid parameters (clamped instead)

### Performance Considerations
- All algorithms modify the voxels object in-place
- Temporary buffers are allocated as needed
- Memory usage: typically 2-4× the volume size
- Processing time scales linearly with volume size (except bilateral filter)

### Best Practices for Algorithm Selection

**For Noise Reduction:**
1. **Light noise + Sharp edges**: Bilateral Filter or GDTV
2. **Medium noise + Feature preservation**: Anisotropic Diffusion
3. **Heavy noise**: Multiple iterations of any filter
4. **Real-time requirements**: Anisotropic Diffusion (fastest)

**For Visualization:**
1. **Low contrast**: Contrast Enhancement
2. **Noisy data**: Filter first, then enhance contrast
3. **Feature enhancement**: GDTV or Anisotropic Diffusion

**Typical Processing Pipeline:**
```cpp
voxels volume = loadVolume("data.raw");

// Step 1: Denoise
volume.bilateralFilter(50.0, 1.5, 1);

// Step 2: Enhance contrast (if needed)
volume.contrastEnhancement(0.5);

// Result: Clean, high-contrast volume
saveVolume(volume, "processed.raw");
```

---

## References

### Anisotropic Diffusion
- Perona, P. and Malik, J. (1990). "Scale-space and edge detection using anisotropic diffusion." IEEE Transactions on Pattern Analysis and Machine Intelligence.

### Bilateral Filter
- Tomasi, C. and Manduchi, R. (1998). "Bilateral filtering for gray and color images." Sixth International Conference on Computer Vision.

### Contrast Enhancement
- Adaptive histogram equalization with resistor network propagation

### GDTV Filter
- Gradient-dependent total variation regularization for image denoising

---

*Document Version: 1.0*  
*Last Updated: April 2026*  
*Library: libcvc 3.0.0*  
*Tested Version: All algorithms validated by the libcvc test suite (volume-ops, geometry, voxels)*
