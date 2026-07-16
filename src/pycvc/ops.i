/*
  ops.i — volume_ops free functions (inc/cvc/volume/volume_ops.h subset).

  Only the field-algebra needed around the smoothed soft-risk pipeline r̃:
  normalize/scale/clip/clamp, go-no-go masking, downsample, stats.  All
  return a new volume by value (shallow-copy semantics are internal to
  libcvc; the returned object is an independent Python-owned wrapper).
*/

namespace cvc {

struct volume_stats {
  double min;
  double max;
  double mean;
  double std_dev;
  unsigned long long num_voxels;
};

// Statistics over the entire volume.
volume_stats compute_stats(const volume &vol);

// Multiply every voxel by a scalar factor.
volume vol_scale(const volume &vol, double factor);

// Linearly remap voxels from [current_min, current_max] to [new_min, new_max]
// — e.g. vol_normalize(v, 0.0, 1.0) forces the r̃ field into [0, 1].
volume vol_normalize(const volume &vol, double new_min, double new_max);

// Zero voxels whose value is >= threshold.
volume vol_clip(const volume &vol, double threshold);

// Clamp voxels to a minimum value.
volume vol_clamp_min(const volume &vol, double min_val);

// Zero voxels where mask is nonzero.
volume vol_mask(const volume &intensity, const volume &mask);

// Zero voxels where mask is zero (inverse mask / go-no-go apply).
volume vol_inverse_mask(const volume &intensity, const volume &mask);

// Downsample by integer factors (nearest-neighbor stride sampling).
volume vol_downsample(const volume &vol, unsigned int factor_x, unsigned int factor_y,
                      unsigned int factor_z);

} // namespace cvc
