/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __CVC_VOLUME_OPS_H__
#define __CVC_VOLUME_OPS_H__

#include <cvc/volume.h>
#include <cvc/bounding_box.h>
#include <cvc/exception.h>

#include <string>
#include <vector>

namespace CVC_NAMESPACE
{
  CVC_DEF_EXCEPTION(dimension_mismatch);

  // ── Volume statistics ──

  struct volume_stats
  {
    double min;
    double max;
    double mean;
    double std_dev;
    uint64 num_voxels;
  };

  // Compute statistics over the entire volume.
  volume_stats compute_stats(const volume& vol);

  // Compute statistics restricted to a bounding box region.
  volume_stats compute_stats(const volume& vol, const bounding_box& region);

  // ── Element-wise arithmetic ──

  // Add two volumes element-wise.  Dimensions must match.
  volume vol_add(const volume& a, const volume& b);

  // Subtract b from a element-wise.  Dimensions must match.
  volume vol_subtract(const volume& a, const volume& b);

  // Absolute difference |a - b|.  Dimensions must match.
  volume vol_difference(const volume& a, const volume& b);

  // Element-wise average of two volumes.  Dimensions must match.
  volume vol_average(const volume& a, const volume& b);

  // ── Scalar operations ──

  // Multiply every voxel by a scalar factor.
  volume vol_scale(const volume& vol, double factor);

  // Linearly remap voxels from [current_min, current_max] to [new_min, new_max].
  volume vol_normalize(const volume& vol, double new_min, double new_max);

  // Zero voxels whose value is >= threshold.
  volume vol_clip(const volume& vol, double threshold);

  // Clamp voxels to a minimum value.
  volume vol_clamp_min(const volume& vol, double min_val);

  // Negate all voxel values (multiply by -1).
  volume vol_negate(const volume& vol);

  // ── Masking ──

  // Zero voxels where mask is nonzero.
  volume vol_mask(const volume& intensity, const volume& mask);

  // Zero voxels where mask is zero (inverse mask).
  volume vol_inverse_mask(const volume& intensity, const volume& mask);

  // ── Spatial ──

  // Downsample by integer factors (nearest-neighbor stride sampling).
  volume vol_downsample(const volume& vol,
                        unsigned int factor_x,
                        unsigned int factor_y,
                        unsigned int factor_z);

  // ── Rotation ──

  // Rotate a volume around the Z-axis by the given angle (radians).
  // Uses trilinear interpolation.  Returns a volume of the same dimensions.
  volume vol_rotate_z(const volume& vol, double angle_rad);

  // ── Structural similarity (SSIM) ──

  struct ssim_result
  {
    double mean_ssim;   // mean SSIM across all voxels
    volume ssim_map;    // per-voxel SSIM values
  };

  // Compute the Structural Similarity Index (Wang et al. 2004)
  // between two volumes.  Dimensions must match.
  // window_size: side length of the Gaussian weighting window (default 11).
  // sigma: standard deviation of the Gaussian kernel (default 1.5).
  ssim_result vol_ssim(const volume& a, const volume& b,
                       int window_size = 11, double sigma = 1.5);

  // ── Projection / back-projection ──

  // Forward-project a volume for each angle in angles_rad (rotation about Y).
  // Returns a volume of dimension (XDim × YDim × num_angles).
  volume vol_project(const volume& vol,
                     const std::vector<double>& angles_rad,
                     double step = 0.5);

  // Filtered back-projection (tomographic reconstruction).
  // projections: volume of dimension (XDim × YDim × num_angles).
  // angles_rad: projection angles in radians.
  // output_dim: side length of the cubic output volume.
  // apply_filter: if true, apply ramp filter via FFT before back-projection.
  volume vol_back_project(const volume& projections,
                          const std::vector<double>& angles_rad,
                          unsigned int output_dim,
                          bool apply_filter = true);

  // ── Image I/O (requires ImageMagick at build time) ──

  // Export volume slices as a numbered image sequence.
  // format: output file pattern, e.g. "slice_%05d.png"
  void vol_to_slices(const volume& vol,
                     const std::string& directory,
                     const std::string& format = "slice_%05d.png");

  // Import a stack of images into a volume.
  // paths: ordered list of image file paths (one per Z-slice).
  volume slices_to_volume(const std::vector<std::string>& paths,
                          const bounding_box& bbox = bounding_box(0,0,0,1,1,1));

  // ── RGBA multi-variable operations ──

  // Merge 4 single-variable volumes into one 4-variable RGBA volume.
  // All input volumes must have matching dimensions.
  volume vol_rgba_merge(const volume& r, const volume& g,
                        const volume& b, const volume& a);

  // Split a multi-variable volume into separate single-variable volumes.
  // Returns a vector of volumes, one per variable in the input.
  std::vector<volume> vol_split_vars(const volume& vol);
}

#endif
