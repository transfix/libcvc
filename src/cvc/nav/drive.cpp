/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// drive.cpp — see drive.h. Float-equivalent (not bit-identical) transcription of
// the torch drive numerics. This TU must be built without -ffast-math /
// -ffp-contract=fast so the float32 op order tracks torch's as closely as
// possible (the fidelity boundary is float-equivalence, but a fused-multiply-add
// or a fast reciprocal widens the residual past the ~1-ULP target).

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/drive.h>

namespace cvc {
namespace nav {

namespace {

// Bilinear sample of one field plane at grid position (gx,gy) in [-1,1], with
// align_corners=True unnormalization and border padding — the exact arithmetic
// of torch's grid_sampler bilinear path, in float32. Writes phi + (nx,ny) raw
// (un-normalized) normal for the plane's three channels.
inline void sample_plane(const field_stack &f, int plane, float gx, float gy, float &phi, float &nx,
                         float &ny) {
  const float Wf1 = static_cast<float>(f.W - 1);
  const float Hf1 = static_cast<float>(f.H - 1);
  // align_corners=True: ((g + 1) / 2) * (size - 1).
  float ix = (gx + 1.0f) * 0.5f * Wf1;
  float iy = (gy + 1.0f) * 0.5f * Hf1;
  // padding_mode="border": clamp the source coordinate to the valid range first
  // (torch clips the unnormalized coordinate, then bilerps).
  ix = std::min(std::max(ix, 0.0f), Wf1);
  iy = std::min(std::max(iy, 0.0f), Hf1);

  const int ix0 = static_cast<int>(std::floor(ix));
  const int iy0 = static_cast<int>(std::floor(iy));
  const int ix1 = ix0 + 1;
  const int iy1 = iy0 + 1;
  const float wx1 = ix - static_cast<float>(ix0);
  const float wx0 = 1.0f - wx1;
  const float wy1 = iy - static_cast<float>(iy0);
  const float wy0 = 1.0f - wy1;
  // Corner weights, torch order (nw, ne, sw, se).
  const float nw = wx0 * wy0;
  const float ne = wx1 * wy0;
  const float sw = wx0 * wy1;
  const float se = wx1 * wy1;
  // Any out-of-range corner carries a zero weight (ix1 can exceed W-1 only when
  // wx1 == 0), so clamping the corner index matches torch's within-bounds gather.
  const int cx0 = std::min(std::max(ix0, 0), f.W - 1);
  const int cx1 = std::min(std::max(ix1, 0), f.W - 1);
  const int cy0 = std::min(std::max(iy0, 0), f.H - 1);
  const int cy1 = std::min(std::max(iy1, 0), f.H - 1);

  const long HW = static_cast<long>(f.H) * f.W;
  const float *pl = f.data + static_cast<long>(plane) * 3 * HW;
  const long nwi = static_cast<long>(cy0) * f.W + cx0;
  const long nei = static_cast<long>(cy0) * f.W + cx1;
  const long swi = static_cast<long>(cy1) * f.W + cx0;
  const long sei = static_cast<long>(cy1) * f.W + cx1;

  const float *ph = pl;          // channel 0 = phi
  const float *px = pl + HW;     // channel 1 = normal_x
  const float *py = pl + 2 * HW; // channel 2 = normal_y
  phi = ph[nwi] * nw + ph[nei] * ne + ph[swi] * sw + ph[sei] * se;
  nx = px[nwi] * nw + px[nei] * ne + px[swi] * sw + px[sei] * se;
  ny = py[nwi] * nw + py[nei] * ne + py[swi] * sw + py[sei] * se;
}

} // namespace

void sdf_sample(const field_stack &f, const float *on, int n, const int *map_id, float *phi_out,
                float *normal_out, int num_threads) {
  const float S = static_cast<float>(f.S);
  const float cx = static_cast<float>(f.cx);
  const float cy = static_cast<float>(f.cy);
  const float mnx = static_cast<float>(f.mnx);
  const float mny = static_cast<float>(f.mny);
  const float mxx = static_cast<float>(f.mxx);
  const float mxy = static_cast<float>(f.mxy);

  detail::parallel_for(n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    // normalized -> world -> [-1,1] grid, in float32 (matches SDFField.sample).
    const float wx = on[2 * i] / S + cx;
    const float wy = on[2 * i + 1] / S + cy;
    const float gx = 2.0f * (wx - mnx) / (mxx - mnx) - 1.0f;
    const float gy = 2.0f * (wy - mny) / (mxy - mny) - 1.0f;

    float phi, nx, ny;
    sample_plane(f, plane, gx, gy, phi, nx, ny);
    phi_out[i] = phi;
    // unit outward normal: nrm / (|nrm| + 1e-6), matching SDFField.sample.
    const float mag = std::sqrt(nx * nx + ny * ny) + 1e-6f;
    normal_out[2 * i] = nx / mag;
    normal_out[2 * i + 1] = ny / mag;
  });
}

} // namespace nav
} // namespace cvc
