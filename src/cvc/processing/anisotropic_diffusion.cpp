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

#include <cvc/app.h>
#include <cvc/volmagick.h>
#include <math.h>

#ifdef CVC_USING_CUDA
#include <cvc/cuda_utils.h>
#endif

namespace cvc {
voxels &voxels::anisotropicDiffusion(unsigned int iterations) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  double K_para = 3;
  double Lamda_para = 0.16f;
  unsigned int m;
  uint64 k, stepnum = 0;

  uint64 numSteps = ZDim() * iterations;

#ifdef CVC_USING_CUDA
  // CUDA implementation - process slice by slice
  if (cuda_device_manager::cuda_available() && using_cuda()) {
    voxels tempt(*this);

    for (m = 0; m < iterations; m++) {
      for (k = 0; k < ZDim(); k++) {
        // Launch CUDA kernel for this slice
        cuda_anisotropic_diffusion_slice(get_data_ptr(),       // source data
                                         tempt.get_data_ptr(), // destination data
                                         XDim(), YDim(), ZDim(),
                                         k, // current slice
                                         K_para, Lamda_para, voxelType());

        _ctx.threadProgress(float(stepnum) / float(numSteps));
        stepnum++;
      }

      // Copy result back for next iteration
      (*this) = tempt;
    }

    _ctx.threadProgress(1.0f);
    return *this;
  }
#endif

  // CPU implementation - original algorithm
  double cn, cs, ce, cw, cu, cd;
  double delta_n, delta_s, delta_e, delta_w, delta_u, delta_d;
  int64_t i, j;
  // Use deep copy for temporary buffer to avoid race conditions with OpenMP
  voxels tempt(_ctx, voxel_dimensions(), voxelType());
  tempt.copy(*this, true); // Deep copy

  for (m = 0; m < iterations; m++) {
    for (k = 0; k < ZDim(); k++) {
#ifdef _OPENMP
#pragma omp parallel for private(i, j, cn, cs, ce, cw, cu, cd, delta_n, delta_s, delta_e, delta_w, \
                                     delta_u, delta_d) schedule(dynamic)
#endif
      for (j = 0; j < YDim(); j++) {
        for (i = 0; i < XDim(); i++) {
          if (j < YDim() - 1)
            delta_s = (*this)(i, j + 1, k) - (*this)(i, j, k);
          else
            delta_s = 0.0;
          if (j > 0)
            delta_n = (*this)(i, j - 1, k) - (*this)(i, j, k);
          else
            delta_n = 0.0;
          if (i < XDim() - 1)
            delta_e = (*this)(i + 1, j, k) - (*this)(i, j, k);
          else
            delta_e = 0.0;
          if (i > 0)
            delta_w = (*this)(i - 1, j, k) - (*this)(i, j, k);
          else
            delta_w = 0.0;
          if (k < ZDim() - 1)
            delta_u = (*this)(i, j, k + 1) - (*this)(i, j, k);
          else
            delta_u = 0.0;
          if (k > 0)
            delta_d = (*this)(i, j, k - 1) - (*this)(i, j, k);
          else
            delta_d = 0.0;

          cn = 1.0f / (1.0f + ((delta_n * delta_n) / (K_para * K_para)));
          cs = 1.0f / (1.0f + ((delta_s * delta_s) / (K_para * K_para)));
          ce = 1.0f / (1.0f + ((delta_e * delta_e) / (K_para * K_para)));
          cw = 1.0f / (1.0f + ((delta_w * delta_w) / (K_para * K_para)));
          cu = 1.0f / (1.0f + ((delta_u * delta_u) / (K_para * K_para)));
          cd = 1.0f / (1.0f + ((delta_d * delta_d) / (K_para * K_para)));

          tempt(i, j, k,
                (*this)(i, j, k) + Lamda_para * (cn * delta_n + cs * delta_s + ce * delta_e +
                                                 cw * delta_w + cu * delta_u + cd * delta_d));
        }
      }

      _ctx.threadProgress(float(stepnum) / float(numSteps));
      stepnum++;
    }

    // Copy result back using deep copy to avoid shallow copy aliasing
    copy(tempt, true);
  }

  _ctx.threadProgress(1.0f);
  return *this;
}
} // namespace cvc
