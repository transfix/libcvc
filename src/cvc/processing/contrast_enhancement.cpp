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

#include <cvc/core/app.h>
#include <cvc/volume/volmagick.h>
#include <math.h>

namespace cvc {
static inline void contrastEnhancementSlice(double resistor, voxels *paramin, voxels *paramax,
                                            voxels *imgavg, unsigned int k) {
  int i, j;
  int xdim = paramin->XDim(), ydim = paramin->YDim();

  voxels tmpmin(paramin->ctx(), dimension(xdim, ydim, 1), paramin->voxelType());
  voxels tmpmax(paramin->ctx(), dimension(xdim, ydim, 1), paramin->voxelType());
  voxels lcmin(paramin->ctx(), dimension(xdim, ydim, 1), paramin->voxelType());
  voxels lcmax(paramin->ctx(), dimension(xdim, ydim, 1), paramin->voxelType());

  // Note: OpenMP removed to maintain deterministic numerical results
  for (j = 0; j < ydim; j++)
    for (i = 0; i < xdim; i++) {
      lcmin(i, j, 0, (*paramin)(i, j, k));
      lcmax(i, j, 0, (*paramax)(i, j, k));
      tmpmin(i, j, 0, (*paramin)(i, j, k));
      tmpmax(i, j, 0, (*paramax)(i, j, k));
    }

  /* Bottom-up */
  for (i = 1; i < xdim; i++) {
    (*imgavg)(i, 0, k,
              (*imgavg)(i, 0, k) + resistor * ((*imgavg)(i - 1, 0, k) - (*imgavg)(i, 0, k)));
    if (tmpmin(i - 1) < tmpmin(i))
      tmpmin(i, tmpmin(i) + resistor * (tmpmin(i - 1) - tmpmin(i)));
    if (tmpmax(i - 1) > tmpmax(i))
      tmpmax(i, tmpmax(i) + resistor * (tmpmax(i - 1) - tmpmax(i)));
  }

  for (i = xdim - 2; i >= 0; i--) {
    (*imgavg)(i, 0, k,
              (*imgavg)(i, 0, k) + resistor * ((*imgavg)(i + 1, 0, k) - (*imgavg)(i, 0, k)));
    if (tmpmin(i + 1) < tmpmin(i))
      tmpmin(i, tmpmin(i) + resistor * (tmpmin(i + 1) - tmpmin(i)));
    if (tmpmax(i + 1) > tmpmax(i))
      tmpmax(i, tmpmax(i) + resistor * (tmpmax(i + 1) - tmpmax(i)));
  }

  for (j = 1; j < ydim; j++) {
    for (i = 0; i < xdim; i++) {
      (*imgavg)(i, j, k,
                (*imgavg)(i, j, k) + resistor * ((*imgavg)(i, j - 1, k) - (*imgavg)(i, j, k)));
      if (tmpmin(i, j - 1, 0) < tmpmin(i, j, 0))
        tmpmin(i, j, 0, tmpmin(i, j, 0) + resistor * (tmpmin(i, j - 1, 0) - tmpmin(i, j, 0)));
      if (tmpmax(i, j - 1, 0) > tmpmax(i, j, 0))
        tmpmax(i, j, 0, tmpmax(i, j, 0) + resistor * (tmpmax(i, j - 1, 0) - tmpmax(i, j, 0)));
    }

    for (i = 1; i < xdim; i++) {
      (*imgavg)(i, j, k,
                (*imgavg)(i, j, k) + resistor * ((*imgavg)(i - 1, j, k) - (*imgavg)(i, j, k)));
      if (tmpmin(i - 1, j, 0) < tmpmin(i, j, 0))
        tmpmin(i, j, 0, tmpmin(i, j, 0) + resistor * (tmpmin(i - 1, j, 0) - tmpmin(i, j, 0)));
      if (tmpmax(i - 1, j, 0) > tmpmax(i, j, 0))
        tmpmax(i, j, 0, tmpmax(i, j, 0) + resistor * (tmpmax(i - 1, j, 0) - tmpmax(i, j, 0)));
    }

    for (i = xdim - 2; i >= 0; i--) {
      (*imgavg)(i, j, k,
                (*imgavg)(i, j, k) + resistor * ((*imgavg)(i + 1, j, k) - (*imgavg)(i, j, k)));
      if (tmpmin(i + 1, j, 0) < tmpmin(i, j, 0))
        tmpmin(i, j, 0, tmpmin(i, j, 0) + resistor * (tmpmin(i + 1, j, 0) - tmpmin(i, j, 0)));
      if (tmpmax(i + 1, j, 0) > tmpmax(i, j, 0))
        tmpmax(i, j, 0, tmpmax(i, j, 0) + resistor * (tmpmax(i + 1, j, 0) - tmpmax(i, j, 0)));
    }
  }

  /* Top-down */
  j = ydim - 1;
  for (i = 1; i < xdim; i++) {
    (*imgavg)(i, j, k,
              (*imgavg)(i, j, k) + resistor * ((*imgavg)(i - 1, j, k) - (*imgavg)(i, j, k)));
    if (lcmin(i - 1, j, 0) < lcmin(i, j, 0))
      lcmin(i, j, 0, lcmin(i, j, 0) + resistor * (lcmin(i - 1, j, 0) - lcmin(i, j, 0)));
    if (lcmax(i - 1, j, 0) > lcmax(i, j, 0))
      lcmax(i, j, 0, lcmax(i, j, 0) + resistor * (lcmax(i - 1, j, 0) - lcmax(i, j, 0)));
  }

  for (i = xdim - 2; i >= 0; i--) {
    (*imgavg)(i, j, k,
              (*imgavg)(i, j, k) + resistor * ((*imgavg)(i + 1, j, k) - (*imgavg)(i, j, k)));
    if (lcmin(i + 1, j, 0) < lcmin(i, j, 0))
      lcmin(i, j, 0, lcmin(i, j, 0) + resistor * (lcmin(i + 1, j, 0) - lcmin(i, j, 0)));
    if (lcmax(i + 1, j, 0) > lcmax(i, j, 0))
      lcmax(i, j, 0, lcmax(i, j, 0) + resistor * (lcmax(i + 1, j, 0) - lcmax(i, j, 0)));
  }

  for (j = ydim - 2; j >= 0; j--) {
    for (i = 0; i < xdim; i++) {
      (*imgavg)(i, j, k,
                (*imgavg)(i, j, k) + resistor * ((*imgavg)(i, j + 1, k) - (*imgavg)(i, j, k)));
      if (lcmin(i, j + 1, 0) < lcmin(i, j, 0))
        lcmin(i, j, 0, lcmin(i, j, 0) + resistor * (lcmin(i, j + 1, 0) - lcmin(i, j, 0)));
      if (lcmax(i, j + 1, 0) > lcmax(i, j, 0))
        lcmax(i, j, 0, lcmax(i, j, 0) + resistor * (lcmax(i, j + 1, 0) - lcmax(i, j, 0)));
    }

    for (i = 1; i < xdim; i++) {
      (*imgavg)(i, j, k,
                (*imgavg)(i, j, k) + resistor * ((*imgavg)(i - 1, j, k) - (*imgavg)(i, j, k)));
      if (lcmin(i - 1, j, 0) < lcmin(i, j, 0))
        lcmin(i, j, 0, lcmin(i, j, 0) + resistor * (lcmin(i - 1, j, 0) - lcmin(i, j, 0)));
      if (lcmax(i - 1, j, 0) > lcmax(i, j, 0))
        lcmax(i, j, 0, lcmax(i, j, 0) + resistor * (lcmax(i - 1, j, 0) - lcmax(i, j, 0)));
    }

    for (i = xdim - 2; i >= 0; i--) {
      (*imgavg)(i, j, k,
                (*imgavg)(i, j, k) + resistor * ((*imgavg)(i + 1, j, k) - (*imgavg)(i, j, k)));
      if (lcmin(i + 1, j, 0) < lcmin(i, j, 0))
        lcmin(i, j, 0, lcmin(i, j, 0) + resistor * (lcmin(i + 1, j, 0) - lcmin(i, j, 0)));
      if (lcmax(i + 1, j, 0) > lcmax(i, j, 0))
        lcmax(i, j, 0, lcmax(i, j, 0) + resistor * (lcmax(i + 1, j, 0) - lcmax(i, j, 0)));
    }
  }

  // Note: OpenMP removed to maintain deterministic numerical results
  for (j = 0; j < ydim; j++)
    for (i = 0; i < xdim; i++) {
      (*paramin)(i, j, k, MIN(lcmin(i, j, 0), tmpmin(i, j, 0)));
      (*paramax)(i, j, k, MAX(lcmax(i, j, 0), tmpmax(i, j, 0)));
    }
}

voxels &voxels::contrastEnhancement(double resistor) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  int i, j, k, curstep = 0;
  double origmin, origmax, lmin, lmax, img, avg;
  double window, a, b, c, alpha;

  // clamp the resistor value between [0.0,1.0]
  resistor = MIN(1.0, MAX(0.0, resistor));

  origmin = min();
  origmax = max();

  map(0.0, 255.0); // not sure if this is necessary

  voxels upmin(*this), upmax(*this);
  voxels downmin(*this), downmax(*this), imgavg(*this);

  /* Bottom-up propagation */
  contrastEnhancementSlice(resistor, &upmin, &upmax, &imgavg, 0);
  _ctx.threadProgress(float(curstep++) / float(ZDim() * 3));

  for (k = 1; k < int(ZDim()); k++) {
    /* propagation from lower slice */
    // Note: OpenMP removed to maintain deterministic numerical results
    for (j = 0; j < int(YDim()); j++)
      for (i = 0; i < int(XDim()); i++) {
        imgavg(i, j, k, imgavg(i, j, k) + resistor * (imgavg(i, j, k - 1) - imgavg(i, j, k)));
        if (upmin(i, j, k - 1) < upmin(i, j, k))
          upmin(i, j, k, upmin(i, j, k) + resistor * (upmin(i, j, k - 1) - upmin(i, j, k)));
        if (upmax(i, j, k - 1) > upmax(i, j, k))
          upmax(i, j, k, upmax(i, j, k) + resistor * (upmax(i, j, k - 1) - upmax(i, j, k)));
      }
    contrastEnhancementSlice(resistor, &upmin, &upmax, &imgavg, k);
    _ctx.threadProgress(float(curstep++) / float(ZDim() * 3));
  }

  /* Top-down propagation */
  contrastEnhancementSlice(resistor, &downmin, &downmax, &imgavg, ZDim() - 1);
  _ctx.threadProgress(float(curstep++) / float(ZDim() * 3));

  for (k = ZDim() - 2; k >= 0; k--) {
    /* propagation from upper slice */
    // Note: OpenMP removed to maintain deterministic numerical results
    for (j = 0; j < int(YDim()); j++)
      for (i = 0; i < int(XDim()); i++) {
        imgavg(i, j, k, imgavg(i, j, k) + resistor * (imgavg(i, j, k + 1) - imgavg(i, j, k)));
        if (downmin(i, j, k + 1) < downmin(i, j, k))
          downmin(i, j, k, downmin(i, j, k) + resistor * (downmin(i, j, k + 1) - downmin(i, j, k)));
        if (downmax(i, j, k + 1) > downmax(i, j, k))
          downmax(i, j, k, downmax(i, j, k) + resistor * (downmax(i, j, k + 1) - downmax(i, j, k)));
      }
    contrastEnhancementSlice(resistor, &downmin, &downmax, &imgavg, k);
    _ctx.threadProgress(float(curstep++) / float(ZDim() * 3));
  }

  /* stretching */
  // Note: CUDA acceleration is not used for contrast enhancement because:
  // 1. The bottom-up and top-down propagation phases (2/3 of the algorithm) are
  //    inherently sequential and cannot be parallelized
  // 2. Only the final stretching phase (1/3) could use CUDA, providing minimal benefit
  // 3. Data movement overhead between CPU propagation and GPU stretching would
  //    likely negate any performance gains
  // A future optimization could implement the entire algorithm on GPU using
  // parallel scan primitives, but that requires significant algorithmic changes.

  for (k = 0; k < int(ZDim()); k++) {
    // Note: OpenMP removed to maintain deterministic numerical results
    for (j = 0; j < int(YDim()); j++)
      for (i = 0; i < int(XDim()); i++) {
        lmin = MIN(upmin(i, j, k), downmin(i, j, k));
        lmax = MAX(upmax(i, j, k), downmax(i, j, k));
        img = (*this)(i, j, k);
        avg = imgavg(i, j, k);
        window = lmax - lmin;
        window = sqrt(window * (510 - window));

        if (lmin != lmax) {
          img = window * (img - lmin) / (lmax - lmin);
          avg = window * (avg - lmin) / (lmax - lmin);
        }

        alpha = (avg - img) / (181.019 * window);
        if (alpha != 0) {
          a = 0.707 * alpha;
          b = 1.414 * alpha * (img - window) - 1;
          c = 0.707 * alpha * img * (img - 2 * window) + img;
          double discriminant = b * b - 4 * a * c;
          if (discriminant >= 0)
            imgavg(i, j, k, lmin + (-b - sqrt(discriminant)) / (2 * a));
          else
            imgavg(i, j, k, img + lmin); // Fallback if discriminant is negative
        } else
          imgavg(i, j, k, img + lmin);
      }
    _ctx.threadProgress(float(curstep++) / float(ZDim() * 3));
  }

  imgavg.unsetMinMax(); // we need to recalculate min and max for imgavg
  copy(imgavg);

  map(origmin, origmax); // restore the original min/max and relative values
  _ctx.threadProgress(1.0f);

  return *this;
}
}; // namespace cvc
