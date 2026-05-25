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

#ifndef __VOLMAGICK_COMPOSITEFUNCTION_H__
#define __VOLMAGICK_COMPOSITEFUNCTION_H__

#include <cvc/voxels.h>

namespace cvc {
class composite_function {
public:
  composite_function() {}
  virtual ~composite_function() {}

  /*
    in_vox - input voxels
    in_i,j,k - input indices specifying the current input voxel being composited
    this_vox - the destination voxels object where the result of the composition will be stored
    this_i,j,k - the destination voxel indices
    returns - the result of the composition
  */
  virtual double operator()(const voxels &in_vox, uint64 in_i, uint64 in_j, uint64 in_k,
                            const voxels &this_vox, uint64 this_i, uint64 this_j,
                            uint64 this_k) const = 0;
};

/*
  Replaces the destination voxel with the input voxel
*/
class copy_func : public composite_function {
public:
  copy_func() {}
  virtual ~copy_func() {}

  virtual double operator()(const voxels &in_vox, uint64 in_i, uint64 in_j, uint64 in_k,
                            const voxels &this_vox, uint64 this_i, uint64 this_j,
                            uint64 this_k) const {
    return in_vox(in_i, in_j, in_k); /* make compiler happy.... */
    this_vox(0);
    this_i = 0;
    this_j = 0;
    this_k = 0;
  }
};

/*
  Adds the input voxel to the destination voxel;
*/
class add_func : public composite_function {
public:
  add_func() {}
  virtual ~add_func() {}

  virtual double operator()(const voxels &in_vox, uint64 in_i, uint64 in_j, uint64 in_k,
                            const voxels &this_vox, uint64 this_i, uint64 this_j,
                            uint64 this_k) const {
    return this_vox(this_i, this_j, this_k) + in_vox(in_i, in_j, in_k);
  }
};

/*
  Subtracts the destination voxel with the input voxel
*/
class subtract_func : public composite_function {
public:
  subtract_func() {}
  virtual ~subtract_func() {}

  virtual double operator()(const voxels &in_vox, uint64 in_i, uint64 in_j, uint64 in_k,
                            const voxels &this_vox, uint64 this_i, uint64 this_j,
                            uint64 this_k) const {
    return this_vox(this_i, this_j, this_k) - in_vox(in_i, in_j, in_k);
  }
};
} // namespace cvc

#endif
