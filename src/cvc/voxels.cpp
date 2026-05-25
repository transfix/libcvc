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

#include <boost/current_function.hpp>
#include <cvc/app.h>
#include <cvc/composite_function.h>
#include <cvc/utility.h>
#include <cvc/voxels.h>

namespace cvc {
voxels::voxels(app &ctx, const dimension &d, data_type vt)
    : _ctx(ctx), _dimension(d), _voxelType(vt), _minIsSet(false), _maxIsSet(false),
      _histogram(nullptr), _histogramSize(0), _histogramDirty(true), _using_cuda(false),
      _cuda_device_id(-1) {
  try {
    uint64 size = XDim() * YDim() * ZDim() * voxelSize();
    _voxels.reset(new unsigned char[size]);
    std::memset(_voxels.get(), 0, size);
  } catch (std::bad_alloc &e) {
    throw memory_allocation_error("Could not allocate memory for voxels!");
  }
}

voxels::voxels(app &ctx, const void *v, const dimension &d, data_type vt)
    : _ctx(ctx), _dimension(d), _voxelType(vt), _minIsSet(false), _maxIsSet(false),
      _histogram(nullptr), _histogramSize(0), _histogramDirty(true), _using_cuda(false),
      _cuda_device_id(-1) {
  try {
    uint64 size = XDim() * YDim() * ZDim() * voxelSize();
    _voxels.reset(new unsigned char[size]);
    std::memcpy(_voxels.get(), v, size);
  } catch (std::bad_alloc &e) {
    throw memory_allocation_error("Could not allocate memory for voxels!");
  }
}

voxels::voxels(const voxels &v)
    : _ctx(v._ctx), _dimension(v.voxel_dimensions()), _voxelType(v.voxelType()), _minIsSet(false),
      _maxIsSet(false), _histogram(nullptr), _histogramSize(0), _histogramDirty(true),
      _using_cuda(v._using_cuda), _cuda_device_id(v._cuda_device_id), _voxels(v._voxels),
      _cuda_unified_ptr(v._cuda_unified_ptr) {
  // Shallow copy: share both CPU and CUDA memory via reference counting
  // boost::shared_array for CPU memory, std::shared_ptr for CUDA unified memory

  if (v.minIsSet() && v.maxIsSet()) {
    min(v.min());
    max(v.max());
  }
}

voxels::~voxels() {
  // No explicit cleanup needed:
  // - boost::shared_array automatically frees CPU memory when refcount hits 0
  // - std::shared_ptr with CudaManagedDeleter automatically frees CUDA memory when refcount hits 0
}

// ------------------------
// voxels::voxel_dimensions
// ------------------------
// Purpose:
//   Changes the dimensions of this voxels dataset.
// ---- Change History ----
// ??/??/2007 -- Joe R. -- Creation.
// 12/09/2025 -- Joe R. -- Updated for typed 3D arrays
void voxels::voxel_dimensions(const dimension &d) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (d.isNull())
    throw null_dimension("Null volume dimension.");

  if (voxel_dimensions() == d)
    return;

  // Shallow copy constructor creates backup sharing memory
  voxels bak(*this);

#ifdef CVC_USING_CUDA
  // Reset CUDA memory reference (shared_ptr will auto-cleanup if last reference)
  if (_using_cuda) {
    _using_cuda = false;
    _cuda_device_id = -1;
    _cuda_unified_ptr.reset();
  }
#endif

  // allocate for the new dimension
  try {
    size_t data_size = d.size() * voxelSize();
    _voxels.reset(new unsigned char[data_size]);
    std::memset(_voxels.get(), 0, data_size);
  } catch (std::bad_alloc &e) {
    throw memory_allocation_error("Could not allocate memory for voxels!");
  }

  _dimension = d;

  // copy the voxels back
  for (uint64 k = 0; k < ZDim() && k < bak.ZDim(); k++)
    for (uint64 j = 0; j < YDim() && j < bak.YDim(); j++)
      for (uint64 i = 0; i < XDim() && i < bak.XDim(); i++)
        (*this)(i, j, k, bak(i, j, k));
}

void voxels::voxelType(data_type vt) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (voxelType() == vt)
    return;

  voxels bak(*this); // backup voxels into bak
  _voxelType = vt;

  // allocate for the new voxel type
  try {
    uint64 size = XDim() * YDim() * ZDim() * voxelSize();
    _voxels.reset(new unsigned char[size]);
  } catch (std::bad_alloc &e) {
    throw memory_allocation_error("Could not allocate memory for voxels!");
  }

  // copy the voxels back with type conversion
  uint64 len = XDim() * YDim() * ZDim();
  for (uint64 i = 0; i < len; i++)
    (*this)(i, bak(i));
}

void voxels::calcMinMax() const {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  double val;
  size_t len = XDim() * YDim() * ZDim();
  if (len == 0)
    return;
  val = (*this)(0, 0, 0);
  _min = _max = val;

  switch (voxelType()) {
  case UChar: {
    unsigned char uchar_min = static_cast<unsigned char>(_min);
    unsigned char uchar_max = static_cast<unsigned char>(_max);
    unsigned char *data = reinterpret_cast<unsigned char *>(_voxels.get());
    for (size_t i = 0; i < len; i++) {
      unsigned char v = data[i];
      if (v < uchar_min)
        uchar_min = v;
      if (v > uchar_max)
        uchar_max = v;
      if ((i % (XDim() * YDim())) == 0) {
        _ctx.threadProgress(float(i / (XDim() * YDim())) / float(ZDim()));
      }
    }
    _min = double(uchar_min);
    _max = double(uchar_max);
    break;
  }
  case UShort: {
    unsigned short ushort_min = static_cast<unsigned short>(_min);
    unsigned short ushort_max = static_cast<unsigned short>(_max);
    unsigned short *data = reinterpret_cast<unsigned short *>(_voxels.get());
    for (size_t i = 0; i < len; i++) {
      unsigned short v = data[i];
      if (v < ushort_min)
        ushort_min = v;
      if (v > ushort_max)
        ushort_max = v;
      if ((i % (XDim() * YDim())) == 0) {
        _ctx.threadProgress(float(i / (XDim() * YDim())) / float(ZDim()));
      }
    }
    _min = double(ushort_min);
    _max = double(ushort_max);
    break;
  }
  case UInt: {
    unsigned int uint_min = static_cast<unsigned int>(_min);
    unsigned int uint_max = static_cast<unsigned int>(_max);
    unsigned int *data = reinterpret_cast<unsigned int *>(_voxels.get());
    for (size_t i = 0; i < len; i++) {
      unsigned int v = data[i];
      if (v < uint_min)
        uint_min = v;
      if (v > uint_max)
        uint_max = v;
      if ((i % (XDim() * YDim())) == 0) {
        _ctx.threadProgress(float(i / (XDim() * YDim())) / float(ZDim()));
      }
    }
    _min = double(uint_min);
    _max = double(uint_max);
    break;
  }
  case Float: {
    float float_min = static_cast<float>(_min);
    float float_max = static_cast<float>(_max);
    float *data = reinterpret_cast<float *>(_voxels.get());
    for (size_t i = 0; i < len; i++) {
      float v = data[i];
      if (v < float_min)
        float_min = v;
      if (v > float_max)
        float_max = v;
      if ((i % (XDim() * YDim())) == 0) {
        _ctx.threadProgress(float(i / (XDim() * YDim())) / float(ZDim()));
      }
    }
    _min = double(float_min);
    _max = double(float_max);
    break;
  }
  case Double: {
    double double_min = _min;
    double double_max = _max;
    double *data = reinterpret_cast<double *>(_voxels.get());
    for (size_t i = 0; i < len; i++) {
      double v = data[i];
      if (v < double_min)
        double_min = v;
      if (v > double_max)
        double_max = v;
      if ((i % (XDim() * YDim())) == 0) {
        _ctx.threadProgress(float(i / (XDim() * YDim())) / float(ZDim()));
      }
    }
    _min = double_min;
    _max = double_max;
    break;
  }
  case UInt64: {
    uint64 uint64_min = static_cast<uint64>(_min);
    uint64 uint64_max = static_cast<uint64>(_max);
    uint64 *data = reinterpret_cast<uint64 *>(_voxels.get());
    for (size_t i = 0; i < len; i++) {
      uint64 v = data[i];
      if (v < uint64_min)
        uint64_min = v;
      if (v > uint64_max)
        uint64_max = v;
      if ((i % (XDim() * YDim())) == 0) {
        _ctx.threadProgress(float(i / (XDim() * YDim())) / float(ZDim()));
      }
    }
    _min = double(uint64_min);
    _max = double(uint64_max);
    break;
  }
  }

  _minIsSet = _maxIsSet = true;
  _ctx.threadProgress(1.0f);
}

void voxels::calcHistogram(uint64 size) const {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (!_histogramDirty && _histogramSize == size)
    return;

  _histogramSize = size;
  _histogram.reset(new uint64[size]);
  memset(_histogram.get(), 0, sizeof(uint64) * size);

  // Compute min/max BEFORE parallel region to avoid race conditions
  // on _min/_max member variables when called from multiple threads
  double current_min = min();
  double current_max = max();
  double range = current_max - current_min;

  // Handle degenerate case where all values are the same
  if (range == 0.0) {
    // All values are the same, put everything in the middle bin
    _histogram[size / 2] = XDim() * YDim() * ZDim();
    _histogramDirty = false;
    _ctx.threadProgress(1.0f);
    return;
  }

  for (uint64 k = 0; k < ZDim(); k++) {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (uint64 j = 0; j < YDim(); j++)
      for (uint64 i = 0; i < XDim(); i++) {
        uint64 offset = uint64((((*this)(i, j, k) - current_min) / range) * double(size - 1));
#ifdef _OPENMP
#pragma omp atomic
#endif
        _histogram[offset]++;
      }
    _ctx.threadProgress(float(k) / float(ZDim()));
  }

  _histogramDirty = false;
  _ctx.threadProgress(1.0f);
}

double voxels::min(uint64 off_x, uint64 off_y, uint64 off_z, const dimension &dim) const {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

#ifdef CVC_USING_CUDA
  // Use CUDA kernel if CUDA is enabled and unified memory is available
  if (_using_cuda && _cuda_unified_ptr) {
    try {
      double result = cuda_compute_min(_cuda_unified_ptr.get(), off_x, off_y, off_z, dim[0], dim[1],
                                       dim[2], voxel_dimensions()[0], voxel_dimensions()[1],
                                       voxel_dimensions()[2], voxelType());

      _ctx.threadProgress(1.0f);
      return result;
    } catch (const cuda_error &e) {
      // Fall back to CPU implementation if CUDA fails
    }
  }
#endif

  // CPU implementation (fallback or when CUDA not available)
  double val = (*this)(off_x, off_y, off_z); // Initialize with first value in subvolume

#ifdef _OPENMP
// Parallel map-reduce: each thread finds min in its chunk, then combine.
// MSVC's LLVM OpenMP frontend (/openmp:llvm) is strict about the form
// permitted inside `collapse(...)`: the inner loop bounds here are
// non-rectangular w.r.t. OpenMP's "canonical loop form" and get rejected
// with C7720. Fall back to a plain outer-loop parallel-for on MSVC.
#if defined(_MSC_VER)
#pragma omp parallel for reduction(min : val) schedule(static)
#else
#pragma omp parallel for reduction(min : val) collapse(3) schedule(static)
#endif
#endif
  for (uint64 k = 0; k < dim[2]; k++)
    for (uint64 j = 0; j < dim[1]; j++)
      for (uint64 i = 0; i < dim[0]; i++) {
        double curr = (*this)(i + off_x, j + off_y, k + off_z);
        if (val > curr)
          val = curr;
      }

  _ctx.threadProgress(1.0f);
  return val;
}

double voxels::max(uint64 off_x, uint64 off_y, uint64 off_z, const dimension &dim) const {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

#ifdef CVC_USING_CUDA
  // Use CUDA kernel if CUDA is enabled and unified memory is available
  if (_using_cuda && _cuda_unified_ptr) {
    try {
      double result = cuda_compute_max(_cuda_unified_ptr.get(), off_x, off_y, off_z, dim[0], dim[1],
                                       dim[2], voxel_dimensions()[0], voxel_dimensions()[1],
                                       voxel_dimensions()[2], voxelType());

      _ctx.threadProgress(1.0f);
      return result;
    } catch (const cuda_error &e) {
      // Fall back to CPU implementation if CUDA fails
    }
  }
#endif

  // CPU implementation (fallback or when CUDA not available)
  double val = (*this)(off_x, off_y, off_z); // Initialize with first value in subvolume

#ifdef _OPENMP
// Parallel map-reduce: each thread finds max in its chunk, then combine.
// See the min() variant above for why collapse(3) is disabled on MSVC.
#if defined(_MSC_VER)
#pragma omp parallel for reduction(max : val) schedule(static)
#else
#pragma omp parallel for reduction(max : val) collapse(3) schedule(static)
#endif
#endif
  for (uint64 k = 0; k < dim[2]; k++)
    for (uint64 j = 0; j < dim[1]; j++)
      for (uint64 i = 0; i < dim[0]; i++) {
        double curr = (*this)(i + off_x, j + off_y, k + off_z);
        if (val < curr)
          val = curr;
      }

  _ctx.threadProgress(1.0f);
  return val;
}

// ------------------------
// voxels::copy (zero-parameter)
// ------------------------
// Purpose:
//   Creates a deep copy of itself. Allocates new memory and copies all data,
//   creating an independent voxels object. This is a convenience method to
//   avoid creating a temporary voxels object just to deep copy from.
// ---- Change History ----
// 12/10/2025 -- Joe R. -- Creation.
voxels &voxels::copy() {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  // Create a temporary shallow copy to copy from
  voxels temp(*this);

  // Now deep copy from the temporary
  return copy(temp, true);
}

voxels &voxels::copy(const voxels &vox, bool deepCopy) {
  if (this == &vox)
    return *this;

  _voxelType = vox._voxelType;
  _dimension = vox._dimension;

  if (deepCopy) {
    // Deep copy: allocate new memory and copy data
    try {
      size_t data_size = XDim() * YDim() * ZDim() * voxelSize();
      _voxels.reset(new unsigned char[data_size]);
      std::memcpy(_voxels.get(), vox._voxels.get(), data_size);

#ifdef CVC_USING_CUDA
      // Deep copy doesn't preserve CUDA state - new independent copy
      _using_cuda = false;
      _cuda_device_id = -1;
      _cuda_unified_ptr.reset(); // Release reference to CUDA memory
#endif
    } catch (std::bad_alloc &e) {
      throw memory_allocation_error("Could not allocate memory for deep copy of voxels!");
    }
  } else {
    // Shallow copy: share the underlying data via reference counting
    _voxels = vox._voxels;

#ifdef CVC_USING_CUDA
    // Shallow copy preserves CUDA state and shares CUDA memory
    _using_cuda = vox._using_cuda;
    _cuda_device_id = vox._cuda_device_id;
    _cuda_unified_ptr = vox._cuda_unified_ptr; // shared_ptr handles reference counting
#endif
  }

  if (vox.minIsSet() && vox.maxIsSet()) {
    min(vox.min());
    max(vox.max());
  } else
    unsetMinMax();

  _histogram = vox._histogram;
  _histogramSize = vox._histogramSize;
  _histogramDirty = vox._histogramDirty;

  return *this;
}

voxels &voxels::sub(uint64 off_x, uint64 off_y, uint64 off_z, const dimension &subvoldim) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (off_x + subvoldim[0] - 1 >= voxel_dimensions()[0] ||
      off_y + subvoldim[1] - 1 >= voxel_dimensions()[1] ||
      off_z + subvoldim[2] - 1 >= voxel_dimensions()[2])
    throw index_out_of_bounds("Subvolume offset and/or dimension is out of bounds");

  // Deep copy constructor creates independent backup
  voxels tmp(*this);

  voxel_dimensions(subvoldim); // change this object's dimension to the subvolume dimension

  // copy the subvolume voxels
  for (uint64 k = 0; k < voxel_dimensions()[2]; k++) {
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (uint64 j = 0; j < voxel_dimensions()[1]; j++)
      for (uint64 i = 0; i < voxel_dimensions()[0]; i++)
        (*this)(i, j, k, tmp(i + off_x, j + off_y, k + off_z));
    _ctx.threadProgress(float(k) / float(voxel_dimensions()[2]));
  }

  _ctx.threadProgress(1.0f);
  return *this;
}

voxels &voxels::fill(double val) { return fillsub(0, 0, 0, voxel_dimensions(), val); }

voxels &voxels::fillsub(uint64 off_x, uint64 off_y, uint64 off_z, const dimension &subvoldim,
                        double val) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (off_x + subvoldim[0] - 1 >= voxel_dimensions()[0] ||
      off_y + subvoldim[1] - 1 >= voxel_dimensions()[1] ||
      off_z + subvoldim[2] - 1 >= voxel_dimensions()[2])
    throw index_out_of_bounds("Subvolume offset and/or dimension is out of bounds");

  for (uint64 k = 0; k < subvoldim[2]; k++) {
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (uint64 j = 0; j < subvoldim[1]; j++)
      for (uint64 i = 0; i < subvoldim[0]; i++)
        (*this)(i + off_x, j + off_y, k + off_z, val);
    _ctx.threadProgress(float(k) / float(subvoldim[2]));
  }

  _ctx.threadProgress(1.0f);
  return *this;
}

voxels &voxels::map(double min_, double max_) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  // Compute min/max BEFORE parallel region to avoid race conditions
  // on _min/_max member variables when called from multiple threads
  double current_min = min();
  double current_max = max();
  double range = current_max - current_min;

  // Handle degenerate case where all values are the same
  if (range == 0.0) {
    // Just set all values to the middle of the target range
    fill((min_ + max_) / 2.0);
    min(min_);
    max(max_);
    return *this;
  }

  double scale = (max_ - min_) / range;

  // Call preWrite() once BEFORE parallel region to:
  // 1. Set _histogramDirty = true (avoid race on this flag)
  // 2. Ensure unique voxel data (copy-on-write if needed)
  // This prevents multiple threads from calling preWrite() simultaneously
  preWrite();

  uint64 xdim = XDim(), ydim = YDim(), zdim = ZDim();
  byte *data = get_data_ptr(); // Get pointer once, reuse in parallel region

  for (uint64 k = 0; k < zdim; k++) {
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (uint64 j = 0; j < ydim; j++)
      for (uint64 i = 0; i < xdim; i++) {
        uint64 idx = k * xdim * ydim + j * xdim + i;
        double new_val = min_ + ((*this)(idx)-current_min) * scale;

        // Direct write bypassing operator() to avoid preWrite() calls
        switch (voxelType()) {
        case Float:
          reinterpret_cast<float *>(data)[idx] = static_cast<float>(new_val);
          break;
        case Double:
          reinterpret_cast<double *>(data)[idx] = static_cast<double>(new_val);
          break;
        default:
          // For integer types, cast appropriately
          reinterpret_cast<unsigned char *>(data)[idx] = static_cast<unsigned char>(new_val);
          break;
        }
      }
    _ctx.threadProgress(float(k) / float(zdim));
  }
  min(min_);
  max(max_); // set the new min and max
  return *this;
}

// ------------------------
// voxels::resizeTrilinearCPU
// ------------------------
// Purpose:
//   Helper function for CPU-based trilinear interpolation resize.
//   Performs the core interpolation loop that is shared between
//   resize(dimension) and resize(bounding_box).
// Parameters:
//   newvox - destination voxels object (must be pre-allocated)
//   offset_x/y/z - starting offset in source voxel coordinates
//   scale_x/y/z - scale factor per destination voxel
//   clampCoords - if true, clamp coords and set xRes/yRes/zRes to 0 at boundaries (bbox mode)
//                 if false, don't clamp or modify xRes/yRes/zRes (dimension mode)
// ---- Change History ----
// 12/28/2025 -- Joe R. -- Created to eliminate code duplication
void voxels::resizeTrilinearCPU(voxels &newvox, double offset_x, double offset_y, double offset_z,
                                double scale_x, double scale_y, double scale_z,
                                bool clampCoords) const {
  uint64 i, j, k;
  double val[8];
  uint64 resXIndex = 0, resYIndex = 0, resZIndex = 0;
  uint64 ValIndex[8];
  double xPosition = 0, yPosition = 0, zPosition = 0;
  double xRes = 0, yRes = 0, zRes = 0;

  for (k = 0; k < newvox.ZDim(); k++) {
    // Calculate position in source voxel coordinate system
    double z = offset_z + double(k) * scale_z;
    if (clampCoords) {
      // Clamp to valid range (bbox mode)
      if (z < 0)
        z = 0;
      if (z >= voxel_dimensions()[2] - 1)
        z = voxel_dimensions()[2] - 1 - 0.001;
    }
    resZIndex = uint64(z);
    zPosition = z - uint64(z);
    zRes = 1;

#ifdef _OPENMP
#pragma omp parallel for private(i, j, resXIndex, resYIndex, xPosition, yPosition, xRes, yRes,     \
                                     val, ValIndex) schedule(dynamic)
#endif
    for (j = 0; j < newvox.YDim(); j++) {
      double y = offset_y + double(j) * scale_y;
      if (clampCoords) {
        // Clamp to valid range (bbox mode)
        if (y < 0)
          y = 0;
        if (y >= voxel_dimensions()[1] - 1)
          y = voxel_dimensions()[1] - 1 - 0.001;
      }
      resYIndex = uint64(y);
      yPosition = y - uint64(y);
      yRes = 1;

      for (i = 0; i < newvox.XDim(); i++) {
        double x = offset_x + double(i) * scale_x;
        if (clampCoords) {
          // Clamp to valid range (bbox mode)
          if (x < 0)
            x = 0;
          if (x >= voxel_dimensions()[0] - 1)
            x = voxel_dimensions()[0] - 1 - 0.001;
        }
        resXIndex = uint64(x);
        xPosition = x - uint64(x);
        xRes = 1;

        // find index to get eight voxel values
        ValIndex[0] = resZIndex * voxel_dimensions()[0] * voxel_dimensions()[1] +
                      resYIndex * voxel_dimensions()[0] + resXIndex;
        ValIndex[1] = ValIndex[0] + 1;
        ValIndex[2] = resZIndex * voxel_dimensions()[0] * voxel_dimensions()[1] +
                      (resYIndex + 1) * voxel_dimensions()[0] + resXIndex;
        ValIndex[3] = ValIndex[2] + 1;
        ValIndex[4] = (resZIndex + 1) * voxel_dimensions()[0] * voxel_dimensions()[1] +
                      resYIndex * voxel_dimensions()[0] + resXIndex;
        ValIndex[5] = ValIndex[4] + 1;
        ValIndex[6] = (resZIndex + 1) * voxel_dimensions()[0] * voxel_dimensions()[1] +
                      (resYIndex + 1) * voxel_dimensions()[0] + resXIndex;
        ValIndex[7] = ValIndex[6] + 1;

        // Handle boundary conditions
        if (resXIndex >= voxel_dimensions()[0] - 1) {
          if (clampCoords)
            xRes = 0; // Only set to 0 in bbox mode
          ValIndex[1] = ValIndex[0];
          ValIndex[3] = ValIndex[2];
          ValIndex[5] = ValIndex[4];
          ValIndex[7] = ValIndex[6];
        }
        if (resYIndex >= voxel_dimensions()[1] - 1) {
          if (clampCoords)
            yRes = 0; // Only set to 0 in bbox mode
          ValIndex[2] = ValIndex[0];
          ValIndex[3] = ValIndex[1];
          ValIndex[6] = ValIndex[4];
          ValIndex[7] = ValIndex[5];
        }
        if (resZIndex >= voxel_dimensions()[2] - 1) {
          if (clampCoords)
            zRes = 0; // Only set to 0 in bbox mode
          ValIndex[4] = ValIndex[0];
          ValIndex[5] = ValIndex[1];
          ValIndex[6] = ValIndex[2];
          ValIndex[7] = ValIndex[3];
        }

        for (int Index = 0; Index < 8; Index++)
          val[Index] = (*this)(ValIndex[Index]);

        newvox(i, j, k, getTriVal(val, xPosition, yPosition, zPosition, xRes, yRes, zRes));
      }
    }

    _ctx.threadProgress(float(k) / float(newvox.ZDim()));
  }
}

voxels &voxels::resize(const dimension &newdim) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  double inSpaceX, inSpaceY, inSpaceZ;
  double val[8];
  uint64 resXIndex = 0, resYIndex = 0, resZIndex = 0;
  uint64 ValIndex[8];
  double xPosition = 0, yPosition = 0, zPosition = 0;
  double xRes = 0, yRes = 0, zRes = 0;
  uint64 i, j, k;
  double x, y, z;

  if (newdim.isNull())
    throw null_dimension("Null voxels dimension.");

  if (voxel_dimensions() == newdim)
    return *this; // nothing needs to be done

  voxels newvox(_ctx, newdim, voxelType());

  // we require a dimension of at least 2^3
  if (newdim < dimension(2, 2, 2)) {
    // resize this object as if it was 2x2x2
    resize(dimension(2, 2, 2));

    // copy it into newvox
    newvox.copy(*this);

    // change this object's dimension to the real dimension (destroying voxel values, hence the
    // backup)
    voxel_dimensions(newdim);

    for (k = 0; k < ZDim(); k++)
      for (j = 0; j < YDim(); j++)
        for (i = 0; i < XDim(); i++)
          (*this)(i, j, k, newvox(i, j, k));

    return *this;
  }

  // inSpace calculation
  inSpaceX = (double)(voxel_dimensions()[0] - 1) / (newdim[0] - 1);
  inSpaceY = (double)(voxel_dimensions()[1] - 1) / (newdim[1] - 1);
  inSpaceZ = (double)(voxel_dimensions()[2] - 1) / (newdim[2] - 1);

#ifdef CVC_USING_CUDA
  // Use CUDA kernel if CUDA is enabled and unified memory is available
  if (_using_cuda && _cuda_unified_ptr) {
    try {
      // Allocate CUDA unified memory for destination
      newvox.enableCUDA(_cuda_device_id);

      // Launch CUDA kernel for trilinear interpolation
      cuda_resize_trilinear(_cuda_unified_ptr.get(),        // source data
                            newvox._cuda_unified_ptr.get(), // destination data
                            voxel_dimensions()[0], voxel_dimensions()[1], voxel_dimensions()[2],
                            newdim[0], newdim[1], newdim[2], inSpaceX, inSpaceY, inSpaceZ,
                            voxelType());

      // Copy the result
      copy(newvox);
      _ctx.threadProgress(1.0f);

      return *this;
    } catch (const cuda_error &e) {
      // Fall back to CPU implementation if CUDA fails
      // (exception message: e.what())
    }
  }
#endif

  // CPU implementation (fallback or when CUDA not available)
  // Use helper function for trilinear interpolation
  // For dimension-based resize, offset is 0 and scale is inSpace
  // Don't clamp coordinates (original behavior for dimension-based resize)
  resizeTrilinearCPU(newvox, 0.0, 0.0, 0.0, inSpaceX, inSpaceY, inSpaceZ, false);

  copy(newvox); // make this into a copy of the interpolated voxels
  _ctx.threadProgress(1.0f);

  return *this;
}

voxels &voxels::resize(const bounding_box &old_bbox, const bounding_box &new_bbox) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (voxel_dimensions().isNull())
    throw null_dimension("Null voxels dimension.");

  // If bounding boxes are the same, no work needed
  if (old_bbox == new_bbox)
    return *this;

  // Create temporary voxel array with same dimensions to hold resampled data
  voxels newvox(_ctx, voxel_dimensions(), voxelType());

  // Calculate the mapping from new bbox coordinates to old bbox coordinates
  // For each voxel in the grid, we need to find its position in the new bbox,
  // then map that to the old bbox coordinate system
  double old_span_x = old_bbox.maxx - old_bbox.minx;
  double old_span_y = old_bbox.maxy - old_bbox.miny;
  double old_span_z = old_bbox.maxz - old_bbox.minz;

  double new_span_x = new_bbox.maxx - new_bbox.minx;
  double new_span_y = new_bbox.maxy - new_bbox.miny;
  double new_span_z = new_bbox.maxz - new_bbox.minz;

  // Calculate inSpace ratios: how much to step in old bbox coordinates per voxel
  // We're mapping from new_bbox space to old_bbox space
  // Each voxel i in [0, xdim-1] maps to position new_bbox.minx + i * new_span_x / (xdim-1)
  // This position then maps to old_bbox space as: (pos - old_bbox.minx) / old_span_x * (xdim-1)
  // Combining: inSpaceX = (new_span_x / old_span_x)
  double scale_x = new_span_x / old_span_x;
  double scale_y = new_span_y / old_span_y;
  double scale_z = new_span_z / old_span_z;

  // Offset in voxel coordinates when mapping from new bbox to old bbox
  double offset_x = (new_bbox.minx - old_bbox.minx) / old_span_x * (voxel_dimensions()[0] - 1);
  double offset_y = (new_bbox.miny - old_bbox.miny) / old_span_y * (voxel_dimensions()[1] - 1);
  double offset_z = (new_bbox.minz - old_bbox.minz) / old_span_z * (voxel_dimensions()[2] - 1);

#ifdef CVC_USING_CUDA
  // Use CUDA kernel if CUDA is enabled and unified memory is available
  if (_using_cuda && _cuda_unified_ptr) {
    try {
      // Allocate CUDA unified memory for destination
      newvox.enableCUDA(_cuda_device_id);

      // For CUDA, we need to pass the transformation parameters
      // The CUDA kernel will compute: old_coord = offset + new_coord * scale
      cuda_resize_bbox_trilinear(_cuda_unified_ptr.get(),        // source data
                                 newvox._cuda_unified_ptr.get(), // destination data
                                 voxel_dimensions()[0], voxel_dimensions()[1],
                                 voxel_dimensions()[2], offset_x, offset_y, offset_z, scale_x,
                                 scale_y, scale_z, voxelType());

      // Copy the result
      copy(newvox);
      _ctx.threadProgress(1.0f);

      return *this;
    } catch (const cuda_error &e) {
      // Fall back to CPU implementation if CUDA fails
      // (exception message: e.what())
    }
  }
#endif

  // CPU implementation (fallback or when CUDA not available)
  // Use helper function for trilinear interpolation with offset and scale
  // Clamp coordinates (bbox mode requires clamping and setting xRes/yRes/zRes to 0)
  resizeTrilinearCPU(newvox, offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, true);

  copy(newvox); // make this into a copy of the interpolated voxels
  _ctx.threadProgress(1.0f);

  return *this;
}

voxels &voxels::composite(const voxels &compVox, int64 off_x, int64 off_y, int64 off_z,
                          const composite_function &func) {
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  uint64 i, j, k;

  for (k = 0; k < compVox.ZDim(); k++) {
#ifdef _OPENMP
#pragma omp parallel for private(i, j) schedule(dynamic)
#endif
    for (j = 0; j < compVox.YDim(); j++)
      for (i = 0; i < compVox.XDim(); i++)
        if ((int64(i) + off_x >= 0) && (int64(i) + off_x < int64(XDim())) &&
            (int64(j) + off_y >= 0) && (int64(j) + off_y < int64(YDim())) &&
            (int64(k) + off_z >= 0) && (int64(k) + off_z < int64(ZDim())))
          (*this)(
              int64(i) + off_x, int64(j) + off_y, int64(k) + off_z,
              func(compVox, i, j, k, *this, int64(i) + off_x, int64(j) + off_y, int64(k) + off_z));
    _ctx.threadProgress(float(k) / float(compVox.ZDim()));
  }

  _ctx.threadProgress(1.0f);
  return *this;
}

// ============================================================================
// CUDA Unified Memory Support
// ============================================================================

bool voxels::cuda_available() { return cuda_device_manager::cuda_available(); }

int voxels::cuda_device_count() { return cuda_device_manager::device_count(); }

std::vector<gpu_device_info> voxels::get_gpu_info() {
  return cuda_device_manager::get_device_info();
}

int voxels::get_current_gpu() { return cuda_device_manager::get_current_device(); }

void voxels::set_current_gpu(int device_id) { cuda_device_manager::set_current_device(device_id); }

void voxels::enableCUDA(int device_id) {
#ifdef CVC_USING_CUDA
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (_using_cuda) {
    // Already using CUDA - maybe switch devices?
    if (device_id >= 0 && device_id != _cuda_device_id) {
      switchGPU(device_id);
    }
    return;
  }

  if (!cuda_available()) {
    throw cuda_not_available("CUDA not available on this system");
  }

  // Use current device if not specified
  if (device_id < 0) {
    device_id = cuda_device_manager::get_current_device();
    if (device_id < 0)
      device_id = 0; // Default to device 0
  }

  // Validate device
  if (device_id >= cuda_device_count()) {
    throw cuda_error("Invalid CUDA device ID");
  }

  if (!cuda_device_manager::supports_unified_memory(device_id)) {
    throw cuda_error("Device does not support unified memory");
  }

  // Migrate data to CUDA unified memory
  migrate_to_cuda(device_id);

  _using_cuda = true;
  _cuda_device_id = device_id;

  _ctx.log(3, "CUDA unified memory enabled on device " + std::to_string(device_id));
#else
  throw cuda_not_available("CVC was not compiled with CUDA support");
#endif
}

void voxels::disableCUDA() {
#ifdef CVC_USING_CUDA
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (!_using_cuda) {
    return; // Already disabled
  }

  // Migrate data back to system RAM
  migrate_from_cuda();

  _using_cuda = false;
  _cuda_device_id = -1;

  _ctx.log(3, "CUDA unified memory disabled");
#endif
}

void voxels::switchGPU(int new_device_id) {
#ifdef CVC_USING_CUDA
  thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  if (!_using_cuda) {
    // Not currently using CUDA, just enable on the new device
    enableCUDA(new_device_id);
    return;
  }

  if (new_device_id == _cuda_device_id) {
    return; // Already on this device
  }

  if (!cuda_available() || new_device_id >= cuda_device_count()) {
    throw cuda_error("Invalid CUDA device ID");
  }

  if (!cuda_device_manager::supports_unified_memory(new_device_id)) {
    throw cuda_error("Target device does not support unified memory");
  }

  int old_device = _cuda_device_id;

  // Check if peer access is possible
  bool can_peer_access = cuda_device_manager::can_access_peer(new_device_id, old_device);

  if (can_peer_access) {
    // Enable peer access for direct GPU-to-GPU copy
    try {
      cuda_device_manager::enable_peer_access(new_device_id, old_device);
    } catch (...) {
      // Peer access failed, fall back to host copy
      can_peer_access = false;
    }
  }

  // Save old data pointer
  byte *old_data = get_data_ptr();
  uint64 data_size = XDim() * YDim() * ZDim() * voxelSize();

  // Allocate new memory on target device
  int current_device = cuda_device_manager::get_current_device();
  cuda_device_manager::set_current_device(new_device_id);

  try {
    allocate_cuda_memory(_voxelType);
  } catch (...) {
    cuda_device_manager::set_current_device(current_device);
    throw;
  }

  byte *new_data = get_data_ptr();

  if (can_peer_access) {
    // Direct peer-to-peer copy
    CUDA_CHECK(cudaMemcpyPeer(new_data, new_device_id, old_data, old_device, data_size));
    _ctx.log(3, "Performed peer-to-peer GPU copy from device " + std::to_string(old_device) +
                    " to device " + std::to_string(new_device_id));
  } else {
    // Copy through host memory
    std::vector<byte> temp_buffer(data_size);

    // Set to old device and copy to host
    cuda_device_manager::set_current_device(old_device);
    CUDA_CHECK(cudaMemcpy(temp_buffer.data(), old_data, data_size, cudaMemcpyDeviceToHost));

    // Set to new device and copy from host
    cuda_device_manager::set_current_device(new_device_id);
    CUDA_CHECK(cudaMemcpy(new_data, temp_buffer.data(), data_size, cudaMemcpyHostToDevice));

    _ctx.log(3, "Performed host-mediated GPU copy from device " + std::to_string(old_device) +
                    " to device " + std::to_string(new_device_id));
  }

  // Restore original device
  cuda_device_manager::set_current_device(current_device);

  _cuda_device_id = new_device_id;
#else
  throw cuda_not_available("CVC was not compiled with CUDA support");
#endif
}

void voxels::allocate_cuda_memory(data_type vt) {
#ifdef CVC_USING_CUDA
  // Reset any existing CUDA memory reference (shared_ptr will auto-cleanup if last reference)
  _cuda_unified_ptr.reset();

  uint64 size = XDim() * YDim() * ZDim();
  uint64 byte_size = size * voxelSize();

  // Allocate CUDA unified memory
  void *raw_ptr = nullptr;
  CUDA_CHECK(cudaMallocManaged(&raw_ptr, byte_size));

  // Wrap in shared_ptr with custom deleter for automatic cleanup
  _cuda_unified_ptr = std::shared_ptr<void>(raw_ptr, CudaManagedDeleter());

  // Initialize to zero
  CUDA_CHECK(cudaMemset(_cuda_unified_ptr.get(), 0, byte_size));
#endif
}

void voxels::free_cuda_memory() {
#ifdef CVC_USING_CUDA
  // Reset shared_ptr (will call CudaManagedDeleter if this is the last reference)
  _cuda_unified_ptr.reset();
#endif
}

void voxels::migrate_to_cuda(int device_id) {
#ifdef CVC_USING_CUDA
  // Set device
  int old_device = cuda_device_manager::get_current_device();
  cuda_device_manager::set_current_device(device_id);

  // Allocate CUDA unified memory and copy data
  uint64 data_size = XDim() * YDim() * ZDim() * voxelSize();
  allocate_cuda_memory(_voxelType);
  std::memcpy(_cuda_unified_ptr.get(), _voxels.get(), data_size);

  // Synchronize to ensure data is uploaded
  CUDA_CHECK(cudaDeviceSynchronize());

  // Restore device
  cuda_device_manager::set_current_device(old_device);
#endif
}

void voxels::migrate_from_cuda() {
#ifdef CVC_USING_CUDA
  if (!_cuda_unified_ptr) {
    return; // Nothing to migrate
  }

  // Ensure data is synchronized to host
  if (_cuda_device_id >= 0) {
    int old_device = cuda_device_manager::get_current_device();
    cuda_device_manager::set_current_device(_cuda_device_id);
    CUDA_CHECK(cudaDeviceSynchronize());
    cuda_device_manager::set_current_device(old_device);
  }

  // Copy CUDA data back to system memory
  uint64 data_size = XDim() * YDim() * ZDim() * voxelSize();
  std::memcpy(_voxels.get(), _cuda_unified_ptr.get(), data_size);

  // Free CUDA memory (reset shared_ptr)
  free_cuda_memory();
#endif
}
} // namespace cvc
