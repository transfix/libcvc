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

#ifndef __VOLMAGICK_VOXELS_H__
#define __VOLMAGICK_VOXELS_H__

#include <cvc/namespace.h>
#include <cvc/types.h>
#include <cvc/dimension.h>
#include <cvc/bounding_box.h>
#include <cvc/exception.h>
#include <cvc/cuda_utils.h>

#include <boost/shared_array.hpp>
#include <boost/tuple/tuple.hpp>

#include <algorithm>
#include <cstring>
#include <cstddef>

namespace CVC_NAMESPACE
{
  // Use std::byte for raw memory representation (C++17) or fallback
  #if __cplusplus >= 201703L
    using byte = std::byte;
  #else
    using byte = unsigned char;
  #endif
}

namespace CVC_NAMESPACE
{
#ifdef CVC_USING_CUDA
  // CUDA kernel launcher for trilinear resize (defined in voxels_kernels.cu)
  extern "C" void cuda_resize_trilinear(
      void* src_data,
      void* dst_data,
      uint64 src_x, uint64 src_y, uint64 src_z,
      uint64 dst_x, uint64 dst_y, uint64 dst_z,
      double inSpaceX, double inSpaceY, double inSpaceZ,
      data_type voxel_type);
  
  // CUDA kernel launcher for bounding box aware trilinear resize (defined in voxels_kernels.cu)
  extern "C" void cuda_resize_bbox_trilinear(
      void* src_data,
      void* dst_data,
      uint64 dim_x, uint64 dim_y, uint64 dim_z,
      double offset_x, double offset_y, double offset_z,
      double scale_x, double scale_y, double scale_z,
      data_type voxel_type);
  
  // CUDA kernel launcher for anisotropic diffusion slice (defined in voxels_kernels.cu)
  extern "C" void cuda_anisotropic_diffusion_slice(
      void* src_data,
      void* dst_data,
      uint64 xdim, uint64 ydim, uint64 zdim,
      uint64 slice_idx,
      double K_para, double Lambda_para,
      data_type voxel_type);
#endif

  class composite_function;

  /**
   * @class voxels
   * @brief Multi-dimensional voxel data container with configurable copy semantics
   * 
   * IMPORTANT - MEMORY SEMANTICS:
   * =============================
   * Voxels uses SHALLOW COPY semantics by default via boost::shared_array for CPU memory
   * and std::shared_ptr for CUDA unified memory (with automatic reference counting).
   * 
   * SHALLOW COPY (default - shares data):
   * - Copy constructor: voxels v2(v1);         // Shares CPU & GPU memory
   * - Assignment operator: v2 = v1;            // Shares CPU & GPU memory
   * - copy() method: v2.copy(v1);              // Shares CPU & GPU memory (default)
   * - copy() method: v2.copy(v1, false);       // Shares CPU & GPU memory (explicit)
   * 
   * With shallow copy, all copies share the same underlying voxel data arrays.
   * Modifications through any copy will affect all other copies!
   * 
   * CUDA MEMORY: When CUDA is enabled, both CPU and GPU memory are reference counted.
   * The custom CudaManagedDeleter ensures CUDA unified memory is automatically freed
   * when the last reference is destroyed.
   * 
   * DEEP COPY (independent data):
   * - copy() method: v2.copy(v1, true);        // Creates independent copy
   * - sub() method: v2.sub(0, 0, 0, v1.voxel_dimensions());  // Creates independent copy
   * 
   * With deep copy, each copy has its own independent voxel data array.
   * Modifications to one copy will NOT affect other copies.
   * 
   * This design enables efficient passing without copying large arrays by default,
   * while still providing explicit deep copy when needed.
   */
  class voxels
  {
  public:
    voxels(const dimension& d = dimension(4,4,4), data_type vt = UChar);
    voxels(const void *v, const dimension& d, data_type vt);
    voxels(const voxels& v);
    virtual ~voxels();

    dimension& voxel_dimensions() { return _dimension; }
    const dimension& voxel_dimensions() const { return _dimension; }
    virtual void voxel_dimensions(const dimension& d);
    uint64 XDim() const { return voxel_dimensions().xdim; }
    uint64 YDim() const { return voxel_dimensions().ydim; }
    uint64 ZDim() const { return voxel_dimensions().zdim; }

    /*
      Voxel I/O - 3D indexing using linear offset calculation
    */
    double operator()(uint64 i, uint64 j, uint64 k) const /* reading a voxel value */
    {
      if(i >= XDim() || j >= YDim() || k >= ZDim()) 
	throw index_out_of_bounds("");
      
      uint64 idx = i + j*XDim() + k*XDim()*YDim();
      const byte* data = get_data_ptr();
      
      switch(voxelType())
	{
	case UChar:
	  return double(data[idx]);
	case UShort:
	  return double(reinterpret_cast<const unsigned short*>(data)[idx]);
	case UInt:
	  return double(reinterpret_cast<const unsigned int*>(data)[idx]);
	case Float:
	  return double(reinterpret_cast<const float*>(data)[idx]);
	case Double:
	  return double(reinterpret_cast<const double*>(data)[idx]);
	case UInt64:
	  return double(reinterpret_cast<const uint64*>(data)[idx]);
	}
      return 0;
    }
    
    // Linear indexing for backward compatibility
    double operator()(uint64 i) const
    {
      if(i >= XDim()*YDim()*ZDim()) 
	throw index_out_of_bounds("");
      uint64 x = i % XDim();
      uint64 y = (i / XDim()) % YDim();
      uint64 z = i / (XDim() * YDim());
      return (*this)(x, y, z);
    }
    
    void operator()(uint64 i, uint64 j, uint64 k, double val) /* writing a voxel value */
    {
      if(i >= XDim() || j >= YDim() || k >= ZDim()) 
	throw index_out_of_bounds("");

      preWrite();

      uint64 idx = i + j*XDim() + k*XDim()*YDim();
      byte* data = get_data_ptr();
      
      switch(voxelType())
	{
	case UChar:
	  data[idx] = static_cast<unsigned char>(val);
	  break;
	case UShort:
	  reinterpret_cast<unsigned short*>(data)[idx] = static_cast<unsigned short>(val);
	  break;
	case UInt:
	  reinterpret_cast<unsigned int*>(data)[idx] = static_cast<unsigned int>(val);
	  break;
	case Float:
	  reinterpret_cast<float*>(data)[idx] = static_cast<float>(val);
	  break;
	case Double:
	  reinterpret_cast<double*>(data)[idx] = static_cast<double>(val);
	  break;
	case UInt64:
	  reinterpret_cast<uint64*>(data)[idx] = static_cast<uint64>(val);
	  break;
	}

      //NOTE: we cant modify min/max here because it would mess up a map() operation, and perhaps other things
      //if(_minIsSet && val < min()) min(val);
      //if(_maxIsSet && val > max()) max(val);
    }
    
    // Linear indexing for backward compatibility
    void operator()(uint64 i, double val)
    {
      if(i >= XDim()*YDim()*ZDim()) 
	throw index_out_of_bounds("");
      uint64 x = i % XDim();
      uint64 y = (i / XDim()) % YDim();
      uint64 z = i / (XDim() * YDim());
      (*this)(x, y, z, val);
    }
    
    unsigned char * operator*() { preWrite(); return get_data_ptr(); }
    const unsigned char * operator*() const { return reinterpret_cast<const unsigned char*>(get_data_ptr()); }

    data_type voxelType() const { return _voxelType; }
    void voxelType(data_type);
    uint64 voxelSize() const { return data_type_sizes[voxelType()]; }
    const char * voxelTypeStr() const { return data_type_strings[voxelType()]; }

     /* min and max values */
    double min() const { if(!_minIsSet) calcMinMax(); return _min; }
    void min(double m) { _min = m; _minIsSet = true; }
    double max() const { if(!_maxIsSet) calcMinMax(); return _max; }
    void max(double m) { _max = m; _maxIsSet = true; }
    void unsetMinMax() { _minIsSet = _maxIsSet = false; }
    bool minIsSet() const { return _minIsSet; }
    bool maxIsSet() const { return _maxIsSet; }

    /* calculate min and max values for selected subvolumes */
    double min(uint64 off_x, uint64 off_y, uint64 off_z,
	       const dimension& dim) const;
    double max(uint64 off_x, uint64 off_y, uint64 off_z,
	       const dimension& dim) const;

    voxels& operator=(const voxels& vox) { copy(vox); return *this; }

    bool operator==(const voxels& vox) const 
      { 
        // Check if different dimensions or types
        if(voxel_dimensions() != vox.voxel_dimensions()) return false;
        if(voxelType() != vox.voxelType()) return false;
        
        // Check if same shared_array pointer (shared data)
        if(_voxels.get() == vox._voxels.get()) return true;
        
        // Compare actual data bytes
        return std::memcmp(get_data_ptr(), vox.get_data_ptr(),
                           voxel_dimensions().size() * voxelSize()) == 0;
      }

    bool operator!=(const voxels& vox) const
    {
      return !(*this == vox);
    }

    boost::tuple<const uint64 *,uint64> 
      histogram(uint64 size = 256) const 
    { 
      calcHistogram(size); return boost::make_tuple(_histogram.get(), _histogramSize);
    }

    /*
      operations!
    */
    virtual voxels& copy(const voxels& vox, bool deepCopy = false); //turns this object into a copy of vox (shallow by default, deep if requested)
    virtual voxels& copy(); //creates a deep copy of itself (allocates new memory and copies all data)
    //subvolume extraction: removes voxels outside of the subvolume specified
    virtual voxels& sub(uint64 off_x, uint64 off_y, uint64 off_z,
			const dimension& subvoldim);
    voxels& fill(double val); //set all voxels to the specified value
    voxels& fillsub(uint64 off_x, uint64 off_y, uint64 off_z,
		    const dimension& subvoldim, double val); //set all voxels in specified subvolume to val
    voxels& map(double min_, double max_); //maps voxels from min to max
    voxels& resize(const dimension& newdim); //resizes this object to the specified dimension using trilinear interpolation
    voxels& resize(const bounding_box& old_bbox, const bounding_box& new_bbox); //resizes voxels from one bounding box to another using trilinear interpolation
    voxels& bilateralFilter(double radiometricSigma = 200.0, double spatialSigma = 1.5, unsigned int filterRadius = 2);
    //voxels& rotate(double deg_x, double deg_y, double deg_z); //rotates the object about the x,y,z axis
    /*
      compose vox into this object using the specified composite function.  Yes, the offset may be negative.
      Only the voxels that overlap will be subject to the composition function.
    */
    virtual voxels& composite(const voxels& compVox, int64 off_x, int64 off_y, int64 off_z, const composite_function& func);
    
    /*
     * Zeyun's Contrast enhancement: enhances contrast between voxel values. 'resistor' must be a value between 0.0 and 1.0
     * Requres memory to hold the original volume + 6x the original volume using float values for voxels...
     */
    virtual voxels& contrastEnhancement(double resistor = 0.95);

    /*
     * Zeyun's anisotropic diffusion: filters noise but preserves edges more than bilateral filter
     */
    virtual voxels& anisotropicDiffusion(unsigned int iterations = 20);

    /*
     * Dr. Zhang's gdtv filter.
     */
    virtual voxels& gdtvFilter(double parameterq, double lambda, unsigned int iteration, unsigned int neigbour);

    /*
     * CUDA Unified Memory Support
     * Enable/disable CUDA unified memory for GPU-accelerated operations.
     * Data is automatically migrated between CPU and GPU memory as needed.
     */
    
    // Check if CUDA is available and supported
    static bool cuda_available();
    static int cuda_device_count();
    static std::vector<gpu_device_info> get_gpu_info();
    
    // Get/set GPU device for this thread
    static int get_current_gpu();
    static void set_current_gpu(int device_id);
    
    // Enable/disable CUDA unified memory for this voxels object
    virtual void enableCUDA(int device_id = -1);
    virtual void disableCUDA();
    bool using_cuda() const { return _using_cuda; }
    int cuda_device() const { return _cuda_device_id; }
    
    // Switch to a different GPU (performs peer-to-peer copy if needed)
    virtual void switchGPU(int new_device_id);

    // Direct data pointer access for legacy compatibility
    unsigned char* data_ptr() { return reinterpret_cast<unsigned char*>(get_data_ptr()); }
    const unsigned char* data_ptr() const { return reinterpret_cast<const unsigned char*>(get_data_ptr()); }
    
    // Legacy compatibility: convert to shared_array for old VolMagick code
    // Now we already use shared_array internally, so just return it directly
    boost::shared_array<unsigned char> data_as_shared_array() const {
      return _voxels;
    }

  protected:
    void calcMinMax() const;
    
    // Helper function for trilinear interpolation resize operations
    // Performs CPU-based trilinear interpolation from (*this) to newvox
    // using provided offset and scale parameters
    // clampCoords: whether to clamp coordinates and set xRes/yRes/zRes to 0 at boundaries
    void resizeTrilinearCPU(voxels& newvox, 
                            double offset_x, double offset_y, double offset_z,
                            double scale_x, double scale_y, double scale_z,
                            bool clampCoords) const;
    
    void preWrite()
    {
      _histogramDirty = true; //invalidate the histogram

      if(is_unique()) return; //nothing to copy if our voxels are already unique

      try
	{
	  uint64 size = XDim()*YDim()*ZDim()*voxelSize();
	  
#ifdef CVC_USING_CUDA
	  // If using CUDA, copy from CUDA memory and disable CUDA for this copy
	  if (_using_cuda && _cuda_unified_ptr) {
	    boost::shared_array<unsigned char> tmp = _voxels;
	    _voxels.reset(new unsigned char[size]);
	    std::memcpy(_voxels.get(), _cuda_unified_ptr.get(), size);
	    // Disable CUDA for this instance (it now has its own CPU-only copy)
	    _using_cuda = false;
	    _cuda_device_id = -1;
	    _cuda_unified_ptr.reset(); // Release reference to CUDA memory
	  } else
#endif
	  {
	    boost::shared_array<unsigned char> tmp = _voxels;
	    _voxels.reset(new unsigned char[size]);
	    std::memcpy(_voxels.get(), tmp.get(), size);
	  }
	}
      catch(std::bad_alloc& e)
	{
	  throw memory_allocation_error("Could not allocate memory for voxels during copy-on-write!");
	}
    }
    void calcHistogram(uint64 size) const;

    // Single shared_array for voxel data (all types stored as bytes, cast as needed)
    boost::shared_array<unsigned char> _voxels;

    dimension _dimension;
    data_type _voxelType;

    mutable bool _minIsSet;
    mutable double _min;
    mutable bool _maxIsSet;
    mutable double _max;

    //computed on demand even for const reference so declare as mutable
    mutable boost::shared_ptr<uint64[]> _histogram;
    mutable uint64 _histogramSize;
    mutable bool _histogramDirty;

#ifdef CVC_USING_CUDA
    // Custom deleter for CUDA unified memory
    struct CudaManagedDeleter {
      void operator()(void* ptr) const {
        if (ptr) {
          cudaFree(ptr); // cudaFree works for cudaMallocManaged allocations
        }
      }
    };
#endif

    // CUDA unified memory state
    bool _using_cuda;
    int _cuda_device_id;
    
    // CUDA unified memory with reference counting via std::shared_ptr
#ifdef CVC_USING_CUDA
    std::shared_ptr<void> _cuda_unified_ptr;
#else
    void* _cuda_unified_ptr;
#endif
    
    // Helper methods for CUDA memory management
    void allocate_cuda_memory(data_type vt);
    void migrate_to_cuda(int device_id);
    void migrate_from_cuda();
    void free_cuda_memory();

    // Helper method to get the active array as a raw pointer
    byte* get_data_ptr() {
#ifdef CVC_USING_CUDA
      // Return CUDA unified memory pointer if enabled
      if (_using_cuda && _cuda_unified_ptr) {
        return reinterpret_cast<byte*>(_cuda_unified_ptr.get());
      }
#endif
      // Otherwise return regular shared_array data
      return _voxels.get();
    }

    const byte* get_data_ptr() const {
#ifdef CVC_USING_CUDA
      // Return CUDA unified memory pointer if enabled
      if (_using_cuda && _cuda_unified_ptr) {
        return reinterpret_cast<const byte*>(_cuda_unified_ptr.get());
      }
#endif
      // Otherwise return regular shared_array data
      return _voxels.get();
    }

    // Check if the shared_array is unique (for copy-on-write)
    bool is_unique() const {
      return _voxels.unique();
    }
  };
}

#endif
