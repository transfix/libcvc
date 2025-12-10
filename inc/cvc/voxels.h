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
#include <cvc/exception.h>
#include <cvc/cuda_utils.h>

#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
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
  class composite_function;

  /**
   * @class voxels
   * @brief Multi-dimensional voxel data container with configurable copy semantics
   * 
   * IMPORTANT - MEMORY SEMANTICS:
   * =============================
   * Voxels uses SHALLOW COPY semantics by default via boost::shared_ptr to boost::multi_array.
   * 
   * SHALLOW COPY (default - shares data):
   * - Copy constructor: voxels v2(v1);         // Shares underlying data
   * - Assignment operator: v2 = v1;            // Shares underlying data  
   * - copy() method: v2.copy(v1);              // Shares underlying data (default)
   * - copy() method: v2.copy(v1, false);       // Shares underlying data (explicit)
   * 
   * With shallow copy, all copies share the same underlying voxel data array.
   * Modifications through any copy will affect all other copies!
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
      Voxel I/O - 3D indexing with typed arrays
    */
    double operator()(uint64 i, uint64 j, uint64 k) const /* reading a voxel value */
    {
      if(i >= XDim() || j >= YDim() || k >= ZDim()) 
	throw index_out_of_bounds("");
      
      switch(voxelType())
	{
	case UChar:
	  return double((*_voxels_uchar)[i][j][k]);
	case UShort:
	  return double((*_voxels_ushort)[i][j][k]);
	case UInt:
	  return double((*_voxels_uint)[i][j][k]);
	case Float:
	  return double((*_voxels_float)[i][j][k]);
	case Double:
	  return double((*_voxels_double)[i][j][k]);
	case UInt64:
	  return double((*_voxels_uint64)[i][j][k]);
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

      switch(voxelType())
	{
	case UChar:
	  (*_voxels_uchar)[i][j][k] = static_cast<unsigned char>(val);
	  break;
	case UShort:
	  (*_voxels_ushort)[i][j][k] = static_cast<unsigned short>(val);
	  break;
	case UInt:
	  (*_voxels_uint)[i][j][k] = static_cast<unsigned int>(val);
	  break;
	case Float:
	  (*_voxels_float)[i][j][k] = static_cast<float>(val);
	  break;
	case Double:
	  (*_voxels_double)[i][j][k] = static_cast<double>(val);
	  break;
	case UInt64:
	  (*_voxels_uint64)[i][j][k] = static_cast<uint64>(val);
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
        
        // Check if same shared_ptr pointer (shared data)
        switch(_voxelType) {
          case UChar: if(_voxels_uchar == vox._voxels_uchar) return true; break;
          case UShort: if(_voxels_ushort == vox._voxels_ushort) return true; break;
          case UInt: if(_voxels_uint == vox._voxels_uint) return true; break;
          case Float: if(_voxels_float == vox._voxels_float) return true; break;
          case Double: if(_voxels_double == vox._voxels_double) return true; break;
          case UInt64: if(_voxels_uint64 == vox._voxels_uint64) return true; break;
        }
        
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
    //subvolume extraction: removes voxels outside of the subvolume specified
    virtual voxels& sub(uint64 off_x, uint64 off_y, uint64 off_z,
			const dimension& subvoldim);
    voxels& fill(double val); //set all voxels to the specified value
    voxels& fillsub(uint64 off_x, uint64 off_y, uint64 off_z,
		    const dimension& subvoldim, double val); //set all voxels in specified subvolume to val
    voxels& map(double min_, double max_); //maps voxels from min to max
    voxels& resize(const dimension& newdim); //resizes this object to the specified dimension using trilinear interpolation
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
    // Note: This creates a new shared_array that shares ownership with the multi_array  
    boost::shared_array<unsigned char> data_as_shared_array() const {
      // Create shared_array with custom deleter that keeps multi_array alive
      switch(_voxelType) {
        case UChar: {
          auto multi_copy = _voxels_uchar;
          return boost::shared_array<unsigned char>(
            reinterpret_cast<unsigned char*>(_voxels_uchar->data()),
            [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
          );
        }
        case UShort: {
          auto multi_copy = _voxels_ushort;
          return boost::shared_array<unsigned char>(
            reinterpret_cast<unsigned char*>(_voxels_ushort->data()),
            [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
          );
        }
        case UInt: {
          auto multi_copy = _voxels_uint;
          return boost::shared_array<unsigned char>(
            reinterpret_cast<unsigned char*>(_voxels_uint->data()),
            [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
          );
        }
        case Float: {
          auto multi_copy = _voxels_float;
          return boost::shared_array<unsigned char>(
            reinterpret_cast<unsigned char*>(_voxels_float->data()),
            [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
          );
        }
        case Double: {
          auto multi_copy = _voxels_double;
          return boost::shared_array<unsigned char>(
            reinterpret_cast<unsigned char*>(_voxels_double->data()),
            [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
          );
        }
        case UInt64: {
          auto multi_copy = _voxels_uint64;
          return boost::shared_array<unsigned char>(
            reinterpret_cast<unsigned char*>(_voxels_uint64->data()),
            [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
          );
        }
      }
      return boost::shared_array<unsigned char>();
    }

  protected:
    void calcMinMax() const;
    void preWrite()
    {
      _histogramDirty = true; //invalidate the histogram

      if(is_unique()) return; //nothing to copy if our voxels are already unique

      try
	{
	  switch(_voxelType) {
	    case UChar: {
	      auto tmp = _voxels_uchar;
	      _voxels_uchar.reset(new uchar_array_type(boost::extents[XDim()][YDim()][ZDim()]));
	      std::memcpy(_voxels_uchar->data(), tmp->data(), XDim()*YDim()*ZDim()*sizeof(unsigned char));
	      break;
	    }
	    case UShort: {
	      auto tmp = _voxels_ushort;
	      _voxels_ushort.reset(new ushort_array_type(boost::extents[XDim()][YDim()][ZDim()]));
	      std::memcpy(_voxels_ushort->data(), tmp->data(), XDim()*YDim()*ZDim()*sizeof(unsigned short));
	      break;
	    }
	    case UInt: {
	      auto tmp = _voxels_uint;
	      _voxels_uint.reset(new uint_array_type(boost::extents[XDim()][YDim()][ZDim()]));
	      std::memcpy(_voxels_uint->data(), tmp->data(), XDim()*YDim()*ZDim()*sizeof(unsigned int));
	      break;
	    }
	    case Float: {
	      auto tmp = _voxels_float;
	      _voxels_float.reset(new float_array_type(boost::extents[XDim()][YDim()][ZDim()]));
	      std::memcpy(_voxels_float->data(), tmp->data(), XDim()*YDim()*ZDim()*sizeof(float));
	      break;
	    }
	    case Double: {
	      auto tmp = _voxels_double;
	      _voxels_double.reset(new double_array_type(boost::extents[XDim()][YDim()][ZDim()]));
	      std::memcpy(_voxels_double->data(), tmp->data(), XDim()*YDim()*ZDim()*sizeof(double));
	      break;
	    }
	    case UInt64: {
	      auto tmp = _voxels_uint64;
	      _voxels_uint64.reset(new uint64_array_type(boost::extents[XDim()][YDim()][ZDim()]));
	      std::memcpy(_voxels_uint64->data(), tmp->data(), XDim()*YDim()*ZDim()*sizeof(uint64));
	      break;
	    }
	  }
	}
      catch(std::bad_alloc& e)
	{
	  throw memory_allocation_error("Could not allocate memory for voxels during copy-on-write!");
	}
    }
    void calcHistogram(uint64 size) const;

    // Typed 3D multi_arrays - one for each supported data type
    typedef boost::multi_array<unsigned char, 3> uchar_array_type;
    typedef boost::multi_array<unsigned short, 3> ushort_array_type;
    typedef boost::multi_array<unsigned int, 3> uint_array_type;
    typedef boost::multi_array<float, 3> float_array_type;
    typedef boost::multi_array<double, 3> double_array_type;
    typedef boost::multi_array<uint64, 3> uint64_array_type;

    boost::shared_ptr<uchar_array_type> _voxels_uchar;
    boost::shared_ptr<ushort_array_type> _voxels_ushort;
    boost::shared_ptr<uint_array_type> _voxels_uint;
    boost::shared_ptr<float_array_type> _voxels_float;
    boost::shared_ptr<double_array_type> _voxels_double;
    boost::shared_ptr<uint64_array_type> _voxels_uint64;

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

    // CUDA unified memory state
    bool _using_cuda;
    int _cuda_device_id;
    
    // CUDA unified memory pointers (one per type, only one active at a time)
    void* _cuda_unified_ptr;
    
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
        return reinterpret_cast<byte*>(_cuda_unified_ptr);
      }
#endif
      // Otherwise return regular multi_array data
      switch(_voxelType) {
        case UChar: return _voxels_uchar ? reinterpret_cast<byte*>(_voxels_uchar->data()) : nullptr;
        case UShort: return _voxels_ushort ? reinterpret_cast<byte*>(_voxels_ushort->data()) : nullptr;
        case UInt: return _voxels_uint ? reinterpret_cast<byte*>(_voxels_uint->data()) : nullptr;
        case Float: return _voxels_float ? reinterpret_cast<byte*>(_voxels_float->data()) : nullptr;
        case Double: return _voxels_double ? reinterpret_cast<byte*>(_voxels_double->data()) : nullptr;
        case UInt64: return _voxels_uint64 ? reinterpret_cast<byte*>(_voxels_uint64->data()) : nullptr;
        default: return nullptr;
      }
    }

    const byte* get_data_ptr() const {
#ifdef CVC_USING_CUDA
      // Return CUDA unified memory pointer if enabled
      if (_using_cuda && _cuda_unified_ptr) {
        return reinterpret_cast<const byte*>(_cuda_unified_ptr);
      }
#endif
      // Otherwise return regular multi_array data
      switch(_voxelType) {
        case UChar: return _voxels_uchar ? reinterpret_cast<const byte*>(_voxels_uchar->data()) : nullptr;
        case UShort: return _voxels_ushort ? reinterpret_cast<const byte*>(_voxels_ushort->data()) : nullptr;
        case UInt: return _voxels_uint ? reinterpret_cast<const byte*>(_voxels_uint->data()) : nullptr;
        case Float: return _voxels_float ? reinterpret_cast<const byte*>(_voxels_float->data()) : nullptr;
        case Double: return _voxels_double ? reinterpret_cast<const byte*>(_voxels_double->data()) : nullptr;
        case UInt64: return _voxels_uint64 ? reinterpret_cast<const byte*>(_voxels_uint64->data()) : nullptr;
        default: return nullptr;
      }
    }

    // Check if the current type array is unique (for copy-on-write)
    bool is_unique() const {
      switch(_voxelType) {
        case UChar: return _voxels_uchar.unique();
        case UShort: return _voxels_ushort.unique();
        case UInt: return _voxels_uint.unique();
        case Float: return _voxels_float.unique();
        case Double: return _voxels_double.unique();
        case UInt64: return _voxels_uint64.unique();
        default: return true;
      }
    }
  };
}

#endif
